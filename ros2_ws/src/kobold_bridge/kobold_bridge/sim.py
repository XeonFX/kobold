"""Firmware simulator — fake drive and sense boards on real PTY devices.

Creates pseudo-terminals that behave exactly like the USB-serial devices, so
the bridge, the ROS 2 stack and the app can all run unmodified with no hardware
attached. Useful while the robot is still on the soldering bench, and useful
afterwards for testing failure paths (cliff trips, watchdog timeouts, version
skew) that are tedious and risky to stage physically.

    python3 -m kobold_bridge.sim

Prints the two device paths to use as `drive_port` / `sense_port`.

The simulated physics is deliberately crude — a first-order velocity response
and perfect encoders. It exists to exercise message plumbing and failure
handling, not to predict how the robot will actually drive.
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import pty
import select
import threading
import time

from .link import FrameDecoder, encode_frame
from .protocol_generated import (
    BOARD_DRIVE,
    BOARD_SENSE,
    FAULT_CLIFF,
    FAULT_ESTOP,
    FAULT_NONE,
    FAULT_SAFETY_LINE,
    FAULT_WATCHDOG,
    MSG_ACK,
    MSG_CLEAR_FAULT,
    MSG_CMD_VEL,
    MSG_ESTOP,
    MSG_RANGES,
    MSG_TELEMETRY,
    MSG_VERSION,
    MSG_VERSION_REQ,
    PROTOCOL_VERSION,
    Ack,
    Ranges,
    Telemetry,
    Version,
)

log = logging.getLogger("sim")

TICKS_PER_REV = 20.0
WHEEL_CIRCUMFERENCE_MM = 65.0 * math.pi
MM_PER_TICK = WHEEL_CIRCUMFERENCE_MM / TICKS_PER_REV
TRACK_WIDTH_MM = 150.0
CMD_VEL_TIMEOUT_S = 0.3


class FakeBoard(threading.Thread):
    """Base: owns a PTY, decodes frames, ticks at a fixed rate."""

    def __init__(self, name: str, board_id: int, rate_hz: float, protocol_version: int):
        super().__init__(name=name, daemon=True)
        self.board_id = board_id
        self.rate_hz = rate_hz
        self.protocol_version = protocol_version

        self._master, self._slave = pty.openpty()
        os.set_blocking(self._master, False)
        self.device = os.ttyname(self._slave)

        self._decoder = FrameDecoder()
        self._seq = 0
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def send(self, msg_type: int, payload: bytes = b""):
        self._seq = (self._seq + 1) & 0xFF
        # encode_frame stamps PROTOCOL_VERSION; for skew testing we patch the
        # version byte back in after COBS would have seen it, so build by hand.
        if self.protocol_version == PROTOCOL_VERSION:
            wire = encode_frame(msg_type, payload, self._seq)
        else:
            from .link import cobs_encode, crc16

            body = bytes(
                [self.protocol_version, msg_type, self._seq, len(payload)]
            ) + payload
            crc = crc16(body)
            wire = cobs_encode(body + bytes([crc & 0xFF, crc >> 8])) + b"\x00"
        try:
            os.write(self._master, wire)
        except OSError:
            pass

    # Recognisable marker so a simulated board is never mistaken for real
    # firmware in a log.
    SIM_GIT_HASH = 0x5119A7ED  # "simulated"

    def _send_version(self):
        self.send(
            MSG_VERSION,
            Version(
                protocol_version=self.protocol_version,
                board_id=self.board_id,
                fw_major=0,
                fw_minor=1,
                fw_patch=0,
                git_hash=self.SIM_GIT_HASH,
            ).pack(),
        )

    def run(self):
        self._send_version()
        period = 1.0 / self.rate_hz
        next_tick = time.monotonic()

        while not self._stop.is_set():
            r, _, _ = select.select([self._master], [], [], 0.005)
            if r:
                try:
                    data = os.read(self._master, 1024)
                except OSError:
                    data = b""
                for frame in self._decoder.feed(data):
                    self.on_frame(frame)

            now = time.monotonic()
            if now >= next_tick:
                next_tick = now + period
                self.tick(period)

    def ack(self, frame, result: int = 0):
        self.send(MSG_ACK, Ack(frame.type, frame.seq, result).pack())

    def on_frame(self, frame):
        raise NotImplementedError

    def tick(self, dt: float):
        raise NotImplementedError


class FakeDrive(FakeBoard):
    def __init__(self, protocol_version: int = PROTOCOL_VERSION):
        super().__init__("sim-drive", BOARD_DRIVE, 50.0, protocol_version)
        self.target_l = 0.0
        self.target_r = 0.0
        self.vel_l = 0.0
        self.vel_r = 0.0
        self.ticks = [0.0, 0.0, 0.0, 0.0]  # fl, fr, rl, rr
        self.last_cmd = 0.0
        self.flags = FAULT_NONE
        self.estop = False
        self.cliff_mask = 0
        self.batt_mv = 12100
        self.t0 = time.monotonic()

    def on_frame(self, frame):
        if frame.type == MSG_CMD_VEL:
            m = frame.decode()
            rot = (m.angular_mrad_s / 1000.0) * (TRACK_WIDTH_MM / 2.0)
            self.target_l = m.linear_mm_s - rot
            self.target_r = m.linear_mm_s + rot
            self.last_cmd = time.monotonic()
            self.ack(frame)
        elif frame.type == MSG_ESTOP:
            self.estop = bool(frame.decode().engage)
            self.ack(frame)
        elif frame.type == MSG_CLEAR_FAULT:
            mask = frame.decode().mask
            # Refuse to clear a cliff fault while still "over the edge",
            # exactly as the firmware does.
            if mask & FAULT_CLIFF and self.cliff_mask:
                self.ack(frame, result=2)
            else:
                self.flags &= ~mask
                self.ack(frame)
        elif frame.type == MSG_VERSION_REQ:
            self._send_version()
        else:
            self.ack(frame)

    def trip_cliff(self, mask: int = 0b0001):
        """Test hook: stage a cliff event without a table."""
        self.cliff_mask = mask
        log.warning("sim: cliff tripped, mask=0b%s", format(mask, "04b"))

    def clear_cliff(self):
        self.cliff_mask = 0

    def tick(self, dt: float):
        self.flags = FAULT_NONE
        if self.estop:
            self.flags |= FAULT_ESTOP
        if self.cliff_mask:
            self.flags |= FAULT_CLIFF
        if time.monotonic() - self.last_cmd > CMD_VEL_TIMEOUT_S:
            self.flags |= FAULT_WATCHDOG

        blocking = FAULT_ESTOP | FAULT_CLIFF | FAULT_WATCHDOG
        tl, tr = (0.0, 0.0) if (self.flags & blocking) else (self.target_l, self.target_r)

        # First-order lag toward the target. Crude on purpose.
        tau = 0.15
        alpha = dt / (tau + dt)
        self.vel_l += (tl - self.vel_l) * alpha
        self.vel_r += (tr - self.vel_r) * alpha

        dl = self.vel_l * dt / MM_PER_TICK
        dr = self.vel_r * dt / MM_PER_TICK
        self.ticks[0] += dl
        self.ticks[2] += dl
        self.ticks[1] += dr
        self.ticks[3] += dr

        # Gyro Z from the velocity difference, in MPU-6050 counts at +/-500 dps.
        omega_rad_s = (self.vel_r - self.vel_l) / TRACK_WIDTH_MM
        gz = int(math.degrees(omega_rad_s) * 65.5)

        self.send(
            MSG_TELEMETRY,
            Telemetry(
                t_ms=int((time.monotonic() - self.t0) * 1000) & 0xFFFFFFFF,
                ticks_fl=int(self.ticks[0]), ticks_fr=int(self.ticks[1]),
                ticks_rl=int(self.ticks[2]), ticks_rr=int(self.ticks[3]),
                gyro_x=0, gyro_y=0, gyro_z=max(-32768, min(32767, gz)),
                accel_x=0, accel_y=0, accel_z=8192,  # 1 g on Z
                batt_mv=self.batt_mv,
                cliff_mask=self.cliff_mask,
                flags=self.flags,
                meas_l_mm_s=int(self.vel_l), meas_r_mm_s=int(self.vel_r),
            ).pack(),
        )


class FakeSense(FakeBoard):
    def __init__(self, protocol_version: int = PROTOCOL_VERSION):
        super().__init__("sim-sense", BOARD_SENSE, 16.0, protocol_version)
        self.ranges = [1500, 2000, 800, 900]
        self.ir_mask = 0
        self.danger_mm = 150
        self.t0 = time.monotonic()

    def on_frame(self, frame):
        if frame.type == MSG_VERSION_REQ:
            self._send_version()
        else:
            self.ack(frame)

    def tick(self, dt: float):
        # Gentle wander so the values visibly change and filters get exercised.
        t = time.monotonic() - self.t0
        self.ranges[0] = int(1500 + 400 * math.sin(t * 0.5))
        self.ranges[2] = int(800 + 200 * math.sin(t * 0.31))

        closest = min(self.ranges)
        flags = FAULT_SAFETY_LINE if closest <= self.danger_mm else FAULT_NONE

        self.send(
            MSG_RANGES,
            Ranges(
                t_ms=int(t * 1000) & 0xFFFFFFFF,
                front_mm=self.ranges[0], back_mm=self.ranges[1],
                left_mm=self.ranges[2], right_mm=self.ranges[3],
                ir_mask=self.ir_mask, flags=flags,
            ).pack(),
        )


def main() -> int:
    ap = argparse.ArgumentParser(description="Kobold firmware simulator")
    ap.add_argument(
        "--bad-version", action="store_true",
        help="report a mismatched protocol version, to exercise the bridge's refusal path",
    )
    ap.add_argument(
        "--cliff-after", type=float, default=0.0,
        help="trip a cliff fault N seconds after start (0 = never)",
    )
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(name)s: %(message)s")

    version = PROTOCOL_VERSION + 1 if args.bad_version else PROTOCOL_VERSION
    drive = FakeDrive(version)
    sense = FakeSense(version)
    drive.start()
    sense.start()

    print("\n  simulated boards ready:\n")
    print(f"    drive_port: {drive.device}")
    print(f"    sense_port: {sense.device}\n")
    print("  ros2 launch kobold_bringup bringup.launch.py \\")
    print(f"      drive_port:={drive.device} sense_port:={sense.device}\n")
    print("  Ctrl-C to stop.\n")

    try:
        start = time.monotonic()
        tripped = False
        while True:
            time.sleep(0.2)
            if (args.cliff_after and not tripped
                    and time.monotonic() - start > args.cliff_after):
                drive.trip_cliff()
                tripped = True
    except KeyboardInterrupt:
        pass
    finally:
        drive.stop()
        sense.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
