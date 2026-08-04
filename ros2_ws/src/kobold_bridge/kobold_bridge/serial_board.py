"""Serial transport for one MCU board.

Owns the port, a reader thread, the frame decoder, and the version handshake.
Deliberately knows nothing about ROS — it hands decoded frames to a callback,
which makes it testable against the simulator without a ROS graph.

The version handshake is the safety interlock for over-USB firmware updates: a
board whose protocol version disagrees with this build is never sent a command.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable

import serial

from .link import Frame, FrameDecoder, encode_frame
from .protocol_generated import (
    BOARD_DRIVE,
    BOARD_HEAD,
    BOARD_SENSE,
    MSG_VERSION,
    MSG_VERSION_REQ,
    PROTOCOL_VERSION,
    Version,
)

log = logging.getLogger(__name__)


class BoardVersionMismatch(RuntimeError):
    def __init__(self, port: str, got: int, expected: int):
        super().__init__(
            f"{port}: firmware speaks protocol v{got}, this bridge speaks "
            f"v{expected}. Reflash the board (make flash-drive / flash-sense) "
            f"or check out the matching commit. Refusing to send commands."
        )
        self.got = got
        self.expected = expected


class BoardIdentityMismatch(RuntimeError):
    """The board on this port is not the board this port is supposed to hold.

    Both ESP32s on this robot are CP2102 with the factory-default serial
    "0001", so udev can only tell them apart by which socket they are in. Swap
    the cables and /dev/robot-drive silently becomes the ultrasonic board.

    The firmware announces its own board id at boot, so the protocol already
    carries the truth. This turns a cable swap into a refusal instead of motor
    commands going somewhere with no motors on it.
    """

    def __init__(self, port: str, got: int, expected: int):
        names = {BOARD_DRIVE: "drive", BOARD_SENSE: "sense", BOARD_HEAD: "head"}
        super().__init__(
            f"{port}: this port expects the {names.get(expected, expected)} board "
            f"(id {expected}) but the firmware reports {names.get(got, got)} "
            f"(id {got}). The USB cables are almost certainly swapped, or the "
            f"wrong firmware was flashed. Refusing to send commands."
        )
        self.got = got
        self.expected = expected


class SerialBoard:
    """One MCU on one serial port."""

    def __init__(
        self,
        port: str,
        baud: int = 921600,
        name: str = "board",
        on_frame: Callable[[Frame], None] | None = None,
        on_disconnect: Callable[[str], None] | None = None,
        expected_board: int | None = None,
    ):
        self.port = port
        self.baud = baud
        self.name = name
        self.expected_board = expected_board
        self._on_frame = on_frame
        self._on_disconnect = on_disconnect

        self._serial: serial.Serial | None = None
        self._decoder = FrameDecoder(on_version_mismatch=self._note_version_mismatch)
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._tx_lock = threading.Lock()
        self._seq = 0

        self.version: Version | None = None
        self.version_ok = False
        self.last_rx_time = 0.0
        self._mismatch_logged = False
        self._seen_version: int | None = None

    # ------------------------------------------------------------ lifecycle

    def open(self, timeout: float = 0.05) -> None:
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=timeout)
        except OSError as exc:
            # A pseudo-terminal (the firmware simulator) rejects the custom
            # baud ioctl that a real USB-serial device accepts. Baud is
            # meaningless on a PTY, so fall back to a standard rate rather than
            # making the simulator a special case for every caller.
            if not self._looks_like_pty():
                raise
            log.debug("%s: %s — falling back to 115200 (pty)", self.port, exc)
            self._serial = serial.Serial(self.port, 115200, timeout=timeout)
        # Give the board a moment: opening the port asserts DTR/RTS, which on
        # a CP2102/CH340 board triggers a reset. Reading immediately would
        # capture the bootloader's output rather than firmware frames.
        time.sleep(0.2)
        self._serial.reset_input_buffer()

        self._stop.clear()
        self._thread = threading.Thread(
            target=self._read_loop, name=f"{self.name}-rx", daemon=True
        )
        self._thread.start()

    def _looks_like_pty(self) -> bool:
        return self.port.startswith("/dev/pts/") or "/dev/ttys" in self.port

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()

    @property
    def connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    # ----------------------------------------------------------- handshake

    def wait_for_version(self, timeout: float = 3.0) -> Version:
        """Block until the board reports its version, then validate it.

        Firmware announces itself unprompted on boot, but we also poll in case
        the board was already running when we opened the port.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.version is not None or self._seen_version is not None:
                break
            self.send(MSG_VERSION_REQ)
            time.sleep(0.25)

        # A board speaking the wrong protocol never produces a decodable
        # version message — its frames are dropped by the decoder. Report that
        # as the version mismatch it is, rather than as a timeout that sends
        # you looking for a loose USB cable.
        if self.version is None and self._seen_version is not None:
            raise BoardVersionMismatch(self.port, self._seen_version, PROTOCOL_VERSION)

        if self.version is None:
            raise TimeoutError(
                f"{self.port}: no version response in {timeout:.0f}s. Is the "
                f"board flashed and is this the right port?"
            )

        if self.version.protocol_version != PROTOCOL_VERSION:
            raise BoardVersionMismatch(
                self.port, self.version.protocol_version, PROTOCOL_VERSION
            )

        # Checked AFTER the protocol version, deliberately: a board speaking
        # the wrong protocol has an unreliable board_id field too, and the
        # protocol mismatch is the more actionable error.
        if self.expected_board is not None and self.version.board_id != self.expected_board:
            raise BoardIdentityMismatch(
                self.port, self.version.board_id, self.expected_board
            )

        self.version_ok = True
        return self.version

    # ------------------------------------------------------------- sending

    def send(self, msg_type: int, payload: bytes = b"") -> bool:
        if not self.connected:
            return False
        with self._tx_lock:
            self._seq = (self._seq + 1) & 0xFF
            wire = encode_frame(msg_type, payload, self._seq)
            try:
                self._serial.write(wire)
                return True
            except (serial.SerialException, OSError) as exc:
                log.error("%s: write failed: %s", self.port, exc)
                self._handle_disconnect()
                return False

    def send_msg(self, msg) -> bool:
        """Send a generated NamedTuple message (it carries its own MSG_ID)."""
        return self.send(msg.MSG_ID, msg.pack())

    # ------------------------------------------------------------ receiving

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._serial.read(512)
            except (serial.SerialException, OSError) as exc:
                if not self._stop.is_set():
                    log.error("%s: read failed: %s", self.port, exc)
                    self._handle_disconnect()
                return

            if not data:
                continue

            self.last_rx_time = time.monotonic()
            for frame in self._decoder.feed(data):
                if frame.type == MSG_VERSION:
                    self._handle_version(frame)
                if self._on_frame:
                    try:
                        self._on_frame(frame)
                    except Exception:  # never let a handler kill the reader
                        log.exception("%s: handler raised on %s", self.port, frame.name)

    def _handle_version(self, frame: Frame) -> None:
        v = frame.decode()
        if v is None:
            return
        self.version = v
        dirty = " (DIRTY TREE)" if v.git_hash & 0x80000000 else ""
        log.info(
            "%s: board=%d fw=%d.%d.%d proto=%d git=%08x%s",
            self.port, v.board_id, v.fw_major, v.fw_minor, v.fw_patch,
            v.protocol_version, v.git_hash & 0x7FFFFFFF, dirty,
        )

    def _note_version_mismatch(self, got: int) -> None:
        self._seen_version = got
        if not self._mismatch_logged:
            self._mismatch_logged = True
            log.error(
                "%s: received protocol v%d frames, expected v%d — ignoring them",
                self.port, got, PROTOCOL_VERSION,
            )

    def _handle_disconnect(self) -> None:
        if self._on_disconnect:
            self._on_disconnect(self.port)

    # ---------------------------------------------------------- diagnostics

    @property
    def stats(self):
        return self._decoder.stats

    def describe(self) -> str:
        s = self.stats
        return (
            f"{self.name} [{self.port}] rx={s.rx_frames} crc_err={s.rx_crc_errors} "
            f"ver_err={s.rx_version_errors} malformed={s.rx_malformed}"
        )
