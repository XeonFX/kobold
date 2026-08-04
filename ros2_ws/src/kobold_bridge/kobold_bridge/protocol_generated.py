"""GENERATED FROM protocol/protocol.yaml -- DO NOT EDIT BY HAND.

"""

import struct
from typing import NamedTuple

PROTOCOL_VERSION = 1

# ---- board ids ----
BOARD_DRIVE = 1
BOARD_SENSE = 2
BOARD_HEAD = 3

# ---- fault bits ----
FAULT_NONE = 0x00
FAULT_CLIFF = 0x01
FAULT_SAFETY_LINE = 0x02
FAULT_WATCHDOG = 0x04
FAULT_LOW_BATTERY = 0x08
FAULT_CRITICAL_BATTERY = 0x10
FAULT_ESTOP = 0x20
FAULT_IMU_ERROR = 0x40
FAULT_OVERCURRENT = 0x80

# ---- message ids ----
MSG_ACK = 0x00
MSG_VERSION = 0x01
MSG_VERSION_REQ = 0x02
MSG_LOG = 0x03
MSG_PING = 0x04
MSG_TELEMETRY = 0x10
MSG_CMD_VEL = 0x11
MSG_SET_MODE = 0x12
MSG_ESTOP = 0x13
MSG_CLEAR_FAULT = 0x14
MSG_SET_PID = 0x15
MSG_RANGES = 0x20
MSG_BUZZER = 0x21
MSG_DISPLAY = 0x22
MSG_SET_THRESHOLDS = 0x23
MSG_HEAD_CMD = 0x30
MSG_HEAD_STATE = 0x31

MSG_NAMES = {
    0x00: 'ack',
    0x01: 'version',
    0x02: 'version_req',
    0x03: 'log',
    0x04: 'ping',
    0x10: 'telemetry',
    0x11: 'cmd_vel',
    0x12: 'set_mode',
    0x13: 'estop',
    0x14: 'clear_fault',
    0x15: 'set_pid',
    0x20: 'ranges',
    0x21: 'buzzer',
    0x22: 'display',
    0x23: 'set_thresholds',
    0x30: 'head_cmd',
    0x31: 'head_state',
}


class Ack(NamedTuple):
    """Acknowledges a host command. Unacked motor commands trip the watchdog."""

    acked_type: int
    acked_seq: int
    result: int

    STRUCT = struct.Struct('<BBB')
    MSG_ID = MSG_ACK

    def pack(self) -> bytes:
        head = self.STRUCT.pack(self.acked_type, self.acked_seq, self.result)
        return head

    @classmethod
    def unpack(cls, data: bytes) -> 'Ack':
        n = cls.STRUCT.size
        vals = cls.STRUCT.unpack(data[:n])
        return cls(*vals)


class Version(NamedTuple):
    """Sent unprompted on boot and in reply to version_req."""

    protocol_version: int
    board_id: int
    fw_major: int
    fw_minor: int
    fw_patch: int
    git_hash: int

    STRUCT = struct.Struct('<BBBBBI')
    MSG_ID = MSG_VERSION

    def pack(self) -> bytes:
        head = self.STRUCT.pack(self.protocol_version, self.board_id, self.fw_major, self.fw_minor, self.fw_patch, self.git_hash)
        return head

    @classmethod
    def unpack(cls, data: bytes) -> 'Version':
        n = cls.STRUCT.size
        vals = cls.STRUCT.unpack(data[:n])
        return cls(*vals)


class VersionReq(NamedTuple):
    """version_req — empty payload."""

    STRUCT = struct.Struct('<')
    MSG_ID = MSG_VERSION_REQ

    def pack(self) -> bytes:
        return b''

    @classmethod
    def unpack(cls, data: bytes) -> 'VersionReq':
        return cls()


class Log(NamedTuple):
    """Human-readable diagnostics. Never used in a control path."""

    level: int
    text: str = ''

    STRUCT = struct.Struct('<B')
    MSG_ID = MSG_LOG

    def pack(self) -> bytes:
        head = self.STRUCT.pack(self.level)
        return head + self.text.encode('utf-8')

    @classmethod
    def unpack(cls, data: bytes) -> 'Log':
        n = cls.STRUCT.size
        vals = cls.STRUCT.unpack(data[:n])
        return cls(*vals, data[n:].decode('utf-8', 'replace'))


class Ping(NamedTuple):
    """ping"""

    nonce: int

    STRUCT = struct.Struct('<I')
    MSG_ID = MSG_PING

    def pack(self) -> bytes:
        head = self.STRUCT.pack(self.nonce)
        return head

    @classmethod
    def unpack(cls, data: bytes) -> 'Ping':
        n = cls.STRUCT.size
        vals = cls.STRUCT.unpack(data[:n])
        return cls(*vals)


class Telemetry(NamedTuple):
    """Cumulative encoder counts, raw IMU, battery and fault state. Tick counts are signed and wrap naturally; the bridge differences them. Direction is applied on the firmware side from the commanded PWM sign, because the LM393 encoders are single-channel and cannot sense it."""

    t_ms: int
    ticks_fl: int
    ticks_fr: int
    ticks_rl: int
    ticks_rr: int
    gyro_x: int
    gyro_y: int
    gyro_z: int
    accel_x: int
    accel_y: int
    accel_z: int
    batt_mv: int
    cliff_mask: int
    flags: int
    meas_l_mm_s: int
    meas_r_mm_s: int

    STRUCT = struct.Struct('<IiiiihhhhhhHBBhh')
    MSG_ID = MSG_TELEMETRY

    def pack(self) -> bytes:
        head = self.STRUCT.pack(self.t_ms, self.ticks_fl, self.ticks_fr, self.ticks_rl, self.ticks_rr, self.gyro_x, self.gyro_y, self.gyro_z, self.accel_x, self.accel_y, self.accel_z, self.batt_mv, self.cliff_mask, self.flags, self.meas_l_mm_s, self.meas_r_mm_s)
        return head

    @classmethod
    def unpack(cls, data: bytes) -> 'Telemetry':
        n = cls.STRUCT.size
        vals = cls.STRUCT.unpack(data[:n])
        return cls(*vals)


class CmdVel(NamedTuple):
    """Body-frame velocity command. The firmware stops the motors if no cmd_vel arrives within CMD_VEL_TIMEOUT_MS — silence means stop, never "carry on"."""

    linear_mm_s: int
    angular_mrad_s: int

    STRUCT = struct.Struct('<hh')
    MSG_ID = MSG_CMD_VEL

    def pack(self) -> bytes:
        head = self.STRUCT.pack(self.linear_mm_s, self.angular_mrad_s)
        return head

    @classmethod
    def unpack(cls, data: bytes) -> 'CmdVel':
        n = cls.STRUCT.size
        vals = cls.STRUCT.unpack(data[:n])
        return cls(*vals)


class SetMode(NamedTuple):
    """Table mode caps speed hard and refuses to clear a cliff fault automatically."""

    mode: int
    max_speed_mm_s: int

    STRUCT = struct.Struct('<BH')
    MSG_ID = MSG_SET_MODE

    def pack(self) -> bytes:
        head = self.STRUCT.pack(self.mode, self.max_speed_mm_s)
        return head

    @classmethod
    def unpack(cls, data: bytes) -> 'SetMode':
        n = cls.STRUCT.size
        vals = cls.STRUCT.unpack(data[:n])
        return cls(*vals)


class Estop(NamedTuple):
    """estop"""

    engage: int

    STRUCT = struct.Struct('<B')
    MSG_ID = MSG_ESTOP

    def pack(self) -> bytes:
        head = self.STRUCT.pack(self.engage)
        return head

    @classmethod
    def unpack(cls, data: bytes) -> 'Estop':
        n = cls.STRUCT.size
        vals = cls.STRUCT.unpack(data[:n])
        return cls(*vals)


class ClearFault(NamedTuple):
    """Clears latched faults. Refused while the triggering condition persists."""

    mask: int

    STRUCT = struct.Struct('<B')
    MSG_ID = MSG_CLEAR_FAULT

    def pack(self) -> bytes:
        head = self.STRUCT.pack(self.mask)
        return head

    @classmethod
    def unpack(cls, data: bytes) -> 'ClearFault':
        n = cls.STRUCT.size
        vals = cls.STRUCT.unpack(data[:n])
        return cls(*vals)


class SetPid(NamedTuple):
    """Live tuning. Gains are ×1000 fixed-point to avoid floats on the wire."""

    kp_x1000: int
    ki_x1000: int
    kd_x1000: int

    STRUCT = struct.Struct('<HHH')
    MSG_ID = MSG_SET_PID

    def pack(self) -> bytes:
        head = self.STRUCT.pack(self.kp_x1000, self.ki_x1000, self.kd_x1000)
        return head

    @classmethod
    def unpack(cls, data: bytes) -> 'SetPid':
        n = cls.STRUCT.size
        vals = cls.STRUCT.unpack(data[:n])
        return cls(*vals)


class Ranges(NamedTuple):
    """Ultrasonic distances in millimetres. RANGE_NO_ECHO (0xFFFF) means no echo returned — deliberately NOT zero, which downstream would read as an obstacle touching the sensor."""

    t_ms: int
    front_mm: int
    back_mm: int
    left_mm: int
    right_mm: int
    ir_mask: int
    flags: int

    STRUCT = struct.Struct('<IHHHHBB')
    MSG_ID = MSG_RANGES

    def pack(self) -> bytes:
        head = self.STRUCT.pack(self.t_ms, self.front_mm, self.back_mm, self.left_mm, self.right_mm, self.ir_mask, self.flags)
        return head

    @classmethod
    def unpack(cls, data: bytes) -> 'Ranges':
        n = cls.STRUCT.size
        vals = cls.STRUCT.unpack(data[:n])
        return cls(*vals)


class Buzzer(NamedTuple):
    """buzzer"""

    pattern: int
    repeats: int

    STRUCT = struct.Struct('<BB')
    MSG_ID = MSG_BUZZER

    def pack(self) -> bytes:
        head = self.STRUCT.pack(self.pattern, self.repeats)
        return head

    @classmethod
    def unpack(cls, data: bytes) -> 'Buzzer':
        n = cls.STRUCT.size
        vals = cls.STRUCT.unpack(data[:n])
        return cls(*vals)


class Display(NamedTuple):
    """display"""

    line: int
    text: str = ''

    STRUCT = struct.Struct('<B')
    MSG_ID = MSG_DISPLAY

    def pack(self) -> bytes:
        head = self.STRUCT.pack(self.line)
        return head + self.text.encode('utf-8')

    @classmethod
    def unpack(cls, data: bytes) -> 'Display':
        n = cls.STRUCT.size
        vals = cls.STRUCT.unpack(data[:n])
        return cls(*vals, data[n:].decode('utf-8', 'replace'))


class SetThresholds(NamedTuple):
    """Distance at which the sense board asserts the hardware safety line."""

    danger_mm: int
    warn_mm: int

    STRUCT = struct.Struct('<HH')
    MSG_ID = MSG_SET_THRESHOLDS

    def pack(self) -> bytes:
        head = self.STRUCT.pack(self.danger_mm, self.warn_mm)
        return head

    @classmethod
    def unpack(cls, data: bytes) -> 'SetThresholds':
        n = cls.STRUCT.size
        vals = cls.STRUCT.unpack(data[:n])
        return cls(*vals)


class HeadCmd(NamedTuple):
    """head_cmd"""

    pan_deg: int
    tilt_deg: int

    STRUCT = struct.Struct('<hh')
    MSG_ID = MSG_HEAD_CMD

    def pack(self) -> bytes:
        head = self.STRUCT.pack(self.pan_deg, self.tilt_deg)
        return head

    @classmethod
    def unpack(cls, data: bytes) -> 'HeadCmd':
        n = cls.STRUCT.size
        vals = cls.STRUCT.unpack(data[:n])
        return cls(*vals)


class HeadState(NamedTuple):
    """head_state"""

    pan_deg: int
    tilt_deg: int
    pir: int

    STRUCT = struct.Struct('<hhB')
    MSG_ID = MSG_HEAD_STATE

    def pack(self) -> bytes:
        head = self.STRUCT.pack(self.pan_deg, self.tilt_deg, self.pir)
        return head

    @classmethod
    def unpack(cls, data: bytes) -> 'HeadState':
        n = cls.STRUCT.size
        vals = cls.STRUCT.unpack(data[:n])
        return cls(*vals)


# Every message, both directions. The bridge only ever receives the
# dev_to_host subset, but the full map lets tests and the firmware
# simulator decode host_to_dev frames too.
DECODERS = {
    MSG_ACK: Ack,
    MSG_VERSION: Version,
    MSG_VERSION_REQ: VersionReq,
    MSG_LOG: Log,
    MSG_PING: Ping,
    MSG_TELEMETRY: Telemetry,
    MSG_CMD_VEL: CmdVel,
    MSG_SET_MODE: SetMode,
    MSG_ESTOP: Estop,
    MSG_CLEAR_FAULT: ClearFault,
    MSG_SET_PID: SetPid,
    MSG_RANGES: Ranges,
    MSG_BUZZER: Buzzer,
    MSG_DISPLAY: Display,
    MSG_SET_THRESHOLDS: SetThresholds,
    MSG_HEAD_CMD: HeadCmd,
    MSG_HEAD_STATE: HeadState,
}

DEV_TO_HOST = frozenset({
    MSG_ACK,
    MSG_VERSION,
    MSG_LOG,
    MSG_TELEMETRY,
    MSG_RANGES,
    MSG_HEAD_STATE,
})

HOST_TO_DEV = frozenset({
    MSG_VERSION_REQ,
    MSG_PING,
    MSG_CMD_VEL,
    MSG_SET_MODE,
    MSG_ESTOP,
    MSG_CLEAR_FAULT,
    MSG_SET_PID,
    MSG_BUZZER,
    MSG_DISPLAY,
    MSG_SET_THRESHOLDS,
    MSG_HEAD_CMD,
})
