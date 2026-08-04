// GENERATED FROM protocol/protocol.yaml -- DO NOT EDIT BY HAND.
#pragma once
#include <stdint.h>

namespace kobold {

static constexpr uint8_t PROTOCOL_VERSION = 1;

// ---- board ids ----
static constexpr uint8_t BOARD_DRIVE = 1;
static constexpr uint8_t BOARD_SENSE = 2;
static constexpr uint8_t BOARD_HEAD = 3;

// ---- fault bits ----
static constexpr uint8_t FAULT_NONE = 0x00;
static constexpr uint8_t FAULT_CLIFF = 0x01;
static constexpr uint8_t FAULT_SAFETY_LINE = 0x02;
static constexpr uint8_t FAULT_WATCHDOG = 0x04;
static constexpr uint8_t FAULT_LOW_BATTERY = 0x08;
static constexpr uint8_t FAULT_CRITICAL_BATTERY = 0x10;
static constexpr uint8_t FAULT_ESTOP = 0x20;
static constexpr uint8_t FAULT_IMU_ERROR = 0x40;
static constexpr uint8_t FAULT_OVERCURRENT = 0x80;

// ---- message ids ----
enum MsgId : uint8_t {
  MSG_ACK = 0x00,
  MSG_VERSION = 0x01,
  MSG_VERSION_REQ = 0x02,
  MSG_LOG = 0x03,
  MSG_PING = 0x04,
  MSG_TELEMETRY = 0x10,
  MSG_CMD_VEL = 0x11,
  MSG_SET_MODE = 0x12,
  MSG_ESTOP = 0x13,
  MSG_CLEAR_FAULT = 0x14,
  MSG_SET_PID = 0x15,
  MSG_RANGES = 0x20,
  MSG_BUZZER = 0x21,
  MSG_DISPLAY = 0x22,
  MSG_SET_THRESHOLDS = 0x23,
  MSG_HEAD_CMD = 0x30,
  MSG_HEAD_STATE = 0x31,
};

// ---- payload layouts (packed, little-endian) ----
// Acknowledges a host command. Unacked motor commands trip the watchdog.
struct __attribute__((packed)) Ack {
  uint8_t acked_type;
  uint8_t acked_seq;
  uint8_t result;  // 0 = ok, non-zero = rejected
};

// Sent unprompted on boot and in reply to version_req.
struct __attribute__((packed)) Version {
  uint8_t protocol_version;
  uint8_t board_id;
  uint8_t fw_major;
  uint8_t fw_minor;
  uint8_t fw_patch;
  uint32_t git_hash;  // First 4 bytes of the build commit
};

// Human-readable diagnostics. Never used in a control path.
struct __attribute__((packed)) Log {
  uint8_t level;  // 0 debug, 1 info, 2 warn, 3 error
};
// NOTE: log is followed by a variable-length 'text' field (no NUL terminator).

struct __attribute__((packed)) Ping {
  uint32_t nonce;
};

// Cumulative encoder counts, raw IMU, battery and fault state. Tick counts are signed and wrap naturally; the bridge differences them. Direction is applied on the firmware side from the commanded PWM sign, because the LM393 encoders are single-channel and cannot sense it.
struct __attribute__((packed)) Telemetry {
  uint32_t t_ms;  // Firmware uptime, milliseconds
  int32_t ticks_fl;
  int32_t ticks_fr;
  int32_t ticks_rl;
  int32_t ticks_rr;
  int16_t gyro_x;  // Raw MPU-6050 counts
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  uint16_t batt_mv;
  uint8_t cliff_mask;  // bit0 FL, bit1 FR, bit2 BL, bit3 BR — set = NO surface
  uint8_t flags;  // Fault bits
  int16_t meas_l_mm_s;  // Measured left-side wheel velocity
  int16_t meas_r_mm_s;
};

// Body-frame velocity command. The firmware stops the motors if no cmd_vel arrives within CMD_VEL_TIMEOUT_MS — silence means stop, never "carry on".
struct __attribute__((packed)) CmdVel {
  int16_t linear_mm_s;
  int16_t angular_mrad_s;
};

// Table mode caps speed hard and refuses to clear a cliff fault automatically.
struct __attribute__((packed)) SetMode {
  uint8_t mode;  // 0 = floor, 1 = table
  uint16_t max_speed_mm_s;
};

struct __attribute__((packed)) Estop {
  uint8_t engage;  // 1 = latch stopped, 0 = clear
};

// Clears latched faults. Refused while the triggering condition persists.
struct __attribute__((packed)) ClearFault {
  uint8_t mask;
};

// Live tuning. Gains are ×1000 fixed-point to avoid floats on the wire.
struct __attribute__((packed)) SetPid {
  uint16_t kp_x1000;
  uint16_t ki_x1000;
  uint16_t kd_x1000;
};

// Ultrasonic distances in millimetres. RANGE_NO_ECHO (0xFFFF) means no echo returned — deliberately NOT zero, which downstream would read as an obstacle touching the sensor.
struct __attribute__((packed)) Ranges {
  uint32_t t_ms;
  uint16_t front_mm;
  uint16_t back_mm;
  uint16_t left_mm;
  uint16_t right_mm;
  uint8_t ir_mask;  // bit0 front, bit1 back — set = obstacle detected
  uint8_t flags;
};

struct __attribute__((packed)) Buzzer {
  uint8_t pattern;  // 0 off, 1 blip, 2 double, 3 alarm, 4 chirp
  uint8_t repeats;
};

struct __attribute__((packed)) Display {
  uint8_t line;
};
// NOTE: display is followed by a variable-length 'text' field (no NUL terminator).

// Distance at which the sense board asserts the hardware safety line.
struct __attribute__((packed)) SetThresholds {
  uint16_t danger_mm;
  uint16_t warn_mm;
};

struct __attribute__((packed)) HeadCmd {
  int16_t pan_deg;
  int16_t tilt_deg;
};

struct __attribute__((packed)) HeadState {
  int16_t pan_deg;
  int16_t tilt_deg;
  uint8_t pir;
};

}  // namespace kobold
