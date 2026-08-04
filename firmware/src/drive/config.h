// Drive controller tunables.
//
// Anything here that describes physical reality (wheel size, encoder counts,
// track width) must be measured on YOUR robot — the defaults are estimates and
// wrong values silently corrupt odometry rather than failing loudly.
#pragma once

#include <stdint.h>

namespace cfg {

// ---- Geometry — MEASURE THESE ---------------------------------------------
// Count the slots on one encoder disc. 20 is typical for the slotted-disc
// modules that ship with this chassis.
constexpr float ENCODER_COUNTS_PER_REV = 20.0f;
constexpr float WHEEL_DIAMETER_MM = 65.0f;
constexpr float WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * 3.14159265f;
constexpr float MM_PER_TICK = WHEEL_CIRCUMFERENCE_MM / ENCODER_COUNTS_PER_REV;  // ~10.2 mm

// Effective track width. On a skid-steer chassis this is NOT the physical
// distance between wheels — the wheels scrub sideways through every turn.
// Calibrate empirically: command ten 360-degree turns, measure the actual
// rotation, and scale until they agree. See docs/PROJECT_PLAN.md section 3.2.
constexpr float TRACK_WIDTH_MM = 150.0f;

// ---- Control loop ---------------------------------------------------------
constexpr uint32_t CONTROL_HZ = 100;
constexpr uint32_t CONTROL_PERIOD_MS = 1000 / CONTROL_HZ;
constexpr uint32_t TELEMETRY_HZ = 50;

// Silence means stop, never "carry on".
constexpr uint32_t CMD_VEL_TIMEOUT_MS = 300;

// ---- Speed limits ---------------------------------------------------------
constexpr int16_t MAX_SPEED_FLOOR_MM_S = 500;
// Table mode: both detection distance and stopping distance scale with speed,
// and on a table you have very little of either.
constexpr int16_t MAX_SPEED_TABLE_MM_S = 150;

// ---- PWM ------------------------------------------------------------------
constexpr uint32_t PWM_FREQ_HZ = 20000;  // above audible
constexpr uint8_t PWM_RESOLUTION_BITS = 10;
constexpr int32_t PWM_MAX = (1 << PWM_RESOLUTION_BITS) - 1;
// Below this duty the motors buzz without turning; don't waste current there.
constexpr int32_t PWM_DEADBAND = 60;

// ---- Velocity PID (per side) ----------------------------------------------
// Gains are conservative starting points. Tune with the Arduino IDE's Serial
// Plotter, or by streaming telemetry — see firmware/README.md.
constexpr float PID_KP_DEFAULT = 0.8f;
constexpr float PID_KI_DEFAULT = 2.5f;
constexpr float PID_KD_DEFAULT = 0.01f;
constexpr float PID_I_LIMIT = 400.0f;  // anti-windup clamp, PWM units

// ---- Battery (3S pack via 47k/10k divider on BATT_ADC) --------------------
constexpr float BATT_DIVIDER_RATIO = 5.7f;  // (47 + 10) / 10
// The ESP32 ADC is markedly nonlinear. Calibrate against a multimeter and
// adjust this scale factor; do not trust the nominal value.
constexpr float BATT_ADC_SCALE = 1.0f;
constexpr uint16_t BATT_WARN_MV = 10200;      // 3.4 V/cell
constexpr uint16_t BATT_CRITICAL_MV = 9600;   // 3.2 V/cell — motors latch off
constexpr uint16_t BATT_HARD_CUTOFF_MV = 9000;

// ---- Cliff sensors --------------------------------------------------------
// Flying-Fish modules read LOW when they see a reflective surface. Inverted
// here so "1" means "no surface" — i.e. a cliff.
constexpr bool CLIFF_ACTIVE_LOW = true;
// Debounce: LM393 comparators have no hysteresis and chatter near threshold.
// Long enough to reject chatter, far short of the ~166 ms budget.
constexpr uint32_t CLIFF_DEBOUNCE_US = 2000;

// ---- Encoders -------------------------------------------------------------
// Reject edges closer together than physically possible at max speed.
constexpr uint32_t ENCODER_MIN_INTERVAL_US = 300;

}  // namespace cfg
