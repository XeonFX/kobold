// Kobold drive controller — ESP32 DevKit V1 (USB-C board).
//
// Tier 0 of the control stack: motor PWM, per-side velocity PID, encoder
// odometry, IMU passthrough, battery monitoring, and the cliff reflex. Nothing
// here waits on the SBC, and every failure path ends with the motors stopped.
//
// Task layout (ESP32 is dual core):
//   core 1  control task, 100 Hz, hard periodic via vTaskDelayUntil
//   core 0  comms task, serial framing + 50 Hz telemetry
//   ISRs    4 encoders, 4 cliff sensors, 1 hardware safety line
//
// Separating the loops means a burst of inbound frames cannot perturb motor
// timing, and a stalled control loop cannot silence telemetry.
#include <Arduino.h>
#include <Wire.h>
#include <kobold_link.h>

#include "cliff.h"
#include "config.h"
#include "encoders.h"
#include "imu.h"
#include "motors.h"
#include "pid.h"
#include "pins.h"

// Set by the build from git describe; see platformio.ini.
#ifndef FW_VERSION_MAJOR
#define FW_VERSION_MAJOR 0
#endif
#ifndef FW_VERSION_MINOR
#define FW_VERSION_MINOR 1
#endif
#ifndef FW_VERSION_PATCH
#define FW_VERSION_PATCH 0
#endif
#ifndef FW_GIT_HASH
#define FW_GIT_HASH 0
#endif

using namespace kobold;

static Link s_link;   // not `link` — collides with POSIX link() from unistd.h

// ---------------------------------------------------------------- state ----

struct State {
  // Commanded body velocity
  int16_t cmd_linear_mm_s = 0;
  int16_t cmd_angular_mrad_s = 0;
  uint32_t last_cmd_ms = 0;

  uint8_t mode = 0;  // 0 floor, 1 table
  int16_t max_speed_mm_s = cfg::MAX_SPEED_FLOOR_MM_S;

  uint8_t flags = FAULT_NONE;
  bool estop = false;
  bool motors_enabled = false;

  uint16_t batt_mv = 0;
  int16_t meas_l_mm_s = 0;
  int16_t meas_r_mm_s = 0;
};

static State st;
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;

static PID pid_left, pid_right;

// Hardware safety line from the sense board. One wire, sub-millisecond, no
// software above the two MCUs in the path.
static volatile bool safety_line_tripped = false;

static void IRAM_ATTR isrSafetyLine() {
  if (digitalRead(pins::SAFETY_IN) == LOW) {  // asserted active-low
    motors::stopFromISR();
    safety_line_tripped = true;
  }
}

// ------------------------------------------------------------- battery ----

static uint16_t readBatteryMv() {
  // Average a few samples: the ESP32 ADC is noisy and this is only read at
  // control rate anyway.
  uint32_t acc = 0;
  for (int i = 0; i < 8; i++) acc += analogReadMilliVolts(pins::BATT_ADC);
  const float pin_mv = acc / 8.0f;
  return static_cast<uint16_t>(pin_mv * cfg::BATT_DIVIDER_RATIO * cfg::BATT_ADC_SCALE);
}

// ------------------------------------------------------------ commands ----

static void applyCmdVel(int16_t linear, int16_t angular) {
  portENTER_CRITICAL(&state_mux);
  st.cmd_linear_mm_s = linear;
  st.cmd_angular_mrad_s = angular;
  st.last_cmd_ms = millis();
  portEXIT_CRITICAL(&state_mux);
}

static void onFrame(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len) {
  uint8_t result = 0;

  switch (type) {
    case MSG_CMD_VEL: {
      if (len != sizeof(CmdVel)) { result = 1; break; }
      CmdVel m;
      memcpy(&m, payload, sizeof(m));
      applyCmdVel(m.linear_mm_s, m.angular_mrad_s);
      break;
    }

    case MSG_SET_MODE: {
      if (len != sizeof(SetMode)) { result = 1; break; }
      SetMode m;
      memcpy(&m, payload, sizeof(m));
      const int16_t ceiling =
          m.mode ? cfg::MAX_SPEED_TABLE_MM_S : cfg::MAX_SPEED_FLOOR_MM_S;
      // Note: plain comparisons rather than std::min — Arduino.h defines min()
      // as a preprocessor macro, which mangles any qualified or templated call.
      int16_t limit = ceiling;
      if (m.max_speed_mm_s > 0 && static_cast<int16_t>(m.max_speed_mm_s) < ceiling) {
        limit = static_cast<int16_t>(m.max_speed_mm_s);
      }
      portENTER_CRITICAL(&state_mux);
      st.mode = m.mode;
      // The host may lower the limit but never raise it above the firmware
      // ceiling for the active mode.
      st.max_speed_mm_s = limit;
      portEXIT_CRITICAL(&state_mux);
      s_link.sendLog(1, m.mode ? "mode: table" : "mode: floor");
      break;
    }

    case MSG_ESTOP: {
      if (len != sizeof(Estop)) { result = 1; break; }
      Estop m;
      memcpy(&m, payload, sizeof(m));
      portENTER_CRITICAL(&state_mux);
      st.estop = m.engage != 0;
      portEXIT_CRITICAL(&state_mux);
      if (st.estop) motors::coast();
      break;
    }

    case MSG_CLEAR_FAULT: {
      if (len != sizeof(ClearFault)) { result = 1; break; }
      ClearFault m;
      memcpy(&m, payload, sizeof(m));
      // Faults may only be cleared once their cause is gone. A latched cliff
      // fault must not be dismissible while still over the edge.
      if (m.mask & FAULT_CLIFF) {
        if (!cliff::clear()) { result = 2; break; }
      }
      if (m.mask & FAULT_SAFETY_LINE) {
        if (digitalRead(pins::SAFETY_IN) == LOW) { result = 2; break; }
        safety_line_tripped = false;
      }
      portENTER_CRITICAL(&state_mux);
      st.flags &= ~m.mask;
      portEXIT_CRITICAL(&state_mux);
      pid_left.reset();
      pid_right.reset();
      break;
    }

    case MSG_SET_PID: {
      if (len != sizeof(SetPid)) { result = 1; break; }
      SetPid m;
      memcpy(&m, payload, sizeof(m));
      const float kp = m.kp_x1000 / 1000.0f;
      const float ki = m.ki_x1000 / 1000.0f;
      const float kd = m.kd_x1000 / 1000.0f;
      pid_left.setGains(kp, ki, kd);
      pid_right.setGains(kp, ki, kd);
      break;
    }

    case MSG_VERSION_REQ: {
      Version v{PROTOCOL_VERSION, BOARD_DRIVE, FW_VERSION_MAJOR, FW_VERSION_MINOR,
                FW_VERSION_PATCH, FW_GIT_HASH};
      s_link.send(MSG_VERSION, v);
      return;  // version is its own reply
    }

    case MSG_PING: {
      if (len != sizeof(Ping)) { result = 1; break; }
      break;
    }

    default:
      result = 3;  // unknown message
      break;
  }

  Ack ack{type, seq, result};
  s_link.send(MSG_ACK, ack);
}

// -------------------------------------------------------- control task ----

static void controlTask(void*) {
  TickType_t next = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(cfg::CONTROL_PERIOD_MS);

  encoders::Counts prev = encoders::snapshot();
  uint32_t prev_us = micros();

  for (;;) {
    vTaskDelayUntil(&next, period);

    const uint32_t now_us = micros();
    const float dt = (now_us - prev_us) / 1e6f;
    prev_us = now_us;
    if (dt <= 0.0f || dt > 0.5f) continue;  // first cycle or a stall

    // ---- measure ----
    const encoders::Counts cur = encoders::snapshot();
    const float left_ticks = ((cur.fl - prev.fl) + (cur.rl - prev.rl)) * 0.5f;
    const float right_ticks = ((cur.fr - prev.fr) + (cur.rr - prev.rr)) * 0.5f;
    prev = cur;

    const float meas_l = left_ticks * cfg::MM_PER_TICK / dt;
    const float meas_r = right_ticks * cfg::MM_PER_TICK / dt;

    const uint16_t batt = readBatteryMv();
    const uint8_t cliff_mask = cliff::poll();

    // ---- faults ----
    uint8_t flags = FAULT_NONE;
    if (cliff::tripped) flags |= FAULT_CLIFF;
    if (safety_line_tripped) flags |= FAULT_SAFETY_LINE;
    if (batt < cfg::BATT_CRITICAL_MV) flags |= FAULT_CRITICAL_BATTERY;
    else if (batt < cfg::BATT_WARN_MV) flags |= FAULT_LOW_BATTERY;

    int16_t cmd_lin, cmd_ang, speed_limit;
    bool estop;
    uint32_t last_cmd;
    portENTER_CRITICAL(&state_mux);
    cmd_lin = st.cmd_linear_mm_s;
    cmd_ang = st.cmd_angular_mrad_s;
    speed_limit = st.max_speed_mm_s;
    estop = st.estop;
    last_cmd = st.last_cmd_ms;
    portEXIT_CRITICAL(&state_mux);

    if (millis() - last_cmd > cfg::CMD_VEL_TIMEOUT_MS) {
      flags |= FAULT_WATCHDOG;
      cmd_lin = 0;
      cmd_ang = 0;
    }
    if (estop) flags |= FAULT_ESTOP;

    // Anything in this set means the wheels stop. No exceptions, no partial
    // motion, no "but it was only a warning".
    const uint8_t blocking = FAULT_CLIFF | FAULT_SAFETY_LINE | FAULT_ESTOP |
                             FAULT_CRITICAL_BATTERY;

    int32_t out_l = 0, out_r = 0;

    if (flags & blocking) {
      motors::coast();
      pid_left.reset();
      pid_right.reset();
    } else {
      // ---- mix: body velocity -> per-side wheel velocity ----
      const float half_track = cfg::TRACK_WIDTH_MM * 0.5f;
      const float rot_mm_s = (cmd_ang / 1000.0f) * half_track;
      float target_l = cmd_lin - rot_mm_s;
      float target_r = cmd_lin + rot_mm_s;

      // Scale both sides together if either exceeds the limit, so that
      // clamping never changes the commanded turn radius.
      const float abs_l = fabsf(target_l);
      const float abs_r = fabsf(target_r);
      const float peak = (abs_l > abs_r) ? abs_l : abs_r;
      if (peak > speed_limit && peak > 0.0f) {
        const float scale = speed_limit / peak;
        target_l *= scale;
        target_r *= scale;
      }

      if (fabsf(target_l) < 1.0f && fabsf(target_r) < 1.0f) {
        motors::set(0, 0);
        pid_left.reset();
        pid_right.reset();
      } else {
        motors::enable(true);
        out_l = static_cast<int32_t>(pid_left.update(target_l, meas_l, dt));
        out_r = static_cast<int32_t>(pid_right.update(target_r, meas_r, dt));
        motors::set(out_l, out_r);
      }
    }

    portENTER_CRITICAL(&state_mux);
    st.flags = flags;
    st.batt_mv = batt;
    st.meas_l_mm_s = static_cast<int16_t>(meas_l);
    st.meas_r_mm_s = static_cast<int16_t>(meas_r);
    portEXIT_CRITICAL(&state_mux);

    (void)cliff_mask;
  }
}

// ---------------------------------------------------------- comms task ----

static void commsTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(1000 / cfg::TELEMETRY_HZ);
  TickType_t next = xTaskGetTickCount();

  for (;;) {
    s_link.poll();

    if (xTaskGetTickCount() >= next) {
      next += period;

      const encoders::Counts c = encoders::snapshot();
      imu::Reading r = imu::read();

      Telemetry t{};
      t.t_ms = millis();
      t.ticks_fl = c.fl;
      t.ticks_fr = c.fr;
      t.ticks_rl = c.rl;
      t.ticks_rr = c.rr;
      t.gyro_x = r.gx;
      t.gyro_y = r.gy;
      t.gyro_z = r.gz;
      t.accel_x = r.ax;
      t.accel_y = r.ay;
      t.accel_z = r.az;

      portENTER_CRITICAL(&state_mux);
      t.batt_mv = st.batt_mv;
      t.flags = st.flags;
      t.meas_l_mm_s = st.meas_l_mm_s;
      t.meas_r_mm_s = st.meas_r_mm_s;
      portEXIT_CRITICAL(&state_mux);

      t.cliff_mask = cliff::mask;

      s_link.send(MSG_TELEMETRY, t);
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ---------------------------------------------------------------- setup ----

void setup() {
  Serial.begin(921600);
  Serial.setRxBufferSize(1024);

  // Motors first: everything else can fail with the wheels already stopped.
  motors::begin();
  motors::coast();

  cliff::begin();
  encoders::begin();

  pinMode(pins::SAFETY_IN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pins::SAFETY_IN), isrSafetyLine, FALLING);

  analogReadResolution(12);
  analogSetPinAttenuation(pins::BATT_ADC, ADC_11db);

  Wire.begin(pins::I2C_SDA, pins::I2C_SCL, 400000);
  const bool imu_ok = imu::begin();

  pid_left.setGains(cfg::PID_KP_DEFAULT, cfg::PID_KI_DEFAULT, cfg::PID_KD_DEFAULT);
  pid_right.setGains(cfg::PID_KP_DEFAULT, cfg::PID_KI_DEFAULT, cfg::PID_KD_DEFAULT);

  s_link.begin(Serial, onFrame);

  // Announce ourselves unprompted: the bridge checks this before sending a
  // single motor command.
  Version v{PROTOCOL_VERSION, BOARD_DRIVE, FW_VERSION_MAJOR, FW_VERSION_MINOR,
            FW_VERSION_PATCH, FW_GIT_HASH};
  s_link.send(MSG_VERSION, v);
  if (!imu_ok) s_link.sendLog(3, "MPU-6050 not responding");

  // Control on core 1, comms on core 0. Arduino's loop() runs on core 1 and is
  // left empty.
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(commsTask, "comms", 8192, nullptr, 4, nullptr, 0);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
