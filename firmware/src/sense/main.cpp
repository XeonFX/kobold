// Kobold sensor hub — ESP32 DevKit V1.
//
// Tier 0b: ultrasonic ranging, horizontal IR, buzzer, OLED status. Advisory
// data goes to the SBC over USB, but the one thing that matters urgently — an
// imminent collision — goes to the drive board over a single wire.
//
// The safety line is the point of this board. Everything else is telemetry.
#include <Arduino.h>
#include <Wire.h>
#include <kobold_link.h>

#include "display.h"
#include "pins.h"
#include "ultrasonic.h"

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

// Distance at which the hardware safety line is asserted. Runtime-tunable so
// the host can tighten it in table mode without a reflash.
static uint16_t danger_mm = 150;
static uint16_t warn_mm = 400;

static bool safety_asserted = false;
static uint32_t last_telemetry_ms = 0;
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 62;  // ~16 Hz

// ---- buzzer --------------------------------------------------------------
// Kept quiet on purpose: a piezo at full volume is unpleasant at feline
// hearing range, and this robot is meant to play with a cat.
static uint8_t buzz_pattern = 0;
static uint8_t buzz_repeats = 0;
static uint32_t buzz_next_ms = 0;
static uint8_t buzz_step = 0;

static void buzzerTick() {
  if (!buzz_pattern || millis() < buzz_next_ms) return;

  struct Step { uint16_t freq; uint16_t ms; };
  static const Step blip[] = {{2200, 40}, {0, 0}};
  static const Step dbl[] = {{2200, 40}, {0, 60}, {2200, 40}, {0, 0}};
  static const Step alarm[] = {{1400, 200}, {0, 120}, {1400, 200}, {0, 400}};
  static const Step chirp[] = {{3000, 25}, {0, 25}, {2400, 25}, {0, 0}};

  const Step* pattern = blip;
  switch (buzz_pattern) {
    case 2: pattern = dbl; break;
    case 3: pattern = alarm; break;
    case 4: pattern = chirp; break;
    default: break;
  }

  const Step& s = pattern[buzz_step];
  if (s.freq) tone(pins::BUZZER, s.freq);
  else noTone(pins::BUZZER);

  if (s.ms == 0) {
    noTone(pins::BUZZER);
    buzz_step = 0;
    if (buzz_repeats > 0) buzz_repeats--;
    if (buzz_repeats == 0) buzz_pattern = 0;
    buzz_next_ms = millis() + 80;
  } else {
    buzz_step++;
    buzz_next_ms = millis() + s.ms;
  }
}

// ---- safety line ---------------------------------------------------------

static void updateSafetyLine(uint16_t closest, uint8_t ir_mask) {
  // Assert on either an ultrasonic reading inside the danger radius or a
  // horizontal IR trip. IR catches what ultrasound misses (chair legs,
  // sound-absorbing surfaces) and vice versa (glass defeats IR).
  const bool danger = (closest <= danger_mm) || (ir_mask != 0);

  if (danger && !safety_asserted) {
    // Drive low: the drive board's ISR fires on the falling edge and coasts
    // the motors before this function returns.
    pinMode(pins::SAFETY_OUT, OUTPUT);
    digitalWrite(pins::SAFETY_OUT, LOW);
    safety_asserted = true;
    s_link.sendLog(2, "safety line asserted");
  } else if (!danger && safety_asserted) {
    // Release by going high-impedance rather than driving high, so the drive
    // board's pull-up defines the idle state and a dead sense board reads as
    // "no danger" rather than shorting the line.
    pinMode(pins::SAFETY_OUT, INPUT);
    safety_asserted = false;
    s_link.sendLog(1, "safety line released");
  }
}

// ---- commands ------------------------------------------------------------

static void onFrame(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len) {
  uint8_t result = 0;

  switch (type) {
    case MSG_BUZZER: {
      if (len != sizeof(Buzzer)) { result = 1; break; }
      Buzzer m;
      memcpy(&m, payload, sizeof(m));
      buzz_pattern = m.pattern;
      buzz_repeats = m.repeats ? m.repeats : 1;
      buzz_step = 0;
      buzz_next_ms = 0;
      if (!m.pattern) noTone(pins::BUZZER);
      break;
    }

    case MSG_DISPLAY: {
      if (len < 1) { result = 1; break; }
      char text[64];
      // Plain comparison rather than std::min: Arduino.h defines min() as a
      // preprocessor macro, which mangles any qualified or templated call.
      uint8_t n = static_cast<uint8_t>(len - 1);
      if (n > sizeof(text) - 1) n = sizeof(text) - 1;
      memcpy(text, payload + 1, n);
      text[n] = '\0';
      display::setLine(payload[0], text);
      break;
    }

    case MSG_SET_THRESHOLDS: {
      if (len != sizeof(SetThresholds)) { result = 1; break; }
      SetThresholds m;
      memcpy(&m, payload, sizeof(m));
      danger_mm = m.danger_mm;
      warn_mm = m.warn_mm;
      break;
    }

    case MSG_VERSION_REQ: {
      Version v{PROTOCOL_VERSION, BOARD_SENSE, FW_VERSION_MAJOR, FW_VERSION_MINOR,
                FW_VERSION_PATCH, FW_GIT_HASH};
      s_link.send(MSG_VERSION, v);
      return;
    }

    case MSG_PING:
      break;

    default:
      result = 3;
      break;
  }

  Ack ack{type, seq, result};
  s_link.send(MSG_ACK, ack);
}

// ---- setup / loop --------------------------------------------------------

void setup() {
  Serial.begin(921600);
  Serial.setRxBufferSize(1024);

  // Release the safety line before anything else: a booting sense board must
  // not hold the drive board stopped.
  pinMode(pins::SAFETY_OUT, INPUT);

  pinMode(pins::IR_FRONT, INPUT);
  pinMode(pins::IR_BACK, INPUT);
  pinMode(pins::BUZZER, OUTPUT);

  const uint8_t echo_pins[4] = {pins::ECHO_FRONT, pins::ECHO_BACK, pins::ECHO_LEFT,
                                pins::ECHO_RIGHT};
  ultrasonic::begin(pins::TRIG, echo_pins);

  Wire.begin(pins::I2C_SDA, pins::I2C_SCL, 400000);
  display::begin();
  display::setLine(0, "kobold sense");

  s_link.begin(Serial, onFrame);

  Version v{PROTOCOL_VERSION, BOARD_SENSE, FW_VERSION_MAJOR, FW_VERSION_MINOR,
            FW_VERSION_PATCH, FW_GIT_HASH};
  s_link.send(MSG_VERSION, v);
}

void loop() {
  s_link.poll();
  ultrasonic::poll(pins::TRIG);
  buzzerTick();

  // Flying-Fish modules read LOW when they detect something.
  uint8_t ir_mask = 0;
  if (digitalRead(pins::IR_FRONT) == LOW) ir_mask |= 1 << 0;
  if (digitalRead(pins::IR_BACK) == LOW) ir_mask |= 1 << 1;

  const uint16_t closest = ultrasonic::closest();
  updateSafetyLine(closest, ir_mask);

  const uint32_t now = millis();
  if (now - last_telemetry_ms >= TELEMETRY_INTERVAL_MS) {
    last_telemetry_ms = now;

    Ranges r{};
    r.t_ms = now;
    r.front_mm = ultrasonic::value(0);
    r.back_mm = ultrasonic::value(1);
    r.left_mm = ultrasonic::value(2);
    r.right_mm = ultrasonic::value(3);
    r.ir_mask = ir_mask;
    r.flags = safety_asserted ? FAULT_SAFETY_LINE : FAULT_NONE;
    s_link.send(MSG_RANGES, r);

    display::showRanges(r.front_mm, r.back_mm, r.left_mm, r.right_mm, safety_asserted);
  }
}
