// Wheel encoders — LM393 slotted-disc modules, single channel.
//
// Single channel means no direction sensing: the ISR only knows that a slot
// passed, not which way. Sign comes from the last commanded motor direction,
// which is correct except during the brief moment a wheel is being back-driven.
// That is an accepted limitation for a slow indoor robot; the gyro carries
// heading, and (with a lidar) scan matching corrects the rest.
#pragma once

#include <Arduino.h>

#include "config.h"
#include "motors.h"
#include "pins.h"

namespace encoders {

// Signed cumulative counts. int32_t at ~10 mm/tick wraps after ~21,000 km.
inline volatile int32_t count_fl = 0;
inline volatile int32_t count_fr = 0;
inline volatile int32_t count_rl = 0;
inline volatile int32_t count_rr = 0;

inline volatile uint32_t last_us_fl = 0;
inline volatile uint32_t last_us_fr = 0;
inline volatile uint32_t last_us_rl = 0;
inline volatile uint32_t last_us_rr = 0;

// Guards multi-counter snapshots so odometry never mixes pre- and post-tick
// values across the four wheels.
inline portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

struct Counts {
  int32_t fl, fr, rl, rr;
};

// Debounce inside the ISR: LM393 outputs have no hysteresis and chatter near
// the threshold, which would otherwise inflate the tick count and make the
// robot think it travelled further than it did.
#define KOBOLD_ENCODER_ISR(NAME, COUNTER, LAST_US, DIR)             \
  inline void IRAM_ATTR NAME() {                                     \
    const uint32_t now = micros();                                   \
    if (now - LAST_US < cfg::ENCODER_MIN_INTERVAL_US) return;        \
    LAST_US = now;                                                   \
    COUNTER += (DIR >= 0) ? 1 : -1;                                  \
  }

KOBOLD_ENCODER_ISR(isrFL, count_fl, last_us_fl, motors::dir_left)
KOBOLD_ENCODER_ISR(isrFR, count_fr, last_us_fr, motors::dir_right)
KOBOLD_ENCODER_ISR(isrRL, count_rl, last_us_rl, motors::dir_left)
KOBOLD_ENCODER_ISR(isrRR, count_rr, last_us_rr, motors::dir_right)

#undef KOBOLD_ENCODER_ISR

inline void begin() {
  // Input-only pins: no internal pull-ups available. The LM393 modules drive
  // their outputs actively, so none are needed.
  pinMode(pins::ENC_FL, INPUT);
  pinMode(pins::ENC_FR, INPUT);
  pinMode(pins::ENC_RL, INPUT);
  pinMode(pins::ENC_RR, INPUT);

  attachInterrupt(digitalPinToInterrupt(pins::ENC_FL), isrFL, RISING);
  attachInterrupt(digitalPinToInterrupt(pins::ENC_FR), isrFR, RISING);
  attachInterrupt(digitalPinToInterrupt(pins::ENC_RL), isrRL, RISING);
  attachInterrupt(digitalPinToInterrupt(pins::ENC_RR), isrRR, RISING);
}

inline Counts snapshot() {
  Counts c;
  portENTER_CRITICAL(&mux);
  c.fl = count_fl;
  c.fr = count_fr;
  c.rl = count_rl;
  c.rr = count_rr;
  portEXIT_CRITICAL(&mux);
  return c;
}

}  // namespace encoders
