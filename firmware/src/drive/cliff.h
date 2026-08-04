// Cliff detection — the highest-priority safety input on the robot.
//
// Four downward-facing IR modules at the chassis corners. Driving on a table
// gives roughly 166 ms between the leading sensor clearing the edge and the
// wheel following it at 0.3 m/s. A round trip through USB, the serial bridge,
// ROS 2 and back is 50-200 ms, so this decision cannot leave the MCU.
//
// Two independent detection paths, deliberately:
//
//   1. Edge interrupts  — sub-millisecond reaction while driving.
//   2. Level polling    — catches the case an edge-triggered interrupt cannot:
//                         powering on with a sensor already over the edge, when
//                         no transition ever occurs.
//
// Path 2 is not redundancy for its own sake. Without it, a robot booted at the
// table edge would consider itself safe.
#pragma once

#include <Arduino.h>

#include "config.h"
#include "motors.h"
#include "pins.h"

namespace cliff {

// bit0 FL, bit1 FR, bit2 BL, bit3 BR. Set = NO surface underneath.
inline volatile uint8_t mask = 0;
inline volatile bool tripped = false;
inline volatile uint32_t trip_count = 0;

inline volatile uint32_t last_us_fl = 0;
inline volatile uint32_t last_us_fr = 0;
inline volatile uint32_t last_us_bl = 0;
inline volatile uint32_t last_us_br = 0;

inline bool IRAM_ATTR readsCliff(uint8_t pin) {
  const int level = digitalRead(pin);
  return cfg::CLIFF_ACTIVE_LOW ? (level == HIGH) : (level == LOW);
}

#define KOBOLD_CLIFF_ISR(NAME, PIN, BIT, LAST_US)                    \
  inline void IRAM_ATTR NAME() {                                      \
    const uint32_t now = micros();                                    \
    if (now - LAST_US < cfg::CLIFF_DEBOUNCE_US) return;               \
    LAST_US = now;                                                    \
    if (readsCliff(PIN)) {                                            \
      motors::stopFromISR(); /* stop first, bookkeep second */        \
      mask |= (1 << BIT);                                             \
      tripped = true;                                                 \
      trip_count++;                                                   \
    } else {                                                          \
      mask &= ~(1 << BIT);                                            \
    }                                                                 \
  }

KOBOLD_CLIFF_ISR(isrFL, pins::CLIFF_FL, 0, last_us_fl)
KOBOLD_CLIFF_ISR(isrFR, pins::CLIFF_FR, 1, last_us_fr)
KOBOLD_CLIFF_ISR(isrBL, pins::CLIFF_BL, 2, last_us_bl)
KOBOLD_CLIFF_ISR(isrBR, pins::CLIFF_BR, 3, last_us_br)

#undef KOBOLD_CLIFF_ISR

inline void begin() {
  pinMode(pins::CLIFF_FL, INPUT);
  pinMode(pins::CLIFF_FR, INPUT);
  pinMode(pins::CLIFF_BL, INPUT);
  pinMode(pins::CLIFF_BR, INPUT);

  attachInterrupt(digitalPinToInterrupt(pins::CLIFF_FL), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::CLIFF_FR), isrFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::CLIFF_BL), isrBL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::CLIFF_BR), isrBR, CHANGE);
}

// Called every control cycle. Returns the current level-read mask and latches
// a trip if any sensor reads a cliff, regardless of whether an edge fired.
inline uint8_t poll() {
  uint8_t m = 0;
  if (readsCliff(pins::CLIFF_FL)) m |= 1 << 0;
  if (readsCliff(pins::CLIFF_FR)) m |= 1 << 1;
  if (readsCliff(pins::CLIFF_BL)) m |= 1 << 2;
  if (readsCliff(pins::CLIFF_BR)) m |= 1 << 3;

  mask = m;
  if (m) {
    if (!tripped) trip_count++;
    tripped = true;
    motors::stopFromISR();
  }
  return m;
}

// Clearing is only permitted once every sensor sees a surface again — a latched
// cliff fault must not be dismissible while the robot is still over the edge.
inline bool clear() {
  if (poll() != 0) return false;
  tripped = false;
  return true;
}

// Which way is safe to retreat? Returns a suggested reverse direction:
// +1 drive forward, -1 reverse, 0 no safe direction — stop and ask.
inline int8_t escapeDirection() {
  const uint8_t m = mask;
  const bool front_bad = m & 0b0011;
  const bool back_bad = m & 0b1100;
  if (front_bad && !back_bad) return -1;
  if (back_bad && !front_bad) return +1;
  return 0;
}

}  // namespace cliff
