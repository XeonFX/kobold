// Motor output stage: 2x TB6612FNG driven per-side.
//
// The only module allowed to touch the STBY pin, so that "stop the motors" has
// exactly one implementation. stopFromISR() is safe to call from an interrupt
// and writes the GPIO register directly rather than going through digitalWrite.
#pragma once

#include <Arduino.h>

#include "config.h"
#include "pins.h"

namespace motors {

// Set by the last commanded direction. Single-channel encoders can't sense
// rotation direction, so this is what gives the tick counts their sign.
inline volatile int8_t dir_left = 0;
inline volatile int8_t dir_right = 0;

// LEDC hardware channels.
//
// This uses the Arduino-ESP32 **2.x** API (ledcSetup + ledcAttachPin, and
// ledcWrite addressed by CHANNEL). The 3.x one-liner `ledcAttach(pin, freq,
// bits)` does not exist here: PlatformIO's official espressif32 platform is
// still on core 2.x, and core 3.x needs the community pioarduino fork. Sticking
// with the official platform is the right trade for a robot.
constexpr uint8_t CH_L = 0;
constexpr uint8_t CH_R = 1;

inline void begin() {
  pinMode(pins::LIN1, OUTPUT);
  pinMode(pins::LIN2, OUTPUT);
  pinMode(pins::RIN1, OUTPUT);
  pinMode(pins::RIN2, OUTPUT);
  pinMode(pins::STBY, OUTPUT);
  digitalWrite(pins::STBY, LOW);  // start coasting

  ledcSetup(CH_L, cfg::PWM_FREQ_HZ, cfg::PWM_RESOLUTION_BITS);
  ledcSetup(CH_R, cfg::PWM_FREQ_HZ, cfg::PWM_RESOLUTION_BITS);
  ledcAttachPin(pins::PWM_L, CH_L);
  ledcAttachPin(pins::PWM_R, CH_R);
  ledcWrite(CH_L, 0);
  ledcWrite(CH_R, 0);
}

// Interrupt-safe hard stop. Direct register write: deterministic, and callable
// from an ISR where digitalWrite's bookkeeping is unwelcome.
inline void IRAM_ATTR stopFromISR() {
  GPIO.out_w1tc = (1UL << pins::STBY);
  dir_left = 0;
  dir_right = 0;
}

inline void enable(bool on) { digitalWrite(pins::STBY, on ? HIGH : LOW); }

inline void coast() {
  ledcWrite(CH_L, 0);
  ledcWrite(CH_R, 0);
  digitalWrite(pins::STBY, LOW);
  dir_left = 0;
  dir_right = 0;
}

// pwm in [-PWM_MAX, +PWM_MAX]. Positive is forward.
inline void setSide(uint8_t in1, uint8_t in2, uint8_t channel, int32_t pwm,
                    volatile int8_t& dir_out) {
  if (pwm > cfg::PWM_MAX) pwm = cfg::PWM_MAX;
  if (pwm < -cfg::PWM_MAX) pwm = -cfg::PWM_MAX;

  if (pwm > cfg::PWM_DEADBAND) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(channel, pwm);
    dir_out = 1;
  } else if (pwm < -cfg::PWM_DEADBAND) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(channel, -pwm);
    dir_out = -1;
  } else {
    // Short brake — both inputs high stops faster than coasting and holds
    // position better on a slope.
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(channel, 0);
    dir_out = 0;
  }
}

inline void set(int32_t left_pwm, int32_t right_pwm) {
  setSide(pins::LIN1, pins::LIN2, CH_L, left_pwm, dir_left);
  setSide(pins::RIN1, pins::RIN2, CH_R, right_pwm, dir_right);
}

}  // namespace motors
