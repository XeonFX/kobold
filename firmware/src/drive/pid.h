// Velocity PID, one instance per side.
//
// Per-side rather than per-wheel: on a skid-steer chassis the wheels on a side
// are coupled through the floor, and independent per-wheel loops end up fighting
// each other through the carpet.
#pragma once

#include <Arduino.h>

#include "config.h"

class PID {
 public:
  void setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    prev_valid_ = false;
  }

  // setpoint and measurement in mm/s; returns PWM counts.
  float update(float setpoint, float measurement, float dt) {
    const float error = setpoint - measurement;

    // Feed-forward carries the bulk of the command so the integrator only has
    // to trim the difference. Without it, PI alone is sluggish off the mark and
    // tends to overshoot once it catches up.
    const float ff = setpoint * (cfg::PWM_MAX / static_cast<float>(cfg::MAX_SPEED_FLOOR_MM_S));

    integral_ += error * dt;
    if (integral_ > cfg::PID_I_LIMIT) integral_ = cfg::PID_I_LIMIT;
    if (integral_ < -cfg::PID_I_LIMIT) integral_ = -cfg::PID_I_LIMIT;

    // Derivative on error is fine here because the setpoint changes in small
    // steps from a velocity controller, not in jumps.
    float derivative = 0.0f;
    if (prev_valid_ && dt > 0.0f) derivative = (error - prev_error_) / dt;
    prev_error_ = error;
    prev_valid_ = true;

    float out = ff + kp_ * error + ki_ * integral_ + kd_ * derivative;

    // Clamp, and bleed the integrator back when saturated so it cannot wind up
    // against a stalled wheel.
    if (out > cfg::PWM_MAX) {
      out = cfg::PWM_MAX;
      integral_ -= error * dt;
    } else if (out < -cfg::PWM_MAX) {
      out = -cfg::PWM_MAX;
      integral_ -= error * dt;
    }
    return out;
  }

  float integral() const { return integral_; }

 private:
  float kp_ = 0.0f, ki_ = 0.0f, kd_ = 0.0f;
  float integral_ = 0.0f;
  float prev_error_ = 0.0f;
  bool prev_valid_ = false;
};
