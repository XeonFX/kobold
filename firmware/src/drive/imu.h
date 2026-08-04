// MPU-6050 — raw register reads only.
//
// No DMP, no on-chip fusion, no library. Raw gyro and accelerometer counts go
// straight out in telemetry and robot_localization's EKF on the SBC does the
// fusion, where it can also see wheel odometry. Duplicating that work here
// would only give the two filters a chance to disagree.
//
// Gyro Z is the load-bearing signal: skid-steer wheels scrub sideways through
// every turn, so encoder-derived heading is close to useless and this is what
// actually knows which way the robot is pointing.
#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace imu {

constexpr uint8_t ADDR = 0x68;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

struct Reading {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t temp;
  bool valid;
};

inline bool present = false;

inline bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

inline bool begin() {
  Wire.beginTransmission(ADDR);
  Wire.write(REG_WHO_AM_I);
  if (Wire.endTransmission(false) != 0) { present = false; return false; }
  if (Wire.requestFrom(static_cast<uint8_t>(ADDR), static_cast<uint8_t>(1)) != 1) {
    present = false;
    return false;
  }
  const uint8_t who = Wire.read();
  // 0x68 for a genuine MPU-6050; clones report 0x70, 0x72, 0x98 and others.
  // Accept anything that answers rather than gatekeeping on the part number.
  if (who == 0xFF || who == 0x00) { present = false; return false; }

  if (!writeReg(REG_PWR_MGMT_1, 0x01)) return false;  // wake, PLL on gyro X
  delay(10);
  // DLPF 44 Hz: the chassis vibrates hard enough that the unfiltered signal is
  // mostly motor noise.
  writeReg(REG_CONFIG, 0x03);
  writeReg(REG_GYRO_CONFIG, 0x08);   // +/- 500 deg/s
  writeReg(REG_ACCEL_CONFIG, 0x08);  // +/- 4 g

  present = true;
  return true;
}

inline Reading read() {
  Reading r{};
  if (!present) return r;

  Wire.beginTransmission(ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return r;
  if (Wire.requestFrom(static_cast<uint8_t>(ADDR), static_cast<uint8_t>(14)) != 14) return r;

  auto rd16 = []() -> int16_t {
    const uint8_t hi = Wire.read();
    const uint8_t lo = Wire.read();
    return static_cast<int16_t>((hi << 8) | lo);
  };

  r.ax = rd16();
  r.ay = rd16();
  r.az = rd16();
  r.temp = rd16();
  r.gx = rd16();
  r.gy = rd16();
  r.gz = rd16();
  r.valid = true;
  return r;
}

// Scale factors for the ranges configured above — applied on the SBC side,
// documented here so the two stay in step.
constexpr float GYRO_LSB_PER_DEG_S = 65.5f;   // +/- 500 deg/s
constexpr float ACCEL_LSB_PER_G = 8192.0f;    // +/- 4 g

}  // namespace imu
