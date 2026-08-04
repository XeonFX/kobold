// Drive controller pin map — ESP32 DevKit V1 (ESP-WROOM-32, 30-pin), USB-C board.
//
// This board is the USB-C variant on purpose: its USB-serial bridge chip has a
// distinct VID/PID, so the udev rule can identify it unambiguously. CH340
// clones frequently share serial numbers, and the failure mode of confusing the
// two boards is sending motor commands to the ultrasonic hub.
//
// Verify against your board silkscreen before soldering. See docs/WIRING.md.
#pragma once

#include <stdint.h>

namespace pins {

// ---- Motors: 2x TB6612FNG, per-side tied ---------------------------------
// Front and rear wheels on a side are coupled through the floor on a skid-steer
// chassis, so their PWM and direction lines are tied together: four independent
// H-bridges for current capacity, but only 3 GPIO per side. Per-side velocity
// PID is also the correct control model here — per-wheel PID makes the wheels
// fight each other through the carpet.
constexpr uint8_t PWM_L = 4;
constexpr uint8_t LIN1 = 5;
constexpr uint8_t LIN2 = 16;
constexpr uint8_t PWM_R = 17;
constexpr uint8_t RIN1 = 18;
constexpr uint8_t RIN2 = 19;
constexpr uint8_t STBY = 23;  // LOW = coast all four motors. The software e-stop.

// ---- Encoders (LM393 modules, D0 only) -----------------------------------
// GPIO 34-39 are input-only, which is exactly right for these. Power the
// modules at 3.3 V so D0 swings 0-3.3 V and needs no level shifting.
constexpr uint8_t ENC_FL = 34;
constexpr uint8_t ENC_FR = 35;
constexpr uint8_t ENC_RL = 36;
constexpr uint8_t ENC_RR = 39;

// ---- Cliff sensors: 4x downward IR at the corners -------------------------
// On the DRIVE board deliberately. Driving on a table gives ~166 ms from edge
// detection to a wheel leaving the surface at 0.3 m/s, and a round trip through
// the SBC is 50-200 ms. These stop the motors locally in under a millisecond.
constexpr uint8_t CLIFF_FL = 13;
constexpr uint8_t CLIFF_FR = 14;
constexpr uint8_t CLIFF_BL = 25;
constexpr uint8_t CLIFF_BR = 26;

// ---- I2C (MPU-6050) -------------------------------------------------------
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;

// ---- Misc -----------------------------------------------------------------
// ADC1 only — ADC2 is unusable while WiFi is active.
constexpr uint8_t BATT_ADC = 32;
// Hardware safety line from the sense board. Sub-millisecond collision stop
// with no software above the two MCUs in the path.
constexpr uint8_t SAFETY_IN = 27;
constexpr uint8_t SPARE = 33;

}  // namespace pins
