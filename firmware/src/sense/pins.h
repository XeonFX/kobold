// Sensor hub pin map — ESP32 DevKit V1 (ESP-WROOM-32, 30-pin).
//
// Ranging lives on its own board because HC-SR04 echo timing is measured in
// microseconds and would otherwise share a core with motor PWM and four encoder
// ISRs. A burst of encoder interrupts during a hard turn would corrupt exactly
// the distance reading you need during a hard turn.
#pragma once

#include <stdint.h>

namespace pins {

// ---- Ultrasonic ----------------------------------------------------------
// One trigger line to all four sensors, fired SEQUENTIALLY. Firing together
// makes them hear each other's pings.
constexpr uint8_t TRIG = 23;

// ECHO is a 5 V output — level shift or divide it. These four GPIOs are
// input-only, which suits them perfectly and frees the output-capable pins.
constexpr uint8_t ECHO_FRONT = 34;
constexpr uint8_t ECHO_BACK = 35;
constexpr uint8_t ECHO_LEFT = 36;
constexpr uint8_t ECHO_RIGHT = 39;

// ---- Horizontal IR -------------------------------------------------------
// The two forward/rearward-facing Flying-Fish modules. The other four face
// down as cliff sensors and are wired to the DRIVE board instead.
constexpr uint8_t IR_FRONT = 13;
constexpr uint8_t IR_BACK = 14;

// ---- I2C (SSD1306 OLED) --------------------------------------------------
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;

// ---- Outputs -------------------------------------------------------------
constexpr uint8_t BUZZER = 5;
// Hardware safety line to the drive board. Active LOW, open-drain style: pull
// low on imminent collision and the drive board coasts on a hardware interrupt
// in under a millisecond, with no software above the two MCUs in the path.
constexpr uint8_t SAFETY_OUT = 25;

}  // namespace pins
