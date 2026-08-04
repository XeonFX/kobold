// HC-SR04 ranging — sequential, interrupt-captured, median filtered.
//
// Three deliberate choices:
//
//   Sequential   Firing all four at once means each sensor hears its
//                neighbours' pings. One at a time, ~60 ms apart.
//
//   Interrupt    Echo width is captured in an ISR rather than with pulseIn(),
//                capture      which blocks for up to the timeout. Blocking here would
//                stall the serial task and delay the safety line.
//
//   Median-of-3  HC-SR04s throw occasional wild outliers. A single bad reading
//                tripping an emergency stop mid-drive is maddening to debug and
//                trivial to prevent.
#pragma once

#include <Arduino.h>

namespace ultrasonic {

constexpr uint8_t COUNT = 4;
constexpr uint16_t NO_ECHO = 0xFFFF;

// Deliberately not zero. Zero reads downstream as "obstacle touching the
// sensor" and would hard-stop the robot every time a ping went unanswered.
constexpr uint16_t MAX_RANGE_MM = 4000;
constexpr uint32_t ECHO_TIMEOUT_US = 25000;  // ~4.3 m round trip
constexpr uint32_t CYCLE_INTERVAL_MS = 15;   // per sensor -> ~16 Hz full sweep

// Speed of sound 343 m/s -> 0.343 mm/us, halved for the round trip.
constexpr float US_TO_MM = 0.1715f;

struct Sensor {
  uint8_t echo_pin;
  volatile uint32_t rise_us;
  volatile uint32_t width_us;
  volatile bool have_reading;
  uint16_t history[3];
  uint8_t history_i;
  uint16_t value_mm;
};

inline Sensor sensors[COUNT];
inline volatile int8_t active = -1;  // which sensor is mid-ping
inline uint32_t last_fire_ms = 0;
inline uint8_t next_sensor = 0;

inline void IRAM_ATTR onEcho() {
  const int8_t i = active;
  if (i < 0) return;
  Sensor& s = sensors[i];
  if (digitalRead(s.echo_pin) == HIGH) {
    s.rise_us = micros();
  } else if (s.rise_us) {
    s.width_us = micros() - s.rise_us;
    s.rise_us = 0;
    s.have_reading = true;
  }
}

inline uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

inline void begin(const uint8_t trig, const uint8_t echo_pins[COUNT]) {
  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);

  for (uint8_t i = 0; i < COUNT; i++) {
    sensors[i].echo_pin = echo_pins[i];
    sensors[i].value_mm = NO_ECHO;
    sensors[i].have_reading = false;
    sensors[i].rise_us = 0;
    sensors[i].history_i = 0;
    for (uint8_t k = 0; k < 3; k++) sensors[i].history[k] = NO_ECHO;

    pinMode(echo_pins[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(echo_pins[i]), onEcho, CHANGE);
  }
}

// Non-blocking. Call frequently; it fires at most one ping per interval and
// harvests the previous result.
inline void poll(const uint8_t trig) {
  const uint32_t now = millis();
  if (now - last_fire_ms < CYCLE_INTERVAL_MS) return;
  last_fire_ms = now;

  // Harvest whatever the previous ping produced before starting the next.
  if (active >= 0) {
    Sensor& s = sensors[active];
    uint16_t mm = NO_ECHO;
    if (s.have_reading && s.width_us < ECHO_TIMEOUT_US) {
      const uint32_t d = static_cast<uint32_t>(s.width_us * US_TO_MM);
      mm = (d > 0 && d <= MAX_RANGE_MM) ? static_cast<uint16_t>(d) : NO_ECHO;
    }
    s.history[s.history_i] = mm;
    s.history_i = (s.history_i + 1) % 3;

    // A median over three where two are NO_ECHO correctly yields NO_ECHO —
    // NO_ECHO being the largest possible value makes this work for free.
    s.value_mm = median3(s.history[0], s.history[1], s.history[2]);
    s.have_reading = false;
    s.width_us = 0;
    s.rise_us = 0;
  }

  // Fire the next one.
  active = next_sensor;
  next_sensor = (next_sensor + 1) % COUNT;

  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
}

inline uint16_t value(uint8_t i) { return sensors[i].value_mm; }

// Closest valid reading across all sensors, or NO_ECHO.
inline uint16_t closest() {
  uint16_t m = NO_ECHO;
  for (uint8_t i = 0; i < COUNT; i++) {
    if (sensors[i].value_mm < m) m = sensors[i].value_mm;
  }
  return m;
}

}  // namespace ultrasonic
