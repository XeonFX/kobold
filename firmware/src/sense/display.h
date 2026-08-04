// SSD1306 status display.
//
// U8g2 in *page buffer* mode (the `_1_` variant): 128 bytes of RAM instead of
// the 1 KB a full frame buffer costs. Blocking I2C during the page loop is fine
// here because echo pulses are captured by interrupt, not by polling — see
// ultrasonic.h. Refresh is throttled well below the telemetry rate; nobody
// reads a 16 Hz display.
#pragma once

#include <Arduino.h>
#include <U8g2lib.h>

#include "pins.h"

namespace display {

inline U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
inline bool present = false;
inline char lines[4][22] = {{0}};

inline bool begin() {
  present = u8g2.begin();
  if (present) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setDrawColor(1);
  }
  return present;
}

inline void render() {
  if (!present) return;
  u8g2.firstPage();
  do {
    for (uint8_t row = 0; row < 4; row++) {
      u8g2.drawStr(0, 12 + row * 13, lines[row]);
    }
  } while (u8g2.nextPage());
}

inline void setLine(uint8_t row, const char* text) {
  if (row >= 4) return;
  strncpy(lines[row], text, sizeof(lines[0]) - 1);
  lines[row][sizeof(lines[0]) - 1] = '\0';
  render();
}

inline void showRanges(uint16_t f, uint16_t b, uint16_t l, uint16_t r, bool safety) {
  static uint32_t last = 0;
  if (millis() - last < 250) return;
  last = millis();

  // Distances shown in centimetres; "---" for no echo.
  auto fmt = [](uint16_t mm, char* out, size_t n) {
    if (mm == 0xFFFF) strncpy(out, "---", n);
    else snprintf(out, n, "%u", mm / 10);
  };

  char fs[6], bs[6], ls[6], rs[6];
  fmt(f, fs, sizeof(fs));
  fmt(b, bs, sizeof(bs));
  fmt(l, ls, sizeof(ls));
  fmt(r, rs, sizeof(rs));

  snprintf(lines[1], sizeof(lines[1]), "F%-5s B%-5s", fs, bs);
  snprintf(lines[2], sizeof(lines[2]), "L%-5s R%-5s", ls, rs);
  snprintf(lines[3], sizeof(lines[3]), "%s", safety ? "!! STOP !!" : "ok");

  render();
}

}  // namespace display
