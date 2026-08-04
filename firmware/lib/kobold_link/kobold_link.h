// Arduino transport for the framed serial link.
//
// Thin wrapper over kobold_codec.h — all framing logic lives there so it can
// be tested natively. This file owns only the Stream plumbing, the receive
// buffer, and the TX mutex.
#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "kobold_codec.h"
#include "protocol_generated.h"

namespace kobold {

// Runs on whichever task calls poll(). Keep handlers short — anything slow
// belongs in a task, not here.
typedef void (*FrameHandler)(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len);

class Link {
 public:
  void begin(Stream& io, FrameHandler handler);

  // Drain the input, dispatching any complete frames.
  void poll();

  // Thread-safe: the control task can log while the comms task streams
  // telemetry without interleaving bytes on the wire.
  bool send(uint8_t type, const void* payload, uint8_t len);

  template <typename T>
  bool send(uint8_t type, const T& msg) {
    static_assert(sizeof(T) <= MAX_PAYLOAD, "payload too large");
    return send(type, &msg, static_cast<uint8_t>(sizeof(T)));
  }

  bool sendEmpty(uint8_t type) { return send(type, nullptr, 0); }
  bool sendLog(uint8_t level, const char* text);

  // Link quality, surfaced in telemetry so it is visible without a scope.
  uint32_t rxFrames() const { return rx_frames_; }
  uint32_t rxCrcErrors() const { return rx_crc_errors_; }
  uint32_t rxVersionErrors() const { return rx_version_errors_; }
  uint32_t rxOverruns() const { return rx_overruns_; }

 private:
  void dispatch();

  Stream* io_ = nullptr;
  FrameHandler handler_ = nullptr;
  SemaphoreHandle_t tx_mutex_ = nullptr;

  uint8_t rx_buf_[MAX_ENCODED];
  size_t rx_len_ = 0;
  bool rx_overflow_ = false;
  uint8_t tx_seq_ = 0;

  uint32_t rx_frames_ = 0;
  uint32_t rx_crc_errors_ = 0;
  uint32_t rx_version_errors_ = 0;
  uint32_t rx_overruns_ = 0;
};

}  // namespace kobold
