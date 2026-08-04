#include "kobold_link.h"

#include <string.h>

namespace kobold {

void Link::begin(Stream& io, FrameHandler handler) {
  io_ = &io;
  handler_ = handler;
  if (!tx_mutex_) tx_mutex_ = xSemaphoreCreateMutex();
  rx_len_ = 0;
  rx_overflow_ = false;
}

void Link::poll() {
  if (!io_) return;

  while (io_->available()) {
    const int c = io_->read();
    if (c < 0) break;

    if (c == 0x00) {
      if (!rx_overflow_ && rx_len_ > 0) dispatch();
      rx_len_ = 0;
      rx_overflow_ = false;
      continue;
    }

    if (rx_len_ >= sizeof(rx_buf_)) {
      // Longer than any legal frame. Drop it and wait for the next delimiter
      // rather than emitting garbage upward.
      if (!rx_overflow_) rx_overruns_++;
      rx_overflow_ = true;
      continue;
    }
    rx_buf_[rx_len_++] = static_cast<uint8_t>(c);
  }
}

void Link::dispatch() {
  uint8_t scratch[MAX_FRAME];
  ParsedFrame f{};

  switch (parse_frame(rx_buf_, rx_len_, scratch, sizeof(scratch), &f)) {
    case ParseResult::Ok:
      rx_frames_++;
      if (handler_) handler_(f.type, f.seq, f.payload, f.len);
      return;

    case ParseResult::BadVersion:
      // Refuse to act on a frame from a mismatched build. The bridge performs
      // the same check and stops — silent version skew is exactly the failure
      // mode that makes remote firmware updates dangerous.
      rx_version_errors_++;
      return;

    case ParseResult::BadCrc:
    case ParseResult::Malformed:
    case ParseResult::LengthMismatch:
    default:
      rx_crc_errors_++;
      return;
  }
}

bool Link::send(uint8_t type, const void* payload, uint8_t len) {
  if (!io_) return false;
  if (tx_mutex_ && xSemaphoreTake(tx_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) return false;

  uint8_t encoded[MAX_ENCODED];
  const size_t n = build_frame(PROTOCOL_VERSION, type, tx_seq_++, payload, len, encoded,
                               sizeof(encoded));
  if (n) {
    io_->write(encoded, n);
    io_->write(static_cast<uint8_t>(0x00));
  }

  if (tx_mutex_) xSemaphoreGive(tx_mutex_);
  return n != 0;
}

bool Link::sendLog(uint8_t level, const char* text) {
  uint8_t buf[MAX_PAYLOAD];
  buf[0] = level;
  const size_t n = strnlen(text, MAX_PAYLOAD - 1);
  memcpy(buf + 1, text, n);
  return send(MSG_LOG, buf, static_cast<uint8_t>(n + 1));
}

}  // namespace kobold
