#include "kobold_codec.h"

#include <string.h>

namespace kobold {

uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

size_t cobs_encode(const uint8_t* in, size_t len, uint8_t* out) {
  size_t read_i = 0, write_i = 1, code_i = 0;
  uint8_t code = 1;

  while (read_i < len) {
    if (in[read_i] == 0) {
      out[code_i] = code;
      code_i = write_i++;
      code = 1;
      read_i++;
    } else {
      out[write_i++] = in[read_i++];
      code++;
      if (code == 0xFF) {
        out[code_i] = code;
        code_i = write_i++;
        code = 1;
      }
    }
  }
  out[code_i] = code;
  return write_i;
}

size_t cobs_decode(const uint8_t* in, size_t len, uint8_t* out, size_t out_cap) {
  size_t read_i = 0, write_i = 0;

  while (read_i < len) {
    const uint8_t code = in[read_i];
    if (code == 0 || read_i + code > len) return 0;  // malformed
    read_i++;

    for (uint8_t i = 1; i < code; i++) {
      if (write_i >= out_cap) return 0;
      out[write_i++] = in[read_i++];
    }
    // A 0xFF group is a full 254-byte run with no implicit zero following it.
    if (code != 0xFF && read_i < len) {
      if (write_i >= out_cap) return 0;
      out[write_i++] = 0;
    }
  }
  return write_i;
}

size_t build_frame(uint8_t version, uint8_t type, uint8_t seq, const void* payload,
                   uint8_t len, uint8_t* out, size_t out_cap) {
  uint8_t frame[MAX_FRAME];
  frame[0] = version;
  frame[1] = type;
  frame[2] = seq;
  frame[3] = len;
  if (len && payload) memcpy(frame + FRAME_HEADER_LEN, payload, len);

  const size_t body = FRAME_HEADER_LEN + len;
  const uint16_t crc = crc16(frame, body);
  frame[body] = static_cast<uint8_t>(crc & 0xFF);
  frame[body + 1] = static_cast<uint8_t>(crc >> 8);

  const size_t total = body + FRAME_CRC_LEN;
  if (out_cap < total + (total / 254) + 2) return 0;
  return cobs_encode(frame, total, out);
}

ParseResult parse_frame(const uint8_t* encoded, size_t encoded_len, uint8_t* scratch,
                        size_t scratch_cap, ParsedFrame* out) {
  const size_t n = cobs_decode(encoded, encoded_len, scratch, scratch_cap);
  if (n < FRAME_HEADER_LEN + FRAME_CRC_LEN) return ParseResult::Malformed;

  const size_t body = n - FRAME_CRC_LEN;
  const uint16_t got = static_cast<uint16_t>(scratch[body]) |
                       (static_cast<uint16_t>(scratch[body + 1]) << 8);
  if (got != crc16(scratch, body)) return ParseResult::BadCrc;

  out->version = scratch[0];
  out->type = scratch[1];
  out->seq = scratch[2];
  out->len = scratch[3];
  out->payload = scratch + FRAME_HEADER_LEN;

  // Version is checked after the CRC so that a corrupted byte is reported as
  // corruption rather than as a spurious version mismatch — the two get very
  // different responses from the bridge.
  if (out->version != PROTOCOL_VERSION) return ParseResult::BadVersion;
  if (out->len != body - FRAME_HEADER_LEN) return ParseResult::LengthMismatch;

  return ParseResult::Ok;
}

}  // namespace kobold
