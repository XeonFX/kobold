// Pure framing codec — no Arduino, no FreeRTOS, no I/O.
//
// Kept free of platform dependencies so the exact bytes the firmware puts on
// the wire can be unit-tested on a development machine (`pio test -e native`)
// and checked against the Python implementation's test vectors. A framing bug
// found on a laptop costs a minute; the same bug found on a robot costs an
// afternoon of staring at a logic analyser.
//
// Wire format — see protocol/protocol.yaml:
//
//     [ COBS-encoded frame ][ 0x00 ]
//     frame = VER(u8) TYPE(u8) SEQ(u8) LEN(u8) PAYLOAD(0..255) CRC16(u16)
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "protocol_generated.h"

namespace kobold {

static constexpr uint8_t FRAME_HEADER_LEN = 4;  // ver, type, seq, len
static constexpr uint8_t FRAME_CRC_LEN = 2;
static constexpr uint16_t MAX_PAYLOAD = 255;
static constexpr uint16_t MAX_FRAME = FRAME_HEADER_LEN + MAX_PAYLOAD + FRAME_CRC_LEN;
static constexpr uint16_t MAX_ENCODED = MAX_FRAME + (MAX_FRAME / 254) + 2;

// CRC16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor.
uint16_t crc16(const uint8_t* data, size_t len);

// Consistent Overhead Byte Stuffing. `out` must have room for MAX_ENCODED.
// Returns the encoded length; the caller appends the 0x00 delimiter.
size_t cobs_encode(const uint8_t* in, size_t len, uint8_t* out);

// Returns the decoded length, or 0 if the input is malformed.
size_t cobs_decode(const uint8_t* in, size_t len, uint8_t* out, size_t out_cap);

// Build a complete frame (without the trailing delimiter) into `out`.
// Returns the encoded length, or 0 if the payload is too long.
size_t build_frame(uint8_t version, uint8_t type, uint8_t seq, const void* payload,
                   uint8_t len, uint8_t* out, size_t out_cap);

enum class ParseResult : uint8_t {
  Ok = 0,
  Malformed,       // COBS decode failed or the frame is too short
  BadCrc,          //
  BadVersion,      // protocol version skew — refuse to act
  LengthMismatch,  // declared length disagrees with the payload
};

struct ParsedFrame {
  uint8_t version;
  uint8_t type;
  uint8_t seq;
  const uint8_t* payload;
  uint8_t len;
};

// Decode one COBS-encoded frame body (delimiter already stripped). `scratch`
// must hold at least MAX_FRAME bytes and owns the memory `out.payload` points
// into, so it must outlive any use of the result.
ParseResult parse_frame(const uint8_t* encoded, size_t encoded_len, uint8_t* scratch,
                        size_t scratch_cap, ParsedFrame* out);

}  // namespace kobold
