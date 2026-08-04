// Host-side tests for the framing codec.  `pio test -e native`
//
// The cross-language vectors below are the important part: they are the same
// bytes asserted by tests/test_link.py on the Python side. If the two framing
// implementations ever drift apart, one of these two test suites fails long
// before a robot goes quiet on the bench.
#include <string.h>
#include <unity.h>

#include <cstdio>
#include <cstdlib>

#include <kobold_codec.h>

using namespace kobold;

// ---------------------------------------------------------------- CRC16 ----

void test_crc16_known_vectors() {
  // CRC16/CCITT-FALSE reference value for "123456789".
  const uint8_t check[] = "123456789";
  TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16(check, 9));

  TEST_ASSERT_EQUAL_HEX16(0xFFFF, crc16(nullptr, 0));

  const uint8_t zero = 0x00;
  TEST_ASSERT_EQUAL_HEX16(0xE1F0, crc16(&zero, 1));
}

// ----------------------------------------------------------------- COBS ----

static void round_trip(const uint8_t* data, size_t len) {
  uint8_t enc[MAX_ENCODED];
  const size_t n = cobs_encode(data, len, enc);

  // The invariant that makes 0x00 a usable delimiter.
  for (size_t i = 0; i < n; i++) {
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, enc[i], "COBS emitted an interior zero");
  }

  uint8_t dec[MAX_FRAME];
  const size_t m = cobs_decode(enc, n, dec, sizeof(dec));
  TEST_ASSERT_EQUAL_UINT32(len, m);
  if (len) TEST_ASSERT_EQUAL_UINT8_ARRAY(data, dec, len);
}

void test_cobs_round_trip_basic() {
  const uint8_t a[] = {0x11, 0x22, 0x33};
  round_trip(a, sizeof(a));

  const uint8_t zeros[] = {0x00, 0x00, 0x00};
  round_trip(zeros, sizeof(zeros));

  const uint8_t mixed[] = {0x00, 0x11, 0x00, 0x00, 0x22};
  round_trip(mixed, sizeof(mixed));

  round_trip(nullptr, 0);
}

void test_cobs_round_trip_long_runs() {
  // 254+ non-zero bytes exercises the 0xFF group-continuation path, which is
  // the part of COBS that is easy to get subtly wrong.
  uint8_t buf[300];
  for (size_t i = 0; i < sizeof(buf); i++) buf[i] = static_cast<uint8_t>((i % 255) + 1);
  round_trip(buf, 254);
  round_trip(buf, 255);
  round_trip(buf, 256);
}

void test_cobs_rejects_malformed() {
  uint8_t out[64];
  const uint8_t bad_zero[] = {0x00, 0x01};
  TEST_ASSERT_EQUAL_UINT32(0, cobs_decode(bad_zero, sizeof(bad_zero), out, sizeof(out)));

  // Code byte promising more data than the buffer holds.
  const uint8_t overrun[] = {0x08, 0x01, 0x02};
  TEST_ASSERT_EQUAL_UINT32(0, cobs_decode(overrun, sizeof(overrun), out, sizeof(out)));
}

// ---------------------------------------------------------------- frames ----

void test_frame_round_trip() {
  const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
  uint8_t enc[MAX_ENCODED];
  const size_t n =
      build_frame(PROTOCOL_VERSION, MSG_CMD_VEL, 42, payload, sizeof(payload), enc, sizeof(enc));
  TEST_ASSERT_GREATER_THAN_UINT32(0, n);

  uint8_t scratch[MAX_FRAME];
  ParsedFrame f{};
  TEST_ASSERT_EQUAL(ParseResult::Ok, parse_frame(enc, n, scratch, sizeof(scratch), &f));
  TEST_ASSERT_EQUAL_UINT8(MSG_CMD_VEL, f.type);
  TEST_ASSERT_EQUAL_UINT8(42, f.seq);
  TEST_ASSERT_EQUAL_UINT8(sizeof(payload), f.len);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, f.payload, sizeof(payload));
}

void test_frame_empty_payload() {
  uint8_t enc[MAX_ENCODED];
  const size_t n =
      build_frame(PROTOCOL_VERSION, MSG_VERSION_REQ, 0, nullptr, 0, enc, sizeof(enc));

  uint8_t scratch[MAX_FRAME];
  ParsedFrame f{};
  TEST_ASSERT_EQUAL(ParseResult::Ok, parse_frame(enc, n, scratch, sizeof(scratch), &f));
  TEST_ASSERT_EQUAL_UINT8(0, f.len);
}

void test_frame_max_payload() {
  uint8_t payload[MAX_PAYLOAD];
  for (size_t i = 0; i < sizeof(payload); i++) payload[i] = static_cast<uint8_t>(i);

  uint8_t enc[MAX_ENCODED];
  const size_t n = build_frame(PROTOCOL_VERSION, MSG_TELEMETRY, 1, payload, MAX_PAYLOAD, enc,
                               sizeof(enc));
  TEST_ASSERT_GREATER_THAN_UINT32(0, n);

  uint8_t scratch[MAX_FRAME];
  ParsedFrame f{};
  TEST_ASSERT_EQUAL(ParseResult::Ok, parse_frame(enc, n, scratch, sizeof(scratch), &f));
  TEST_ASSERT_EQUAL_UINT8(MAX_PAYLOAD, f.len);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, f.payload, MAX_PAYLOAD);
}

void test_frame_detects_corruption() {
  const uint8_t payload[] = {0x01, 0x02, 0x03, 0x04};
  uint8_t enc[MAX_ENCODED];
  const size_t n =
      build_frame(PROTOCOL_VERSION, MSG_CMD_VEL, 7, payload, sizeof(payload), enc, sizeof(enc));

  // Flip a bit in every position in turn; not one may parse as valid.
  for (size_t i = 0; i < n; i++) {
    uint8_t corrupt[MAX_ENCODED];
    memcpy(corrupt, enc, n);
    corrupt[i] ^= 0x01;
    if (corrupt[i] == 0x00) continue;  // would be read as a delimiter, not a frame

    uint8_t scratch[MAX_FRAME];
    ParsedFrame f{};
    const ParseResult r = parse_frame(corrupt, n, scratch, sizeof(scratch), &f);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(ParseResult::Ok, r, "corrupted frame parsed as valid");
  }
}

void test_frame_rejects_version_skew() {
  const uint8_t payload[] = {0x00};
  uint8_t enc[MAX_ENCODED];
  const size_t n = build_frame(PROTOCOL_VERSION + 1, MSG_CMD_VEL, 0, payload, 1, enc, sizeof(enc));

  uint8_t scratch[MAX_FRAME];
  ParsedFrame f{};
  TEST_ASSERT_EQUAL(ParseResult::BadVersion, parse_frame(enc, n, scratch, sizeof(scratch), &f));
}

// ------------------------------------------------------- cross-language ----

// These exact bytes are asserted by tests/test_link.py. Both implementations
// are pinned to them; neither can drift without a test going red.
void test_cross_language_vectors() {
  struct Vector {
    uint8_t type;
    uint8_t seq;
    uint8_t payload[8];
    uint8_t payload_len;
    uint8_t expected[24];
    uint8_t expected_len;
  };

  // cmd_vel(linear=300, angular=-120), seq=0
  //
  //   frame body = 01 11 00 04 | 2C 01 88 FF | 7C B3
  //                ^ver ^type ^seq ^len  payload   crc(LE)
  //
  //   seq=0 puts a zero inside the frame, so this vector also exercises COBS
  //   doing its actual job rather than passing data through untouched:
  //     group 1: code 03, then 01 11   (two bytes, then the elided zero)
  //     group 2: code 08, then 04 2C 01 88 FF 7C B3
  const Vector v1 = {MSG_CMD_VEL,
                     0,
                     {0x2C, 0x01, 0x88, 0xFF},
                     4,
                     {0x03, 0x01, 0x11, 0x08, 0x04, 0x2C, 0x01, 0x88, 0xFF, 0x7C, 0xB3},
                     11};

  uint8_t enc[MAX_ENCODED];
  const size_t n = build_frame(PROTOCOL_VERSION, v1.type, v1.seq, v1.payload, v1.payload_len,
                               enc, sizeof(enc));

  char msg[128];
  snprintf(msg, sizeof(msg), "encoded %u bytes, expected %u", (unsigned)n,
           (unsigned)v1.expected_len);
  TEST_ASSERT_EQUAL_UINT32_MESSAGE(v1.expected_len, n, msg);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(v1.expected, enc, v1.expected_len);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crc16_known_vectors);
  RUN_TEST(test_cobs_round_trip_basic);
  RUN_TEST(test_cobs_round_trip_long_runs);
  RUN_TEST(test_cobs_rejects_malformed);
  RUN_TEST(test_frame_round_trip);
  RUN_TEST(test_frame_empty_payload);
  RUN_TEST(test_frame_max_payload);
  RUN_TEST(test_frame_detects_corruption);
  RUN_TEST(test_frame_rejects_version_skew);
  RUN_TEST(test_cross_language_vectors);
  return UNITY_END();
}

void setUp() {}
void tearDown() {}
