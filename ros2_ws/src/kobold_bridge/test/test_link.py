"""Framing codec tests — the Python half of the cross-language contract.

The vectors in test_cross_language_vectors are byte-identical to those asserted
by firmware/test/test_codec/test_codec.cpp. If the two framing implementations ever
drift apart, one of these suites goes red long before a robot goes quiet on the
bench.

    pytest ros2_ws/src/kobold_bridge/test/
"""

import random

import pytest
from kobold_bridge.link import (
    FrameDecoder,
    cobs_decode,
    cobs_encode,
    crc16,
    encode_frame,
)
from kobold_bridge.protocol_generated import (
    MSG_CMD_VEL,
    MSG_TELEMETRY,
    MSG_VERSION_REQ,
    PROTOCOL_VERSION,
    CmdVel,
    Telemetry,
)

# ------------------------------------------------------------------ CRC16 --

def test_crc16_known_vectors():
    assert crc16(b"123456789") == 0x29B1  # CRC16/CCITT-FALSE reference
    assert crc16(b"") == 0xFFFF
    assert crc16(b"\x00") == 0xE1F0


# ------------------------------------------------------------------- COBS --

@pytest.mark.parametrize(
    "data",
    [
        b"",
        b"\x11\x22\x33",
        b"\x00\x00\x00",
        b"\x00\x11\x00\x00\x22",
        bytes(range(1, 255)),          # 254 non-zero bytes: group continuation
        bytes(range(1, 256)),          # 255
        bytes(range(1, 256)) + b"\x01",
    ],
)
def test_cobs_round_trip(data):
    encoded = cobs_encode(data)
    assert 0 not in encoded, "COBS emitted an interior zero"
    assert cobs_decode(encoded) == data


def test_cobs_rejects_malformed():
    with pytest.raises(ValueError):
        cobs_decode(b"\x00\x01")
    with pytest.raises(ValueError):
        cobs_decode(b"\x08\x01\x02")  # code promises more data than exists


# ----------------------------------------------------------------- frames --

def test_frame_round_trip():
    payload = b"\xde\xad\xbe\xef"
    frames = list(FrameDecoder().feed(encode_frame(MSG_CMD_VEL, payload, seq=42)))
    assert len(frames) == 1
    assert frames[0].type == MSG_CMD_VEL
    assert frames[0].seq == 42
    assert frames[0].payload == payload


def test_frame_empty_payload():
    frames = list(FrameDecoder().feed(encode_frame(MSG_VERSION_REQ)))
    assert len(frames) == 1 and frames[0].payload == b""


def test_frame_max_payload():
    payload = bytes(i & 0xFF for i in range(255))
    frames = list(FrameDecoder().feed(encode_frame(MSG_TELEMETRY, payload)))
    assert frames[0].payload == payload


def test_payload_too_long_raises():
    with pytest.raises(ValueError):
        encode_frame(MSG_TELEMETRY, b"\x00" * 256)


def test_typed_round_trip():
    t = Telemetry(
        t_ms=1234, ticks_fl=10, ticks_fr=-10, ticks_rl=10, ticks_rr=-10,
        gyro_x=1, gyro_y=2, gyro_z=3, accel_x=4, accel_y=5, accel_z=6,
        batt_mv=11800, cliff_mask=0b0010, flags=0x01,
        meas_l_mm_s=150, meas_r_mm_s=-150,
    )
    frames = list(FrameDecoder().feed(encode_frame(MSG_TELEMETRY, t.pack())))
    assert frames[0].decode() == t


# ------------------------------------------------------------ corruption --

def test_single_bit_corruption_is_always_detected():
    wire = encode_frame(MSG_CMD_VEL, CmdVel(300, -120).pack(), seq=7)
    body = wire[:-1]

    for i in range(len(body)):
        for bit in range(8):
            corrupt = bytearray(body)
            corrupt[i] ^= 1 << bit
            if corrupt[i] == 0:
                continue  # becomes a delimiter, not a corrupt frame
            got = list(FrameDecoder().feed(bytes(corrupt) + b"\x00"))
            assert got == [], f"corruption at byte {i} bit {bit} parsed as valid"


def test_decoder_resynchronises_after_garbage():
    decoder = FrameDecoder()
    good = encode_frame(MSG_CMD_VEL, CmdVel(100, 0).pack(), seq=1)

    corrupt = bytearray(encode_frame(MSG_CMD_VEL, CmdVel(999, 0).pack(), seq=2))
    corrupt[2] ^= 0xFF

    noise = bytes([random.randint(1, 255) for _ in range(37)]) + b"\x00"

    frames = list(decoder.feed(bytes(corrupt) + noise + good))
    assert len(frames) == 1
    assert frames[0].decode() == CmdVel(100, 0)
    assert decoder.stats.rx_crc_errors >= 1


def test_version_skew_is_counted_not_raised():
    seen = []
    decoder = FrameDecoder(on_version_mismatch=seen.append)

    payload = CmdVel(0, 0).pack()
    body = bytes([PROTOCOL_VERSION + 1, MSG_CMD_VEL, 0, len(payload)]) + payload
    crc = crc16(body)
    body += bytes([crc & 0xFF, crc >> 8])
    wire = cobs_encode(body) + b"\x00"

    assert list(decoder.feed(wire)) == []
    assert decoder.stats.rx_version_errors == 1
    assert seen == [PROTOCOL_VERSION + 1]


def test_split_across_chunks():
    """A real serial read returns arbitrary chunk boundaries."""
    decoder = FrameDecoder()
    wire = encode_frame(MSG_CMD_VEL, CmdVel(42, -42).pack())

    frames = []
    for i in range(len(wire)):
        frames.extend(decoder.feed(wire[i : i + 1]))
    assert len(frames) == 1 and frames[0].decode() == CmdVel(42, -42)


def test_oversized_garbage_does_not_wedge_decoder():
    decoder = FrameDecoder()
    flood = bytes([0x42] * 5000)
    good = encode_frame(MSG_CMD_VEL, CmdVel(1, 1).pack())

    frames = list(decoder.feed(flood + b"\x00" + good))
    assert len(frames) == 1
    assert decoder.stats.rx_malformed >= 1


# ------------------------------------------------------- cross-language --

def test_cross_language_vectors():
    """Byte-for-byte agreement with firmware/test/test_codec/test_codec.cpp.

    cmd_vel(linear=300, angular=-120), seq=0:

        frame body = 01 11 00 04 | 2C 01 88 FF | 7C B3
                     ver type seq len  payload    crc(LE)

    seq=0 puts a zero inside the frame, so this also exercises COBS actually
    doing its job rather than passing data through untouched.
    """
    wire = encode_frame(MSG_CMD_VEL, CmdVel(300, -120).pack(), seq=0)

    expected = bytes(
        [0x03, 0x01, 0x11, 0x08, 0x04, 0x2C, 0x01, 0x88, 0xFF, 0x7C, 0xB3, 0x00]
    )
    assert wire == expected, f"got {wire.hex(' ')}, want {expected.hex(' ')}"


def test_random_payloads_never_emit_interior_zero():
    """The invariant that makes 0x00 a usable frame delimiter."""
    for _ in range(500):
        payload = bytes(random.randint(0, 255) for _ in range(random.randint(0, 200)))
        wire = encode_frame(MSG_TELEMETRY, payload, seq=random.randint(0, 255))
        assert 0 not in wire[:-1], "interior zero would break framing"
        assert next(iter(FrameDecoder().feed(wire))).payload == payload
