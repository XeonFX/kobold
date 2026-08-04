"""Host side of the kobold framed serial link.

Mirrors firmware/lib/kobold_protocol/kobold_link.cpp exactly. The shared test
vectors in tests/test_link.py pin both implementations to the same bytes — if
you change the framing, change it in both places and update the vectors.
"""

from __future__ import annotations

import logging
from collections.abc import Callable, Iterator
from dataclasses import dataclass

from .protocol_generated import DECODERS, MSG_NAMES, PROTOCOL_VERSION

log = logging.getLogger(__name__)

FRAME_HEADER_LEN = 4
FRAME_CRC_LEN = 2
MAX_PAYLOAD = 255
MAX_FRAME = FRAME_HEADER_LEN + MAX_PAYLOAD + FRAME_CRC_LEN
MAX_ENCODED = MAX_FRAME + (MAX_FRAME // 254) + 2

DELIMITER = b"\x00"


class ProtocolVersionError(RuntimeError):
    """Firmware speaks a different protocol version than this bridge."""


def crc16(data: bytes) -> int:
    """CRC16/CCITT-FALSE — poly 0x1021, init 0xFFFF, no reflection, no xorout."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    code_i = 0
    out.append(0)  # placeholder for the first code byte
    code = 1

    for byte in data:
        if byte == 0:
            out[code_i] = code
            code_i = len(out)
            out.append(0)
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_i] = code
                code_i = len(out)
                out.append(0)
                code = 1

    out[code_i] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    n = len(data)

    while i < n:
        code = data[i]
        if code == 0 or i + code > n:
            raise ValueError("malformed COBS frame")
        i += 1
        out += data[i : i + code - 1]
        i += code - 1
        if code != 0xFF and i < n:
            out.append(0)

    return bytes(out)


@dataclass
class Frame:
    type: int
    seq: int
    payload: bytes

    @property
    def name(self) -> str:
        return MSG_NAMES.get(self.type, f"unknown(0x{self.type:02X})")

    def decode(self):
        """Return the typed NamedTuple for this message, or None if unknown."""
        decoder = DECODERS.get(self.type)
        return decoder.unpack(self.payload) if decoder else None


@dataclass
class LinkStats:
    rx_frames: int = 0
    rx_crc_errors: int = 0
    rx_version_errors: int = 0
    rx_malformed: int = 0
    tx_frames: int = 0


def encode_frame(msg_type: int, payload: bytes = b"", seq: int = 0) -> bytes:
    """Build one complete on-the-wire frame including its trailing delimiter."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload {len(payload)} exceeds {MAX_PAYLOAD}")

    body = bytes([PROTOCOL_VERSION, msg_type, seq & 0xFF, len(payload)]) + payload
    crc = crc16(body)
    body += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    return cobs_encode(body) + DELIMITER


class FrameDecoder:
    """Incremental decoder. Feed it arbitrary byte chunks; it yields Frames.

    Never raises on bad input — a robot link sees corruption, and the right
    response is to count it and resynchronise, not to crash the bridge.
    """

    def __init__(self, on_version_mismatch: Callable[[int], None] | None = None):
        self._buf = bytearray()
        self._overflow = False
        self.stats = LinkStats()
        self._on_version_mismatch = on_version_mismatch

    def feed(self, chunk: bytes) -> Iterator[Frame]:
        for byte in chunk:
            if byte == 0x00:
                if not self._overflow and self._buf:
                    frame = self._parse(bytes(self._buf))
                    if frame is not None:
                        yield frame
                self._buf.clear()
                self._overflow = False
                continue

            if len(self._buf) >= MAX_ENCODED:
                if not self._overflow:
                    self.stats.rx_malformed += 1
                self._overflow = True
                continue

            self._buf.append(byte)

    def _parse(self, encoded: bytes) -> Frame | None:
        try:
            body = cobs_decode(encoded)
        except ValueError:
            self.stats.rx_malformed += 1
            return None

        if len(body) < FRAME_HEADER_LEN + FRAME_CRC_LEN:
            self.stats.rx_malformed += 1
            return None

        payload_end = len(body) - FRAME_CRC_LEN
        got = body[payload_end] | (body[payload_end + 1] << 8)
        if got != crc16(body[:payload_end]):
            self.stats.rx_crc_errors += 1
            return None

        ver, msg_type, seq, length = body[0], body[1], body[2], body[3]

        if ver != PROTOCOL_VERSION:
            self.stats.rx_version_errors += 1
            if self._on_version_mismatch:
                self._on_version_mismatch(ver)
            return None

        payload = body[FRAME_HEADER_LEN:payload_end]
        if length != len(payload):
            self.stats.rx_malformed += 1
            return None

        self.stats.rx_frames += 1
        return Frame(type=msg_type, seq=seq, payload=payload)
