"""RK3588 hardware video encode via MPP, with a software fallback.

The vendor GStreamer plugin (gstreamer1.0-rockchip1 1.14-4) manages 0.9 fps for
work MPP itself does at 250-300, stalling in poll_packet_locked. Its upstream
repository is a 404 and Radxa ships no newer build, so this talks to MPP
directly through a small C shim instead.

Measured, 1280x960, 60 distinct real frames:

    MJPEG q80    2.51 ms cpu/frame   57.1 kB   5.61 Mbit/s @12fps   3.0% of a core
    H.264 2Mbit  1.26 ms cpu/frame    7.0 kB   0.69 Mbit/s @12fps   1.5% of a core

Software equivalents at the same rate: jpegenc ~6.8% of a core, x264enc 70.4%.
"""

from __future__ import annotations

import ctypes
import os

import numpy as np

MJPEG, H264 = 0, 1

_LIB = os.environ.get("KOBOLD_MPP_LIB", "/usr/lib/libkobold_mppenc.so")
_lib = None
_reason = "not initialised"


def _init() -> None:
    global _lib, _reason
    if _lib is not None or _reason != "not initialised":
        return
    if not os.path.exists("/dev/mpp_service"):
        _reason = "/dev/mpp_service not present (pass --device /dev/mpp_service)"
        return
    try:
        lib = ctypes.CDLL(_LIB)
    except OSError as exc:
        _reason = f"cannot load {_LIB}: {exc}"
        return
    lib.kobold_mpp_enc_create.argtypes = [ctypes.c_int] * 4
    lib.kobold_mpp_enc_create.restype = ctypes.c_void_p
    lib.kobold_mpp_enc_encode.argtypes = [ctypes.c_void_p, ctypes.c_void_p,
                                          ctypes.c_void_p, ctypes.c_int]
    lib.kobold_mpp_enc_encode.restype = ctypes.c_int
    lib.kobold_mpp_enc_destroy.argtypes = [ctypes.c_void_p]
    _lib = lib
    _reason = "ok"


def available() -> bool:
    _init()
    return _lib is not None


def status() -> str:
    _init()
    return "MPP hardware encode active" if _lib else f"MPP unavailable — {_reason}"


class Encoder:
    """One hardware encoder context. Not thread-safe; give each stream its own.

    Falls back to cv2.imencode for MJPEG if MPP is unavailable, so a missing
    accelerator degrades the stream rather than breaking it. There is no
    software fallback for H.264 — x264enc at 70% of a core is not something to
    enable silently.
    """

    def __init__(self, width: int, height: int, coding: int = MJPEG, quality: int = 80):
        _init()
        self.width, self.height, self.coding = width, height, coding
        self._h = None
        self._out = np.empty(width * height * 3, dtype=np.uint8)
        if _lib is not None:
            self._h = _lib.kobold_mpp_enc_create(width, height, coding, quality)
            if not self._h:
                self._h = None

    @property
    def hardware(self) -> bool:
        return self._h is not None

    def encode(self, nv12: np.ndarray) -> bytes | None:
        """NV12 (H*3/2, W) uint8 -> encoded bytes, or None on failure."""
        if self._h is not None:
            src = np.ascontiguousarray(nv12)
            n = _lib.kobold_mpp_enc_encode(self._h, src.ctypes.data,
                                           self._out.ctypes.data, self._out.size)
            if n > 0:
                return bytes(self._out[:n])
            return None

        if self.coding != MJPEG:
            return None
        import cv2
        bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        return buf.tobytes() if ok else None

    def close(self) -> None:
        if self._h is not None and _lib is not None:
            _lib.kobold_mpp_enc_destroy(self._h)
            self._h = None

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()
