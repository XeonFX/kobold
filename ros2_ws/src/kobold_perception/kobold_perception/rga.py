"""RK3588 RGA (2D accelerator) bindings, with a CPU fallback.

WHY THIS EXISTS
---------------
Colour conversion and scaling are what the RGA is for, and doing them on the
CPU instead is measurably expensive. Measured on the robot, 1280x960 NV12,
inside the perception container against a real camera frame:

    cv2  NV12->BGR                1.00 ms wall    7.73 ms cpu
    RGA  NV12->BGR                2.62 ms wall    0.98 ms cpu
    cv2  NV12->BGR + letterbox    3.82 ms wall   20.28 ms cpu
    RGA  NV12->BGR + letterbox    2.08 ms wall    0.69 ms cpu

Note that OpenCV's CPU time EXCEEDS its wall time — it is fast because it uses
every core it can find. That is borrowed, not free: on this robot those cores
are running ASR at 2.8x real time and an LLM at 8 tok/s. The wall-clock
difference is modest; the ~29x CPU difference is the point.

Correctness was verified against cv2.cvtColor on a real frame: mean absolute
difference 0.38/255, max 4, zero pixels more than 8 off. That is rounding in
the YUV->RGB matrix.

FALLBACK
--------
Every entry point degrades to OpenCV if librga, /dev/rga, or the shim is
missing. A perception node that refuses to start because an accelerator is
absent is worse than one that runs slower, so availability is checked once and
logged, never asserted.
"""

from __future__ import annotations

import ctypes
import os

import cv2
import numpy as np

_LIB_PATH = os.environ.get("KOBOLD_RGA_LIB", "/usr/lib/libkobold_rga.so")

_lib: ctypes.CDLL | None = None
_available = False
_reason = "not initialised"


def _init() -> None:
    global _lib, _available, _reason
    if _lib is not None or _reason != "not initialised":
        return
    if not os.path.exists("/dev/rga"):
        _reason = "/dev/rga not present (pass --device /dev/rga)"
        return
    try:
        lib = ctypes.CDLL(_LIB_PATH)
    except OSError as exc:
        _reason = f"cannot load {_LIB_PATH}: {exc}"
        return

    lib.kobold_rga_version.restype = ctypes.c_char_p
    lib.kobold_rga_nv12_to_bgr.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    lib.kobold_rga_nv12_to_bgr.restype = ctypes.c_int
    lib.kobold_rga_nv12_to_bgr_letterbox.argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.c_int,
        ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_ubyte]
    lib.kobold_rga_nv12_to_bgr_letterbox.restype = ctypes.c_int
    lib.kobold_rga_bgr_letterbox.argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.c_int,
        ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_ubyte]
    lib.kobold_rga_bgr_letterbox.restype = ctypes.c_int

    _lib = lib
    _available = True
    _reason = "ok"


def available() -> bool:
    _init()
    return _available


def status() -> str:
    """Human-readable one-liner for the node to log at startup."""
    _init()
    if _available and _lib is not None:
        try:
            ver = _lib.kobold_rga_version().decode(errors="replace").split("\n")[0]
        except (OSError, UnicodeError, AttributeError):
            ver = "unknown"
        return f"RGA active ({ver.strip()})"
    return f"RGA unavailable, using CPU — {_reason}"


def nv12_to_bgr(nv12: np.ndarray, width: int, height: int) -> np.ndarray:
    """NV12 (H*3/2, W) uint8 -> BGR (H, W, 3) uint8."""
    _init()
    if not _available or _lib is None:
        return cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)

    src = np.ascontiguousarray(nv12)
    dst = np.empty((height, width, 3), dtype=np.uint8)
    rc = _lib.kobold_rga_nv12_to_bgr(src.ctypes.data, dst.ctypes.data, width, height)
    if rc != 0:
        # One failure should not become a permanently broken pipeline.
        return cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
    return dst


def nv12_to_bgr_letterbox(nv12: np.ndarray, sw: int, sh: int,
                          dw: int, dh: int, pad: int = 114) -> np.ndarray:
    """NV12 -> BGR, scaled into dw x dh preserving aspect ratio, padded.

    This is the detector's input path: it replaces a cvtColor plus a resize
    plus a canvas paste with a single accelerated pass.
    """
    _init()
    if _available and _lib is not None:
        dst = np.empty((dh, dw, 3), dtype=np.uint8)
        src = np.ascontiguousarray(nv12)
        rc = _lib.kobold_rga_nv12_to_bgr_letterbox(
            src.ctypes.data, sw, sh, dst.ctypes.data, dw, dh, pad)
        if rc == 0:
            return dst

    # CPU fallback, matching the same geometry.
    bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
    scale = min(dw / sw, dh / sh)
    nw, nh = round(sw * scale) & ~1, round(sh * scale) & ~1
    resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((dh, dw, 3), pad, dtype=np.uint8)
    ox, oy = ((dw - nw) // 2) & ~1, ((dh - nh) // 2) & ~1
    canvas[oy:oy + nh, ox:ox + nw] = resized
    return canvas


def bgr_letterbox(bgr: np.ndarray, dw: int, dh: int, pad: int = 114) -> np.ndarray:
    """BGR -> BGR letterboxed into dw x dh. The detector's input path."""
    _init()
    sh, sw = bgr.shape[:2]
    if _available and _lib is not None:
        src = np.ascontiguousarray(bgr)
        dst = np.empty((dh, dw, 3), dtype=np.uint8)
        rc = _lib.kobold_rga_bgr_letterbox(
            src.ctypes.data, sw, sh, dst.ctypes.data, dw, dh, pad)
        if rc == 0:
            return dst

    scale, ox, oy = letterbox_params(sw, sh, dw, dh)
    nw, nh = round(sw * scale) & ~1, round(sh * scale) & ~1
    canvas = np.full((dh, dw, 3), pad, dtype=np.uint8)
    canvas[oy:oy + nh, ox:ox + nw] = cv2.resize(bgr, (nw, nh),
                                                interpolation=cv2.INTER_LINEAR)
    return canvas


def letterbox_params(sw: int, sh: int, dw: int, dh: int):
    """Scale and offsets used by both paths, so boxes can be mapped back."""
    scale = min(dw / sw, dh / sh)
    nw, nh = round(sw * scale) & ~1, round(sh * scale) & ~1
    return scale, ((dw - nw) // 2) & ~1, ((dh - nh) // 2) & ~1
