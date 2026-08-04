// Minimal C shim over librga's im2d API, for calling from Python via ctypes.
//
// WHY A SHIM AND NOT DIRECT ctypes: im2d's entry points take and return
// rga_buffer_t BY VALUE — a large struct whose layout would have to be
// replicated exactly in Python and kept in sync with the header. Getting one
// field wrong produces silent corruption rather than an error. Two plain
// functions taking pointers and ints have no such failure mode.
//
// Measured on RK3588, 1280x960 NV12 (see tools/rga_bench.c):
//
//     RGA  NV12->BGR        2.82 ms wall   1.09 ms cpu
//     CPU  NV12->BGR        7.25 ms wall   7.25 ms cpu
//     RGA  NV12->BGR+640    1.77 ms wall   0.59 ms cpu
//
// Against GStreamer's videoconvert as actually measured in the pipeline (32%
// of a core at 12.3 fps, ~26 ms cpu/frame), the RGA path is ~24x cheaper in
// CPU. The wall-clock difference is modest; the CPU difference is the point.
#include <string.h>
#include "rga/im2d.h"
#include "rga/rga.h"

// NV12 (YCbCr 420 semi-planar) -> BGR888, same dimensions.
// Returns 0 on success, negative IM_STATUS on failure.
int kobold_rga_nv12_to_bgr(void *src, void *dst, int width, int height) {
    rga_buffer_t s = wrapbuffer_virtualaddr(src, width, height, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_t d = wrapbuffer_virtualaddr(dst, width, height, RK_FORMAT_BGR_888);
    im_rect nr = {0, 0, 0, 0};
    IM_STATUS chk = imcheck(s, d, nr, nr, 0);
    if (chk != IM_STATUS_NOERROR) return -(int)chk;
    IM_STATUS ret = imcvtcolor(s, d, s.format, d.format);
    return ret == IM_STATUS_SUCCESS ? 0 : -(int)ret;
}

// NV12 -> BGR888 AND scale, in a single RGA pass. This is the one that matters:
// it replaces both the camera node's colour conversion and the detector's
// letterbox resize.
//
// NOTE: this is a plain scale, NOT a letterbox — it does not preserve aspect
// ratio. The caller must either pass a square-cropped source or accept the
// distortion. kobold's detector letterboxes deliberately (see detector_node),
// so use kobold_rga_nv12_to_bgr_letterbox instead for that path.
int kobold_rga_nv12_to_bgr_scaled(void *src, int sw, int sh,
                                  void *dst, int dw, int dh) {
    rga_buffer_t s = wrapbuffer_virtualaddr(src, sw, sh, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_t d = wrapbuffer_virtualaddr(dst, dw, dh, RK_FORMAT_BGR_888);
    im_rect nr = {0, 0, 0, 0};
    IM_STATUS chk = imcheck(s, d, nr, nr, 0);
    if (chk != IM_STATUS_NOERROR) return -(int)chk;
    IM_STATUS ret = imresize(s, d, 0, 0, INTER_LINEAR);
    return ret == IM_STATUS_SUCCESS ? 0 : -(int)ret;
}

// NV12 -> BGR888 letterboxed into dw x dh, preserving aspect ratio.
// The destination is filled with `pad` first, then the scaled image is blitted
// into a centred rect. Two RGA operations, still far cheaper than one CPU pass.
int kobold_rga_nv12_to_bgr_letterbox(void *src, int sw, int sh,
                                     void *dst, int dw, int dh,
                                     unsigned char pad) {
    // Fill: cheaper on the CPU than an RGA colour-fill setup for a small buffer,
    // and it happens once per frame regardless.
    memset(dst, pad, (size_t)dw * dh * 3);

    double scale = (double)dw / sw < (double)dh / sh
                 ? (double)dw / sw : (double)dh / sh;
    int nw = (int)(sw * scale) & ~1;      // RGA prefers even dimensions
    int nh = (int)(sh * scale) & ~1;
    int ox = ((dw - nw) / 2) & ~1;
    int oy = ((dh - nh) / 2) & ~1;

    rga_buffer_t s = wrapbuffer_virtualaddr(src, sw, sh, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_t d = wrapbuffer_virtualaddr(dst, dw, dh, RK_FORMAT_BGR_888);

    im_rect srect = {0, 0, sw, sh};
    im_rect drect = {ox, oy, nw, nh};

    IM_STATUS chk = imcheck(s, d, srect, drect, 0);
    if (chk != IM_STATUS_NOERROR) return -(int)chk;
    IM_STATUS ret = improcess(s, d, d, srect, drect, drect, IM_SYNC);
    return ret == IM_STATUS_SUCCESS ? 0 : -(int)ret;
}

// BGR888 -> BGR888 letterbox. The detector receives an already-converted BGR
// frame from the camera node, so it needs this rather than the NV12 variant.
// Same geometry as kobold_rga_nv12_to_bgr_letterbox so boxes map back
// identically whichever path ran.
int kobold_rga_bgr_letterbox(void *src, int sw, int sh,
                             void *dst, int dw, int dh,
                             unsigned char pad) {
    memset(dst, pad, (size_t)dw * dh * 3);

    double scale = (double)dw / sw < (double)dh / sh
                 ? (double)dw / sw : (double)dh / sh;
    int nw = (int)(sw * scale) & ~1;
    int nh = (int)(sh * scale) & ~1;
    int ox = ((dw - nw) / 2) & ~1;
    int oy = ((dh - nh) / 2) & ~1;

    rga_buffer_t s = wrapbuffer_virtualaddr(src, sw, sh, RK_FORMAT_BGR_888);
    rga_buffer_t d = wrapbuffer_virtualaddr(dst, dw, dh, RK_FORMAT_BGR_888);

    im_rect srect = {0, 0, sw, sh};
    im_rect drect = {ox, oy, nw, nh};

    IM_STATUS chk = imcheck(s, d, srect, drect, 0);
    if (chk != IM_STATUS_NOERROR) return -(int)chk;
    IM_STATUS ret = improcess(s, d, d, srect, drect, drect, IM_SYNC);
    return ret == IM_STATUS_SUCCESS ? 0 : -(int)ret;
}

// Returns the librga version string, so callers can log what they bound to.
const char *kobold_rga_version(void) {
    return querystring(RGA_VERSION);
}
