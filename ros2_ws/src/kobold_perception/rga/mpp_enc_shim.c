// Hardware video encode on the RK3588 VPU, via MPP directly.
//
// WHY NOT THE GSTREAMER PLUGIN
// ----------------------------
// gstreamer1.0-rockchip1 1.14-4 encodes frame 0 in 47 ms and every frame after
// in ~4,660 ms, stalled in gst_mpp_enc_poll_packet_locked waiting for a packet
// MPP has already produced. Measured 0.9 fps against MPP's own 211-250 fps for
// the identical work. Camera, buffer path, RGA conversion and resolution were
// each ruled out, and it reproduces on a fresh boot with zero softirq load.
// Its upstream (rockchip-linux/gstreamer-rockchip) is a 404 and Radxa ships no
// newer build, so there is nothing to upgrade to.
//
// MPP itself is fine. This talks to it directly.
//
// MEASURED — 1280x960, 60 DISTINCT real camera frames (not one frame repeated,
// which makes H.264 P-frames unrealistically empty):
//
//     MJPEG q80    5.29 ms wall  2.51 ms cpu   57.1 kB/frame   5.61 Mbit/s @12fps
//     H.264 2Mbit  4.09 ms wall  1.26 ms cpu    7.0 kB/frame   0.69 Mbit/s @12fps
//
// For comparison at the same 12 fps camera rate:
//     software jpegenc   ~6.8% of a core, 5.65 Mbit/s
//     software x264enc   70.4% of a core, 1.36 Mbit/s
//     MJPEG here          3.0% of a core
//     H.264 here          1.5% of a core, and half the bitrate of software x264
//
// So H.264 on the VPU is ~47x cheaper in CPU than x264enc AND smaller on the
// wire. It is the right choice the moment bandwidth matters; MJPEG stays the
// default only because an <img> tag can display it with no client machinery.
//
// WHY AN OPAQUE HANDLE
// --------------------
// Same reasoning as rga_shim.c: everything MPP-shaped stays on this side of the
// boundary. Python gets a void* and three functions, so there is no struct
// layout to replicate and no silent corruption when a header changes.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rockchip/rk_mpi.h"
#include "rockchip/mpp_buffer.h"
#include "rockchip/mpp_frame.h"
#include "rockchip/mpp_packet.h"

// MPP_ALIGN lives in mpp_common.h, which is not part of the installed public
// headers. Without this it resolves as an implicit int-returning function —
// which compiles, links, and silently truncates the stride.
#ifndef MPP_ALIGN
#define MPP_ALIGN(x, a) (((x) + (a) - 1) & ~((a) - 1))
#endif

typedef struct {
    MppCtx          ctx;
    MppApi         *mpi;
    MppBufferGroup  frm_grp;
    MppBuffer       frm_buf;
    int             width, height;
    int             hor_stride, ver_stride;
    size_t          frame_size;
} KoboldEnc;

// coding: 0 = MJPEG, 1 = H.264
// quality: MJPEG q-factor 1..99, or H.264 bitrate in bits/sec
void *kobold_mpp_enc_create(int width, int height, int coding, int quality) {
    KoboldEnc *e = calloc(1, sizeof(KoboldEnc));
    if (!e) return NULL;

    e->width  = width;
    e->height = height;
    // MPP wants 16-aligned strides. Getting this wrong produces a sheared
    // image rather than an error, which is a miserable thing to debug.
    e->hor_stride = MPP_ALIGN(width, 16);
    e->ver_stride = MPP_ALIGN(height, 16);
    e->frame_size = (size_t)e->hor_stride * e->ver_stride * 3 / 2;   // NV12

    MppCodingType type = coding ? MPP_VIDEO_CodingAVC : MPP_VIDEO_CodingMJPEG;

    if (mpp_create(&e->ctx, &e->mpi) != MPP_OK) goto fail;
    if (mpp_init(e->ctx, MPP_CTX_ENC, type) != MPP_OK) goto fail;

    MppEncCfg cfg = NULL;
    if (mpp_enc_cfg_init(&cfg) != MPP_OK) goto fail;
    if (e->mpi->control(e->ctx, MPP_ENC_GET_CFG, cfg) != MPP_OK) goto fail_cfg;

    mpp_enc_cfg_set_s32(cfg, "prep:width",      width);
    mpp_enc_cfg_set_s32(cfg, "prep:height",     height);
    mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", e->hor_stride);
    mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", e->ver_stride);
    mpp_enc_cfg_set_s32(cfg, "prep:format",     MPP_FMT_YUV420SP);   // NV12

    if (coding) {
        // H.264: constant bitrate, all-intra disabled, sane defaults.
        mpp_enc_cfg_set_s32(cfg, "rc:mode",     MPP_ENC_RC_MODE_CBR);
        mpp_enc_cfg_set_s32(cfg, "rc:bps_target", quality);
        mpp_enc_cfg_set_s32(cfg, "rc:bps_max",  quality * 17 / 16);
        mpp_enc_cfg_set_s32(cfg, "rc:bps_min",  quality * 15 / 16);
        mpp_enc_cfg_set_s32(cfg, "rc:fps_in_num",  30);
        mpp_enc_cfg_set_s32(cfg, "rc:fps_out_num", 30);
        mpp_enc_cfg_set_s32(cfg, "rc:gop",      60);
        mpp_enc_cfg_set_s32(cfg, "h264:profile", 100);   // High
        mpp_enc_cfg_set_s32(cfg, "h264:level",   40);
        mpp_enc_cfg_set_s32(cfg, "h264:cabac_en", 1);
    } else {
        // MJPEG: fixed quality, every frame independent.
        mpp_enc_cfg_set_s32(cfg, "rc:mode",     MPP_ENC_RC_MODE_FIXQP);
        mpp_enc_cfg_set_s32(cfg, "jpeg:q_factor",     quality);
        mpp_enc_cfg_set_s32(cfg, "jpeg:qf_min",       1);
        mpp_enc_cfg_set_s32(cfg, "jpeg:qf_max",       99);
    }

    if (e->mpi->control(e->ctx, MPP_ENC_SET_CFG, cfg) != MPP_OK) goto fail_cfg;
    mpp_enc_cfg_deinit(cfg);

    if (mpp_buffer_group_get_internal(&e->frm_grp, MPP_BUFFER_TYPE_DRM) != MPP_OK) goto fail;
    if (mpp_buffer_get(e->frm_grp, &e->frm_buf, e->frame_size) != MPP_OK) goto fail;

    return e;

fail_cfg:
    mpp_enc_cfg_deinit(cfg);
fail:
    if (e->frm_buf) mpp_buffer_put(e->frm_buf);
    if (e->frm_grp) mpp_buffer_group_put(e->frm_grp);
    if (e->ctx)     mpp_destroy(e->ctx);
    free(e);
    return NULL;
}

// Encode one NV12 frame. Returns bytes written to `out`, or negative on error.
int kobold_mpp_enc_encode(void *handle, const void *nv12, void *out, int out_cap) {
    KoboldEnc *e = (KoboldEnc *)handle;
    if (!e || !nv12 || !out) return -1;

    // Copy into the DRM buffer, honouring stride. When width is already
    // 16-aligned this is one memcpy; otherwise it is row by row.
    char *dst = (char *)mpp_buffer_get_ptr(e->frm_buf);
    const char *src = (const char *)nv12;
    if (e->hor_stride == e->width) {
        memcpy(dst, src, (size_t)e->width * e->height * 3 / 2);
    } else {
        for (int y = 0; y < e->height; y++)                       // Y plane
            memcpy(dst + (size_t)y * e->hor_stride, src + (size_t)y * e->width, e->width);
        char *dst_uv = dst + (size_t)e->hor_stride * e->ver_stride;
        const char *src_uv = src + (size_t)e->width * e->height;
        for (int y = 0; y < e->height / 2; y++)                   // UV plane
            memcpy(dst_uv + (size_t)y * e->hor_stride, src_uv + (size_t)y * e->width, e->width);
    }

    MppFrame frame = NULL;
    if (mpp_frame_init(&frame) != MPP_OK) return -2;
    mpp_frame_set_width(frame,      e->width);
    mpp_frame_set_height(frame,     e->height);
    mpp_frame_set_hor_stride(frame, e->hor_stride);
    mpp_frame_set_ver_stride(frame, e->ver_stride);
    mpp_frame_set_fmt(frame,        MPP_FMT_YUV420SP);
    mpp_frame_set_buffer(frame,     e->frm_buf);
    mpp_frame_set_eos(frame,        0);

    MPP_RET ret = e->mpi->encode_put_frame(e->ctx, frame);
    mpp_frame_deinit(&frame);
    if (ret != MPP_OK) return -3;

    MppPacket packet = NULL;
    ret = e->mpi->encode_get_packet(e->ctx, &packet);
    if (ret != MPP_OK || !packet) return -4;

    void *pdata = mpp_packet_get_pos(packet);
    size_t plen = mpp_packet_get_length(packet);
    int written = -5;
    if ((int)plen <= out_cap) {
        memcpy(out, pdata, plen);
        written = (int)plen;
    }
    mpp_packet_deinit(&packet);
    return written;
}

void kobold_mpp_enc_destroy(void *handle) {
    KoboldEnc *e = (KoboldEnc *)handle;
    if (!e) return;
    if (e->mpi && e->ctx) e->mpi->reset(e->ctx);
    if (e->frm_buf) mpp_buffer_put(e->frm_buf);
    if (e->frm_grp) mpp_buffer_group_put(e->frm_grp);
    if (e->ctx)     mpp_destroy(e->ctx);
    free(e);
}
