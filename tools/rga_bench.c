// RGA vs CPU for the two operations kobold's perception pipeline does every
// frame: NV12 -> BGR colour conversion, and letterbox-scale to the detector's
// 640x640 input.
//
// Reports wall time AND cpu time separately, because that is the whole point:
// RGA should cost wall-clock latency but almost no CPU, freeing the core for
// something else. A benchmark that only reports wall time would miss the win.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/resource.h>
#include "rga/im2d.h"
#include "rga/rga.h"

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static double cpu_ms(void) {
    struct rusage r;
    getrusage(RUSAGE_SELF, &r);
    return (r.ru_utime.tv_sec + r.ru_stime.tv_sec) * 1000.0
         + (r.ru_utime.tv_usec + r.ru_stime.tv_usec) / 1000.0;
}

// Straightforward scalar NV12 -> BGR888. Not SIMD, so it is a pessimistic
// stand-in for GStreamer's videoconvert; the measured videoconvert figure is
// the number to trust for the CPU side.
static void cpu_nv12_to_bgr(const unsigned char *y, const unsigned char *uv,
                            unsigned char *bgr, int w, int h) {
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            int Y = y[j * w + i];
            int U = uv[(j / 2) * w + (i & ~1)] - 128;
            int V = uv[(j / 2) * w + (i & ~1) + 1] - 128;
            int r = Y + ((91881 * V) >> 16);
            int g = Y - ((22554 * U + 46802 * V) >> 16);
            int b = Y + ((116130 * U) >> 16);
            unsigned char *p = bgr + (j * w + i) * 3;
            p[0] = b < 0 ? 0 : (b > 255 ? 255 : b);
            p[1] = g < 0 ? 0 : (g > 255 ? 255 : g);
            p[2] = r < 0 ? 0 : (r > 255 ? 255 : r);
        }
    }
}

int main(void) {
    const int W = 1280, H = 960, N = 100;
    const int DW = 640, DH = 640;

    unsigned char *nv12 = malloc((size_t)W * H * 3 / 2);
    unsigned char *bgr  = malloc((size_t)W * H * 3);
    unsigned char *small = malloc((size_t)DW * DH * 3);
    if (!nv12 || !bgr || !small) { fprintf(stderr, "alloc failed\n"); return 1; }
    for (size_t i = 0; i < (size_t)W * H * 3 / 2; i++) nv12[i] = (unsigned char)(i * 7);

    printf("librga: %s\n", querystring(RGA_VERSION));
    printf("frame : %dx%d NV12, %d iterations\n\n", W, H, N);

    // ---------------------------------------------------- RGA: NV12 -> BGR --
    {
        rga_buffer_t s = wrapbuffer_virtualaddr(nv12, W, H, RK_FORMAT_YCbCr_420_SP);
        rga_buffer_t d = wrapbuffer_virtualaddr(bgr,  W, H, RK_FORMAT_BGR_888);
        im_rect nr = {0, 0, 0, 0};
        IM_STATUS chk = imcheck(s, d, nr, nr, 0);
        if (chk != IM_STATUS_NOERROR) { printf("RGA cvtcolor unsupported: %s\n", imStrError(chk)); }
        else {
            imcvtcolor(s, d, s.format, d.format);          // warm up
            double w0 = now_ms(), c0 = cpu_ms();
            for (int i = 0; i < N; i++) imcvtcolor(s, d, s.format, d.format);
            double wall = now_ms() - w0, cpu = cpu_ms() - c0;
            printf("RGA  NV12->BGR       %6.2f ms wall  %6.2f ms cpu   (%.1f%% cpu)\n",
                   wall / N, cpu / N, 100.0 * cpu / wall);
        }
    }

    // ---------------------------------------------------- CPU: NV12 -> BGR --
    {
        cpu_nv12_to_bgr(nv12, nv12 + (size_t)W * H, bgr, W, H);
        double w0 = now_ms(), c0 = cpu_ms();
        for (int i = 0; i < N; i++) cpu_nv12_to_bgr(nv12, nv12 + (size_t)W * H, bgr, W, H);
        double wall = now_ms() - w0, cpu = cpu_ms() - c0;
        printf("CPU  NV12->BGR       %6.2f ms wall  %6.2f ms cpu   (%.1f%% cpu)\n",
               wall / N, cpu / N, 100.0 * cpu / wall);
    }

    // ------------------------------------------ RGA: convert + scale to 640 --
    {
        rga_buffer_t s = wrapbuffer_virtualaddr(nv12,  W,  H,  RK_FORMAT_YCbCr_420_SP);
        rga_buffer_t d = wrapbuffer_virtualaddr(small, DW, DH, RK_FORMAT_BGR_888);
        im_rect nr = {0, 0, 0, 0};
        IM_STATUS chk = imcheck(s, d, nr, nr, 0);
        if (chk != IM_STATUS_NOERROR) { printf("RGA resize unsupported: %s\n", imStrError(chk)); }
        else {
            imresize(s, d, 0, 0, INTER_LINEAR);            // warm up
            double w0 = now_ms(), c0 = cpu_ms();
            for (int i = 0; i < N; i++) imresize(s, d, 0, 0, INTER_LINEAR);
            double wall = now_ms() - w0, cpu = cpu_ms() - c0;
            printf("RGA  NV12->BGR+640   %6.2f ms wall  %6.2f ms cpu   (%.1f%% cpu)\n",
                   wall / N, cpu / N, 100.0 * cpu / wall);
        }
    }

    free(nv12); free(bgr); free(small);
    return 0;
}
