/*
 * Copyright (c) 2026, BlackBerry Limited. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdbool.h>
#include <stdint.h>

#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>

#include "../proto.h"

/**
 * Mapping from camera_frametype_t values to the associated ffmpeg AVPixelFormat.
 *
 * If the mapping results in AV_PIX_FMT_NONE then the input camera_frametype_t
 * value is NOT supported by this application.
 *
 * Mostly used to configure a CSC (Colour Space Conversion) in ffmpeg from the
 * input camera frametype to the B8G8R8A8 format supported by GTK's textures.
 */
static const enum AVPixelFormat Frametype2AVPixelFormat[CAMERA_FRAMETYPE_NUMFRAMETYPES] = {
    [CAMERA_FRAMETYPE_UNSPECIFIED] = AV_PIX_FMT_NONE,
    [CAMERA_FRAMETYPE_NV12] = AV_PIX_FMT_NV12,
    [CAMERA_FRAMETYPE_RGB8888] = AV_PIX_FMT_BGRA,
    [CAMERA_FRAMETYPE_RGB888] = AV_PIX_FMT_BGR8,
    [CAMERA_FRAMETYPE_GRAY8] = AV_PIX_FMT_GRAY8,
    [CAMERA_FRAMETYPE_METADATA] = AV_PIX_FMT_NONE,
    [CAMERA_FRAMETYPE_BAYER] = AV_PIX_FMT_NONE,
    [CAMERA_FRAMETYPE_CBYCRY] = AV_PIX_FMT_UYVY422,
    [CAMERA_FRAMETYPE_COMPRESSEDVIDEO] = AV_PIX_FMT_NONE,
    [CAMERA_FRAMETYPE_COMPRESSEDAUDIO] = AV_PIX_FMT_NONE,
    [CAMERA_FRAMETYPE_RGB565] = AV_PIX_FMT_RGB565,
    [CAMERA_FRAMETYPE_YCBCR420P] = AV_PIX_FMT_YUV420P,
    [CAMERA_FRAMETYPE_YCBYCR] = AV_PIX_FMT_YUYV422,
    [CAMERA_FRAMETYPE_YCRYCB] = AV_PIX_FMT_YVYU422,
    [CAMERA_FRAMETYPE_CRYCBY] = AV_PIX_FMT_NONE,
    [CAMERA_FRAMETYPE_ROI] = AV_PIX_FMT_NONE,
    [CAMERA_FRAMETYPE_BAYER14_RGGB_PADLO16] = AV_PIX_FMT_NONE,
    [CAMERA_FRAMETYPE_NV16] = AV_PIX_FMT_NV16,
    [CAMERA_FRAMETYPE_NV24_Y12] = AV_PIX_FMT_NV24,
    [CAMERA_FRAMETYPE_BGR8888] = AV_PIX_FMT_ARGB,
    [CAMERA_FRAMETYPE_GRAY10P] = AV_PIX_FMT_GRAY10,
    [CAMERA_FRAMETYPE_GRAY12P] = AV_PIX_FMT_GRAY12,
    [CAMERA_FRAMETYPE_RGB_IR_8BIT] = AV_PIX_FMT_NONE,
    [CAMERA_FRAMETYPE_RGB_IR_10BIT] = AV_PIX_FMT_NONE,
    [CAMERA_FRAMETYPE_NV21] = AV_PIX_FMT_NV21,
    [CAMERA_FRAMETYPE_NV61] = AV_PIX_FMT_NONE,
    [CAMERA_FRAMETYPE_YCRCB420P] = AV_PIX_FMT_YUV420P,
    [CAMERA_FRAMETYPE_CBYCRY_10BIT] = AV_PIX_FMT_NONE,
    [CAMERA_FRAMETYPE_BAYER8_RGGB] = AV_PIX_FMT_BAYER_RGGB8,
    [CAMERA_FRAMETYPE_BAYER12_GRBG] = AV_PIX_FMT_NONE,
    [CAMERA_FRAMETYPE_BAYER10_BGGR_PACKED] = AV_PIX_FMT_NONE,
    [CAMERA_FRAMETYPE_GRAY8I] = AV_PIX_FMT_NONE,
    [CAMERA_FRAMETYPE_GRAY12I] = AV_PIX_FMT_NONE,
    [CAMERA_FRAMETYPE_Z16] = AV_PIX_FMT_NONE
};

static SwsContext *CscCtx = NULL;

/* Report whether ffmpeg knows how to convert this camera frame type. */
static bool csc_ffmpeg_is_supported(camera_frametype_t ftype) {
    if (ftype <= ARRAYSZ(Frametype2AVPixelFormat)) {
        return Frametype2AVPixelFormat[ftype] != AV_PIX_FMT_NONE;
    }

    return false;
}

/* Allocate the ffmpeg scaling context used for frame conversion. */
static bool csc_ffmpeg_init(camera_frametype_t ftype, int width, int height) {
    CscCtx = sws_getContext(width, height, Frametype2AVPixelFormat[ftype],
            width, height, AV_PIX_FMT_BGRA,
            0, NULL, NULL, NULL);
    if (CscCtx == NULL) {
        printf("Unable to create CSC context.\n");
        return false;
    }

    return true;
}

/* Tear down the cached ffmpeg conversion context. */
static void csc_ffmpeg_deinit(void) {
    if (CscCtx != NULL) sws_freeContext(CscCtx);
}

/* Convert the incoming frame, setting up planar sources when needed. */
static bool csc_ffmpeg_do_conversion(camera_buffer_t *cbuf, uint8_t *dst, int dstStride) {
    /* At most there are three input planes and one output plane. */
    const uint8_t *srcs[3];
    int srcStrides[3];
    uint8_t *dsts[1];
    int dstStrides[1];
    int outHeight;
    int height;
    int stride;

    get_camera_frame_properties(cbuf, NULL, &height, &stride);

    srcs[0] = cbuf->framebuf;
    srcStrides[0] = stride;
    dsts[0] = dst;
    dstStrides[0] = dstStride;

    /* Planar formats need their extra source planes wired up explicitly. */
    switch (cbuf->frametype) {
        case CAMERA_FRAMETYPE_NV12:
            srcs[1] = cbuf->framebuf + cbuf->framedesc.nv12.uv_offset;
            srcStrides[1] = cbuf->framedesc.nv12.uv_stride;
            break;
        case CAMERA_FRAMETYPE_YCBCR420P:
            srcs[1] = cbuf->framebuf + cbuf->framedesc.ycbcr420p.cb_offset;
            srcStrides[1] = cbuf->framedesc.ycbcr420p.cb_stride;
            srcs[2] = cbuf->framebuf + cbuf->framedesc.ycbcr420p.cr_offset;
            srcStrides[2] = cbuf->framedesc.ycbcr420p.cr_stride;
            break;
        case CAMERA_FRAMETYPE_NV16:
            srcs[1] = cbuf->framebuf + cbuf->framedesc.nv16.uv_offset;
            srcStrides[1] = cbuf->framedesc.nv16.uv_stride;
            break;
        case CAMERA_FRAMETYPE_NV24_Y12:
            srcs[1] = cbuf->framebuf + cbuf->framedesc.nv24.uv_offset;
            srcStrides[1] = cbuf->framedesc.nv24.uv_stride;
            break;
        case CAMERA_FRAMETYPE_NV21:
            srcs[1] = cbuf->framebuf + cbuf->framedesc.nv21.vu_offset;
            srcStrides[1] = cbuf->framedesc.nv21.vu_stride;
            break;
        case CAMERA_FRAMETYPE_YCRCB420P:
            srcs[1] = cbuf->framebuf + cbuf->framedesc.ycbcr420p.cr_offset;
            srcStrides[1] = cbuf->framedesc.ycbcr420p.cr_stride;
            srcs[2] = cbuf->framebuf + cbuf->framedesc.ycbcr420p.cb_offset;
            srcStrides[2] = cbuf->framedesc.ycbcr420p.cb_stride;
            break;
        default:
            /* Packed formats only use the primary plane. */
            break;
    }

    outHeight = sws_scale(CscCtx, srcs, srcStrides, 0, height, dsts, dstStrides);
    if (outHeight != height) {
        printf("Something went wrong during CSC!");
        return false;
    }

    return true;
}

const csc_api_t CscFFMpeg = {
    .name = "ffmpeg",
    .is_supported = csc_ffmpeg_is_supported,
    .init = csc_ffmpeg_init,
    .deinit = csc_ffmpeg_deinit,
    .do_conversion = csc_ffmpeg_do_conversion
};
