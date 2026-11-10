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

#include <libyuv.h>

#include "../proto.h"

/* Report whether libyuv has a direct conversion path for this format. */
static bool csc_libyuv_is_supported(camera_frametype_t ftype) {
    switch (ftype) {
        case CAMERA_FRAMETYPE_NV12:
        case CAMERA_FRAMETYPE_RGB8888:
        case CAMERA_FRAMETYPE_RGB888:
        case CAMERA_FRAMETYPE_GRAY8:
        case CAMERA_FRAMETYPE_CBYCRY:
        case CAMERA_FRAMETYPE_RGB565:
        case CAMERA_FRAMETYPE_YCBCR420P:
        case CAMERA_FRAMETYPE_YCBYCR:
        case CAMERA_FRAMETYPE_BGR8888:
        case CAMERA_FRAMETYPE_NV21:
            return true;
        default:
            return false;
    }
}

/* Keep the libyuv backend aligned with the shared CSC interface. */
static bool csc_libyuv_init(camera_frametype_t ftype, int width, int height) {
    return true;
}

/* No backend state is allocated for the libyuv conversion path. */
static void csc_libyuv_deinit(void) {
}

/* Convert camera buffers into ARGB data that GTK can upload as textures. */
static bool csc_libyuv_do_conversion(camera_buffer_t *cbuf, uint8_t *dst, int dstStride) {
    int width;
    int height;
    int stride;
    int rc;

    get_camera_frame_properties(cbuf, &width, &height, &stride);

    switch (cbuf->frametype) {
        case CAMERA_FRAMETYPE_NV12:
            rc = NV12ToARGB(
                    cbuf->framebuf, stride, 
                    cbuf->framebuf + cbuf->framedesc.nv12.uv_offset, cbuf->framedesc.nv12.uv_stride,
                    dst, dstStride,
                    width, height);
            break;
        case CAMERA_FRAMETYPE_RGB8888:
            rc = ARGBToARGB(
                    cbuf->framebuf, stride,
                    dst, dstStride,
                    width, height);
            break;
        case CAMERA_FRAMETYPE_RGB888:
            rc = RGB24ToARGB(
                    cbuf->framebuf, stride,
                    dst, dstStride,
                    width, height);
            break;
        case CAMERA_FRAMETYPE_GRAY8:
            rc = I400ToARGB(
                    cbuf->framebuf, stride,
                    dst, dstStride,
                    width, height);
            break;
        case CAMERA_FRAMETYPE_CBYCRY:
            rc = UYVYToARGB(
                    cbuf->framebuf, stride,
                    dst, dstStride,
                    width, height);
            break;
        case CAMERA_FRAMETYPE_RGB565:
            rc = RGB565ToARGB(
                    cbuf->framebuf, stride,
                    dst, dstStride,
                    width, height);
            break;
        case CAMERA_FRAMETYPE_YCBCR420P:
            rc = I420ToARGB(
                    cbuf->framebuf, stride, 
                    cbuf->framebuf + cbuf->framedesc.ycbcr420p.cb_offset, cbuf->framedesc.ycbcr420p.cb_stride,
                    cbuf->framebuf + cbuf->framedesc.ycbcr420p.cr_offset, cbuf->framedesc.ycbcr420p.cr_stride,
                    dst, dstStride,
                    width, height);
            break;
        case CAMERA_FRAMETYPE_YCBYCR:
            rc = YUY2ToARGB(
                    cbuf->framebuf, stride,
                    dst, dstStride,
                    width, height);
            break;
        case CAMERA_FRAMETYPE_BGR8888:
            rc = BGRAToARGB(
                    cbuf->framebuf, stride,
                    dst, dstStride,
                    width, height);
            break;
        case CAMERA_FRAMETYPE_NV21:
            rc = NV21ToARGB(
                    cbuf->framebuf, stride, 
                    cbuf->framebuf + cbuf->framedesc.nv21.vu_offset, cbuf->framedesc.nv21.vu_stride,
                    dst, dstStride,
                    width, height);
            break;
        default:
            rc = 1;
    }

    return rc == 0;
}

const csc_api_t CscLibYUV = {
    .name = "libyuv",
    .is_supported = csc_libyuv_is_supported,
    .init = csc_libyuv_init,
    .deinit = csc_libyuv_deinit,
    .do_conversion = csc_libyuv_do_conversion
};
