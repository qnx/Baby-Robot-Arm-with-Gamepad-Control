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

#ifndef PROTO_H_
#define PROTO_H_

#include <stdbool.h>
#include <stdint.h>

#include <camera/camera_defs.h>

typedef bool (*csc_is_supported_t)(camera_frametype_t ftype);
typedef bool (*csc_init_t)(camera_frametype_t ftype, int width, int height);
typedef void (*csc_deinit_t)(void);
typedef bool (*csc_do_conversion_t)(camera_buffer_t *cbuf, uint8_t *dst, int dstStride);

typedef struct {
    const char *name;

    csc_is_supported_t is_supported;
    csc_init_t init;
    csc_deinit_t deinit;
    csc_do_conversion_t do_conversion;
} csc_api_t;

/**
 * Return the size (number of elements) in a fixed sized array.
 *
 * @param[in] a The name of the array.
 *
 * @return The element capacity of the array.
 */
#define ARRAYSZ(a) (sizeof((a))/sizeof(*(a)))

/* Read the active frame dimensions and stride from a camera buffer. */
void get_camera_frame_properties(const camera_buffer_t *cbuf, int *width, int *height, int *stride);

#endif
