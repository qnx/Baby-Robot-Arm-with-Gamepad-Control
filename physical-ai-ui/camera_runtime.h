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

#ifndef CAMERA_RUNTIME_H_
#define CAMERA_RUNTIME_H_

#include <stdbool.h>

#include <camera/camera_api.h>
#include <sys/neutrino.h>

#include "proto.h"

#define VIDEO_PULSE_FRAME_AVAILABLE (_PULSE_CODE_MINAVAIL + 0)
#define VIDEO_PULSE_STOP (_PULSE_CODE_MINAVAIL + 1)

typedef enum {
    CAMERA_ACCESS_POLICY_AUTO = 0,
    CAMERA_ACCESS_POLICY_FORCE_RO,
} camera_access_policy_t;

typedef struct {
    camera_handle_t handle;
    camera_eventkey_t frame_event_key;
    struct sigevent frame_event;
    camera_access_policy_t access_policy;
    uint32_t access_mode;
    int chid;
    int coid;
    camera_unit_t unit;
    bool camera_unit_requested;
    camera_frametype_t ftype;
    int width;
    int height;
    bool running;
    bool fullscreen;
    const csc_api_t *csc;
} camera_runtime_t;

/* Open the camera stack and discover a usable viewfinder configuration. */
bool camera_runtime_init(camera_runtime_t *runtime);

/* Start delivering viewfinder frames through the registered pulse channel. */
bool camera_runtime_start_viewfinder(camera_runtime_t *runtime);

/* Stop viewfinder streaming if it is currently active. */
void camera_runtime_stop_viewfinder(camera_runtime_t *runtime);

/* Release camera and pulse resources owned by the runtime. */
void camera_runtime_cleanup(camera_runtime_t *runtime);

#endif
