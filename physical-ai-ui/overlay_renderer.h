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

#ifndef OVERLAY_RENDERER_H_
#define OVERLAY_RENDERER_H_

#include <gtk/gtk.h>

#include "overlay_controls.h"
#include "pose_data.h"

typedef enum {
    ROI_EDGE_NONE = 0,
    ROI_EDGE_LEFT,
    ROI_EDGE_TOP,
    ROI_EDGE_RIGHT,
    ROI_EDGE_BOTTOM
} roi_edge_t;

typedef struct {
    int x;
    int y;
    int width;
    int height;
} camera_display_rect_t;

/* Fit the camera frame into the destination area while preserving aspect ratio. */
camera_display_rect_t overlay_renderer_get_camera_display_rect(
    int src_width,
    int src_height,
    int dst_width,
    int dst_height);

/* Draw pose, ROI, and debug overlays on top of the current camera frame. */
void overlay_renderer_draw(cairo_t *cr,
                           int width,
                           int height,
                           const resource_info_t *info,
                           const overlay_controls_state_t *controls,
                           const roi_info_t *adjustment_roi,
                           roi_edge_t active_roi_edge,
                           roi_edge_t dragged_roi_edge,
                           const char *app_version,
                           gint64 app_start_us);

#endif
