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

#ifndef APP_UI_H_
#define APP_UI_H_

#include <gtk/gtk.h>
#include <stdbool.h>

#include "overlay_controls.h"
#include "overlay_renderer.h"
#include "pose_data.h"

typedef struct {
    bool initialized;
    roi_info_t input_roi;
    roi_info_t adjustment_roi;
    int frame_width;
    int frame_height;
    camera_display_rect_t display_rect;
    double pointer_x;
    double pointer_y;
    double drag_start_x;
    double drag_start_y;
    roi_edge_t active_edge;
    roi_edge_t dragged_edge;
    roi_info_t drag_start_adjustment_roi;
} roi_editor_state_t;

typedef struct {
    GtkWidget *window;
    GtkWidget *picture;
    GtkWidget *camera_error;
    GtkWidget *overlay;
    GtkWidget *topbar;
    GtkWidget *topbar_logo;
    GtkWidget *topbar_qr;
    GtkWidget *camera_card;
    GtkWidget *controls_box;
    GtkWidget *content_box;
    GtkWidget *camera_brand;
    GtkWidget *camera_header;
    GtkWidget *camera_subtitle;
    GtkWidget *camera_subtitle_secondary;
    GtkWidget *camera_title;
    GtkWidget *roi_hint_chip;
    GtkWidget *hotkeys_popup;
    GtkWidget *qr_popup;
    GtkWidget *robot_mode_popup;
    GtkWidget *robot_mode_stack;
    pose_data_t pose_data;
    overlay_controls_state_t controls;
    roi_editor_state_t roi_editor;
    gint64 app_start_us;
    int controls_width_px;
    char current_robot_mode[MAX_ROBOT_MODE_LEN];
    char current_screensaver_mode[MAX_SCREENSAVER_MODE_LEN];
    bool *fullscreen;
} app_ui_t;

/* Build the GTK window tree and wire it to the runtime services. */
bool app_ui_init(app_ui_t *ui, GtkApplication *app,
                 bool show_video_flip,
                 bool *fullscreen, gint64 app_start_us);

/* Present the main window once initialization succeeds. */
void app_ui_present(app_ui_t *ui);

/* Stop background pose data work before application shutdown. */
void app_ui_shutdown(app_ui_t *ui);

/* Return the picture widget that displays camera frames. */
GtkWidget *app_ui_get_picture(const app_ui_t *ui);

/* Return the overlay widget used for skeleton and ROI drawing. */
GtkWidget *app_ui_get_overlay(const app_ui_t *ui);

#endif
