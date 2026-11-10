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

#ifndef OVERLAY_CONTROLS_H_
#define OVERLAY_CONTROLS_H_

#include <gtk/gtk.h>

typedef enum {
    DEBUG_INFO_TOP_RIGHT = 0,
    DEBUG_INFO_BOTTOM_RIGHT,
    DEBUG_INFO_BOTTOM_LEFT,
    DEBUG_INFO_TOP_LEFT
} debug_info_corner_t;

typedef struct {
    void (*video_flip_changed_cb)(gboolean enabled, gpointer user_data);
    gpointer video_flip_changed_user_data;
    gboolean flip_video;
    gboolean show_video_flip;
    gboolean (*robot_control_toggled_cb)(gboolean enabled, gpointer user_data);
    gpointer robot_control_toggled_user_data;
    gboolean (*wrist_control_toggled_cb)(gboolean enabled, gpointer user_data);
    gpointer wrist_control_toggled_user_data;
    GtkWidget *robot_mode_label;
    GtkWidget *skeleton_toggle;
    GtkWidget *joints_toggle;
    GtkWidget *roi_toggle;
    GtkWidget *gestures_toggle;
    GtkWidget *confidence_toggle;
    GtkWidget *robot_toggle;
    GtkWidget *wrist_toggle;
    GtkWidget *confidence_threshold_block;
    GtkWidget *confidence_scale;
    GtkWidget *confidence_range_label;
    gboolean session_card_visible;
    gboolean roi_available;
    gboolean robot_control_available;
    double confidence_threshold;
    gboolean show_skeleton;
    gboolean show_joints;
    gboolean show_roi;
    gboolean show_gestures;
    gboolean filter_low_confidence;
    gboolean enable_robot_control;
    gboolean enable_wrist_control;
    gboolean show_overlay_fills;
    gboolean show_controls;
    gboolean show_debug_info;
    gboolean show_joint_popups;
    debug_info_corner_t debug_info_corner;
    gboolean updating_robot_toggle;
    gboolean updating_wrist_toggle;
} overlay_controls_state_t;

/* Populate the control panel state with its default values. */
void overlay_controls_state_init(overlay_controls_state_t *state);

/* Update the robot mode label based on the latest controller state. */
void overlay_controls_set_robot_mode(overlay_controls_state_t *state,
                                     const char *robot_mode,
                                     const char *screensaver_mode);

/* Create the controls panel that sits beside the camera overlay. */
GtkWidget *overlay_controls_create(GtkWidget *drawing_area,
                                   overlay_controls_state_t *state);

/* Map a percentage threshold into the normalized range used by sliders. */
double overlay_controls_confidence_threshold_to_normalized(double value);

/* Derive the three confidence band cutoffs from the active threshold. */
void overlay_controls_get_confidence_color_thresholds(double value,
                                                      double *low_min_out,
                                                      double *medium_min_out,
                                                      double *high_min_out);

#endif
