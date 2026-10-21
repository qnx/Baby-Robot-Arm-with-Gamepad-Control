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

#include "overlay_controls.h"

#include "app_config.h"

#include <math.h>

/* Convert a section heading string into the shared card title styling. */
static GtkWidget *create_section_title(const char *text)
{
    GtkWidget *label = gtk_label_new(text);

    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_widget_add_css_class(label, "panel-section-title");
    return label;
}

/* Create a compact label for a keyboard shortcut. */
static GtkWidget *create_status_hotkey_chip(const char *text)
{
    GtkWidget *label = gtk_label_new(text);

    gtk_widget_set_halign(label, GTK_ALIGN_END);
    gtk_widget_set_valign(label, GTK_ALIGN_CENTER);
    gtk_widget_add_css_class(label, "status-hotkey-chip");
    return label;
}

/* Create a styled container for a controls card. */
static GtkWidget *create_card_box(void)
{
    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 12);

    gtk_widget_add_css_class(box, "panel-card");
    return box;
}

/* Normalize a confidence threshold for color interpolation. */
double overlay_controls_confidence_threshold_to_normalized(double value)
{
    return value / 100.0;
}

/* Split the active threshold into low, medium, and high confidence bands. */
void overlay_controls_get_confidence_color_thresholds(double value,
                                                      double *low_min_out,
                                                      double *medium_min_out,
                                                      double *high_min_out)
{
    double remaining = 100.0 - value;
    double step = remaining / 3.0;

    if (low_min_out != NULL) {
        *low_min_out = 0.0;
    }
    if (medium_min_out != NULL) {
        *medium_min_out = value + step;
    }
    if (high_min_out != NULL) {
        *high_min_out = value + 2.0 * step;
    }
}

/* Refresh the label that explains the current confidence color ranges. */
static void update_confidence_range_label(overlay_controls_state_t *state)
{
    double medium_min;
    double high_min;
    int medium_start;
    int medium_end;
    int high_start;
    gchar *label_text;

    if (state == NULL || state->confidence_range_label == NULL) {
        return;
    }

    overlay_controls_get_confidence_color_thresholds(state->confidence_threshold,
                                                     NULL,
                                                     &medium_min,
                                                     &high_min);
    medium_start = (int)ceil(medium_min);
    medium_end = (int)floor(high_min) - 1;
    high_start = (int)ceil(high_min);

    if (medium_start < 1) {
        medium_start = 1;
    }
    if (medium_end < medium_start) {
        medium_end = medium_start;
    }
    if (high_start < medium_end + 1) {
        high_start = medium_end + 1;
    }
    if (high_start > 100) {
        high_start = 100;
    }

    label_text = g_strdup_printf("Low: 0-%d  Med: %d-%d  High: %d-100",
                                 medium_start - 1,
                                 medium_start, medium_end,
                                 high_start);
    gtk_label_set_text(GTK_LABEL(state->confidence_range_label), label_text);
    g_free(label_text);
}

/* Create a labeled switch row and bind its callback. */
static GtkWidget *create_toggle_row(const char *title,
                                    const char *subtitle,
                                    GtkWidget *control)
{
    GtkWidget *row = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    GtkWidget *header_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 12);
    GtkWidget *title_label = gtk_label_new(title);
    GtkWidget *subtitle_label = NULL;

    gtk_widget_add_css_class(row, "toggle-row");
    gtk_widget_set_hexpand(row, TRUE);
    gtk_widget_set_hexpand(header_row, TRUE);

    gtk_widget_set_halign(title_label, GTK_ALIGN_START);
    gtk_widget_set_hexpand(title_label, TRUE);
    gtk_widget_add_css_class(title_label, "toggle-title");

    gtk_widget_set_halign(control, GTK_ALIGN_END);
    gtk_widget_set_valign(control, GTK_ALIGN_CENTER);

    gtk_box_append(GTK_BOX(header_row), title_label);
    gtk_box_append(GTK_BOX(header_row), control);
    gtk_box_append(GTK_BOX(row), header_row);
    if (subtitle != NULL && subtitle[0] != '\0') {
        subtitle_label = gtk_label_new(subtitle);
        gtk_widget_set_halign(subtitle_label, GTK_ALIGN_START);
        gtk_widget_add_css_class(subtitle_label, "toggle-subtitle");
        gtk_label_set_wrap(GTK_LABEL(subtitle_label), FALSE);
        gtk_label_set_ellipsize(GTK_LABEL(subtitle_label), PANGO_ELLIPSIZE_END);
        gtk_label_set_single_line_mode(GTK_LABEL(subtitle_label), TRUE);
        gtk_widget_set_hexpand(subtitle_label, TRUE);
        gtk_box_append(GTK_BOX(row), subtitle_label);
    }

    return row;
}

/* Create the confidence-threshold slider and value label. */
static GtkWidget *create_slider_block(const char *title,
                                      const char *subtitle,
                                      GtkWidget *scale)
{
    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
    GtkWidget *title_label = NULL;
    GtkWidget *subtitle_label = NULL;

    gtk_widget_add_css_class(box, "slider-block");
    gtk_widget_set_hexpand(box, TRUE);
    if (title != NULL && title[0] != '\0') {
        title_label = gtk_label_new(title);
        gtk_widget_set_halign(title_label, GTK_ALIGN_START);
        gtk_widget_add_css_class(title_label, "toggle-title");
        gtk_box_append(GTK_BOX(box), title_label);
    }
    if (subtitle != NULL && subtitle[0] != '\0') {
        subtitle_label = gtk_label_new(subtitle);
        gtk_widget_set_halign(subtitle_label, GTK_ALIGN_START);
        gtk_widget_add_css_class(subtitle_label, "toggle-subtitle");
        gtk_label_set_wrap(GTK_LABEL(subtitle_label), FALSE);
        gtk_label_set_ellipsize(GTK_LABEL(subtitle_label), PANGO_ELLIPSIZE_END);
        gtk_label_set_single_line_mode(GTK_LABEL(subtitle_label), TRUE);
        gtk_widget_set_hexpand(subtitle_label, TRUE);
        gtk_box_append(GTK_BOX(box), subtitle_label);
    }
    gtk_widget_set_hexpand(scale, TRUE);
    gtk_box_append(GTK_BOX(box), scale);

    return box;
}

/* Return the user-facing label for a robot mode. */
static const char *robot_mode_to_display(const char *robot_mode)
{
    if (robot_mode == NULL || robot_mode[0] == '\0') {
        return "Unknown";
    }

    if (strcmp(robot_mode, "joystick") == 0) {
        return "Joystick";
    }

    if (strcmp(robot_mode, "joint") == 0) {
        return "Joint";
    }

    if (strcmp(robot_mode, "ik_joint") == 0) {
        return "IK Joint";
    }

    if (strcmp(robot_mode, "screensaver") == 0) {
        return "Screensaver";
    }

    return robot_mode;
}

/* Return the user-facing label for a screensaver mode. */
static const char *screensaver_mode_to_display(const char *screensaver_mode)
{
    if (screensaver_mode == NULL || screensaver_mode[0] == '\0') {
        return NULL;
    }

    if (strcmp(screensaver_mode, "figure_8") == 0) {
        return "Figure 8";
    }

    if (strcmp(screensaver_mode, "wave") == 0) {
        return "Wave";
    }

    if (strcmp(screensaver_mode, "cobra") == 0) {
        return "Cobra";
    }

    if (strcmp(screensaver_mode, "conductor") == 0) {
        return "Conductor";
    }

    return screensaver_mode;
}

/* Apply skeleton visibility changes. */
static void on_toggle_skeleton(GObject *object,
                               GParamSpec *pspec,
                               gpointer user_data)
{
    overlay_controls_state_t *state = user_data;

    (void)pspec;
    state->show_skeleton = gtk_switch_get_active(GTK_SWITCH(object));
}

/* Apply joint-marker visibility changes. */
static void on_toggle_joints(GObject *object,
                             GParamSpec *pspec,
                             gpointer user_data)
{
    overlay_controls_state_t *state = user_data;

    (void)pspec;
    state->show_joints = gtk_switch_get_active(GTK_SWITCH(object));
}

/* Apply ROI visibility changes. */
static void on_toggle_roi(GObject *object,
                          GParamSpec *pspec,
                          gpointer user_data)
{
    overlay_controls_state_t *state = user_data;

    (void)pspec;
    state->show_roi = gtk_switch_get_active(GTK_SWITCH(object));
}

/* Apply gesture-label visibility changes. */
static void on_toggle_gestures(GObject *object,
                               GParamSpec *pspec,
                               gpointer user_data)
{
    overlay_controls_state_t *state = user_data;

    (void)pspec;
    state->show_gestures = gtk_switch_get_active(GTK_SWITCH(object));
}

/* Apply confidence-label visibility changes. */
static void on_toggle_confidence(GObject *object,
                                 GParamSpec *pspec,
                                 gpointer user_data)
{
    overlay_controls_state_t *state = user_data;

    (void)pspec;
    state->filter_low_confidence = gtk_switch_get_active(GTK_SWITCH(object));
}

/* Publish a new confidence threshold and redraw the overlay. */
static void on_confidence_threshold_changed(GtkRange *range, gpointer user_data)
{
    overlay_controls_state_t *state = user_data;

    state->confidence_threshold = gtk_range_get_value(range);
    update_confidence_range_label(state);
    g_print("Confidence threshold: %.0f\n", state->confidence_threshold);
}

/* Request a robot-control state change. */
static gboolean on_toggle_robot(GtkSwitch *toggle,
                                gboolean enabled,
                                gpointer user_data)
{
    overlay_controls_state_t *state = user_data;

    if (state->updating_robot_toggle) {
        gtk_switch_set_state(toggle, enabled);
        return TRUE;
    }

    if (state->robot_control_toggled_cb != NULL &&
        !state->robot_control_toggled_cb(
            enabled, state->robot_control_toggled_user_data)) {
        state->updating_robot_toggle = TRUE;
        gtk_switch_set_active(toggle, state->enable_robot_control);
        gtk_switch_set_state(toggle, state->enable_robot_control);
        state->updating_robot_toggle = FALSE;
        gtk_widget_set_sensitive(GTK_WIDGET(toggle), FALSE);
        return TRUE;
    }

    state->enable_robot_control = enabled;
    gtk_switch_set_state(toggle, enabled);
    return TRUE;
}

/* Request a wrist-control state change. */
static gboolean on_toggle_wrist(GtkSwitch *toggle,
                                gboolean enabled,
                                gpointer user_data)
{
    overlay_controls_state_t *state = user_data;

    if (state->updating_wrist_toggle) {
        gtk_switch_set_state(toggle, enabled);
        return TRUE;
    }

    if (state->wrist_control_toggled_cb != NULL &&
        !state->wrist_control_toggled_cb(
            enabled, state->wrist_control_toggled_user_data)) {
        state->updating_wrist_toggle = TRUE;
        gtk_switch_set_active(toggle, state->enable_wrist_control);
        gtk_switch_set_state(toggle, state->enable_wrist_control);
        state->updating_wrist_toggle = FALSE;
        gtk_widget_set_sensitive(GTK_WIDGET(toggle), FALSE);
        return TRUE;
    }

    state->enable_wrist_control = enabled;
    gtk_switch_set_state(toggle, enabled);
    return TRUE;
}

/* Apply video mirroring changes. */
static void on_toggle_video_flip(GObject *object,
                                 GParamSpec *pspec,
                                 gpointer user_data)
{
    overlay_controls_state_t *state = user_data;

    (void)pspec;
    state->flip_video = gtk_switch_get_active(GTK_SWITCH(object));
    if (state->video_flip_changed_cb != NULL) {
        state->video_flip_changed_cb(state->flip_video,
                                     state->video_flip_changed_user_data);
    }
}

/* Redraw the overlay after a toggle changes. */
static void queue_draw_after_toggle(GtkWidget *widget, gpointer user_data)
{
    GtkWidget *drawing_area = user_data;

    (void)widget;
    if (!DEBUG_DISABLE_OVERLAY_QUEUE_DRAW) {
        gtk_widget_queue_draw(drawing_area);
    }
}

/* Redraw the overlay after a switch state notification. */
static void queue_draw_after_switch_notify(GObject *object,
                                           GParamSpec *pspec,
                                           gpointer user_data)
{
    (void)object;
    (void)pspec;
    queue_draw_after_toggle(NULL, user_data);
}

/* Initialize overlay controls to their default state. */
void overlay_controls_state_init(overlay_controls_state_t *state)
{
    state->video_flip_changed_cb = NULL;
    state->video_flip_changed_user_data = NULL;
    state->flip_video = TRUE;
    state->show_video_flip = FALSE;
    state->robot_control_toggled_cb = NULL;
    state->robot_control_toggled_user_data = NULL;
    state->wrist_control_toggled_cb = NULL;
    state->wrist_control_toggled_user_data = NULL;
    state->robot_mode_label = NULL;
    state->skeleton_toggle = NULL;
    state->joints_toggle = NULL;
    state->roi_toggle = NULL;
    state->gestures_toggle = NULL;
    state->confidence_toggle = NULL;
    state->robot_toggle = NULL;
    state->wrist_toggle = NULL;
    state->confidence_threshold_block = NULL;
    state->confidence_scale = NULL;
    state->confidence_range_label = NULL;
    state->session_card_visible = TRUE;
    state->roi_available = TRUE;
    state->robot_control_available = TRUE;
    state->confidence_threshold = CONFIDENCE_THRESHOLD_DEFAULT;
    state->show_skeleton = TRUE;
    state->show_joints = TRUE;
    state->show_roi = TRUE;
    state->show_gestures = TRUE;
    state->filter_low_confidence = TRUE;
    state->enable_robot_control = TRUE;
    state->enable_wrist_control = TRUE;
    state->show_overlay_fills = TRUE;
    state->show_controls = TRUE;
    state->show_debug_info = FALSE;
    state->show_joint_popups = TRUE;
    state->debug_info_corner = DEBUG_INFO_TOP_RIGHT;
    state->updating_robot_toggle = FALSE;
    state->updating_wrist_toggle = FALSE;
}

/* Update the robot mode summary shown in the controls panel. */
void overlay_controls_set_robot_mode(overlay_controls_state_t *state,
                                     const char *robot_mode,
                                     const char *screensaver_mode)
{
    const char *display_value;
    const char *screensaver_display;
    gchar *label_text;

    if (state == NULL || state->robot_mode_label == NULL) {
        return;
    }

    display_value = robot_mode_to_display(robot_mode);
    screensaver_display = screensaver_mode_to_display(screensaver_mode);
    if (g_strcmp0(robot_mode, "screensaver") == 0 &&
        screensaver_display != NULL) {
        label_text = g_strdup_printf("Robot Mode  %s (%s)",
                                     display_value,
                                     screensaver_display);
    } else {
        label_text = g_strdup_printf("Robot Mode  %s", display_value);
    }
    gtk_label_set_text(GTK_LABEL(state->robot_mode_label), label_text);
    g_free(label_text);
}

/* Build the control panel widgets and bind them to the shared state. */
GtkWidget *overlay_controls_create(GtkWidget *drawing_area,
                                   overlay_controls_state_t *state)
{
    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 18);
    GtkWidget *hero_box;
    GtkWidget *hero_header;
    GtkWidget *title_label;
    GtkWidget *status_card;
    GtkWidget *status_hotkey_chip;
    GtkWidget *confidence_section;
    GtkWidget *visuals_card;
    GtkWidget *robot_card;
    GtkWidget *camera_card;
    GtkWidget *confidence_cb;
    GtkWidget *confidence_row;
    GtkWidget *confidence_scale;
    GtkWidget *confidence_range_label;
    GtkWidget *video_flip_cb = NULL;
    GtkWidget *joints_cb;
    GtkWidget *robot_mode_label;
    GtkWidget *skeleton_cb;
    GtkWidget *roi_cb = NULL;
    GtkWidget *gestures_cb = NULL;
    GtkWidget *robot_cb = NULL;
    GtkWidget *wrist_cb = NULL;
    GtkWidget *simulator_label;

    gtk_widget_add_css_class(box, "control-panel");
    gtk_widget_set_hexpand(box, TRUE);

    hero_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_widget_add_css_class(hero_box, "hero-block");
    hero_header = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 12);
    gtk_widget_set_hexpand(hero_header, TRUE);
    title_label = gtk_label_new("Control Studio");
    gtk_widget_set_halign(title_label, GTK_ALIGN_START);
    gtk_widget_set_hexpand(title_label, TRUE);
    gtk_widget_add_css_class(title_label, "hero-title");
    status_hotkey_chip = create_status_hotkey_chip("H - Hotkeys");
    gtk_box_append(GTK_BOX(hero_header), title_label);
    gtk_box_append(GTK_BOX(hero_header), status_hotkey_chip);
    gtk_box_append(GTK_BOX(hero_box), hero_header);

    status_card = NULL;
    if (state->session_card_visible) {
        GtkWidget *status_header = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 12);
        GtkWidget *status_title = create_section_title("Session");

        status_card = create_card_box();
        gtk_widget_set_hexpand(status_header, TRUE);
        gtk_widget_set_hexpand(status_title, TRUE);
        gtk_box_append(GTK_BOX(status_header), status_title);

        simulator_label = gtk_label_new("Live Camera Feed");
        gtk_widget_set_halign(simulator_label, GTK_ALIGN_START);
        gtk_widget_add_css_class(simulator_label, "status-chip");

        robot_mode_label = gtk_label_new(NULL);
        gtk_widget_set_halign(robot_mode_label, GTK_ALIGN_START);
        gtk_widget_set_valign(robot_mode_label, GTK_ALIGN_CENTER);
        gtk_widget_add_css_class(robot_mode_label, "status-value");
        state->robot_mode_label = robot_mode_label;
        overlay_controls_set_robot_mode(state, NULL, NULL);

        gtk_box_append(GTK_BOX(status_card), status_header);
        gtk_box_append(GTK_BOX(status_card), simulator_label);
        gtk_box_append(GTK_BOX(status_card), robot_mode_label);
    } else {
        state->robot_mode_label = NULL;
    }

    skeleton_cb = gtk_switch_new();
    gtk_switch_set_active(GTK_SWITCH(skeleton_cb), state->show_skeleton);
    state->skeleton_toggle = skeleton_cb;

    joints_cb = gtk_switch_new();
    gtk_switch_set_active(GTK_SWITCH(joints_cb), state->show_joints);
    state->joints_toggle = joints_cb;

    confidence_cb = gtk_switch_new();
    gtk_switch_set_active(GTK_SWITCH(confidence_cb), state->filter_low_confidence);
    state->confidence_toggle = confidence_cb;
    if (state->roi_available) {
        roi_cb = gtk_switch_new();
        gtk_switch_set_active(GTK_SWITCH(roi_cb), state->show_roi);
        state->roi_toggle = roi_cb;
    }
    gestures_cb = gtk_switch_new();
    gtk_switch_set_active(GTK_SWITCH(gestures_cb), state->show_gestures);
    state->gestures_toggle = gestures_cb;
    if (state->robot_control_available) {
        robot_cb = gtk_switch_new();
        gtk_switch_set_active(GTK_SWITCH(robot_cb), state->enable_robot_control);
        state->robot_toggle = robot_cb;
        wrist_cb = gtk_switch_new();
        gtk_switch_set_active(GTK_SWITCH(wrist_cb), state->enable_wrist_control);
        state->wrist_toggle = wrist_cb;
    }

    confidence_scale = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL,
                                                CONFIDENCE_THRESHOLD_MIN,
                                                CONFIDENCE_THRESHOLD_MAX,
                                                1.0);
    gtk_range_set_value(GTK_RANGE(confidence_scale), state->confidence_threshold);
    gtk_scale_set_digits(GTK_SCALE(confidence_scale), 0);
    gtk_scale_set_draw_value(GTK_SCALE(confidence_scale), TRUE);
    gtk_widget_add_css_class(confidence_scale, "designer-scale");
    state->confidence_scale = confidence_scale;

    confidence_range_label = gtk_label_new(NULL);
    gtk_widget_set_halign(confidence_range_label, GTK_ALIGN_START);
    gtk_widget_add_css_class(confidence_range_label, "confidence-range-label");
    state->confidence_range_label = confidence_range_label;

    visuals_card = create_card_box();
    gtk_box_append(GTK_BOX(visuals_card), create_section_title("Visual Layers"));
    gtk_box_append(GTK_BOX(visuals_card),
                   create_toggle_row("Skeleton",
                                     "Body lines over video.",
                                     skeleton_cb));
    gtk_box_append(GTK_BOX(visuals_card),
                   create_toggle_row("Joints",
                                     "Joint points for each pose.",
                                     joints_cb));
    if (state->roi_available) {
        gtk_box_append(GTK_BOX(visuals_card),
                       create_toggle_row("ROI",
                                         "Tracking region overlay.",
                                         roi_cb));
    }
    if (gestures_cb != NULL) {
        gtk_box_append(GTK_BOX(visuals_card),
                       create_toggle_row("Show gestures",
                                         "Detected hand gestures on box labels.",
                                         gestures_cb));
    }
    confidence_section = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_box_append(GTK_BOX(confidence_section),
                   create_toggle_row("Filter Low Confidence",
                                     "Filters out low confidence detections.",
                                     confidence_cb));

    confidence_row = create_slider_block("",
                                         "",
                                         confidence_scale);
    state->confidence_threshold_block = confidence_row;
    gtk_box_append(GTK_BOX(confidence_row), confidence_range_label);
    update_confidence_range_label(state);
    gtk_box_append(GTK_BOX(confidence_section), confidence_row);
    gtk_box_append(GTK_BOX(visuals_card), confidence_section);

    robot_card = NULL;
    if (state->robot_control_available) {
        robot_card = create_card_box();
        gtk_box_append(GTK_BOX(robot_card), create_section_title("Robot Control"));
        gtk_box_append(GTK_BOX(robot_card),
                       create_toggle_row("Enable Robot",
                                         "",
                                         robot_cb));
        gtk_box_append(GTK_BOX(robot_card),
                       create_toggle_row("Wrist control",
                                         "",
                                         wrist_cb));
    }

    if (state->show_video_flip) {
        camera_card = create_card_box();
        gtk_box_append(GTK_BOX(camera_card), create_section_title("Camera"));
        video_flip_cb = gtk_switch_new();
        gtk_switch_set_active(GTK_SWITCH(video_flip_cb), state->flip_video);
        gtk_box_append(GTK_BOX(camera_card),
                       create_toggle_row("Flip Video",
                                         "Mirror video and all camera overlays.",
                                         video_flip_cb));
    } else {
        camera_card = NULL;
    }

    gtk_box_append(GTK_BOX(box), hero_box);
    if (status_card != NULL) {
        gtk_box_append(GTK_BOX(box), status_card);
    }
    gtk_box_append(GTK_BOX(box), visuals_card);
    if (robot_card != NULL) {
        gtk_box_append(GTK_BOX(box), robot_card);
    }
    if (camera_card != NULL) {
        gtk_box_append(GTK_BOX(box), camera_card);
    }

    if (video_flip_cb != NULL) {
        g_signal_connect(video_flip_cb, "notify::active",
                         G_CALLBACK(on_toggle_video_flip), state);
    }
    g_signal_connect(skeleton_cb, "notify::active",
                     G_CALLBACK(on_toggle_skeleton), state);
    g_signal_connect(joints_cb, "notify::active",
                     G_CALLBACK(on_toggle_joints), state);
    if (roi_cb != NULL) {
        g_signal_connect(roi_cb, "notify::active",
                         G_CALLBACK(on_toggle_roi), state);
    }
    if (gestures_cb != NULL) {
        g_signal_connect(gestures_cb, "notify::active",
                         G_CALLBACK(on_toggle_gestures), state);
    }
    g_signal_connect(confidence_cb, "notify::active",
                     G_CALLBACK(on_toggle_confidence), state);
    g_signal_connect(confidence_scale, "value-changed",
                     G_CALLBACK(on_confidence_threshold_changed), state);
    if (robot_cb != NULL) {
        g_signal_connect(robot_cb, "state-set",
                         G_CALLBACK(on_toggle_robot), state);
        g_signal_connect(wrist_cb, "state-set",
                         G_CALLBACK(on_toggle_wrist), state);
    }

    g_signal_connect(skeleton_cb, "notify::active",
                     G_CALLBACK(queue_draw_after_switch_notify), drawing_area);
    g_signal_connect(joints_cb, "notify::active",
                     G_CALLBACK(queue_draw_after_switch_notify), drawing_area);
    if (roi_cb != NULL) {
        g_signal_connect(roi_cb, "notify::active",
                         G_CALLBACK(queue_draw_after_switch_notify), drawing_area);
    }
    if (gestures_cb != NULL) {
        g_signal_connect(gestures_cb, "notify::active",
                         G_CALLBACK(queue_draw_after_switch_notify), drawing_area);
    }
    g_signal_connect(confidence_cb, "notify::active",
                     G_CALLBACK(queue_draw_after_switch_notify), drawing_area);
    g_signal_connect(confidence_scale, "value-changed",
                     G_CALLBACK(queue_draw_after_toggle), drawing_area);
    if (robot_cb != NULL) {
        g_signal_connect(robot_cb, "notify::active",
                         G_CALLBACK(queue_draw_after_switch_notify), drawing_area);
    }

    return box;
}
