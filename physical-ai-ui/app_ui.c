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

#include "app_ui.h"

#include "app_config.h"
#include "app_theme.h"
#include "overlay_renderer.h"
#include "user_settings.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define ROI_HIT_RADIUS_PX 12
#define CONFIDENCE_HOTKEY_STEP 1.0

#define APP_HEADER_TITLE "Model Robot Arm with Physical AI"
#define APP_HEADER_SUBTITLE \
    "Move your hand or a gamepad to control the robot arm — then build your own!"
#define APP_HEADER_SUBTITLE_SECONDARY "Powered by Raspberry Pi 5, ROS 2, and QNX."

/* Report whether the robot mode supports pose joint popups. */
static bool pose_popups_available_for_robot_mode(const char *robot_mode)
{
    return strcmp(robot_mode, "joint") == 0 ||
           strcmp(robot_mode, "ik_joint") == 0;
}

/* Write a text value to a QPP control endpoint. */
static bool write_text_control_file(const char *path, const char *value)
{
    char buffer[256];
    int saved_errno = 0;
    int fd;
    int len;
    ssize_t bytes_written;

    if (path == NULL || value == NULL) {
        return false;
    }

    len = snprintf(buffer, sizeof(buffer), "%s\n", value);
    if (len < 0 || (size_t)len >= sizeof(buffer)) {
        printf("QPP control value too long for %s\n", path);
        return false;
    }

    fd = open(path, O_WRONLY);
    if (fd == -1) {
        printf("Failed to open QPP control %s: %s\n",
               path, strerror(errno));
        return false;
    }

    do {
        bytes_written = write(fd, buffer, (size_t)len);
    } while (bytes_written < 0 && errno == EINTR);
    if (bytes_written < 0) {
        saved_errno = errno;
    }
    close(fd);

    if (bytes_written != len) {
        if (bytes_written < 0) {
            printf("Failed to write QPP control %s: %s\n",
                   path, strerror(saved_errno));
        } else {
            printf("Short QPP control write to %s (%zd/%d bytes)\n",
                   path, bytes_written, len);
        }
        return false;
    }

    return true;
}

/* Read a QPP boolean value snapshot. QPP's text interface returns true/false. */
static bool read_qpp_boolean(const char *path, gboolean *value_out)
{
    char buffer[32];
    int saved_errno = 0;
    int fd;
    ssize_t bytes_read;

    if (path == NULL || value_out == NULL) {
        return false;
    }

    fd = open(path, O_RDONLY);
    if (fd == -1) {
        printf("Failed to open QPP boolean %s: %s\n", path, strerror(errno));
        return false;
    }

    do {
        bytes_read = read(fd, buffer, sizeof(buffer) - 1);
    } while (bytes_read < 0 && errno == EINTR);
    if (bytes_read < 0) {
        saved_errno = errno;
    }
    close(fd);

    if (bytes_read < 0) {
        printf("Failed to read QPP boolean %s: %s\n", path,
               strerror(saved_errno));
        return false;
    }

    buffer[bytes_read] = '\0';
    g_strstrip(buffer);
    if (strcmp(buffer, "true") == 0) {
        *value_out = TRUE;
        return true;
    }
    if (strcmp(buffer, "false") == 0) {
        *value_out = FALSE;
        return true;
    }

    printf("Invalid QPP boolean value from %s: %s\n", path, buffer);
    return false;
}

/* QPP represents the robot ROI as four comma-separated int8 percentages. */
static bool write_qpp_robot_roi(const roi_info_t *roi)
{
    char value[32];
    int len;

    if (roi == NULL) {
        return false;
    }

    len = snprintf(value, sizeof(value), "%d,%d,%d,%d",
                   CLAMP(roi->x, 0, 100), CLAMP(roi->y, 0, 100),
                   CLAMP(roi->end_x, 0, 100), CLAMP(roi->end_y, 0, 100));
    if (len < 0 || (size_t)len >= sizeof(value)) {
        return false;
    }

    return write_text_control_file(PHYSICAL_QPP_ROBOT_ROI_CONTROL_PATH, value);
}

/* Synchronize side-panel visibility with the window layout. */
static void sync_side_studio_visibility(app_ui_t *ui)
{
    if (ui == NULL) {
        return;
    }

    if (ui->controls_box != NULL) {
        gtk_widget_set_visible(ui->controls_box, ui->controls.show_controls);
    }
}

/* Show or hide the side control studio. */
static void set_control_studio_visible(app_ui_t *ui, gboolean visible)
{
    if (ui == NULL) {
        return;
    }
    ui->controls.show_controls = visible;
    sync_side_studio_visibility(ui);
}

/* Toggle visibility of the side control studio. */
static void toggle_control_studio(app_ui_t *ui)
{
    if (ui->controls.show_controls) {
        set_control_studio_visible(ui, FALSE);
    } else {
        set_control_studio_visible(ui, TRUE);
    }
}

/* Mirror the picture and redraw pose overlays using the same display state. */
static void on_video_flip_changed(gboolean enabled, gpointer user_data)
{
    app_ui_t *ui = user_data;

    if (enabled) {
        gtk_widget_add_css_class(ui->picture, "video-mirrored");
    } else {
        gtk_widget_remove_css_class(ui->picture, "video-mirrored");
    }
    gtk_widget_queue_draw(ui->overlay);
    if (!user_settings_save_video_flip(enabled)) {
        puts("Failed to save video flip setting");
    }
}

/* Apply robot-control changes requested by the controls panel. */
static gboolean on_robot_control_toggled(gboolean enabled, gpointer user_data)
{
    (void)user_data;

    return write_text_control_file(PHYSICAL_QPP_ROBOT_ENABLED_CONTROL_PATH,
                                   enabled ? "true" : "false");
}

/* Apply wrist-control changes requested by the controls panel. */
static gboolean on_wrist_control_toggled(gboolean enabled, gpointer user_data)
{
    (void)user_data;

    return write_text_control_file(PHYSICAL_QPP_WRIST_CONTROL_WRITE_PATH,
                                   enabled ? "true" : "false");
}

/* Queue a redraw of the overlay widget when it is available. */
static void queue_overlay_draw(GtkWidget *overlay)
{
    if (!DEBUG_DISABLE_OVERLAY_QUEUE_DRAW) {
        gtk_widget_queue_draw(overlay);
    }
}

/* Clamp an integer value to the supplied range. */
static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/* Clamp a floating-point value to the supplied range. */
static double clamp_double(double value, double min_value, double max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/* Update a switch without emitting a redundant state change. */
static void sync_switch_state(GtkWidget *toggle, gboolean active)
{
    if (toggle == NULL) {
        return;
    }

    gtk_switch_set_active(GTK_SWITCH(toggle), active);
}

/* Queue an overlay redraw for the application UI. */
static void queue_overlay_draw_for_ui(app_ui_t *ui)
{
    if (!DEBUG_DISABLE_OVERLAY_QUEUE_DRAW) {
        gtk_widget_queue_draw(ui->overlay);
    }
}

/* Report whether joint popups should be visible in the current UI state. */
static gboolean joint_popups_visible_for_ui(const app_ui_t *ui)
{
    return ui->controls.show_joint_popups &&
           pose_popups_available_for_robot_mode(ui->current_robot_mode);
}

/* Move the debug panel to the requested corner. */
static void move_debug_info(app_ui_t *ui, debug_info_corner_t corner)
{
    ui->controls.debug_info_corner = corner;
    queue_overlay_draw_for_ui(ui);
}

/* Show or hide the keyboard-shortcuts popup. */
static void set_hotkeys_popup_visible(app_ui_t *ui, gboolean visible)
{
    if (ui->hotkeys_popup == NULL) {
        return;
    }

    gtk_widget_set_visible(ui->hotkeys_popup, visible);
}

/* Show or hide the robot-mode popup. */
static void set_robot_mode_popup_visible(app_ui_t *ui, gboolean visible)
{
    if (ui->robot_mode_popup == NULL) {
        return;
    }

    if (gtk_widget_get_visible(ui->robot_mode_popup) != visible) {
        gtk_widget_set_visible(ui->robot_mode_popup, visible);
    }
}

/* Select the popup child associated with the screensaver state. */
static const char *robot_mode_popup_screensaver_child_name(
    const char *screensaver_mode)
{
    if (strcmp(screensaver_mode, "cobra") == 0) {
        return "screensaver-cobra";
    }

    if (strcmp(screensaver_mode, "conductor") == 0) {
        return "screensaver-conductor";
    }

    if (strcmp(screensaver_mode, "figure_8") == 0) {
        return "screensaver-figure-eight";
    }

    if (strcmp(screensaver_mode, "wave") == 0) {
        return "screensaver-wave";
    }

    return "screensaver";
}

/* Update robot-mode popup content from the latest modes. */
static void update_robot_mode_popup(app_ui_t *ui, const char *robot_mode,
                                    const char *screensaver_mode)
{
    const char *popup_child_name = NULL;

    if (ui->robot_mode_stack == NULL) {
        return;
    }

    if (robot_mode == NULL) {
        robot_mode = "";
    }
    if (screensaver_mode == NULL) {
        screensaver_mode = "";
    }

    if (strcmp(ui->current_robot_mode, robot_mode) != 0) {
        strlcpy(ui->current_robot_mode, robot_mode,
                sizeof(ui->current_robot_mode));
    }
    if (strcmp(ui->current_screensaver_mode, screensaver_mode) != 0) {
        strlcpy(ui->current_screensaver_mode, screensaver_mode,
                sizeof(ui->current_screensaver_mode));
    }

    if (ui->qr_popup != NULL && gtk_widget_get_visible(ui->qr_popup)) {
        set_robot_mode_popup_visible(ui, FALSE);
        return;
    }

    if (strcmp(robot_mode, "joystick") == 0) {
        popup_child_name = "joystick";
    } else if (strcmp(robot_mode, "screensaver") == 0) {
        popup_child_name =
            robot_mode_popup_screensaver_child_name(screensaver_mode);
    }

    if (popup_child_name == NULL) {
        set_robot_mode_popup_visible(ui, FALSE);
        return;
    }

    if (g_strcmp0(gtk_stack_get_visible_child_name(
                      GTK_STACK(ui->robot_mode_stack)),
                  popup_child_name) != 0) {
        gtk_stack_set_visible_child_name(GTK_STACK(ui->robot_mode_stack),
                                         popup_child_name);
    }
    set_robot_mode_popup_visible(ui, TRUE);
}

/* Toggle visibility of the keyboard-shortcuts popup. */
static void toggle_hotkeys_popup(app_ui_t *ui)
{
    if (ui->hotkeys_popup == NULL) {
        return;
    }

    if (ui->qr_popup != NULL) {
        gtk_widget_set_visible(ui->qr_popup, FALSE);
    }
    gtk_widget_set_visible(ui->hotkeys_popup,
                           !gtk_widget_get_visible(ui->hotkeys_popup));
}

/* Show or hide the QR code popup. */
static void set_qr_popup_visible(app_ui_t *ui, gboolean visible)
{
    if (ui->qr_popup == NULL) {
        return;
    }

    gtk_widget_set_visible(ui->qr_popup, visible);
}

/* Toggle visibility of the QR code popup. */
static void toggle_qr_popup(app_ui_t *ui)
{
    if (ui->qr_popup == NULL) {
        return;
    }

    set_hotkeys_popup_visible(ui, FALSE);
    gtk_widget_set_visible(ui->qr_popup,
                           !gtk_widget_get_visible(ui->qr_popup));
    update_robot_mode_popup(ui, ui->current_robot_mode,
                            ui->current_screensaver_mode);
}

/* Close the QR popup when its backdrop is pressed. */
static void on_qr_popup_pressed(GtkGestureClick *gesture,
                                int n_press,
                                double x,
                                double y,
                                gpointer user_data)
{
    app_ui_t *ui = user_data;

    (void)gesture;
    (void)n_press;
    (void)x;
    (void)y;
    toggle_qr_popup(ui);
}

/* Apply layout changes required by the current window mode. */
static void sync_window_mode_layout(app_ui_t *ui)
{
    gboolean is_fullscreen = *ui->fullscreen;
    if (ui->topbar != NULL) {
        gtk_widget_set_visible(ui->topbar, is_fullscreen);
    }
    if (ui->camera_card != NULL) {
        if (is_fullscreen) {
            gtk_widget_remove_css_class(ui->camera_card, "camera-card-compact");
        } else {
            gtk_widget_add_css_class(ui->camera_card, "camera-card-compact");
        }
    }
    if (ui->camera_header != NULL) {
        gtk_widget_set_halign(ui->camera_header,
                              is_fullscreen ? GTK_ALIGN_FILL :
                                              GTK_ALIGN_START);
        gtk_widget_set_hexpand(ui->camera_header, TRUE);
        if (is_fullscreen) {
            gtk_widget_remove_css_class(ui->camera_header, "camera-header-compact");
        } else {
            gtk_widget_add_css_class(ui->camera_header, "camera-header-compact");
        }
    }
    if (ui->camera_brand != NULL) {
        gtk_widget_set_visible(ui->camera_brand, !is_fullscreen);
    }
    if (ui->camera_title != NULL) {
        gtk_label_set_text(GTK_LABEL(ui->camera_title),
                           APP_HEADER_TITLE);
        gtk_widget_set_halign(ui->camera_title,
                              GTK_ALIGN_START);
        gtk_label_set_xalign(GTK_LABEL(ui->camera_title),
                             0.0f);
    }
    if (ui->camera_subtitle != NULL) {
        gtk_label_set_text(GTK_LABEL(ui->camera_subtitle),
                           APP_HEADER_SUBTITLE);
        gtk_widget_set_visible(ui->camera_subtitle, is_fullscreen);
    }
    if (ui->camera_subtitle_secondary != NULL) {
        gtk_label_set_text(GTK_LABEL(ui->camera_subtitle_secondary),
                           APP_HEADER_SUBTITLE_SECONDARY);
        gtk_widget_set_visible(ui->camera_subtitle_secondary,
                               is_fullscreen);
    }
}

/* Set and display the current confidence threshold. */
static void set_confidence_threshold(app_ui_t *ui, double value)
{
    double clamped_value = clamp_double(value,
                                        CONFIDENCE_THRESHOLD_MIN,
                                        CONFIDENCE_THRESHOLD_MAX);

    ui->controls.confidence_threshold = clamped_value;
    if (ui->controls.confidence_scale != NULL) {
        gtk_range_set_value(GTK_RANGE(ui->controls.confidence_scale),
                            clamped_value);
    }
}

/* Adjust the confidence threshold by a keyboard-driven increment. */
static void adjust_confidence_threshold(app_ui_t *ui, double delta)
{
    set_confidence_threshold(ui, ui->controls.confidence_threshold + delta);
}

/* Enable or disable robot control through the QPP interface. */
static gboolean set_robot_enabled(app_ui_t *ui, gboolean enabled)
{
    if (ui->controls.robot_control_toggled_cb != NULL &&
        !ui->controls.robot_control_toggled_cb(
            enabled, ui->controls.robot_control_toggled_user_data)) {
        return FALSE;
    }

    ui->controls.enable_robot_control = enabled;
    if (ui->controls.robot_toggle != NULL) {
        ui->controls.updating_robot_toggle = TRUE;
        gtk_switch_set_active(GTK_SWITCH(ui->controls.robot_toggle), enabled);
        gtk_switch_set_state(GTK_SWITCH(ui->controls.robot_toggle), enabled);
        ui->controls.updating_robot_toggle = FALSE;
    }
    return TRUE;
}

/* Toggle robot control while preserving UI state on failure. */
static void toggle_robot_enabled(app_ui_t *ui)
{
    (void)set_robot_enabled(ui, !ui->controls.enable_robot_control);
}

/* Build the hotkey reference popup shown from the header bar. */
static GtkWidget *create_hotkeys_popup(void)
{
    static const char *hotkey_lines[] = {
        "H  Show or hide hotkeys",
        "Q  Show or hide GitHub QR code",
        "F  Toggle fullscreen",
        "C  Show or hide Control Studio",
        "D  Toggle debug info",
        "1  Move debug info to top left",
        "2  Move debug info to top right",
        "3  Move debug info to bottom left",
        "4  Move debug info to bottom right",
        "P  Show or hide joint/IK popups",
        "E/K  Enable or disable robot",
        "R  Toggle ROI",
        "S  Toggle skeleton",
        "J  Toggle joints",
        "L  Toggle low-confidence filter",
        "[  Decrease confidence",
        "]  Increase confidence",
        "ESC  Quit"
    };
    const size_t hotkey_count = sizeof(hotkey_lines) / sizeof(hotkey_lines[0]);
    GtkWidget *frame = gtk_frame_new(NULL);
    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
    GtkWidget *title = gtk_label_new("Hotkeys");
    size_t i;

    gtk_widget_add_css_class(frame, "hotkeys-popup");
    gtk_widget_set_halign(frame, GTK_ALIGN_END);
    gtk_widget_set_valign(frame, GTK_ALIGN_START);
    gtk_widget_set_margin_top(frame, 24);
    gtk_widget_set_margin_end(frame, 24);
    gtk_widget_set_visible(frame, FALSE);

    gtk_widget_add_css_class(box, "hotkeys-popup-box");
    gtk_widget_set_margin_top(box, 16);
    gtk_widget_set_margin_bottom(box, 16);
    gtk_widget_set_margin_start(box, 18);
    gtk_widget_set_margin_end(box, 18);

    gtk_widget_add_css_class(title, "hotkeys-popup-title");
    gtk_widget_set_halign(title, GTK_ALIGN_START);
    gtk_box_append(GTK_BOX(box), title);

    for (i = 0; i < hotkey_count; ++i) {
        GtkWidget *row = gtk_label_new(hotkey_lines[i]);

        gtk_widget_add_css_class(row, "hotkeys-popup-row");
        gtk_widget_set_halign(row, GTK_ALIGN_START);
        gtk_label_set_xalign(GTK_LABEL(row), 0.0f);
        gtk_box_append(GTK_BOX(box), row);
    }

    gtk_frame_set_child(GTK_FRAME(frame), box);
    return frame;
}

/* Build the QR dialog used to show the GitHub handoff link. */
static GtkWidget *create_qr_popup(app_ui_t *ui)
{
    GtkWidget *frame = gtk_frame_new(NULL);
    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 12);
    GtkWidget *title = gtk_label_new("Scan to open GitHub");
    GtkWidget *picture =
        gtk_picture_new_for_resource("/com/qnx/physical-ai-ui/ui/qr-code-github.png");
    GtkGesture *click_gesture = gtk_gesture_click_new();

    gtk_widget_add_css_class(frame, "qr-popup");
    gtk_widget_set_halign(frame, GTK_ALIGN_CENTER);
    gtk_widget_set_valign(frame, GTK_ALIGN_CENTER);
    gtk_widget_set_visible(frame, FALSE);

    gtk_widget_add_css_class(box, "qr-popup-box");
    gtk_widget_set_margin_top(box, 20);
    gtk_widget_set_margin_bottom(box, 20);
    gtk_widget_set_margin_start(box, 20);
    gtk_widget_set_margin_end(box, 20);

    gtk_widget_add_css_class(title, "qr-popup-title");
    gtk_widget_set_halign(title, GTK_ALIGN_CENTER);
    gtk_box_append(GTK_BOX(box), title);

    gtk_widget_add_css_class(picture, "qr-popup-image");
    gtk_picture_set_content_fit(GTK_PICTURE(picture), GTK_CONTENT_FIT_CONTAIN);
    gtk_widget_set_size_request(picture, 320, 320);
    gtk_widget_set_halign(picture, GTK_ALIGN_CENTER);
    gtk_widget_set_valign(picture, GTK_ALIGN_CENTER);
    gtk_widget_set_cursor_from_name(picture, "pointer");
    gtk_gesture_single_set_button(GTK_GESTURE_SINGLE(click_gesture),
                                  GDK_BUTTON_PRIMARY);
    gtk_widget_add_controller(picture, GTK_EVENT_CONTROLLER(click_gesture));
    g_signal_connect(click_gesture, "pressed",
                     G_CALLBACK(on_qr_popup_pressed), ui);
    gtk_box_append(GTK_BOX(box), picture);

    gtk_frame_set_child(GTK_FRAME(frame), box);
    return frame;
}

/* Build the artwork popup used for joystick and screensaver states. */
static GtkWidget *create_robot_mode_popup(app_ui_t *ui)
{
    GtkWidget *frame = gtk_frame_new(NULL);
    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    GtkWidget *stack = gtk_stack_new();
    GtkWidget *joystick_picture =
        gtk_picture_new_for_resource("/com/qnx/physical-ai-ui/ui/robot_mode_joystick.png");
    GtkWidget *screensaver_picture =
        gtk_picture_new_for_resource("/com/qnx/physical-ai-ui/ui/robot_mode_screensaver.png");
    GtkWidget *screensaver_cobra_picture =
        gtk_picture_new_for_resource("/com/qnx/physical-ai-ui/ui/robot_mode_screensaver_cobra.png");
    GtkWidget *screensaver_conductor_picture =
        gtk_picture_new_for_resource(
            "/com/qnx/physical-ai-ui/ui/robot_mode_screensaver_conductor.png");
    GtkWidget *screensaver_figure_eight_picture =
        gtk_picture_new_for_resource(
            "/com/qnx/physical-ai-ui/ui/robot_mode_screensaver_figure_eight.png");
    GtkWidget *screensaver_wave_picture =
        gtk_picture_new_for_resource("/com/qnx/physical-ai-ui/ui/robot_mode_screensaver_wave.png");

    gtk_widget_add_css_class(frame, "robot-mode-popup");
    gtk_widget_set_halign(frame, GTK_ALIGN_CENTER);
    gtk_widget_set_valign(frame, GTK_ALIGN_CENTER);
    gtk_widget_set_hexpand(frame, TRUE);
    gtk_widget_set_vexpand(frame, TRUE);
    gtk_widget_set_visible(frame, FALSE);

    gtk_widget_add_css_class(box, "robot-mode-popup-box");
    gtk_widget_set_hexpand(box, TRUE);
    gtk_widget_set_vexpand(box, TRUE);
    gtk_widget_set_margin_top(box, 20);
    gtk_widget_set_margin_bottom(box, 20);
    gtk_widget_set_margin_start(box, 20);
    gtk_widget_set_margin_end(box, 20);

    gtk_widget_add_css_class(stack, "robot-mode-popup-stack");
    gtk_stack_set_transition_type(GTK_STACK(stack),
                                  GTK_STACK_TRANSITION_TYPE_NONE);
    gtk_widget_set_hexpand(stack, TRUE);
    gtk_widget_set_vexpand(stack, TRUE);
    gtk_widget_set_halign(stack, GTK_ALIGN_CENTER);
    gtk_widget_set_valign(stack, GTK_ALIGN_CENTER);

    gtk_widget_add_css_class(joystick_picture, "robot-mode-popup-image");
    gtk_picture_set_content_fit(GTK_PICTURE(joystick_picture),
                                GTK_CONTENT_FIT_CONTAIN);
    gtk_widget_set_hexpand(joystick_picture, TRUE);
    gtk_widget_set_vexpand(joystick_picture, TRUE);
    gtk_widget_set_halign(joystick_picture, GTK_ALIGN_CENTER);
    gtk_widget_set_valign(joystick_picture, GTK_ALIGN_CENTER);

    gtk_widget_add_css_class(screensaver_picture, "robot-mode-popup-image");
    gtk_picture_set_content_fit(GTK_PICTURE(screensaver_picture),
                                GTK_CONTENT_FIT_CONTAIN);
    gtk_widget_set_hexpand(screensaver_picture, TRUE);
    gtk_widget_set_vexpand(screensaver_picture, TRUE);
    gtk_widget_set_halign(screensaver_picture, GTK_ALIGN_CENTER);
    gtk_widget_set_valign(screensaver_picture, GTK_ALIGN_CENTER);

    gtk_widget_add_css_class(screensaver_cobra_picture, "robot-mode-popup-image");
    gtk_picture_set_content_fit(GTK_PICTURE(screensaver_cobra_picture),
                                GTK_CONTENT_FIT_CONTAIN);
    gtk_widget_set_hexpand(screensaver_cobra_picture, TRUE);
    gtk_widget_set_vexpand(screensaver_cobra_picture, TRUE);
    gtk_widget_set_halign(screensaver_cobra_picture, GTK_ALIGN_CENTER);
    gtk_widget_set_valign(screensaver_cobra_picture, GTK_ALIGN_CENTER);

    gtk_widget_add_css_class(screensaver_conductor_picture, "robot-mode-popup-image");
    gtk_picture_set_content_fit(GTK_PICTURE(screensaver_conductor_picture),
                                GTK_CONTENT_FIT_CONTAIN);
    gtk_widget_set_hexpand(screensaver_conductor_picture, TRUE);
    gtk_widget_set_vexpand(screensaver_conductor_picture, TRUE);
    gtk_widget_set_halign(screensaver_conductor_picture, GTK_ALIGN_CENTER);
    gtk_widget_set_valign(screensaver_conductor_picture, GTK_ALIGN_CENTER);

    gtk_widget_add_css_class(screensaver_figure_eight_picture, "robot-mode-popup-image");
    gtk_picture_set_content_fit(GTK_PICTURE(screensaver_figure_eight_picture),
                                GTK_CONTENT_FIT_CONTAIN);
    gtk_widget_set_hexpand(screensaver_figure_eight_picture, TRUE);
    gtk_widget_set_vexpand(screensaver_figure_eight_picture, TRUE);
    gtk_widget_set_halign(screensaver_figure_eight_picture, GTK_ALIGN_CENTER);
    gtk_widget_set_valign(screensaver_figure_eight_picture, GTK_ALIGN_CENTER);

    gtk_widget_add_css_class(screensaver_wave_picture, "robot-mode-popup-image");
    gtk_picture_set_content_fit(GTK_PICTURE(screensaver_wave_picture),
                                GTK_CONTENT_FIT_CONTAIN);
    gtk_widget_set_hexpand(screensaver_wave_picture, TRUE);
    gtk_widget_set_vexpand(screensaver_wave_picture, TRUE);
    gtk_widget_set_halign(screensaver_wave_picture, GTK_ALIGN_CENTER);
    gtk_widget_set_valign(screensaver_wave_picture, GTK_ALIGN_CENTER);

    gtk_stack_add_named(GTK_STACK(stack), joystick_picture, "joystick");
    gtk_stack_add_named(GTK_STACK(stack), screensaver_picture, "screensaver");
    gtk_stack_add_named(GTK_STACK(stack), screensaver_cobra_picture,
                        "screensaver-cobra");
    gtk_stack_add_named(GTK_STACK(stack), screensaver_conductor_picture,
                        "screensaver-conductor");
    gtk_stack_add_named(GTK_STACK(stack), screensaver_figure_eight_picture,
                        "screensaver-figure-eight");
    gtk_stack_add_named(GTK_STACK(stack), screensaver_wave_picture,
                        "screensaver-wave");
    gtk_box_append(GTK_BOX(box), stack);

    gtk_frame_set_child(GTK_FRAME(frame), box);
    ui->robot_mode_stack = stack;
    return frame;
}

/* Convert a display coordinate to a percentage of the camera image. */
static int display_coord_to_percent(double coord,
                                    int display_offset,
                                    int display_size)
{
    double normalized;

    if (display_size <= 0) {
        return 0;
    }

    normalized = ((coord - (double)display_offset) * 100.0) /
                 (double)display_size;
    normalized = clamp_double(normalized, 0.0, 100.0);
    return (int)lround(normalized);
}

/* Convert a horizontal ROI display coordinate to a percentage. */
static int roi_display_x_to_percent(const app_ui_t *ui, double x)
{
    int percent = display_coord_to_percent(x,
                                           ui->roi_editor.display_rect.x,
                                           ui->roi_editor.display_rect.width);

    return ui->controls.flip_video ? 100 - percent : percent;
}

/* Convert a percentage to a coordinate within the camera image. */
static int percent_to_display_coord(double percent,
                                    int display_offset,
                                    int display_size)
{
    if (display_size <= 0) {
        return display_offset;
    }

    return display_offset +
           (int)lround((percent / 100.0) * (double)display_size);
}

/* Synchronize the ROI editor with the latest input data. */
static void roi_editor_sync_from_input(app_ui_t *ui, const resource_info_t *info)
{
    if (info->width <= 0 || info->height <= 0) {
        return;
    }

    ui->roi_editor.input_roi = info->roi;
    ui->roi_editor.frame_width = info->width;
    ui->roi_editor.frame_height = info->height;
    if (!ui->roi_editor.initialized &&
        ui->roi_editor.dragged_edge == ROI_EDGE_NONE) {
        ui->roi_editor.adjustment_roi = info->roi;
    }
    ui->roi_editor.initialized = true;
}

/* Report whether a pointer coordinate is close enough to an ROI edge. */
static bool roi_edge_contains_pointer(int edge_coord,
                                      double pointer_coord,
                                      int axis_start,
                                      int axis_end,
                                      double orthogonal_coord)
{
    int min_axis = MIN(axis_start, axis_end) - ROI_HIT_RADIUS_PX;
    int max_axis = MAX(axis_start, axis_end) + ROI_HIT_RADIUS_PX;

    return fabs(pointer_coord - (double)edge_coord) <= ROI_HIT_RADIUS_PX &&
           orthogonal_coord >= (double)min_axis &&
           orthogonal_coord <= (double)max_axis;
}

/* Find the editable ROI edge under the pointer. */
static roi_edge_t roi_editor_hit_test(const app_ui_t *ui, double x, double y)
{
    const roi_editor_state_t *editor = &ui->roi_editor;
    int left;
    int right;
    int top;
    int bottom;
    double best_distance = 1e9;
    roi_edge_t best_edge = ROI_EDGE_NONE;

    if (!ui->controls.show_roi ||
        !editor->initialized ||
        editor->display_rect.width <= 0 ||
        editor->display_rect.height <= 0) {
        return ROI_EDGE_NONE;
    }

    if (x < editor->display_rect.x - ROI_HIT_RADIUS_PX ||
        x > editor->display_rect.x + editor->display_rect.width + ROI_HIT_RADIUS_PX ||
        y < editor->display_rect.y - ROI_HIT_RADIUS_PX ||
        y > editor->display_rect.y + editor->display_rect.height + ROI_HIT_RADIUS_PX) {
        return ROI_EDGE_NONE;
    }

    if (ui->controls.flip_video) {
        left = percent_to_display_coord(100 - editor->input_roi.end_x,
                                        editor->display_rect.x,
                                        editor->display_rect.width);
        right = percent_to_display_coord(100 - editor->input_roi.x,
                                         editor->display_rect.x,
                                         editor->display_rect.width);
    } else {
        left = percent_to_display_coord(editor->input_roi.x,
                                        editor->display_rect.x,
                                        editor->display_rect.width);
        right = percent_to_display_coord(editor->input_roi.end_x,
                                         editor->display_rect.x,
                                         editor->display_rect.width);
    }
    top = percent_to_display_coord(editor->input_roi.y,
                                   editor->display_rect.y,
                                   editor->display_rect.height);
    bottom = percent_to_display_coord(editor->input_roi.end_y,
                                      editor->display_rect.y,
                                      editor->display_rect.height);

    if (roi_edge_contains_pointer(left, x, top, bottom, y)) {
        best_distance = fabs(x - (double)left);
        best_edge = ui->controls.flip_video ? ROI_EDGE_RIGHT : ROI_EDGE_LEFT;
    }

    if (roi_edge_contains_pointer(right, x, top, bottom, y) &&
        fabs(x - (double)right) < best_distance) {
        best_distance = fabs(x - (double)right);
        best_edge = ui->controls.flip_video ? ROI_EDGE_LEFT : ROI_EDGE_RIGHT;
    }

    if (roi_edge_contains_pointer(top, y, left, right, x) &&
        fabs(y - (double)top) < best_distance) {
        best_distance = fabs(y - (double)top);
        best_edge = ROI_EDGE_TOP;
    }

    if (roi_edge_contains_pointer(bottom, y, left, right, x) &&
        fabs(y - (double)bottom) < best_distance) {
        best_edge = ROI_EDGE_BOTTOM;
    }

    return best_edge;
}

/* Refresh the active ROI edge from the current pointer position. */
static void roi_editor_refresh_active_edge(app_ui_t *ui)
{
    roi_editor_state_t *editor = &ui->roi_editor;

    if (editor->dragged_edge != ROI_EDGE_NONE) {
        editor->active_edge = editor->dragged_edge;
    } else {
        editor->adjustment_roi = editor->input_roi;
        editor->active_edge = roi_editor_hit_test(ui,
                                                  editor->pointer_x,
                                                  editor->pointer_y);
    }
}

/* Apply an ROI edge drag and publish the updated bounds. */
static void roi_editor_apply_drag(app_ui_t *ui, double offset_x, double offset_y)
{
    roi_editor_state_t *editor = &ui->roi_editor;
    int min_size_x;
    int min_size_y;

    if (editor->dragged_edge == ROI_EDGE_NONE ||
        editor->display_rect.width <= 0 ||
        editor->display_rect.height <= 0) {
        return;
    }

    min_size_x = MAX(1, (int)lround(100.0 / (double)editor->display_rect.width));
    min_size_y = MAX(1, (int)lround(100.0 / (double)editor->display_rect.height));

    switch (editor->dragged_edge) {
    case ROI_EDGE_LEFT: {
        int x = roi_display_x_to_percent(ui, editor->drag_start_x + offset_x);
        editor->adjustment_roi.x = clamp_int(
            x, 0, editor->drag_start_adjustment_roi.end_x - min_size_x);
        break;
    }
    case ROI_EDGE_RIGHT: {
        int end_x = roi_display_x_to_percent(ui,
                                             editor->drag_start_x + offset_x);
        editor->adjustment_roi.end_x = clamp_int(
            end_x, editor->drag_start_adjustment_roi.x + min_size_x, 100);
        break;
    }
    case ROI_EDGE_TOP: {
        int y = display_coord_to_percent(editor->drag_start_y + offset_y,
                                         editor->display_rect.y,
                                         editor->display_rect.height);
        editor->adjustment_roi.y = clamp_int(
            y, 0, editor->drag_start_adjustment_roi.end_y - min_size_y);
        break;
    }
    case ROI_EDGE_BOTTOM: {
        int end_y = display_coord_to_percent(editor->drag_start_y + offset_y,
                                             editor->display_rect.y,
                                             editor->display_rect.height);
        editor->adjustment_roi.end_y = clamp_int(
            end_y, editor->drag_start_adjustment_roi.y + min_size_y, 100);
        break;
    }
    case ROI_EDGE_NONE:
    default:
        break;
    }
}

/* Update ROI hover state as the pointer moves over the overlay. */
static void on_overlay_motion(GtkEventControllerMotion *controller,
                              double x,
                              double y,
                              gpointer user_data)
{
    app_ui_t *ui = user_data;

    (void)controller;
    ui->roi_editor.pointer_x = x;
    ui->roi_editor.pointer_y = y;
    roi_editor_refresh_active_edge(ui);
    queue_overlay_draw(ui->overlay);
}

/* Clear ROI hover state when the pointer leaves the overlay. */
static void on_overlay_leave(GtkEventControllerMotion *controller,
                             gpointer user_data)
{
    app_ui_t *ui = user_data;

    (void)controller;
    if (ui->roi_editor.dragged_edge == ROI_EDGE_NONE) {
        ui->roi_editor.active_edge = ROI_EDGE_NONE;
        queue_overlay_draw(ui->overlay);
    }
}

/* Begin dragging the ROI edge under the pointer. */
static void on_roi_drag_begin(GtkGestureDrag *gesture,
                              double start_x,
                              double start_y,
                              gpointer user_data)
{
    app_ui_t *ui = user_data;
    roi_editor_state_t *editor = &ui->roi_editor;

    (void)gesture;
    editor->pointer_x = start_x;
    editor->pointer_y = start_y;
    roi_editor_refresh_active_edge(ui);
    if (editor->active_edge == ROI_EDGE_NONE) {
        return;
    }

    editor->dragged_edge = editor->active_edge;
    editor->drag_start_x = start_x;
    editor->drag_start_y = start_y;
    editor->adjustment_roi = editor->input_roi;
    editor->drag_start_adjustment_roi = editor->adjustment_roi;
    queue_overlay_draw(ui->overlay);
}

/* Apply the current pointer offset to the active ROI edge. */
static void on_roi_drag_update(GtkGestureDrag *gesture,
                               double offset_x,
                               double offset_y,
                               gpointer user_data)
{
    app_ui_t *ui = user_data;

    (void)gesture;
    ui->roi_editor.pointer_x = ui->roi_editor.drag_start_x + offset_x;
    ui->roi_editor.pointer_y = ui->roi_editor.drag_start_y + offset_y;
    roi_editor_apply_drag(ui, offset_x, offset_y);
    queue_overlay_draw(ui->overlay);
}

/* Finish the active ROI drag operation. */
static void on_roi_drag_end(GtkGestureDrag *gesture,
                            double offset_x,
                            double offset_y,
                            gpointer user_data)
{
    app_ui_t *ui = user_data;
    bool set_ok = false;

    (void)gesture;
    if (ui->roi_editor.dragged_edge == ROI_EDGE_NONE) {
        return;
    }
    ui->roi_editor.pointer_x = ui->roi_editor.drag_start_x + offset_x;
    ui->roi_editor.pointer_y = ui->roi_editor.drag_start_y + offset_y;
    roi_editor_apply_drag(ui, offset_x, offset_y);
    set_ok = write_qpp_robot_roi(&ui->roi_editor.adjustment_roi);
    if (!set_ok) {
        puts("Failed to submit ROI adjustment");
    }
    ui->roi_editor.dragged_edge = ROI_EDGE_NONE;
    roi_editor_refresh_active_edge(ui);
    queue_overlay_draw(ui->overlay);
}

/* Choose an initial window size that fits the active monitor. */
static void set_initial_window_size(GtkWindow *window)
{
    GdkDisplay *display;
    GdkMonitor *monitor;
    GdkRectangle geometry;
    GListModel *monitors;
    guint monitor_count;
    int height = 540;
    int width = 960;

    display = gdk_display_get_default();
    if (display == NULL) {
        gtk_window_set_default_size(window, width, height);
        return;
    }

    monitors = gdk_display_get_monitors(display);
    monitor_count = g_list_model_get_n_items(monitors);
    if (monitor_count == 0) {
        gtk_window_set_default_size(window, width, height);
        return;
    }

    monitor = GDK_MONITOR(g_list_model_get_item(monitors, 0));
    if (monitor == NULL) {
        gtk_window_set_default_size(window, width, height);
        return;
    }

    gdk_monitor_get_geometry(monitor, &geometry);
    g_object_unref(monitor);

    width = MAX(geometry.width / 2, 960);
    height = MAX(geometry.height / 2, 540);
    gtk_window_set_default_size(window, width, height);
}

/* Apply the standard window flags used by the demo shell. */
static void configure_main_window(GtkWindow *window)
{
    gtk_window_set_resizable(window, TRUE);
    gtk_window_set_decorated(window, TRUE);
    gtk_window_set_default_size(window, 960, 540);
    gtk_widget_set_size_request(GTK_WIDGET(window), 640, 360);
}

/* Install the mode-specific CSS rules for the current session. */
static void install_app_css(void)
{
    GtkCssProvider *provider = gtk_css_provider_new();

    gtk_css_provider_load_from_resource(
        provider, "/com/qnx/physical-ai-ui/ui/style.css");
    gtk_style_context_add_provider_for_display(
        gdk_display_get_default(),
        GTK_STYLE_PROVIDER(provider),
        GTK_STYLE_PROVIDER_PRIORITY_USER);
    g_object_unref(provider);
}

/* Toggle the QR popup when the header QR code is pressed. */
static void on_header_qr_pressed(GtkGestureClick *gesture,
                                 int n_press,
                                 double x,
                                 double y,
                                 gpointer user_data)
{
    app_ui_t *ui = user_data;

    (void)gesture;
    (void)n_press;
    (void)x;
    (void)y;
    toggle_qr_popup(ui);
}

/* Return the height of the monitor containing the widget. */
static int get_monitor_height_for_widget(GtkWidget *widget)
{
    GtkNative *native;
    GdkSurface *surface;
    GdkDisplay *display;
    GdkMonitor *monitor = NULL;
    GListModel *monitors;
    GdkRectangle geometry;

    if (widget == NULL) {
        return 1080;
    }

    native = gtk_widget_get_native(widget);
    if (native != NULL) {
        surface = gtk_native_get_surface(native);
        if (surface != NULL) {
            display = gdk_surface_get_display(surface);
            if (display != NULL) {
                monitor = gdk_display_get_monitor_at_surface(display, surface);
            }
        }
    }

    if (monitor != NULL) {
        gdk_monitor_get_geometry(monitor, &geometry);
        return geometry.height;
    }

    display = gdk_display_get_default();
    if (display == NULL) {
        return 1080;
    }

    monitors = gdk_display_get_monitors(display);
    if (monitors != NULL && g_list_model_get_n_items(monitors) > 0) {
        monitor = GDK_MONITOR(g_list_model_get_item(monitors, 0));
        if (monitor != NULL) {
            gdk_monitor_get_geometry(monitor, &geometry);
            g_object_unref(monitor);
            return geometry.height;
        }
    }

    return 1080;
}

/* Scale the header branding for the available width. */
static void update_topbar_brand_sizing(app_ui_t *ui)
{
    const int topbar_vertical_padding_px = 34;
    int monitor_height;
    int topbar_max_height;
    int brand_height;
    int qr_size;

    if (ui == NULL) {
        return;
    }

    monitor_height = get_monitor_height_for_widget(ui->window);
    topbar_max_height = MAX((int)(monitor_height * 0.10), 56);
    brand_height = MAX(topbar_max_height - topbar_vertical_padding_px, 28);
    qr_size = CLAMP(brand_height, 28, 72);

    if (ui->topbar_logo != NULL) {
        gtk_widget_set_size_request(ui->topbar_logo, -1, brand_height);
    }
    if (ui->topbar_qr != NULL) {
        gtk_widget_set_size_request(ui->topbar_qr, qr_size, qr_size);
    }
}

/* Create the QR code shown in the application header. */
static GtkWidget *create_header_qr_code(app_ui_t *ui)
{
    GtkWidget *picture =
        gtk_picture_new_for_resource("/com/qnx/physical-ai-ui/ui/qr-code-github.png");
    GtkGesture *click_gesture = gtk_gesture_click_new();

    gtk_widget_add_css_class(picture, "header-qr-image");
    gtk_picture_set_content_fit(GTK_PICTURE(picture), GTK_CONTENT_FIT_CONTAIN);
    g_object_set(G_OBJECT(picture), "can-shrink", TRUE, NULL);
    gtk_widget_set_halign(picture, GTK_ALIGN_END);
    gtk_widget_set_valign(picture, GTK_ALIGN_CENTER);
    gtk_widget_set_cursor_from_name(picture, "pointer");

    gtk_gesture_single_set_button(GTK_GESTURE_SINGLE(click_gesture),
                                  GDK_BUTTON_PRIMARY);
    gtk_widget_add_controller(picture, GTK_EVENT_CONTROLLER(click_gesture));
    g_signal_connect(click_gesture, "pressed",
                     G_CALLBACK(on_header_qr_pressed), ui);

    return picture;
}

/* Keep the side controls sized to the current window. */
static gboolean update_controls_width_tick(GtkWidget *widget,
                                           GdkFrameClock *frame_clock,
                                           gpointer user_data)
{
    app_ui_t *ui = user_data;
    int content_width;
    int target_width;

    (void)widget;
    (void)frame_clock;

    if (ui->content_box == NULL || ui->controls_box == NULL) {
        return G_SOURCE_CONTINUE;
    }

    content_width = gtk_widget_get_width(ui->content_box);
    if (content_width <= 0) {
        return G_SOURCE_CONTINUE;
    }

    target_width = MAX(1, content_width / 4);
    if (target_width == ui->controls_width_px) {
        return G_SOURCE_CONTINUE;
    }

    ui->controls_width_px = target_width;
    gtk_widget_set_size_request(ui->controls_box, target_width, -1);
    gtk_scrolled_window_set_min_content_width(
        GTK_SCROLLED_WINDOW(ui->controls_box), 0);
    gtk_scrolled_window_set_max_content_width(
        GTK_SCROLLED_WINDOW(ui->controls_box), target_width);
    return G_SOURCE_CONTINUE;
}

/* Refresh header branding during a frame tick. */
static gboolean update_topbar_brand_tick(GtkWidget *widget,
                                         GdkFrameClock *frame_clock,
                                         gpointer user_data)
{
    app_ui_t *ui = user_data;

    (void)widget;
    (void)frame_clock;

    update_topbar_brand_sizing(ui);
    return G_SOURCE_CONTINUE;
}

/* Refresh robot-mode popup state during a frame tick. */
static gboolean sync_robot_mode_popup_tick(GtkWidget *widget,
                                           GdkFrameClock *frame_clock,
                                           gpointer user_data)
{
    app_ui_t *ui = user_data;
    resource_info_t info;

    (void)widget;
    (void)frame_clock;

    pose_data_get_info(&ui->pose_data, &info);
    update_robot_mode_popup(ui, info.robot_mode, info.screensaver_mode);
    return G_SOURCE_CONTINUE;
}

/* Handle application keyboard shortcuts. */
static gboolean handle_key(GtkEventControllerKey *self, guint keyval,
                           guint keycode, GdkModifierType state,
                           gpointer user_data)
{
    app_ui_t *ui = user_data;
    GtkWindow *window = GTK_WINDOW(ui->window);
    GdkModifierType modifiers = state & gtk_accelerator_get_default_mod_mask();

    (void)self;
    (void)keycode;

    if (modifiers != 0 && modifiers != GDK_SHIFT_MASK) {
        return FALSE;
    }

    keyval = gdk_keyval_to_lower(keyval);

    switch (keyval) {
    case GDK_KEY_c:
        toggle_control_studio(ui);
        set_hotkeys_popup_visible(ui, FALSE);
        set_qr_popup_visible(ui, FALSE);
        break;
    case GDK_KEY_d:
        if (ui->controls.show_debug_info) {
            ui->controls.show_debug_info = FALSE;
            if (pose_popups_available_for_robot_mode(ui->current_robot_mode)) {
                ui->controls.show_joint_popups = TRUE;
            }
        } else {
            ui->controls.show_debug_info = TRUE;
            if (joint_popups_visible_for_ui(ui)) {
                ui->controls.show_joint_popups = FALSE;
            }
        }
        queue_overlay_draw_for_ui(ui);
        set_hotkeys_popup_visible(ui, FALSE);
        set_qr_popup_visible(ui, FALSE);
        break;
    case GDK_KEY_v:
        ui->controls.show_overlay_fills = !ui->controls.show_overlay_fills;
        queue_overlay_draw_for_ui(ui);
        set_hotkeys_popup_visible(ui, FALSE);
        set_qr_popup_visible(ui, FALSE);
        break;
    case GDK_KEY_r:
        ui->controls.show_roi = !ui->controls.show_roi;
        sync_switch_state(ui->controls.roi_toggle, ui->controls.show_roi);
        queue_overlay_draw_for_ui(ui);
        set_hotkeys_popup_visible(ui, FALSE);
        set_qr_popup_visible(ui, FALSE);
        break;
    case GDK_KEY_s:
        ui->controls.show_skeleton = !ui->controls.show_skeleton;
        sync_switch_state(ui->controls.skeleton_toggle,
                          ui->controls.show_skeleton);
        queue_overlay_draw_for_ui(ui);
        set_hotkeys_popup_visible(ui, FALSE);
        set_qr_popup_visible(ui, FALSE);
        break;
    case GDK_KEY_j:
        ui->controls.show_joints = !ui->controls.show_joints;
        sync_switch_state(ui->controls.joints_toggle, ui->controls.show_joints);
        queue_overlay_draw_for_ui(ui);
        set_hotkeys_popup_visible(ui, FALSE);
        set_qr_popup_visible(ui, FALSE);
        break;
    case GDK_KEY_l:
        ui->controls.filter_low_confidence = !ui->controls.filter_low_confidence;
        sync_switch_state(ui->controls.confidence_toggle,
                          ui->controls.filter_low_confidence);
        queue_overlay_draw_for_ui(ui);
        set_hotkeys_popup_visible(ui, FALSE);
        set_qr_popup_visible(ui, FALSE);
        break;
    case GDK_KEY_bracketleft:
        adjust_confidence_threshold(ui, -CONFIDENCE_HOTKEY_STEP);
        queue_overlay_draw_for_ui(ui);
        set_hotkeys_popup_visible(ui, FALSE);
        set_qr_popup_visible(ui, FALSE);
        break;
    case GDK_KEY_bracketright:
        adjust_confidence_threshold(ui, CONFIDENCE_HOTKEY_STEP);
        queue_overlay_draw_for_ui(ui);
        set_hotkeys_popup_visible(ui, FALSE);
        set_qr_popup_visible(ui, FALSE);
        break;
    case GDK_KEY_e:
    case GDK_KEY_k:
        toggle_robot_enabled(ui);
        set_hotkeys_popup_visible(ui, FALSE);
        set_qr_popup_visible(ui, FALSE);
        break;
    case GDK_KEY_1:
        move_debug_info(ui, DEBUG_INFO_TOP_LEFT);
        set_hotkeys_popup_visible(ui, FALSE);
        set_qr_popup_visible(ui, FALSE);
        break;
    case GDK_KEY_2:
        move_debug_info(ui, DEBUG_INFO_TOP_RIGHT);
        set_hotkeys_popup_visible(ui, FALSE);
        set_qr_popup_visible(ui, FALSE);
        break;
    case GDK_KEY_3:
        move_debug_info(ui, DEBUG_INFO_BOTTOM_LEFT);
        set_hotkeys_popup_visible(ui, FALSE);
        set_qr_popup_visible(ui, FALSE);
        break;
    case GDK_KEY_4:
        move_debug_info(ui, DEBUG_INFO_BOTTOM_RIGHT);
        set_hotkeys_popup_visible(ui, FALSE);
        set_qr_popup_visible(ui, FALSE);
        break;
    case GDK_KEY_p:
        if (!pose_popups_available_for_robot_mode(ui->current_robot_mode)) {
            return FALSE;
        }
        ui->controls.show_joint_popups = !ui->controls.show_joint_popups;
        queue_overlay_draw_for_ui(ui);
        set_hotkeys_popup_visible(ui, FALSE);
        set_qr_popup_visible(ui, FALSE);
        break;
    case GDK_KEY_h:
        toggle_hotkeys_popup(ui);
        break;
    case GDK_KEY_q:
        toggle_qr_popup(ui);
        break;
    case GDK_KEY_f:
        if (*ui->fullscreen) {
            gtk_window_unfullscreen(window);
        } else {
            gtk_window_fullscreen(window);
        }
        *ui->fullscreen = !*ui->fullscreen;
        sync_window_mode_layout(ui);
        set_hotkeys_popup_visible(ui, FALSE);
        set_qr_popup_visible(ui, FALSE);
        break;
    case GDK_KEY_Escape:
        g_application_quit(G_APPLICATION(gtk_window_get_application(window)));
        break;
    default:
        return FALSE;
    }

    return TRUE;
}

/* Render the current pose and control overlays. */
static void draw_overlay_cb(GtkDrawingArea *area, cairo_t *cr,
                            int width, int height, gpointer user_data)
{
    app_ui_t *ui = user_data;
    resource_info_t info;

    (void)area;
    pose_data_get_info(&ui->pose_data, &info);
    roi_editor_sync_from_input(ui, &info);
    ui->roi_editor.display_rect =
        overlay_renderer_get_camera_display_rect(info.width, info.height,
                                                 width, height);
    overlay_controls_set_robot_mode(&ui->controls, info.robot_mode,
                                    info.screensaver_mode);
    if (ui->roi_hint_chip != NULL) {
        gtk_widget_set_visible(ui->roi_hint_chip, ui->controls.show_roi);
    }
    overlay_renderer_draw(cr, width, height, &info,
                          &ui->controls,
                          ui->roi_editor.active_edge != ROI_EDGE_NONE ?
                              &ui->roi_editor.adjustment_roi : NULL,
                          ui->roi_editor.active_edge,
                          ui->roi_editor.dragged_edge,
                          APP_VERSION,
                          ui->app_start_us);
}

/* Construct the full application window and wire callbacks to the runtime. */
bool app_ui_init(app_ui_t *ui, GtkApplication *app,
                 bool show_video_flip,
                 bool *fullscreen, gint64 app_start_us)
{
    bool saved_video_flip;
    GtkEventController *controller;
    GtkEventController *motion_controller;
    GtkGesture *drag_gesture;
    GtkWidget *app_shell;
    GtkWidget *brand_box;
    GtkWidget *camera_card;
    GtkWidget *camera_brand;
    GtkWidget *camera_brand_logo;
    GtkWidget *camera_frame;
    GtkWidget *camera_header;
    GtkWidget *camera_header_copy;
    GtkWidget *camera_header_actions;
    GtkWidget *content_box;
    GtkWidget *controls_panel;
    GtkWidget *controls_scroller;
    GtkWidget *controls_box;
    GtkWidget *header_box;
    GtkWidget *header_logo;
    GtkWidget *root_overlay;
    GtkWidget *overlay;
    GtkWidget *title_label;
    GtkWidget *subtitle_label;
    GtkWidget *subtitle_secondary_label;

    if (ui == NULL || app == NULL || fullscreen == NULL) {
        return false;
    }

    *ui = (app_ui_t){0};
    overlay_controls_state_init(&ui->controls);
    if (user_settings_load_video_flip(&saved_video_flip)) {
        ui->controls.flip_video = saved_video_flip;
    }
    ui->controls.show_video_flip = show_video_flip;
    ui->controls.video_flip_changed_cb = on_video_flip_changed;
    ui->controls.video_flip_changed_user_data = ui;
    ui->controls.robot_control_toggled_cb = on_robot_control_toggled;
    ui->controls.robot_control_toggled_user_data = ui;
    ui->controls.wrist_control_toggled_cb = on_wrist_control_toggled;
    ui->controls.wrist_control_toggled_user_data = ui;
    ui->controls.session_card_visible = true;
    ui->controls.roi_available = true;
    ui->controls.robot_control_available = true;
    ui->controls.show_roi = true;
    ui->controls.enable_robot_control = true;
    if (!read_qpp_boolean(PHYSICAL_QPP_ROBOT_ENABLED_PATH,
                          &ui->controls.enable_robot_control)) {
        printf("Using default robot enabled state because QPP state is unavailable\n");
    }
    if (!read_qpp_boolean(PHYSICAL_QPP_WRIST_CONTROL_PATH,
                          &ui->controls.enable_wrist_control)) {
        printf("Using default wrist control state because QPP state is unavailable\n");
    }
    ui->controls.show_controls = false;
    pose_data_init(&ui->pose_data);

    ui->app_start_us = app_start_us;
    ui->controls_width_px = 0;
    ui->current_robot_mode[0] = '\0';
    ui->fullscreen = fullscreen;

    ui->window = gtk_application_window_new(app);
    gtk_window_set_title(GTK_WINDOW(ui->window), APP_HEADER_TITLE);
    configure_main_window(GTK_WINDOW(ui->window));
    set_initial_window_size(GTK_WINDOW(ui->window));
    install_app_css();
    g_object_set(gtk_settings_get_default(),
                 "gtk-font-name",
                 "Sans 11",
                 NULL);

    controller = gtk_event_controller_key_new();
    gtk_event_controller_set_propagation_phase(controller,
                                               GTK_PHASE_CAPTURE);
    gtk_widget_add_controller(GTK_WIDGET(ui->window), controller);
    g_signal_connect(controller, "key-pressed",
                     G_CALLBACK(handle_key), ui);

    overlay = gtk_overlay_new();
    gtk_widget_set_hexpand(overlay, TRUE);
    gtk_widget_set_vexpand(overlay, TRUE);
    ui->picture = gtk_picture_new();
    if (ui->controls.flip_video) {
        gtk_widget_add_css_class(ui->picture, "video-mirrored");
    }
    gtk_widget_set_hexpand(ui->picture, TRUE);
    gtk_widget_set_vexpand(ui->picture, TRUE);
    gtk_overlay_set_child(GTK_OVERLAY(overlay), ui->picture);

    ui->overlay = gtk_drawing_area_new();
    gtk_widget_set_halign(ui->overlay, GTK_ALIGN_FILL);
    gtk_widget_set_valign(ui->overlay, GTK_ALIGN_FILL);
    gtk_drawing_area_set_draw_func(GTK_DRAWING_AREA(ui->overlay),
                                   draw_overlay_cb, ui, NULL);
    motion_controller = gtk_event_controller_motion_new();
    gtk_widget_add_controller(ui->overlay, motion_controller);
    g_signal_connect(motion_controller, "motion",
                     G_CALLBACK(on_overlay_motion), ui);
    g_signal_connect(motion_controller, "leave",
                     G_CALLBACK(on_overlay_leave), ui);

    drag_gesture = gtk_gesture_drag_new();
    gtk_gesture_single_set_button(GTK_GESTURE_SINGLE(drag_gesture),
                                  GDK_BUTTON_PRIMARY);
    gtk_widget_add_controller(ui->overlay, GTK_EVENT_CONTROLLER(drag_gesture));
    g_signal_connect(drag_gesture, "drag-begin",
                     G_CALLBACK(on_roi_drag_begin), ui);
    g_signal_connect(drag_gesture, "drag-update",
                     G_CALLBACK(on_roi_drag_update), ui);
    g_signal_connect(drag_gesture, "drag-end",
                     G_CALLBACK(on_roi_drag_end), ui);
    gtk_overlay_add_overlay(GTK_OVERLAY(overlay), ui->overlay);
    ui->camera_error = gtk_label_new(NULL);
    gtk_widget_set_halign(ui->camera_error, GTK_ALIGN_CENTER);
    gtk_widget_set_valign(ui->camera_error, GTK_ALIGN_CENTER);
    gtk_label_set_justify(GTK_LABEL(ui->camera_error), GTK_JUSTIFY_CENTER);
    gtk_label_set_wrap(GTK_LABEL(ui->camera_error), TRUE);
    gtk_widget_set_can_target(ui->camera_error, FALSE);
    gtk_widget_set_visible(ui->camera_error, FALSE);
    gtk_overlay_add_overlay(GTK_OVERLAY(overlay), ui->camera_error);
    ui->robot_mode_stack = NULL;
    ui->robot_mode_popup = create_robot_mode_popup(ui);
    gtk_overlay_add_overlay(GTK_OVERLAY(overlay), ui->robot_mode_popup);
    ui->qr_popup = create_qr_popup(ui);
    if (ui->qr_popup != NULL) {
        gtk_overlay_add_overlay(GTK_OVERLAY(overlay), ui->qr_popup);
    }

    controls_panel = overlay_controls_create(ui->overlay, &ui->controls);
    gtk_widget_set_valign(controls_panel, GTK_ALIGN_START);

    controls_scroller = gtk_scrolled_window_new();
    gtk_widget_add_css_class(controls_scroller, "controls-scroller");
    gtk_widget_set_hexpand(controls_scroller, FALSE);
    gtk_widget_set_halign(controls_scroller, GTK_ALIGN_START);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(controls_scroller),
                                   GTK_POLICY_NEVER,
                                   GTK_POLICY_AUTOMATIC);
    gtk_widget_set_vexpand(controls_scroller, TRUE);
    gtk_scrolled_window_set_child(GTK_SCROLLED_WINDOW(controls_scroller),
                                  controls_panel);

    controls_box = controls_scroller;
    ui->controls_box = controls_box;
    gtk_widget_set_vexpand(ui->controls_box, TRUE);

    app_shell = gtk_box_new(GTK_ORIENTATION_VERTICAL, 20);
    gtk_widget_add_css_class(app_shell, "app-shell");
    gtk_widget_set_hexpand(app_shell, TRUE);
    gtk_widget_set_vexpand(app_shell, TRUE);

    header_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 16);
    ui->topbar = header_box;
    gtk_widget_add_css_class(header_box, "app-topbar");
    brand_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 24);
    gtk_widget_set_hexpand(brand_box, TRUE);
    header_logo = gtk_picture_new_for_resource("/com/qnx/physical-ai-ui/ui/QNXEverywhereLogo.png");
    ui->topbar_logo = header_logo;
    gtk_widget_add_css_class(header_logo, "brand-logo");
    gtk_picture_set_content_fit(GTK_PICTURE(header_logo),
                                GTK_CONTENT_FIT_CONTAIN);
    g_object_set(G_OBJECT(header_logo), "can-shrink", TRUE, NULL);
    gtk_widget_set_halign(header_logo, GTK_ALIGN_START);
    gtk_widget_set_valign(header_logo, GTK_ALIGN_CENTER);
    gtk_box_append(GTK_BOX(brand_box), header_logo);
    gtk_box_append(GTK_BOX(header_box), brand_box);

    camera_header_actions = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    gtk_widget_set_halign(camera_header_actions, GTK_ALIGN_END);
    ui->topbar_qr = create_header_qr_code(ui);
    gtk_box_append(GTK_BOX(camera_header_actions), ui->topbar_qr);
    gtk_box_append(GTK_BOX(header_box), camera_header_actions);

    content_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 20);
    ui->content_box = content_box;
    gtk_widget_set_hexpand(content_box, TRUE);
    gtk_widget_set_vexpand(content_box, TRUE);

    camera_card = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    ui->camera_card = camera_card;
    gtk_widget_add_css_class(camera_card, "camera-card");
    gtk_widget_set_hexpand(camera_card, TRUE);
    gtk_widget_set_vexpand(camera_card, TRUE);

    camera_header = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 12);
    ui->camera_header = camera_header;
    gtk_widget_add_css_class(camera_header, "camera-header");
    gtk_widget_set_valign(camera_header, GTK_ALIGN_CENTER);
    camera_brand = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
    gtk_widget_add_css_class(camera_brand, "camera-brand");
    gtk_widget_set_valign(camera_brand, GTK_ALIGN_CENTER);
    camera_brand_logo = gtk_picture_new_for_resource("/com/qnx/physical-ai-ui/ui/QNXLogo.png");
    gtk_widget_add_css_class(camera_brand_logo, "camera-brand-logo");
    gtk_picture_set_content_fit(GTK_PICTURE(camera_brand_logo),
                                GTK_CONTENT_FIT_CONTAIN);
    gtk_widget_set_halign(camera_brand_logo, GTK_ALIGN_START);
    gtk_widget_set_valign(camera_brand_logo, GTK_ALIGN_CENTER);
    gtk_box_append(GTK_BOX(camera_brand), camera_brand_logo);
    ui->camera_brand = camera_brand;
    gtk_box_append(GTK_BOX(camera_header), camera_brand);
    camera_header_copy = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
    gtk_widget_set_halign(camera_header_copy, GTK_ALIGN_START);
    gtk_widget_set_valign(camera_header_copy, GTK_ALIGN_CENTER);
    title_label = gtk_label_new(APP_HEADER_TITLE);
    ui->camera_title = title_label;
    gtk_widget_set_halign(title_label, GTK_ALIGN_START);
    gtk_widget_set_valign(title_label, GTK_ALIGN_CENTER);
    gtk_widget_add_css_class(title_label, "camera-title");
    subtitle_label = gtk_label_new(APP_HEADER_SUBTITLE);
    gtk_widget_set_halign(subtitle_label, GTK_ALIGN_START);
    gtk_widget_add_css_class(subtitle_label, "camera-subtitle");
    gtk_label_set_wrap(GTK_LABEL(subtitle_label), TRUE);
    ui->camera_subtitle = subtitle_label;
    subtitle_secondary_label = gtk_label_new(APP_HEADER_SUBTITLE_SECONDARY);
    gtk_widget_set_halign(subtitle_secondary_label, GTK_ALIGN_START);
    gtk_widget_add_css_class(subtitle_secondary_label, "camera-subtitle");
    gtk_label_set_wrap(GTK_LABEL(subtitle_secondary_label), TRUE);
    ui->camera_subtitle_secondary = subtitle_secondary_label;
    gtk_box_append(GTK_BOX(camera_header_copy), title_label);
    gtk_box_append(GTK_BOX(camera_header_copy), subtitle_label);
    gtk_box_append(GTK_BOX(camera_header_copy), subtitle_secondary_label);
    gtk_box_append(GTK_BOX(camera_header), camera_header_copy);
    ui->roi_hint_chip = NULL;
    gtk_box_append(GTK_BOX(camera_card), camera_header);

    camera_frame = gtk_frame_new(NULL);
    gtk_widget_add_css_class(camera_frame, "camera-frame");
    gtk_widget_set_hexpand(camera_frame, TRUE);
    gtk_widget_set_vexpand(camera_frame, TRUE);
    gtk_frame_set_child(GTK_FRAME(camera_frame), overlay);
    gtk_box_append(GTK_BOX(camera_card), camera_frame);

    gtk_box_append(GTK_BOX(content_box), camera_card);
    gtk_box_append(GTK_BOX(content_box), ui->controls_box);

    gtk_box_append(GTK_BOX(app_shell), header_box);
    gtk_box_append(GTK_BOX(app_shell), content_box);

    root_overlay = gtk_overlay_new();
    gtk_overlay_set_child(GTK_OVERLAY(root_overlay), app_shell);
    ui->hotkeys_popup = create_hotkeys_popup();
    gtk_overlay_add_overlay(GTK_OVERLAY(root_overlay), ui->hotkeys_popup);

    gtk_window_set_child(GTK_WINDOW(ui->window), root_overlay);
    gtk_widget_add_tick_callback(app_shell, update_controls_width_tick, ui, NULL);
    gtk_widget_add_tick_callback(app_shell, update_topbar_brand_tick, ui, NULL);
    gtk_widget_add_tick_callback(app_shell, sync_robot_mode_popup_tick, ui, NULL);

    if (*ui->fullscreen) {
        gtk_window_fullscreen(GTK_WINDOW(ui->window));
    }
    sync_side_studio_visibility(ui);
    update_topbar_brand_sizing(ui);
    sync_window_mode_layout(ui);

    if (!pose_data_start(&ui->pose_data, ui->overlay)) {
        return false;
    }

    return true;
}

/* Show the already-built top-level window. */
void app_ui_present(app_ui_t *ui)
{
    gtk_window_present(GTK_WINDOW(ui->window));
    gtk_widget_grab_focus(ui->window);
}

/* Stop background readers and destroy the top-level window. */
void app_ui_shutdown(app_ui_t *ui)
{
    pose_data_stop(&ui->pose_data);
}

/* Return the widget that receives camera textures from the pipeline. */
GtkWidget *app_ui_get_picture(const app_ui_t *ui)
{
    return ui->picture;
}

/* Return the drawing area used for the pose and ROI overlay. */
GtkWidget *app_ui_get_overlay(const app_ui_t *ui)
{
    return ui->overlay;
}
