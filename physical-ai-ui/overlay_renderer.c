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

#include "overlay_renderer.h"

#include "app_config.h"
#include "app_theme.h"

#include <gdk-pixbuf/gdk-pixbuf.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Add a rounded rectangle path without filling or stroking it yet. */
static void rounded_rect(cairo_t *cr,
                         double x,
                         double y,
                         double width,
                         double height,
                         double radius)
{
    double r = radius;

    if (width <= 0.0 || height <= 0.0) {
        return;
    }

    if (r > width / 2.0) {
        r = width / 2.0;
    }
    if (r > height / 2.0) {
        r = height / 2.0;
    }

    cairo_new_sub_path(cr);
    cairo_arc(cr, x + width - r, y + r, r, -G_PI / 2.0, 0.0);
    cairo_arc(cr, x + width - r, y + height - r, r, 0.0, G_PI / 2.0);
    cairo_arc(cr, x + r, y + height - r, r, G_PI / 2.0, G_PI);
    cairo_arc(cr, x + r, y + r, r, G_PI, 3.0 * G_PI / 2.0);
    cairo_close_path(cr);
}

/* Draw the frosted panel used behind debug and status overlays. */
static void draw_glass_panel(cairo_t *cr,
                             const app_theme_t *theme,
                             double x,
                             double y,
                             double width,
                             double height,
                             double radius)
{
    rounded_rect(cr, x + 2.0, y + 8.0, width, height, radius);
    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 0.20);
    cairo_fill(cr);

    rounded_rect(cr, x, y, width, height, radius);
    cairo_set_source_rgba(cr, theme->glass_panel_fill[0],
                          theme->glass_panel_fill[1],
                          theme->glass_panel_fill[2],
                          theme->glass_panel_fill[3]);
    cairo_fill_preserve(cr);
    cairo_set_source_rgba(cr, theme->glass_panel_stroke[0],
                          theme->glass_panel_stroke[1],
                          theme->glass_panel_stroke[2],
                          theme->glass_panel_stroke[3]);
    cairo_set_line_width(cr, 1.2);
    cairo_stroke(cr);
}

/* Choose a skeleton edge color. */
static void set_skeleton_edge_color(cairo_t *cr,
                                    const app_theme_t *theme,
                                    const resource_info_t *info,
                                    const pose_info_t *pose)
{
    if (strcmp(info->selection_state, "selected") == 0 &&
        pose->id == info->selected_pose_id) {
        cairo_set_source_rgb(cr, theme->pose_selected[0],
                             theme->pose_selected[1],
                             theme->pose_selected[2]);
        return;
    }

    cairo_set_source_rgb(cr, theme->pose_joint_confidence_low[0],
                         theme->pose_joint_confidence_low[1],
                         theme->pose_joint_confidence_low[2]);
}

/* Calculate the camera image rectangle within the overlay. */
camera_display_rect_t overlay_renderer_get_camera_display_rect(
    int src_width,
    int src_height,
    int dst_width,
    int dst_height)
{
    camera_display_rect_t rect = { 0, 0, 0, 0 };
    double src_aspect;
    double dst_aspect;

    if (src_width <= 0 || src_height <= 0 || dst_width <= 0 || dst_height <= 0) {
        return rect;
    }

    src_aspect = (double)src_width / (double)src_height;
    dst_aspect = (double)dst_width / (double)dst_height;

    if (dst_aspect > src_aspect) {
        rect.height = dst_height;
        rect.width = (int)lround((double)rect.height * src_aspect);
        rect.x = (dst_width - rect.width) / 2;
        rect.y = 0;
    } else {
        rect.width = dst_width;
        rect.height = (int)lround((double)rect.width / src_aspect);
        rect.x = 0;
        rect.y = (dst_height - rect.height) / 2;
    }

    if (rect.width < 1) {
        rect.width = 1;
    }
    if (rect.height < 1) {
        rect.height = 1;
    }

    return rect;
}

/* Translate a camera-space point into display coordinates. */
static void translate_point(int x, int y,
                            int src_w, int src_h,
                            const camera_display_rect_t *display_rect,
                            int *tx, int *ty)
{
    float sx;
    float sy;

    if (src_w == 0 || src_h == 0 || display_rect == NULL ||
        display_rect->width <= 0 || display_rect->height <= 0) {
        *tx = 0;
        *ty = 0;
        return;
    }

    sx = (float)display_rect->width / (float)src_w;
    sy = (float)display_rect->height / (float)src_h;

    *tx = display_rect->x + (int)lround((double)x * sx);
    *ty = display_rect->y + (int)lround((double)y * sy);
}

/* Translate an ROI percentage point into display coordinates. */
static void translate_roi_point(int x_percent, int y_percent,
                                const camera_display_rect_t *display_rect,
                                int *tx, int *ty)
{
    if (display_rect == NULL || display_rect->width <= 0 ||
        display_rect->height <= 0) {
        *tx = 0;
        *ty = 0;
        return;
    }

    *tx = display_rect->x +
          (int)lround((x_percent / 100.0) * (double)display_rect->width);
    *ty = display_rect->y +
          (int)lround((y_percent / 100.0) * (double)display_rect->height);
}

/* Translate one detected person into display coordinates. */
static void translate_person(person_info_t *person,
                             int src_w, int src_h,
                             const camera_display_rect_t *display_rect)
{
    int br_x;
    int br_y;

    translate_point(person->x, person->y,
                    src_w, src_h,
                    display_rect,
                    &person->tx, &person->ty);
    translate_point(person->x + person->width, person->y + person->height,
                    src_w, src_h,
                    display_rect,
                    &br_x, &br_y);

    person->t_width = br_x - person->tx;
    person->t_height = br_y - person->ty;
    if (person->t_width < 1) {
        person->t_width = 1;
    }
    if (person->t_height < 1) {
        person->t_height = 1;
    }
}

/* Translate all input overlays into display coordinates. */
static void translate_all(resource_info_t *info,
                          const camera_display_rect_t *display_rect)
{
    size_t i;
    size_t j;

    translate_roi_point(info->roi.x, info->roi.y,
                        display_rect,
                        &info->roi.tx, &info->roi.ty);
    translate_roi_point(info->roi.end_x, info->roi.end_y,
                        display_rect,
                        &info->roi.t_end_x, &info->roi.t_end_y);

    for (i = 0; i < info->pose_count; i++) {
        pose_info_t *pose = &info->poses[i];

        translate_person(&pose->person,
                         info->width, info->height,
                         display_rect);

        for (j = 0; j < pose->joint_count; j++) {
            joint_info_t *joint = &pose->joints[j];

            translate_point(joint->x, joint->y,
                            info->width, info->height,
                            display_rect,
                            &joint->tx, &joint->ty);
        }
    }

}

/* Reflect display coordinates only, keeping source coordinates and identities
 * intact. Use the widget center to match the camera picture's CSS transform,
 * including when the fitted camera rectangle has an odd letterbox margin.
 */
static void mirror_pose(pose_info_t *pose, int width)
{
    size_t j;

    pose->person.tx = width - pose->person.tx - pose->person.t_width;
    for (j = 0; j < pose->joint_count; j++) {
        pose->joints[j].tx = width - pose->joints[j].tx;
    }
}

/* Mirror all pose overlay coordinates horizontally. */
static void mirror_pose_overlays(resource_info_t *info, int width)
{
    size_t i;
    for (i = 0; i < info->pose_count; i++) {
        mirror_pose(&info->poses[i], width);
    }
}

/* Mirror the ROI bounds horizontally. */
static void mirror_roi_display(roi_info_t *roi, int width)
{
    int mirrored_start = width - roi->t_end_x;

    roi->t_end_x = width - roi->tx;
    roi->tx = mirrored_start;
}

/* Return the edge corresponding to a horizontal mirror. */
static roi_edge_t mirror_roi_edge(roi_edge_t edge)
{
    if (edge == ROI_EDGE_LEFT) {
        return ROI_EDGE_RIGHT;
    }
    if (edge == ROI_EDGE_RIGHT) {
        return ROI_EDGE_LEFT;
    }
    return edge;
}

/* Translate ROI percentages into display coordinates. */
static void translate_roi(roi_info_t *roi,
                          const camera_display_rect_t *display_rect)
{
    translate_roi_point(roi->x, roi->y, display_rect, &roi->tx, &roi->ty);
    translate_roi_point(roi->end_x, roi->end_y, display_rect,
                        &roi->t_end_x, &roi->t_end_y);
}

/* Choose the overlay color for a detected person. */
static void get_person_overlay_color(const resource_info_t *info,
                                     const app_theme_t *theme,
                                     const pose_info_t *pose,
                                     double *r,
                                     double *g,
                                     double *b)
{
    *r = theme->person_overlay_default[0];
    *g = theme->person_overlay_default[1];
    *b = theme->person_overlay_default[2];

    if (pose->id != info->selected_pose_id) {
        return;
    }

    if (strcmp(info->selection_state, "selecting") == 0) {
        *r = theme->person_overlay_selecting[0];
        *g = theme->person_overlay_selecting[1];
        *b = theme->person_overlay_selecting[2];
        return;
    }

    if (strcmp(info->selection_state, "selected") == 0) {
        *r = theme->person_overlay_selected[0];
        *g = theme->person_overlay_selected[1];
        *b = theme->person_overlay_selected[2];
    }
}

/* Choose a joint color from its confidence value. */
static void set_joint_color(cairo_t *cr,
                            const app_theme_t *theme,
                            const resource_info_t *info,
                            const overlay_controls_state_t *controls,
                            const pose_info_t *pose,
                            const joint_info_t *joint)
{
    double confidence_percent;
    double low_min;
    double medium_min;
    double high_min;

    (void)info;
    (void)pose;

    if (joint == NULL) {
        cairo_set_source_rgb(cr, theme->pose_joint_confidence_low[0],
                             theme->pose_joint_confidence_low[1],
                             theme->pose_joint_confidence_low[2]);
        return;
    }

    confidence_percent = (double)joint->confidence * 100.0;
    overlay_controls_get_confidence_color_thresholds(
        controls->confidence_threshold,
        &low_min,
        &medium_min,
        &high_min);

    if (confidence_percent >= high_min) {
        cairo_set_source_rgb(cr, theme->pose_joint_confidence_high[0],
                             theme->pose_joint_confidence_high[1],
                             theme->pose_joint_confidence_high[2]);
        return;
    }

    if (confidence_percent >= medium_min) {
        cairo_set_source_rgb(cr, theme->pose_joint_confidence_medium[0],
                             theme->pose_joint_confidence_medium[1],
                             theme->pose_joint_confidence_medium[2]);
        return;
    }

    if (confidence_percent >= low_min) {
        cairo_set_source_rgb(cr, theme->pose_joint_confidence_low[0],
                             theme->pose_joint_confidence_low[1],
                             theme->pose_joint_confidence_low[2]);
        return;
    }

    cairo_set_source_rgb(cr, theme->pose_joint_confidence_low[0],
                         theme->pose_joint_confidence_low[1],
                         theme->pose_joint_confidence_low[2]);
}

/* Draw one highlighted edge of the robot ROI. */
static void draw_roi_edge(cairo_t *cr, const roi_info_t *roi, roi_edge_t edge)
{
    switch (edge) {
    case ROI_EDGE_LEFT:
        cairo_move_to(cr, roi->tx, roi->ty);
        cairo_line_to(cr, roi->tx, roi->t_end_y);
        break;
    case ROI_EDGE_TOP:
        cairo_move_to(cr, roi->tx, roi->ty);
        cairo_line_to(cr, roi->t_end_x, roi->ty);
        break;
    case ROI_EDGE_RIGHT:
        cairo_move_to(cr, roi->t_end_x, roi->ty);
        cairo_line_to(cr, roi->t_end_x, roi->t_end_y);
        break;
    case ROI_EDGE_BOTTOM:
        cairo_move_to(cr, roi->tx, roi->t_end_y);
        cairo_line_to(cr, roi->t_end_x, roi->t_end_y);
        break;
    case ROI_EDGE_NONE:
    default:
        break;
    }
}

/* Draw the robot ROI and any active edit affordance. */
static void draw_roi_overlay(cairo_t *cr, const app_theme_t *theme,
                             const resource_info_t *info,
                             gboolean show_fill,
                             const roi_info_t *adjustment_roi,
                             roi_edge_t active_roi_edge,
                             roi_edge_t dragged_roi_edge)
{
    int roi_width = info->roi.t_end_x - info->roi.tx;
    int roi_height = info->roi.t_end_y - info->roi.ty;

    if (roi_width < 1) {
        roi_width = 1;
    }
    if (roi_height < 1) {
        roi_height = 1;
    }

    rounded_rect(cr, info->roi.tx, info->roi.ty, roi_width, roi_height, 14.0);
    if (show_fill) {
        cairo_set_source_rgba(cr,
                              theme->roi_overlay[0],
                              theme->roi_overlay[1],
                              theme->roi_overlay[2],
                              0.09);
        cairo_fill_preserve(cr);
    }
    cairo_set_source_rgba(cr,
                          theme->roi_overlay[0],
                          theme->roi_overlay[1],
                          theme->roi_overlay[2],
                          0.95);
    cairo_set_line_width(cr, 2.2);
    cairo_stroke(cr);

    if (adjustment_roi != NULL && active_roi_edge != ROI_EDGE_NONE) {
        cairo_set_line_width(cr, 4.0);
        cairo_set_source_rgba(cr,
                              theme->roi_overlay_active[0],
                              theme->roi_overlay_active[1],
                              theme->roi_overlay_active[2],
                              1.0);
        draw_roi_edge(cr, adjustment_roi, active_roi_edge);
        cairo_stroke(cr);
    }

    if (adjustment_roi != NULL && dragged_roi_edge != ROI_EDGE_NONE) {
        cairo_set_line_width(cr, 5.0);
        cairo_set_source_rgba(cr,
                              theme->roi_overlay_drag[0],
                              theme->roi_overlay_drag[1],
                              theme->roi_overlay_drag[2],
                              1.0);
        draw_roi_edge(cr, adjustment_roi, dragged_roi_edge);
        cairo_stroke(cr);
    }
}

/* Draw a single person's skeleton, joints, and selection highlights. */
static void draw_person_overlay(cairo_t *cr, int width,
                                const app_theme_t *theme,
                                const resource_info_t *info,
                                const pose_info_t *pose,
                                gboolean show_fill,
                                gboolean show_debug_info)
{
    const int line_gap = 4;
    const int label_margin = 8;
    cairo_font_extents_t debug_font_extents;
    cairo_font_extents_t pose_font_extents;
    cairo_text_extents_t debug_extents;
    cairo_text_extents_t pose_extents;
    char debug_text[MAX_POSE_DETECTED_TEXT_LEN + 16];
    char pose_text[32];
    double label_width;
    double label_height;
    double label_top;
    double label_x;
    double label_box_width;
    double label_box_height;
    double pose_baseline;
    double debug_baseline;
    double color_r;
    double color_g;
    double color_b;

    get_person_overlay_color(info, theme, pose, &color_r, &color_g, &color_b);

    rounded_rect(cr, pose->person.tx, pose->person.ty,
                 pose->person.t_width, pose->person.t_height, 16.0);
    if (show_fill) {
        cairo_set_source_rgba(cr, color_r, color_g, color_b, 0.10);
        cairo_fill_preserve(cr);
    }
    cairo_set_source_rgba(cr, color_r, color_g, color_b, 0.96);
    cairo_set_line_width(cr, 2.4);
    cairo_stroke(cr);

    snprintf(pose_text, sizeof(pose_text), "Pose %d (%.2f)",
             pose->id, pose->person.confidence);
    if (pose->pose_detected_text[0] != '\0') {
        snprintf(debug_text, sizeof(debug_text), "Detected: %s",
                 pose->pose_detected_text);
    } else {
        snprintf(debug_text, sizeof(debug_text), "Detected: none");
    }

    cairo_select_font_face(cr, "Sans",
                           CAIRO_FONT_SLANT_NORMAL,
                           CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, PERSON_LABEL_FONT_SIZE_PX);
    cairo_font_extents(cr, &pose_font_extents);
    cairo_text_extents(cr, pose_text, &pose_extents);

    label_width = pose_extents.width;
    label_height = pose_font_extents.height;

    if (show_debug_info) {
        cairo_select_font_face(cr, "Sans",
                               CAIRO_FONT_SLANT_NORMAL,
                               CAIRO_FONT_WEIGHT_NORMAL);
        cairo_set_font_size(cr, DEBUG_INFO_FONT_SIZE_PX - 4.0);
        cairo_font_extents(cr, &debug_font_extents);
        cairo_text_extents(cr, debug_text, &debug_extents);

        if (debug_extents.width > label_width) {
            label_width = debug_extents.width;
        }
        label_height += line_gap + debug_font_extents.height;
    }

    label_box_width = label_width + 2.0 * label_margin;
    label_box_height = label_height + 2.0 * label_margin;
    label_top = pose->person.ty - label_box_height;
    if (label_top < SCREEN_MARGIN_PX) {
        label_top = SCREEN_MARGIN_PX;
    }

    label_x = pose->person.tx;
    if (label_x + label_box_width > width - SCREEN_MARGIN_PX) {
        label_x = width - SCREEN_MARGIN_PX - label_box_width;
    }
    if (label_x < SCREEN_MARGIN_PX) {
        label_x = SCREEN_MARGIN_PX;
    }

    draw_glass_panel(cr, theme, label_x, label_top, label_box_width, label_box_height, 14.0);
    rounded_rect(cr, label_x, label_top, label_box_width, label_box_height, 14.0);
    cairo_set_source_rgba(cr, color_r, color_g, color_b, 0.85);
    cairo_set_line_width(cr, 1.8);
    cairo_stroke(cr);

    pose_baseline = label_top + label_margin + pose_font_extents.ascent;

    cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
    cairo_select_font_face(cr, "Sans",
                           CAIRO_FONT_SLANT_NORMAL,
                           CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, PERSON_LABEL_FONT_SIZE_PX);
    cairo_move_to(cr, label_x + label_margin, pose_baseline);
    cairo_show_text(cr, pose_text);

    if (show_debug_info) {
        debug_baseline = pose_baseline + line_gap + debug_font_extents.ascent +
                         (pose_font_extents.height - pose_font_extents.ascent);
        cairo_select_font_face(cr, "Sans",
                               CAIRO_FONT_SLANT_NORMAL,
                               CAIRO_FONT_WEIGHT_NORMAL);
        cairo_set_font_size(cr, DEBUG_INFO_FONT_SIZE_PX - 4.0);
        cairo_move_to(cr, label_x + label_margin, debug_baseline);
        cairo_show_text(cr, debug_text);
    }
}

/* Format application uptime for the debug overlay. */
static void format_uptime(char *buf, size_t buf_size, gint64 app_start_us)
{
    gint64 now_us = g_get_monotonic_time();
    guint64 elapsed_sec = (guint64)((now_us - app_start_us) / G_USEC_PER_SEC);
    guint64 days = elapsed_sec / 86400;
    guint64 hours = (elapsed_sec % 86400) / 3600;
    guint64 minutes = (elapsed_sec % 3600) / 60;
    guint64 seconds = elapsed_sec % 60;

    if (days > 0) {
        snprintf(buf, buf_size, "%llud %02llu:%02llu:%02llu",
                 (unsigned long long)days,
                 (unsigned long long)hours,
                 (unsigned long long)minutes,
                 (unsigned long long)seconds);
    } else {
        snprintf(buf, buf_size, "%02llu:%02llu:%02llu",
                 (unsigned long long)hours,
                 (unsigned long long)minutes,
                 (unsigned long long)seconds);
    }
}

/* Draw the textual debug panel in the configured screen corner. */
static void draw_debug_info(cairo_t *cr, int width, int height,
                            const app_theme_t *theme,
                            const resource_info_t *info,
                            debug_info_corner_t corner,
                            const char *app_version,
                            gint64 app_start_us)
{
    const int line_spacing = 8;
    cairo_text_extents_t extents;
    cairo_font_extents_t heading_extents;
    cairo_font_extents_t body_extents;
    const char *title = "Debug";
    char info_text[64];
    const char *qpp_selected_pose;
    const char *qpp_selected_pose_state;
    char uptime_text[64];
    char window_text[64];
    double card_width = 360.0;
    double card_height;
    double card_x;
    double card_y;
    double text_x;
    double baseline;

    cairo_select_font_face(cr, "Sans",
                           CAIRO_FONT_SLANT_NORMAL,
                           CAIRO_FONT_WEIGHT_BOLD);

    snprintf(window_text, sizeof(window_text), "%d x %d", width, height);
    snprintf(info_text, sizeof(info_text), "%d x %d", info->width, info->height);
    format_uptime(uptime_text, sizeof(uptime_text), app_start_us);
    qpp_selected_pose = info->qpp_selected_pose[0] != '\0' ?
        info->qpp_selected_pose : "(empty)";
    qpp_selected_pose_state = info->qpp_selected_pose_state[0] != '\0' ?
        info->qpp_selected_pose_state : "(unavailable)";

    cairo_set_font_size(cr, DEBUG_INFO_FONT_SIZE_PX - 2.0);
    cairo_font_extents(cr, &heading_extents);
    cairo_set_font_size(cr, DEBUG_INFO_FONT_SIZE_PX - 4.0);
    cairo_font_extents(cr, &body_extents);

    card_height = 26.0 + heading_extents.height +
                  (body_extents.height * 6.0) +
                  (line_spacing * 5.0) +
                  24.0;

    switch (corner) {
    case DEBUG_INFO_BOTTOM_RIGHT:
        card_x = width - card_width - SCREEN_MARGIN_PX;
        card_y = height - card_height - SCREEN_MARGIN_PX;
        break;
    case DEBUG_INFO_BOTTOM_LEFT:
        card_x = SCREEN_MARGIN_PX;
        card_y = height - card_height - SCREEN_MARGIN_PX;
        break;
    case DEBUG_INFO_TOP_LEFT:
        card_x = SCREEN_MARGIN_PX;
        card_y = SCREEN_MARGIN_PX;
        break;
    case DEBUG_INFO_TOP_RIGHT:
    default:
        card_x = width - card_width - SCREEN_MARGIN_PX;
        card_y = SCREEN_MARGIN_PX;
        break;
    }

    draw_glass_panel(cr, theme, card_x, card_y, card_width, card_height, 18.0);

    text_x = card_x + 18.0;
    baseline = card_y + 20.0 + heading_extents.ascent;

    cairo_set_font_size(cr, DEBUG_INFO_FONT_SIZE_PX - 2.0);
    cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
    cairo_move_to(cr, text_x, baseline);
    cairo_show_text(cr, title);

    cairo_set_source_rgba(cr, theme->debug_accent[0],
                          theme->debug_accent[1],
                          theme->debug_accent[2],
                          theme->debug_accent[3]);
    cairo_set_font_size(cr, DEBUG_INFO_FONT_SIZE_PX - 4.0);
    cairo_text_extents(cr, app_version, &extents);
    cairo_move_to(cr, card_x + card_width - extents.width - 18.0, baseline);
    cairo_show_text(cr, app_version);

    baseline += heading_extents.descent + 18.0;
    baseline += body_extents.height + line_spacing;
    cairo_set_source_rgba(cr, theme->debug_text_primary[0],
                          theme->debug_text_primary[1],
                          theme->debug_text_primary[2],
                          theme->debug_text_primary[3]);
    cairo_move_to(cr, text_x, baseline);
    cairo_show_text(cr, "Uptime");
    cairo_text_extents(cr, uptime_text, &extents);
    cairo_move_to(cr, card_x + card_width - extents.width - 18.0, baseline);
    cairo_show_text(cr, uptime_text);

    baseline += body_extents.height + line_spacing;
    cairo_set_source_rgba(cr, theme->debug_text_secondary[0],
                          theme->debug_text_secondary[1],
                          theme->debug_text_secondary[2],
                          theme->debug_text_secondary[3]);
    cairo_move_to(cr, text_x, baseline);
    cairo_show_text(cr, "Window");
    cairo_text_extents(cr, window_text, &extents);
    cairo_move_to(cr, card_x + card_width - extents.width - 18.0, baseline);
    cairo_show_text(cr, window_text);

    baseline += body_extents.height + line_spacing;
    cairo_move_to(cr, text_x, baseline);
    cairo_show_text(cr, "Input");
    cairo_text_extents(cr, info_text, &extents);
    cairo_move_to(cr, card_x + card_width - extents.width - 18.0, baseline);
    cairo_show_text(cr, info_text);

    baseline += body_extents.height + line_spacing;
    cairo_move_to(cr, text_x, baseline);
    cairo_show_text(cr, "Selected pose");
    cairo_text_extents(cr, qpp_selected_pose, &extents);
    cairo_move_to(cr, card_x + card_width - extents.width - 18.0, baseline);
    cairo_show_text(cr, qpp_selected_pose);

    baseline += body_extents.height + line_spacing;
    cairo_move_to(cr, text_x, baseline);
    cairo_show_text(cr, "Selected state");
    cairo_text_extents(cr, qpp_selected_pose_state, &extents);
    cairo_move_to(cr, card_x + card_width - extents.width - 18.0, baseline);
    cairo_show_text(cr, qpp_selected_pose_state);
}

/* Load the physical joint/IK popup artwork on demand. */
static GdkPixbuf *load_popup_pixbuf(const char *resource_path)
{
    static GdkPixbuf *selected_left_pixbuf = NULL;
    static GdkPixbuf *selected_right_pixbuf = NULL;
    static GdkPixbuf *hand_selected_left_pixbuf = NULL;
    static GdkPixbuf *hand_selected_right_pixbuf = NULL;
    static GdkPixbuf *not_selected_pixbuf = NULL;
    static GdkPixbuf *hand_not_selected_pixbuf = NULL;
    GError *error = NULL;
    GdkPixbuf **slot = NULL;

    if (strcmp(resource_path,
               "/com/qnx/physical-ai-ui/ui/robot_mode_joints_selected_left.png") == 0) {
        slot = &selected_left_pixbuf;
    } else if (strcmp(resource_path,
                      "/com/qnx/physical-ai-ui/ui/robot_mode_joints_selected_right.png") == 0) {
        slot = &selected_right_pixbuf;
    } else if (strcmp(resource_path,
                      "/com/qnx/physical-ai-ui/ui/robot_mode_hand_selected_left.png") == 0) {
        slot = &hand_selected_left_pixbuf;
    } else if (strcmp(resource_path,
                      "/com/qnx/physical-ai-ui/ui/robot_mode_hand_selected_right.png") == 0) {
        slot = &hand_selected_right_pixbuf;
    } else if (strcmp(resource_path,
                      "/com/qnx/physical-ai-ui/ui/robot_mode_joints_not_selected.png") == 0) {
        slot = &not_selected_pixbuf;
    } else if (strcmp(resource_path,
                      "/com/qnx/physical-ai-ui/ui/robot_mode_hand_not_selected.png") == 0) {
        slot = &hand_not_selected_pixbuf;
    } else {
        return NULL;
    }

    if (*slot != NULL) {
        return *slot;
    }

    *slot = gdk_pixbuf_new_from_resource(resource_path, &error);
    if (*slot == NULL) {
        if (error != NULL) {
            g_warning("Failed to load popup resource %s: %s",
                      resource_path,
                      error->message);
            g_error_free(error);
        }
        return NULL;
    }

    return *slot;
}

static bool pose_popups_available_for_robot_mode(const char *robot_mode)
{
    return strcmp(robot_mode, "joint") == 0 ||
           strcmp(robot_mode, "ik_joint") == 0;
}

/* Return the selected-joint artwork for the left arm. */
static const char *get_selected_left_joint_popup_resource(
    const char *robot_mode)
{
    if (strcmp(robot_mode, "ik_joint") == 0) {
        return "/com/qnx/physical-ai-ui/ui/robot_mode_hand_selected_left.png";
    }

    return "/com/qnx/physical-ai-ui/ui/robot_mode_joints_selected_left.png";
}

/* Return the selected-joint artwork for the right arm. */
static const char *get_selected_right_joint_popup_resource(
    const char *robot_mode)
{
    if (strcmp(robot_mode, "ik_joint") == 0) {
        return "/com/qnx/physical-ai-ui/ui/robot_mode_hand_selected_right.png";
    }

    return "/com/qnx/physical-ai-ui/ui/robot_mode_joints_selected_right.png";
}

/* Return the popup artwork for an unselected joint. */
static const char *get_not_selected_joint_popup_resource(
    const char *robot_mode)
{
    if (strcmp(robot_mode, "ik_joint") == 0) {
        return "/com/qnx/physical-ai-ui/ui/robot_mode_hand_not_selected.png";
    }

    return "/com/qnx/physical-ai-ui/ui/robot_mode_joints_not_selected.png";
}

/* Draw a joint status popup anchored to a detected joint. */
static void draw_joint_popup_card(cairo_t *cr,
                                  const app_theme_t *theme,
                                  GdkPixbuf *pixbuf,
                                  double card_x,
                                  double card_y,
                                  double card_width,
                                  double card_height,
                                  double padding)
{
    const double radius = 18.0;
    double available_width;
    double available_height;
    double image_width;
    double image_height;
    double image_x;
    double image_y;
    double scale;

    if (pixbuf == NULL || card_width <= 0.0 || card_height <= 0.0) {
        return;
    }

    rounded_rect(cr, card_x + 2.0, card_y + 8.0, card_width, card_height, radius);
    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 0.20);
    cairo_fill(cr);

    rounded_rect(cr, card_x, card_y, card_width, card_height, radius);
    cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
    cairo_fill_preserve(cr);
    cairo_set_source_rgba(cr, theme->glass_panel_stroke[0],
                          theme->glass_panel_stroke[1],
                          theme->glass_panel_stroke[2],
                          theme->glass_panel_stroke[3]);
    cairo_set_line_width(cr, 1.2);
    cairo_stroke(cr);

    available_width = card_width - (2.0 * padding);
    available_height = card_height - (2.0 * padding);
    if (available_width <= 0.0 || available_height <= 0.0) {
        return;
    }

    scale = MIN(available_width / (double)gdk_pixbuf_get_width(pixbuf),
                available_height / (double)gdk_pixbuf_get_height(pixbuf));
    image_width = (double)gdk_pixbuf_get_width(pixbuf) * scale;
    image_height = (double)gdk_pixbuf_get_height(pixbuf) * scale;
    image_x = card_x + ((card_width - image_width) / 2.0);
    image_y = card_y + ((card_height - image_height) / 2.0);

    cairo_save(cr);
    rounded_rect(cr, card_x + 1.0, card_y + 1.0, card_width - 2.0,
                 card_height - 2.0, radius - 1.0);
    cairo_clip(cr);
    cairo_translate(cr, image_x, image_y);
    cairo_scale(cr, scale, scale);
    gdk_cairo_set_source_pixbuf(cr, pixbuf, 0.0, 0.0);
    cairo_paint(cr);
    cairo_restore(cr);
}

/* Draw the robot-mode-specific artwork popups for joint and IK states. */
static void draw_joint_mode_popups(cairo_t *cr,
                                   int width,
                                   int height,
                                   const app_theme_t *theme,
                                   const resource_info_t *info)
{
    camera_display_rect_t display_rect =
        overlay_renderer_get_camera_display_rect(info->width, info->height,
                                                 width, height);
    GdkPixbuf *left_pixbuf = NULL;
    GdkPixbuf *right_pixbuf = NULL;
    gboolean is_selected = FALSE;
    double popup_width;
    double max_popup_width;
    double popup_height;
    double popup_padding;
    double popup_y;
    double left_x;
    double right_x;

    if (display_rect.width <= 0 || display_rect.height <= 0) {
        return;
    }

    if (strcmp(info->selection_state, "selected") == 0) {
        is_selected = TRUE;
        left_pixbuf = load_popup_pixbuf(
            get_selected_left_joint_popup_resource(info->robot_mode));
        right_pixbuf = load_popup_pixbuf(
            get_selected_right_joint_popup_resource(info->robot_mode));
    } else {
        left_pixbuf = load_popup_pixbuf(
            get_not_selected_joint_popup_resource(info->robot_mode));
    }

    if (left_pixbuf == NULL) {
        return;
    }

    popup_width = floor((double)display_rect.width * 0.33);
    max_popup_width =
        floor(((double)width - (3.0 * SCREEN_MARGIN_PX)) / 2.0);
    popup_width = MAX(1.0, popup_width);
    popup_width = MIN(popup_width, max_popup_width);

    popup_height = MIN((double)height - (2.0 * SCREEN_MARGIN_PX),
                       is_selected ? popup_width * 1.45 : popup_width);
    popup_height = MAX(1.0, popup_height);
    popup_padding = is_selected ? 4.0 : 10.0;
    popup_y = ((double)height - popup_height) / 2.0;
    left_x = SCREEN_MARGIN_PX;

    draw_joint_popup_card(cr, theme, left_pixbuf,
                          left_x, popup_y, popup_width, popup_height,
                          popup_padding);

    if (right_pixbuf == NULL) {
        return;
    }

    right_x = (double)width - popup_width - SCREEN_MARGIN_PX;
    draw_joint_popup_card(cr, theme, right_pixbuf,
                          right_x, popup_y, popup_width, popup_height,
                          popup_padding);
}

/* Draw crosshairs at the center of the camera image. */
static void draw_camera_center(cairo_t *cr, int width, int height,
                               const resource_info_t *info)
{
    int size = 10;
    int tx;
    int ty;
    camera_display_rect_t display_rect;

    if (info->width == 0 || info->height == 0) {
        return;
    }

    display_rect = overlay_renderer_get_camera_display_rect(info->width,
                                                            info->height,
                                                            width, height);
    translate_point(info->width / 2, info->height / 2,
                    info->width, info->height,
                    &display_rect,
                    &tx, &ty);

    cairo_set_source_rgb(cr, 1.0, 0.0, 0.0);
    cairo_set_line_width(cr, 2.0);
    cairo_move_to(cr, tx - size, ty - size);
    cairo_line_to(cr, tx + size, ty + size);
    cairo_move_to(cr, tx - size, ty + size);
    cairo_line_to(cr, tx + size, ty - size);
    cairo_stroke(cr);
}

/* Draw crosshairs at the center of the overlay surface. */
static void draw_screen_center(cairo_t *cr, int width, int height)
{
    int cx = width / 2;
    int cy = height / 2;
    int size = 10;

    cairo_set_source_rgb(cr, 0.0, 1.0, 0.0);
    cairo_set_line_width(cr, 2.0);
    cairo_move_to(cr, cx - size, cy);
    cairo_line_to(cr, cx + size, cy);
    cairo_move_to(cr, cx, cy - size);
    cairo_line_to(cr, cx, cy + size);
    cairo_stroke(cr);
}

/* Report whether joint popups should be rendered. */
static gboolean joint_popups_visible_for_overlay(
    const overlay_controls_state_t *controls,
    const resource_info_t *info)
{
    return controls->show_joint_popups &&
           pose_popups_available_for_robot_mode(info->robot_mode);
}

/* Render every overlay layer for the latest pose snapshot. */
void overlay_renderer_draw(cairo_t *cr,
                           int width,
                           int height,
                           const resource_info_t *info_in,
                           const overlay_controls_state_t *controls,
                           const roi_info_t *adjustment_roi_in,
                           roi_edge_t active_roi_edge,
                           roi_edge_t dragged_roi_edge,
                           const char *app_version,
                           gint64 app_start_us)
{
    const app_theme_t *theme = app_theme_get();
    const double radius = 4.0;
    camera_display_rect_t display_rect;
    roi_info_t adjustment_roi;
    roi_info_t *adjustment_roi_ptr = NULL;
    resource_info_t info;
    size_t i;

    info = *info_in;
    display_rect = overlay_renderer_get_camera_display_rect(info.width,
                                                            info.height,
                                                            width, height);
    translate_all(&info, &display_rect);
    if (controls->flip_video) {
        mirror_pose_overlays(&info, width);
        mirror_roi_display(&info.roi, width);
    }
    if (adjustment_roi_in != NULL) {
        adjustment_roi = *adjustment_roi_in;
        translate_roi(&adjustment_roi, &display_rect);
        if (controls->flip_video) {
            mirror_roi_display(&adjustment_roi, width);
        }
        adjustment_roi_ptr = &adjustment_roi;
    }

    if (controls->show_roi) {
        draw_roi_overlay(cr, theme, &info, controls->show_overlay_fills,
                         adjustment_roi_ptr,
                         controls->flip_video ?
                             mirror_roi_edge(active_roi_edge) : active_roi_edge,
                         controls->flip_video ?
                             mirror_roi_edge(dragged_roi_edge) : dragged_roi_edge);
    }

    if (controls->show_debug_info &&
        !joint_popups_visible_for_overlay(controls, &info)) {
        draw_screen_center(cr, width, height);
        draw_camera_center(cr, width, height, &info);
    }

    if (!DEBUG_DISABLE_POSE_PIPELINE) {
        for (i = 0; i < info.pose_count; i++) {
            pose_info_t *pose = &info.poses[i];
            size_t e;
            size_t j;

            draw_person_overlay(cr, width, theme, &info, pose,
                                controls->show_overlay_fills,
                                controls->show_gestures ||
                                    (controls->show_debug_info &&
                                     !joint_popups_visible_for_overlay(
                                         controls, &info)));

            if (controls->show_skeleton) {
                cairo_set_line_width(cr, 3.0);

                for (e = 0; e < info.keypoint_edge_count; e++) {
                    int from_index = info.keypoint_edges[e].from_index;
                    int to_index = info.keypoint_edges[e].to_index;
                    const joint_info_t *j1 = NULL;
                    const joint_info_t *j2 = NULL;

                    if (from_index < 0 || to_index < 0 ||
                        (size_t)from_index >= pose->joint_count ||
                        (size_t)to_index >= pose->joint_count) {
                        continue;
                    }

                    j1 = &pose->joints[from_index];
                    j2 = &pose->joints[to_index];

                    if (controls->filter_low_confidence &&
                        (j1->confidence <
                             overlay_controls_confidence_threshold_to_normalized(
                                 controls->confidence_threshold) ||
                         j2->confidence <
                             overlay_controls_confidence_threshold_to_normalized(
                                 controls->confidence_threshold))) {
                        continue;
                    }

                    set_skeleton_edge_color(cr, theme, &info, pose);
                    cairo_move_to(cr, j1->tx, j1->ty);
                    cairo_line_to(cr, j2->tx, j2->ty);
                    cairo_stroke(cr);
                }
            }

            if (controls->show_joints) {
                for (j = 0; j < pose->joint_count; j++) {
                    joint_info_t *joint = &pose->joints[j];

                    if (controls->filter_low_confidence &&
                        joint->confidence <
                            overlay_controls_confidence_threshold_to_normalized(
                                controls->confidence_threshold)) {
                        continue;
                    }

                    set_joint_color(cr, theme, &info, controls, pose, joint);
                    cairo_arc(cr, joint->tx, joint->ty, radius + 1.0,
                              0.0, 2.0 * G_PI);
                    cairo_fill_preserve(cr);
                    cairo_set_source_rgba(cr,
                                          theme->pose_joint_outline[0],
                                          theme->pose_joint_outline[1],
                                          theme->pose_joint_outline[2],
                                          0.95);
                    cairo_set_line_width(cr, 1.2);
                    cairo_stroke(cr);
                }
            }
        }

    }

    if (controls->show_debug_info &&
        !joint_popups_visible_for_overlay(controls, &info)) {
        draw_debug_info(cr, width, height, theme, &info,
                        controls->debug_info_corner,
                        app_version, app_start_us);
    }

    if (joint_popups_visible_for_overlay(controls, &info)) {
        draw_joint_mode_popups(cr, width, height, theme, &info);
    }

}
