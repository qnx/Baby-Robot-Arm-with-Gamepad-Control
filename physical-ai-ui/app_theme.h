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

#ifndef APP_THEME_H_
#define APP_THEME_H_

typedef struct {
    double person_overlay_default[3];
    double person_overlay_selecting[3];
    double person_overlay_selected[3];
    double roi_overlay[3];
    double roi_overlay_active[3];
    double roi_overlay_drag[3];
    double pose_joint_confidence_high[3];
    double pose_joint_confidence_medium[3];
    double pose_joint_confidence_low[3];
    double pose_joint_outline[3];
    double pose_selected[3];
    double glass_panel_fill[4];
    double glass_panel_stroke[4];
    double debug_accent[4];
    double debug_text_primary[4];
    double debug_text_secondary[4];
} app_theme_t;

/* Return the physical AI color and styling palette. */
const app_theme_t *app_theme_get(void);

#endif
