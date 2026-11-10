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

#include "app_theme.h"

#include "app_config.h"

static const app_theme_t APP_THEME = {
    .person_overlay_default = {
        PHYSICAL_PERSON_OVERLAY_DEFAULT_R,
        PHYSICAL_PERSON_OVERLAY_DEFAULT_G,
        PHYSICAL_PERSON_OVERLAY_DEFAULT_B
    },
    .person_overlay_selecting = {
        PHYSICAL_PERSON_OVERLAY_SELECTING_R,
        PHYSICAL_PERSON_OVERLAY_SELECTING_G,
        PHYSICAL_PERSON_OVERLAY_SELECTING_B
    },
    .person_overlay_selected = {
        PHYSICAL_PERSON_OVERLAY_SELECTED_R,
        PHYSICAL_PERSON_OVERLAY_SELECTED_G,
        PHYSICAL_PERSON_OVERLAY_SELECTED_B
    },
    .roi_overlay = {
        PHYSICAL_ROI_OVERLAY_R,
        PHYSICAL_ROI_OVERLAY_G,
        PHYSICAL_ROI_OVERLAY_B
    },
    .roi_overlay_active = {
        PHYSICAL_ROI_OVERLAY_ACTIVE_R,
        PHYSICAL_ROI_OVERLAY_ACTIVE_G,
        PHYSICAL_ROI_OVERLAY_ACTIVE_B
    },
    .roi_overlay_drag = {
        PHYSICAL_ROI_OVERLAY_DRAG_R,
        PHYSICAL_ROI_OVERLAY_DRAG_G,
        PHYSICAL_ROI_OVERLAY_DRAG_B
    },
    .pose_joint_confidence_high = {
        PHYSICAL_POSE_JOINT_CONFIDENCE_HIGH_COLOR_R,
        PHYSICAL_POSE_JOINT_CONFIDENCE_HIGH_COLOR_G,
        PHYSICAL_POSE_JOINT_CONFIDENCE_HIGH_COLOR_B
    },
    .pose_joint_confidence_medium = {
        PHYSICAL_POSE_JOINT_CONFIDENCE_MEDIUM_COLOR_R,
        PHYSICAL_POSE_JOINT_CONFIDENCE_MEDIUM_COLOR_G,
        PHYSICAL_POSE_JOINT_CONFIDENCE_MEDIUM_COLOR_B
    },
    .pose_joint_confidence_low = {
        PHYSICAL_POSE_JOINT_CONFIDENCE_LOW_COLOR_R,
        PHYSICAL_POSE_JOINT_CONFIDENCE_LOW_COLOR_G,
        PHYSICAL_POSE_JOINT_CONFIDENCE_LOW_COLOR_B
    },
    .pose_joint_outline = {
        PHYSICAL_POSE_JOINT_OUTLINE_COLOR_R,
        PHYSICAL_POSE_JOINT_OUTLINE_COLOR_G,
        PHYSICAL_POSE_JOINT_OUTLINE_COLOR_B
    },
    .pose_selected = {
        PHYSICAL_POSE_SELECTED_COLOR_R,
        PHYSICAL_POSE_SELECTED_COLOR_G,
        PHYSICAL_POSE_SELECTED_COLOR_B
    },
    .glass_panel_fill = {
        PHYSICAL_GLASS_PANEL_FILL_R,
        PHYSICAL_GLASS_PANEL_FILL_G,
        PHYSICAL_GLASS_PANEL_FILL_B,
        PHYSICAL_GLASS_PANEL_FILL_A
    },
    .glass_panel_stroke = {
        PHYSICAL_GLASS_PANEL_STROKE_R,
        PHYSICAL_GLASS_PANEL_STROKE_G,
        PHYSICAL_GLASS_PANEL_STROKE_B,
        PHYSICAL_GLASS_PANEL_STROKE_A
    },
    .debug_accent = {
        PHYSICAL_DEBUG_ACCENT_R,
        PHYSICAL_DEBUG_ACCENT_G,
        PHYSICAL_DEBUG_ACCENT_B,
        PHYSICAL_DEBUG_ACCENT_A
    },
    .debug_text_primary = {
        PHYSICAL_DEBUG_TEXT_PRIMARY_R,
        PHYSICAL_DEBUG_TEXT_PRIMARY_G,
        PHYSICAL_DEBUG_TEXT_PRIMARY_B,
        PHYSICAL_DEBUG_TEXT_PRIMARY_A
    },
    .debug_text_secondary = {
        PHYSICAL_DEBUG_TEXT_SECONDARY_R,
        PHYSICAL_DEBUG_TEXT_SECONDARY_G,
        PHYSICAL_DEBUG_TEXT_SECONDARY_B,
        PHYSICAL_DEBUG_TEXT_SECONDARY_A
    }
};

/* Return the shared application color and typography theme. */
const app_theme_t *app_theme_get(void)
{
    return &APP_THEME;
}
