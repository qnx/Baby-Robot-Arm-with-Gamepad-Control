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

#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

#define APP_VERSION "v3.1"
#define PHYSICAL_QPP_ROBOT_MODE_PATH "/dev/qpp/physicalai/bra/mode?text"
#define PHYSICAL_QPP_ROBOT_ENABLED_PATH "/dev/qpp/physicalai/bra/enabled?text"
#define PHYSICAL_QPP_ROBOT_ENABLED_CONTROL_PATH "/dev/qpp/physicalai/bra/.enabled.control?text"
#define PHYSICAL_QPP_ROBOT_ROI_PATH "/dev/qpp/physicalai/hands/roi?text"
#define PHYSICAL_QPP_ROBOT_ROI_CONTROL_PATH "/dev/qpp/physicalai/hands/.roi.control?text"
#define PHYSICAL_QPP_WRIST_CONTROL_PATH "/dev/qpp/physicalai/hands/controls/wrists?text"
#define PHYSICAL_QPP_WRIST_CONTROL_WRITE_PATH \
    "/dev/qpp/physicalai/hands/controls/.wrists.control?text"
#define PHYSICAL_QPP_SCREENSAVER_MODE_PATH "/dev/qpp/physicalai/bra/screensaver?text"
#define PHYSICAL_QPP_HAND_INFO_PATH "/dev/qpp/physicalai/hands/info?text"
#define PHYSICAL_QPP_LEFT_HAND_DETECTION_PATH "/dev/qpp/physicalai/hands/detections/left_hand1?text"
#define PHYSICAL_QPP_RIGHT_HAND_DETECTION_PATH \
    "/dev/qpp/physicalai/hands/detections/right_hand1?text"
#define PHYSICAL_QPP_HAND_SELECTED_POSE_PATH "/dev/qpp/physicalai/hands/selected_pose?text"
#define PHYSICAL_QPP_HAND_SELECTED_POSE_STATE_PATH \
    "/dev/qpp/physicalai/hands/selected_pose_state?text"

#define CONFIDENCE_THRESHOLD_MIN 0.0
#define CONFIDENCE_THRESHOLD_MAX 50.0
#define CONFIDENCE_THRESHOLD_DEFAULT 30.0
#define DEBUG_ENABLE_FRAME_PIPELINE_LOGS false
#define DROP_COUNT_PERIOD_NS (5ull * 1000 * 1000 * 1000)
#define FRAME_STAGE_STATS_PERIOD_NS (5ull * 1000 * 1000 * 1000)
#define POSE_DATA_ERROR_LOG_THROTTLE_NS (5ull * 1000 * 1000 * 1000)

#define CAMERA_REQUESTED_WIDTH 960
#define CAMERA_REQUESTED_HEIGHT 540
#define CAMERA_REQUESTED_FRAMERATE 15.0

#define PERSON_LABEL_FONT_SIZE_PX 20.0
#define DEBUG_INFO_FONT_SIZE_PX 20.0
#define SCREEN_MARGIN_PX 16

/* Physical overlay theme */
#define PHYSICAL_PERSON_OVERLAY_DEFAULT_R 0.09
#define PHYSICAL_PERSON_OVERLAY_DEFAULT_G 0.76
#define PHYSICAL_PERSON_OVERLAY_DEFAULT_B 0.96
#define PHYSICAL_PERSON_OVERLAY_SELECTING_R 0.98
#define PHYSICAL_PERSON_OVERLAY_SELECTING_G 0.75
#define PHYSICAL_PERSON_OVERLAY_SELECTING_B 0.24
#define PHYSICAL_PERSON_OVERLAY_SELECTED_R 0.19
#define PHYSICAL_PERSON_OVERLAY_SELECTED_G 0.89
#define PHYSICAL_PERSON_OVERLAY_SELECTED_B 0.58
#define PHYSICAL_ROI_OVERLAY_R 0.53
#define PHYSICAL_ROI_OVERLAY_G 0.80
#define PHYSICAL_ROI_OVERLAY_B 1.0
#define PHYSICAL_ROI_OVERLAY_ACTIVE_R 1.0
#define PHYSICAL_ROI_OVERLAY_ACTIVE_G 0.83
#define PHYSICAL_ROI_OVERLAY_ACTIVE_B 0.32
#define PHYSICAL_ROI_OVERLAY_DRAG_R 0.25
#define PHYSICAL_ROI_OVERLAY_DRAG_G 0.95
#define PHYSICAL_ROI_OVERLAY_DRAG_B 0.92
#define PHYSICAL_POSE_JOINT_CONFIDENCE_HIGH_COLOR_R 0.0
#define PHYSICAL_POSE_JOINT_CONFIDENCE_HIGH_COLOR_G 0.78
#define PHYSICAL_POSE_JOINT_CONFIDENCE_HIGH_COLOR_B 0.24
#define PHYSICAL_POSE_JOINT_CONFIDENCE_MEDIUM_COLOR_R 0.98
#define PHYSICAL_POSE_JOINT_CONFIDENCE_MEDIUM_COLOR_G 0.80
#define PHYSICAL_POSE_JOINT_CONFIDENCE_MEDIUM_COLOR_B 0.18
#define PHYSICAL_POSE_JOINT_CONFIDENCE_LOW_COLOR_R 0.90
#define PHYSICAL_POSE_JOINT_CONFIDENCE_LOW_COLOR_G 0.18
#define PHYSICAL_POSE_JOINT_CONFIDENCE_LOW_COLOR_B 0.18
#define PHYSICAL_POSE_JOINT_OUTLINE_COLOR_R 1.0
#define PHYSICAL_POSE_JOINT_OUTLINE_COLOR_G 1.0
#define PHYSICAL_POSE_JOINT_OUTLINE_COLOR_B 1.0
#define PHYSICAL_POSE_SELECTED_COLOR_R 0.0
#define PHYSICAL_POSE_SELECTED_COLOR_G 0.55
#define PHYSICAL_POSE_SELECTED_COLOR_B 0.18
#define PHYSICAL_GLASS_PANEL_FILL_R 0.03
#define PHYSICAL_GLASS_PANEL_FILL_G 0.08
#define PHYSICAL_GLASS_PANEL_FILL_B 0.13
#define PHYSICAL_GLASS_PANEL_FILL_A 0.78
#define PHYSICAL_GLASS_PANEL_STROKE_R 1.0
#define PHYSICAL_GLASS_PANEL_STROKE_G 1.0
#define PHYSICAL_GLASS_PANEL_STROKE_B 1.0
#define PHYSICAL_GLASS_PANEL_STROKE_A 0.10
#define PHYSICAL_DEBUG_ACCENT_R 0.60
#define PHYSICAL_DEBUG_ACCENT_G 0.87
#define PHYSICAL_DEBUG_ACCENT_B 1.0
#define PHYSICAL_DEBUG_ACCENT_A 1.0
#define PHYSICAL_DEBUG_TEXT_PRIMARY_R 0.90
#define PHYSICAL_DEBUG_TEXT_PRIMARY_G 0.96
#define PHYSICAL_DEBUG_TEXT_PRIMARY_B 1.0
#define PHYSICAL_DEBUG_TEXT_PRIMARY_A 0.92
#define PHYSICAL_DEBUG_TEXT_SECONDARY_R 0.90
#define PHYSICAL_DEBUG_TEXT_SECONDARY_G 0.96
#define PHYSICAL_DEBUG_TEXT_SECONDARY_B 1.0
#define PHYSICAL_DEBUG_TEXT_SECONDARY_A 0.72

#define DEBUG_DISABLE_POSE_PIPELINE false
#define DEBUG_DISABLE_OVERLAY_QUEUE_DRAW false
#define DEBUG_DISABLE_LOWER_FRAME_SIZE_SELECTION false

#endif
