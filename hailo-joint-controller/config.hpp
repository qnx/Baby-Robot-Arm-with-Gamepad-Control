/**
 * Copyright (c) 2026, BlackBerry Limited. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <optional>
#include <string>

#include <camera/camera_api.h>
#include <sensor/sensor_interim_data.h>
#include <spdlog/spdlog.h>

#include "util.hpp"

struct HailoJointConfig {
    /// Path to either VIT pose detection
    std::string pose_model_path;
    std::optional<std::string> depth_model_path;

    /// Catalog-backed QPP pose branch. The signals must already exist when the
    /// controller starts.
    std::string qpp_pose_path = "/dev/qpp/physicalai/pose";

    std::string qpp_bra_path = "/dev/qpp/physicalai/bra";

    /**
     * Region of screen where model will be run. Before running the image from screen will be first cropped to the ROI
     * Then scalled down to the required size for the model.
     * defined as <x_offset,y_offset, x_end, y_end>
     */
    joint::ROIBounds region_of_interest = {0, 0, 100, 100};

    /// Which camera should be used for model. Defaults to 1.
    camera_unit_t camera_unit = CAMERA_UNIT_1;

    /// Topic which joint angles will be published on.
    std::string ros2_joint_topic = "joint/bra";

    spdlog::level::level_enum log_level = spdlog::level::info;

    float detection_threshold_ = 0.4;
};

/**
 * @brief Parses cli options for the joint controller
 *
 * @param argc Number of arguments
 * @param argv Argument values
 * @param[out] On success contains configuration for controller.
 * @return EOK on success otherwise an associated error code.
 */
int parse_cli(int argc, char *argv[], HailoJointConfig &config);
