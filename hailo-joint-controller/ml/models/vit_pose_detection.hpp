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

#include <array>
#include <string>

#include <hailo/hailort.hpp>
#include <opencv2/opencv.hpp>

#include "ml/output_tensor.hpp"
#include "ml/pose_detection.hpp"

namespace joint::ml::model {

class VitPoseDetection : public PoseDetection {

   public:
    VitPoseDetection(float joint_conf_threshold = 0.3);

    /**
     * @brief Process a frame
     *
     * @param image Image in BGR format
     *
     * @return
     */
    hailort::Expected<std::vector<PoseDetection::Person>> process(const cv::Mat &image) override;

    std::array<std::string, num_joints_> joint_names_ = {
        "nose",           "left_eye",   "right_eye",   "left_ear",   "right_ear",   "left_shoulder",
        "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist", "left_hip",
        "right_hip",      "left_knee",  "right_knee",  "left_ankle", "right_ankle"};

    // Previous confidence that it is a person. we will use this to assign Ids
    float previous_score_ = 0;
    uint16_t person_score_threshold_ = 30; /// Scale 0-100
};

}  // namespace joint::ml::model
