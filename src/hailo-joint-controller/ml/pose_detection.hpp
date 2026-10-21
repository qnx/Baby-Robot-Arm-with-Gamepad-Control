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
#include "util.hpp"

namespace joint::ml {

/**
 * @brief Generic Pose detection wrapper using coco 17 points
 *
 *
 */
class PoseDetection {
   protected:
    static constexpr size_t num_joints_ = 17;

   public:
    struct Point {
        std::string name;
        int x;
        int y;
        float z;
        float confidence;
    };
    struct Person {
        int id;  // ID associated with the pose
        float confidence;
        hailo_bbox_t box;
        std::array<Point, num_joints_> key_points;
        std::vector<std::string> detected_poses;
    };

   public:
    PoseDetection(float joint_conf_threshold = 0.3);
    ~PoseDetection();

    /**
     * @brief Setup Hailo and pose model
     *
     *
     * @return
     */
    virtual int init(const std::string &hef_path);

    /**
     * @brief Process a frame
     *
     * @param image
     *
     * @return
     */
    virtual hailort::Expected<std::vector<Person>> process(const cv::Mat &image) = 0;

    ROIBounds get_roi() const;
    int set_roi(const ROIBounds &roi);

   protected:
    // Hailo Objects
    std::vector<hailort::InputVStream> inputs_{};
    std::vector<hailort::OutputVStream> outputs_{};
    std::vector<OutputTensor> output_tensors_{};
    std::shared_ptr<hailort::ConfiguredNetworkGroup> network_group_{};

    std::array<uint8_t, 4> roi_;

    // Model Specific configuration
    float joint_conf_threshold_;  // What confidence do we need to identify a joint
};

}  // namespace joint::ml
