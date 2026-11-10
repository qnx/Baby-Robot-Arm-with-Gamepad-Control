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
#include <memory>
#include <string>

#include <hailo/hailort.hpp>
#include <opencv2/opencv.hpp>

#include "ml/output_tensor.hpp"
#include "util.hpp"

namespace joint::ml {

/**
 * @brief Detects depth of an area
 */
class DepthDetection {
   public:
    using DepthMap = std::array<std::array<float, 224>, 224>;
    using DepthMapPtr = std::unique_ptr<DepthMap>;


   public:
    DepthDetection();
    ~DepthDetection();

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
     * @param image Image in BGR format
     *
     * @return
     */
    virtual DepthMapPtr process(const cv::Mat &image) = 0;

   protected:
    // Hailo Objects
    std::vector<hailort::InputVStream> inputs_{};
    std::vector<hailort::OutputVStream> outputs_{};
    std::vector<OutputTensor> output_tensors_{};
    std::shared_ptr<hailort::ConfiguredNetworkGroup> network_group_{};
};

}  // namespace joint::ml
