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

#include "ml/models/fast_depth_detection.hpp"

#include <spdlog/spdlog.h>

#include "util.hpp"

using joint::ml::model::FastDepthDetection;

FastDepthDetection::FastDepthDetection() = default;

FastDepthDetection::DepthMapPtr FastDepthDetection::process(const cv::Mat &src_image) {
    if (src_image.empty()) {
        spdlog::warn("Invalid image(empty)");
        return nullptr;
    }

    cv::Mat image;
    cv::resize(src_image, image, cv::Size(224, 224));

    size_t input_size = inputs_.at(0).get_frame_size();
    if (image.total() * image.elemSize() != input_size) {
        spdlog::warn("Invalid image(size mismatch)");
        return nullptr;
    }

    auto hailo_res = inputs_.at(0).write(hailort::MemoryView(image.data, input_size));
    if (HAILO_SUCCESS != hailo_res) {
        std::cerr << "Failed to write input" << std::endl;
        return nullptr;
    }

    inputs_.at(0).flush();
    for (auto &output : output_tensors_) {
        auto res = output.read();
        if (HAILO_SUCCESS != res) {
            spdlog::warn("Failed to read output {}", output.name());
            return nullptr;
        }
    }

    // The input image is just returned with 1 channel holding the depth estimates.
    DepthMapPtr depth = std::make_unique<DepthMap>();
    auto& data = output_tensors_[0].quantize();

    int stride = output_tensors_[0].width();
    for (int y = 0; y < output_tensors_[0].height(); y++) {
        for (int x = 0; x < stride; x++) {
            (*depth)[y][x] = data[(y * stride) + x];
        }
    }

    return std::move(depth);
}
