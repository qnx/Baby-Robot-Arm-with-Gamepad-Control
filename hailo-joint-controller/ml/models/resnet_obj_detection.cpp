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
#include "ml/models/resnet_obj_detection.hpp"

#include <filesystem>
#include <fstream>

#include <spdlog/spdlog.h>

#include "util.hpp"

using joint::ml::model::ResNetObjDetection;
using joint::ml::ObjectDetection;

ResNetObjDetection::ResNetObjDetection() = default;

std::vector<ObjectDetection::DetectionResult> ResNetObjDetection::process(const cv::Mat &src_image) {
    if (src_image.empty()) {
        std::cerr << "Invalid_image" << std::endl;
        return {};
    }

    cv::Mat image;
    cv::resize(src_image, image, cv::Size(224, 224), 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    if (image.type() != CV_8UC3) {
        image.convertTo(image, CV_8UC3);
    }

    size_t input_size = inputs_.at(0).get_frame_size();
    if (image.total() * image.elemSize() != input_size) {
        std::cerr << "Image size mismatch" << std::endl;
        return {};
    }

    if (HAILO_SUCCESS != inputs_.at(0).write(hailort::MemoryView(image.data, input_size))) {
        std::cerr << "Failed to write input" << std::endl;
        return {};
    }

    output_tensors_[0].read();
    const auto& data = output_tensors_[0].quantize();

    // Fill in results and find max at the same time
    int max_class_id = 0;
    for (int i = 0; i < results_.size(); i++) {
        results_[i].class_id = i;
        results_[i].confidence = 100 * data[i];
        results_[i].name = cls_names_[i].c_str();

        if (results_[max_class_id].confidence < results_[i].confidence) {
            max_class_id = i;
        }
    }

    std::vector<DetectionResult> results;
    results.push_back(results_[max_class_id]);
    return results;
}
