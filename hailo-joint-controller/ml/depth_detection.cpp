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

#include "ml/depth_detection.hpp"
#include "ml/model_manager.hpp"

#include <spdlog/spdlog.h>


using joint::ml::DepthDetection;
using joint::ml::HailoModelManager;

DepthDetection::DepthDetection() = default;

DepthDetection::~DepthDetection() {
    spdlog::trace("DepthDetection Destructor");
    output_tensors_.clear();
    inputs_.clear();
    outputs_.clear();
    network_group_.reset();
    spdlog::trace("DepthDetection Destructor exit");
}

int DepthDetection::init(const std::string &hef_path) {

    // ensure the device is created before create.
    HailoModelManager::instance();

    auto hef = hailort::Hef::create(hef_path);
    if (!hef) {
        std::cerr << "Failed to load HEF: " << hef_path << std::endl;
        return 1;
    }

    auto configure_params = HailoModelManager::instance().dev_->create_configure_params(hef.value());
    if (!configure_params) {
        std::cerr << "Failed to create configure params" << std::endl;
        return 1;
    }

    auto network_groups = HailoModelManager::instance().dev_->configure(hef.value(), configure_params.value());
    if (!network_groups) {
        std::cerr << "Failed to configure network group" << std::endl;
        return 1;
    }

    if (1 != network_groups->size()) {
        std::cerr << "Invalid amount of network groups" << std::endl;
        return 1;
    }
    network_group_ = std::move(network_groups->front());

    spdlog::info("Creating input vstreams");
    auto input_params = network_group_->make_input_vstream_params(
        false, HAILO_FORMAT_TYPE_UINT8, HAILO_DEFAULT_VSTREAM_TIMEOUT_MS, HAILO_DEFAULT_VSTREAM_QUEUE_SIZE);
    if (!input_params) {
        std::cerr << "Failed to create input vstream params" << std::endl;
        return 1;
    }

    auto input_vstreams = hailort::VStreamsBuilder::create_input_vstreams(*network_group_, input_params.value());
    if (!input_vstreams) {
        std::cerr << "Failed to create input vstreams" << std::endl;
        return 1;
    }

    spdlog::info("Creating output vstreams");
    auto output_params = network_group_->make_output_vstream_params(
        true, HAILO_FORMAT_TYPE_AUTO, HAILO_DEFAULT_VSTREAM_TIMEOUT_MS, HAILO_DEFAULT_VSTREAM_QUEUE_SIZE);
    if (!output_params) {
        std::cerr << "Failed to create output vstream params" << std::endl;
        return 1;
    }

    auto output_vstreams = hailort::VStreamsBuilder::create_output_vstreams(*network_group_, output_params.value());
    if (!output_vstreams) {
        std::cerr << "Failed to create output vstreams" << std::endl;
        return 1;
    }

    // Now that everything is setup set our variables
    inputs_ = std::move(input_vstreams.value());
    outputs_ = std::move(output_vstreams.value());

    // Display info about the input and output stream
    spdlog::info("Input Streams: ");
    for (const auto &input : inputs_) {
        spdlog::info("stream {}: frame_size={} type={} order={}", input.name(), input.get_frame_size(),
                     (int)input.get_info().format.type, (int)input.get_info().format.order);
    }
    spdlog::info("Output Streams: ");
    for (auto &output : outputs_) {
        spdlog::info("stream {}: frame_size={} type={} order={} shape: {}x{}x{}", output.name(),
                     output.get_frame_size(), (int)output.get_info().format.type, (int)output.get_info().format.order,
                     output.get_info().shape.features, output.get_info().shape.width, output.get_info().shape.height);
        output_tensors_.emplace_back(output);
    }
    return EOK;
}
