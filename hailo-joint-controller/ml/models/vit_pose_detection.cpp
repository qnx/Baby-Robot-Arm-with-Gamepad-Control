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

#include "ml/models/vit_pose_detection.hpp"

#include <spdlog/spdlog.h>

#include "util.hpp"

using joint::ml::model::VitPoseDetection;

VitPoseDetection::VitPoseDetection(float joint_conf_threshold) : joint::ml::PoseDetection(joint_conf_threshold){};

hailort::Expected<std::vector<VitPoseDetection::Person>> VitPoseDetection::process(const cv::Mat &src_image) {
    if (src_image.empty()) {
        spdlog::warn("Invalid image(empty)");
        return hailort::make_unexpected(HAILO_INVALID_ARGUMENT);
    }

    // crop the image based on the roi
    int x_offset = src_image.cols * roi_[0] / 100.0;
    int y_offset = src_image.rows * roi_[1] / 100.0;
    int x_end = src_image.cols * roi_[2] / 100.0;
    int y_end = src_image.rows * roi_[3] / 100.0;
    spdlog::debug("ROI x_offset={} y_offset={} x_end={} y_end={}", x_offset, y_offset, x_end, y_end);

    cv::Mat cropped_image = src_image(cv::Rect(x_offset, y_offset, x_end - x_offset, y_end - y_offset));

    cv::Mat image;
    cv::resize(cropped_image, image, cv::Size(192, 256), 0, 0, cv::INTER_LINEAR);

    // The size difference between our source and processed frame
    float image_scale_x = static_cast<float>(cropped_image.cols) / image.cols;
    float image_scale_y = static_cast<float>(cropped_image.rows) / image.rows;
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    if (!image.isContinuous()) {
        image = image.clone();
    }

    size_t input_size = inputs_.at(0).get_frame_size();
    if (image.total() * image.elemSize() != input_size) {
        spdlog::warn("Invalid image(size mismatch)");
        return hailort::make_unexpected(HAILO_INVALID_ARGUMENT);
    }

    auto hailo_res = inputs_.at(0).write(hailort::MemoryView(image.data, input_size));
    if (HAILO_SUCCESS != hailo_res) {
        std::cerr << "Failed to write input" << std::endl;
        return hailort::make_unexpected(hailo_res);
    }

    inputs_.at(0).flush();
    for (auto &output : output_tensors_) {
        auto res = output.read();
        if (HAILO_SUCCESS != res) {
            spdlog::warn("Failed to read output {}", output.name());
            return hailort::make_unexpected(static_cast<hailo_status>(res));
        }
    }

    auto data = output_tensors_[0].quantize();

    std::vector<Person> people(1);
    int stride = (output_tensors_[0].width() * 17);
    for (int y = 0; y < output_tensors_[0].height(); y++) {
        for (int x = 0; x < output_tensors_[0].width(); x++) {
            for (int joint = 0; joint < num_joints_; joint++) {
                int idx = (y * stride) + (x * num_joints_) + joint;
                if (data[idx] > people[0].key_points[joint].confidence) {
                    people[0].key_points[joint].confidence = data[idx];

                    // Multiple by 4 for the output scale and + 2 to get the center of the result
                    people[0].key_points[joint].x = (x * 4 + 2) * image_scale_x + x_offset;
                    people[0].key_points[joint].y = (y * 4 + 2) * image_scale_y + y_offset;
                    people[0].key_points[joint].z = -1;
                }
            }
        }
    }

    // we use x_max and x_min of the first point as our baseline.
    // There is a slight risk here if the confidence of the first point is low
    // but lets pretend that won't happen :)
    static int s_id = -1;
    float score = 0;
    people[0].box.score = 0;
    people[0].box.x_max = people[0].key_points[0].x;
    people[0].box.x_min = people[0].key_points[0].x;
    people[0].box.y_min = people[0].key_points[0].y;
    people[0].box.y_min = people[0].key_points[0].y;

    for (int i = 0; i < num_joints_; i++) {
        people[0].key_points[i].name = joint_names_[i];
        score += people[0].key_points[i].confidence;
        if (people[0].key_points[i].confidence < joint_conf_threshold_) {
            // filter out bad points
            people[0].key_points[i].confidence = 0;
        } else {
            // If we are confident about the point use it for the box detection
            const auto &point = people[0].key_points[i];
            people[0].box.x_min = (people[0].box.x_min > point.x) ? point.x : people[0].box.x_min;
            people[0].box.x_max = (people[0].box.x_max < point.x) ? point.x : people[0].box.x_max;
            people[0].box.y_min = (people[0].box.y_min > point.y) ? point.y : people[0].box.y_min;
            people[0].box.y_max = (people[0].box.y_max < point.y) ? point.y : people[0].box.y_max;
        }
    }

    // Simulate to score by using the average of all our points
    score /= num_joints_;
    score *= 100;
    people[0].box.score = score;

    // Check if someone new was detected
    if (previous_score_ < person_score_threshold_ && people[0].box.score >= person_score_threshold_) {
        s_id++;
    }

    people[0].id = s_id;
    previous_score_ = people[0].box.score;

    if (people[0].box.score < person_score_threshold_) {
        people.clear();
        return {std::move(people)};  // no one detected
    }

    // Simulate a bit of padding on the box so it looks more realistic
    if (people[0].box.score > joint_conf_threshold_) {
        people[0].box.x_min -= (people[0].box.x_max - people[0].box.x_min) * 0.10;
        people[0].box.x_max += (people[0].box.x_max - people[0].box.x_min) * 0.10;
        people[0].box.y_min -= (people[0].box.y_max - people[0].box.y_min) * 0.10;
        people[0].box.y_max += (people[0].box.y_max - people[0].box.y_min) * 0.10;
    }

    return {std::move(people)};
}
