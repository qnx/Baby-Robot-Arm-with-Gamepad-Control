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

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

#include "config.hpp"
#include "ml/depth_detection.hpp"
#include "ml/pose_detection.hpp"
#include "rpi_cam.hpp"
#include "util.hpp"

namespace joint {

class HailoJointController {
   public:
    using JointInfo = ml::PoseDetection::Point;
    using PoseInfo = ml::PoseDetection::Person;

    enum class PoseSelectionState {
        NO_ONE,
        SELECTING,
        SELECTED,
    };

    struct Snapshot {
        std::vector<PoseInfo> poses;
        PoseSelectionState selection_state = PoseSelectionState::NO_ONE;
        int selected_pose_id = -1;
        bool robot_enabled = true;
        ROIBounds roi = {0, 0, 100, 100};
        int width = 0;
        int height = 0;
    };

    using JointUpdateCB = std::function<void(const Snapshot &snapshot)>;

   private:
    enum class JointIndex : size_t {
        NOSE = 0,
        LEFT_EYE,
        RIGHT_EYE,
        LEFT_EAR,
        RIGHT_EAR,
        LEFT_SHOULDER,
        RIGHT_SHOULDER,
        LEFT_ELBOW,
        RIGHT_ELBOW,
        LEFT_WRIST,
        RIGHT_WRIST,
        LEFT_HIP,
        RIGHT_HIP,
        LEFT_KNEE,
        RIGHT_KNEE,
        LEFT_ANKLE,
        RIGHT_ANKLE
    };

   public:
    HailoJointController(const HailoJointConfig &cfg);
    ~HailoJointController();

    int init();
    int start();
    int stop();

    // Screen Info
    ROIBounds get_roi() const;
    int set_roi(const ROIBounds &roi);

    std::pair<int, int> get_screen_size() const;

    // Pose info
    std::vector<PoseInfo> get_poses();

    // TODO I think we should move this to PoseSelection
    /**
     * @brief Get info about the currently selected pose.
     *
     * @note if no pose is selected {NO_ONE, -1} will be returned so it is important
     *       to check the first value before blindly using the second.
     *
     * @return tuple containing the current state of the pose and the id of the pose if it exists.
     */
    std::pair<PoseSelectionState, int> get_selected_pose_info();

    /**
     * @brief Sets if we should be publishing results to ros2
     *
     * @param should_pub true to enable publishing, false otherwise.
     *
     */
    void set_publish_results(bool should_pub);
    bool get_publish_results() const;

    void add_joint_update_cb(JointUpdateCB joint_cb);

   private:
    // Callbacks
    void camera_callback(const cv::Mat &image);

    /**
     * @brief Detects if a persons hands are above their shoulders and fills in detected_poses array
     */
    bool hands_above_shoulders(joint::ml::PoseDetection::Person &person);

   private:
    HailoJointConfig cfg_;
    mutable std::mutex pose_mtx_{};
    mutable std::mutex model_mtx_{};
    std::vector<PoseInfo> poses_{};
    ROIBounds roi_ = {0, 0, 100, 100};

    bool publish_results_ = true;
    int selected_pose_ = -1;
    PoseSelectionState selection_state_ = PoseSelectionState::NO_ONE;

    /// The time in which selecting was started
    std::chrono::steady_clock::time_point detection_started{};

    /// Time in second before transitioning from selecting to selected in UI
    std::chrono::seconds detection_threshold_{3};

    std::unique_ptr<RpiCam> cam_{};
    std::unique_ptr<joint::ml::PoseDetection> pose_model_{};
    std::unique_ptr<joint::ml::DepthDetection> depth_model_{};
    std::vector<JointUpdateCB> joint_cbs_{};
};

}  // namespace joint
