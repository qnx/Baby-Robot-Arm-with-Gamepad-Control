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

#include "hailo_joint_controller.hpp"

#include "ml/models/fast_depth_detection.hpp"
#include "ml/models/vit_pose_detection.hpp"
using joint::HailoJointController;

HailoJointController::HailoJointController(const HailoJointConfig &cfg) : cfg_(cfg), roi_(cfg.region_of_interest){};
HailoJointController::~HailoJointController() {
    stop();
}

int HailoJointController::init() {
    cam_ = std::make_unique<RpiCam>(cfg_.camera_unit, cfg_.camera_data_unit);
    int res = cam_->init();
    if (res != EOK) {
        spdlog::error("Failed to initialize camera");
        return res;
    }

    pose_model_ = std::make_unique<ml::model::VitPoseDetection>(cfg_.detection_threshold_);
    res = pose_model_->init(cfg_.pose_model_path);
    if (res != EOK) {
        spdlog::error("Failed to initialize Hailo Pose detection");
        return res;
    }

    /**
     * The ROI is applied to the pose detection to figure out how much of the image should be
     * passed to the ML.
     */
    res = pose_model_->set_roi(cfg_.region_of_interest);
    if (res != EOK) {
        spdlog::error("Failed to set ROI with: {}", strerror(res));
        return res;
    }


    /**
     * The Depth detection is optional and is used for rotating hips but it has been found
     * that ML depth detection is unreliable and error prone so Titling the body provided the best
     * controls.
     */
    if (cfg_.depth_model_path.has_value()) {
        spdlog::info("Using depth model Z value will be calculated");
        depth_model_ = std::make_unique<ml::model::FastDepthDetection>();
        res = depth_model_->init(*cfg_.depth_model_path);
        if (res != EOK) {
            spdlog::error("Failed to initialize Hailo depth detection");
            return res;
        }
    }

    return EOK;
}

int HailoJointController::start() {
    if (cam_ == nullptr) {
        spdlog::error("Camera not initalized");
        return ENODEV;
    }

    int res = cam_->start(std::bind(&HailoJointController::camera_callback, this, std::placeholders::_1));
    if (res != EOK) {
        spdlog::error("Failed to start camera(id={})", static_cast<int>(cfg_.camera_unit));
        return res;
    }

    return EOK;
}

int HailoJointController::stop() {
    if (cam_ != nullptr) {
        cam_->stop();
    }
    return EOK;
}

joint::ROIBounds HailoJointController::get_roi() const {
    std::lock_guard<std::mutex> lk(pose_mtx_);
    return roi_;
}

int HailoJointController::set_roi(const ROIBounds &roi) {
    int res;
    {
        std::lock_guard<std::mutex> lk(model_mtx_);
        res = pose_model_->set_roi(roi);
    }
    if (res == EOK) {
        std::lock_guard<std::mutex> lk(pose_mtx_);
        roi_ = roi;
    }
    return res;
}

std::vector<HailoJointController::PoseInfo> HailoJointController::get_poses() {
    std::lock_guard<std::mutex> lk(pose_mtx_);
    return poses_;
}

std::pair<HailoJointController::PoseSelectionState, int> HailoJointController::get_selected_pose_info() {
    std::lock_guard<std::mutex> lk(pose_mtx_);
    return std::make_pair(selection_state_, selected_pose_);
}

std::pair<int, int> HailoJointController::get_screen_size() const {
    return cam_->get_screen_size();
}

bool HailoJointController::hands_above_shoulders(joint::ml::PoseDetection::Person &person) {
    // First find their shoulders

    const auto &left_shoulder = person.key_points[(int)JointIndex::LEFT_SHOULDER];
    const auto &right_shoulder = person.key_points[(int)JointIndex::RIGHT_SHOULDER];
    const auto &left_elbow = person.key_points[(int)JointIndex::LEFT_ELBOW];
    const auto &right_elbow = person.key_points[(int)JointIndex::RIGHT_ELBOW];
    // Ensure the points are valid
    if (left_shoulder.confidence < cfg_.detection_threshold_ || right_shoulder.confidence < cfg_.detection_threshold_
        || left_elbow.confidence < cfg_.detection_threshold_ || right_elbow.confidence < cfg_.detection_threshold_) {
        return false;
    }

    // Detect both arms and add them to the list
    size_t poses_detected = person.detected_poses.size();
    if (left_elbow.y < left_shoulder.y) {
        person.detected_poses.push_back("left_arm_up");
    }

    if (right_elbow.y < right_shoulder.y) {
        person.detected_poses.push_back("right_arm_up");
    }

    // If the list increated by 2 we know both arms are up.
    return (person.detected_poses.size() == (poses_detected + 2));
}

void HailoJointController::camera_callback(const cv::Mat &image) {
    spdlog::trace("HailoJointController::camera_callback");
    auto poses = [&]() {
        std::lock_guard<std::mutex> lk(model_mtx_);
        return pose_model_->process(image);
    }();

    if (!poses.has_value()) {
        spdlog::warn("Failed to process frame with: {}", hailo_get_status_message(poses.status()));
        return;
    }

    // Check if the person we have selected is still in the frame This way we
    // don't check for poses unless we have to
    int selected_pose = selected_pose_;
    auto selection_state = selection_state_;
    if (selected_pose != -1) {
        auto pose =
            std::find_if(poses->begin(), poses->end(), [&](const auto &pose) { return pose.id == selected_pose; });

        if (pose == poses->end()) {
            spdlog::info("Person {} no longer in frame", selected_pose);
            selected_pose = -1;
            selection_state = PoseSelectionState::NO_ONE;
        } else if (selection_state == PoseSelectionState::SELECTING) {
            // If the state is still selecting check if they have waited long enough to be selected
            if (!hands_above_shoulders(*pose)) {
                selected_pose = -1;
                selection_state = PoseSelectionState::NO_ONE;
            } else if (std::chrono::steady_clock::now() - detection_started > detection_threshold_) {
                selection_state = PoseSelectionState::SELECTED;
            }
        }
    }

    // Do some rough pose detection to see if we are trying to select someone
    if (selected_pose == -1) {
        for (auto &pose : *poses) {
            if (hands_above_shoulders(pose)) {
                selection_state = PoseSelectionState::SELECTING;
                selected_pose = pose.id;
                spdlog::info("Found pose {} with hands up switching to selecting", selected_pose);
                detection_started = std::chrono::steady_clock::now();
                break;
            }
        }
    }

    // Apply depth to joints, only running on the bounding box (this only happens if we provided a depth model)
    if (selection_state == PoseSelectionState::SELECTED) {
        auto &pose =
            *std::find_if(poses->begin(), poses->end(), [&](const auto &pose) { return pose.id == selected_pose; });

        // Check if they want to be "released" :) by them putting their hands together above their head
        // Left arm and right elbow away from shoulder
        // Left and right wrist between shoulders
        const auto &left_shoulder = pose.key_points[(int)JointIndex::LEFT_SHOULDER];
        const auto &right_shoulder = pose.key_points[(int)JointIndex::RIGHT_SHOULDER];
        const auto &left_elbow = pose.key_points[(int)JointIndex::LEFT_ELBOW];
        const auto &right_elbow = pose.key_points[(int)JointIndex::RIGHT_ELBOW];
        const auto &left_wrist = pose.key_points[(int)JointIndex::LEFT_WRIST];
        const auto &right_wrist = pose.key_points[(int)JointIndex::RIGHT_WRIST];
        const auto &left_hip = pose.key_points[(int)JointIndex::LEFT_HIP];
        const auto &right_hip = pose.key_points[(int)JointIndex::RIGHT_HIP];
        auto conf = cfg_.detection_threshold_;

        // Note image if flipped meaning so signs are flipped for x
        bool confident_cross = left_wrist.confidence >= conf && left_elbow.confidence >= conf
            && left_shoulder.confidence >= conf && right_wrist.confidence >= conf && right_elbow.confidence >= conf
            && right_shoulder.confidence >= conf;
        bool confident_drop = left_wrist.confidence >= conf && left_hip.confidence >= conf
            && right_wrist.confidence >= conf && right_hip.confidence >= conf;

        if ((confident_cross &&  // We are confident this position is correct
             left_elbow.x > left_shoulder.x && left_elbow.y < left_shoulder.y &&      // left elbow
             right_elbow.x < right_shoulder.x && right_elbow.y < right_shoulder.y &&  // right elbow
             left_wrist.x < left_shoulder.x && left_wrist.y < left_elbow.y &&         // left wrist
             right_wrist.x > right_shoulder.x && right_wrist.y < right_elbow.y)       // right wrist
            || (confident_drop && left_wrist.y > left_hip.y && right_wrist.y > right_hip.y)) {
            spdlog::info("Deselected pose {}", pose.id);
            selection_state = PoseSelectionState::NO_ONE;
            selected_pose = -1;
        } else if (depth_model_ != nullptr) {
            auto depth = depth_model_->process(image);
            if (depth != nullptr) {
                float image_scale_x = static_cast<float>(image.cols) / depth.get()[0].size();
                float image_scale_y = static_cast<float>(image.rows) / depth->size();
                for (auto &joint : pose.key_points) {
                    int estimated_x =
                        std::max(0, std::min((int)((joint.x) / image_scale_x), (int)depth.get()[0].size()));
                    int estimated_y = std::max(0, std::min((int)((joint.y) / image_scale_y), (int)depth->size()));
                    joint.z = (*depth)[estimated_y][estimated_x];
                }
            } else {
                spdlog::warn("Failed to detect depth");
            }
        }
    }

    // Finally submit the result and run our callbacks
    Snapshot snapshot;
    std::vector<JointUpdateCB> callbacks;
    {
        std::unique_lock<std::mutex> lk(pose_mtx_);
        // Update our internal pose we do this in 2 steps to ensure the mutex isn't locked when we process
        // the image.
        poses_ = std::move(poses.value());
        selected_pose_ = selected_pose;
        selection_state_ = selection_state;

        snapshot.poses = poses_;
        snapshot.selection_state = selection_state_;
        snapshot.selected_pose_id = selected_pose_;
        snapshot.robot_enabled = publish_results_;
        snapshot.roi = roi_;
        const auto screen = cam_->get_screen_size();
        snapshot.width = screen.first;
        snapshot.height = screen.second;
        callbacks = joint_cbs_;
    }

    // IPC callbacks may serialize and write data, so never run them while the
    // controller state lock is held.
    for (const auto &cb : callbacks) {
        cb(snapshot);
    }
}

void HailoJointController::set_publish_results(bool should_pub) {
    std::lock_guard<std::mutex> lk(pose_mtx_);
    publish_results_ = should_pub;
    spdlog::info("Setting pose output enabled={}", publish_results_);
}

bool HailoJointController::get_publish_results() const {
    std::lock_guard<std::mutex> lk(pose_mtx_);
    return publish_results_;
}

void HailoJointController::add_joint_update_cb(JointUpdateCB joint_cb) {
    std::lock_guard<std::mutex> lk(pose_mtx_);
    joint_cbs_.emplace_back(joint_cb);
}
