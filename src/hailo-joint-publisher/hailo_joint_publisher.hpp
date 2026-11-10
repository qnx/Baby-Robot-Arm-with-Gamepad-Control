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
#include <atomic>
#include <deque>
#include <thread>

#include <cjson/cJSON.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "config.hpp"
#include "detection.hpp"
#include "joint.hpp"

class HailoJointPublisher : public rclcpp::Node {
   public:
    HailoJointPublisher(const HailoJointPublisherConfig &cfg);
    virtual ~HailoJointPublisher();

    int publish(const std::vector<Person> &info);

   private:
    void read_loop();

    /// Parse pose snapshots published through QPP.
    void handle_hailo_message(const char *message);
    int parse_hailo_message(const char *message, Person &person_out);
    int parse_hailo_pose(cJSON *json, int expected_idx, Person &person_out);

    /**
     * @brief No fail wrapper for getting an object which displays an error message and aborts on failure
     *
     * @param object Object containing field
     * @param field no case sensitive field name
     * @param type expected type
     *
     * @return Object on success, otherwise doesn't return and abort() is called
     */
    cJSON *json_get_item_nf(cJSON *object, const char *field, int type);

    /**
     * @brief Waits for the catalog-backed QPP pose signal to become available.
     *
     * This will block until it is started or running_ = false
     */
    void wait_for_pose_signal();

    void wait_for_bra_signal();

    void update_joint_from_points(Point &root, Point &stem,
                                  Joint::JointType type);

    /**
     * @brief Adds joint to a message and update internal state
     *
     * @param joint_msg Message containing all joint angles
     * @param joint     Joint to set the angle too
     * @param angle     angle to set it joint if empty will use the old position (optional)
     *
     */
    void add_joint(sensor_msgs::msg::JointState &joint_msg, Joint::JointType joint);
    void add_joint(sensor_msgs::msg::JointState &joint_msg, Joint::JointType joint, float angle);

   private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_{};
    HailoJointPublisherConfig cfg_{};

    std::thread read_loop_{};
    int pipe_fds[2] = {-1, -1};
    int pose_fd_ = -1;
    int bra_fd_ = -1;
    std::atomic_bool running_{true};

    // Joint State
    Person prev_frame_{};
    Person first_frame_{};
    std::array<Joint, 7> joints_{};
    bool person_detected_ = false;
    std::chrono::steady_clock::time_point last_frame_time_;

    static constexpr float s_joint_confidence_ = 0.4;
    static constexpr float s_base_joint_deg_velocity = 20;

    std::array<std::string, 17> s_joint_map = {
        "nose",           "left_eye",   "right_eye",   "left_ear",   "right_ear",   "left_shoulder",
        "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist", "left_hip",
        "right_hip",      "left_knee",  "right_knee",  "left_ankle", "right_ankle"};
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
};
