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
#include "hailo_joint_publisher.hpp"

#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <random>

#include <cjson/cJSON.h>
#include <fcntl.h>
#include <libgen.h>
#include <poll.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

HailoJointPublisher::HailoJointPublisher(const HailoJointPublisherConfig &cfg) :
    rclcpp::Node("hailo_joint_publisher"), cfg_(cfg) {
    RCLCPP_INFO(this->get_logger(), "Starting HailoJointPublisher");
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(cfg_.ros2_joint_topic, 10);
    if (pipe(pipe_fds) == -1) {
        RCLCPP_FATAL(this->get_logger(), "Failed to create pipe exiting.");
        rclcpp::shutdown();
    }

    pose_fd_ = open(cfg_.pose_info_path.c_str(), O_RDONLY);
    if (pose_fd_ == -1) {
        RCLCPP_WARN(this->get_logger(), "Failed to open QPP pose signal %s", cfg_.pose_info_path.c_str());
    }

    bra_fd_ = open(cfg_.bra_enabled_path.c_str(), O_RDONLY);
    if (bra_fd_ == -1) {
        RCLCPP_WARN(this->get_logger(), "Failed to open QPP bra enabled signal %s", cfg_.pose_info_path.c_str());
    }

    read_loop_ = std::thread(&HailoJointPublisher::read_loop, this);
}

HailoJointPublisher::~HailoJointPublisher() {
    int exit_msg = 1;
    write(pipe_fds[1], &exit_msg, sizeof(int));
    running_ = false;
    read_loop_.join();
    if (pose_fd_ != -1) close(pose_fd_);
    if (bra_fd_ != -1) close(bra_fd_);
    close(pipe_fds[1]);
    close(pipe_fds[0]);
}

void HailoJointPublisher::wait_for_pose_signal() {
    while (pose_fd_ == -1 && running_) {
        if (std::filesystem::exists(cfg_.pose_info_path)) {
            pose_fd_ = open(cfg_.pose_info_path.c_str(), O_RDONLY);
            if (pose_fd_ == -1) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open QPP pose signal with: %s", strerror(errno));
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            RCLCPP_INFO(this->get_logger(), "Connected to QPP pose signal");
            break;
        }

        RCLCPP_WARN(this->get_logger(), "Waiting for QPP pose signal %s", cfg_.pose_info_path.c_str());
        while (running_ && waitfor_attach(cfg_.pose_info_path.c_str(), 2000) != 0) {
            RCLCPP_DEBUG(this->get_logger(), "Waiting");
        }
    }
}

void HailoJointPublisher::wait_for_bra_signal() {
    while (bra_fd_ == -1 && running_) {
        if (std::filesystem::exists(cfg_.bra_enabled_path)) {
            bra_fd_ = open(cfg_.bra_enabled_path.c_str(), O_RDONLY);
            if (bra_fd_ == -1) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open QPP bra enabled signal with: %s", strerror(errno));
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            RCLCPP_INFO(this->get_logger(), "Connected to QPP bra enabled signal");
            break;
        }

        RCLCPP_WARN(this->get_logger(), "Waiting for QPP bra enabled signal %s", cfg_.bra_enabled_path.c_str());
        while (running_ && waitfor_attach(cfg_.bra_enabled_path.c_str(), 2000) != 0) {
            RCLCPP_DEBUG(this->get_logger(), "Waiting");
        }
    }
}


void HailoJointPublisher::update_joint_from_points(Point &root, Point &stem, Joint::JointType type) {
    auto &joint = joints_[(int)type];

    joint.type = type;
    joint.len_sq = (root.x - stem.x) * (root.x - stem.x) + (root.y - stem.y) * (root.y - stem.y);

    if (!person_detected_) {
        joint.angle_rad = 0;
        joint.start_len_sq = joint.len_sq;
        joint.root = root;
        joint.stem = stem;
    }

    if (root.confidence >= s_joint_confidence_ && stem.confidence >= s_joint_confidence_) {
        joint.root_points.push_back(root);
        joint.stem_points.push_back(stem);
    } else {
        return;
    }

    if (joint.root_points.size() > 10) {
        joint.root_points.pop_front();
        joint.stem_points.pop_front();
    }
}

void HailoJointPublisher::read_loop() {
    struct pollfd pollfd[3];
    pollfd[0].fd = pipe_fds[0];
    pollfd[0].events = POLLIN;
    pollfd[1].fd = pose_fd_;
    pollfd[1].events = POLLIN;
    pollfd[2].fd = bra_fd_;
    pollfd[2].events = POLLIN;

    std::array<char, 16384> read_buf{};
    bool enabled = false;

    // Read initial enabled state
    if (bra_fd_ != -1) {
        bool initial = false;
        if (read(bra_fd_, &initial, sizeof(initial)) == sizeof(initial)) {
            enabled = initial;
        }
    }

    while (running_) {
        if (bra_fd_ == -1) {
            wait_for_bra_signal();
            if (!running_) return;
            pollfd[2].fd = bra_fd_;
        }

        if (pose_fd_ == -1) {
            wait_for_pose_signal();
            if (!running_) return;
            pollfd[1].fd = pose_fd_;
        }

        int res = poll(pollfd, 3, -1);
        if (res < 0) {
            if (errno == EINTR) continue;
            RCLCPP_WARN(this->get_logger(), "poll failed with: %s", strerror(errno));
            break;
        }

        // Exit requested
        if ((pollfd[0].revents & POLLIN) != 0) {
            RCLCPP_INFO(this->get_logger(), "Exit Requested");
            return;
        }

        if ((pollfd[2].revents & POLLIN) != 0) {
            bool val = false;
            if (read(bra_fd_, &val, sizeof(val)) == sizeof(val)) {
                enabled = val;
                RCLCPP_INFO(this->get_logger(), "BRA enabled=%d", enabled);
            }
        }

        if ((pollfd[2].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            RCLCPP_WARN(this->get_logger(), "QPP bra enabled signal disconnected");
            close(bra_fd_);
            bra_fd_ = -1;
            enabled = false;
            continue;
        }

        if (!enabled) continue;

        // Pose data arrived
        if ((pollfd[1].revents & POLLIN) != 0) {
            res = read(pose_fd_, read_buf.data(), read_buf.size() - 1);
            if (res < 0) {
                RCLCPP_WARN(this->get_logger(), "read from %s failed with: %s",
                            cfg_.pose_info_path.c_str(), strerror(errno));
                close(pose_fd_);
                pose_fd_ = -1;
                continue;
            }
            read_buf[static_cast<size_t>(res)] = '\0';
            handle_hailo_message(read_buf.data());
        }

        if ((pollfd[1].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            RCLCPP_WARN(this->get_logger(), "QPP pose signal disconnected");
            close(pose_fd_);
            pose_fd_ = -1;
        }
    }
}

void HailoJointPublisher::handle_hailo_message(const char *message) {
    /**
     * Parse out the data.
     *
     * In order to do this we will take a couple steps.
     * 1. We check if someone has been detected or not
     *     a. If no one has been detect just ignore the output
     * 2. Get the index of the person detected
     * 3. Search the list to find the correct detected person with index
     * 4. Parse out their joint data
     * 5. If this is the first time we detect them we will save their info in order to
     *
     * Later on we will use the joint data to control the robot based on the joint in the real world
     **/

    // Grab when the frame came in.
    auto now = std::chrono::steady_clock::now();

    Person person;
    auto res = parse_hailo_message(message, person);
    if (res == EAGAIN) {
        // No one is there or the robot isn't detected
        if (person_detected_) {
            // reset the robot
            person_detected_ = false;
            sensor_msgs::msg::JointState joint_msg;
            add_joint(joint_msg, Joint::JointType::BASE, 0);
            add_joint(joint_msg, Joint::JointType::WRIST, 0);
            add_joint(joint_msg, Joint::JointType::GRIPPER, 0);
            add_joint(joint_msg, Joint::JointType::SHOULDER, 0);
            add_joint(joint_msg, Joint::JointType::ELBOW, 0);
            add_joint(joint_msg, Joint::JointType::HAND, 0);
            publisher_->publish(joint_msg);

            // 20% chance the robot will clap for you
            if (rand() % 5 == 0) {
                // Have the robot clap for you
                for (int i = 0; i < 10; i++) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(400));
                    sensor_msgs::msg::JointState joint_msg;
                    add_joint(joint_msg, Joint::JointType::GRIPPER, (i % 2 == 1) ? M_PI_2 : -M_PI_2);
                    add_joint(joint_msg, Joint::JointType::HAND, (i % 2 == 1) ? -M_PI / 8 : M_PI / 8);
                    publisher_->publish(joint_msg);
                }
                // Reset the gripper to avoid stressing the servo
                {
                    add_joint(joint_msg, Joint::JointType::GRIPPER, 0);
                    publisher_->publish(joint_msg);
                }
            }
        }
        RCLCPP_DEBUG(this->get_logger(), "robot not enabled");
        return;
    } else if (res != EOK) {
        RCLCPP_WARN(this->get_logger(), "Failed to read Hailo message: %s", strerror(res));
        return;
    }

    sensor_msgs::msg::JointState joint_msg;

    if (!person_detected_) {
        person_detected_ = true;
        last_frame_time_ = now;

        // The base as it uses "Thank controls"
        // and wrist as we won't use it
        add_joint(joint_msg, Joint::JointType::BASE, 0);
        add_joint(joint_msg, Joint::JointType::WRIST, 0);
        publisher_->publish(joint_msg);

        // Wait for joints to center
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return;
    }

    // First update the base to see if there is rotation. as if there is only trigger the base movement
    update_joint_from_points(person.key_points[(int)JointIndex::LEFT_SHOULDER],
                             person.key_points[(int)JointIndex::RIGHT_SHOULDER], Joint::JointType::BASE);

    // Base
    {
        bool base_moved = false;
#if 0
        // Calculate rotation.
        // Shoulder rotation implementations
        auto left_shoulder_z_avg = joints_[(int)Joint::JointType::BASE].get_average_root().z;
        auto right_shoulder_z_avg = joints_[(int)Joint::JointType::BASE].get_average_stem().z;
        if (std::fabs(left_shoulder_z_avg - right_shoulder_z_avg) < 0.2) {
            // roughly centered
        } else if (left_shoulder_z_avg < right_shoulder_z_avg) {
            RCLCPP_INFO(this->get_logger(), "rotated left");
            joints_[(int)Joint::JointType::BASE].angle_rad -= 0.05;
            base_moved = true;
            if (joints_[(int)Joint::JointType::BASE].angle_rad < -3 * M_PI_4) {
                joints_[(int)Joint::JointType::BASE].angle_rad = -3 * M_PI_4;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "rotated right");
            joints_[(int)Joint::JointType::BASE].angle_rad += 0.05;
            base_moved = true;
            if (joints_[(int)Joint::JointType::BASE].angle_rad > 3 * M_PI_4) {
                joints_[(int)Joint::JointType::BASE].angle_rad = 3 * M_PI_4;
            }
        }
#else
        // Shoulder tilt implementation
        const auto &left_shoulder = joints_[(int)Joint::JointType::BASE].get_average_root();
        const auto &right_shoulder = joints_[(int)Joint::JointType::BASE].get_average_stem();

        // Distance from nose to centerpoint in shoulder
        auto mid_point_x = (left_shoulder.x + right_shoulder.x) / 2.0;
        auto mid_point_y = (left_shoulder.y + right_shoulder.y) / 2.0;

        const auto &nose = joints_[(int)Joint::JointType::FACE].get_average_root();

        auto distance_x = nose.x - mid_point_x;
        auto distance_y = nose.y - mid_point_y;
        auto distance = std::sqrt(distance_x * distance_x + distance_y * distance_y);

        auto time_velocity =
            (float)(now - last_frame_time_).count() / std::chrono::nanoseconds(std::chrono::seconds(1)).count();
        if (std::fabs(left_shoulder.y - right_shoulder.y) < distance / 3.3) {
            // roughly centered
        } else if (left_shoulder.y > right_shoulder.y) {
            base_moved = true;
            joints_[(int)Joint::JointType::BASE].angle_rad -= s_base_joint_deg_velocity * time_velocity * M_PI / 180;
            if (joints_[(int)Joint::JointType::BASE].angle_rad < -3 * M_PI_4) {
                joints_[(int)Joint::JointType::BASE].angle_rad = -3 * M_PI_4;
            }
        } else {
            base_moved = true;
            joints_[(int)Joint::JointType::BASE].angle_rad += s_base_joint_deg_velocity * time_velocity * M_PI / 180;
            if (joints_[(int)Joint::JointType::BASE].angle_rad > 3 * M_PI_4) {
                joints_[(int)Joint::JointType::BASE].angle_rad = 3 * M_PI_4;
            }
        }
#endif

        // If the base moves we don't want to update any other joints to make movement slightly easier
        add_joint(joint_msg, Joint::JointType::BASE);
        if (base_moved) {
            last_frame_time_ = now;
            publisher_->publish(joint_msg);
            return;
        }
    }

    update_joint_from_points(person.key_points[(int)JointIndex::LEFT_SHOULDER],
                             person.key_points[(int)JointIndex::LEFT_ELBOW], Joint::JointType::SHOULDER);
    update_joint_from_points(person.key_points[(int)JointIndex::LEFT_ELBOW],
                             person.key_points[(int)JointIndex::LEFT_WRIST], Joint::JointType::ELBOW);
    update_joint_from_points(person.key_points[(int)JointIndex::RIGHT_SHOULDER],
                             person.key_points[(int)JointIndex::RIGHT_ELBOW], Joint::JointType::HAND);
    update_joint_from_points(person.key_points[(int)JointIndex::RIGHT_ELBOW],
                             person.key_points[(int)JointIndex::RIGHT_WRIST], Joint::JointType::GRIPPER);

    // Fake Joint for nose
    update_joint_from_points(person.key_points[(int)JointIndex::NOSE], person.key_points[(int)JointIndex::NOSE],
                             Joint::JointType::FACE);

    // Shoulder
    {
        auto angle =
            joints_[(int)Joint::JointType::SHOULDER].get_2d_angle_from_joint(joints_[(int)Joint::JointType::BASE])
            - M_PI;

        /**
         * For the shoulder the servo goes -3pi/4 -> 3pi/4
         */
        if (angle > -M_PI_2 && angle < M_PI_2 /* 180*/) {
            // -180 to go from 90-270 -> -90, 90
            add_joint(joint_msg, Joint::JointType::SHOULDER, -angle);
        } else {
            add_joint(joint_msg, Joint::JointType::SHOULDER);
        }
    }

    // Elbow
    {
        auto angle =
            joints_[(int)Joint::JointType::ELBOW].get_2d_angle_from_joint(joints_[(int)Joint::JointType::SHOULDER]);

        // Convert range from 180-360 -> 0->180 and reverse the angle
        if (angle > M_PI) {
            add_joint(joint_msg, Joint::JointType::ELBOW, 2 * M_PI - angle);
        } else {
            add_joint(joint_msg, Joint::JointType::ELBOW);
        }
    }

    // Wrist (we don't use this so always set to 0)
    add_joint(joint_msg, Joint::JointType::WRIST, 0);

    // Hand
    {
        /**
         * For the hand we want to make it relative to the shoulder using your right hand.
         * your right hand will be in quadrant 1(0-90) and 4 (270-360) meaning we need to treat them in 2 cases
         *
         * For the first case < 90 we need  to flip the sign to be -90 to 0
         *   90                   -90
         *   ^                    ^
         *   |           -->      |
         *   |--> 0               |--> 0
         *
         * which can be done by taking the angle and subtracting 90
         *
         * For the second case we need to get the relative angle between
         *
         *    |---- 0
         *    | \      <-- need this angle + 90
         *    |  \
         *   270
         *
         * which can be done by subtracting the angle from 360 and adding 90 to get the range -90, 90
         */
        auto angle = joints_[(int)Joint::JointType::HAND].get_2d_angle_from_joint(joints_[(int)Joint::JointType::BASE]);
        if (angle < M_PI_2 /* 90 */) {  // -90 range
            add_joint(joint_msg, Joint::JointType::HAND, angle);

        } else if (angle > 3 * M_PI_2 /* 270 */) {  // 0 - 90
            add_joint(joint_msg, Joint::JointType::HAND, -(2 * M_PI - angle));
        } else {
            add_joint(joint_msg, Joint::JointType::HAND);
        }
    }

    // Gripper
    {
        auto angle =
            joints_[(int)Joint::JointType::GRIPPER].get_2d_angle_from_joint(joints_[(int)Joint::JointType::HAND]);
        add_joint(joint_msg, Joint::JointType::GRIPPER, angle);

        if (angle < M_PI_2) {
            add_joint(joint_msg, Joint::JointType::GRIPPER, M_PI_2);
        } else if (angle < M_PI) {
            add_joint(joint_msg, Joint::JointType::GRIPPER, -M_PI_2);
        } else {
            add_joint(joint_msg, Joint::JointType::GRIPPER);
        }
    }

#if 0
    std::cout << "Joints: {";
    for (size_t i = 0; i < joint_msg.name.size(); i++) {
        std::cout << "(" << joint_msg.name[i] << ", " << joint_msg.position[i] << "), ";
    }
    std::cout << "}" << std::endl;
#endif
    last_frame_time_ = now;

    publisher_->publish(joint_msg);
}

void HailoJointPublisher::add_joint(sensor_msgs::msg::JointState &joint_msg, Joint::JointType joint) {
    add_joint(joint_msg, joint, joints_[(int)joint].angle_rad);
}

void HailoJointPublisher::add_joint(sensor_msgs::msg::JointState &joint_msg, Joint::JointType joint, float angle) {
    switch (joint) {
        case Joint::JointType::BASE:
            joint_msg.name.push_back("base");
            break;
        case Joint::JointType::SHOULDER:
            joint_msg.name.push_back("shoulder");
            break;
        case Joint::JointType::ELBOW:
            joint_msg.name.push_back("elbow");
            break;
        case Joint::JointType::WRIST:
            joint_msg.name.push_back("wrist");
            break;
        case Joint::JointType::HAND:
            joint_msg.name.push_back("hand");
            break;
        case Joint::JointType::GRIPPER:
            joint_msg.name.push_back("gripper");
            break;
    }
    joint_msg.position.push_back(angle);
    joints_[(int)joint].angle_rad = angle;
}

int HailoJointPublisher::parse_hailo_message(const char *message, Person &person_out) {
    cJSON *j = cJSON_Parse(message);
    int res = EOK;
    if (j == nullptr) {
        res = EINVAL;
        RCLCPP_WARN(this->get_logger(), "Failed to parse pose.info JSON");
        return res;
    }

    // Poor mans goto so we can always free the json after it has been parsed
    do {
        // First ensure the robot is even enabled and if it isn't just skip the full message
        if (cJSON_IsFalse(json_get_item_nf(j, "robot_enabled", cJSON_True | cJSON_False))) {
            res = EAGAIN;
            break;
        }

        // There is no pose selected so skip the message.
        if (strcmp("selected", json_get_item_nf(j, "selection_state", cJSON_String)->valuestring) != 0) {
            res = EAGAIN;
            break;
        }

        // When searching we will only parse the selected pose
        int selected_pose = json_get_item_nf(j, "selected_pose_id", cJSON_Number)->valueint;

        // Parse the poses
        cJSON *person_json = nullptr;
        cJSON_ArrayForEach(person_json, json_get_item_nf(j, "poses", cJSON_Array)) {
            res = parse_hailo_pose(person_json, selected_pose, person_out);
            if ((res != EOK) || (person_out.id == selected_pose)) {
                break;
            }
        }
    } while (0);

    cJSON_free(j);
    return res;
}

int HailoJointPublisher::parse_hailo_pose(cJSON *json, int expected_idx, Person &person_out) {
    person_out.id = json_get_item_nf(json, "id", cJSON_Number)->valueint;
    if (person_out.id != expected_idx) {
        return EOK;
    }

    // Parse Joints
    cJSON *joint = nullptr;
    cJSON_ArrayForEach(joint, cJSON_GetObjectItem(json, "joints")) {
        char *name = json_get_item_nf(joint, "name", cJSON_String)->valuestring;
        int x = json_get_item_nf(joint, "x", cJSON_Number)->valueint;
        int y = json_get_item_nf(joint, "y", cJSON_Number)->valueint;

        float z = -1;
        if (cJSON_HasObjectItem(joint, "z")) {
            z = json_get_item_nf(joint, "z", cJSON_Number)->valuedouble;
        }
        float conf = json_get_item_nf(joint, "confidence", cJSON_Number)->valuedouble;

        // Map the joint
        size_t joint_idx = 0;
        for (joint_idx = 0; joint_idx < s_joint_map.size(); joint_idx++) {
            if (s_joint_map[joint_idx] == name) {
                break;
            }
        }

        if (joint_idx == s_joint_map.size()) {
            RCLCPP_WARN(this->get_logger(), "unknown joint name %s", name);
            return EINVAL;
        }

        person_out.key_points[joint_idx].confidence = conf;
        person_out.key_points[joint_idx].x = x;
        person_out.key_points[joint_idx].y = y;
        person_out.key_points[joint_idx].z = z;
    }

    return EOK;
}

cJSON *HailoJointPublisher::json_get_item_nf(cJSON *object, const char *field, int type) {
    auto item = cJSON_GetObjectItem(object, field);
    if (item == nullptr) {
        RCLCPP_FATAL(this->get_logger(), "Field to find item %s", field);
        abort();
    }

    // Check all the types to ensure we find at least one expected type
    if ((type & item->type) == 0) {
        RCLCPP_FATAL(this->get_logger(), "item=%s has invalid type %d expected %d", field, item->type, type);
        abort();
    }
    return item;
}
