/**
 * Copyright (c) 2026, BlackBerry Limited. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */
#include "qpp/hailo_joint_qpp.hpp"

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <functional>

#include <cjson/cJSON.h>
#include <fcntl.h>
#include <poll.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

using joint::qpp::HailoJointQpp;
using joint::HailoJointController;

namespace {

std::string signal_path(const std::string &branch, const char *leaf) {
    return branch + "/" + leaf;
}

int open_signal(const std::string &path, int flags) {
    int fd = open(path.c_str(), flags);
    if (fd == -1) {
        int error = errno;
        spdlog::error("Failed to open QPP signal {}: {}", path, strerror(error));
        return -error;
    }
    return fd;
}

ssize_t write_value(int fd, const void *data, size_t size, const char *name) {
    ssize_t written = write(fd, data, size);
    if (written == static_cast<ssize_t>(size)) {
        return written;
    }

    if (written == -1) {
        spdlog::error("Failed to publish QPP signal {}: {}", name, strerror(errno));
    } else {
        spdlog::error("Short write to QPP signal {}: expected {}, wrote {}", name, size, written);
    }
    return written;
}

ssize_t write_string(int fd, const std::string &value, const char *name) {
    return write_value(fd, value.c_str(), value.size() + 1, name);
}

const char *selection_state_name(HailoJointController::PoseSelectionState state) {
    switch (state) {
        case HailoJointController::PoseSelectionState::SELECTING:
            return "selecting";
        case HailoJointController::PoseSelectionState::SELECTED:
            return "selected";
        default:
        case HailoJointController::PoseSelectionState::NO_ONE:
            return "noone";
    }
}

void populate_pose_json(cJSON *pose_json, const HailoJointController::PoseInfo &pose) {
    cJSON_AddNumberToObject(pose_json, "id", pose.id);

    cJSON *box = cJSON_AddObjectToObject(pose_json, "person");
    cJSON_AddNumberToObject(box, "confidence", pose.box.score);
    cJSON_AddNumberToObject(box, "x", pose.box.x_min);
    cJSON_AddNumberToObject(box, "y", pose.box.y_min);
    cJSON_AddNumberToObject(box, "width", pose.box.x_max - pose.box.x_min);
    cJSON_AddNumberToObject(box, "height", pose.box.y_max - pose.box.y_min);

    cJSON *detected = cJSON_AddArrayToObject(pose_json, "pose_detected");
    for (const auto &name : pose.detected_poses) {
        cJSON_AddItemToArray(detected, cJSON_CreateString(name.c_str()));
    }

    cJSON *joints = cJSON_AddArrayToObject(pose_json, "joints");
    for (const auto &joint : pose.key_points) {
        cJSON *joint_json = cJSON_CreateObject();
        cJSON_AddStringToObject(joint_json, "name", joint.name.c_str());
        cJSON_AddNumberToObject(joint_json, "x", joint.x);
        cJSON_AddNumberToObject(joint_json, "y", joint.y);
        if (joint.z != -1) {
            cJSON_AddNumberToObject(joint_json, "z", joint.z);
        }
        cJSON_AddNumberToObject(joint_json, "confidence", joint.confidence);
        cJSON_AddItemToArray(joints, joint_json);
    }
}

std::string print_json(cJSON *root) {
    char *raw = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (raw == nullptr) {
        return {};
    }

    std::string result(raw);
    cJSON_free(raw);
    return result;
}

std::string serialize_snapshot(const HailoJointController::Snapshot &snapshot) {
    cJSON *root = cJSON_CreateObject();
    if (root == nullptr) {
        return {};
    }

    cJSON_AddNumberToObject(root, "width", snapshot.width);
    cJSON_AddNumberToObject(root, "height", snapshot.height);

    // UI edges index the COCO-17 keypoints array in detections/pose1.
    // Keep the legacy snapshot fields below for the ROS publisher.
    static constexpr int edges[][2] = {
        {0, 1}, {0, 2}, {1, 3}, {2, 4}, {5, 6}, {5, 7}, {7, 9}, {6, 8},
        {8, 10}, {5, 11}, {6, 12}, {11, 12}, {11, 13}, {13, 15}, {12, 14}, {14, 16}
    };
    cJSON *mapping = cJSON_AddArrayToObject(root, "keypoints_mapping");
    for (const auto &edge : edges) {
        cJSON_AddItemToArray(mapping, cJSON_CreateIntArray(edge, 2));
    }

    cJSON_AddNumberToObject(root, "selected_pose_id", snapshot.selected_pose_id);
    cJSON_AddStringToObject(root, "selection_state", selection_state_name(snapshot.selection_state));
    cJSON_AddBoolToObject(root, "robot_enabled", snapshot.robot_enabled);

    cJSON *pose_array = cJSON_AddArrayToObject(root, "poses");
    for (const auto &pose : snapshot.poses) {
        cJSON *pose_json = cJSON_CreateObject();
        populate_pose_json(pose_json, pose);
        cJSON_AddItemToArray(pose_array, pose_json);
    }

    return print_json(root);
}

std::string empty_snapshot() {
    cJSON *root = cJSON_CreateObject();
    if (root == nullptr) return "{}";

    cJSON_AddNumberToObject(root, "width", 0);
    cJSON_AddNumberToObject(root, "height", 0);

    static constexpr int edges[][2] = {
        {0, 1}, {0, 2}, {1, 3}, {2, 4}, {5, 6}, {5, 7}, {7, 9}, {6, 8},
        {8, 10}, {5, 11}, {6, 12}, {11, 12}, {11, 13}, {13, 15}, {12, 14}, {14, 16}
    };
    cJSON *mapping = cJSON_AddArrayToObject(root, "keypoints_mapping");
    for (const auto &edge : edges) {
        cJSON_AddItemToArray(mapping, cJSON_CreateIntArray(edge, 2));
    }

    cJSON_AddNumberToObject(root, "selected_pose_id", -1);
    cJSON_AddStringToObject(root, "selection_state", "noone");
    cJSON_AddBoolToObject(root, "robot_enabled", false);
    cJSON_AddArrayToObject(root, "poses");

    return print_json(root);
}

std::string serialize_pose(const HailoJointController::Snapshot &snapshot) {
    if (snapshot.poses.empty()) {
        // The UI treats an empty JSON object as a valid no-detection update.
        return "{}";
    }
    cJSON *root = cJSON_CreateObject();
    if (root == nullptr) return {};
    const auto &pose = snapshot.poses.front();
    cJSON_AddNumberToObject(root, "id", pose.id);
    cJSON *box = cJSON_AddObjectToObject(root, "bbox");
    // ViTPose reports its aggregate score as a percentage; QPP uses [0,1].
    cJSON_AddNumberToObject(box, "conf", pose.box.score / 100.0);
    cJSON_AddNumberToObject(box, "x", pose.box.x_min);
    cJSON_AddNumberToObject(box, "y", pose.box.y_min);
    cJSON_AddNumberToObject(box, "w", pose.box.x_max - pose.box.x_min);
    cJSON_AddNumberToObject(box, "h", pose.box.y_max - pose.box.y_min);
    cJSON *keypoints = cJSON_AddArrayToObject(root, "keypoints");
    for (const auto &joint : pose.key_points) {
        cJSON *point = cJSON_CreateObject();
        cJSON_AddNumberToObject(point, "x", joint.x);
        cJSON_AddNumberToObject(point, "y", joint.y);
        if (joint.z != -1) cJSON_AddNumberToObject(point, "z", joint.z);
        cJSON_AddNumberToObject(point, "conf", joint.confidence);
        cJSON_AddItemToArray(keypoints, point);
    }
    cJSON *detected = cJSON_AddArrayToObject(root, "pose_detected");
    for (const auto &name : pose.detected_poses) {
        cJSON_AddItemToArray(detected, cJSON_CreateString(name.c_str()));
    }
    return print_json(root);
}

}  // namespace

HailoJointQpp::HailoJointQpp(const std::string &pose_path, const std::string &bra_path, HailoJointController &controller) :
    pose_path_(pose_path), bra_path_(bra_path),controller_(controller) {
    controller_.add_joint_update_cb(
        std::bind(&HailoJointQpp::publish_snapshot, this, std::placeholders::_1));
}

HailoJointQpp::~HailoJointQpp() {
    stop();
    close_signals();
}

int HailoJointQpp::init() {
    int result = open_signals();
    if (result != EOK) {
        close_signals();
        return result;
    }

    if (pipe(wake_pipe_) == -1) {
        result = errno;
        spdlog::error("Failed to create QPP command wake pipe: {}", strerror(result));
        close_signals();
        return result;
    }

    // Ignore control requests left behind by a previous controller instance.
    (void)lseek(roi_control_fd_, 0, SEEK_END);
    (void)lseek(enabled_control_fd_, 0, SEEK_END);
    return EOK;
}

int HailoJointQpp::open_signals() {
    info_fd_ = open_signal(signal_path(pose_path_, "info"), O_WRONLY);
    if (info_fd_ < 0) return -info_fd_;
    pose_detection_fd_ = open_signal(signal_path(pose_path_, "detections/pose1"), O_WRONLY);
    if (pose_detection_fd_ < 0) return -pose_detection_fd_;
    selected_pose_fd_ = open_signal(signal_path(pose_path_, "selected_pose"), O_WRONLY);
    if (selected_pose_fd_ < 0) return -selected_pose_fd_;
    selected_state_fd_ = open_signal(signal_path(pose_path_, "selected_pose_state"), O_WRONLY);
    if (selected_state_fd_ < 0) return -selected_state_fd_;
    roi_value_fd_ = open_signal(signal_path(pose_path_, "roi"), O_RDWR);
    if (roi_value_fd_ < 0) return -roi_value_fd_;
    roi_control_fd_ = open_signal(signal_path(pose_path_, "roi?control"), O_RDONLY);
    if (roi_control_fd_ < 0) return -roi_control_fd_;
    enabled_value_fd_ = open_signal(signal_path(pose_path_, "enabled"), O_RDWR);
    if (enabled_value_fd_ < 0) return -enabled_value_fd_;
    enabled_control_fd_ = open_signal(signal_path(pose_path_, "enabled?control"), O_RDONLY);
    if (enabled_control_fd_ < 0) return -enabled_control_fd_;
    mode_value_fd_ = open_signal(signal_path(bra_path_, "mode"), O_RDONLY);
    if (mode_value_fd_ < 0) return -mode_value_fd_;
    return EOK;
}

int HailoJointQpp::start() {
    if (running_.exchange(true)) {
        return EALREADY;
    }
    command_thread_ = std::thread(&HailoJointQpp::command_loop, this);
    return EOK;
}

int HailoJointQpp::stop() {
    if (!running_.exchange(false)) {
        return EOK;
    }

    const uint8_t wake = 1;
    (void)write(wake_pipe_[1], &wake, sizeof(wake));
    if (command_thread_.joinable()) {
        command_thread_.join();
    }
    return EOK;
}

void HailoJointQpp::close_signals() {
    int *fds[] = {&info_fd_,          &pose_detection_fd_, &selected_pose_fd_, &selected_state_fd_,
                  &roi_value_fd_,     &roi_control_fd_,     &enabled_value_fd_, &enabled_control_fd_,
                  &mode_value_fd_,    &wake_pipe_[0],     &wake_pipe_[1]};
    for (auto fd : fds) {
        if (*fd >= 0) {
            close(*fd);
        }
        *fd = -1;
    }
}

void HailoJointQpp::publish_snapshot(const HailoJointController::Snapshot &snapshot) {
    char buf[32];
    ssize_t size = read(mode_value_fd_, buf, sizeof(buf) - 1);
    // Strip null terminator if present
    if (size > 0) {
        if (buf[size - 1] == '\0') size--;
        mode_ = std::string(buf, size);
    }
    if (mode_ != "joint" || !snapshot.robot_enabled) {
        (void)write_string(info_fd_, empty_snapshot(), "pose.info");
        (void)write_string(pose_detection_fd_, "{}", "pose.detections.pose1");
        (void)write_string(selected_pose_fd_, "", "pose.selected_pose");
        (void)write_string(selected_state_fd_, "noone", "pose.selected_pose_state");
        return;
    }
    std::string json = serialize_snapshot(snapshot);
    if (json.empty()) {
        spdlog::error("Failed to serialize pose snapshot");
        return;
    }

    (void)write_string(info_fd_, json, "pose.info");
    (void)write_string(pose_detection_fd_, serialize_pose(snapshot), "pose.detections.pose1");
    const std::string selected =
        snapshot.selected_pose_id < 0 ? std::string{} : std::to_string(snapshot.selected_pose_id);
    (void)write_string(selected_pose_fd_, selected, "pose.selected_pose");
    (void)write_string(selected_state_fd_, selection_state_name(snapshot.selection_state),
                       "pose.selected_pose_state");
}

void HailoJointQpp::publish_roi(const ROIBounds &roi) {
    std::array<int8_t, 4> value = {static_cast<int8_t>(roi[0]), static_cast<int8_t>(roi[1]),
                                   static_cast<int8_t>(roi[2]), static_cast<int8_t>(roi[3])};
    (void)write_value(roi_value_fd_, value.data(), value.size(), "pose.roi");
}

void HailoJointQpp::publish_enabled(bool enabled) {
    (void)write_value(enabled_value_fd_, &enabled, sizeof(enabled), "pose.enabled");
}

void HailoJointQpp::handle_roi_request() {
    std::array<int8_t, 4> raw{};
    ssize_t size = read(roi_control_fd_, raw.data(), raw.size());
    if (size != static_cast<ssize_t>(raw.size())) {
        spdlog::warn("Ignoring pose.roi request with invalid size {}", size);
        return;
    }
    for (auto value : raw) {
        if (value < 0 || value > 100) {
            spdlog::warn("Ignoring pose.roi request with out-of-range value {}", value);
            return;
        }
    }

    ROIBounds roi = {static_cast<uint8_t>(raw[0]), static_cast<uint8_t>(raw[1]), static_cast<uint8_t>(raw[2]),
                     static_cast<uint8_t>(raw[3])};
    int result = controller_.set_roi(roi);
    if (result != EOK) {
        spdlog::warn("Ignoring invalid pose.roi rectangle");
        return;
    }
    publish_roi(roi);
}

void HailoJointQpp::handle_enabled_request() {
    bool enabled = false;
    ssize_t size = read(enabled_control_fd_, &enabled, sizeof(enabled));
    if (size != sizeof(enabled)) {
        spdlog::warn("Ignoring pose.enabled request with invalid size {}", size);
        return;
    }
    controller_.set_publish_results(enabled);
    publish_enabled(enabled);
}

void HailoJointQpp::command_loop() {
    std::array<pollfd, 4> fds = {{{wake_pipe_[0], POLLIN, 0},
                                   {roi_control_fd_, POLLIN, 0},
                                   {enabled_control_fd_, POLLIN, 0}}};

    while (running_) {
        int result = poll(fds.data(), fds.size(), -1);
        if (result == -1) {
            if (errno == EINTR) {
                continue;
            }
            spdlog::error("QPP command poll failed: {}", strerror(errno));
            break;
        }

        if ((fds[0].revents & POLLIN) != 0) break;
        bool disconnected = false;
        for (size_t index = 1; index < fds.size(); ++index) {
            if ((fds[index].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                disconnected = true;
            }
        }
        if (disconnected) {
            spdlog::error("A required QPP signal disconnected; stopping the command loop");
            running_ = false;
            break;
        }
        if ((fds[1].revents & POLLIN) != 0) handle_roi_request();
        if ((fds[2].revents & POLLIN) != 0) handle_enabled_request();
    }
}
