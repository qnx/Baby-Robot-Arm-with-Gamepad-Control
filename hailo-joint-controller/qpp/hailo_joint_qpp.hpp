/**
 * Copyright (c) 2026, BlackBerry Limited. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */
#pragma once

#include <atomic>
#include <string>
#include <thread>

#include "hailo_joint_controller.hpp"

namespace joint::qpp {

/**
 * Publishes controller state to, and consumes actuator requests from, a
 * catalog-backed QNX POSIX PubSub signal tree.
 */
class HailoJointQpp {
   public:
    HailoJointQpp(const std::string &pose_path, const std::string &bra_path, HailoJointController &controller);
    ~HailoJointQpp();

    int init();
    int start();
    int stop();

   private:
    int open_signals();
    void close_signals();
    void command_loop();

    void publish_snapshot(const HailoJointController::Snapshot &snapshot);
    void publish_roi(const ROIBounds &roi);
    void publish_enabled(bool enabled);

    void handle_roi_request();
    void handle_enabled_request();

    std::string pose_path_;
    std::string bra_path_;
    std::string mode_;
    HailoJointController &controller_;

    int info_fd_ = -1;
    int pose_detection_fd_ = -1;
    int selected_pose_fd_ = -1;
    int selected_state_fd_ = -1;
    int roi_value_fd_ = -1;
    int roi_control_fd_ = -1;
    int enabled_value_fd_ = -1;
    int enabled_control_fd_ = -1;
    int mode_value_fd_ = -1;
    int wake_pipe_[2] = {-1, -1};

    std::thread command_thread_{};
    std::atomic_bool running_{false};
};

}  // namespace joint::qpp
