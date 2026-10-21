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

#include <functional>
#include <mutex>

#include <camera/camera_api.h>

#include <optional>

#include <opencv2/opencv.hpp>

/**
 * Sensor Framework wrapper for RpiCam module.
 *
 * @note This wrapper should be able to be used with any camera that uses nv12 encoding but in our usecase we will
 *       Only use the camera module 3.
 */
class RpiCam {
   public:
    /**
     * @brief Frame callback containing the image has an OpenCV frame in BGR format.
     *
     * @param frame BGR format frame.
     */
    using FrameCb = std::function<void(const cv::Mat &frame)>;

   public:
    RpiCam(camera_unit_t camera_id);

    /**
     * @brief Setup the camera with sensor framework
     *
     * @return EOK on success, otherwise an error code
     */
    int init();

    /**
     * @brief Starts the viewfinder for sensor framework with a given callback.
     *
     * @param callback CB which is called on a new frame.
     *
     * @return EOK on success, otherwise an error code.
     */
    int start(FrameCb callback);

    /**
     * @brief Stop the viewfinder for sensor framework
     *
     *
     * @return EOK on success, otherwise an error code.
     */
    int stop();

    /// Gets the screen size in <width, height> format
    std::pair<int, int> get_screen_size() const;

   private:
    /// Internal callback for sf frame
    static void sf_cam_cb(camera_handle_t handle, camera_buffer_t *buf, void *data);

   private:
    FrameCb cb_{};
    camera_unit_t id_{};
    std::optional<sensor_interim_data_unit_t> data_id_;

    // Camera info
    camera_handle_t handle_ = CAMERA_HANDLE_INVALID;
    int width_{};
    int height_{};
};
