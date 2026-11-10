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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "mediapipe/framework/formats/classification.pb.h"
#include "mediapipe/framework/formats/image.h"
#include "mediapipe/tasks/cc/vision/gesture_recognizer/gesture_recognizer.h"
#include "mediapipe/tasks/cc/vision/hand_landmarker/hand_landmarker_result.h"

#include "qpp_reader.h"
#include "rpi_cam.h"

class LandmarkFinder {
 public:
  /// Tuning values, all settable from the CLI. See main.cc.
  struct Options {
    camera_unit_t camera_unit{};
    std::string model_path{};
    std::string qpp_path{"/dev/qpp"};
    int num_hands = 2;
    bool verbose = false;  /// Prints output to console

    // Publishes gestures. Note the ML always runs but this will toggle if it is used
    // or if the processing is done higher up
    bool publish_gestures = true;
  };

 private:
  enum class State { NON_INIT, INIT, RUNNING };
  /// Region of interest as QPP carries it: percentages of the full frame.
  struct ROIBounds {
    int8_t x;
    int8_t y;
    int8_t end_x;
    int8_t end_y;
  };

  /// The same region resolved against the current frame size. Kept alongside
  /// ROIBounds so the percent -> pixel conversion happens when the ROI changes
  /// rather than on every frame.
  struct ROIPixels {
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
  };

 public:
  LandmarkFinder();
  ~LandmarkFinder();

  int init(const Options &options);  // Setup state
  int start();                       // Start publishing and detection

  /**
   * @brief Pauses the detection and camera to lower idle resources
   */
  int stop();
 private:
  // rpi_cam callback to process the camera frame
  void on_frame(const cv::Mat &frame);

  /// @brief converts hand to qpp json format
  std::string hand_to_json(
      const mediapipe::tasks::vision::hand_landmarker::HandLandmarkerResult
          &result,
      size_t index, const ROIPixels &roi,
      const std::vector<mediapipe::ClassificationList> &gestures) const;

  /// Current ROI resolved against the frame size. Takes roi_mtx_ itself.
  /// Recomputed per frame and again per result rather than cached: an ROI
  /// change landing between the two mismaps a single frame, which is cheaper
  /// than keeping a second copy of the state in sync.
  ROIPixels roi_pixels();

  /// Write one detection file, skipping the write when the value is unchanged.
  void publish_hand(const std::string &path, const std::string &payload,
                    std::string &last_payload);

  /// Publish the static stream description (frame size + joint links) to QPP.
  /// Written once at init: readers need it to scale normalised landmarks back
  /// into pixels and to know how to connect them.
  int publish_info();

  /// MediaPipe detection result. Runs on MediaPipe's own thread.
  void on_result(
      absl::StatusOr<
          mediapipe::tasks::vision::gesture_recognizer::GestureRecognizerResult>
          result,
      const mediapipe::Image &image, int64_t timestamp_ms);

  // Callbacks for QPP
  void qpp_lm_enable(const std::string &name, const QPPReader::QPPValue &value,
                      const QPPReader::QPPTypeInfo &info);
  void qpp_lm_roi(const std::string &name, const QPPReader::QPPValue &value,
                  const QPPReader::QPPTypeInfo &info);

 private:
  Options opts_{};

  std::unique_ptr<QPPReader> qpp_reader_{};
  std::unique_ptr<RpiCam> cam_{};
  std::unique_ptr<
      mediapipe::tasks::vision::gesture_recognizer::GestureRecognizer>
      landmarker_{};


  // If false no frames will be processed even if they slip through
  std::atomic<bool> qpp_enabled_{true}; // If ML should be disabled by QPP
  std::atomic<bool> processing_{false};
  bool cam_started_ = false;

  // camera frame in pixels
  int frame_width_ = 0;
  int frame_height_ = 0;

  // What was published last to avoid double publishing waking everyone up
  std::string last_left_{};
  std::string last_right_{};

  std::chrono::steady_clock::time_point start_time_{};
  std::atomic<uint64_t> frames_in_{0};
  std::atomic<uint64_t> results_out_{0};

  // Region of interest
  std::mutex roi_mtx_{};
  ROIBounds roi_ = {0, 0, 100, 100};  /// Helps us map robots x,y

  State state_ = State::NON_INIT;
  std::mutex state_mtx_{};
};
