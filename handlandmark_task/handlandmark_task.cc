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

#include "handlandmark_task.h"

#include <errno.h>

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <tuple>
#include <unordered_map>
#include <utility>

#include "absl/log/absl_log.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/tasks/cc/core/base_options.h"
#include "mediapipe/tasks/cc/vision/core/running_mode.h"
#include "mediapipe/tasks/cc/vision/hand_landmarker/hand_landmarker_result.h"
#include "nlohmann/json.hpp"

namespace {
const std::string QPP_LM_PATH = "physicalai/hands/";

// Writing paths
const std::string QPP_LM_INFO_PATH = QPP_LM_PATH + "info";
const std::string QPP_LM_LEFT_HAND_PATH = QPP_LM_PATH + "detections/left_hand1";
const std::string QPP_LM_RIGHT_HAND_PATH =
    QPP_LM_PATH + "detections/right_hand1";

// Reading/ctrl paths
const std::string QPP_LM_ROI_PATH = QPP_LM_PATH + "roi";
std::string QPP_LM_ENABLED = QPP_LM_PATH + "enabled";

/// Wrist is landmark 0 in the MediaPipe hand topology.
constexpr int WRIST_LANDMARK = 0;

constexpr int HAND_CONNECTIONS[][2] = {
    {0, 1},   {0, 5},   {9, 13},  {13, 17}, {5, 9},   {0, 17},  {1, 2},
    {2, 3},   {3, 4},   {5, 6},   {6, 7},   {7, 8},   {9, 10},  {10, 11},
    {11, 12}, {13, 14}, {14, 15}, {15, 16}, {17, 18}, {18, 19}, {19, 20}};

// Padding around the bounding box so it isn't directly on the points
constexpr double BBOX_PADDING = 0.10;

/// ROI arrives as [x, y, end_x, end_y].
constexpr size_t ROI_ELEMENTS = 4;

/**
 * @brief Convert the mediapipe label to our expected label.
 *
 * @return converted label or the label itself if not found
 */
std::string gesture_label(const std::string &canned) {
  static const std::unordered_map<std::string, std::string> kNames = {
      {"Open_Palm", "hand_open"},
      {"Closed_Fist", "hand_closed"},
      {"Thumb_Up", "thumb_up"},
      {"Thumb_Down", "thumb_down"},
      {"Victory", "peace"},
  };
  const auto it = kNames.find(canned);
  return it == kNames.end() ? canned : it->second;
}

/// Written to a detection file when that hand is not in the frame.
const std::string QPP_NO_HAND = "{}";

/// How many results between throughput reports. Frames arrive faster than
/// inference completes, so the gap between the two counters is the drop rate.
constexpr uint64_t STATS_EVERY_N_RESULTS = 60;
}  // namespace

namespace hand_landmarker = ::mediapipe::tasks::vision::hand_landmarker;
namespace gesture_recognizer = ::mediapipe::tasks::vision::gesture_recognizer;

LandmarkFinder::LandmarkFinder() = default;

LandmarkFinder::~LandmarkFinder() {
  stop();  // Drop any in-flight frames before we start tearing things down.

  std::lock_guard<std::mutex> lock(state_mtx_);
  // stop() only pauses, so the real teardown lives here. The camera calls back
  // into this object, so its thread has to be stopped before any member dies.
  if (cam_) {
    cam_->stop();
    cam_started_ = false;
  }
  if (landmarker_) {
    absl::Status status = landmarker_->Close();
    if (!status.ok()) {
      ABSL_LOG(WARNING) << "GestureRecognizer::Close failed: " << status;
    }
  }
  if (qpp_reader_) {
    qpp_reader_->stop();
  }
}

int LandmarkFinder::init(const Options &options) {
  std::lock_guard<std::mutex> lock(state_mtx_);
  if (state_ != State::NON_INIT) {
    return EALREADY;
  }
  opts_ = options;

  cam_ = std::make_unique<RpiCam>(opts_.camera_unit);
  int err = cam_->init();
  if (err != EOK) {
    ABSL_LOG(ERROR) << "Camera init failed: " << strerror(err);
    cam_.reset();
    return err;
  }

  std::tie(frame_width_, frame_height_) = cam_->get_frame_size();

  auto mp_options =
      std::make_unique<gesture_recognizer::GestureRecognizerOptions>();
  mp_options->base_options.model_asset_path = opts_.model_path;
  mp_options->num_hands = opts_.num_hands;
  // LIVE_STREAM is the only mode that honours RpiCam's "do not block the camera
  // thread" contract -- RecognizeAsync hands off and returns immediately.
  mp_options->running_mode =
      mediapipe::tasks::vision::core::RunningMode::LIVE_STREAM;
  mp_options->result_callback =
      [this](absl::StatusOr<gesture_recognizer::GestureRecognizerResult> result,
             const mediapipe::Image &image, int64_t timestamp_ms) {
        on_result(std::move(result), image, timestamp_ms);
      };

  auto landmarker =
      gesture_recognizer::GestureRecognizer::Create(std::move(mp_options));
  if (!landmarker.ok()) {
    ABSL_LOG(ERROR) << "GestureRecognizer::Create failed: "
                    << landmarker.status();
    cam_.reset();
    return EINVAL;
  }
  landmarker_ = std::move(*landmarker);

  if (!std::filesystem::is_directory(opts_.qpp_path)) {
    ABSL_LOG(ERROR) << "QPP is not mounted at " << opts_.qpp_path
                    << " -- is the qpp service running?";
    cam_.reset();
    landmarker_.reset();
    return ENOENT;
  }
  qpp_reader_ = std::make_unique<QPPReader>(opts_.qpp_path);

  // Both are actuators: QPP delivers writes on the ".control" file and expects
  // the value file to be written back as the acknowledgement.
  qpp_reader_->add_value_listener(
      QPP_LM_ENABLED,
      QPPReader::QPPTypeInfo{QPPReader::QPPType::BOOL, false, true},
      std::bind(&LandmarkFinder::qpp_lm_enable, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  qpp_reader_->add_value_listener(
      QPP_LM_ROI_PATH,
      QPPReader::QPPTypeInfo{QPPReader::QPPType::INT8, true, true},
      std::bind(&LandmarkFinder::qpp_lm_roi, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));

  if (publish_info() < 0) {
    ABSL_LOG(ERROR) << "Failed to publish " << QPP_LM_INFO_PATH;
    cam_.reset();
    landmarker_.reset();
    qpp_reader_.reset();
    return EIO;
  }

  // Listeners must be registered before go(): add_value_listener() refuses
  // once the poll thread is running.
  if (qpp_reader_->go() != EOK) {
    ABSL_LOG(ERROR) << "Failed to start the QPP reader thread";
    cam_.reset();
    landmarker_.reset();
    qpp_reader_.reset();
    return EIO;
  }

  start_time_ = std::chrono::steady_clock::now();
  state_ = State::INIT;
  ABSL_LOG(INFO) << "Initialised: " << opts_.num_hands << " hand(s), model "
                 << opts_.model_path;
  return EOK;
}

int LandmarkFinder::start() {
  std::lock_guard<std::mutex> lock(state_mtx_);
  if (state_ != State::INIT) {
    return (state_ == State::RUNNING) ? EALREADY : ENODEV;
  }

  // Only start the viewfinder the first time. On resume it is still streaming
  // and we just re-open the gate.
  if (!cam_started_) {
    int err = cam_->start([this](const cv::Mat &frame) { on_frame(frame); });
    if (err != EOK) {
      ABSL_LOG(ERROR) << "Camera start failed: " << strerror(err);
      return err;
    }
    cam_started_ = true;
  }

  processing_ = true;
  state_ = State::RUNNING;
  return EOK;
}

int LandmarkFinder::stop() {
  std::lock_guard<std::mutex> lock(state_mtx_);
  if (state_ != State::RUNNING) {
    return EOK;
  }

  // Pause only. The camera keeps streaming and the model stays loaded; frames
  // are dropped in on_frame() until start() re-opens the gate. Teardown is the
  // destructor's job.
  processing_ = false;
  state_ = State::INIT;
  return EOK;
}

LandmarkFinder::ROIPixels LandmarkFinder::roi_pixels() {
  std::lock_guard<std::mutex> lock(roi_mtx_);
  const ROIBounds bounds = roi_;

  // The ROI has already been prechecked to be valid.
  ROIPixels px;
  px.x = std::clamp(static_cast<int>(std::lround(frame_width_ * bounds.x / 100.0)), 0,
                    frame_width_ - 1);
  px.y = std::clamp(static_cast<int>(std::lround(frame_height_ * bounds.y / 100.0)), 0,
                    frame_height_ - 1);
  px.w =
      std::clamp(static_cast<int>(std::lround(frame_width_ * (bounds.end_x - bounds.x) / 100.0)),
                 1, frame_width_ - px.x);
  px.h =
      std::clamp(static_cast<int>(std::lround(frame_height_ * (bounds.end_y - bounds.y) / 100.0)),
                 1, frame_height_ - px.y);
  return px;
}

void LandmarkFinder::qpp_lm_enable(const std::string &name,
                                   const QPPReader::QPPValue &value,
                                   const QPPReader::QPPTypeInfo &info) {
  const bool *enabled = std::get_if<bool>(&value);
  if (enabled == nullptr) {
    ABSL_LOG(WARNING) << name << ": expected a boolean, ignoring";
    return;
  }

  qpp_enabled_ = *enabled;
  ABSL_LOG(INFO) << "QPP " << name << " -> " << (*enabled ? "true" : "false");

  // Acknowledge on the value file (not the control file we were called from).
  if (qpp_reader_->write_value(QPP_LM_ENABLED,
                               QPPReader::QPPTypeInfo{QPPReader::QPPType::BOOL},
                               *enabled) < 0) {
    ABSL_LOG(WARNING) << "Failed to acknowledge " << QPP_LM_ENABLED;
  }
}

void LandmarkFinder::qpp_lm_roi(const std::string &name,
                                const QPPReader::QPPValue &value,
                                const QPPReader::QPPTypeInfo &info) {
  const auto *values = std::get_if<QPPReader::QPPIntArray>(&value);
  if (values == nullptr || values->size() != ROI_ELEMENTS) {
    ABSL_LOG(WARNING) << name << ": expected " << ROI_ELEMENTS
                      << " ints [x, y, end_x, end_y], ignoring";
    return;
  }

  const int64_t x = (*values)[0], y = (*values)[1];
  const int64_t end_x = (*values)[2], end_y = (*values)[3];

  // Percentages, and the end has to be past the start. Rejecting outright beats
  // cropping to something nonsensical and silently publishing bad coordinates.
  const bool in_range = x >= 0 && y >= 0 && end_x <= 100 && end_y <= 100;
  if (!in_range || end_x <= x || end_y <= y) {
    ABSL_LOG(WARNING) << name << ": rejecting ROI [" << x << "," << y << ","
                      << end_x << "," << end_y
                      << "] -- need 0 <= start < end <= 100";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(roi_mtx_);
    roi_ = {static_cast<int8_t>(x), static_cast<int8_t>(y),
            static_cast<int8_t>(end_x), static_cast<int8_t>(end_y)};
  }

  const ROIPixels px = roi_pixels();
  ABSL_LOG(INFO) << "QPP " << name << " -> [" << x << "," << y << "," << end_x
                 << "," << end_y << "] = " << px.w << "x" << px.h << "+" << px.x
                 << "+" << px.y << " px";

  if (qpp_reader_->write_value(
          QPP_LM_ROI_PATH,
          QPPReader::QPPTypeInfo{QPPReader::QPPType::INT8, true},
          *values) < 0) {
    ABSL_LOG(WARNING) << "Failed to acknowledge " << QPP_LM_ROI_PATH;
  }
}

std::string LandmarkFinder::hand_to_json(
    const hand_landmarker::HandLandmarkerResult &result, size_t index,
    const ROIPixels &roi,
    const std::vector<mediapipe::ClassificationList> &gestures) const {
  const auto &landmarks = result.hand_landmarks[index].landmarks;

  // MediaPipe reports a score per hand, not per landmark: the optional
  // visibility/presence fields are left unset by the hand landmarker. Use the
  // hand's own score for each keypoint rather than inventing a number.
  float score = 0.0f;
  if (index < result.handedness.size() &&
      !result.handedness[index].categories.empty()) {
    score = result.handedness[index].categories[0].score;
  }

  // Landmarks are normalised 0-1 against the *cropped* image, and the template
  // is in pixels on the full frame -- so each axis needs the ROI's scale factor
  // and its offset. With a full-frame ROI this reduces to x * frame_width_.
  auto to_px_x = [&roi](float x) -> int { return std::lround(roi.x + x * roi.w); };
  auto to_px_y = [&roi](float y) -> int { return std::lround(roi.y + y * roi.h); };

  nlohmann::json keypoints = nlohmann::json::array();
  int min_x = INT32_MAX;
  int min_y = INT32_MAX;
  int max_x = INT32_MIN;
  int max_y = INT32_MIN;

  for (const auto &landmark : landmarks) {
    const int x = std::clamp(to_px_x(landmark.x), roi.x, roi.x + roi.w);
    const int y = std::clamp(to_px_y(landmark.y), roi.y, roi.y + roi.h);
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
    keypoints.push_back({{"x", x}, {"y", y}, {"z", landmark.z}, {"conf", score}});
  }

  // No bbox comes out of the hand landmarker, so derive it from the keypoints,
  // padded outwards and clipped to the frame so it stays a valid crop region.
  const long pad_x = std::lround((max_x - min_x) * BBOX_PADDING);
  const long pad_y = std::lround((max_y - min_y) * BBOX_PADDING);
  const long x0 = std::max(0L, min_x - pad_x);
  const long y0 = std::max(0L, min_y - pad_y);
  const long x1 = std::min(static_cast<long>(frame_width_), max_x + pad_x);
  const long y1 = std::min(static_cast<long>(frame_height_), max_y + pad_y);

  nlohmann::json hand;
  hand["bbox"] = {
      {"conf", score}, {"x", x0}, {"y", y0}, {"w", x1 - x0}, {"h", y1 - y0}};
  hand["keypoints"] = std::move(keypoints);

  nlohmann::json poses = nlohmann::json::array();
  if (opts_.publish_gestures && index < gestures.size()) {
    for (const auto &gesture : gestures[index].classification()) {
      poses.push_back(gesture_label(gesture.label()));
    }
  }
  hand["pose_detected"] = std::move(poses);
  return hand.dump();
}

int LandmarkFinder::publish_info() {
  nlohmann::json info;
  info["width"] = frame_width_;
  info["height"] = frame_height_;

  nlohmann::json mapping = nlohmann::json::array();
  for (const auto &connection : HAND_CONNECTIONS) {
    mapping.push_back({connection[0], connection[1]});
  }
  info["keypoints_mapping"] = std::move(mapping);

  const std::string payload = info.dump();
  ABSL_LOG(INFO) << "Publishing " << QPP_LM_INFO_PATH << ": " << payload;

  return qpp_reader_->write_value(
      QPP_LM_INFO_PATH, QPPReader::QPPTypeInfo{QPPReader::QPPType::STRING},
      payload);
}

void LandmarkFinder::on_frame(const cv::Mat &frame) {
  // Dropped while paused (stop()) or disabled over QPP. Cheapest possible
  // early-out: this runs on the camera thread for every single frame.
  if (!processing_ || !qpp_enabled_ || !landmarker_ || frame.empty()) {
    return;
  }

  const ROIPixels roi = roi_pixels();

  // Only take a sub-image when the ROI is actually narrower than the frame --
  // a full-frame "crop" is a pointless Mat header for every frame.
  const bool full_frame =
      roi.x == 0 && roi.y == 0 && roi.w == frame.cols && roi.h == frame.rows;
  const cv::Mat input =
      full_frame ? frame : frame(cv::Rect(roi.x, roi.y, roi.w, roi.h));

  // Wrap the frame in an ImageFrame. RpiCam already hands us RGB, so this is a
  // straight copy.
  auto image_frame = std::make_shared<mediapipe::ImageFrame>(
      mediapipe::ImageFormat::SRGB, input.cols, input.rows,
      mediapipe::ImageFrame::kDefaultAlignmentBoundary);
  cv::Mat image_frame_mat = mediapipe::formats::MatView(image_frame.get());
  input.copyTo(image_frame_mat);

  const int64_t timestamp_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start_time_)
          .count();

  frames_in_++;
  absl::Status status = landmarker_->RecognizeAsync(
      mediapipe::Image(std::move(image_frame)), timestamp_ms);
  if (!status.ok()) {
    // Most likely a non-monotonic timestamp: two frames landing in the same
    // millisecond. Losing one frame is fine, so log rather than bail.
    ABSL_LOG_EVERY_N_SEC(WARNING, 5) << "RecognizeAsync failed: " << status;
  }
}

void LandmarkFinder::on_result(
    absl::StatusOr<gesture_recognizer::GestureRecognizerResult> result,
    const mediapipe::Image &image, int64_t timestamp_ms) {
  if (!result.ok()) {
    ABSL_LOG_EVERY_N_SEC(WARNING, 5) << "Detection failed: " << result.status();
    return;
  }

  const uint64_t results = ++results_out_;
  if (results % STATS_EVERY_N_RESULTS == 0) {
    // frames_in vs results_out is the inference drop rate.
    printf("[stats] frames_in=%llu results_out=%llu t=%lldms\n",
           static_cast<unsigned long long>(frames_in_.load()),
           static_cast<unsigned long long>(results),
           static_cast<long long>(timestamp_ms));
  }

  // Both files are rewritten every result. A hand that is no longer in frame
  // gets "{}" rather than being left at its last known position.
  std::string left = QPP_NO_HAND;
  std::string right = QPP_NO_HAND;

  // MediaPipe's num_hands is only an upper bound -- it will happily return two
  // hands it both labels "Left". Since there is exactly one file per side, keep
  // whichever detection the handedness classifier was most confident about
  // rather than letting the last one in the list win.
  float best_left_score = -1.0f;
  float best_right_score = -1.0f;

  // Landmarks are normalised against the cropped image, so mapping them back to
  // full-frame pixels needs the ROI. Re-read rather than carried alongside the
  // frame: an ROI change in flight mismaps one frame, which is acceptable.
  const ROIPixels roi = roi_pixels();

  // The recognizer returns raw protos; this converts them into the same
  // HandLandmarkerResult the rest of this function already works with, so the
  // handedness, bbox and publishing logic below is unchanged.
  const hand_landmarker::HandLandmarkerResult lm =
      hand_landmarker::ConvertToHandLandmarkerResult(
          result->handedness, result->hand_landmarks,
          result->hand_world_landmarks);

  const auto &hands = lm.hand_landmarks;
  for (size_t i = 0; i < hands.size(); i++) {
    // handedness and hand_landmarks are parallel: index i is the same hand.
    const char *label = "?";
    float score = 0.0f;
    if (i < lm.handedness.size() && !lm.handedness[i].categories.empty()) {
      const auto &category = lm.handedness[i].categories[0];
      score = category.score;
      if (category.category_name.has_value()) {
        label = category.category_name->c_str();
      }
    }

    const auto &landmarks = hands[i].landmarks;
    if (landmarks.empty()) {
      continue;
    }

    if (strcmp(label, "Left") == 0) {
      if (score > best_left_score) {
        best_left_score = score;
        left = hand_to_json(lm, i, roi, result->gestures);
      }
    } else if (strcmp(label, "Right") == 0) {
      if (score > best_right_score) {
        best_right_score = score;
        right = hand_to_json(lm, i, roi, result->gestures);
      }
    }

    const auto &wrist = landmarks[WRIST_LANDMARK];

    if (opts_.verbose) {
      printf("[%6lld ms] %-5s (%.2f)  wrist=(%.3f,%.3f,%.3f)  %zu pts\n",
             static_cast<long long>(timestamp_ms), label, score, wrist.x,
             wrist.y, wrist.z, landmarks.size());
      for (size_t j = 0; j < landmarks.size(); j++) {
        printf("    [%2zu] %.4f %.4f %.4f\n", j, landmarks[j].x, landmarks[j].y,
               landmarks[j].z);
      }
    }
  }

  publish_hand(QPP_LM_LEFT_HAND_PATH, left, last_left_);
  publish_hand(QPP_LM_RIGHT_HAND_PATH, right, last_right_);
}

void LandmarkFinder::publish_hand(const std::string &path,
                                  const std::string &payload,
                                  std::string &last_payload) {
  // Skip the write when nothing changed -- write_value() reopens the file every
  // call, so rewriting an identical "{}" on every frame is pure syscall churn.
  if (payload == last_payload) {
    return;
  }
  if (qpp_reader_->write_value(
          path, QPPReader::QPPTypeInfo{QPPReader::QPPType::STRING}, payload) <
      0) {
    ABSL_LOG_EVERY_N_SEC(WARNING, 5) << "Failed to publish " << path;
    return;
  }
  last_payload = payload;
}
