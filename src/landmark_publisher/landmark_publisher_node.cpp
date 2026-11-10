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
#include "landmark_publisher_node.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace {
const std::string PUBLISH_MODE = "ik_joint";
const std::string QPP_BRA_PATH = "physicalai/bra/";
const std::string QPP_LM_PATH = "physicalai/hands/";

// Reading paths
const std::string QPP_BRA_MODE_PATH = QPP_BRA_PATH + "mode";
const std::string QPP_BRA_ENABLED_PATH = QPP_BRA_PATH + "enabled";

const std::string QPP_LM_ROI_PATH = QPP_LM_PATH + "roi";
const std::string QPP_LM_INFO_PATH = QPP_LM_PATH + "info";
const std::string QPP_LM_LEFT_HAND_PATH = QPP_LM_PATH + "detections/left_hand1";
const std::string QPP_LM_RIGHT_HAND_PATH =
    QPP_LM_PATH + "detections/right_hand1";

const std::string QPP_LM_CTRL_PATH = QPP_LM_PATH + "controls/";
const std::string QPP_LM_CTRL_WRISTS_PATH = QPP_LM_CTRL_PATH + "wrists";

// writing paths
std::string QPP_LM_ENABLED = QPP_LM_PATH + "enabled";
std::string QPP_LM_SELECTED_STATE = QPP_LM_PATH + "selected_pose_state";
std::string QPP_LM_SELECTED_POSE = QPP_LM_PATH + "selected_pose";

std::string POSE_HAND_OPEN = "hand_open";
std::string POSE_HAND_CLOSED = "hand_closed";
std::string POSE_PEACE = "peace";
std::string POSE_THUMBS_UP = "thumb_up";

}  // namespace

using QPPType = QPPReader::QPPType;

LandmarkPublisher::LandmarkPublisher() : Node("landmark_publisher") {
  this->declare_parameter<std::string>("qpp_base_path", "/dev/qpp/");
  qpp_base_path_ = this->get_parameter("qpp_base_path").as_string();

  this->declare_parameter<std::vector<double>>("cart_min_limits",
                                               {-0.25, -0.25, 0.05});
  this->declare_parameter<std::vector<double>>("cart_max_limits",
                                               {0.25, 0.25, 0.35});

  /**
   * @note These limits MUST match the ik solver exactly
   */
  // --- Load Cartesian Limits ---
  auto cart_min = this->get_parameter("cart_min_limits").as_double_array();
  auto cart_max = this->get_parameter("cart_max_limits").as_double_array();

  if (cart_min.size() != 3 || cart_max.size() != 3) {
    RCLCPP_WARN(this->get_logger(),
                "Cartesian limits arrays invalid. Using defaults.");
    cart_limits_[0] = {.min = -0.25, .max = 0.25};
    cart_limits_[1] = {.min = -0.25, .max = 0.25};
    cart_limits_[2] = {.min = -0.25, .max = 0.25};
  } else {
    cart_limits_[0] = {.min = cart_min[0], .max = cart_max[0]};
    cart_limits_[1] = {.min = cart_min[1], .max = cart_max[1]};
    cart_limits_[2] = {.min = cart_min[2], .max = cart_max[2]};
  }

  RCLCPP_INFO(this->get_logger(), "Cartesian limits:");
  RCLCPP_INFO(this->get_logger(), "  X: [%.3f, %.3f] m", cart_limits_[0].min,
              cart_limits_[0].max);
  RCLCPP_INFO(this->get_logger(), "  Y: [%.3f, %.3f] m", cart_limits_[1].min,
              cart_limits_[1].max);
  RCLCPP_INFO(this->get_logger(), "  Z: [%.3f, %.3f] m", cart_limits_[2].min,
              cart_limits_[2].max);
}

LandmarkPublisher::~LandmarkPublisher() { qpp_reader_->stop(); }

int LandmarkPublisher::go() {
  RCLCPP_INFO(get_logger(), "Starting Landmark publisher");

  /**
   * Publisher to the IK node which controls the robot
   */
  target_pos_pub_ =
      create_publisher<geometry_msgs::msg::Point>("/TargetPositions", 10);

  joint_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/joint", 10);

  ttl_timer_ =
      this->create_wall_timer(std::chrono::seconds(1), [this]() -> void {
        std::unique_lock<std::mutex> lk(this->selection_mtx_);
        if (this->selected_state_ != SelectionState::NO_ONE) {
          auto &hand = (this->selected_pose_ == QPP_LM_LEFT_HAND_PATH)
                           ? left_hand_
                           : right_hand_;
          auto now = std::chrono::steady_clock::now();

          if ((now - hand.last_detection) > HandLandmark::DETECTION_TTL) {
            this->update_selection_state("", SelectionState::NO_ONE);
          }
        }
      });

  /**
   * Configure all the signals we need.
   *
   * @note QPP reader is single threaded so thread safety isn't needed between
   * values.
   */
  qpp_reader_ = std::make_unique<QPPReader>(qpp_base_path_);
  qpp_reader_->add_value_listener(
      QPP_BRA_MODE_PATH, {.type = QPPType::STRING},
      std::bind(&LandmarkPublisher::qpp_bra_mode, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));

  // Robot kill-switch (read the control to service the request)
  qpp_reader_->add_value_listener(
      QPP_BRA_ENABLED_PATH,
      {.type = QPPType::BOOL, .is_array = false, .is_ctrl = true},
      std::bind(&LandmarkPublisher::qpp_bra_enable, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));

  qpp_reader_->add_value_listener(
      QPP_LM_INFO_PATH, {.type = QPPType::STRING},
      std::bind(&LandmarkPublisher::qpp_lm_info, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));

  // Save ROI
  qpp_reader_->add_value_listener(
      QPP_LM_ROI_PATH, {.type = QPPType::INT8, .is_array = true},
      std::bind(&LandmarkPublisher::qpp_lm_roi, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));

  // Have the same update path for both.
  auto detection_func = std::bind(&LandmarkPublisher::qpp_lm_detection, this,
                                  std::placeholders::_1, std::placeholders::_2,
                                  std::placeholders::_3);
  qpp_reader_->add_value_listener(QPP_LM_LEFT_HAND_PATH,
                                  {.type = QPPType::STRING}, detection_func);
  qpp_reader_->add_value_listener(QPP_LM_RIGHT_HAND_PATH,
                                  {.type = QPPType::STRING}, detection_func);

  // Controls
  qpp_reader_->add_value_listener(
      QPP_LM_CTRL_WRISTS_PATH, {.type = QPPType::BOOL, .is_ctrl = true},
      std::bind(&LandmarkPublisher::qpp_lm_ctrl_wrists, this,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));

  return qpp_reader_->go();
}

void LandmarkPublisher::qpp_bra_mode(const std::string &name,
                                     const QPPReader::QPPValue &value,
                                     const QPPReader::QPPTypeInfo &) {
  const std::string &mode = std::get<std::string>(value);
  RCLCPP_INFO_STREAM(get_logger(), name << ": " << mode);

  std::lock_guard<std::mutex> lk(selection_mtx_);
  if (publish_ != (mode == PUBLISH_MODE)) {
    pinching_ = false;
  }

  // Depending on if our mode matches we should flip the enable flag for ML.
  if (mode == PUBLISH_MODE) {
    publish_ = true;
    qpp_reader_->write_value(
        QPP_LM_ENABLED,
        {.type = QPPType::BOOL, .is_array = false, .is_ctrl = true}, true);
  } else {
    publish_ = false;
    qpp_reader_->write_value(
        QPP_LM_ENABLED,
        {.type = QPPType::BOOL, .is_array = false, .is_ctrl = true}, false);
  }
}

void LandmarkPublisher::qpp_bra_enable(const std::string &,
                                       const QPPReader::QPPValue &value,
                                       const QPPReader::QPPTypeInfo &) {
  enabled_ = std::get<bool>(value);

  // Write the value to ack the update
  qpp_reader_->write_value(QPP_BRA_ENABLED_PATH, {.type = QPPType::BOOL},
                           enabled_);
}

void LandmarkPublisher::qpp_lm_roi(const std::string &name,
                                   const QPPReader::QPPValue &value,
                                   const QPPReader::QPPTypeInfo &) {
  const auto &roi = std::get<QPPReader::QPPIntArray>(value);
  if (roi.size() != 4) {
    RCLCPP_WARN_STREAM(get_logger(), "Invalid ROI array with size="
                                         << roi.size() << " (expected size=4)");
    return;
  } else if ((roi[0] >= roi[2]) || (roi[1] >= roi[3])) {
    RCLCPP_WARN(get_logger(),
                "Invalid ROI array arguments x_end and y_end must be "
                "greater than x and y respectively");
    return;
  }

  for (auto x : roi) {
    if (x < 0 || x > 100) {
      RCLCPP_WARN(get_logger(), "Invalid ROI array arguments must be [0,100]");
      return;
    }
  }

  roi_ = {.x = static_cast<int8_t>(roi[0]),
          .y = static_cast<int8_t>(roi[1]),
          .end_x = static_cast<int8_t>(roi[2]),
          .end_y = static_cast<int8_t>(roi[3])};
  RCLCPP_INFO(get_logger(), "%s Set to %d,%d,%d,%d", name.c_str(), roi_.x,
              roi_.y, roi_.end_x, roi_.end_y);
}

void LandmarkPublisher::qpp_lm_info(const std::string &name,
                                    const QPPReader::QPPValue &value,
                                    const QPPReader::QPPTypeInfo &) {
  const std::string &info = std::get<std::string>(value);
  RCLCPP_INFO_STREAM(get_logger(), name << ": " << info);

  try {
    // Hard code to camera for now because it isn't reporting correctly
    auto j = nlohmann::json::parse(info);
    screen_size_.width = j.at("width");
    screen_size_.height = j.at("height");
  } catch (std::exception &e) {
    RCLCPP_WARN_STREAM(get_logger(), name << ": Invalid JSON");
    return;
  }
}

float LandmarkPublisher::dist_sq(const HandLandmark::KeyPoint &a,
                                 const HandLandmark::KeyPoint &b, bool use_z) {
  float dx = a.x - b.x;
  float dy = a.y - b.y;
  float dz = 0;
  if (use_z) {  // Z is relative might not be useful here
    dz = a.z - b.z;
  }
  return dx * dx + dy * dy + dz * dz;
}

// Checks if a point is within 2 points (this works if the image is flipped or
// not) (we can do this by distance but don't sqrt it to lower resources)
int LandmarkPublisher::is_finger_closed(const HandLandmark::KeyPoint &wrist,
                                        const HandLandmark::KeyPoint &mcp,
                                        const HandLandmark::KeyPoint &tip) {
  return (dist_sq(wrist, tip) < dist_sq(wrist, mcp)) ? 1 : 0;
};

float LandmarkPublisher::finger_angle(const HandLandmark::KeyPoint &mcp,
                                      const HandLandmark::KeyPoint &tip) {
  float y = tip.y - mcp.y;
  float len = std::sqrtf(dist_sq(tip, mcp));
  if (len == 0) {
    return 0;
  }
  return std::acosf(y / len) * 180.0f / M_PI;
}

bool LandmarkPublisher::is_hand_closed(const HandLandmark &landmark) {
  if (landmark.detected_poses_provided) {
    // No detection needed we can just use the provided insight (requires less
    // resources so prefer this)
    return std::find(landmark.detected_poses.begin(),
                     landmark.detected_poses.end(),
                     POSE_HAND_CLOSED) != landmark.detected_poses.end();
  }

  // C++ array need 2 {{}} wrapped for some weird reason...
  static std::array<std::array<KP, 2>, 5> fingers = {{
      {KP::THUMB_MCP, KP::THUMB_TIP},
      {KP::INDEX_FINGER_MCP, KP::INDEX_FINGER_TIP},
      {KP::MIDDLE_FINGER_MCP, KP::MIDDLE_FINGER_TIP},
      {KP::RING_FINGER_MCP, KP::RING_FINGER_TIP},
      {KP::PINKY_MCP, KP::PINKY_TIP},
  }};

  // Okay lets manually guess assuming that we say if 3 of 5 fingers are closed
  // a hand is closed
  int closed_fingers = 0;
  for (const auto &finger : fingers) {
    closed_fingers +=
        is_finger_closed(landmark.keypoints[static_cast<int>(KP::WRIST)],
                         landmark.keypoints[static_cast<int>(finger[0])],
                         landmark.keypoints[static_cast<int>(finger[1])]);
  }

  return closed_fingers >= 3;
}

bool LandmarkPublisher::is_peace(const HandLandmark &landmark) {
  if (landmark.detected_poses_provided) {
    // No detection needed we can just use the provided insight (requires less
    // resources so prefer this)
    return std::find(landmark.detected_poses.begin(),
                     landmark.detected_poses.end(),
                     POSE_PEACE) != landmark.detected_poses.end();
  }

  /**
   * The peace sign is your hand is closed besides your middle and index
   * pointing up. we wont worry about point up as it is such an awkward pose
   * anyways, and should always use ML if supported
   */

  // C++ array need 2 {{}} wrapped for some weird reason...
  static std::array<std::array<KP, 2>, 5> fingers_open = {{
      {KP::INDEX_FINGER_MCP, KP::INDEX_FINGER_TIP},
      {KP::MIDDLE_FINGER_MCP, KP::MIDDLE_FINGER_TIP},
  }};

  static std::array<std::array<KP, 2>, 5> fingers_closed = {{
      {KP::THUMB_MCP, KP::THUMB_TIP},
      {KP::RING_FINGER_MCP, KP::RING_FINGER_TIP},
      {KP::PINKY_MCP, KP::PINKY_TIP},
  }};

  int closed_fingers = 0;
  for (const auto &finger : fingers_closed) {
    closed_fingers +=
        is_finger_closed(landmark.keypoints[static_cast<int>(KP::WRIST)],
                         landmark.keypoints[static_cast<int>(finger[0])],
                         landmark.keypoints[static_cast<int>(finger[1])]);
  }

  int open_fingers = 0;
  for (const auto &finger : fingers_open) {
    open_fingers +=
        !is_finger_closed(landmark.keypoints[static_cast<int>(KP::WRIST)],
                          landmark.keypoints[static_cast<int>(finger[0])],
                          landmark.keypoints[static_cast<int>(finger[1])]);
  }

  return closed_fingers == 3 && open_fingers == 2;
}

bool LandmarkPublisher::is_hand_open(const HandLandmark &landmark) {
  if (landmark.detected_poses_provided) {
    // No detection needed we can just use the provided insight (requires less
    // resources so prefer this)
    return std::find(landmark.detected_poses.begin(),
                     landmark.detected_poses.end(),
                     POSE_HAND_OPEN) != landmark.detected_poses.end();
  }

  const auto &kp = landmark.keypoints;
  static std::array<std::array<KP, 2>, 5> fingers = {{
      {KP::THUMB_MCP, KP::THUMB_TIP},
      {KP::INDEX_FINGER_MCP, KP::INDEX_FINGER_TIP},
      {KP::MIDDLE_FINGER_MCP, KP::MIDDLE_FINGER_TIP},
      {KP::RING_FINGER_MCP, KP::RING_FINGER_TIP},
      {KP::PINKY_MCP, KP::PINKY_TIP},
  }};

  int closed_fingers = 0;
  for (const auto &finger : fingers) {
    closed_fingers += is_finger_closed(kp[static_cast<int>(KP::WRIST)],
                                       kp[static_cast<int>(finger[0])],
                                       kp[static_cast<int>(finger[1])]);
  }

  // consider a hand open only if all fingers are open
  return closed_fingers == 0;
}

bool LandmarkPublisher::is_thumb_up(const HandLandmark &landmark) {
  if (landmark.detected_poses_provided) {
    // No detection needed we can just use the provided insight (requires less
    // resources so prefer this)
    return std::find(landmark.detected_poses.begin(),
                     landmark.detected_poses.end(),
                     POSE_THUMBS_UP) != landmark.detected_poses.end();
  }

  // C++ array need 2 {{}} wrapped for some weird reason...
  static std::array<std::array<KP, 2>, 4> fingers = {{
      {KP::INDEX_FINGER_DIP, KP::INDEX_FINGER_TIP},
      {KP::MIDDLE_FINGER_DIP, KP::MIDDLE_FINGER_TIP},
      {KP::RING_FINGER_DIP, KP::RING_FINGER_TIP},
      {KP::PINKY_DIP, KP::PINKY_TIP},
  }};

  int closed_fingers = 0;
  for (const auto &finger : fingers) {
    closed_fingers +=
        is_finger_closed(landmark.keypoints[static_cast<int>(KP::WRIST)],
                         landmark.keypoints[static_cast<int>(finger[0])],
                         landmark.keypoints[static_cast<int>(finger[1])]);

    // Make sure the thumb is actually up we can do this by checking
    // the the thumb middle digit is higher than the tip of all other fingers
    // remembering y is from top so the sign is flipped.
    if (landmark.keypoints[static_cast<int>(KP::THUMB_MCP)].y >
        landmark.keypoints[static_cast<int>(finger[1])].y) {
      return false;
    }
  }

  // Make sure all other fingers are closed and thumb is extended
  // But really just check 3 as it is close enough
  if (closed_fingers < 3 ||
      is_finger_closed(landmark.keypoints[static_cast<int>(KP::WRIST)],
                       landmark.keypoints[static_cast<int>(KP::THUMB_MCP)],
                       landmark.keypoints[static_cast<int>(KP::THUMB_TIP)])) {
    return false;
  }

  auto angle =
      finger_angle(landmark.keypoints[static_cast<int>(KP::THUMB_MCP)],
                   landmark.keypoints[static_cast<int>(KP::THUMB_TIP)]);

  return (angle > 140);
}

bool LandmarkPublisher::is_pinching(const HandLandmark &landmark,
                                    bool already_pinching) {
  const auto &kp = landmark.keypoints;

  const float palm_width_sq =
      dist_sq(kp[static_cast<int>(KP::INDEX_FINGER_MCP)],
              kp[static_cast<int>(KP::PINKY_MCP)]);
  const float pinch_distance_sq =
      dist_sq(kp[static_cast<int>(KP::THUMB_TIP)],
              kp[static_cast<int>(KP::INDEX_FINGER_TIP)]);
  // A wider release threshold prevents mode flicker near the contact point.
  const float ratio =
      already_pinching ? PINCH_HOLD_DIST_SQ_TH : PINCH_DIST_SQ_TH;
  if (!std::isfinite(palm_width_sq) || palm_width_sq <= 1.0f ||
      !std::isfinite(pinch_distance_sq) ||
      pinch_distance_sq >= ratio * ratio * palm_width_sq) {
    return false;
  }

  // Check the other 3 fingers are not closed
  // C++ array need 2 {{}} wrapped for some weird reason...
  static std::array<std::array<KP, 2>, 3> fingers = {{
      {KP::MIDDLE_FINGER_MCP, KP::MIDDLE_FINGER_TIP},
      {KP::RING_FINGER_MCP, KP::RING_FINGER_TIP},
      {KP::PINKY_MCP, KP::PINKY_TIP},
  }};

  for (const auto &finger : fingers) {
    if (is_finger_closed(landmark.keypoints[static_cast<int>(KP::WRIST)],
                         landmark.keypoints[static_cast<int>(finger[0])],
                         landmark.keypoints[static_cast<int>(finger[1])])) {
      return false;
    }
  }

  return true;
}

void LandmarkPublisher::qpp_lm_detection(const std::string &name,
                                         const QPPReader::QPPValue &value,
                                         const QPPReader::QPPTypeInfo &) {
  if (!publish_ || !enabled_) {
    RCLCPP_DEBUG_STREAM(get_logger(),
                        name << " received detection not not publishing");
    return;
  }

  const std::string &detect_str = std::get<std::string>(value);
  if (detect_str.empty()) {
    // No object
    return;
  }

  RCLCPP_DEBUG_STREAM(get_logger(), name << ": " << detect_str);
  HandLandmark landmark;
  try {
    auto detect = nlohmann::json::parse(detect_str);
    if (detect.size() == 0) {
      // No object ignore
      return;
    }

    // Don't write into the landmark directly in case there is a parse error
    if (detect.contains("bbox")) {
      // If the confidence is low don't waste time parsing the rest
      landmark.conf = detect.at("bbox").at("conf");
      if (landmark.conf < detection_threshold_) {
        RCLCPP_INFO_STREAM(get_logger(), name
                                             << " low confidence ignoring conf="
                                             << landmark.conf);
        std::lock_guard<std::mutex> lk(selection_mtx_);
        if (selected_state_ == SelectionState::SELECTING) {
          update_selection_state("", SelectionState::NO_ONE);
        }
        return;
      }

      // BBOX
      landmark.bbox.x = detect.at("bbox").at("x");
      landmark.bbox.y = detect.at("bbox").at("y");
      landmark.bbox.width = detect.at("bbox").at("w");
      landmark.bbox.height = detect.at("bbox").at("h");
    }

    // Poses
    if (detect.contains("pose_detected")) {
      landmark.detected_poses =
          detect.at("pose_detected").get<std::vector<std::string>>();
      landmark.detected_poses_provided = !landmark.detected_poses.empty();
    } else {
      landmark.detected_poses_provided = false;
    }

    // Landmarks
    if (detect.at("keypoints").size() !=
        static_cast<size_t>(HandLandmark::Names::MAX) + 1) {
      RCLCPP_WARN_STREAM(get_logger(), name << "Expected 21 key points");
      return;
    }

    int i = 0;
    for (const auto &kp : detect["keypoints"]) {
      landmark.keypoints[i].x = kp.at("x");
      landmark.keypoints[i].y = kp.at("y");
      if (kp.contains("z")) {
        landmark.keypoints[i].z = kp.at("z");
      } else {
        // Simulated Z if one wasn't given
        landmark.keypoints[i].z = 0.0;
      }
      landmark.keypoints[i].conf = kp.at("conf");
      i++;
    }

  } catch (const std::exception &e) {
    RCLCPP_WARN_STREAM(get_logger(), name << ": Invalid JSON - " << e.what());
    return;
  }

  // Must be locked as our TTL wall timer also access the hands.
  {
    std::lock_guard<std::mutex> lk(selection_mtx_);
    // Parse success write into the hand
    auto &hand = (name == QPP_LM_LEFT_HAND_PATH) ? left_hand_ : right_hand_;
    hand = std::move(landmark);
    hand.last_detection = std::chrono::steady_clock::now();

    // First try to select a landmark if one is
    // already selected this is a no-op then publish if needed
    select_landmarks(name, hand);
    publish_landmark_position(name, hand);
  }
}

void LandmarkPublisher::qpp_lm_ctrl_wrists(const std::string &,
                                           const QPPReader::QPPValue &value,
                                           const QPPReader::QPPTypeInfo &) {
  use_wrists_ = std::get<bool>(value);

  // Write the value to ack the update
  qpp_reader_->write_value(QPP_LM_CTRL_WRISTS_PATH, {.type = QPPType::BOOL},
                           use_wrists_);
}

float LandmarkPublisher::get_palm_len_sq(const HandLandmark &landmark) {
  return (dist_sq(landmark.keypoints[static_cast<int>(KP::INDEX_FINGER_MCP)],
                  landmark.keypoints[static_cast<int>(KP::WRIST)]) +
          dist_sq(landmark.keypoints[static_cast<int>(KP::INDEX_FINGER_MCP)],
                  landmark.keypoints[static_cast<int>(KP::PINKY_MCP)]) +
          dist_sq(landmark.keypoints[static_cast<int>(KP::PINKY_MCP)],
                  landmark.keypoints[static_cast<int>(KP::WRIST)])) /
         3.0f;
}

LandmarkPublisher::HandLandmark::KeyPoint LandmarkPublisher::get_avg_palm_kp(
    const HandLandmark &landmark) {
  HandLandmark::KeyPoint point = {.x = 0, .y = 0, .z = 0, .conf = 0};

  static std::array<KP, 3> points = {
      {KP::WRIST, KP::INDEX_FINGER_MCP, KP::PINKY_MCP}};
  for (const auto &p : points) {
    point.x += landmark.keypoints[static_cast<int>(p)].x;
    point.y += landmark.keypoints[static_cast<int>(p)].y;
    point.z += landmark.keypoints[static_cast<int>(p)].z;
    point.conf += landmark.keypoints[static_cast<int>(p)].conf;
  }

  point.x /= points.size();
  point.y /= points.size();
  point.z /= points.size();
  point.conf /= points.size();
  return point;
}

void LandmarkPublisher::select_landmarks(const std::string &name,
                                         HandLandmark &landmark) {
  auto now = std::chrono::steady_clock::now();

  switch (selected_state_) {
    case SelectionState::SELECTED:
      if (name != selected_pose_) {
        return;
      }
      if (is_thumb_up(landmark)) {
        if ((now - deselection_started_) > deselection_timer_) {
          update_selection_state("", SelectionState::NO_ONE);
        }
      } else {
        // Always just assume we are deselecting as long as we
        // get a frame in the next 1 second we are fine and if we don't
        // we can't trust our data anyways.
        deselection_started_ = now;
      }
      break;
    case SelectionState::SELECTING:
      // See if they are still making the pose and this is the correct person
      if (name != selected_pose_) {
        return;
      }

      // Verify that we can still find the hand closed position for detection.
      if (!is_peace(landmark)) {
        // No longer making selection pose.
        update_selection_state("", SelectionState::NO_ONE);
        return;
      }

      if ((now - selection_started_) > selection_timer_) {
        // They have made the pose long enough select them
        update_selection_state(name, SelectionState::SELECTED);
        init_palm_len_sq_ = get_palm_len_sq(landmark);
      }
      break;
    case SelectionState::NO_ONE:
      if (is_peace(landmark)) {
        update_selection_state(name, SelectionState::SELECTING);
        selection_started_ = now;
      }
      break;
  }
}

// Map a point from 1 range to another used to map from the screen to the IK
// range
static double map_range(double x, double in_min, double in_max, double out_min,
                        double out_max) {
  // Clamp x to the input range where this can happen if ROI isn't implemented
  // in the ML
  x = std::clamp(x, in_min, in_max);
  return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

void LandmarkPublisher::publish_landmark_position(
    const std::string &name, const HandLandmark &landmark) {
  if (selected_state_ != SelectionState::SELECTED || name != selected_pose_) {
    return;
  }

  if (screen_size_.width <= 0 || screen_size_.height <= 0 ||
      !std::isfinite(init_palm_len_sq_) || init_palm_len_sq_ <= 0) {
    return;
  }

  // Apply the ROI so the full range of motion can be applied within the ROI Box
  // also add a buffer so that you can reach the edges easier
  auto start_x = roi_.x / 100.f * screen_size_.width;
  auto end_x = roi_.end_x / 100.f * screen_size_.width;
  auto start_y = roi_.y / 100.f * screen_size_.height;
  auto end_y = roi_.end_y / 100.f * screen_size_.height;

  // Use the palm instead of the full hand as open and close will cause more
  // jitter of movement if whole hand is considered
  auto avg_palm_pos = get_avg_palm_kp(landmark);

  auto point = geometry_msgs::msg::Point();
  point.x = -map_range(avg_palm_pos.x, start_x, end_x, cart_limits_[0].min,
                       cart_limits_[0].max);

  // Height is from top down.
  point.y = map_range(screen_size_.height - avg_palm_pos.y, start_y, end_y,
                      cart_limits_[1].min, cart_limits_[1].max);

  /**
   * For Z we need to guess how much we should go forward and back.
   * To do this we use the width of the hand so as it gets closer it moves
   * backwards and has it pulls away so does the robot
   */
  auto palm_len = get_palm_len_sq(landmark);
  point.z =
      map_range(palm_len, init_palm_len_sq_ * 0.5, init_palm_len_sq_ * 1.5,
                cart_limits_[2].min, cart_limits_[2].max);
  RCLCPP_DEBUG(get_logger(), "init = %f, z val = %f -> %f", init_palm_len_sq_,
               palm_len, avg_palm_pos.z);

  if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z) || !std::isfinite(avg_palm_pos.z)) {
    return;
  }

  point.x = std::clamp(point.x, cart_limits_[0].min, cart_limits_[0].max);
  point.y = std::clamp(point.y, cart_limits_[1].min, cart_limits_[1].max);
  point.z = std::clamp(point.z, cart_limits_[2].min, cart_limits_[2].max);

  RCLCPP_DEBUG(
      get_logger(), "Publishing position screen=%dx%d %f->%f %f->%f %f->%f",
      screen_size_.width, screen_size_.height, landmark.keypoints[0].x, point.x,
      landmark.keypoints[0].y, point.y, landmark.keypoints[0].z, point.z);
  target_pos_pub_->publish(point);

  auto msg = sensor_msgs::msg::JointState();

  // Pinching
  {
    // If this is our first time pinching update the relative pinch z so it
    // doesn't jump on connect
    auto pinch = is_pinching(landmark, pinching_);
    auto pinch_value =
        landmark.keypoints[static_cast<int>(KP::MIDDLE_FINGER_MCP)].z;

    if (pinch && !pinching_) {
      pinch_offset_ =
          std::clamp(last_pinch_z_ - pinch_value, MIN_PINCH_Z, MAX_PINCH_Z);
      RCLCPP_DEBUG(get_logger(), "Pinch with offset %f", pinch_offset_);
    }
    pinching_ = pinch;

    msg.name.push_back("hand");
    if (pinching_) {
      // Only update the value when pinching.
      last_pinch_z_ =
          std::clamp(pinch_value + pinch_offset_, MIN_PINCH_Z, MAX_PINCH_Z);

      RCLCPP_DEBUG(get_logger(), "Pinch val %f offset %f", last_pinch_z_,
                   pinch_offset_);
    }
    msg.position.push_back(
        map_range(last_pinch_z_, MIN_PINCH_Z, MAX_PINCH_Z, -M_PI_2, M_PI_2));
  }

  // Gripper
  {
    /**
     * To avoid the hand opening when moving around there is a small delay
     * before we open the hand. This allows use to get a clean frame between
     * fast movements.
     *
     * We also dont want to open the gripper when pinching to allow fine
     * movements with the gripper closed.
     */
    auto now = std::chrono::steady_clock::now();
    if (is_hand_open(landmark) && !pinching_) {
      if ((now - hand_open_started_) > hand_open_timer_) {
        gripper_command_ = M_PI_2;
      }
    } else {
      if (is_hand_closed(landmark)) {
        gripper_command_ = -M_PI_2;
      }
      hand_open_started_ = now;
    }

    msg.name.push_back("gripper");
    msg.position.push_back(gripper_command_);
  }

  // Wrists -- Only use writs if enabled as it is hard on the wrists
  if (use_wrists_) {
    msg.name.push_back("wrist");
    msg.position.push_back(map_range(
        landmark.keypoints[static_cast<int>(KP::PINKY_MCP)].z -
            landmark.keypoints[static_cast<int>(KP::INDEX_FINGER_MCP)].z,
        -0.03, 0.02, -M_PI_2, M_PI_2));
  }

  joint_pub_->publish(msg);
}

void LandmarkPublisher::home_robot() {
  auto point = geometry_msgs::msg::Point();
  point.x = (cart_limits_[0].max + cart_limits_[0].min) / 2.0;
  point.y = (cart_limits_[1].max + cart_limits_[1].min) / 2.0;
  point.z = (cart_limits_[2].max + cart_limits_[2].min) / 2.0;
  target_pos_pub_->publish(point);

  pinching_ = false;
  gripper_command_ = 0;
  last_pinch_z_ = 0;

  auto msg = sensor_msgs::msg::JointState();
  for (const auto &joint : {"gripper", "hand", "wrist"}) {
    msg.name.push_back(joint);
    msg.position.push_back(0);
  }

  joint_pub_->publish(msg);
}

void LandmarkPublisher::update_selection_state(const std::string &name,
                                               SelectionState state) {
  if (name != selected_pose_) {
    selected_pose_ = name;
    RCLCPP_INFO_STREAM(get_logger(),
                       "Updating selected pose too " << selected_pose_);
    qpp_reader_->write_value(QPP_LM_SELECTED_POSE, {.type = QPPType::STRING},
                             std::filesystem::path(selected_pose_).filename());
  }

  if (state != selected_state_) {
    std::string str = "";
    selected_state_ = state;
    switch (selected_state_) {
      case SelectionState::NO_ONE:
        home_robot();
        str = "noone";
        break;
      case SelectionState::SELECTING:
        str = "selecting";
        break;
      case SelectionState::SELECTED:
        str = "selected";
        break;
    }
    RCLCPP_INFO_STREAM(get_logger(), "Updating selection state too " << str);
    qpp_reader_->write_value(QPP_LM_SELECTED_STATE, {.type = QPPType::STRING},
                             str);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto lm_pub = std::make_shared<LandmarkPublisher>();
  auto ret = lm_pub->go();
  if (ret != EOK) {
    return EXIT_FAILURE;
  }
  rclcpp::spin(lm_pub);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}