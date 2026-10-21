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
#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "qpp_reader.hpp"

class LandmarkPublisher : public rclcpp::Node {
 private:
  using TimePoint = std::chrono::steady_clock::time_point;

  struct ROIBounds {
    int8_t x;
    int8_t y;
    int8_t end_x;
    int8_t end_y;
  };

  struct HandLandmark {
    // Index a key point is in the keypoints map
    enum class Names {
      WRIST = 0,
      THUMB_CMC,
      THUMB_MCP,
      THUMB_IP,
      THUMB_TIP,
      INDEX_FINGER_MCP,
      INDEX_FINGER_PIP,
      INDEX_FINGER_DIP,
      INDEX_FINGER_TIP,
      MIDDLE_FINGER_MCP,
      MIDDLE_FINGER_PIP,
      MIDDLE_FINGER_DIP,
      MIDDLE_FINGER_TIP,
      RING_FINGER_MCP,
      RING_FINGER_PIP,
      RING_FINGER_DIP,
      RING_FINGER_TIP,
      PINKY_MCP,
      PINKY_PIP,
      PINKY_DIP,
      PINKY_TIP,
      MAX = PINKY_TIP
    };

    struct KeyPoint {
      float x;
      float y;
      float z;
      float conf;
    };

    struct BBox {
      float x;
      float y;
      float width;
      float height;
    };

    // Overall confidence
    float conf;
    BBox bbox;  // Bounding box
    std::array<KeyPoint, static_cast<size_t>(Names::MAX) + 1> keypoints;
    std::vector<std::string> detected_poses;  // hand_open, hand_close
    bool
        detected_poses_provided;  // If hand_open and hand closed are provided
                                  // or if I need to figure them out in this app

    /// How long a pose can be not detected before focus is lost
    static constexpr std::chrono::seconds DETECTION_TTL{2};

    /// Timepoint which the landmark was last seen.
    TimePoint last_detection;
  };
  using KP = HandLandmark::Names;


  enum class SelectionState {
    NO_ONE,
    SELECTING,
    SELECTED,
  };

  struct ScreenSize {
    int width;
    int height;
  };

  struct CartLimit {
    double min;
    double max;
  };

 public:
  LandmarkPublisher();

  int go();

  ~LandmarkPublisher();

 private:
  // Callbacks from QPP
  void qpp_bra_mode(const std::string &name, const QPPReader::QPPValue &value,
                    const QPPReader::QPPTypeInfo &info);
  void qpp_bra_enable(const std::string &name, const QPPReader::QPPValue &value,
                      const QPPReader::QPPTypeInfo &info);
  void qpp_lm_roi(const std::string &name, const QPPReader::QPPValue &value,
                  const QPPReader::QPPTypeInfo &info);
  void qpp_lm_info(const std::string &name, const QPPReader::QPPValue &value,
                   const QPPReader::QPPTypeInfo &info);
  void qpp_lm_detection(const std::string &name,
                        const QPPReader::QPPValue &value,
                        const QPPReader::QPPTypeInfo &info);
  void qpp_lm_ctrl_wrists(const std::string &name,
                          const QPPReader::QPPValue &value,
                          const QPPReader::QPPTypeInfo &info);

  /**
   * Selects a landmark based on it's selected poses
   */
  void select_landmarks(const std::string &name, HandLandmark &landmark);

  /**
   * Publish the landmark position for pose
   */
  void publish_landmark_position(const std::string &name,
                                 const HandLandmark &landmark);

  /**
   * Updates the selection state and pushes changes to qpp
   */
  void update_selection_state(const std::string &name, SelectionState state);
  void home_robot();

  /**
   * Figures out if the hand is detected as closed. If detected_poses_provided
   * is provided detected_poses vector will be used otherwise it will be
   * calculated by using the joints.
   *
   * @param landmark The hand to check if it is closed.
   *
   * @return true iff the hand is closed, otherwise false.
   */
  static bool is_hand_closed(const HandLandmark &landmark);
  static bool is_hand_open(const HandLandmark &landmark);
  static bool is_thumb_up(const HandLandmark &landmark);
  static bool is_peace(const HandLandmark &landmark);
  static bool is_pinching(const HandLandmark &landmark, bool already_pinching);

  static int is_finger_closed(const HandLandmark::KeyPoint &wrist,
                              const HandLandmark::KeyPoint &mcp,
                              const HandLandmark::KeyPoint &tip);
  static float dist_sq(const HandLandmark::KeyPoint &a,
                       const HandLandmark::KeyPoint &b, bool use_z = true);
  static float finger_angle(const HandLandmark::KeyPoint &mcp,
                            const HandLandmark::KeyPoint &tip);

  static float get_palm_len_sq(const HandLandmark &landmark);

  /**
   * Gets the average point only for the palm using
   * WRIST, THUMB_CMC, INDEX_FINGER_MCP, PINKY_MCP
   */
  static HandLandmark::KeyPoint get_avg_palm_kp(const HandLandmark &landmark);

 private:
  std::string qpp_base_path_ = "/dev/qpp";

  bool publish_ = false;  /// Only publish when our mode matches what the robot
                          /// is asking for.
  bool enabled_ =
      true;  /// Blocks publishing if the kill switch is on similar to
             /// @publish_ but from an external control not the robot itself

  ROIBounds roi_ = {0, 0, 100, 100};  /// Helps us map robots x,y
  ScreenSize screen_size_ = {1920, 1080};
  float detection_threshold_ = 0.5;

  std::unique_ptr<QPPReader> qpp_reader_;

  /// Position Limits for physical robot. (x y z)
  /// @note this must match IK node exactly
  std::array<CartLimit, 3> cart_limits_{};

  /**
   * Selection state for movement.
   *
   * At any given time either the left or right hand can be selected but not both
   * This is defined by selected_pose_.
   *
   * The current state is defined by @selected_state_
   */
  std::mutex selection_mtx_{};
  HandLandmark left_hand_;
  HandLandmark right_hand_;
  SelectionState selected_state_ = SelectionState::NO_ONE;
  std::string selected_pose_ = "";

  /**
   * Timers for selection and deselecting hand landmarks.
   */
  TimePoint selection_started_{};
  std::chrono::seconds selection_timer_{2};
  TimePoint deselection_started_{};
  std::chrono::seconds deselection_timer_{1};

  // Open hand timer. Sometime the ML has a small moment where it detects the
  // hand is open when moving fast so don't open the hand unless multiple
  // detection show the hand is open.
  TimePoint hand_open_started_{};
  std::chrono::milliseconds hand_open_timer_{250};

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pos_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_{};
  rclcpp::TimerBase::SharedPtr ttl_timer_;

  // controls
  bool use_wrists_ = false;

  /// On first detection we save the z value to use that as our base for
  /// forward and backwards motion;
  float init_palm_len_sq_ = 0;

  // Pinch to adjust pitch; When adjusting we will use the relative Z to where
  // you started
  bool pinching_ = false;
  float last_pinch_z_ = 0; // The last reported value of z when pinching.
  float pinch_offset_ = 0;

  // Current state of robot.
  double gripper_command_ = 0;

  static constexpr float MIN_PINCH_Z = -0.02f;
  static constexpr float MAX_PINCH_Z = 0.03f;
  static constexpr float PINCH_DIST_SQ_TH = 0.20f;
  static constexpr float PINCH_HOLD_DIST_SQ_TH = 0.30f;



};