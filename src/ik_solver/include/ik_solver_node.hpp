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
 *
 * @file ik_solver_node.hpp
 * @brief Position-based inverse kinematics for the Baby Robot Arm.
 *
 * The Cartesian box in this node is an operational command bound, not a
 * collision-safety system. Joint limits are enforced here as a second layer,
 * but the actuator controller must always retain its independent calibrated
 * limits and the physical system still requires an E-stop/fail-off mechanism.
 */

#ifndef IK_SOLVER_NODE_HPP
#define IK_SOLVER_NODE_HPP

#include "safety_validation.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <urdf/model.h>

class IKSolverNode : public rclcpp::Node
{
public:
  IKSolverNode();

private:
  // Configuration and model loading.
  bool load_and_validate_parameters();
  std::string find_URDF(const std::string & filename);
  bool read_URDF_file(const std::string & path, std::string & xml) const;
  bool load_URDF();
  bool validate_model(const urdf::Model & model);
  void extract_joint_names();
  bool extract_joint_limits(const urdf::Model & model);
  bool init_solvers();

  // Safety validation.
  bool clamp_cartesian_target(KDL::Frame & target);
  bool frame_within_cartesian_bounds(const KDL::Frame & frame) const;
  bool validate_joint_solution(
    const KDL::JntArray & solution, bool enforce_step, std::string & reason) const;
  bool frame_is_finite(const KDL::Frame & frame) const;
  bool command_is_well_formed(
    const std_msgs::msg::Float64MultiArray & msg, bool & home_pressed,
    std::uint64_t & session_epoch) const;
  void advance_home_position();
  [[noreturn]] void fail_active_control(const std::string & reason) const;

  // ROS callbacks and publishing.
  void current_positions_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void cartesian_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void publish_joint_command(const KDL::JntArray & joint_angles);

  // Timing. A steady clock and fixed integration period prevent wall/ROS
  // clock changes and input frequency from changing commanded speed.
  std::chrono::steady_clock::time_point last_solve_time_;
  double update_rate_hz_ = 50.0;
  double nominal_period_sec_ = 0.02;
  double max_command_interval_sec_ = 0.25;

  // ROS communication. Topic names are relative so namespaces and SROS2
  // enclave policies can isolate individual robots.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    cartesian_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pos_subscription_;

  // URDF and KDL.
  std::string urdf_file_;
  std::string urdf_xml_;
  std::string base_link_;
  std::string end_effector_link_;
  KDL::Chain kdl_chain_;
  std::vector<std::string> joint_names_;
  const std::vector<std::string> expected_joint_names_{
    "base", "shoulder", "elbow", "wrist", "hand"};
  std::size_t expected_joint_count_ = 5U;
  unsigned int num_joints_ = 0U;

  // Cartesian workspace limits (meters in the base frame).
  double cart_x_min_ = -0.25;
  double cart_x_max_ = 0.25;
  double cart_y_min_ = -0.25;
  double cart_y_max_ = 0.25;
  double cart_z_min_ = 0.05;
  double cart_z_max_ = 0.35;

  // State and solver-enforced joint limits.
  KDL::JntArray current_joint_positions_;
  KDL::JntArray joint_lower_limits_;
  KDL::JntArray joint_upper_limits_;
  KDL::Frame cartesian_target_;
  bool target_initialized_ = false;
  ik_solver::safety::SessionEpochGate session_epoch_gate_;
  double max_joint_step_rad_ = 0.10;

  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_pos_solver_;

  double velocity_scale_ = 0.10;

  static constexpr std::size_t kMaxUrdfBytes = 1024U * 1024U;
  static constexpr double kMaxCartesianCommand = 1.0;
  static constexpr double kFlagTolerance = 1.0e-9;
};

#endif  // IK_SOLVER_NODE_HPP
