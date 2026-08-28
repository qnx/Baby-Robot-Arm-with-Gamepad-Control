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
 * @file ik_solver_node.cpp
 * @brief Implementation of the IK Solver ROS2 node.
 */

#include "ik_solver_node.hpp"
#include <fstream>
#include <kdl_parser/kdl_parser.hpp>
#include <unistd.h>

// ======================================================================
// Constructor
// ======================================================================

IKSolverNode::IKSolverNode() : Node("ik_solver_node") {
  RCLCPP_INFO(this->get_logger(), "IK Solver Node started.");

  last_update_time_ = this->now().seconds();

  // --- Declare ROS2 Parameters ---

  this->declare_parameter<std::string>("urdf_file", "arm5dof.urdf");
  this->declare_parameter<std::string>("base_link", "world");
  this->declare_parameter<std::string>("end_effector_link", "Arm_03_1");
  this->declare_parameter<double>("update_rate", 50.0);
  this->declare_parameter<double>("velocity_scale", 0.1);

  this->declare_parameter<std::vector<double>>("cart_min_limits", {-0.25, -0.25, 0.05});
  this->declare_parameter<std::vector<double>>("cart_max_limits", {0.25, 0.25, 0.35});

  // --- Load Cartesian Limits ---

  auto cart_min = this->get_parameter("cart_min_limits").as_double_array();
  auto cart_max = this->get_parameter("cart_max_limits").as_double_array();

  if (cart_min.size() >= 3 && cart_max.size() >= 3) {
    cart_x_min_ = cart_min[0];
    cart_y_min_ = cart_min[1];
    cart_z_min_ = cart_min[2];
    cart_x_max_ = cart_max[0];
    cart_y_max_ = cart_max[1];
    cart_z_max_ = cart_max[2];
  } else {
    RCLCPP_WARN(this->get_logger(), "Cartesian limits arrays too short. Using defaults.");
  }

  RCLCPP_INFO(this->get_logger(), "Cartesian limits:");
  RCLCPP_INFO(this->get_logger(), "  X: [%.3f, %.3f] m", cart_x_min_, cart_x_max_);
  RCLCPP_INFO(this->get_logger(), "  Y: [%.3f, %.3f] m", cart_y_min_, cart_y_max_);
  RCLCPP_INFO(this->get_logger(), "  Z: [%.3f, %.3f] m", cart_z_min_, cart_z_max_);

  // --- Load Other Parameters ---

  std::string urdf_filename = this->get_parameter("urdf_file").as_string();

  RCLCPP_INFO(this->get_logger(), "urdf_file parameter value: '%s'", urdf_filename.c_str());

  urdf_file_ = this->get_parameter("urdf_file").as_string();
  base_link_ = this->get_parameter("base_link").as_string();
  end_effector_link_ = this->get_parameter("end_effector_link").as_string();
  velocity_scale_ = this->get_parameter("velocity_scale").as_double();

  RCLCPP_INFO(this->get_logger(), "  Configuration:");
  RCLCPP_INFO(this->get_logger(), "  URDF file: %s", urdf_file_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Base link: %s", base_link_.c_str());
  RCLCPP_INFO(this->get_logger(), "  End effector: %s", end_effector_link_.c_str());

  // --- Find and Load URDF ---

  urdf_file_ = find_URDF(urdf_filename);

  if (urdf_file_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "URDF not found. Exiting.");
    rclcpp::shutdown();
    return;
  }

  if (!load_URDF()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF! Node will exit.");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "  URDF loaded successfully!");
  RCLCPP_INFO(this->get_logger(), "  KDL chain has %u joints", kdl_chain_.getNrOfJoints());
  RCLCPP_INFO(this->get_logger(), "  Joint names:");
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "    [%zu] %s", i, joint_names_[i].c_str());
  }

  // --- Initialize Solvers ---

  if (!init_solvers()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize solvers. Exiting.");
    rclcpp::shutdown();
    return;
  }

  // --- Set Up ROS2 Publishers and Subscriptions ---

  joint_command_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/mov", 10);

  cartesian_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/CartesianCmd", 10, std::bind(&IKSolverNode::cartesian_callback, this, std::placeholders::_1));

  target_positions_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/TargetPositions", 10, std::bind(&IKSolverNode::target_positions_callback, this, std::placeholders::_1));

  pos_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/CurrentPositions", 10, std::bind(&IKSolverNode::current_positions_callback, this, std::placeholders::_1));
}

// ======================================================================
// URDF Loading and Configuration
// ======================================================================

std::string IKSolverNode::find_URDF(const std::string &filename) {
  const char *config_dir = std::getenv("URDF_PATH");

  if (config_dir == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "URDF_PATH not set");
    return "";
  }

  std::string full_path = std::string(config_dir) + "/" + filename;

  RCLCPP_INFO(this->get_logger(), "Looking for URDF at: '%s'", full_path.c_str());

  int access_result = access(full_path.c_str(), F_OK);
  if (access_result == 0) {
    RCLCPP_INFO(this->get_logger(), "  URDF found!");
    return full_path;
  } else {
    RCLCPP_ERROR(this->get_logger(), "  URDF not found at path: '%s'", full_path.c_str());
    return "";
  }
}

bool IKSolverNode::load_URDF() {
  RCLCPP_INFO(this->get_logger(), "Loading URDF from: %s", urdf_file_.c_str());

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromFile(urdf_file_, kdl_tree)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree from URDF");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "KDL tree constructed");
  RCLCPP_INFO(this->get_logger(), "  Tree has %u segments", kdl_tree.getNrOfSegments());

  if (!kdl_tree.getChain(base_link_, end_effector_link_, kdl_chain_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to extract chain from '%s' to '%s'", base_link_.c_str(),
                 end_effector_link_.c_str());

    auto segments = kdl_tree.getSegments();
    RCLCPP_INFO(this->get_logger(), "Available segments in tree:");
    for (const auto &seg : segments) {
      RCLCPP_INFO(this->get_logger(), "  - %s", seg.first.c_str());
    }
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "  KDL chain extracted");
  RCLCPP_INFO(this->get_logger(), "  Chain: %s -> %s", base_link_.c_str(), end_effector_link_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Segments: %u", kdl_chain_.getNrOfSegments());
  RCLCPP_INFO(this->get_logger(), "  Joints: %u", kdl_chain_.getNrOfJoints());

  extract_joint_names();
  return true;
}

void IKSolverNode::extract_joint_names() {
  joint_names_.clear();

  for (unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); ++i) {
    KDL::Segment segment = kdl_chain_.getSegment(i);
    KDL::Joint joint = segment.getJoint();

    if (joint.getType() != KDL::Joint::None) {
      joint_names_.push_back(joint.getName());
    }
  }

  RCLCPP_INFO(this->get_logger(), "  Extracted %zu joint names", joint_names_.size());
}

// ======================================================================
// Solver Initialization
// ======================================================================

bool IKSolverNode::init_solvers() {
  // FK solver: joint angles → Cartesian position
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

  // LMA weight vector for 5-DOF position-priority solving
  // [X, Y, Z, Roll, Pitch, Yaw]
  Eigen::Matrix<double, 6, 1> L;
  L << 1, 1, 1, 0, 0, 0;

  // Position IK solver with position-priority weighting
  ik_pos_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_, L);

  // Initialize joint arrays
  num_joints_ = kdl_chain_.getNrOfJoints();
  current_joint_positions_ = KDL::JntArray(num_joints_);
  for (unsigned int i = 0; i < num_joints_; ++i) {
    current_joint_positions_(i) = 0.0;
  }

  // Compute home Cartesian position using FK
  // All joints at 0 rad = all servos at 50% = 1500µs neutral
  KDL::Frame initial_frame;
  fk_solver_->JntToCart(current_joint_positions_, initial_frame);
  cartesian_target_ = initial_frame;
  target_initialized_ = true;

  RCLCPP_INFO(this->get_logger(), "Solvers initialized");
  RCLCPP_INFO(this->get_logger(), "  Joints: %u", num_joints_);
  RCLCPP_INFO(this->get_logger(), "  Home cartesian position: [%.3f, %.3f, %.3f]", initial_frame.p.x(),
              initial_frame.p.y(), initial_frame.p.z());
  return true;
}

// ======================================================================
// Cartesian Workspace Limits
// ======================================================================

bool IKSolverNode::clamp_cartesian_target(KDL::Frame &target) {
  bool clamped = false;

  if (target.p.x() < cart_x_min_) {
    target.p.x(cart_x_min_);
    clamped = true;
  } else if (target.p.x() > cart_x_max_) {
    target.p.x(cart_x_max_);
    clamped = true;
  }

  if (target.p.y() < cart_y_min_) {
    target.p.y(cart_y_min_);
    clamped = true;
  } else if (target.p.y() > cart_y_max_) {
    target.p.y(cart_y_max_);
    clamped = true;
  }

  if (target.p.z() < cart_z_min_) {
    target.p.z(cart_z_min_);
    clamped = true;
  } else if (target.p.z() > cart_z_max_) {
    target.p.z(cart_z_max_);
    clamped = true;
  }

  if (clamped) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Cartesian target clamped to workspace: "
                         "[%.3f, %.3f, %.3f]",
                         target.p.x(), target.p.y(), target.p.z());
  }

  return clamped;
}

// ======================================================================
// ROS2 Callbacks
// ======================================================================

void IKSolverNode::current_positions_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() < num_joints_) {
    RCLCPP_WARN(this->get_logger(),
                "CurrentPositions message too short. "
                "Expected at least %u values, got %zu",
                num_joints_, msg->data.size());
    return;
  }

  double flag = msg->data[msg->data.size() - 1];

  // Only process position data (flag == 0.0)
  if (flag == 0.0) {
    for (unsigned int i = 0; i < num_joints_; ++i) {
      current_joint_positions_(i) = msg->data[i];
    }
    fk_solver_->JntToCart(current_joint_positions_, cartesian_target_);
  }
}

void IKSolverNode::cartesian_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (!target_initialized_) {
    return;
  }

  if (msg->data.size() < 4) {
    RCLCPP_WARN(this->get_logger(),
                "CartesianCmd message too short. "
                "Expected 4 values got %zu",
                msg->data.size());
    return;
  }

  double x_vel = msg->data[0];
  double y_vel = msg->data[1];
  double z_vel = msg->data[2];
  bool home_pressed = (msg->data[3] > 0.5);

  // Calculate dt since last update
  double now = this->now().seconds();
  double dt = now - last_update_time_;
  if (dt > 0.5) {
    dt = 0.02; // Cap large gaps
  }
  last_update_time_ = now;

  // Handle home button — full state reset
  if (home_pressed) {
    for (unsigned int i = 0; i < num_joints_; ++i) {
      current_joint_positions_(i) = 0.0;
    }
    fk_solver_->JntToCart(current_joint_positions_, cartesian_target_);
    publish_joint_command(current_joint_positions_);
    return;
  }

  // Save pre-velocity target for rollback on failure
  KDL::Vector pre_velocity_target = cartesian_target_.p;

  // Integrate velocity into Cartesian target
  cartesian_target_.p.x(cartesian_target_.p.x() + x_vel * velocity_scale_ * dt);
  cartesian_target_.p.y(cartesian_target_.p.y() + y_vel * velocity_scale_ * dt);
  cartesian_target_.p.z(cartesian_target_.p.z() + (-z_vel) * velocity_scale_ * dt);

  // Clamp Cartesian target to workspace before solving
  clamp_cartesian_target(cartesian_target_);

  // Solve position IK using current joints as initial guess
  KDL::JntArray solution(num_joints_);
  int result = ik_pos_solver_->CartToJnt(current_joint_positions_, cartesian_target_, solution);

  // Handle solve failure — restore pre-velocity target
  if (result < 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Position IK failed (result=%d). "
                         "Target may be out of reach: [%.3f, %.3f, %.3f]",
                         result, cartesian_target_.p.x(), cartesian_target_.p.y(), cartesian_target_.p.z());

    cartesian_target_.p = pre_velocity_target;
    return;
  }

  // Success — update joint positions and publish
  current_joint_positions_ = solution;
  publish_joint_command(solution);
}

void IKSolverNode::target_positions_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
  if (!target_initialized_) {
    return;
  }

  // Save the previous target so a failed solve does not corrupt the state
  // used by subsequent absolute or velocity commands.
  KDL::Vector previous_target = cartesian_target_.p;

  // TargetPositions contains absolute coordinates in the configured base
  // frame, unlike CartesianCmd which contains velocities to integrate.
  cartesian_target_.p.x(msg->x);
  cartesian_target_.p.y(msg->y);
  cartesian_target_.p.z(msg->z);

  clamp_cartesian_target(cartesian_target_);

  KDL::JntArray solution(num_joints_);
  int result = ik_pos_solver_->CartToJnt(current_joint_positions_, cartesian_target_, solution);

  if (result < 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Absolute position IK failed (result=%d). "
                         "Target may be out of reach: [%.3f, %.3f, %.3f]",
                         result, cartesian_target_.p.x(), cartesian_target_.p.y(), cartesian_target_.p.z());
    cartesian_target_.p = previous_target;
    return;
  }

  current_joint_positions_ = solution;
  publish_joint_command(solution);
}

// ======================================================================
// Publishing
// ======================================================================

void IKSolverNode::publish_joint_command(const KDL::JntArray &joint_angles) {
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = this->now();

  for (unsigned int i = 0; i < num_joints_; ++i) {
    msg.name.push_back(joint_names_[i]);
    msg.position.push_back(joint_angles(i));
  }
  joint_command_publisher_->publish(msg);
}

// ======================================================================
// Main Entry Point
// ======================================================================

/**
 * @brief Entry point for the IK solver node.
 *
 * Initializes ROS2, creates the IKSolverNode, and spins until shutdown.
 * The node operates entirely via callbacks — IK solving is triggered
 * by incoming Cartesian commands from the arm controller.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IKSolverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
