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

#include "ik_solver_node.hpp"
#include "safety_validation.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <functional>
#include <stdexcept>
#include <sys/stat.h>
#include <unordered_set>
#include <unistd.h>

namespace
{

rcl_interfaces::msg::ParameterDescriptor read_only_descriptor(const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor bounded_double_descriptor(
  const std::string & description, double minimum, double maximum)
{
  auto descriptor = read_only_descriptor(description);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = minimum;
  range.to_value = maximum;
  range.step = 0.0;
  descriptor.floating_point_range.push_back(range);
  return descriptor;
}

}  // namespace

IKSolverNode::IKSolverNode()
: Node("ik_solver_node")
{
  RCLCPP_INFO(get_logger(), "IK Solver Node starting.");

  if (!load_and_validate_parameters()) {
    throw std::runtime_error("IK solver parameter validation failed");
  }

  RCLCPP_WARN(
    get_logger(),
    "Residual safety limitation: this node does not perform self-collision, environment-collision, "
    "or swept-path checking. Cartesian bounds are operational limits only; retain actuator limits "
    "and an independent E-stop/fail-off mechanism.");
  RCLCPP_WARN(
    get_logger(),
    "Model structure and file confinement are validated, but physical axes/origins/geometry and "
    "model provenance still require an independently approved deployment manifest.");

  urdf_file_ = find_URDF(get_parameter("urdf_file").as_string());
  if (urdf_file_.empty() || !read_URDF_file(urdf_file_, urdf_xml_)) {
    throw std::runtime_error("approved URDF could not be opened safely");
  }
  if (!load_URDF()) {
    throw std::runtime_error("URDF/KDL model validation failed");
  }
  if (!init_solvers()) {
    throw std::runtime_error("IK/FK solver initialization failed");
  }

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  joint_command_publisher_ = create_publisher<sensor_msgs::msg::JointState>("mov", qos);
  cartesian_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    "CartesianCmd", qos,
    std::bind(&IKSolverNode::cartesian_callback, this, std::placeholders::_1));
  pos_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    "CurrentPositions", qos,
    std::bind(&IKSolverNode::current_positions_callback, this, std::placeholders::_1));

  last_solve_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    get_logger(),
    "IK model ready with %u bounded joints; awaiting CurrentPositions synchronization.",
    num_joints_);
}

bool IKSolverNode::load_and_validate_parameters()
{
  declare_parameter<std::string>(
    "urdf_file", "arm5dof.urdf",
    read_only_descriptor("Basename of the approved URDF in this package's config directory."));
  declare_parameter<std::string>(
    "base_link", "world", read_only_descriptor("KDL chain root link."));
  declare_parameter<std::string>(
    "end_effector_link", "Arm_03_1",
    read_only_descriptor("Terminal tool-center-point link; must be a leaf in the URDF."));
  declare_parameter<double>(
    "update_rate", 50.0,
    bounded_double_descriptor("Maximum IK solve and fixed integration rate in hertz.", 1.0, 200.0));
  declare_parameter<double>(
    "max_command_interval", 0.25,
    bounded_double_descriptor("Discard a command after a longer steady-clock gap.", 0.005, 0.5));
  declare_parameter<double>(
    "velocity_scale", 0.1,
    bounded_double_descriptor("Cartesian meters per second for a unit command.", 0.001, 0.25));
  declare_parameter<double>(
    "max_joint_step_rad", 0.1,
    bounded_double_descriptor("Maximum accepted per-solve joint change in radians.", 0.001, 0.5));
  declare_parameter<std::vector<double>>(
    "cart_min_limits", {-0.25, -0.25, 0.05},
    read_only_descriptor("Exactly three finite Cartesian minima: X, Y, Z in meters."));
  declare_parameter<std::vector<double>>(
    "cart_max_limits", {0.25, 0.25, 0.35},
    read_only_descriptor("Exactly three finite Cartesian maxima: X, Y, Z in meters."));

  const auto cart_min = get_parameter("cart_min_limits").as_double_array();
  const auto cart_max = get_parameter("cart_max_limits").as_double_array();
  if (!ik_solver::safety::finite_vector(cart_min, 3U) ||
    !ik_solver::safety::finite_vector(cart_max, 3U))
  {
    RCLCPP_ERROR(get_logger(), "Cartesian limits must contain exactly three finite values.");
    return false;
  }
  for (std::size_t i = 0; i < 3U; ++i) {
    if (!ik_solver::safety::valid_limit_pair(cart_min[i], cart_max[i])) {
      RCLCPP_ERROR(get_logger(), "Cartesian minimum must be less than maximum on axis %zu.", i);
      return false;
    }
  }

  update_rate_hz_ = get_parameter("update_rate").as_double();
  max_command_interval_sec_ = get_parameter("max_command_interval").as_double();
  velocity_scale_ = get_parameter("velocity_scale").as_double();
  max_joint_step_rad_ = get_parameter("max_joint_step_rad").as_double();
  base_link_ = get_parameter("base_link").as_string();
  end_effector_link_ = get_parameter("end_effector_link").as_string();

  if (base_link_.empty() || end_effector_link_.empty() ||
    !ik_solver::safety::finite(update_rate_hz_) ||
    !ik_solver::safety::finite(max_command_interval_sec_) ||
    !ik_solver::safety::finite(velocity_scale_) ||
    !ik_solver::safety::finite(max_joint_step_rad_) || update_rate_hz_ < 1.0 ||
    update_rate_hz_ > 200.0 || max_command_interval_sec_ < 0.005 ||
    max_command_interval_sec_ > 0.5 || velocity_scale_ < 0.001 ||
    velocity_scale_ > 0.25 || max_joint_step_rad_ < 0.001 ||
    max_joint_step_rad_ > 0.5)
  {
    RCLCPP_ERROR(get_logger(), "One or more scalar IK parameters are invalid.");
    return false;
  }

  nominal_period_sec_ = 1.0 / update_rate_hz_;
  if (max_command_interval_sec_ < nominal_period_sec_) {
    RCLCPP_ERROR(
      get_logger(), "max_command_interval (%.6f) must be at least one update period (%.6f).",
      max_command_interval_sec_, nominal_period_sec_);
    return false;
  }

  cart_x_min_ = cart_min[0];
  cart_y_min_ = cart_min[1];
  cart_z_min_ = cart_min[2];
  cart_x_max_ = cart_max[0];
  cart_y_max_ = cart_max[1];
  cart_z_max_ = cart_max[2];

  RCLCPP_INFO(
    get_logger(), "Validated Cartesian limits X[%.3f, %.3f] Y[%.3f, %.3f] Z[%.3f, %.3f].",
    cart_x_min_, cart_x_max_, cart_y_min_, cart_y_max_, cart_z_min_, cart_z_max_);
  return true;
}

std::string IKSolverNode::find_URDF(const std::string & filename)
{
  namespace fs = std::filesystem;

  const fs::path requested(filename);
  if (filename.empty() || requested.is_absolute() || requested != requested.filename() ||
    requested == "." || requested == "..")
  {
    RCLCPP_ERROR(get_logger(), "urdf_file must be a plain basename without path components.");
    return {};
  }

  std::error_code error;
  fs::path config_root;
  try {
    config_root = fs::canonical(
      fs::path(ament_index_cpp::get_package_share_directory("ik_solver")) / "config");
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "Unable to resolve installed IK config directory: %s", exception.what());
    return {};
  }

  const fs::path unresolved = config_root / requested;
  const auto unresolved_status = fs::symlink_status(unresolved, error);
  if (error || fs::is_symlink(unresolved_status)) {
    RCLCPP_ERROR(get_logger(), "URDF must exist and must not be a symbolic link.");
    return {};
  }

  const fs::path candidate = fs::canonical(unresolved, error);
  if (error || candidate.parent_path() != config_root || !fs::is_regular_file(candidate, error) || error) {
    RCLCPP_ERROR(get_logger(), "URDF is not a regular file confined to the installed config directory.");
    return {};
  }

  RCLCPP_INFO(get_logger(), "Using confined URDF: %s", candidate.c_str());
  return candidate.string();
}

bool IKSolverNode::read_URDF_file(const std::string & path, std::string & xml) const
{
  int flags = O_RDONLY;
#ifdef O_CLOEXEC
  flags |= O_CLOEXEC;
#endif
#ifdef O_NOFOLLOW
  flags |= O_NOFOLLOW;
#endif

  const int fd = open(path.c_str(), flags);
  if (fd < 0) {
    RCLCPP_ERROR(get_logger(), "Unable to open URDF safely: %s", std::strerror(errno));
    return false;
  }

  struct stat metadata {};
  if (fstat(fd, &metadata) != 0 || !S_ISREG(metadata.st_mode) || metadata.st_size <= 0 ||
    static_cast<std::uintmax_t>(metadata.st_size) > kMaxUrdfBytes ||
    (metadata.st_mode & (S_IWGRP | S_IWOTH)) != 0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "URDF must be a non-empty, non-group/world-writable regular file no larger than %zu bytes.",
      kMaxUrdfBytes);
    close(fd);
    return false;
  }

  xml.assign(static_cast<std::size_t>(metadata.st_size), '\0');
  std::size_t offset = 0U;
  while (offset < xml.size()) {
    const auto count = read(fd, xml.data() + offset, xml.size() - offset);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count <= 0) {
      RCLCPP_ERROR(get_logger(), "URDF read failed or changed while being read.");
      close(fd);
      xml.clear();
      return false;
    }
    offset += static_cast<std::size_t>(count);
  }

  close(fd);
  return true;
}

bool IKSolverNode::load_URDF()
{
  urdf::Model model;
  if (!model.initString(urdf_xml_)) {
    RCLCPP_ERROR(get_logger(), "Failed to parse URDF XML.");
    return false;
  }

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
    RCLCPP_ERROR(get_logger(), "Failed to construct KDL tree from URDF.");
    return false;
  }
  if (!kdl_tree.getChain(base_link_, end_effector_link_, kdl_chain_)) {
    RCLCPP_ERROR(
      get_logger(), "Failed to extract KDL chain from '%s' to '%s'.", base_link_.c_str(),
      end_effector_link_.c_str());
    return false;
  }

  extract_joint_names();
  return validate_model(model) && extract_joint_limits(model);
}

bool IKSolverNode::validate_model(const urdf::Model & model)
{
  bool valid = true;
  const auto base = model.getLink(base_link_);
  const auto tip = model.getLink(end_effector_link_);
  if (!base || !tip) {
    RCLCPP_ERROR(get_logger(), "Configured base or end-effector link is absent from the URDF.");
    return false;
  }
  if (!tip->child_joints.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "End-effector '%s' is not terminal; %zu child joint(s) remain outside the safety chain.",
      end_effector_link_.c_str(), tip->child_joints.size());
    valid = false;
  }
  if (kdl_chain_.getNrOfJoints() != expected_joint_count_ ||
    joint_names_.size() != expected_joint_count_)
  {
    RCLCPP_ERROR(
      get_logger(), "Expected %zu movable joints, but the configured chain contains %u.",
      expected_joint_count_, kdl_chain_.getNrOfJoints());
    valid = false;
  }
  if (joint_names_ != expected_joint_names_) {
    RCLCPP_ERROR(
      get_logger(),
      "IK chain joint names/order do not match expected_joint_names; refusing an ambiguous mapping.");
    valid = false;
  }

  std::unordered_set<std::string> unique_names;
  for (const auto & name : joint_names_) {
    if (name.empty() || !unique_names.insert(name).second) {
      RCLCPP_ERROR(get_logger(), "IK chain contains an empty or duplicate joint name.");
      valid = false;
    }
  }
  return valid;
}

void IKSolverNode::extract_joint_names()
{
  joint_names_.clear();
  for (unsigned int i = 0U; i < kdl_chain_.getNrOfSegments(); ++i) {
    const auto joint = kdl_chain_.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::None) {
      joint_names_.push_back(joint.getName());
    }
  }
}

bool IKSolverNode::extract_joint_limits(const urdf::Model & model)
{
  num_joints_ = kdl_chain_.getNrOfJoints();
  joint_lower_limits_ = KDL::JntArray(num_joints_);
  joint_upper_limits_ = KDL::JntArray(num_joints_);

  for (unsigned int i = 0U; i < num_joints_; ++i) {
    const auto joint = model.getJoint(joint_names_[i]);
    if (!joint || joint->type != urdf::Joint::REVOLUTE || !joint->limits ||
      !ik_solver::safety::valid_limit_pair(joint->limits->lower, joint->limits->upper))
    {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must be revolute with finite, ordered lower/upper URDF limits.",
        joint_names_[i].c_str());
      return false;
    }
    joint_lower_limits_(i) = joint->limits->lower;
    joint_upper_limits_(i) = joint->limits->upper;
  }
  return true;
}

bool IKSolverNode::init_solvers()
{
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

  Eigen::Matrix<double, 6, 1> position_weights;
  position_weights << 1, 1, 1, 0, 0, 0;
  ik_pos_solver_ =
    std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_, position_weights);

  current_joint_positions_ = KDL::JntArray(num_joints_);
  for (unsigned int i = 0U; i < num_joints_; ++i) {
    if (!ik_solver::safety::within_closed_range(
        0.0, joint_lower_limits_(i), joint_upper_limits_(i)))
    {
      RCLCPP_ERROR(get_logger(), "Joint '%s' excludes the defined zero/home position.",
        joint_names_[i].c_str());
      return false;
    }
    current_joint_positions_(i) = 0.0;
  }

  KDL::Frame initial_frame;
  if (fk_solver_->JntToCart(current_joint_positions_, initial_frame) < 0 ||
    !frame_is_finite(initial_frame))
  {
    RCLCPP_ERROR(get_logger(), "Forward kinematics failed for the validated home state.");
    return false;
  }
  if (!frame_within_cartesian_bounds(initial_frame))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Validated zero/home TCP [%.3f, %.3f, %.3f] is outside the configured Cartesian bounds.",
      initial_frame.p.x(), initial_frame.p.y(), initial_frame.p.z());
    return false;
  }
  cartesian_target_ = initial_frame;
  // A valid model is not evidence of the physical arm's current state. Do not
  // emit IK output until the controller supplies one validated synchronization.
  target_initialized_ = false;
  return true;
}

bool IKSolverNode::frame_is_finite(const KDL::Frame & frame) const
{
  if (!ik_solver::safety::finite(frame.p.x()) ||
    !ik_solver::safety::finite(frame.p.y()) ||
    !ik_solver::safety::finite(frame.p.z()))
  {
    return false;
  }
  for (unsigned int row = 0U; row < 3U; ++row) {
    for (unsigned int column = 0U; column < 3U; ++column) {
      if (!ik_solver::safety::finite(frame.M(row, column))) {
        return false;
      }
    }
  }
  return true;
}

bool IKSolverNode::clamp_cartesian_target(KDL::Frame & target)
{
  if (!frame_is_finite(target)) {
    return false;
  }

  const KDL::Vector before = target.p;
  target.p.x(std::clamp(target.p.x(), cart_x_min_, cart_x_max_));
  target.p.y(std::clamp(target.p.y(), cart_y_min_, cart_y_max_));
  target.p.z(std::clamp(target.p.z(), cart_z_min_, cart_z_max_));

  const bool clamped = target.p.x() != before.x() || target.p.y() != before.y() ||
    target.p.z() != before.z();
  if (clamped) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Cartesian target clamped to operational bounds: [%.3f, %.3f, %.3f].",
      target.p.x(), target.p.y(), target.p.z());
  }
  return true;
}

bool IKSolverNode::frame_within_cartesian_bounds(const KDL::Frame & frame) const
{
  if (!frame_is_finite(frame)) {
    return false;
  }
  return ik_solver::safety::cartesian_position_within_bounds(
    {frame.p.x(), frame.p.y(), frame.p.z()},
    {cart_x_min_, cart_y_min_, cart_z_min_},
    {cart_x_max_, cart_y_max_, cart_z_max_});
}

bool IKSolverNode::validate_joint_solution(
  const KDL::JntArray & solution, bool enforce_step, std::string & reason) const
{
  if (solution.rows() != num_joints_) {
    reason = "joint count mismatch";
    return false;
  }

  for (unsigned int i = 0U; i < num_joints_; ++i) {
    const double value = solution(i);
    if (!ik_solver::safety::within_closed_range(
        value, joint_lower_limits_(i), joint_upper_limits_(i)))
    {
      reason = "joint '" + joint_names_[i] + "' is non-finite or outside its URDF limits";
      return false;
    }
    if (enforce_step &&
      !ik_solver::safety::within_step(current_joint_positions_(i), value, max_joint_step_rad_))
    {
      reason = "joint '" + joint_names_[i] + "' exceeds max_joint_step_rad";
      return false;
    }
  }
  return true;
}

bool IKSolverNode::command_is_well_formed(
  const std_msgs::msg::Float64MultiArray & msg, bool & home_pressed,
  std::uint64_t & session_epoch) const
{
  if (!ik_solver::safety::finite_vector(msg.data, 5U) ||
    !ik_solver::safety::exact_session_epoch(msg.data[4], session_epoch))
  {
    return false;
  }
  if (std::abs(msg.data[0]) > kMaxCartesianCommand ||
    std::abs(msg.data[1]) > kMaxCartesianCommand ||
    std::abs(msg.data[2]) > kMaxCartesianCommand)
  {
    return false;
  }

  if (std::abs(msg.data[3]) <= kFlagTolerance) {
    home_pressed = false;
    return true;
  }
  if (std::abs(msg.data[3] - 1.0) <= kFlagTolerance) {
    home_pressed = true;
    return true;
  }
  return false;
}

void IKSolverNode::advance_home_position()
{
  KDL::JntArray next(current_joint_positions_);
  for (unsigned int i = 0U; i < num_joints_; ++i) {
    const double delta = std::clamp(
      -current_joint_positions_(i), -max_joint_step_rad_, max_joint_step_rad_);
    next(i) = current_joint_positions_(i) + delta;
  }

  std::string reason;
  if (!validate_joint_solution(next, true, reason)) {
    fail_active_control("bounded home solution failed validation: " + reason);
  }

  KDL::Frame home_frame;
  if (fk_solver_->JntToCart(next, home_frame) < 0 || !frame_is_finite(home_frame)) {
    fail_active_control("forward kinematics failed during bounded home movement");
  }
  // Joint-bounded interpolation can still make the TCP leave the configured
  // Cartesian box. Reject before committing state or publishing any command.
  if (!frame_within_cartesian_bounds(home_frame)) {
    fail_active_control("bounded home step would leave Cartesian workspace");
  }

  current_joint_positions_ = next;
  cartesian_target_ = home_frame;
  publish_joint_command(next);
}

[[noreturn]] void IKSolverNode::fail_active_control(const std::string & reason) const
{
  RCLCPP_FATAL(
    get_logger(),
    "Active IK safety fault; terminating for supervisor fault-stop: %s.", reason.c_str());
  throw std::runtime_error("active IK safety fault: " + reason);
}

void IKSolverNode::current_positions_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  using ik_solver::safety::SynchronizationStatus;
  const std::size_t payload_size = static_cast<std::size_t>(num_joints_);
  std::uint64_t session_epoch = 0U;
  const auto synchronization_status = ik_solver::safety::begin_synchronization_message(
    msg->data, payload_size, session_epoch_gate_, session_epoch);
  if (synchronization_status != SynchronizationStatus::ready) {
    if (synchronization_status == SynchronizationStatus::malformed) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejected CurrentPositions: expected exactly %zu joint values plus one epoch.",
        payload_size);
    } else if (synchronization_status == SynchronizationStatus::invalid_epoch) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejected CurrentPositions: final value must be an exact positive bounded session epoch.");
    } else if (synchronization_status == SynchronizationStatus::stale_epoch) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejected CurrentPositions: session epoch %llu is not newer than %llu.",
        static_cast<unsigned long long>(session_epoch),
        static_cast<unsigned long long>(session_epoch_gate_.highest_seen_epoch()));
    } else {
      // The helper has already burned this valid newer epoch. Leave IK
      // inhibited so a corrupted synchronization cannot preserve old output.
      target_initialized_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejected CurrentPositions: joint state contains a non-finite value.");
    }
    return;
  }

  // Receipt of a strictly newer authority epoch revokes the old solution
  // stream immediately. Invalid state for the new epoch leaves IK inhibited.
  target_initialized_ = false;

  KDL::JntArray candidate(num_joints_);
  for (unsigned int i = 0U; i < num_joints_; ++i) {
    candidate(i) = msg->data[i];
  }
  std::string reason;
  if (!validate_joint_solution(candidate, false, reason)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Rejected CurrentPositions: %s.", reason.c_str());
    return;
  }

  KDL::Frame synchronized_target;
  if (fk_solver_->JntToCart(candidate, synchronized_target) < 0 ||
    !frame_is_finite(synchronized_target))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Rejected CurrentPositions: forward kinematics failed.");
    return;
  }
  if (!frame_within_cartesian_bounds(synchronized_target))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Rejected CurrentPositions: synchronized TCP is outside operational bounds.");
    return;
  }

  current_joint_positions_ = candidate;
  cartesian_target_ = synchronized_target;
  last_solve_time_ = std::chrono::steady_clock::now();
  if (!session_epoch_gate_.commit_synchronization(session_epoch)) {
    fail_active_control("session epoch changed during synchronization commit");
  }
  target_initialized_ = true;
}

void IKSolverNode::cartesian_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  bool home_pressed = false;
  std::uint64_t session_epoch = 0U;
  if (!command_is_well_formed(*msg, home_pressed, session_epoch)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Rejected CartesianCmd: require [finite X,Y,Z in -1..1, home flag 0 or 1, "
      "exact positive bounded session epoch].");
    return;
  }

  if (!target_initialized_ || !session_epoch_gate_.has_active_epoch()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Ignoring CartesianCmd until one valid CurrentPositions synchronization is received.");
    return;
  }
  if (!session_epoch_gate_.command_matches(session_epoch)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Rejected CartesianCmd: session epoch %llu does not match active epoch %llu.",
      static_cast<unsigned long long>(session_epoch),
      static_cast<unsigned long long>(session_epoch_gate_.active_epoch()));
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const double elapsed = std::chrono::duration<double>(now - last_solve_time_).count();
  const auto timing = ik_solver::safety::classify_elapsed(
    elapsed, nominal_period_sec_, max_command_interval_sec_);
  if (timing == ik_solver::safety::FixedStepStatus::too_soon) {
    return;
  }
  if (timing == ik_solver::safety::FixedStepStatus::stale_or_invalid) {
    last_solve_time_ = now;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Discarded stale/invalid CartesianCmd interval (%.6f s); next fresh sample may proceed.",
      elapsed);
    return;
  }
  last_solve_time_ = now;

  if (home_pressed) {
    advance_home_position();
    return;
  }

  const KDL::Frame previous_target = cartesian_target_;
  cartesian_target_.p.x(
    cartesian_target_.p.x() + msg->data[0] * velocity_scale_ * nominal_period_sec_);
  cartesian_target_.p.y(
    cartesian_target_.p.y() + msg->data[1] * velocity_scale_ * nominal_period_sec_);
  cartesian_target_.p.z(
    cartesian_target_.p.z() - msg->data[2] * velocity_scale_ * nominal_period_sec_);

  if (!frame_is_finite(cartesian_target_) || !clamp_cartesian_target(cartesian_target_)) {
    cartesian_target_ = previous_target;
    fail_active_control("Cartesian target became non-finite after validated integration");
  }

  KDL::JntArray solution(num_joints_);
  const int result =
    ik_pos_solver_->CartToJnt(current_joint_positions_, cartesian_target_, solution);
  if (result < 0) {
    cartesian_target_ = previous_target;
    fail_active_control("position IK solver failed with result " + std::to_string(result));
  }

  std::string reason;
  if (!validate_joint_solution(solution, true, reason)) {
    cartesian_target_ = previous_target;
    fail_active_control("IK solution failed validation: " + reason);
  }

  current_joint_positions_ = solution;
  publish_joint_command(solution);
}

void IKSolverNode::publish_joint_command(const KDL::JntArray & joint_angles)
{
  if (!target_initialized_ || !session_epoch_gate_.has_active_epoch()) {
    fail_active_control("publish attempted without an active synchronized session epoch");
  }
  std::string reason;
  if (!validate_joint_solution(joint_angles, false, reason)) {
    fail_active_control("publish invariant failed: " + reason);
  }

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now();
  // The actuator checks this exact frame before mutating any target, so output
  // computed under a revoked session cannot cross the hardware boundary.
  msg.header.frame_id =
    "ik-session:" + std::to_string(session_epoch_gate_.active_epoch());
  msg.name.reserve(num_joints_);
  msg.position.reserve(num_joints_);
  for (unsigned int i = 0U; i < num_joints_; ++i) {
    msg.name.push_back(joint_names_[i]);
    msg.position.push_back(joint_angles(i));
  }
  joint_command_publisher_->publish(msg);
}

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IKSolverNode>();
    // Active-control safety faults deliberately escape callbacks so this catch
    // returns nonzero and the launcher can fault-stop the supervised unit.
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "IK solver terminated safely: %s\n", exception.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
