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
 * @brief Header for the IK Solver ROS2 node implementing position-based
 *        inverse kinematics for a 5-DOF robotic arm on QNX.
 *
 * This node receives Cartesian velocity commands from the arm controller
 * node, integrates them into a Cartesian position target, and uses the
 * KDL Levenberg-Marquardt (LMA) position IK solver to compute joint
 * angles that reach the target.
 *
 * 5-DOF Handling:
 *     The LMA solver is initialized with a weight matrix [1,1,1,0,0,0]
 *     that prioritizes position (X, Y, Z) over orientation (Roll, Pitch,
 *     Yaw), allowing the solver to freely choose orientation while
 *     accurately reaching the desired position.
 *
 * Cartesian Workspace Limits:
 *     A 3D bounding box defined in meters from the base frame origin
 *     constrains the Cartesian target BEFORE IK solving. This prevents
 *     the solver from receiving unreachable targets and eliminates the
 *     need for post-solve joint limit clamping.
 *
 * Servo Hardware:
 *     - Joints 0-2: DS3218 servos, 270 degree mode [DS3218 Datasheet]
 *     - Joints 3-4: MG90S micro servos, 180 degree mode [MG90S Datasheet]
 *     - Joint 5 (Gripper): Controlled directly by arm controller
 *
 * ROS2 Topics:
 *     Subscriptions:
 *         /CartesianCmd      - Cartesian velocity commands [X, Y, Z, home]
 *         /TargetPositions   - Absolute Cartesian target position [X, Y, Z]
 *         /CurrentPositions  - Joint positions for state synchronization
 *     Publications:
 *         /mov               - Computed joint angles in radians
 *
 * References:
 *     - Orocos KDL: https://www.orocos.org/wiki/orocos/kdl-wiki
 */

#ifndef IK_SOLVER_NODE_HPP
#define IK_SOLVER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <vector>
#include <string>
#include <memory>

/**
 * @class IKSolverNode
 * @brief ROS2 node that solves position-based inverse kinematics for
 *        a 5-DOF robotic arm using the KDL LMA solver.
 *
 * The node maintains a Cartesian target position that is updated by
 * velocity commands from the arm controller. The LMA solver computes
 * joint angles to reach the target, prioritizing position accuracy
 * over orientation for the under-determined 5-DOF system.
 */
class IKSolverNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructs the IKSolverNode.
     *
     * Initialization sequence:
     *     1. Declare and load ROS2 parameters
     *     2. Load Cartesian workspace limits
     *     3. Find URDF file from URDF_PATH environment variable
     *     4. Load URDF and construct KDL kinematic chain
     *     5. Initialize FK and IK solvers with position-priority weighting
     *     6. Compute initial Cartesian target from FK at home position
     *     7. Set up ROS2 subscriptions and publishers
     */
    IKSolverNode();

private:
    // ======================================================================
    // URDF Loading and Configuration
    // ======================================================================

    /**
     * @brief Searches for the URDF file in the URDF_PATH directory.
     *
     * @param filename Name of the URDF file to find (e.g. "arm5dof.urdf").
     * @return Full path to the URDF file, or empty string if not found.
     */
    std::string find_URDF(const std::string &filename);

    /**
     * @brief Loads the URDF file and constructs the KDL kinematic chain.
     *
     * Parses URDF XML into a KDL tree, extracts the kinematic chain
     * from base_link to end_effector_link, and collects movable joint names.
     *
     * @return true if URDF loaded and chain extracted successfully.
     */
    bool load_URDF();

    /**
     * @brief Extracts movable joint names from the KDL chain.
     *
     * Skips fixed joints (KDL::Joint::None) such as the world_to_base joint.
     */
    void extract_joint_names();

    // ======================================================================
    // Solver Initialization
    // ======================================================================

    /**
     * @brief Initializes FK and position IK solvers.
     *
     * Creates ChainFkSolverPos_recursive and ChainIkSolverPos_LMA with
     * position-priority weight matrix [1,1,1,0,0,0] for 5-DOF handling.
     * Computes initial Cartesian target from FK at home position.
     *
     * @return true if both solvers initialized successfully.
     */
    bool init_solvers();

    // ======================================================================
    // Cartesian Workspace Limits
    // ======================================================================

    /**
     * @brief Clamps the Cartesian target to the defined workspace box.
     *
     * Called BEFORE IK solving so the solver only receives targets
     * within the safe workspace. Replaces joint limit clamping.
     *
     * @param target The Cartesian target frame to clamp.
     * @return true if any axis was clamped.
     */
    bool clamp_cartesian_target(KDL::Frame &target);

    // ======================================================================
    // ROS2 Callbacks
    // ======================================================================

    /**
     * @brief Receives current joint positions from the arm controller
     *        for state synchronization.
     *
     * Processes position data (flag 0.0) by updating current_joint_positions_
     * and resyncing cartesian_target_ via FK.
     *
     * @param msg Float64MultiArray with joint data and flag.
     */
    void current_positions_callback(
        const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    /**
     * @brief Main IK solving callback triggered by Cartesian velocity commands.
     *
     * Integrates velocity into Cartesian target, clamps to workspace,
     * solves IK, and publishes joint angles on success. Rolls back
     * Cartesian target on solve failure.
     *
     * @param msg Float64MultiArray [X_vel, Y_vel, Z_vel, home_flag].
     */
    void cartesian_callback(
        const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    /**
     * @brief Solves IK for an absolute Cartesian end-effector position.
     *
     * The requested coordinates are expressed in metres in the configured
     * base frame. The target is clamped to the Cartesian workspace before
     * solving, and the resulting joint angles are published on /mov.
     *
     * @param msg Absolute target position [X, Y, Z].
     */
    void target_positions_callback(
        const geometry_msgs::msg::Point::SharedPtr msg);

    // ======================================================================
    // Publishing
    // ======================================================================

    /**
     * @brief Publishes computed joint angles to the /mov topic.
     *
     * @param joint_angles KDL JntArray with solved joint angles in radians.
     */
    void publish_joint_command(const KDL::JntArray &joint_angles);

    // ======================================================================
    // Member Variables
    // ======================================================================

    // --- Timing ---

    /// @brief Timestamp of last Cartesian command for dt calculation.
    double last_update_time_ = 0.0;

    // --- ROS2 Communication ---

    /// @brief Publisher for computed joint angles to /mov topic.
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        joint_command_publisher_ = nullptr;

    /// @brief Subscription for Cartesian velocity commands.
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        cartesian_subscription_ = nullptr;

    /// @brief Subscription for absolute Cartesian end-effector positions.
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr
        target_positions_subscription_ = nullptr;

    /// @brief Subscription for current positions from arm controller.
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        pos_subscription_ = nullptr;

    // --- URDF and KDL ---

    /// @brief Full path to the URDF file.
    std::string urdf_file_;

    /// @brief Root link name in kinematic chain.
    std::string base_link_;

    /// @brief End effector link name in kinematic chain.
    std::string end_effector_link_;

    /// @brief KDL kinematic chain from base to end effector.
    KDL::Chain kdl_chain_;

    /// @brief Names of movable joints from KDL chain.
    std::vector<std::string> joint_names_;

    /// @brief Number of movable joints (5 for this arm).
    unsigned int num_joints_;

    // --- Cartesian Workspace Limits ---

    /// @brief Minimum X coordinate in meters from base frame.
    double cart_x_min_ = -0.25;

    /// @brief Maximum X coordinate in meters from base frame.
    double cart_x_max_ = 0.25;

    /// @brief Minimum Y coordinate in meters from base frame.
    double cart_y_min_ = -0.25;

    /// @brief Maximum Y coordinate in meters from base frame.
    double cart_y_max_ = 0.25;

    /// @brief Minimum Z coordinate in meters from base frame.
    double cart_z_min_ = 0.05;

    /// @brief Maximum Z coordinate in meters from base frame.
    double cart_z_max_ = 0.35;

    // --- Solver State ---

    /// @brief Current joint angles used as initial guess for IK solver.
    KDL::JntArray current_joint_positions_;

    /// @brief Target Cartesian pose updated by velocity commands.
    KDL::Frame cartesian_target_;

    /// @brief Flag indicating solvers are initialized and target is valid.
    bool target_initialized_ = false;

    // --- Solvers ---

    /// @brief FK solver: joint angles to Cartesian position.
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

    /// @brief Position IK solver with 5-DOF position-priority weighting.
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_pos_solver_;

    // --- Configuration ---

    /// @brief Cartesian velocity scale factor (m/s per unit).
    double velocity_scale_ = 0.01;
};

#endif
