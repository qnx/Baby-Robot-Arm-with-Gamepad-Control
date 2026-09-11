#!/bin/bash
#
# Copyright (c) 2026, BlackBerry Limited. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# --- Set Environment Variables ---
# These paths are needed for ROS2 to find its libraries and Python packages.
SCRIPT_DIR=$(dirname "${BASH_SOURCE}")

export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/opt/ros/jazzy/lib"
export ROS2_NODES_INSTALL="${SCRIPT_DIR}/../install"
export URDF_PATH="${ROS2_NODES_INSTALL}/share/ik_solver/config"
export PYTHONPATH="$PYTHONPATH:/opt/ros/jazzy/usr/lib/python3.11/site-packages/:/data/home/qnxuser/.local/lib/python3.11/site-packages/"
export COLCON_PYTHON_EXECUTABLE=/system/bin/python3

# --- Sourcing ROS2 ---
# Source the main ROS2 environment
if [ -f /opt/ros/jazzy/setup.bash ]; then
    . /opt/ros/jazzy/setup.bash
else
    echo "Error: ROS2 global setup file not found!"
    exit 1
fi

# Source your workspace's local setup file to find your custom nodes
if [ -f ${ROS2_NODES_INSTALL}/local_setup.bash ]; then
    . ${ROS2_NODES_INSTALL}/local_setup.bash
else
    echo "Error: ROS2 node install not found! ROS2_NODES_INSTALL=${ROS2_NODES_INSTALL}"
    exit 1
fi

# =========================================================================
# ARM LIMITS MODIFIERS
# Array order:
# Default is 0.0 for MIN and 100.0 for MAX. Change these based on clearance.
# =========================================================================

# Base: 25 (restricted left), Shoulder/Elbow: 0 (full forward), Gripper: 15 (closed)
SERVO_MIN_LIMITS="[25.0, 0.0, 50.0, 0.0, 0.0, 15.0]"

# Base: 75 (restricted right), Shoulder/Elbow: 50 (stops at upright), Gripper: 65 (open)
SERVO_MAX_LIMITS="[75.0, 50.0, 100.0, 100.0, 100.0, 65.0]"

# Cartesian limits for the IK solver (X, Y, Z)
CART_MIN_LIMITS="[-0.16,  0.10, -0.145]"
CART_MAX_LIMITS="[ 0.16,  0.21, -0.014]"

# Default mode [joystick, joint, ik_joystick, ik_joint]
ROBOT_MODE="joint"

# --- Starting the ROS2 Nodes ---
echo "Starting Joy Teleop, IK Solver, and Arm Controller nodes..."

# Run the C++ joystick node in the background
ros2 run joy_teleop_hiddi joy_teleop_node &

# Run the IK solver node in the background with the Cartesian limits.
ros2 run ik_solver ik_solver_node \
    --ros-args \
    -p cart_min_limits:="${CART_MIN_LIMITS}" \
    -p cart_max_limits:="${CART_MAX_LIMITS}" &

# Run the Python arm controller node in the background with the limits.
ros2 run arm_controller arm_controller_node.py \
    --ros-args \
    -p mode:="${ROBOT_MODE}" \
    -p servo_min_limits:="${SERVO_MIN_LIMITS}" \
    -p servo_max_limits:="${SERVO_MAX_LIMITS}" &

# --- Wait for all background nodes to exit ---
echo "All nodes started. Press Ctrl+C in this terminal to stop both."
wait

echo "All nodes have been shut down. Script finished."