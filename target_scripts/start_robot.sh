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

if [ "$(id -u)" -ne "0" ]; then
    echo "Error: Must be run as root for HID input"
    exit 1
fi

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
CART_MIN_LIMITS="[-0.16,  0.11, -0.145]"
CART_MAX_LIMITS="[ 0.16,  0.31, -0.03]"

# Default mode [joystick, joint, ik_joystick, ik_joint]
ROBOT_MODE="joystick"
CAMERA_UNIT=1 # Default to camera unit 1 if not specified
RUN_HAND=false
RUN_POSE=false

while [ "$#" -gt 0 ]; do
    case "$1" in
        --mode)
            if [ "$#" -lt 2 ]; then
                echo "Error: --mode requires a value." >&2
                exit 1
            fi
            case "$2" in
                joystick|joint|ik_joystick|ik_joint) ;;
                *)
                    echo "Error: --mode must be joystick, joint, ik_joystick or ik_joint." >&2
                    exit 1
                    ;;
            esac
            ROBOT_MODE="$2"
            shift 2
            ;;
        --camera)
            if [ "$#" -lt 2 ] || [[ ! "$2" =~ ^[0-9]+$ ]]; then
                echo "Error: --camera requires a non-negative integer." >&2
                exit 1
            fi
            CAMERA_UNIT="$2"
            shift 2
            ;;
        --hand)
            RUN_HAND=true
            ROBOT_MODE="ik_joint"
            shift
            ;;
        --pose)
            RUN_POSE=true
            ROBOT_MODE="joint"
            shift
            ;;
        --help)
            echo "Usage: $0 [--mode <mode>] [--camera <unit>] [--hand] [--pose]"
            echo "Valid modes: joystick, joint, ik_joystick, ik_joint"
            echo "Valid camera units: 0, 1, 2, ... (default: 1)"
            echo "  --hand: Run the MediaPipe hand landmark publisher"
            echo "  --pose: Run the Hailo based pose detection demo (not yet implemented)"
            exit 0
            ;;
        *)
            echo "Unknown flag: $1" >&2
            echo "Usage: $0 [--mode <mode>] [--camera <unit>] [--help]"
            echo "Valid modes: joystick, joint, ik_joystick, ik_joint"
            exit 1
            ;;
    esac
done

# On exit we need to manually kill qpp and playback-connector
# unlike ros2 node they are not catching the ctrl+c of the parent
cleanup() {
    slay qpp playback-connector handlandmark_task physical-ai-ui
}

start_ros() {
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
    CART_MIN_LIMITS="[-0.16,  0.11, -0.145]"
    CART_MAX_LIMITS="[ 0.16,  0.31, -0.03]"

    # --- Starting the ROS2 Nodes ---
    echo "Starting Ros2"

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

    if ${RUN_HAND}; then
        ros2 run mode_publisher mode_publisher_node \
            --ros-args \
            -p qpp_path:=/dev/qpp/physicalai/bra/ &

        ros2 run landmark_publisher landmark_publisher \
            --ros-args \
            -p cart_min_limits:="${CART_MIN_LIMITS}" \
            -p cart_max_limits:="${CART_MAX_LIMITS}" &
    fi

    if ${RUN_POSE}; then
        ros2 run mode_publisher mode_publisher_node \
            --ros-args \
            -p qpp_path:=/dev/qpp/physicalai/bra/ &

        ros2 run hailo_joint_publisher hailo_joint_publisher &
    fi
}

trap cleanup EXIT

if ${RUN_HAND}; then
    echo "Starting QPP"
    if ! [ -d /dev/qpp ]; then
        qpp -c "${SCRIPT_DIR}/../etc/physical-ai_v1.json" &
        waitfor /dev/qpp

        playback-connector -s /dev/qpp &
    else
      echo "Warning: Another QPP instance running skipping..."
    fi

    # Start handlandmark task
    HAND_BIN="${SCRIPT_DIR}/../mediapipe/bazel-bin/mediapipe/handlandmark_task/handlandmark_task"
    # The model is a bazel data dep, so it lives in the binary's runfiles.
    HAND_MODEL="${HAND_BIN}.runfiles/mediapipe/mediapipe/handlandmark_task/gesture_recognizer.task"

    if [ -x "${HAND_BIN}" ]; then
        echo "Starting handlandmark_task"
        "${HAND_BIN}" --camera_unit="${CAMERA_UNIT}" --model_path="${HAND_MODEL}" &
    else
        echo "Running without Mediapipe. Build it with ./build.sh --hand"
    fi
fi

if ${RUN_POSE}; then
    #TODO: Add Hailo demo run command here once it is available
    echo "Pose detection is not yet implemented."
fi

start_ros &

# Start up the UI
if ${RUN_POSE}; then
    #TODO: Add Hailo demo run command here once it is available
    echo "Pose detection is not yet implemented."
    exit 1

    POSE_MODEL=${SCRIPT_DIR}/../vit_pose_small.hef
    "${SCRIPT_DIR}/../hailo-joint-controller/hailo-joint-controller" ${POSE_MODEL} --cam-id="${CAMERA_UNIT}" &
fi
"${SCRIPT_DIR}/../physical-ai-ui/physical-ai-ui" --camera-unit="${CAMERA_UNIT}" --show-video-flip &

# --- Wait for all background nodes to exit ---
echo "All nodes started. Press Ctrl+C in this terminal to stop both."
wait

echo "All nodes have been shut down. Script finished."