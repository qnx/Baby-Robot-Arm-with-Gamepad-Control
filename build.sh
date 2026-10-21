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

set -e
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG

ROS2_PKGS="arm_controller ik_solver joy_teleop_hiddi"
BUILD_HAND=false
BUILD_POSE=false
BUILD_UI=false

# Default to just building base BRA code if no arguments provided
if [ $# -eq 0 ]; then
    echo "No flags specified, building only ROS nodes."
fi


for arg in "$@"; do
    case "$arg" in
        --hand)
            BUILD_HAND=true
            BUILD_UI=true
            ;;
        --pose)
            BUILD_POSE=true
            BUILD_UI=true
            ;;
        --help)
            echo "Usage: $0 [--hand] [--pose]"
            echo "  --hand: Build the MediaPipe physical AI demo"
            echo "  --pose: Build the pose estimation demo (not yet implemented)"
            exit 0
            ;;
        *)
            echo "Unknown flag: $arg"
            echo "Usage: $0 [--hand] [--pose]"
            exit 1
            ;;
    esac
done

if $BUILD_HAND; then
    ROS2_PKGS="${ROS2_PKGS} mode_publisher landmark_publisher"
    if [ -d mediapipe ]; then
        echo "mediapipe already present, skipping clone"
    else
        echo "=== [--hand] Cloning mediapipe ==="
        git clone https://github.com/qnx-ports/mediapipe.git --depth 1 --branch qnx-v0.10.26 mediapipe

        ln -s ../../handlandmark_task mediapipe/mediapipe/handlandmark_task

        # Build script for hand landmarking
        ln -s ../handlandmark_task/build_handlandmarker.sh mediapipe/build_handlandmarker.sh
    fi
fi

if $BUILD_POSE; then
    ROS2_PKGS="${ROS2_PKGS} mode_publisher hailo-joint-publisher"
    echo "Not yet supported"
    # TODO build hailo-joint-controller
    # This is disabled as we aren't able to build until the hailort apk is created.
    exit 1
fi

# build UI
if $BUILD_UI; then
    pushd physical-ai-ui
    make
    popd
fi

# --- ROS2 Installation Discovery ---
if [ -f "/opt/ros/jazzy/local_setup.bash" ]; then
    ROS2_HOST_INSTALLATION_PATH=/opt/ros/jazzy
    echo "Found ROS2 Installation in $ROS2_HOST_INSTALLATION_PATH"
else
    echo "Failed to find ROS2 in expected locations, please run 'sudo apk add ros2-jazzy'"
    exit 1
fi

# Source the ROS2 environment for cross-compilation
. "${ROS2_HOST_INSTALLATION_PATH}/local_setup.bash"

echo "--- ROS Environment Variables ---"
printenv | grep "ROS"
echo "-------------------------------"

# --- Build Execution ---
colcon build --merge-install --cmake-force-configure --base-paths src --packages-select ${ROS2_PKGS}

rc=$?
if [ $rc -eq 0 ]; then
    echo "Success"
else
    echo "Error: $rc"
    exit $rc
fi

# build mediapipe
if $BUILD_HAND; then
    echo "=== [--hand] Building the hand landmark publisher ==="

    pushd mediapipe
    ./build_handlandmarker.sh
    popd

    HAND_BIN=mediapipe/bazel-bin/mediapipe/handlandmark_task/handlandmark_task
    if [ ! -x "$HAND_BIN" ]; then
        echo "Error: build finished but $HAND_BIN is missing." >&2
        exit 1
    fi
    echo "[--hand] Built $HAND_BIN"
fi

echo " "
