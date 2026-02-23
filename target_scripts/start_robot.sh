#!/bin/bash
 
# --- Set Environment Variables ---
# These paths are needed for ROS2 to find its libraries and Python packages.
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/data/home/qnxuser/opt/ros/humble/lib
export PYTHONPATH=$PYTHONPATH:/data/home/qnxuser/opt/ros/humble/usr/lib/python3.11/site-packages/:/data/home/qnxuser/.local/lib/python3.11/site-packages/
export COLCON_PYTHON_EXECUTABLE=/system/bin/python3
 
# --- Sourcing ROS2 ---
# Source the main ROS2 environment
if [ -f /data/home/qnxuser/opt/ros/humble/setup.bash ]; then
    . /data/home/qnxuser/opt/ros/humble/setup.bash
else
    echo "Error: ROS2 global setup file not found!"
    exit 1
fi
 
# Source your workspace's local setup file to find your custom nodes
if [ -f /data/home/qnxuser/opt/ros/nodes/local_setup.bash ]; then
    . /data/home/qnxuser/opt/ros/nodes/local_setup.bash
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

# --- Starting the ROS2 Nodes ---
echo "Starting Joy Teleop and Arm Controller nodes..."
 
# Run the C++ joystick node in the background
ros2 run joy_teleop_hiddi joy_teleop_node &
 
# Run the Python arm controller node in the background with the limits.
ros2 run arm_controller arm_controller_node.py \
    --ros-args \
    -p servo_min_limits:="${SERVO_MIN_LIMITS}" \
    -p servo_max_limits:="${SERVO_MAX_LIMITS}" &
 
# --- Wait for all background nodes to exit ---
echo "All nodes started. Press Ctrl+C in this terminal to stop both."
wait
 
echo "All nodes have been shut down. Script finished."
 