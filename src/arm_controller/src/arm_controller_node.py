#!/system/bin/env python3
"""
Copyright (c) 2025, BlackBerry Limited. All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

@file arm_controller_node.py
@brief ROS2 node for controlling a 5-DOF robotic arm with a joystick.

This node manages all hardware-level servo control for a 5-DOF robotic arm
(plus gripper) using joystick input and/or inverse kinematics (IK) commands.
It subscribes to joystick messages(/joy) and IK solver output(/mov), converts commands
to PWM signals, and sends them to the servos via a PCA9685 PWM driver over I2C.

Key Features:
- Multi Control Modes: Supports 3 control modes
    - Direct Joystick Mode (enabled with Back + A)
        - Each Joint is controlled independently using a gamepad
    - IK Joystick Mode (enabled with Back + B)
        - Joints will move together making use of IK-based Cartesian control for more a
          intuitive control scheme
    - Direct Joint Mode: (enabled with Back + X)
        - Uses an external program such as AI to control servos directly
- Inverse Kinematics Integration: Subscribes to /mov topic for joint angles
  from the IK solver node, converts radians to servo percentages using
  asymmetric joint limit mapping where 0 rad always equals 50% (neutral).
- Cartesian Command Publishing: In IK mode, publishes joystick axes as
  Cartesian velocity commands to /CartesianCmd for the IK solver.
- Position Feedback: Publishes current servo positions as radians to
  /CurrentPositions for IK solver state synchronization.
- Smoothing Factor: Implements an interpolation (easing) function to ensure
  all arm movements are smooth/controlled.
- Individual Servo Speed Tuning: Allows each of the 6 servos to have a unique
  speed setting.
- Home Position: A dedicated joystick button returns the arm to a neutral,
  centered position.
- Deadzone and MIN/MAX: Includes configurable deadzone and gripper open/close
  percentages to prevent servo drift and stalling.
- Inactivity timer to release servos, and state management for
  Homing and Screensaver modes
"""

from arm_control import ArmController
from arm_controller_input import (
    ArmControllerIKJointInput,
    GamepadButton,
    ArmControllerJoystickInput,
    ArmControllerScreensaverInput,
    ArmControllerJointInput,
    ArmControllerIKJoystickInput,
    ScreenSaverDance,
)
from enum import Enum
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String
import time
import math
import signal


def signal_handler(sig, frame):
    print("You pressed Ctrl+C!")
    rclpy.try_shutdown()


## --------------------------------------------------------------------------
## Tunable Parameters
## --------------------------------------------------------------------------
DEADZONE = 0.08


class ArmControlMode(Enum):
    """
    What the robot arm is being controlled.
    """

    JOYSTICK = 0   # Controlled using the gamepad controller published on /joy
    JOINT = 1      # Controlled directly using joint angles published on /joint
    IK_JOYSTICK = 2 # Controlled by Joystick using Inverse kinematic
    IK_JOINT = 3    # Controlled by external input using IK and directory joints on /joint and /mov
    MAX = IK_JOINT


class ArmControllerNode(Node):
    """
    Manages arm hardware, joystick input, and controls all arm states.
    """

    SERVO_RELEASE_TIMEOUT_SEC = 5.0  # Seconds of inactivity before servos relax

    def __init__(self):
        """
        @brief Initializes the ArmControllerNode.
        """
        super().__init__("arm_controller_node")
        self.get_logger().info("Arm Controller Node has started (Merged Version).")

        ## --------------------------------------------------------------------------
        ## Servo Limits (ROS 2 Parameters)
        ## --------------------------------------------------------------------------
        # Declare parameters with default 0.0 to 100.0 boundaries
        self.declare_parameter("servo_min_limits", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("servo_max_limits", [100.0, 100.0, 100.0, 100.0, 100.0, 100.0])
        self.declare_parameter("mode", "joystick")

        self.controller = None
        try:
            self.controller = ArmController(
                self.get_logger(),
                self.get_parameter("servo_min_limits").value,
                self.get_parameter("servo_max_limits").value,
            )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ArmController: {e}")
            rclpy.try_shutdown()
            return

        ## @brief Publisher for Cartesian velocity commands sent to
        ## the IK solver when in IK mode.
        ## Message format: [X_vel, Y_vel, Z_vel, home_flag]
        self.cartesian_pub = self.create_publisher(
            Float64MultiArray,
            '/CartesianCmd',
            10
        )

        ## @brief Publisher for current joint positions in radians,
        ## used by the IK solver for state synchronization.
        self.curr_pos_pub = self.create_publisher(
            Float64MultiArray,
            '/CurrentPositions',
            10
        )

        ## @brief Subscription to IK solver output containing joint
        ## angles in radians for joints 0-4.
        self.mov_subscription = self.create_subscription(JointState, '/mov', self.mov_callback, 10)
        self.joy_subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.joint_subscription = self.create_subscription(JointState, "/joint", self.joint_callback, 10)
        self.mode_publisher = self.create_publisher(String, "/mode", 10)


        self.input_joystick = ArmControllerJoystickInput(self.controller, self.get_logger())
        self.input_screensaver = ArmControllerScreensaverInput(self.controller, self.get_logger())
        self.input_joint = ArmControllerJointInput(self.controller, self.get_logger())
        self.input_ik_joy = ArmControllerIKJoystickInput(self.controller, self.get_logger(), self.cartesian_pub, self.curr_pos_pub)
        self.input_ik_joint = ArmControllerIKJointInput(self.controller, self.get_logger(), self.cartesian_pub, self.curr_pos_pub)
        self.active_input = self.input_joystick # By default the joystick input will be used

        # The default mode is the joystick controls. Force the update to publish the state at least one at start
        self.control_mode = ArmControlMode.JOYSTICK
        default_mode = self.get_parameter("mode").value.lower()
        match default_mode:
            case "joystick":
                self.control_mode = ArmControlMode.JOYSTICK
            case "joint":
                self.control_mode = ArmControlMode.JOINT
            case "ik_joint":
                self.control_mode = ArmControlMode.IK_JOINT
            case "ik_joystick":
                self.control_mode = ArmControlMode.IK_JOYSTICK
            case _:
                self.get_logger().error(f"Invalid mode argument {default_mode}")
                rclpy.try_shutdown()
                return

        self.screensaver_enabled = False
        self._update_mode(self.control_mode, force=True)

        self.get_logger().info("Centering arm on startup...")
        self.controller.center_all_servos()
        self.get_logger().info("Arm centered. Waiting for commands...")

        # The Main Control Loop for smoothing and PWM, runs at 50Hz (0.02s)
        self.smoothing_timer = self.create_timer(0.02, self.smoothing_loop)

    def _update_mode(self, mode: ArmControlMode, force: bool = False):
        """
        @brief Updates the control mode of the robot arm and publishes it to /mode

        @note we will disable the screen saver as we do not consider
              start and select modifier as movement for disabling the screen saver

        @param mode The new mode for the arm control
        @param force If the mode is the same publish anyways.
        """
        if self.control_mode == mode and not force:
            return
        self.screensaver_enabled = False
        self.control_mode = mode

        # By default we should have position limits on the robot arm where we only remove them
        # in cases that the mode will provide it's own limits in a solver
        self.controller.enable_position_limits(True)

        # Update the control mode to the right handler
        match (self.control_mode):
            case ArmControlMode.JOYSTICK:
                self.active_input = self.input_joystick
            case ArmControlMode.JOINT:
                self.active_input = self.input_joint
            case ArmControlMode.IK_JOYSTICK:
                self.active_input = self.input_ik_joy
                self.controller.enable_position_limits(False)
            case ArmControlMode.IK_JOINT:
                self.active_input = self.input_ik_joint
                self.controller.enable_position_limits(False)

        msg = String()
        msg.data = mode.name.lower()
        self.mode_publisher.publish(msg)
        self.active_input.focus()

    def _enable_screensaver(self, dance: ScreenSaverDance):
        """
        @brief Enables the screen saver and publishes the state
        """
        self.screensaver_enabled = True
        self.input_screensaver.start(dance)
        msg = String()
        msg.data = f"screensaver:{dance.name.lower()}"
        self.mode_publisher.publish(msg)

    def _disable_screensaver(self):
        """
        @brief Disables the screensaver and republishes the state which was previously selected
        """
        self._update_mode(self.control_mode, force=True)

    ## --------------------------------------------------------------------------
    ## Input Callback
    ## --------------------------------------------------------------------------
    def joy_callback(self, msg: Joy):
        """
        This callback runs ONLY when a message from the Gamepad is received.

        This callback will take care of all global controls as well as filter out
        joystick input with the dead zone

        Controls:

        Select Modifier: (Change input mode)
            A -> Joystick (default)
            B -> Inverse Kinematic
            X -> Direct Joint

        Start Modifier: (Select Screen saver dance)
            Currently only supports 1 so no mod currently needed
        """

        # If any of the modifier are pressed consider it a global input so don't pass it down to input controllers.
        if msg.buttons[GamepadButton.SELECT.value]:
            prev_mode = self.control_mode
            if msg.buttons[GamepadButton.A.value]:
                self._update_mode(ArmControlMode.JOYSTICK)
            elif msg.buttons[GamepadButton.B.value]:
                self._update_mode(ArmControlMode.IK_JOYSTICK)
            elif msg.buttons[GamepadButton.X.value]:
                self._update_mode(ArmControlMode.JOINT)
            elif msg.buttons[GamepadButton.Y.value]:
                self._update_mode(ArmControlMode.IK_JOINT)
            if prev_mode != self.control_mode:
                self.get_logger().info(f"Info switching to mode: {self.control_mode.name}")
            return

        if msg.buttons[GamepadButton.START.value]:
            # only one dance for now but will add more
            prev_mode = self.input_screensaver.dance
            was_dance_active = self.screensaver_enabled
            if msg.buttons[GamepadButton.A.value]:
                self._enable_screensaver(ScreenSaverDance.FIGURE_8)
            elif msg.buttons[GamepadButton.B.value]:
                self._enable_screensaver(ScreenSaverDance.WAVE)
            elif msg.buttons[GamepadButton.X.value]:
                self._enable_screensaver(ScreenSaverDance.COBRA)
            elif msg.buttons[GamepadButton.Y.value]:
                self._enable_screensaver(ScreenSaverDance.CONDUCTOR)

            # Only log when state changes (new dance selected or we enabled the screen saver)
            if prev_mode != self.input_screensaver.dance or (self.screensaver_enabled and not was_dance_active):
                self.get_logger().info(f"Enable Screensaver: {self.input_screensaver.dance.name}")
            return

        # Check for gamepad input to reset inactivity timer and clamp to [-1,1]
        axes = [min(max(a, -1), 1) if math.fabs(a) > DEADZONE else 0.0 for a in msg.axes]
        msg.axes = axes

        has_input = any(axes) or any(msg.buttons)
        if self.screensaver_enabled and has_input:
            self.get_logger().info(f"Disabled Screensaver: by movement")
            self._disable_screensaver()

        # No input no point going through the callback
        if not has_input or self.screensaver_enabled:
            return

        self.active_input.joy_callback(msg)

    def joint_callback(self, msg: JointState):
        """
        Handles Joint information callbacks. This is only enabled if the mode
        is set to ArmControlMode.JOINT
        """
        self.get_logger().debug("Joint callback")

        if self.screensaver_enabled:
            return

        self.active_input.joint_callback(msg)

    def mov_callback(self, msg: JointState):
        """
        Handles Joint information callbacks for IK. This is only enabled if the mode
        is set to ArmControlMode.IK
        """
        self.get_logger().debug("Mov callback")

        if self.screensaver_enabled:
            return

        self.active_input.mov_callback(msg)

    def smoothing_loop(self):
        """
        This is the main hardware loop. It runs at 50Hz and smoothly
        moves the servos to the self.target_positions, regardless of
        what controller set them.
        """

        # In screensaver mode we will update the movement
        if self.screensaver_enabled:
            self.input_screensaver.update()
        elif not self.controller.servos_are_released:  # Check for inactivity timeout (only if screensaver is off)
            has_timed_out = (time.time() - self.controller.last_input_time) > self.SERVO_RELEASE_TIMEOUT_SEC
            if has_timed_out:
                self.get_logger().info("Inactivity detected. Releasing servos.")
                self.controller.release_all_servos()
                return  # No point updating the controller if we know it is released
            self.active_input.update()
        self.controller.update()

    def shutdown_controller(self):
        # Center the arm before exiting.
        if self.controller is not None:
            self.smoothing_timer.cancel()
            self.controller.center_all_servos()
            while not self.controller.is_servos_centered():
                self.controller.update()
                time.sleep(0.02)
            self.controller.release_all_servos()
            self.controller.pwm.software_reset()
            time.sleep(1)


## --------------------------------------------------------------------------
## Main Function
## --------------------------------------------------------------------------
def main(args=None):

    # Capture kill and slay commands so we can shutdown correctly
    signal.signal(signal.SIGTERM, signal_handler)

    rclpy.init(args=args)
    arm_controller_node = ArmControllerNode()
    if arm_controller_node.controller is None:
        arm_controller_node.get_logger().info("Failed to create Arm Controller check I2C wiring")
        exit(1)

    try:
        rclpy.spin(arm_controller_node)
    except Exception as e:
        print(e)
    finally:
        # shut down the node.
        arm_controller_node.shutdown_controller()


if __name__ == "__main__":
    main()
