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

from arm_control import JointNum, ArmController
from enum import Enum
import math
from sensor_msgs.msg import Joy, JointState
import time
from rclpy.impl.rcutils_logger import RcutilsLogger
from std_msgs.msg import Float64MultiArray

class ArmControllerInput:

    GRIPPER_CLOSED_PERCENT = 15
    GRIPPER_OPEN_PERCENT = 65

    def __init__(self, controller: ArmController, logger: RcutilsLogger):
        self.controller = controller
        self._logger = logger

    def get_logger(self) -> RcutilsLogger:
        return self._logger

    def joy_callback(self, msg: Joy):
        '''
        Called with joystick messages callback.

        By default this will be ignored unless function is overridden by child input
        '''
        # Not implemented by default.
        return

    def joint_callback(self, msg: JointState):
        '''
        Called with joint callback messages to move the robot

        By default this will be ignored unless function is overridden by child input
        '''
        # Not implemented by default.
        return

    def mov_callback(self, msg: JointState):
        '''
        Called with joint callback messages to move the robot with Inverse kinematics

        By default this will be ignored unless function is overridden by child input
        '''
        # Not implemented by default.

    def update(self):
        '''
        @brief Called every update tick of servo movement.

        By default this will be ignored unless function is overridden by child input
        '''
        # Not implemented by default.

    def focus(self):
        '''
        @brief Called whenever the input mode is moved into focus. This can happen either on state
        change or if screensaver is turned on than off.

        By default this will be ignored unless function is overridden by child input
        '''


## --------------------------------------------------------------------------
## Gamepad specific inputs
## --------------------------------------------------------------------------
class GamepadButton(Enum):
    A = 0
    B = 1
    X = 2
    Y = 3
    L1 = 4
    R1 = 5
    L2 = 6
    R2 = 7
    SELECT = 8
    START = 9
    L3 = 10
    R3 = 11


class GamepadAxis(Enum):
    X = 0
    Y = 1
    RX = 2
    RY = 3
    DX = 4  # DPAD X
    DY = 5  # DPAD Y


class ArmControllerJoystickInput(ArmControllerInput):
    """
    Joystick input arm controller. Controls the robot based on /joy messages

    Controls:
        Left Stick X     -> Base        (servo 0)
        Left Stick Y     -> Shoulder    (servo 1)
        Right Stick X    -> Elbow       (servo 2)
        Right Stick Y    -> Hand tilt   (servo 4)
        DPAD X           -> Hand rotate (servo 3)
        Shoulder Buttons -> Gripper     (servo 5)
        Stick Buttons    -> Gripper     (servo 5) Note same as Shoulder Buttons

        Y                -> Center all servos
    """

    def joy_callback(self, msg: Joy):
        """
        This controls the robot using the Joint using the joystick where different buttons are mapped
        to the joint of the robot
        """

        # Home button ('Y') is a master override
        if msg.buttons[GamepadButton.Y.value] == 1:
            self.get_logger().info("Home button pressed. Setting target to safe center.")
            self.controller.center_all_servos()
            return

        # --- Normal Joystick Control Logic ---
        self.controller.move_joint(JointNum.BASE, -1 * msg.axes[GamepadAxis.X.value])
        self.controller.move_joint(JointNum.SHOULDER, msg.axes[GamepadAxis.Y.value])
        self.controller.move_joint(JointNum.ELBOW, msg.axes[GamepadAxis.RY.value])
        self.controller.move_joint(JointNum.WRIST, msg.axes[GamepadAxis.DX.value])
        self.controller.move_joint(JointNum.HAND, -1 * msg.axes[GamepadAxis.RX.value])

        # Gripper Logic: Shoulders (4/5) OR Stick Clicks (10/11)
        if msg.buttons[GamepadButton.L1.value] == 1 or msg.buttons[GamepadButton.L3.value] == 1:
            self.controller.set_joint(JointNum.GRIPPER, self.GRIPPER_CLOSED_PERCENT)
            self.get_logger().info("Closing gripper.")
        elif msg.buttons[GamepadButton.R1.value] == 1 or msg.buttons[GamepadButton.R3.value] == 1:
            self.controller.set_joint(JointNum.GRIPPER, self.GRIPPER_OPEN_PERCENT)
            self.get_logger().info("Opening gripper.")


class ScreenSaverDance(Enum):
    FIGURE_8 = 1
    WAVE = 2
    COBRA = 3
    CONDUCTOR = 4


class ArmControllerScreensaverInput(ArmControllerInput):

    SCREENSAVER_SPEED = 0.05
    SCREENSAVER_WIDTH = 15.0
    SCREENSAVER_HEIGHT = 10.0
    SCREENSAVER_PAUSE_SEC = 1.0

    def __init__(self, controller: ArmController, logger: RcutilsLogger):
        super().__init__(controller, logger)
        self.screensaver_time = 0.0
        self.screensaver_is_paused = False
        self.pause_start_time = 0.0

        ## --------------------------------------------------------------------------
        ## Drawing Pose & Screensaver Parameters
        ## --------------------------------------------------------------------------
        ELBOW_DOWNWARD_BEND = 85.0
        SHOULDER_FORWARD_REACH = 40.0
        WRIST_CORRECTION = -5.0
        self.SHOULDER_COMPENSATION = 0.5
        self.WRIST_COMPENSATION = 1.0
        self.DRAWING_POSE = [
            50.0,  # Servo 0: Base
            SHOULDER_FORWARD_REACH,  # Servo 1: Shoulder
            ELBOW_DOWNWARD_BEND,  # Servo 2: Elbow
            (100 - ELBOW_DOWNWARD_BEND) + WRIST_CORRECTION,  # Servo 3: Wrist Pitch (Auto-calculated)
            50.0,  # Servo 4: Wrist Roll (Keep centered)
            50.0,  # Servo 5: Gripper
        ]

        self.dance = ScreenSaverDance.FIGURE_8

        self.DANCE_REGISTRY = {
            ScreenSaverDance.FIGURE_8: self._dance_figure_eight,
            ScreenSaverDance.WAVE: self._dance_wave,
            ScreenSaverDance.COBRA: self._dance_cobra,
            ScreenSaverDance.CONDUCTOR: self._dance_conductor,
        }

    def update(self):
        """
        @brief Updates the arm based on the screensaver time

        Between screen saver logic there will be a self.SCREENSAVER_PAUSE_SEC delay
        """
        # --- Screensaver Motion Logic with Pause ---
        if self.screensaver_is_paused:
            if (time.time() - self.pause_start_time) >= self.SCREENSAVER_PAUSE_SEC:
                self.screensaver_is_paused = False
                self.screensaver_time = 0.0
            return

        self.screensaver_time += self.SCREENSAVER_SPEED
        if self.screensaver_time >= (2 * math.pi):
            self.screensaver_is_paused = True
            self.pause_start_time = time.time()
            return

        position = self.DANCE_REGISTRY[self.dance]()
        for i in range(len(position)):
            self.controller.set_joint(JointNum(i), position[i])

    def start(self, dance: ScreenSaverDance):
        """
        @brief Sets the ScreenSave back to the default state.
               This should be used before starting the screensaver.
        """
        self.screensaver_time = 0.0
        self.screensaver_is_paused = False
        self.dance = dance

    ## --------------------------------------------------------------------------
    ## Dance Registry
    ## Each dance is method(self) -> list[float]  (a full 6-element target_positions array).
    ##
    ## To add a new dance:
    ##   1. Define a method following the signature below.
    ##   2. Add entry to ScreenSaverDance
    ##   3. Append ScreenSaverDance: func mapping to DANCE_REGISTRY
    ##   4. Add key bind to arm_controller_node.py
    ## --------------------------------------------------------------------------

    def _dance_figure_eight(self):
        """Classic figure-eight: base sweeps left/right while elbow traces a
        vertical lemniscate (sin 2t gives two lobes per cycle)."""
        pose = self.DRAWING_POSE.copy()
        positions = list(pose)
        offset_x = self.SCREENSAVER_WIDTH * math.cos(self.screensaver_time)
        offset_z = self.SCREENSAVER_HEIGHT * math.sin(2 * self.screensaver_time)
        positions[0] = pose[0] + offset_x
        positions[1] = pose[1] - (offset_z * self.SHOULDER_COMPENSATION)
        positions[2] = pose[2] + offset_z
        positions[3] = pose[3] - (offset_z * self.WRIST_COMPENSATION)
        positions[4] = pose[4]
        positions[5] = pose[5]
        return positions

    def _dance_wave(self):
        """Graceful bowing arc — like waving hello. The shoulder dips forward and
        back while the elbow lags by 45 degrees for a whip-like quality. The wrist
        pitch opens as the arm bows to reach out, and the base adds a gentle
        pendulum sweep."""
        pose = self.DRAWING_POSE.copy()
        positions = list(pose)
        shoulder_swing = self.SCREENSAVER_HEIGHT * math.sin(self.screensaver_time)
        elbow_bend = self.SCREENSAVER_HEIGHT * 0.8 * math.sin(self.screensaver_time + math.pi / 4)
        wrist_open = self.SCREENSAVER_HEIGHT * 1.2 * math.sin(self.screensaver_time)
        base_swing = self.SCREENSAVER_WIDTH * 0.5 * math.sin(self.screensaver_time)
        positions[0] = pose[0] + base_swing
        positions[1] = pose[1] + shoulder_swing
        positions[2] = pose[2] - elbow_bend
        positions[3] = pose[3] + wrist_open
        positions[4] = pose[4]
        positions[5] = pose[5]
        return positions

    def _dance_cobra(self):
        """Hypnotic rise and sway — like a cobra rearing up. The arm rises and
        drops once per cycle while the base sways side-to-side at half frequency,
        so two rear-and-drop cycles complete for every one left-right sway. The
        wrist auto-levels throughout to keep the end effector flat."""
        pose = self.DRAWING_POSE.copy()
        positions = list(pose)
        rise = self.SCREENSAVER_HEIGHT * math.sin(self.screensaver_time)
        sway = self.SCREENSAVER_WIDTH * math.sin(self.screensaver_time / 2)
        positions[0] = pose[0] + sway
        positions[1] = pose[1] - rise * 1.2
        positions[2] = pose[2] - rise
        positions[3] = pose[3] + (rise * self.WRIST_COMPENSATION) + (rise * self.SHOULDER_COMPENSATION * 1.2)
        positions[4] = pose[4]
        positions[5] = pose[5]
        return positions

    def _dance_conductor(self):
        """Orchestra conductor's baton. The elbow beats at 2x frequency (downbeat
        on every half-cycle), the base sweeps laterally 90 degrees out of phase
        with the melodic arc, and the wrist roll tilts 45 degrees ahead of the
        beat for an expressive wrist-flick effect."""
        pose = self.DRAWING_POSE.copy()
        positions = list(pose)
        beat = self.SCREENSAVER_HEIGHT * math.sin(2 * self.screensaver_time)
        arc = self.SCREENSAVER_HEIGHT * 0.6 * math.sin(self.screensaver_time - math.pi / 2)
        sweep = self.SCREENSAVER_WIDTH * math.cos(self.screensaver_time)
        roll_tilt = self.SCREENSAVER_HEIGHT * 0.4 * math.sin(2 * self.screensaver_time + math.pi / 4)
        positions[0] = pose[0] + sweep
        positions[1] = pose[1] + arc
        positions[2] = pose[2] + beat
        positions[3] = pose[3] - (beat * self.WRIST_COMPENSATION)
        positions[4] = pose[4] + roll_tilt
        positions[5] = pose[5]
        return positions


class ArmControllerJointInput(ArmControllerInput):
    """
    @brief Controls the arm using joint_state messages
    where each joint is mapped to a servo on the robot arm.
    """

    joint_map = {
        "base": JointNum.BASE,
        "shoulder": JointNum.SHOULDER,
        "elbow": JointNum.ELBOW,
        "wrist": JointNum.WRIST,
        "hand": JointNum.HAND,
        "gripper": JointNum.GRIPPER,
    }

    def joint_callback(self, msg: JointState):
        joint_count = len(msg.name)
        for joint_idx in range(joint_count):
            if msg.name[joint_idx] not in self.joint_map:
                self.get_logger().warn(f"Invalid joint {msg.name[joint_idx]} in joint message")
                continue

            self.controller.set_joint_rad(self.joint_map[msg.name[joint_idx]], msg.position[joint_idx])


class ArmControllerIKJoystickInput(ArmControllerInput):
    """
    @brief Controls robot with joystick using IK controls rather than per joint controls.
    Each joint is mapped to a servo on the robot arm with the exception of the gripper which
    is still controlled through the gamepad input directly.
    """

    joint_map = {
        "base": JointNum.BASE,
        "shoulder": JointNum.SHOULDER,
        "elbow": JointNum.ELBOW,
        "wrist": JointNum.WRIST,
        "hand": JointNum.HAND,
        "gripper": JointNum.GRIPPER,
    }

    def __init__(self, controller: ArmController, logger: RcutilsLogger, cartesian_pub, curr_pos_pub):
        super().__init__(controller, logger)
        self.cartesian_pub = cartesian_pub
        self.curr_pos_pub = curr_pos_pub

    def focus(self):
        """
        Publishes the current position of the arm position on state change back to IK mode
        """
        cmd = Float64MultiArray()
        cmd.data = [
            self.controller.get_joint_rad(JointNum.BASE),
            self.controller.get_joint_rad(JointNum.SHOULDER),
            self.controller.get_joint_rad(JointNum.ELBOW),
            self.controller.get_joint_rad(JointNum.WRIST),
            self.controller.get_joint_rad(JointNum.HAND),
            0.0
        ]
        self.curr_pos_pub.publish(cmd)

    def center_ortn_servos(self):
        """
        @brief centers orientation servos, respecting the safe limits.
        """
        for joint in [JointNum.WRIST, JointNum.HAND, JointNum.GRIPPER]:
            self.controller.center_joint(joint)

    def joy_callback(self, msg: Joy):
        # --- Joystick Control Logic ---
        cartesian_input = (
            msg.axes[GamepadAxis.X.value] != 0.0
            or msg.axes[GamepadAxis.Y.value] != 0.0
            or msg.axes[GamepadAxis.RY.value] != 0.0
            or msg.buttons[GamepadButton.Y.value] == 1
        )

        # A gripper-only button press must not trigger another IK solve.
        if cartesian_input:
            cmd = Float64MultiArray()
            cmd.data = [
                -msg.axes[GamepadAxis.X.value],
                msg.axes[GamepadAxis.Y.value],
                msg.axes[GamepadAxis.RY.value],
                float(msg.buttons[GamepadButton.Y.value]) # data[3]: home button
            ]
            self.cartesian_pub.publish(cmd)

        if msg.buttons[GamepadButton.Y.value] == 1:
            self.get_logger().info("Home button pressed. Setting target to safe center.")
            self.center_ortn_servos()
            return

        self.controller.move_joint(JointNum.WRIST, msg.axes[GamepadAxis.DX.value])
        self.controller.move_joint(JointNum.HAND, -1 * msg.axes[GamepadAxis.RX.value])

        # Gripper Logic: Shoulders OR Stick Clicks
        if msg.buttons[GamepadButton.L1.value] == 1 or msg.buttons[GamepadButton.L3.value] == 1:
            self.get_logger().info("Closing gripper.")
            self.controller.set_joint(JointNum.GRIPPER, self.GRIPPER_CLOSED_PERCENT)
        elif msg.buttons[GamepadButton.R1.value] == 1 or msg.buttons[GamepadButton.R3.value] == 1:
            self.get_logger().info("Opening gripper.")
            self.controller.set_joint(JointNum.GRIPPER, self.GRIPPER_OPEN_PERCENT)

    def mov_callback(self, msg: JointState):
        """
        @param msg: Float64MultiArray containing joint angles in radians.
            data[0]: Joint 0 angle (Base)
            data[1]: Joint 1 angle (Shoulder)
            data[2]: Joint 2 angle (Elbow)
            data[3]: Joint 3 angle (Wrist Pitch)
            data[4]: Joint 4 angle (Wrist Roll)
        """
        if len(msg.position) < 3:
            return

        self.controller.set_joint_rad(JointNum.BASE, msg.position[0])
        self.controller.set_joint_rad(JointNum.SHOULDER, msg.position[1])
        self.controller.set_joint_rad(JointNum.ELBOW, msg.position[2])

class ArmControllerIKJointInput(ArmControllerIKJoystickInput):
    """
    @brief Controls robot with joystick using IK controls rather than per joint controls.
    This is the same as @ArmControllerInverseKinematicInput but uses external controls
    which talk directly with the IK mode for the first 3 joints instead of the joystick.
    The next 3 joint are controlled through joint messages.
    """

    # Only the last 3 can be controlled with joint_map
    joint_map = {
        "wrist": JointNum.WRIST,
        "hand": JointNum.HAND,
        "gripper": JointNum.GRIPPER,
    }

    def __init__(self, controller: ArmController, logger: RcutilsLogger, cartesian_pub, curr_pos_pub):
        super().__init__(controller, logger, cartesian_pub, curr_pos_pub)

    def joint_callback(self, msg: JointState):
        """
        Controls the top 3 joints based on joint callbacks from an External
        """
        joint_count = len(msg.name)
        for joint_idx in range(joint_count):
            if msg.name[joint_idx] not in self.joint_map:
                self.get_logger().warn(f"Invalid joint {msg.name[joint_idx]} in joint message")
                continue
            self.controller.set_joint_rad(self.joint_map[msg.name[joint_idx]], msg.position[joint_idx])


    def joy_callback(self, msg: Joy):
        """
        The joy callback for robot isn't used for ExternalIKMode.
        We do haver want to support centering in cases that the robot gets in a odd position
        """

        if msg.buttons[GamepadButton.Y.value] == 1:
            self.get_logger().info("Home button pressed. Setting target to safe center.")
            cmd = Float64MultiArray()

            # Request that the robot center itself
            cmd.data = [0, 0, 0, 1]
            self.cartesian_pub.publish(cmd)
            self.center_ortn_servos()
            return
