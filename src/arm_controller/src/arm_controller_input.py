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

from enum import Enum
import math
import time

from sensor_msgs.msg import Joy, JointState
from rclpy.impl.rcutils_logger import RcutilsLogger
from std_msgs.msg import Float64MultiArray

from arm_control import NUM_SERVOS, JointNum, ArmController
from safety import finite_float, validate_ik_session_epoch, validate_joint_state


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
        Right Stick Y    -> Elbow       (servo 2)
        Right Stick X    -> Hand tilt   (servo 4)
        DPAD X           -> Hand rotate (servo 3)
        Shoulder Buttons -> Gripper     (servo 5)
        Right Stick      -> Open gripper (servo 5); L3 is reserved for dead-man

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
            self.controller.center_all_servos(record_activity=False)
            return

        # --- Normal Joystick Control Logic ---
        self.controller.move_joint(JointNum.BASE, -1 * msg.axes[GamepadAxis.X.value], record_activity=False)
        self.controller.move_joint(JointNum.SHOULDER, msg.axes[GamepadAxis.Y.value], record_activity=False)
        self.controller.move_joint(JointNum.ELBOW, msg.axes[GamepadAxis.RY.value], record_activity=False)
        self.controller.move_joint(JointNum.WRIST, msg.axes[GamepadAxis.DX.value], record_activity=False)
        self.controller.move_joint(JointNum.HAND, -1 * msg.axes[GamepadAxis.RX.value], record_activity=False)

        # L3 is the dedicated operator-enable input and must never command a joint.
        if msg.buttons[GamepadButton.L1.value] == 1:
            self.controller.set_joint(JointNum.GRIPPER, self.GRIPPER_CLOSED_PERCENT, record_activity=False)
            self.get_logger().info("Closing gripper.")
        elif msg.buttons[GamepadButton.R1.value] == 1 or msg.buttons[GamepadButton.R3.value] == 1:
            self.controller.set_joint(JointNum.GRIPPER, self.GRIPPER_OPEN_PERCENT, record_activity=False)
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
            50.0,  # Servo 3: Wrist Roll (Keep centered)
            (100 - ELBOW_DOWNWARD_BEND) + WRIST_CORRECTION,  # Servo 4: Wrist Pitch (Auto-calculated)
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
            if (time.monotonic() - self.pause_start_time) >= self.SCREENSAVER_PAUSE_SEC:
                self.screensaver_is_paused = False
                self.screensaver_time = 0.0
            return

        self.screensaver_time += self.SCREENSAVER_SPEED
        if self.screensaver_time >= (2 * math.pi):
            self.screensaver_is_paused = True
            self.pause_start_time = time.monotonic()
            return

        position = self.DANCE_REGISTRY[self.dance]()
        # Site calibration may be narrower than a demonstration curve. Clip
        # internal motion at that immutable envelope; external commands remain
        # reject-only so malformed input cannot hide behind clamping.
        commands = tuple(
            (
                JointNum(index),
                self.controller.constrain_internal_target_percent(JointNum(index), value),
            )
            for index, value in enumerate(position)
        )
        # Apply the complete bounded pose atomically and never refresh the
        # operator dead-man lease from internally generated motion.
        self.controller.set_joint_targets_percent_atomic(
            commands,
            record_activity=False,
        )

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
        positions[3] = pose[3]
        positions[4] = pose[4] - (offset_z * self.WRIST_COMPENSATION)
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
        positions[3] = pose[3]
        positions[4] = pose[4] + wrist_open
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
        positions[1] = pose[1] - rise
        positions[2] = pose[2] - rise
        positions[3] = pose[3]
        positions[4] = pose[4] + (rise * self.WRIST_COMPENSATION) + (rise * self.SHOULDER_COMPENSATION * 1.2)
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
        positions[3] = pose[3] + roll_tilt
        positions[4] = pose[4] - (beat * self.WRIST_COMPENSATION)
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
        validated = validate_joint_state(msg.name, msg.position, self.joint_map, NUM_SERVOS)
        # A DDS publisher is not an operator-enable source. Only a validated Joy
        # sample with held L3 may renew the actuator lease at the node boundary.
        self.controller.set_joint_targets_rad_atomic(
            ((self.joint_map[name], radians) for name, radians in validated),
            record_activity=False,
        )


class ArmControllerInverseKinematicInput(ArmControllerInput):
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
        self._session_epoch = None

    def begin_session(self, epoch: int) -> None:
        """Bind all solver traffic to one actuator-granted authority epoch."""
        self._session_epoch = validate_ik_session_epoch(epoch)

    def clear_session(self) -> None:
        """Revoke the adapter's ability to publish IK commands."""
        self._session_epoch = None

    def _require_session_epoch(self) -> int:
        if self._session_epoch is None:
            raise RuntimeError("IK input has no active authority epoch")
        return validate_ik_session_epoch(self._session_epoch)

    def focus(self):
        """
        Publishes the current position of the arm position on state change back to IK mode
        """
        epoch = self._require_session_epoch()
        cmd = Float64MultiArray()
        cmd.data = [
            self.controller.get_joint_rad(JointNum.BASE),
            self.controller.get_joint_rad(JointNum.SHOULDER),
            self.controller.get_joint_rad(JointNum.ELBOW),
            self.controller.get_joint_rad(JointNum.WRIST),
            self.controller.get_joint_rad(JointNum.HAND),
            # Float64 is exact for every bounded integer epoch accepted above.
            float(epoch),
        ]
        self.curr_pos_pub.publish(cmd)

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
            epoch = self._require_session_epoch()
            cmd = Float64MultiArray()
            cmd.data = [
                finite_float(-msg.axes[GamepadAxis.X.value], "Cartesian X"),
                finite_float(msg.axes[GamepadAxis.Y.value], "Cartesian Y"),
                finite_float(msg.axes[GamepadAxis.RY.value], "Cartesian Z"),
                float(msg.buttons[GamepadButton.Y.value]),  # data[3]: home button
                float(epoch),
            ]
            self.cartesian_pub.publish(cmd)
        
        if msg.buttons[GamepadButton.Y.value] == 1:
            # The solver owns the bounded incremental center trajectory. Moving
            # the local target ahead of its first response would manufacture a
            # large reverse step and trip the actuator discontinuity guard.
            self.get_logger().info("Center request sent to the IK solver.")
            return

        # The IK solver owns all five articulated-joint targets after one full
        # state synchronization. Local wrist/hand updates would make its seed
        # stale and could undo motion or trip the actuator discontinuity guard.
        # Add a stamped, acknowledged resynchronization protocol before
        # introducing any direct joint adjustments in this mode.

        # L3 is the dedicated operator-enable input and must never command a joint.
        if msg.buttons[GamepadButton.L1.value] == 1:
            self.get_logger().info("Closing gripper.")
            self.controller.set_joint(JointNum.GRIPPER, self.GRIPPER_CLOSED_PERCENT, record_activity=False)
        elif msg.buttons[GamepadButton.R1.value] == 1 or msg.buttons[GamepadButton.R3.value] == 1:
            self.get_logger().info("Opening gripper.")
            self.controller.set_joint(JointNum.GRIPPER, self.GRIPPER_OPEN_PERCENT, record_activity=False)

    def mov_callback(self, msg: JointState):
        """
        @param msg: JointState with the validated five-joint IK schema.
        """
        if len(msg.position) != 5:
            raise ValueError("IK JointState must contain exactly five positions")
        # IK output never renews the operator dead-man lease; only fresh joystick
        # input may keep IK control armed.
        self.controller.set_joint_targets_rad_atomic(
            (
                (JointNum.BASE, finite_float(msg.position[0], "IK base")),
                (JointNum.SHOULDER, finite_float(msg.position[1], "IK shoulder")),
                (JointNum.ELBOW, finite_float(msg.position[2], "IK elbow")),
                (JointNum.WRIST, finite_float(msg.position[3], "IK wrist")),
                (JointNum.HAND, finite_float(msg.position[4], "IK hand")),
            ),
            record_activity=False,
        )
