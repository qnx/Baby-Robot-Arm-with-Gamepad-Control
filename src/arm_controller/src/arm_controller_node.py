#!/system/bin/env python3
"""ROS 2 hardware boundary for the five-joint arm and gripper.

Outputs start disabled. With the current open-loop hardware, the operator must
place/clear the arm, hold L3, and press Select+Start+A to arm. Releasing L3 or
pressing Select+Start+B immediately disarms. A normally-disabled hardware OE or servo-power watchdog is still
required for protection from SIGKILL, process loss, or a failed I2C bus.
"""

from __future__ import annotations

from enum import Enum
import math
import time
from typing import Dict, Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray, String, UInt64

from arm_control import ArmController, NUM_SERVOS
from arm_controller_input import (
    ArmControllerInverseKinematicInput,
    ArmControllerJointInput,
    ArmControllerJoystickInput,
    ArmControllerScreensaverInput,
    GamepadButton,
    ScreenSaverDance,
)
from PCA9685 import PCA9685
from safety import (
    deadman_is_held,
    MAX_IK_SESSION_EPOCH,
    SafetyValidationError,
    requested_arm_action,
    requested_control_mode,
    validate_duration,
    validate_joy_message,
    validate_joint_state,
    validate_ik_session_epoch,
    validate_message_stamp,
    validate_servo_limits,
)


DEADZONE = 0.08
DEFAULT_COMMAND_TIMEOUT_SEC = 0.5
DEFAULT_AUTONOMOUS_LEASE_SEC = 0.35
DEFAULT_AUTONOMOUS_MAX_DURATION_SEC = 30.0
DEFAULT_SERVO_MIN_LIMITS = [25.0, 0.0, 0.0, 0.0, 0.0, 15.0]
DEFAULT_SERVO_MAX_LIMITS = [75.0, 50.0, 100.0, 100.0, 100.0, 65.0]
IK_JOINT_NAMES = ("base", "shoulder", "elbow", "wrist", "hand")

class ArmControlMode(Enum):
    JOYSTICK = 0
    JOINT = 1
    IK = 2


class ArmControllerNode(Node):
    """Validates command samples before they can reach physical actuators."""

    def __init__(self):
        super().__init__("arm_controller_node")
        self.controller: Optional[ArmController] = None
        self.smoothing_timer = None
        self._rejection_log_times: Dict[str, float] = {}
        self._last_message_stamps: Dict[str, int] = {}
        self._last_arm_action: Optional[str] = None
        self._joint_mode_grant_ns: Optional[int] = None
        self._joint_epoch_counter = 0
        self._active_joint_epoch: Optional[int] = None
        self._ik_epoch_counter = 0
        self._active_ik_epoch: Optional[int] = None
        self._autonomous_started_at: Optional[float] = None
        self._autonomous_lease_at: Optional[float] = None

        readonly = ParameterDescriptor(read_only=True)
        self.declare_parameter(
            "servo_min_limits", DEFAULT_SERVO_MIN_LIMITS,
            ParameterDescriptor(description="Immutable calibrated servo minimum percentages", read_only=True),
        )
        self.declare_parameter(
            "servo_max_limits", DEFAULT_SERVO_MAX_LIMITS,
            ParameterDescriptor(description="Immutable calibrated servo maximum percentages", read_only=True),
        )
        self.declare_parameter("command_timeout_sec", DEFAULT_COMMAND_TIMEOUT_SEC, readonly)
        self.declare_parameter("autonomous_lease_sec", DEFAULT_AUTONOMOUS_LEASE_SEC, readonly)
        self.declare_parameter(
            "autonomous_max_duration_sec", DEFAULT_AUTONOMOUS_MAX_DURATION_SEC, readonly
        )
        # Repository data cannot attest the physical wrist channel order. Keep
        # demonstration trajectories unavailable until a deployment explicitly
        # verifies its wiring, linkages, and clear zone.
        self.declare_parameter("enable_autonomous_mode", False, readonly)
        # Never energize by default when process death cannot assert OE/power off.
        self.declare_parameter("allow_software_only_output", False, readonly)
        # The checked-in URDF is not a verified terminal five-joint chain. IK is
        # therefore an explicit deployment opt-in and fails closed by default.
        self.declare_parameter("enable_ik_mode", False, readonly)
        self.declare_parameter("i2c_interface", 1, readonly)
        self.declare_parameter("i2c_address", 0x40, readonly)

        try:
            minimums, maximums = validate_servo_limits(
                self.get_parameter("servo_min_limits").value,
                self.get_parameter("servo_max_limits").value,
                NUM_SERVOS,
            )
            self.command_timeout_sec = validate_duration(
                self.get_parameter("command_timeout_sec").value,
                "command_timeout_sec",
                0.05,
                5.0,
            )
            self.autonomous_lease_sec = validate_duration(
                self.get_parameter("autonomous_lease_sec").value,
                "autonomous_lease_sec",
                0.05,
                self.command_timeout_sec,
            )
            self.autonomous_max_duration_sec = validate_duration(
                self.get_parameter("autonomous_max_duration_sec").value,
                "autonomous_max_duration_sec",
                self.autonomous_lease_sec,
                300.0,
            )
            self.enable_autonomous_mode = self.get_parameter("enable_autonomous_mode").value
            if not isinstance(self.enable_autonomous_mode, bool):
                raise SafetyValidationError("enable_autonomous_mode must be boolean")
            self.allow_software_only_output = self.get_parameter("allow_software_only_output").value
            if not isinstance(self.allow_software_only_output, bool):
                raise SafetyValidationError("allow_software_only_output must be boolean")
            self.enable_ik_mode = self.get_parameter("enable_ik_mode").value
            if not isinstance(self.enable_ik_mode, bool):
                raise SafetyValidationError("enable_ik_mode must be boolean")

            i2c_interface = self.get_parameter("i2c_interface").value
            i2c_address = self.get_parameter("i2c_address").value
            if isinstance(i2c_interface, bool) or not isinstance(i2c_interface, int) or i2c_interface < 0:
                raise SafetyValidationError("i2c_interface must be a non-negative integer")
            if isinstance(i2c_address, bool) or not isinstance(i2c_address, int):
                raise SafetyValidationError("i2c_address must be an integer")

            self.controller = ArmController(
                self.get_logger(),
                minimums,
                maximums,
                pwm_factory=lambda: PCA9685(
                    i2c_interface,
                    i2c_address,
                    lock_path=(
                        f"/var/run/baby_robot_arm/"
                        f"pca9685-i2c{i2c_interface}-0x{i2c_address:02x}.lock"
                    ),
                ),
                # This repository has no board-specific OE GPIO implementation.
                # The secured launch profile therefore refuses to arm.
                hardware_output_hook=None,
                allow_software_only_output=self.allow_software_only_output,
            )
        except Exception as exc:
            self.get_logger().error(f"Arm controller initialization failed safely: {exc}")
            rclpy.try_shutdown()
            return

        # Relative names allow per-robot namespace isolation and SROS2 policy.
        self.cartesian_pub = self.create_publisher(Float64MultiArray, "CartesianCmd", 1)
        self.curr_pos_pub = self.create_publisher(Float64MultiArray, "CurrentPositions", 1)
        self.mov_subscription = self.create_subscription(JointState, "mov", self.mov_callback, 1)
        self.joy_subscription = self.create_subscription(Joy, "joy", self.joy_callback, 1)
        self.joint_subscription = self.create_subscription(JointState, "joint", self.joint_callback, 1)
        self.mode_publisher = self.create_publisher(String, "mode", 1)
        # A source timestamp cannot prove cross-topic happened-after when the
        # producer clock is skewed. Direct-joint producers must echo this grant
        # in JointState.header.frame_id as ``joint-session:N``.
        self.joint_session_publisher = self.create_publisher(UInt64, "joint_session", 1)

        self.input_joystick = ArmControllerJoystickInput(self.controller, self.get_logger())
        self.input_screensaver = ArmControllerScreensaverInput(self.controller, self.get_logger())
        self.input_joint = ArmControllerJointInput(self.controller, self.get_logger())
        self.input_ik = ArmControllerInverseKinematicInput(
            self.controller,
            self.get_logger(),
            self.cartesian_pub,
            self.curr_pos_pub,
        )
        self.control_mode = ArmControlMode.JOYSTICK
        self.active_input = self.input_joystick
        self.screensaver_enabled = False
        self._publish_mode()
        self._publish_joint_session()

        self.smoothing_timer = self.create_timer(0.02, self.smoothing_loop)
        self.get_logger().warn(
            "Outputs are disabled. Clear/place the arm, hold L3, then press "
            "Select+Start+A to arm; releasing L3 or pressing Select+Start+B disarms."
        )

    def _publish_mode(self) -> None:
        message = String()
        message.data = self.control_mode.name.lower()
        self.mode_publisher.publish(message)

    def _publish_joint_session(self) -> None:
        """Announce the current direct-joint grant; zero means revoked."""
        message = UInt64()
        message.data = 0 if self._active_joint_epoch is None else self._active_joint_epoch
        self.joint_session_publisher.publish(message)

    def _fault_stop_and_revoke(self, reason: str) -> bool:
        """Disable hardware first, then revoke the direct-producer session."""
        success = self.controller.fault_stop(reason)
        self._joint_mode_grant_ns = None
        self._active_joint_epoch = None
        self._last_message_stamps.pop("joint", None)
        try:
            self._publish_joint_session()
        except Exception as exc:
            # The actuator is already disabled and a later re-arm rotates the
            # epoch. A failed advisory publication must not skip local cleanup.
            self.get_logger().error(f"Failed to publish direct-joint revocation: {exc}")
        return success

    def _warn_rejected(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self._rejection_log_times.get(key, -math.inf) >= 2.0:
            self.get_logger().warn(message)
            self._rejection_log_times[key] = now

    def _controller_receipt_time_ns(self) -> int:
        receipt_ns = self.get_clock().now().nanoseconds
        if isinstance(receipt_ns, bool) or not isinstance(receipt_ns, int) or receipt_ns < 0:
            raise SafetyValidationError("controller ROS receipt time is invalid")
        return receipt_ns

    def _validated_stamp(self, message, channel: str, ros_now_ns: Optional[int] = None):
        stamp = message.header.stamp
        if ros_now_ns is None:
            ros_now_ns = self._controller_receipt_time_ns()
        accepted_stamp = validate_message_stamp(
            stamp.sec,
            stamp.nanosec,
            ros_now_ns,
            self._last_message_stamps.get(channel),
            self.command_timeout_sec,
        )
        # Preserve the source sample's remaining authority instead of granting
        # a delayed queued sample a fresh full lease at receipt time.
        transport_age_sec = max(0.0, (ros_now_ns - accepted_stamp) / 1_000_000_000.0)
        return accepted_stamp, transport_age_sec

    def _consume_joy_stop_stamp_best_effort(self, message: Joy) -> None:
        """Burn a valid newer stop stamp only after output is already disabled.

        Dead-man release must never depend on a usable clock or header. When the
        stop sample does carry a valid newer stamp, recording it prevents the
        same timestamp from being replayed with an arming payload afterward.
        """
        try:
            receipt_ns = self._controller_receipt_time_ns()
            stamp, _transport_age_sec = self._validated_stamp(message, "joy", receipt_ns)
            self._last_message_stamps["joy"] = stamp
        except Exception:
            # The safety transition happened before this helper. Missing, stale,
            # replayed, or otherwise unusable metadata cannot defer revocation.
            return

    def _reset_mode_authority(self, grant_receipt_ns: Optional[int] = None) -> None:
        """Revoke queued producer traffic and establish one fresh mode grant."""
        self._joint_mode_grant_ns = None
        self._active_joint_epoch = None
        self._active_ik_epoch = None
        self.input_ik.clear_session()
        self._last_message_stamps.pop("joint", None)
        self._last_message_stamps.pop("mov", None)

        if self.control_mode == ArmControlMode.JOINT:
            receipt_ns = (
                self._controller_receipt_time_ns()
                if grant_receipt_ns is None
                else grant_receipt_ns
            )
            if isinstance(receipt_ns, bool) or not isinstance(receipt_ns, int) or receipt_ns < 0:
                raise SafetyValidationError("direct-joint grant receipt time is invalid")
            # The grant is local callback receipt time, deliberately not the Joy
            # source stamp. The separately published epoch supplies the actual
            # cross-topic happened-after proof; this time remains a freshness
            # check and defense in depth.
            if self._joint_epoch_counter >= MAX_IK_SESSION_EPOCH:
                self._publish_joint_session()
                raise SafetyValidationError("direct-joint session epoch space is exhausted")
            self._joint_epoch_counter += 1
            self._active_joint_epoch = self._joint_epoch_counter
            self._joint_mode_grant_ns = receipt_ns
        elif self.control_mode == ArmControlMode.IK:
            if self._ik_epoch_counter >= MAX_IK_SESSION_EPOCH:
                raise SafetyValidationError("IK session epoch space is exhausted")
            next_epoch = validate_ik_session_epoch(self._ik_epoch_counter + 1)
            self.input_ik.begin_session(next_epoch)
            self._ik_epoch_counter = next_epoch
            self._active_ik_epoch = next_epoch
        self._publish_joint_session()

    def _update_mode(
        self,
        mode: ArmControlMode,
        force: bool = False,
        grant_receipt_ns: Optional[int] = None,
    ) -> None:
        if self.control_mode == mode and not force:
            return
        # A fresh mode lease must never continue a target queued by the prior
        # producer. ``force`` covers autonomous cancellation back into the
        # already-selected manual mode.
        self.controller.freeze_targets_at_current()
        self.screensaver_enabled = False
        self.control_mode = mode
        if mode == ArmControlMode.JOYSTICK:
            self.active_input = self.input_joystick
        elif mode == ArmControlMode.JOINT:
            self.active_input = self.input_joint
        else:
            self.active_input = self.input_ik
        # Establish the new grant before focus() can publish a synchronization
        # sample. Leaving IK clears the old epoch at this same boundary.
        self._reset_mode_authority(grant_receipt_ns)
        # Calibrated actuator limits remain enabled in every mode.
        self.controller.enable_position_limits(True)
        self._publish_mode()
        self.active_input.focus()

    def _handle_arm_chord(
        self,
        buttons,
        axes,
        transport_age_sec: float,
        grant_receipt_ns: int,
    ) -> bool:
        action = requested_arm_action(buttons, axes, DEADZONE)
        if action is None:
            self._last_arm_action = None
            return False
        if action == self._last_arm_action:
            return True

        self._last_arm_action = action
        self.screensaver_enabled = False
        if action == "arm":
            self.controller.arm()
            # A recognized arm chord is a fresh target-authority boundary even
            # if outputs were already armed. This also drops an autonomous
            # trajectory when the chord exits a dance in joystick mode.
            self.controller.freeze_targets_at_current()
            if self.control_mode in (ArmControlMode.JOINT, ArmControlMode.IK):
                # Re-arming, including while already armed, is a new authority
                # epoch. Invalidate every queued DDS command before the new
                # actuator lease is acknowledged.
                self._reset_mode_authority(grant_receipt_ns)
                self.active_input.focus()
            self.controller.note_command_activity(transport_age_sec)
            self.get_logger().warn("Arm outputs armed by explicit operator chord.")
        else:
            self._fault_stop_and_revoke("operator disarm")
            self._publish_mode()
            self.get_logger().info("Arm outputs disarmed.")
        return True

    @staticmethod
    def _requested_dance(buttons) -> Optional[ScreenSaverDance]:
        choices = (
            (GamepadButton.A, ScreenSaverDance.FIGURE_8),
            (GamepadButton.B, ScreenSaverDance.WAVE),
            (GamepadButton.X, ScreenSaverDance.COBRA),
            (GamepadButton.Y, ScreenSaverDance.CONDUCTOR),
        )
        selected = [(button, dance) for button, dance in choices if buttons[button.value]]
        if not selected:
            return None
        if len(selected) != 1:
            raise SafetyValidationError("select exactly one autonomous motion pattern")
        button, dance = selected[0]
        pressed = {index for index, value in enumerate(buttons) if value}
        expected = {
            button.value,
            GamepadButton.START.value,
            GamepadButton.L3.value,
        }
        if pressed != expected:
            # Autonomous entry transfers target ownership, so unrelated held
            # inputs must not be interpreted as consent for that transition.
            raise SafetyValidationError("autonomous motion chord contains extra or missing buttons")
        return dance

    def _enable_or_renew_screensaver(
        self, dance: ScreenSaverDance, transport_age_sec: float
    ) -> None:
        now = time.monotonic()
        if not self.screensaver_enabled:
            # Starting an autonomous producer transfers target ownership; drop
            # any manual-mode trajectory before its lease is renewed.
            self.controller.freeze_targets_at_current()
            self.controller.enable_position_limits(True)
            self.screensaver_enabled = True
            self._autonomous_started_at = now
            self.input_screensaver.start(dance)
        elif self.input_screensaver.dance != dance:
            self.controller.freeze_targets_at_current()
            self.input_screensaver.start(dance)
        self._autonomous_lease_at = now - transport_age_sec
        self.controller.note_command_activity(transport_age_sec)
        message = String()
        message.data = f"screensaver:{dance.name.lower()}"
        self.mode_publisher.publish(message)

    def _disable_screensaver(self, grant_receipt_ns: Optional[int] = None) -> None:
        self.screensaver_enabled = False
        self._autonomous_started_at = None
        self._autonomous_lease_at = None
        self._update_mode(
            self.control_mode,
            force=True,
            grant_receipt_ns=grant_receipt_ns,
        )

    def _active_authority_is_current(self, now: Optional[float] = None) -> bool:
        """Fail-stop an expired authority before a callback can renew it.

        The node runs these callbacks and the smoothing timer on rclpy's
        single-threaded executor. Keeping the age check in the same callback as
        the ensuing mutation closes the interval in which an already-expired
        lease could otherwise be renewed before the timer observed it.
        """
        if not self.controller.is_armed:
            return False

        current_time = time.monotonic() if now is None else now
        if self.screensaver_enabled:
            lease_expired = (
                self._autonomous_lease_at is None
                or current_time - self._autonomous_lease_at > self.autonomous_lease_sec
            )
            duration_expired = (
                self._autonomous_started_at is None
                or current_time - self._autonomous_started_at
                > self.autonomous_max_duration_sec
            )
            if not (lease_expired or duration_expired):
                return True

            self.screensaver_enabled = False
            self._autonomous_started_at = None
            self._autonomous_lease_at = None
            self._fault_stop_and_revoke("autonomous motion lease expired")
            self._publish_mode()
            self.get_logger().warn("Autonomous motion stopped: lease or duration expired.")
            return False

        if self.controller.command_age(current_time) <= self.command_timeout_sec:
            return True

        self._fault_stop_and_revoke("command lease expired")
        self._publish_mode()
        self.get_logger().warn("Command lease expired; outputs disabled.")
        return False

    def joy_callback(self, message: Joy) -> None:
        try:
            axes, buttons = validate_joy_message(message.axes, message.buttons)
            message.axes = [axis if abs(axis) > DEADZONE else 0.0 for axis in axes]
            message.buttons = list(buttons)

            # Safe-stop input is intentionally independent of command freshness:
            # a paused/rolled-back ROS clock or replayed neutral may deny service,
            # but it must never postpone output disable.  Shape validation still
            # precedes indexing, while timestamps remain mandatory for authority.
            if self.controller.is_armed and not deadman_is_held(message.buttons):
                self.screensaver_enabled = False
                self._fault_stop_and_revoke("operator dead-man released")
                self._consume_joy_stop_stamp_best_effort(message)
                self._publish_mode()
                self._warn_rejected("deadman", "Operator enable released; outputs disabled.")
                return

            receipt_ns = self._controller_receipt_time_ns()
            stamp, transport_age_sec = self._validated_stamp(message, "joy", receipt_ns)
            # Consume the source high-water mark as soon as validation succeeds.
            # Even a command that discovers an expired local lease must not be
            # replayable with different button content as a later arm chord.
            self._last_message_stamps["joy"] = stamp

            if self._handle_arm_chord(
                message.buttons,
                message.axes,
                transport_age_sec,
                receipt_ns,
            ):
                return
            if not self.controller.is_armed:
                self._warn_rejected("unarmed-joy", "Ignored joystick command while outputs are disarmed.")
                return

            # An ordinary Joy sample must not resurrect an authority lease that
            # expired between timer ticks. The exact arm chord above is the only
            # input allowed to establish a new frozen authority boundary.
            if not self._active_authority_is_current():
                return

            # A connected idle controller is not operator intent. Every active
            # mode therefore requires the dedicated L3 enable to remain held.
            self.controller.note_command_activity(transport_age_sec)

            if message.buttons[GamepadButton.SELECT.value]:
                mode_name = requested_control_mode(message.buttons, self.enable_ik_mode)
                if mode_name is not None:
                    requested_mode = {
                        "joystick": ArmControlMode.JOYSTICK,
                        "joint": ArmControlMode.JOINT,
                        "ik": ArmControlMode.IK,
                    }[mode_name]
                    self._update_mode(
                        requested_mode,
                        grant_receipt_ns=receipt_ns,
                    )
                return

            if message.buttons[GamepadButton.START.value]:
                if not self.enable_autonomous_mode:
                    self._warn_rejected(
                        "dance-disabled",
                        "Autonomous demonstrations are disabled until the physical joint map is verified.",
                    )
                    return
                dance = self._requested_dance(message.buttons)
                if dance is None:
                    self._warn_rejected("dance-select", "Select exactly one autonomous motion pattern.")
                else:
                    self._enable_or_renew_screensaver(dance, transport_age_sec)
                return

            has_input = any(message.axes) or any(message.buttons)
            if self.screensaver_enabled:
                if has_input:
                    self._disable_screensaver(receipt_ns)
                else:
                    # Releasing the hold-to-run button does not renew the lease.
                    return

            if has_input:
                self.active_input.joy_callback(message)
        except SafetyValidationError as exc:
            # Once armed, an invalid source sample cannot prove continued
            # operator authority. Prefer a denial of service to retaining live
            # actuator output until the remaining lease happens to expire.
            if self.controller.is_armed:
                self.screensaver_enabled = False
                self._fault_stop_and_revoke(f"invalid joystick input: {exc}")
                self._consume_joy_stop_stamp_best_effort(message)
                self._publish_mode()
            self._warn_rejected("joy", f"Rejected joystick command: {exc}")
        except Exception as exc:
            self._fault_stop_and_revoke(f"joystick callback failed: {exc}")
            self._consume_joy_stop_stamp_best_effort(message)
            self.get_logger().error(f"Joystick callback fault-stopped the arm: {exc}")

    def joint_callback(self, message: JointState) -> None:
        if not self.controller.is_armed or self.control_mode != ArmControlMode.JOINT or self.screensaver_enabled:
            return
        try:
            # Direct commands are subordinate to the operator's Joy lease and
            # cannot mutate targets after that lease has already expired.
            if not self._active_authority_is_current():
                return
            receipt_ns = self._controller_receipt_time_ns()
            validate_joint_state(message.name, message.position, self.input_joint.joint_map, NUM_SERVOS)
            if self._active_joint_epoch is None:
                raise SafetyValidationError("direct-joint mode has no active session epoch")
            expected_frame = f"joint-session:{self._active_joint_epoch}"
            if message.header.frame_id != expected_frame:
                raise SafetyValidationError(
                    f"direct-joint authority frame must be exactly '{expected_frame}'"
                )
            stamp, _transport_age_sec = self._validated_stamp(message, "joint", receipt_ns)
            if self._joint_mode_grant_ns is None:
                raise SafetyValidationError("direct-joint mode has no active authority grant")
            if stamp <= self._joint_mode_grant_ns:
                raise SafetyValidationError("direct-joint command predates the active mode grant")
            if stamp > receipt_ns:
                raise SafetyValidationError("direct-joint command timestamp is in the future")
            self.active_input.joint_callback(message)
            self._last_message_stamps["joint"] = stamp
        except (SafetyValidationError, ValueError) as exc:
            self._warn_rejected("joint", f"Rejected direct joint command: {exc}")
        except Exception as exc:
            self._fault_stop_and_revoke(f"joint callback failed: {exc}")
            self.get_logger().error(f"Joint callback fault-stopped the arm: {exc}")

    def mov_callback(self, message: JointState) -> None:
        if not self.controller.is_armed or self.control_mode != ArmControlMode.IK or self.screensaver_enabled:
            return
        try:
            # Solver output cannot outrun expiry of the operator authority that
            # created its current IK session.
            if not self._active_authority_is_current():
                return
            validated = validate_joint_state(
                message.name, message.position, IK_JOINT_NAMES, len(IK_JOINT_NAMES)
            )
            if tuple(name for name, _ in validated) != IK_JOINT_NAMES:
                raise SafetyValidationError("IK joint names/order do not match the actuator schema")
            if self._active_ik_epoch is None:
                raise SafetyValidationError("IK mode has no active authority epoch")
            expected_frame = f"ik-session:{self._active_ik_epoch}"
            if message.header.frame_id != expected_frame:
                raise SafetyValidationError(
                    f"IK output authority frame must be exactly '{expected_frame}'"
                )
            stamp, _transport_age_sec = self._validated_stamp(message, "mov")
            self.active_input.mov_callback(message)
            self._last_message_stamps["mov"] = stamp
        except (SafetyValidationError, ValueError) as exc:
            # IK output is generated inside this supervised control unit. An
            # invariant breach indicates solver/model/channel corruption, not
            # an operator typo, so continuing with the last target is unsafe.
            self._fault_stop_and_revoke(f"invalid IK output: {exc}")
            self._warn_rejected("mov", f"Rejected IK joint command: {exc}")
        except Exception as exc:
            self._fault_stop_and_revoke(f"IK callback failed: {exc}")
            self.get_logger().error(f"IK callback fault-stopped the arm: {exc}")

    def smoothing_loop(self) -> None:
        if not self.controller.is_armed:
            return
        now = time.monotonic()
        try:
            if not self._active_authority_is_current(now):
                return
            if self.screensaver_enabled:
                self.input_screensaver.update()
            else:
                self.active_input.update()
            self.controller.update()
        except Exception as exc:
            self._fault_stop_and_revoke(f"control loop failed: {exc}")
            self.get_logger().error(f"Control loop fault-stopped the arm: {exc}")

    def shutdown_controller(self) -> None:
        if self.smoothing_timer is not None:
            self.smoothing_timer.cancel()
        if self.controller is None:
            return
        # A catchable rclpy signal or executor exit cannot prove that the
        # surrounding control unit is healthy, so cleanup disables output
        # without motion. SIGKILL still requires normally-off hardware.
        self._fault_stop_and_revoke("node shutdown")
        self.controller.close()


def main(args=None) -> int:
    # Make SIGINT/SIGTERM cleanup explicit instead of relying on rclpy's
    # current default. Both signals shut the context and unwind through finally.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.ALL)
    node = ArmControllerNode()
    if node.controller is None:
        node.destroy_node()
        return 1

    faulted = False
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        faulted = True
    except Exception as exc:
        faulted = True
        node.get_logger().error(f"Executor failure: {exc}")
    finally:
        node.shutdown_controller()
        node.destroy_node()
        rclpy.try_shutdown()
    return 1 if faulted else 0


if __name__ == "__main__":
    raise SystemExit(main())
