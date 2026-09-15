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

"""Actuator boundary and fail-safe state for the Baby Robot Arm."""

from __future__ import annotations

from enum import Enum
import math
import time
from typing import Callable, Iterable, List, Optional, Tuple

from safety import SafetyValidationError, finite_float, validate_duration, validate_servo_limits


class JointNum(Enum):
    BASE = 0
    SHOULDER = 1
    ELBOW = 2
    WRIST = 3
    HAND = 4
    GRIPPER = 5


NUM_SERVOS = JointNum.GRIPPER.value + 1


class Joint:
    """Immutable calibration plus the controller's commanded joint state."""

    SMOOTHING_FACTOR = 0.075
    MAX_STEP_PERCENT = 2.0

    SERVO_SPEEDS = [0.5, 0.4, 0.4, 0.7, 1.5, 1.5]
    SERVO_PWM_LIMITS = [
        (602, 1012),
        (602, 1012),
        (602, 1012),
        (602, 1012),
        (602, 1012),
        (620, 930),
    ]
    SERVO_ANGLE_RANGE_DEG = [270, 270, 270, 180, 180, 180]

    def __init__(self, joint: JointNum, min_pos: float, max_pos: float):
        self.joint = joint
        self._min_pos = min_pos
        self._max_pos = max_pos
        self.center = max(min_pos, min(max_pos, 50.0))
        self.target = self.center
        # Physical pose is unknown while outputs are disabled. The explicit arm
        # transition acknowledges that limitation before a first pulse is sent.
        self.current: Optional[float] = None
        self.speed = self.SERVO_SPEEDS[joint.value]
        self.max_rad = math.radians(self.SERVO_ANGLE_RANGE_DEG[joint.value] / 2)
        self.pwm_pin = joint.value
        self.min_pulse, self.max_pulse = self.SERVO_PWM_LIMITS[joint.value]

    @property
    def min_pos(self) -> float:
        return self._min_pos

    @property
    def max_pos(self) -> float:
        return self._max_pos

    def establish_initial_reference(self) -> None:
        if self.current is None:
            self.current = self.target

    def update(self) -> float:
        if self.current is None:
            raise RuntimeError("joint position reference has not been established")
        error = self.target - self.current
        step = max(-self.MAX_STEP_PERCENT, min(self.MAX_STEP_PERCENT, error * self.SMOOTHING_FACTOR))
        self.current += step
        return self.current


class ArmController:
    """Owns servo targets and keeps the actuator boundary fail closed.

    Integrators should provide ``hardware_output_hook`` to drive an independent,
    normally-disabled PCA9685 OE or servo-power circuit. The callback is invoked
    with ``False`` before I2C cleanup and with ``True`` only after all PWM
    channels have been preloaded. Software-only arming is an explicit bench-only
    opt-in because it cannot fail safe after process or I2C loss.
    """

    MAX_EXTERNAL_TARGET_STEP_PERCENT = 20.0

    def __init__(
        self,
        logger,
        servo_min_limits: Iterable[float],
        servo_max_limits: Iterable[float],
        pwm_factory: Optional[Callable[[], object]] = None,
        hardware_output_hook: Optional[Callable[[bool], None]] = None,
        allow_software_only_output: bool = False,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ):
        self._logger = logger
        minimums, maximums = validate_servo_limits(
            tuple(servo_min_limits), tuple(servo_max_limits), NUM_SERVOS
        )
        self._servo_min_limits = minimums
        self._servo_max_limits = maximums
        self._joints: List[Joint] = [
            Joint(JointNum(index), minimums[index], maximums[index]) for index in range(NUM_SERVOS)
        ]

        self._monotonic = monotonic
        self._sleep = sleep
        self._hardware_output_hook = hardware_output_hook
        if not isinstance(allow_software_only_output, bool):
            raise SafetyValidationError("allow_software_only_output must be boolean")
        self._allow_software_only_output = allow_software_only_output
        self._armed = False
        self.servos_are_released = True
        self.pwm_outputs_enabled = False
        self.last_input_time: Optional[float] = None
        self._fault_reason: Optional[str] = "not armed"

        # Assert an independent output-disable hook before touching I2C.
        if self._hardware_output_hook is not None:
            self._hardware_output_hook(False)

        if pwm_factory is None:
            from PCA9685 import PCA9685

            pwm_factory = PCA9685

        self.pwm = None
        try:
            self.pwm = pwm_factory()
            self.pwm.set_pwm_freq(50)
            self.get_logger().info("PCA9685 initialized with outputs disabled.")
        except Exception:
            if self._hardware_output_hook is not None:
                try:
                    self._hardware_output_hook(False)
                except Exception:
                    pass
            if self.pwm is not None:
                try:
                    self.pwm.disable_all_pwm()
                    self.pwm.close()
                except Exception:
                    pass
            raise

        self.get_logger().info(f"Loaded immutable MIN limits: {minimums}")
        self.get_logger().info(f"Loaded immutable MAX limits: {maximums}")

    @property
    def is_armed(self) -> bool:
        return self._armed

    @property
    def fault_reason(self) -> Optional[str]:
        return self._fault_reason

    def arm(self) -> None:
        """Acknowledge the unknown initial pose and permit a first safe pulse."""
        if self._armed:
            return
        if self._hardware_output_hook is None and not self._allow_software_only_output:
            raise RuntimeError(
                "hardware OE/power hook is required; software-only output was not authorized"
            )
        for joint in self._joints:
            joint.establish_initial_reference()
        self._fault_reason = None
        self._armed = True
        self.servos_are_released = False
        self.pwm_outputs_enabled = False
        self.note_command_activity()
        if self._hardware_output_hook is None:
            self.get_logger().warn(
                "Bench-only software output override active; crash-safe disable is unavailable."
            )

    def note_command_activity(self, transport_age_sec: float = 0.0) -> None:
        """Renew authority without granting queued samples a second full lease."""
        if not self._armed:
            raise RuntimeError("cannot renew command lease while controller is disarmed")
        safe_age = finite_float(transport_age_sec, "command transport age")
        if safe_age < 0.0:
            raise SafetyValidationError("command transport age must be non-negative")
        self.last_input_time = self._monotonic() - safe_age

    def command_age(self, now: Optional[float] = None) -> float:
        if self.last_input_time is None:
            return math.inf
        current_time = self._monotonic() if now is None else finite_float(now, "monotonic time")
        return max(0.0, current_time - self.last_input_time)

    def update(self) -> None:
        if not self._armed or self.servos_are_released:
            return
        try:
            for joint in self._joints:
                self._set_percent(joint, joint.update())

            if not self.pwm_outputs_enabled:
                # Clearing global FULL_OFF exposes all 16 PCA9685 channels.
                # Explicitly FULL_OFF every unmanaged channel first so stale
                # warm-restart register values cannot energize other outputs.
                for channel in range(NUM_SERVOS, 16):
                    self.pwm.disable_channel(channel)
                self.pwm.enable_all_pwm()
                if self._hardware_output_hook is not None:
                    self._hardware_output_hook(True)
                self.pwm_outputs_enabled = True
        except Exception as exc:
            self.fault_stop(f"PWM update failed: {exc}")
            raise

    def _require_armed(self) -> None:
        if not self._armed:
            raise RuntimeError("controller is disarmed")

    def move_joint(self, joint_num: JointNum, delta: float, record_activity: bool = True) -> float:
        self._require_armed()
        safe_delta = finite_float(delta, "joint delta")
        if not -1.0 <= safe_delta <= 1.0:
            raise SafetyValidationError("joint delta must be in [-1, 1]")
        joint = self._joints[joint_num.value]
        return self.set_joint(
            joint_num, joint.target + safe_delta * joint.speed, record_activity=record_activity
        )

    def set_joint(self, joint_num: JointNum, abs_pos: float, record_activity: bool = True) -> float:
        self._require_armed()
        joint = self._joints[joint_num.value]
        safe_position = finite_float(abs_pos, f"{joint_num.name} target")
        safe_position = min(max(safe_position, joint.min_pos), joint.max_pos)
        joint.target = safe_position
        if record_activity:
            self.note_command_activity()
        return safe_position

    def _percent_from_radians(self, joint_num: JointNum, radians: float) -> float:
        joint = self._joints[joint_num.value]
        safe_radians = finite_float(radians, f"{joint_num.name} radians")
        if not -joint.max_rad <= safe_radians <= joint.max_rad:
            raise SafetyValidationError(f"{joint_num.name} radians exceed the nominal servo range")
        percent = (safe_radians + joint.max_rad) / (joint.max_rad * 2.0) * 100.0
        if not joint.min_pos <= percent <= joint.max_pos:
            raise SafetyValidationError(f"{joint_num.name} radians exceed calibrated limits")
        return percent

    def set_joint_targets_rad_atomic(
        self,
        commands: Iterable[Tuple[JointNum, float]],
        record_activity: bool = True,
    ) -> Tuple[float, ...]:
        self._require_armed()
        prepared = []
        seen = set()
        for joint_num, radians in tuple(commands):
            if not isinstance(joint_num, JointNum) or joint_num in seen:
                raise SafetyValidationError("joint command contains an invalid or duplicate joint")
            seen.add(joint_num)
            percent = self._percent_from_radians(joint_num, radians)
            joint = self._joints[joint_num.value]
            if abs(percent - joint.target) > self.MAX_EXTERNAL_TARGET_STEP_PERCENT:
                raise SafetyValidationError(f"{joint_num.name} command exceeds maximum target step")
            prepared.append((joint, percent))
        if not prepared:
            raise SafetyValidationError("joint command is empty")

        for joint, percent in prepared:
            joint.target = percent
        if record_activity:
            self.note_command_activity()
        return tuple(percent for _, percent in prepared)

    def set_joint_targets_percent_atomic(
        self,
        commands: Iterable[Tuple[JointNum, float]],
        record_activity: bool = False,
    ) -> Tuple[float, ...]:
        self._require_armed()
        prepared = []
        seen = set()
        for joint_num, value in tuple(commands):
            if not isinstance(joint_num, JointNum) or joint_num in seen:
                raise SafetyValidationError("joint command contains an invalid or duplicate joint")
            seen.add(joint_num)
            joint = self._joints[joint_num.value]
            percent = finite_float(value, f"{joint_num.name} target")
            if not joint.min_pos <= percent <= joint.max_pos:
                raise SafetyValidationError(f"{joint_num.name} target exceeds calibrated limits")
            prepared.append((joint, percent))
        if not prepared:
            raise SafetyValidationError("joint command is empty")

        for joint, percent in prepared:
            joint.target = percent
        if record_activity:
            self.note_command_activity()
        return tuple(percent for _, percent in prepared)

    def constrain_internal_target_percent(self, joint_num: JointNum, value: float) -> float:
        """Bound an internally generated trajectory to immutable calibration.

        External command paths still reject out-of-range samples atomically;
        deterministic demonstration curves may be clipped at a configured edge
        so a narrower site calibration stops motion without faulting the unit.
        """
        if not isinstance(joint_num, JointNum):
            raise SafetyValidationError("internal target contains an invalid joint")
        joint = self._joints[joint_num.value]
        percent = finite_float(value, f"{joint_num.name} internal target")
        return min(joint.max_pos, max(joint.min_pos, percent))

    def set_joint_rad(self, joint_num: JointNum, radians: float, record_activity: bool = True) -> float:
        return self.set_joint_targets_rad_atomic(
            ((joint_num, radians),), record_activity=record_activity
        )[0]

    def get_joint_rad(self, joint_num: JointNum) -> float:
        joint = self._joints[joint_num.value]
        if joint.current is None:
            raise RuntimeError("joint position is unknown while controller is disarmed")
        return joint.current * (joint.max_rad * 2.0) / 100.0 - joint.max_rad

    def center_all_servos(self, record_activity: bool = True) -> None:
        self._require_armed()
        self.set_joint_targets_percent_atomic(
            ((joint.joint, joint.center) for joint in self._joints),
            record_activity=record_activity,
        )

    def center_joint(self, joint_num: JointNum, record_activity: bool = True) -> float:
        joint = self._joints[joint_num.value]
        return self.set_joint(joint_num, joint.center, record_activity=record_activity)

    def freeze_targets_at_current(self) -> Tuple[float, ...]:
        """Drop queued motion without changing the output-enable state.

        A mode transition transfers command authority to a different producer.
        Freezing first prevents that producer's lease from continuing a target
        left behind by the previous mode or an autonomous trajectory.
        """
        frozen_targets = tuple(
            joint.center if joint.current is None else joint.current for joint in self._joints
        )
        for joint, frozen_target in zip(self._joints, frozen_targets):
            joint.target = frozen_target
        return frozen_targets

    def fault_stop(self, reason: str = "fault stop") -> bool:
        """Disable output immediately; never command a parking movement here."""
        self._armed = False
        self.servos_are_released = True
        self.pwm_outputs_enabled = False
        self._fault_reason = reason

        # Drop every queued trajectory as part of the stop transition. Snapshot
        # first so all targets are committed from one controller state; a joint
        # with no commanded reference yet retains its safe initial center.
        self.freeze_targets_at_current()

        success = True

        # Use an independent hook first because the I2C path itself may be the fault.
        if self._hardware_output_hook is not None:
            try:
                self._hardware_output_hook(False)
            except Exception as exc:
                success = False
                self.get_logger().error(f"Hardware output-disable hook failed: {exc}")
        try:
            self.pwm.disable_all_pwm()
        except Exception as exc:
            success = False
            self.get_logger().error(f"PCA9685 FULL_OFF failed: {exc}")
        return success

    def release_all_servos(self) -> None:
        self.fault_stop("servo release")

    def is_servos_centered(self) -> bool:
        return all(
            joint.current is not None
            and math.isfinite(joint.current)
            and abs(joint.current - joint.center) <= Joint.SMOOTHING_FACTOR
            for joint in self._joints
        )

    def park_and_stop(self, timeout_sec: float) -> bool:
        """Optional clean shutdown path; faults must call :meth:`fault_stop`."""
        timeout = validate_duration(timeout_sec, "park timeout", 0.1, 30.0)
        if not self._armed:
            self.fault_stop("park requested while disarmed")
            return True
        deadline = self._monotonic() + timeout
        try:
            self.center_all_servos(record_activity=False)
            while not self.is_servos_centered():
                if self._monotonic() >= deadline:
                    self.fault_stop("parking timed out")
                    return False
                self.update()
                self._sleep(0.02)
            self.fault_stop("clean parking complete")
            return True
        except Exception as exc:
            self.fault_stop(f"parking failed: {exc}")
            return False

    def enable_position_limits(self, enabled: bool) -> None:
        """Compatibility shim: calibrated hardware limits are immutable."""
        if not enabled:
            raise SafetyValidationError("calibrated actuator limits cannot be disabled")

    def close(self) -> None:
        self.fault_stop("controller closed")
        close_method = getattr(self.pwm, "close", None)
        if close_method is not None:
            try:
                close_method(disable=False)
            except TypeError:
                close_method()

    def get_logger(self):
        return self._logger

    def _set_percent(self, joint: Joint, percentage: float) -> None:
        safe_percentage = finite_float(percentage, f"{joint.joint.name} output")
        if not joint.min_pos <= safe_percentage <= joint.max_pos:
            raise SafetyValidationError(f"{joint.joint.name} output exceeded calibrated limits")
        pwm_value = int(
            joint.min_pulse
            + (safe_percentage / 100.0) * (joint.max_pulse - joint.min_pulse)
        )
        self.pwm.set_pwm(joint.pwm_pin, 500, pwm_value)
