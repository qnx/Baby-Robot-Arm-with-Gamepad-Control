"""Pure validation helpers for the arm controller safety boundary."""

from __future__ import annotations

import math
from typing import Iterable, Sequence, Tuple


EXPECTED_JOY_AXES = 6
EXPECTED_JOY_BUTTONS = 12
DEADMAN_BUTTON_INDEX = 10  # Logitech L3; kept out of the gripper mapping.
MAX_FUTURE_COMMAND_SEC = 0.1
# Float64MultiArray is retained for wire compatibility with the teaching
# project.  Restrict session identifiers to IEEE-754's consecutive integer
# range so an authority epoch cannot be rounded into a different epoch in DDS.
MAX_IK_SESSION_EPOCH = (1 << 53) - 1


class SafetyValidationError(ValueError):
    """Raised when a command or configuration violates a safety invariant."""


def validate_ik_session_epoch(value: object) -> int:
    """Return an exactly representable positive IK authority epoch."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise SafetyValidationError("IK session epoch must be an integer")
    if not 1 <= value <= MAX_IK_SESSION_EPOCH:
        raise SafetyValidationError(
            f"IK session epoch must be in [1, {MAX_IK_SESSION_EPOCH}]"
        )
    return value


def finite_float(value: object, field_name: str) -> float:
    """Return *value* as a finite float or reject it."""
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise SafetyValidationError(f"{field_name} must be numeric") from exc
    if not math.isfinite(result):
        raise SafetyValidationError(f"{field_name} must be finite")
    return result


def validate_duration(value: object, field_name: str, minimum: float, maximum: float) -> float:
    result = finite_float(value, field_name)
    if not minimum <= result <= maximum:
        raise SafetyValidationError(f"{field_name} must be in [{minimum}, {maximum}]")
    return result


def validate_joy_message(
    axes: Sequence[object],
    buttons: Sequence[object],
) -> Tuple[Tuple[float, ...], Tuple[int, ...]]:
    """Validate the one supported gamepad report shape atomically."""
    if len(axes) != EXPECTED_JOY_AXES:
        raise SafetyValidationError(f"joy.axes must contain exactly {EXPECTED_JOY_AXES} values")
    if len(buttons) != EXPECTED_JOY_BUTTONS:
        raise SafetyValidationError(f"joy.buttons must contain exactly {EXPECTED_JOY_BUTTONS} values")

    safe_axes = tuple(finite_float(value, f"joy.axes[{index}]") for index, value in enumerate(axes))
    if any(value < -1.0 or value > 1.0 for value in safe_axes):
        raise SafetyValidationError("joy axes must be in [-1, 1]")

    safe_buttons = []
    for index, value in enumerate(buttons):
        if isinstance(value, bool):
            button = int(value)
        elif isinstance(value, int):
            button = value
        else:
            raise SafetyValidationError(f"joy.buttons[{index}] must be an integer")
        if button not in (0, 1):
            raise SafetyValidationError(f"joy.buttons[{index}] must be 0 or 1")
        safe_buttons.append(button)
    return safe_axes, tuple(safe_buttons)


def requested_control_mode(buttons: Sequence[int], enable_ik_mode: bool) -> str | None:
    """Resolve the Select+A/B/X mode chord without silently enabling IK."""
    if len(buttons) != EXPECTED_JOY_BUTTONS or not buttons[8]:
        return None
    choices = ((0, "joystick"), (1, "ik"), (2, "joint"))
    selected = [(index, name) for index, name in choices if buttons[index]]
    if len(selected) != 1:
        raise SafetyValidationError("select exactly one control mode")
    selected_index, selected_name = selected[0]
    # Mode changes transfer command authority. Require the exact documented
    # dead-man chord so unrelated held buttons cannot make that transition.
    pressed = {index for index, value in enumerate(buttons) if value}
    if pressed != {selected_index, 8, DEADMAN_BUTTON_INDEX}:
        raise SafetyValidationError("control mode chord contains extra or missing buttons")
    if selected_name == "ik" and not enable_ik_mode:
        raise SafetyValidationError("IK mode is disabled until a verified five-joint model is configured")
    return selected_name


def requested_arm_action(
    buttons: Sequence[int], axes: Sequence[float], deadzone: float
) -> str | None:
    """Resolve the exact neutral arm/disarm chord.

    Arming includes the held L3 dead-man so releasing the face-button chord
    leaves an unbroken operator-enable signal. Disarming remains available
    whether L3 is held or has already been released.
    """
    if len(buttons) != EXPECTED_JOY_BUTTONS or not (buttons[8] and buttons[9]):
        return None
    if any(abs(axis) > deadzone for axis in axes):
        raise SafetyValidationError("arm/disarm chord requires neutral axes")

    pressed = {index for index, value in enumerate(buttons) if value}
    if pressed == {0, 8, 9, DEADMAN_BUTTON_INDEX}:
        return "arm"
    if pressed in ({1, 8, 9}, {1, 8, 9, DEADMAN_BUTTON_INDEX}):
        return "disarm"
    raise SafetyValidationError("arm/disarm chord is ambiguous or contains extra buttons")


def deadman_is_held(buttons: Sequence[int]) -> bool:
    """Return whether the validated joystick sample carries operator enable."""
    return len(buttons) == EXPECTED_JOY_BUTTONS and buttons[DEADMAN_BUTTON_INDEX] == 1


def validate_joint_state(
    names: Sequence[object],
    positions: Sequence[object],
    allowed_names: Iterable[str],
    max_joints: int,
) -> Tuple[Tuple[str, float], ...]:
    """Validate a named joint command before any target is changed."""
    if len(names) != len(positions):
        raise SafetyValidationError("JointState name and position arrays must have equal length")
    if not 1 <= len(names) <= max_joints:
        raise SafetyValidationError(f"JointState must contain between 1 and {max_joints} joints")

    allowed = frozenset(allowed_names)
    seen = set()
    result = []
    for index, (raw_name, raw_position) in enumerate(zip(names, positions)):
        if not isinstance(raw_name, str) or raw_name not in allowed:
            raise SafetyValidationError(f"JointState name[{index}] is not supported")
        if raw_name in seen:
            raise SafetyValidationError(f"JointState contains duplicate joint '{raw_name}'")
        seen.add(raw_name)
        result.append((raw_name, finite_float(raw_position, f"JointState position[{index}]")))
    return tuple(result)


def validate_servo_limits(
    minimums: Sequence[object],
    maximums: Sequence[object],
    expected_count: int,
) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
    """Return immutable, ordered, finite servo calibrations."""
    if len(minimums) != expected_count or len(maximums) != expected_count:
        raise SafetyValidationError(f"servo limit arrays must each contain exactly {expected_count} values")

    safe_minimums = tuple(finite_float(value, f"servo_min_limits[{index}]") for index, value in enumerate(minimums))
    safe_maximums = tuple(finite_float(value, f"servo_max_limits[{index}]") for index, value in enumerate(maximums))
    for index, (minimum, maximum) in enumerate(zip(safe_minimums, safe_maximums)):
        if not 0.0 <= minimum < maximum <= 100.0:
            raise SafetyValidationError(
                f"servo {index} limits must satisfy 0 <= minimum < maximum <= 100"
            )
    return safe_minimums, safe_maximums


def validate_message_stamp(
    seconds: object,
    nanoseconds: object,
    now_nanoseconds: int,
    previous_nanoseconds: int | None,
    maximum_age_sec: float,
) -> int:
    """Reject missing, replayed, future, or queued command timestamps."""
    if not isinstance(seconds, int) or not isinstance(nanoseconds, int):
        raise SafetyValidationError("message timestamp fields must be integers")
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise SafetyValidationError("message timestamp is invalid")

    stamp = seconds * 1_000_000_000 + nanoseconds
    if stamp == 0:
        raise SafetyValidationError("actuator commands require a non-zero timestamp")
    if previous_nanoseconds is not None and stamp <= previous_nanoseconds:
        raise SafetyValidationError("message timestamp is replayed or out of order")

    age = (int(now_nanoseconds) - stamp) / 1_000_000_000.0
    if age > maximum_age_sec:
        raise SafetyValidationError("message is stale")
    if age < -MAX_FUTURE_COMMAND_SEC:
        raise SafetyValidationError("message timestamp is too far in the future")
    return stamp
