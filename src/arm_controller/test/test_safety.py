import math
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from safety import (  # noqa: E402
    MAX_IK_SESSION_EPOCH,
    SafetyValidationError,
    deadman_is_held,
    requested_arm_action,
    requested_control_mode,
    validate_joy_message,
    validate_ik_session_epoch,
    validate_joint_state,
    validate_message_stamp,
    validate_servo_limits,
)


class SafetyValidationTests(unittest.TestCase):
    def test_ik_session_epoch_is_positive_integral_and_float_exact(self):
        self.assertEqual(validate_ik_session_epoch(1), 1)
        self.assertEqual(
            validate_ik_session_epoch(MAX_IK_SESSION_EPOCH),
            MAX_IK_SESSION_EPOCH,
        )
        for invalid in (False, 0, -1, 1.0, MAX_IK_SESSION_EPOCH + 1):
            with self.subTest(invalid=invalid):
                with self.assertRaises(SafetyValidationError):
                    validate_ik_session_epoch(invalid)

    def test_joy_schema_is_exact_and_finite(self):
        axes, buttons = validate_joy_message([0.0] * 6, [0] * 12)
        self.assertEqual(len(axes), 6)
        self.assertEqual(len(buttons), 12)
        with self.assertRaises(SafetyValidationError):
            validate_joy_message([0.0] * 5, [0] * 12)
        with self.assertRaises(SafetyValidationError):
            validate_joy_message([0.0, 0.0, math.nan, 0.0, 0.0, 0.0], [0] * 12)
        with self.assertRaises(SafetyValidationError):
            validate_joy_message([0.0] * 6, [0] * 11 + [2])

    def test_joint_state_rejects_shape_duplicates_and_nan(self):
        allowed = ("base", "shoulder")
        self.assertEqual(
            validate_joint_state(["base"], [0.0], allowed, 2),
            (("base", 0.0),),
        )
        with self.assertRaises(SafetyValidationError):
            validate_joint_state(["base"], [], allowed, 2)
        with self.assertRaises(SafetyValidationError):
            validate_joint_state(["base", "base"], [0.0, 0.0], allowed, 2)
        with self.assertRaises(SafetyValidationError):
            validate_joint_state(["base"], [math.nan], allowed, 2)

    def test_servo_limits_are_ordered_and_immutable(self):
        minimums, maximums = validate_servo_limits([0] * 6, [100] * 6, 6)
        self.assertIsInstance(minimums, tuple)
        self.assertIsInstance(maximums, tuple)
        with self.assertRaises(SafetyValidationError):
            validate_servo_limits([0] * 5, [100] * 6, 6)
        with self.assertRaises(SafetyValidationError):
            validate_servo_limits([0, 0, 0, 0, 0, 60], [100, 100, 100, 100, 100, 50], 6)

    def test_timestamp_rejects_stale_future_and_replay(self):
        now = 10_000_000_000
        accepted = validate_message_stamp(9, 900_000_000, now, None, 0.5)
        self.assertEqual(accepted, 9_900_000_000)
        with self.assertRaises(SafetyValidationError):
            validate_message_stamp(9, 900_000_000, now, accepted, 0.5)
        with self.assertRaises(SafetyValidationError):
            validate_message_stamp(9, 0, now, None, 0.5)
        with self.assertRaises(SafetyValidationError):
            validate_message_stamp(10, 200_000_000, now, None, 0.5)

    def test_ik_is_opt_in_but_direct_joint_mode_remains_available(self):
        buttons = [0] * 12
        buttons[8] = 1  # Select
        buttons[10] = 1  # Held L3 operator enable
        buttons[1] = 1  # B / IK
        with self.assertRaises(SafetyValidationError):
            requested_control_mode(buttons, enable_ik_mode=False)
        self.assertEqual(requested_control_mode(buttons, enable_ik_mode=True), "ik")

        buttons[1] = 0
        buttons[2] = 1  # X / direct joint
        self.assertEqual(requested_control_mode(buttons, enable_ik_mode=False), "joint")

    def test_mode_chord_rejects_unrelated_held_buttons(self):
        for extra_button in (3, 4, 11):
            buttons = [0] * 12
            for index in (0, 8, 10, extra_button):
                buttons[index] = 1
            with self.subTest(extra_button=extra_button):
                with self.assertRaises(SafetyValidationError):
                    requested_control_mode(buttons, enable_ik_mode=False)

    def test_arming_requires_exact_neutral_chord(self):
        buttons = [0] * 12
        buttons[0] = buttons[8] = buttons[9] = buttons[10] = 1
        self.assertEqual(requested_arm_action(buttons, [0.0] * 6, 0.08), "arm")
        self.assertTrue(deadman_is_held(buttons))
        buttons[0] = 0
        buttons[1] = 1
        self.assertEqual(requested_arm_action(buttons, [0.0] * 6, 0.08), "disarm")
        buttons[4] = 1
        with self.assertRaises(SafetyValidationError):
            requested_arm_action(buttons, [0.0] * 6, 0.08)
        buttons[4] = 0
        with self.assertRaises(SafetyValidationError):
            requested_arm_action(buttons, [0.2] + [0.0] * 5, 0.08)

        buttons = [0] * 12
        buttons[0] = buttons[8] = buttons[9] = 1
        with self.assertRaises(SafetyValidationError):
            requested_arm_action(buttons, [0.0] * 6, 0.08)
        self.assertFalse(deadman_is_held(buttons))


if __name__ == "__main__":
    unittest.main()
