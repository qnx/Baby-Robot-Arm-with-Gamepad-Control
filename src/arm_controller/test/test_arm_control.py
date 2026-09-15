import math
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from arm_control import ArmController, JointNum  # noqa: E402
from safety import SafetyValidationError  # noqa: E402


class FakeLogger:
    def __init__(self):
        self.messages = []

    def _record(self, level, message):
        self.messages.append((level, message))

    def info(self, message):
        self._record("info", message)

    def warn(self, message):
        self._record("warn", message)

    def error(self, message):
        self._record("error", message)

    def debug(self, message):
        self._record("debug", message)


class FakePWM:
    def __init__(self, fail_frequency=False):
        self.calls = []
        self.closed = False
        self.fail_frequency = fail_frequency

    def set_pwm_freq(self, frequency):
        self.calls.append(("frequency", frequency))
        if self.fail_frequency:
            raise OSError("I2C frequency write failed")

    def set_pwm(self, channel, on, off):
        self.calls.append(("pwm", channel, on, off))

    def enable_all_pwm(self):
        self.calls.append(("enable",))

    def disable_channel(self, channel):
        self.calls.append(("disable_channel", channel))

    def disable_all_pwm(self):
        self.calls.append(("disable",))

    def close(self, disable=True):
        self.closed = True


class FakeClock:
    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value

    def sleep(self, duration):
        self.value += duration


class ArmControllerTests(unittest.TestCase):
    def make_controller(self, hook=None):
        self.clock = FakeClock()
        self.pwm = FakePWM()
        self.logger = FakeLogger()
        limits_min = [25.0, 0.0, 0.0, 0.0, 0.0, 15.0]
        limits_max = [75.0, 50.0, 100.0, 100.0, 100.0, 65.0]
        return ArmController(
            self.logger,
            limits_min,
            limits_max,
            pwm_factory=lambda: self.pwm,
            hardware_output_hook=hook,
            allow_software_only_output=hook is None,
            monotonic=self.clock,
            sleep=self.clock.sleep,
        )

    def test_starts_disarmed_and_requires_explicit_arm(self):
        controller = self.make_controller()
        self.assertFalse(controller.is_armed)
        with self.assertRaises(RuntimeError):
            controller.set_joint(JointNum.BASE, 50.0)
        controller.update()
        self.assertFalse(any(call[0] == "enable" for call in self.pwm.calls))

    def test_production_default_refuses_arm_without_hardware_output_hook(self):
        clock = FakeClock()
        pwm = FakePWM()
        controller = ArmController(
            FakeLogger(),
            [25.0, 0.0, 0.0, 0.0, 0.0, 15.0],
            [75.0, 50.0, 100.0, 100.0, 100.0, 65.0],
            pwm_factory=lambda: pwm,
            monotonic=clock,
            sleep=clock.sleep,
        )
        with self.assertRaises(RuntimeError):
            controller.arm()
        self.assertFalse(controller.is_armed)

    def test_atomic_joint_command_rejects_nan_without_partial_update(self):
        controller = self.make_controller()
        controller.arm()
        before = tuple(joint.target for joint in controller._joints)
        with self.assertRaises(SafetyValidationError):
            controller.set_joint_targets_rad_atomic(
                ((JointNum.BASE, 0.0), (JointNum.ELBOW, math.nan))
            )
        self.assertEqual(before, tuple(joint.target for joint in controller._joints))

    def test_calibrated_limit_and_target_step_cannot_be_bypassed(self):
        controller = self.make_controller()
        controller.arm()
        with self.assertRaises(SafetyValidationError):
            controller.set_joint_rad(JointNum.BASE, controller._joints[0].max_rad)
        with self.assertRaises(SafetyValidationError):
            controller.set_joint_rad(JointNum.BASE, controller._joints[0].max_rad / 2.0)
        with self.assertRaises(SafetyValidationError):
            controller.enable_position_limits(False)

    def test_output_hook_is_disabled_first_and_on_fault(self):
        hook_events = []
        controller = self.make_controller(hook_events.append)
        self.assertEqual(hook_events, [False])
        controller.arm()
        controller.update()
        self.assertEqual(hook_events[-1], True)
        controller.fault_stop("test")
        self.assertEqual(hook_events[-1], False)
        self.assertFalse(controller.is_armed)

    def test_fault_stop_freezes_pose_and_drops_stale_target_before_rearm(self):
        controller = self.make_controller()
        controller.arm()
        controller.set_joint(JointNum.BASE, 75.0)
        controller.update()

        base = controller._joints[JointNum.BASE.value]
        frozen_position = base.current
        self.assertIsNotNone(frozen_position)
        self.assertNotEqual(base.target, frozen_position)

        controller.fault_stop("test fault")
        self.assertEqual(base.target, frozen_position)
        self.assertEqual(
            tuple(joint.target for joint in controller._joints),
            tuple(joint.current for joint in controller._joints),
        )

        calls_before_rearm = len(self.pwm.calls)
        controller.arm()
        controller.update()
        self.assertEqual(base.current, frozen_position)
        self.assertEqual(base.target, frozen_position)

        preload_calls = [
            call for call in self.pwm.calls[calls_before_rearm:] if call[0] == "pwm"
        ]
        self.assertEqual(len(preload_calls), len(controller._joints))
        expected_pwm = int(
            base.min_pulse
            + (frozen_position / 100.0) * (base.max_pulse - base.min_pulse)
        )
        self.assertEqual(preload_calls[JointNum.BASE.value], ("pwm", 0, 500, expected_pwm))

    def test_live_target_freeze_stops_prior_mode_slew(self):
        controller = self.make_controller()
        controller.arm()
        controller.set_joint(JointNum.BASE, 75.0)
        controller.update()
        base = controller._joints[JointNum.BASE.value]
        frozen_position = base.current

        controller.freeze_targets_at_current()
        controller.update()

        self.assertTrue(controller.is_armed)
        self.assertEqual(base.target, frozen_position)
        self.assertEqual(base.current, frozen_position)

    def test_unmanaged_channels_are_disabled_before_global_enable(self):
        controller = self.make_controller()
        controller.arm()
        controller.update()
        enable_index = self.pwm.calls.index(("enable",))
        for channel in range(6, 16):
            self.assertIn(("disable_channel", channel), self.pwm.calls[:enable_index])

    def test_internal_trajectory_is_constrained_without_weakening_external_rejection(self):
        controller = self.make_controller()
        self.assertEqual(controller.constrain_internal_target_percent(JointNum.WRIST, -5.0), 0.0)
        self.assertEqual(controller.constrain_internal_target_percent(JointNum.WRIST, 120.0), 100.0)
        controller.arm()
        with self.assertRaises(SafetyValidationError):
            controller.set_joint_targets_percent_atomic(((JointNum.WRIST, -5.0),))

    def test_command_age_uses_injected_monotonic_clock(self):
        controller = self.make_controller()
        controller.arm()
        self.clock.value += 0.4
        self.assertAlmostEqual(controller.command_age(), 0.4)

    def test_queued_command_keeps_only_its_remaining_lease(self):
        controller = self.make_controller()
        controller.arm()
        controller.note_command_activity(0.4)
        self.assertAlmostEqual(controller.command_age(), 0.4)
        with self.assertRaises(SafetyValidationError):
            controller.note_command_activity(-0.1)

    def test_parking_is_bounded_and_fault_stops_on_timeout(self):
        controller = self.make_controller()
        controller.arm()
        controller.set_joint(JointNum.BASE, 75.0)
        controller.update()
        self.assertFalse(controller.park_and_stop(0.1))
        self.assertFalse(controller.is_armed)
        self.assertEqual(controller.fault_reason, "parking timed out")

    def test_partial_pwm_initialization_is_disabled_and_closed(self):
        pwm = FakePWM(fail_frequency=True)
        with self.assertRaises(OSError):
            ArmController(
                FakeLogger(),
                [0.0] * 6,
                [100.0] * 6,
                pwm_factory=lambda: pwm,
            )
        self.assertIn(("disable",), pwm.calls)
        self.assertTrue(pwm.closed)


if __name__ == "__main__":
    unittest.main()
