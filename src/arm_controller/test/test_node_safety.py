from pathlib import Path
import sys
import time
import types
import unittest


SRC = str(Path(__file__).resolve().parents[1] / "src")
sys.path.insert(0, SRC)


def module(name):
    value = sys.modules.get(name)
    if value is None:
        value = types.ModuleType(name)
        sys.modules[name] = value
    return value


# Import the node logic without requiring a host ROS installation.
rclpy = module("rclpy")
rclpy.try_shutdown = lambda: None
rclpy.init = lambda args=None, signal_handler_options=None: None
rclpy.spin = lambda node: None
rclpy_node = module("rclpy.node")
rclpy_node.Node = type("Node", (), {})
rclpy.node = rclpy_node
rclpy_impl = module("rclpy.impl")
rclpy_logger = module("rclpy.impl.rcutils_logger")
rclpy_logger.RcutilsLogger = type("RcutilsLogger", (), {})
rclpy_impl.rcutils_logger = rclpy_logger
rclpy.impl = rclpy_impl
rclpy_signals = module("rclpy.signals")
rclpy_signals.SignalHandlerOptions = type("SignalHandlerOptions", (), {"ALL": object()})
rclpy.signals = rclpy_signals

rcl_interfaces = module("rcl_interfaces")
rcl_interfaces_msg = module("rcl_interfaces.msg")
rcl_interfaces_msg.ParameterDescriptor = type("ParameterDescriptor", (), {})
rcl_interfaces.msg = rcl_interfaces_msg

sensor_msgs = module("sensor_msgs")
sensor_msgs_msg = module("sensor_msgs.msg")
sensor_msgs_msg.Joy = getattr(sensor_msgs_msg, "Joy", type("Joy", (), {}))
sensor_msgs_msg.JointState = getattr(sensor_msgs_msg, "JointState", type("JointState", (), {}))
sensor_msgs.msg = sensor_msgs_msg

std_msgs = module("std_msgs")
std_msgs_msg = module("std_msgs.msg")
std_msgs_msg.Float64MultiArray = getattr(
    std_msgs_msg, "Float64MultiArray", type("Float64MultiArray", (), {})
)
std_msgs_msg.String = type("String", (), {})
std_msgs_msg.UInt64 = type("UInt64", (), {})
std_msgs.msg = std_msgs_msg

from arm_controller_node import ArmControlMode, ArmControllerNode  # noqa: E402
from arm_controller_input import ScreenSaverDance  # noqa: E402
from safety import MAX_IK_SESSION_EPOCH, SafetyValidationError  # noqa: E402


class FakeLogger:
    def __init__(self):
        self.messages = []

    def _add(self, level, message):
        self.messages.append((level, message))

    def info(self, message):
        self._add("info", message)

    def warn(self, message):
        self._add("warn", message)

    def error(self, message):
        self._add("error", message)


class RecordingPublisher:
    def __init__(self):
        self.values = []

    def publish(self, message):
        self.values.append(message.data)


class FakeController:
    def __init__(self):
        self.is_armed = False
        self.notes = []
        self.faults = []
        self.freezes = 0
        self.age = 0.0

    def arm(self):
        self.is_armed = True

    def note_command_activity(self, age=0.0):
        self.notes.append(age)
        self.age = age

    def fault_stop(self, reason):
        self.faults.append(reason)
        self.is_armed = False
        # Match ArmController.fault_stop(), which atomically drops every queued
        # trajectory as part of disabling output.
        self.freeze_targets_at_current()

    def command_age(self, _now=None):
        return self.age

    def freeze_targets_at_current(self):
        self.freezes += 1

    def enable_position_limits(self, enabled):
        if not enabled:
            raise AssertionError("tests must not disable calibrated limits")

    def update(self):
        pass


class FakeInput:
    def __init__(self):
        self.messages = []
        self.joint_messages = []
        self.mov_messages = []
        self.focus_count = 0
        self.session_epoch = None
        self.joint_map = {
            "base": 0,
            "shoulder": 1,
            "elbow": 2,
            "wrist": 3,
            "hand": 4,
            "gripper": 5,
        }

    def joy_callback(self, message):
        self.messages.append(message)

    def update(self):
        pass

    def focus(self):
        self.focus_count += 1

    def joint_callback(self, message):
        self.joint_messages.append(message)

    def mov_callback(self, message):
        self.mov_messages.append(message)

    def begin_session(self, epoch):
        self.session_epoch = epoch

    def clear_session(self):
        self.session_epoch = None


class Message:
    def __init__(
        self,
        buttons=None,
        axes=None,
        age=0.0,
        names=None,
        positions=None,
        stamp_ns=1,
        frame_id="",
    ):
        self.buttons = [0] * 12 if buttons is None else list(buttons)
        self.axes = [0.0] * 6 if axes is None else list(axes)
        self.transport_age = age
        self.name = [] if names is None else list(names)
        self.position = [] if positions is None else list(positions)
        self.stamp_ns = stamp_ns
        self.header = type(
            "Header",
            (),
            {
                "frame_id": frame_id,
                "stamp": type(
                    "Stamp",
                    (),
                    {
                        "sec": stamp_ns // 1_000_000_000,
                        "nanosec": stamp_ns % 1_000_000_000,
                    },
                )(),
            },
        )()


class NodeSafetyBoundaryTests(unittest.TestCase):
    def make_node(self):
        node = object.__new__(ArmControllerNode)
        node.controller = FakeController()
        node._last_message_stamps = {}
        node._last_arm_action = None
        node._joint_mode_grant_ns = None
        node._joint_epoch_counter = 0
        node._active_joint_epoch = None
        node._ik_epoch_counter = 0
        node._active_ik_epoch = None
        node._rejection_log_times = {}
        node._autonomous_started_at = None
        node._autonomous_lease_at = None
        node.screensaver_enabled = False
        node.command_timeout_sec = 0.5
        node.autonomous_lease_sec = 0.35
        node.autonomous_max_duration_sec = 30.0
        node.enable_ik_mode = False
        node.enable_autonomous_mode = False
        node.control_mode = ArmControlMode.JOYSTICK
        node.active_input = FakeInput()
        node.input_joystick = node.active_input
        node.input_joint = FakeInput()
        node.input_ik = FakeInput()
        node.input_screensaver = FakeInput()
        node.mode_publisher = type("Publisher", (), {"publish": lambda self, msg: None})()
        node.joint_session_publisher = RecordingPublisher()
        node.get_logger = lambda: FakeLogger()
        node._publish_mode = lambda: None
        node._receipt_ns = 100
        node._controller_receipt_time_ns = lambda: node._receipt_ns
        node._validated_stamp = lambda message, channel, ros_now_ns=None: (
            message.stamp_ns,
            message.transport_age,
        )
        return node

    def test_exact_arm_then_held_l3_renews_remaining_lease(self):
        node = self.make_node()
        buttons = [0] * 12
        for index in (0, 8, 9, 10):
            buttons[index] = 1
        node.joy_callback(Message(buttons=buttons, age=0.2))
        self.assertTrue(node.controller.is_armed)
        self.assertEqual(node.controller.notes, [0.2])

        held = [0] * 12
        held[10] = 1
        node.joy_callback(Message(buttons=held, age=0.1))
        self.assertEqual(node.controller.notes, [0.2, 0.1])

    def test_deadman_release_neutral_faults_before_renewal(self):
        node = self.make_node()
        node.controller.is_armed = True
        node.joy_callback(Message())
        self.assertFalse(node.controller.is_armed)
        self.assertEqual(node.controller.notes, [])
        self.assertEqual(node.controller.faults, ["operator dead-man released"])

    def test_deadman_stop_stamp_cannot_be_replayed_as_arm_chord(self):
        node = self.make_node()
        node._validated_stamp = types.MethodType(ArmControllerNode._validated_stamp, node)
        node.controller.is_armed = True
        node._last_message_stamps["joy"] = 10

        node.joy_callback(Message(buttons=[0] * 12, stamp_ns=20))
        self.assertFalse(node.controller.is_armed)
        self.assertEqual(node._last_message_stamps["joy"], 20)

        arm_buttons = [0] * 12
        for index in (0, 8, 9, 10):
            arm_buttons[index] = 1
        node.joy_callback(Message(buttons=arm_buttons, stamp_ns=20))

        self.assertFalse(node.controller.is_armed)
        self.assertEqual(node.controller.notes, [])
        self.assertEqual(node.controller.faults, ["operator dead-man released"])

    def test_deadman_release_preempts_invalid_arm_chords(self):
        # A release must fault-stop before arm-chord validation can reject an
        # ambiguous or non-neutral sample and postpone the stop until timeout.
        cases = (
            ({0, 8, 9}, [0.0] * 6),
            ({0, 2, 8, 9}, [0.0] * 6),
            ({0, 8, 9}, [0.25, 0.0, 0.0, 0.0, 0.0, 0.0]),
        )
        for pressed, axes in cases:
            with self.subTest(pressed=pressed, axes=axes):
                node = self.make_node()
                node.controller.is_armed = True
                buttons = [int(index in pressed) for index in range(12)]

                node.joy_callback(Message(buttons=buttons, axes=axes))

                self.assertFalse(node.controller.is_armed)
                self.assertEqual(node.controller.notes, [])
                self.assertEqual(node.controller.faults, ["operator dead-man released"])

    def test_deadman_release_does_not_depend_on_command_timestamp(self):
        node = self.make_node()
        node.controller.is_armed = True
        node._validated_stamp = lambda message, channel, ros_now_ns=None: (_ for _ in ()).throw(
            SafetyValidationError("clock rollback")
        )
        ambiguous = [0] * 12
        for index in (0, 8, 9):
            ambiguous[index] = 1

        node.joy_callback(Message(buttons=ambiguous))

        self.assertFalse(node.controller.is_armed)
        self.assertEqual(node.controller.notes, [])
        self.assertEqual(node.controller.faults, ["operator dead-man released"])

    def test_stale_joy_does_not_renew_authority(self):
        node = self.make_node()
        node.controller.is_armed = True
        node._validated_stamp = lambda message, channel, ros_now_ns=None: (_ for _ in ()).throw(
            SafetyValidationError("stale")
        )
        held = [0] * 12
        held[10] = 1
        node.joy_callback(Message(buttons=held))
        self.assertEqual(node.controller.notes, [])
        self.assertFalse(node.controller.is_armed)
        self.assertEqual(node.controller.faults, ["invalid joystick input: stale"])

    def test_malformed_joy_fault_stops_armed_output(self):
        cases = (
            Message(buttons=[0] * 11),
            Message(axes=[float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0]),
        )
        for message in cases:
            with self.subTest(button_count=len(message.buttons), axes=message.axes):
                node = self.make_node()
                node.controller.is_armed = True

                node.joy_callback(message)

                self.assertFalse(node.controller.is_armed)
                self.assertEqual(node.controller.notes, [])
                self.assertTrue(node.controller.faults[0].startswith("invalid joystick input:"))

    def test_malformed_stop_stamp_cannot_be_replayed_as_arm_chord(self):
        node = self.make_node()
        node._validated_stamp = types.MethodType(ArmControllerNode._validated_stamp, node)
        node.controller.is_armed = True
        node._last_message_stamps["joy"] = 10

        node.joy_callback(Message(buttons=[0] * 11, stamp_ns=20))
        self.assertFalse(node.controller.is_armed)
        self.assertEqual(node._last_message_stamps["joy"], 20)

        arm_buttons = [0] * 12
        for index in (0, 8, 9, 10):
            arm_buttons[index] = 1
        node.joy_callback(Message(buttons=arm_buttons, stamp_ns=20))

        self.assertFalse(node.controller.is_armed)
        self.assertEqual(node.controller.notes, [])
        self.assertTrue(node.controller.faults[0].startswith("invalid joystick input:"))

    def test_generic_prestamp_fault_consumes_valid_stop_stamp(self):
        class ExplodingAxes:
            def __len__(self):
                return 6

            def __iter__(self):
                raise RuntimeError("synthetic pre-stamp callback failure")

        node = self.make_node()
        node._validated_stamp = types.MethodType(ArmControllerNode._validated_stamp, node)
        node.controller.is_armed = True
        node._last_message_stamps["joy"] = 10
        broken = Message(stamp_ns=20)
        broken.axes = ExplodingAxes()

        node.joy_callback(broken)

        self.assertFalse(node.controller.is_armed)
        self.assertEqual(node._last_message_stamps["joy"], 20)
        self.assertTrue(node.controller.faults[0].startswith("joystick callback failed:"))

    def test_missing_reports_expire_lease(self):
        node = self.make_node()
        node.controller.is_armed = True
        node.controller.age = 0.51
        node.smoothing_loop()
        self.assertEqual(node.controller.faults, ["command lease expired"])

    def test_ordinary_held_deadman_cannot_resurrect_expired_lease(self):
        node = self.make_node()
        node.controller.is_armed = True
        node.controller.age = 0.51
        held = [0] * 12
        held[10] = 1

        node.joy_callback(Message(buttons=held, stamp_ns=10))

        self.assertFalse(node.controller.is_armed)
        self.assertEqual(node.controller.notes, [])
        self.assertEqual(node.controller.faults, ["command lease expired"])
        self.assertEqual(node.controller.freezes, 1)
        self.assertEqual(node.input_joystick.messages, [])
        self.assertEqual(node._last_message_stamps["joy"], 10)

    def test_expired_sample_stamp_cannot_be_replayed_as_arm_chord(self):
        node = self.make_node()
        node.controller.is_armed = True
        node.controller.age = 0.51

        def validate_once(message, channel, ros_now_ns=None):
            previous = node._last_message_stamps.get(channel)
            if previous is not None and message.stamp_ns <= previous:
                raise SafetyValidationError("replayed timestamp")
            return message.stamp_ns, message.transport_age

        node._validated_stamp = validate_once
        held = [0] * 12
        held[10] = 1
        node.joy_callback(Message(buttons=held, stamp_ns=10))
        self.assertFalse(node.controller.is_armed)

        arm_buttons = [0] * 12
        for index in (0, 8, 9, 10):
            arm_buttons[index] = 1
        node.joy_callback(Message(buttons=arm_buttons, stamp_ns=10))

        self.assertFalse(node.controller.is_armed)
        self.assertEqual(node.controller.notes, [])
        self.assertEqual(node.controller.faults, ["command lease expired"])

    def test_expired_direct_joint_command_cannot_mutate_targets(self):
        node = self.make_node()
        node.controller.is_armed = True
        node.controller.age = 0.51
        node.control_mode = ArmControlMode.JOINT
        node.active_input = node.input_joint
        node._joint_mode_grant_ns = 100
        node._receipt_ns = 200

        node.joint_callback(
            Message(names=["base"], positions=[0.0], stamp_ns=150)
        )

        self.assertFalse(node.controller.is_armed)
        self.assertEqual(node.controller.faults, ["command lease expired"])
        self.assertEqual(node.controller.freezes, 1)
        self.assertEqual(node.input_joint.joint_messages, [])
        self.assertNotIn("joint", node._last_message_stamps)

    def test_expired_ik_output_cannot_mutate_targets(self):
        node = self.make_node()
        node.controller.is_armed = True
        node.controller.age = 0.51
        node.control_mode = ArmControlMode.IK
        node.active_input = node.input_ik
        node._ik_epoch_counter = 7
        node._active_ik_epoch = 7
        node.input_ik.begin_session(7)

        node.mov_callback(
            Message(
                names=list(("base", "shoulder", "elbow", "wrist", "hand")),
                positions=[0.0] * 5,
                stamp_ns=50,
                frame_id="ik-session:7",
            )
        )

        self.assertFalse(node.controller.is_armed)
        self.assertEqual(node.controller.faults, ["command lease expired"])
        self.assertEqual(node.controller.freezes, 1)
        self.assertEqual(node.input_ik.mov_messages, [])
        self.assertNotIn("mov", node._last_message_stamps)

    def test_exact_arm_chord_establishes_new_frozen_grant_after_expiry(self):
        node = self.make_node()
        node.controller.is_armed = True
        node.controller.age = 0.51
        node.control_mode = ArmControlMode.JOINT
        node.active_input = node.input_joint
        node._joint_mode_grant_ns = 50
        node._joint_epoch_counter = 3
        node._active_joint_epoch = 3
        held = [0] * 12
        held[10] = 1

        node.joy_callback(Message(buttons=held, stamp_ns=90))
        self.assertFalse(node.controller.is_armed)
        self.assertEqual(node.controller.freezes, 1)

        arm_buttons = [0] * 12
        for index in (0, 8, 9, 10):
            arm_buttons[index] = 1
        node._receipt_ns = 200
        node.joy_callback(Message(buttons=arm_buttons, stamp_ns=200))

        self.assertTrue(node.controller.is_armed)
        self.assertEqual(node._joint_mode_grant_ns, 200)
        self.assertEqual(node._active_joint_epoch, 4)
        self.assertEqual(node.joint_session_publisher.values[-2:], [0, 4])
        self.assertEqual(node.controller.freezes, 2)
        self.assertEqual(node.controller.notes, [0.0])
        self.assertEqual(node._last_message_stamps["joy"], 200)

    def test_repeated_dance_chord_cannot_renew_expired_autonomous_authority(self):
        for started_at, lease_at, boundary in (
            (99.0, 99.64, "renewal lease"),
            (69.99, 99.9, "maximum duration"),
        ):
            with self.subTest(boundary=boundary):
                node = self.make_node()
                node.controller.is_armed = True
                node.controller.age = 0.1
                node.enable_autonomous_mode = True
                node.screensaver_enabled = True
                node._autonomous_started_at = started_at
                node._autonomous_lease_at = lease_at
                node.input_screensaver.dance = ScreenSaverDance.WAVE
                wave = [0] * 12
                for index in (1, 9, 10):
                    wave[index] = 1

                original_monotonic = time.monotonic
                try:
                    time.monotonic = lambda: 100.0
                    node.joy_callback(Message(buttons=wave, stamp_ns=10))
                finally:
                    time.monotonic = original_monotonic

                self.assertFalse(node.controller.is_armed)
                self.assertFalse(node.screensaver_enabled)
                self.assertEqual(node.controller.notes, [])
                self.assertEqual(
                    node.controller.faults,
                    ["autonomous motion lease expired"],
                )
                self.assertEqual(node.controller.freezes, 1)

    def test_autonomous_request_is_disabled_by_default(self):
        node = self.make_node()
        node.controller.is_armed = True
        buttons = [0] * 12
        for index in (0, 9, 10):
            buttons[index] = 1
        node.joy_callback(Message(buttons=buttons))
        self.assertFalse(node.screensaver_enabled)

    def test_autonomous_mapping_and_chord_are_exact(self):
        wave = [0] * 12
        for index in (1, 9, 10):
            wave[index] = 1
        self.assertEqual(
            ArmControllerNode._requested_dance(wave),
            ScreenSaverDance.WAVE,
        )

        node = self.make_node()
        node.controller.is_armed = True
        node.enable_autonomous_mode = True
        wave[5] = 1
        node.joy_callback(Message(buttons=wave))
        self.assertFalse(node.controller.is_armed)
        self.assertTrue(node.controller.faults[0].startswith("invalid joystick input:"))

    def test_autonomous_cancel_freezes_previous_trajectory(self):
        node = self.make_node()
        node.controller.is_armed = True
        node.screensaver_enabled = True
        held = [0] * 12
        held[10] = 1

        node.joy_callback(Message(buttons=held))

        self.assertFalse(node.screensaver_enabled)
        self.assertEqual(node.controller.freezes, 1)

    def test_arm_chord_drops_autonomous_target_even_in_joystick_mode(self):
        node = self.make_node()
        node.controller.is_armed = True
        node.screensaver_enabled = True
        arm_buttons = [0] * 12
        for index in (0, 8, 9, 10):
            arm_buttons[index] = 1

        node.joy_callback(Message(buttons=arm_buttons))

        self.assertFalse(node.screensaver_enabled)
        self.assertEqual(node.controller.freezes, 1)

    def test_mode_switch_freezes_previous_mode_target(self):
        node = self.make_node()
        node.controller.is_armed = True
        buttons = [0] * 12
        for index in (2, 8, 10):
            buttons[index] = 1

        node.joy_callback(Message(buttons=buttons))

        self.assertEqual(node.control_mode, ArmControlMode.JOINT)
        self.assertEqual(node.controller.freezes, 1)

    def test_direct_joint_grant_uses_receipt_time_not_mode_message_stamp(self):
        node = self.make_node()
        node.controller.is_armed = True
        mode_buttons = [0] * 12
        for index in (2, 8, 10):
            mode_buttons[index] = 1

        # This JointState was sourced after the old Joy source timestamp but
        # before the controller received the mode grant at local time 100.
        node.joy_callback(Message(buttons=mode_buttons, stamp_ns=10))
        self.assertEqual(node._joint_mode_grant_ns, 100)
        self.assertEqual(node._active_joint_epoch, 1)
        node._receipt_ns = 110
        queued = Message(
            names=["base"],
            positions=[0.0],
            stamp_ns=50,
            frame_id="joint-session:1",
        )
        node.joint_callback(queued)

        self.assertEqual(node.input_joint.joint_messages, [])

    def test_direct_joint_stamp_must_be_after_grant_and_not_after_receipt(self):
        cases = ((100, False), (101, True), (201, False))
        for stamp_ns, accepted in cases:
            with self.subTest(stamp_ns=stamp_ns):
                node = self.make_node()
                node.controller.is_armed = True
                node.control_mode = ArmControlMode.JOINT
                node.active_input = node.input_joint
                node._joint_mode_grant_ns = 100
                node._joint_epoch_counter = 7
                node._active_joint_epoch = 7
                node._receipt_ns = 200

                node.joint_callback(
                    Message(
                        names=["base"],
                        positions=[0.0],
                        stamp_ns=stamp_ns,
                        frame_id="joint-session:7",
                    )
                )

                self.assertEqual(bool(node.input_joint.joint_messages), accepted)

    def test_rearm_in_joint_mode_rotates_grant_and_drops_queued_command(self):
        node = self.make_node()
        node.controller.is_armed = True
        node.control_mode = ArmControlMode.JOINT
        node.active_input = node.input_joint
        node._joint_mode_grant_ns = 50
        node._joint_epoch_counter = 4
        node._active_joint_epoch = 4
        arm_buttons = [0] * 12
        for index in (0, 8, 9, 10):
            arm_buttons[index] = 1

        node._receipt_ns = 100
        node.joy_callback(Message(buttons=arm_buttons, stamp_ns=90))
        self.assertEqual(node._joint_mode_grant_ns, 100)
        self.assertEqual(node._active_joint_epoch, 5)
        self.assertEqual(node.controller.freezes, 1)
        node._receipt_ns = 110
        node.joint_callback(
            Message(
                names=["base"],
                positions=[0.0],
                stamp_ns=105,
                frame_id="joint-session:4",
            )
        )

        self.assertEqual(node.input_joint.joint_messages, [])

    def test_direct_joint_session_rejects_delayed_pregrant_future_stamp(self):
        node = self.make_node()
        node.controller.is_armed = True
        node.control_mode = ArmControlMode.JOINT
        node.active_input = node.input_joint
        node._joint_mode_grant_ns = 100
        node._joint_epoch_counter = 2
        node._active_joint_epoch = 2
        node._receipt_ns = 200

        # The source timestamp appears post-grant because its clock was ahead,
        # but the sample was produced under the previous mode grant. The epoch,
        # not a cross-clock timestamp comparison, supplies happened-after.
        node.joint_callback(
            Message(
                names=["base"],
                positions=[0.0],
                stamp_ns=150,
                frame_id="joint-session:1",
            )
        )

        self.assertEqual(node.input_joint.joint_messages, [])

    def test_direct_joint_session_frame_is_exact_and_current(self):
        for frame_id, accepted in (
            ("", False),
            ("joint-session:6", False),
            ("joint-session:07", False),
            ("joint-session:8", False),
            ("joint-session:7", True),
        ):
            with self.subTest(frame_id=frame_id):
                node = self.make_node()
                node.controller.is_armed = True
                node.control_mode = ArmControlMode.JOINT
                node.active_input = node.input_joint
                node._joint_mode_grant_ns = 100
                node._joint_epoch_counter = 7
                node._active_joint_epoch = 7
                node._receipt_ns = 200

                node.joint_callback(
                    Message(
                        names=["base"],
                        positions=[0.0],
                        stamp_ns=150,
                        frame_id=frame_id,
                    )
                )

                self.assertEqual(bool(node.input_joint.joint_messages), accepted)

    def test_direct_joint_entry_leave_and_reentry_rotate_published_session(self):
        node = self.make_node()
        node.controller.is_armed = True

        node._update_mode(ArmControlMode.JOINT, grant_receipt_ns=100)
        self.assertEqual(node._active_joint_epoch, 1)
        self.assertEqual(node.joint_session_publisher.values[-1], 1)

        node._update_mode(ArmControlMode.JOYSTICK, grant_receipt_ns=101)
        self.assertIsNone(node._active_joint_epoch)
        self.assertEqual(node.joint_session_publisher.values[-1], 0)

        node._update_mode(ArmControlMode.JOINT, grant_receipt_ns=102)
        self.assertEqual(node._active_joint_epoch, 2)
        self.assertEqual(node.joint_session_publisher.values[-1], 2)

    def test_fault_stop_revokes_direct_joint_session_before_rearm(self):
        node = self.make_node()
        node.controller.is_armed = True
        node.control_mode = ArmControlMode.JOINT
        node.active_input = node.input_joint
        node._joint_mode_grant_ns = 100
        node._joint_epoch_counter = 7
        node._active_joint_epoch = 7

        # Releasing the held dead-man is an immediate output fault and must also
        # tell the intended producer to stop using its now-invalid grant.
        node.joy_callback(Message(buttons=[0] * 12, stamp_ns=150))

        self.assertFalse(node.controller.is_armed)
        self.assertIsNone(node._joint_mode_grant_ns)
        self.assertIsNone(node._active_joint_epoch)
        self.assertEqual(node.joint_session_publisher.values[-1], 0)

        arm_buttons = [0] * 12
        for index in (0, 8, 9, 10):
            arm_buttons[index] = 1
        node._receipt_ns = 200
        node.joy_callback(Message(buttons=arm_buttons, stamp_ns=200))

        self.assertTrue(node.controller.is_armed)
        self.assertEqual(node._active_joint_epoch, 8)
        self.assertEqual(node.joint_session_publisher.values[-1], 8)

    def test_direct_joint_epoch_exhaustion_revokes_and_fails_closed(self):
        node = self.make_node()
        node.controller.is_armed = True
        node.control_mode = ArmControlMode.JOINT
        node.active_input = node.input_joint
        node._joint_epoch_counter = MAX_IK_SESSION_EPOCH
        node._active_joint_epoch = MAX_IK_SESSION_EPOCH

        with self.assertRaisesRegex(SafetyValidationError, "epoch space is exhausted"):
            node._update_mode(ArmControlMode.JOINT, force=True, grant_receipt_ns=100)

        self.assertIsNone(node._active_joint_epoch)
        self.assertEqual(node.joint_session_publisher.values[-1], 0)

    def test_ik_entry_force_reset_and_same_mode_rearm_rotate_epoch(self):
        node = self.make_node()
        node.controller.is_armed = True

        node._update_mode(ArmControlMode.IK, grant_receipt_ns=100)
        self.assertEqual(node._active_ik_epoch, 1)
        self.assertEqual(node.input_ik.session_epoch, 1)
        self.assertEqual(node.input_ik.focus_count, 1)

        node._update_mode(ArmControlMode.IK, force=True, grant_receipt_ns=101)
        self.assertEqual(node._active_ik_epoch, 2)
        self.assertEqual(node.input_ik.focus_count, 2)

        arm_buttons = [0] * 12
        for index in (0, 8, 9, 10):
            arm_buttons[index] = 1
        node._receipt_ns = 102
        node.joy_callback(Message(buttons=arm_buttons, stamp_ns=102))
        self.assertEqual(node._active_ik_epoch, 3)
        self.assertEqual(node.input_ik.session_epoch, 3)
        self.assertEqual(node.input_ik.focus_count, 3)

        node._update_mode(ArmControlMode.JOYSTICK, grant_receipt_ns=103)
        self.assertIsNone(node._active_ik_epoch)
        self.assertIsNone(node.input_ik.session_epoch)

    def test_rearm_in_ik_rejects_prior_session_output(self):
        node = self.make_node()
        node.controller.is_armed = True
        node.control_mode = ArmControlMode.IK
        node.active_input = node.input_ik
        node._ik_epoch_counter = 4
        node._active_ik_epoch = 4
        node.input_ik.begin_session(4)
        arm_buttons = [0] * 12
        for index in (0, 8, 9, 10):
            arm_buttons[index] = 1

        node._receipt_ns = 100
        node.joy_callback(Message(buttons=arm_buttons, stamp_ns=90))
        self.assertEqual(node._active_ik_epoch, 5)
        node.mov_callback(
            Message(
                names=list(("base", "shoulder", "elbow", "wrist", "hand")),
                positions=[0.0] * 5,
                stamp_ns=101,
                frame_id="ik-session:4",
            )
        )

        self.assertEqual(node.input_ik.mov_messages, [])
        self.assertFalse(node.controller.is_armed)
        self.assertIn("invalid IK output", node.controller.faults[0])

    def test_ik_epoch_exhaustion_fails_closed(self):
        node = self.make_node()
        node.control_mode = ArmControlMode.IK
        node.active_input = node.input_ik
        node._ik_epoch_counter = MAX_IK_SESSION_EPOCH

        with self.assertRaisesRegex(SafetyValidationError, "epoch space is exhausted"):
            node._update_mode(ArmControlMode.IK, force=True, grant_receipt_ns=100)
        self.assertIsNone(node._active_ik_epoch)
        self.assertIsNone(node.input_ik.session_epoch)

    def test_mov_requires_exact_active_ik_authority_frame(self):
        for frame_id, accepted in (
            ("ik-session:6", False),
            ("ik-session:07", False),
            ("ik-session:8", False),
            ("ik-session:7", True),
        ):
            with self.subTest(frame_id=frame_id):
                node = self.make_node()
                node.controller.is_armed = True
                node.control_mode = ArmControlMode.IK
                node.active_input = node.input_ik
                node._ik_epoch_counter = 7
                node._active_ik_epoch = 7
                node.input_ik.begin_session(7)
                node.mov_callback(
                    Message(
                        names=list(("base", "shoulder", "elbow", "wrist", "hand")),
                        positions=[0.0] * 5,
                        stamp_ns=50,
                        frame_id=frame_id,
                    )
                )

                self.assertEqual(bool(node.input_ik.mov_messages), accepted)
                self.assertEqual(bool(node.controller.faults), not accepted)

    def test_invalid_ik_output_fault_stops(self):
        node = self.make_node()
        node.controller.is_armed = True
        node.control_mode = ArmControlMode.IK
        node.mov_callback(Message(names=["wrong"], positions=[0.0]))
        self.assertFalse(node.controller.is_armed)
        self.assertIn("invalid IK output", node.controller.faults[0])


if __name__ == "__main__":
    unittest.main()
