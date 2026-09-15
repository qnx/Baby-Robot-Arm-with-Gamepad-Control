from pathlib import Path
import sys
import types
import unittest


SRC = str(Path(__file__).resolve().parents[1] / "src")
sys.path.insert(0, SRC)

# Keep this host test independent of a ROS installation; the input adapter only
# needs the message attributes exercised below.
sensor_msgs = types.ModuleType("sensor_msgs")
sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
sensor_msgs_msg.Joy = type("Joy", (), {})
sensor_msgs_msg.JointState = type("JointState", (), {})
sensor_msgs.msg = sensor_msgs_msg
sys.modules.setdefault("sensor_msgs", sensor_msgs)
sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_msg)

rclpy = types.ModuleType("rclpy")
rclpy_impl = types.ModuleType("rclpy.impl")
rclpy_logger = types.ModuleType("rclpy.impl.rcutils_logger")
rclpy_logger.RcutilsLogger = type("RcutilsLogger", (), {})
rclpy.impl = rclpy_impl
rclpy_impl.rcutils_logger = rclpy_logger
sys.modules.setdefault("rclpy", rclpy)
sys.modules.setdefault("rclpy.impl", rclpy_impl)
sys.modules.setdefault("rclpy.impl.rcutils_logger", rclpy_logger)

std_msgs = types.ModuleType("std_msgs")
std_msgs_msg = types.ModuleType("std_msgs.msg")
std_msgs_msg.Float64MultiArray = type("Float64MultiArray", (), {})
std_msgs.msg = std_msgs_msg
sys.modules.setdefault("std_msgs", std_msgs)
sys.modules.setdefault("std_msgs.msg", std_msgs_msg)

from arm_controller_input import (  # noqa: E402
    ArmControllerInverseKinematicInput,
    ArmControllerJointInput,
    ArmControllerScreensaverInput,
)
from safety import MAX_IK_SESSION_EPOCH  # noqa: E402


class FakeController:
    def __init__(self):
        self.calls = []
        self.center_calls = 0
        self.move_calls = []
        self.set_calls = []

    def set_joint_targets_rad_atomic(self, commands, record_activity=True):
        self.calls.append((tuple(commands), record_activity))

    def center_all_servos(self, record_activity=True):
        self.center_calls += 1

    def move_joint(self, joint, delta, record_activity=True):
        self.move_calls.append((joint, delta, record_activity))

    def set_joint(self, joint, value, record_activity=True):
        self.set_calls.append((joint, value, record_activity))


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class DirectJointLeaseTests(unittest.TestCase):
    def test_dds_joint_command_cannot_renew_operator_lease(self):
        controller = FakeController()
        adapter = ArmControllerJointInput(controller, object())
        message = type("Message", (), {"name": ["base"], "position": [0.0]})()

        adapter.joint_callback(message)

        self.assertEqual(len(controller.calls), 1)
        self.assertFalse(controller.calls[0][1])

    def test_autonomous_wrist_channels_match_published_servo_order(self):
        adapter = ArmControllerScreensaverInput(FakeController(), object())
        self.assertEqual(adapter.DRAWING_POSE[3], 50.0)  # wrist roll
        self.assertEqual(adapter.DRAWING_POSE[4], 10.0)  # wrist pitch
        conductor = adapter._dance_conductor()
        self.assertNotEqual(conductor[3], adapter.DRAWING_POSE[3])
        self.assertEqual(conductor[4], adapter.DRAWING_POSE[4])

    def test_ik_center_request_does_not_prestage_actuator_target(self):
        controller = FakeController()
        cartesian = FakePublisher()
        adapter = ArmControllerInverseKinematicInput(
            controller,
            type("Logger", (), {"info": lambda self, message: None})(),
            cartesian,
            FakePublisher(),
        )
        adapter.begin_session(41)
        message = type("JoyMessage", (), {"axes": [0.0] * 6, "buttons": [0] * 12})()
        message.buttons[3] = 1  # Y / bounded center request

        adapter.joy_callback(message)

        self.assertEqual(controller.center_calls, 0)
        self.assertEqual(len(cartesian.messages), 1)

    def test_ik_mode_does_not_desynchronize_solver_with_local_wrist_motion(self):
        controller = FakeController()
        adapter = ArmControllerInverseKinematicInput(
            controller,
            type("Logger", (), {"info": lambda self, message: None})(),
            FakePublisher(),
            FakePublisher(),
        )
        adapter.begin_session(42)
        message = type("JoyMessage", (), {"axes": [0.0] * 6, "buttons": [0] * 12})()
        message.axes[2] = 0.5  # Right-stick X was a local hand command.
        message.axes[4] = 1.0  # D-pad X was a local wrist command.

        adapter.joy_callback(message)

        self.assertEqual(controller.move_calls, [])

    def test_ik_sync_and_cartesian_commands_carry_same_exact_epoch(self):
        controller = FakeController()
        cartesian = FakePublisher()
        positions = FakePublisher()
        adapter = ArmControllerInverseKinematicInput(
            controller,
            type("Logger", (), {"info": lambda self, message: None})(),
            cartesian,
            positions,
        )
        controller.get_joint_rad = lambda joint: float(joint.value)
        adapter.begin_session(MAX_IK_SESSION_EPOCH)

        adapter.focus()
        message = type("JoyMessage", (), {"axes": [0.0] * 6, "buttons": [0] * 12})()
        message.axes[0] = 0.25
        adapter.joy_callback(message)

        self.assertEqual(positions.messages[0].data[-1], float(MAX_IK_SESSION_EPOCH))
        self.assertEqual(cartesian.messages[0].data[-1], float(MAX_IK_SESSION_EPOCH))

    def test_cleared_ik_session_cannot_publish_commands(self):
        adapter = ArmControllerInverseKinematicInput(
            FakeController(),
            type("Logger", (), {"info": lambda self, message: None})(),
            FakePublisher(),
            FakePublisher(),
        )
        adapter.begin_session(1)
        adapter.clear_session()
        message = type("JoyMessage", (), {"axes": [0.0] * 6, "buttons": [0] * 12})()
        message.axes[0] = 0.25

        with self.assertRaisesRegex(RuntimeError, "no active authority epoch"):
            adapter.joy_callback(message)


if __name__ == "__main__":
    unittest.main()
