from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from PCA9685 import (  # noqa: E402
    ALL_LED_OFF_H,
    ExclusiveActuatorLock,
    FULL_OFF,
    LED0_ON_L,
    MODE1,
    PCA9685,
)


class FakeBus:
    def __init__(self, interface):
        self.interface = interface
        self.calls = []
        self.closed = False

    def write_byte_data(self, address, register, value):
        self.calls.append(("write_byte_data", address, register, value))

    def read_byte_data(self, address, register):
        self.calls.append(("read_byte_data", address, register))
        return 0

    def write_byte(self, address, value):
        self.calls.append(("write_byte", address, value))

    def close(self):
        self.closed = True


class PCA9685Tests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.lock_path = str(Path(self.temp_dir.name) / "actuator.lock")

    def tearDown(self):
        self.temp_dir.cleanup()

    def make_driver(self, **kwargs):
        driver = PCA9685(lock_path=self.lock_path, **kwargs)
        self.addCleanup(driver.close)
        return driver

    @mock.patch("PCA9685.time.sleep")
    def test_bus_is_instance_owned_and_requested_interface_is_used(self, _sleep):
        buses = []

        def factory(interface):
            bus = FakeBus(interface)
            buses.append(bus)
            return bus

        driver = self.make_driver(interface=3, bus_factory=factory)
        self.assertEqual(driver.bus.interface, 3)
        self.assertEqual(len(buses), 1)
        self.assertIn(("write_byte_data", 0x40, ALL_LED_OFF_H, FULL_OFF), driver.bus.calls)
        self.assertIn(("write_byte_data", 0x40, MODE1, 0x00), driver.bus.calls)

    @mock.patch("PCA9685.time.sleep")
    def test_software_reset_uses_general_call_data_byte(self, _sleep):
        driver = self.make_driver(bus_factory=FakeBus)
        driver.software_reset()
        self.assertIn(("write_byte", 0x00, 0x06), driver.bus.calls)

    @mock.patch("PCA9685.time.sleep")
    def test_channel_and_value_validation_precedes_bus_write(self, _sleep):
        driver = self.make_driver(bus_factory=FakeBus)
        before = list(driver.bus.calls)
        with self.assertRaises(ValueError):
            driver.set_pwm(61, 0, 100)
        self.assertEqual(before, driver.bus.calls)

    @mock.patch("PCA9685.time.sleep")
    def test_individual_channel_full_off_survives_global_enable(self, _sleep):
        driver = self.make_driver(bus_factory=FakeBus)
        driver.disable_channel(15)
        driver.enable_all_pwm()
        channel_register = LED0_ON_L + 4 * 15
        full_off_call = ("write_byte_data", 0x40, channel_register + 3, FULL_OFF)
        global_enable_call = ("write_byte_data", 0x40, ALL_LED_OFF_H, 0x00)
        self.assertLess(driver.bus.calls.index(full_off_call), driver.bus.calls.index(global_enable_call))

    @mock.patch("PCA9685.time.sleep")
    def test_close_disables_and_closes_once(self, _sleep):
        driver = self.make_driver(bus_factory=FakeBus)
        driver.close()
        driver.close()
        self.assertTrue(driver.bus.closed)
        self.assertEqual(
            driver.bus.calls.count(("write_byte_data", 0x40, ALL_LED_OFF_H, FULL_OFF)),
            2,
        )

    def test_second_process_cannot_acquire_actuator_lock(self):
        lock = ExclusiveActuatorLock(self.lock_path)
        source_dir = str(Path(__file__).resolve().parents[1] / "src")
        code = (
            "import sys; "
            f"sys.path.insert(0, {source_dir!r}); "
            "from PCA9685 import ExclusiveActuatorLock; "
            f"path={self.lock_path!r}; "
            "\ntry:\n ExclusiveActuatorLock(path)\nexcept RuntimeError:\n sys.exit(0)\n"
            "sys.exit(1)"
        )
        try:
            result = subprocess.run([sys.executable, "-c", code], check=False)
            self.assertEqual(result.returncode, 0)
        finally:
            lock.close()

    def test_same_process_cannot_acquire_normalized_lock_alias(self):
        real_root = Path(self.temp_dir.name) / "real"
        lock_directory = real_root / "locks"
        lock_directory.mkdir(parents=True, mode=0o700)
        real_root.chmod(0o700)
        lock_directory.chmod(0o700)
        alias_root = Path(self.temp_dir.name) / "alias"
        alias_root.symlink_to(real_root, target_is_directory=True)

        direct_path = str(lock_directory / "actuator.lock")
        alias_path = str(alias_root / "locks" / "actuator.lock")
        lock = ExclusiveActuatorLock(direct_path)
        try:
            with self.assertRaisesRegex(RuntimeError, "already held in this process"):
                ExclusiveActuatorLock(alias_path)
        finally:
            lock.close()

    def test_close_is_idempotent_and_allows_reacquisition(self):
        first = ExclusiveActuatorLock(self.lock_path)
        first.close()
        first.close()

        second = ExclusiveActuatorLock(self.lock_path)
        second.close()

    @mock.patch("PCA9685.fcntl.lockf", side_effect=OSError("injected lock failure"))
    def test_acquisition_failure_releases_process_registry(self, _lockf):
        with self.assertRaises(RuntimeError):
            ExclusiveActuatorLock(self.lock_path)

        with mock.patch("PCA9685.fcntl.lockf"):
            lock = ExclusiveActuatorLock(self.lock_path)
        lock.close()

    def test_unsafe_lock_directory_mode_is_rejected(self):
        unsafe_dir = Path(self.temp_dir.name) / "unsafe"
        unsafe_dir.mkdir(mode=0o770)
        unsafe_dir.chmod(0o770)
        with self.assertRaises(RuntimeError):
            ExclusiveActuatorLock(str(unsafe_dir / "actuator.lock"))

    def test_hard_linked_lock_file_is_rejected_without_truncation(self):
        original = Path(self.temp_dir.name) / "protected"
        original.write_text("do-not-change", encoding="utf-8")
        linked = Path(self.temp_dir.name) / "actuator.lock"
        linked.hardlink_to(original)
        with self.assertRaises(RuntimeError):
            ExclusiveActuatorLock(str(linked))
        self.assertEqual(original.read_text(encoding="utf-8"), "do-not-change")

    def test_overly_permissive_lock_file_is_rejected(self):
        lock_file = Path(self.lock_path)
        lock_file.touch(mode=0o600)
        lock_file.chmod(0o644)
        with self.assertRaises(RuntimeError):
            ExclusiveActuatorLock(self.lock_path)


if __name__ == "__main__":
    unittest.main()
