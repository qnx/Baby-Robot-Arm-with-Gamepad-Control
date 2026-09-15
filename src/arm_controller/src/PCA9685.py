# Copyright (c) 2016 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Small, fail-closed PCA9685 driver used by the arm controller."""

from __future__ import annotations

import logging
import math
import fcntl
import os
import stat
import threading
import time
from typing import Callable, Optional


SMBUS_INTERFACE = 1

# Registers/etc.
PCA9685_ADDRESS = 0x40
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06
LED0_ON_H = 0x07
LED0_OFF_L = 0x08
LED0_OFF_H = 0x09
ALL_LED_OFF_H = 0xFD

# Bits.
RESTART = 0x80
SLEEP = 0x10
OUTDRV = 0x04
FULL_OFF = 0x10

logger = logging.getLogger(__name__)


class ExclusiveActuatorLock:
    """Hold a non-blocking, process-lifetime POSIX lock for one I2C device."""

    _registry_lock = threading.Lock()
    _held_paths = set()

    def __init__(self, path: str):
        self._fd = -1
        self._close_lock = threading.Lock()
        self._registry_path = None
        self._registry_registered = False
        if (
            not isinstance(path, str)
            or not path.startswith("/")
            or "\x00" in path
            or os.path.normpath(path) != path
        ):
            raise ValueError("actuator lock path must be an absolute path")
        no_follow = getattr(os, "O_NOFOLLOW", None)
        directory_only = getattr(os, "O_DIRECTORY", None)
        if no_follow is None or directory_only is None or os.open not in os.supports_dir_fd:
            raise RuntimeError("this target lacks safe openat/no-follow actuator-lock support")

        # POSIX record locks are process-scoped, so lockf alone permits a second
        # object in this process. Resolve path aliases and reserve the device
        # before touching the lock file so concurrent constructors fail closed.
        self._registry_path = os.path.normcase(os.path.realpath(path))
        with self._registry_lock:
            if self._registry_path in self._held_paths:
                raise RuntimeError(f"actuator lock is already held in this process: {path}")
            self._held_paths.add(self._registry_path)
            self._registry_registered = True

        try:
            parent, basename = os.path.split(path)
            directory_fd = os.open(
                parent,
                os.O_RDONLY | directory_only | no_follow | getattr(os, "O_CLOEXEC", 0),
            )
            try:
                directory_status = os.fstat(directory_fd)
                if (
                    not stat.S_ISDIR(directory_status.st_mode)
                    or directory_status.st_uid != os.geteuid()
                    or directory_status.st_mode & 0o077
                ):
                    raise RuntimeError(
                        "actuator lock directory must be owned by the service user with mode 0700"
                    )
                flags = os.O_RDWR | os.O_CREAT | no_follow | getattr(os, "O_CLOEXEC", 0)
                self._fd = os.open(basename, flags, 0o600, dir_fd=directory_fd)
            finally:
                os.close(directory_fd)

            try:
                file_status = os.fstat(self._fd)
                if (
                    not stat.S_ISREG(file_status.st_mode)
                    or file_status.st_uid != os.geteuid()
                    or file_status.st_mode & 0o077
                    or file_status.st_nlink != 1
                ):
                    raise RuntimeError(
                        "actuator lock file must be singly linked, service-owned, and mode 0600"
                    )
                fcntl.lockf(self._fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except Exception as error:
                raise RuntimeError(
                    f"actuator lock is already held or unusable: {path}"
                ) from error
        except Exception:
            # Constructor failures must not poison the in-process registry.
            self.close()
            raise

    def _release_registry(self) -> None:
        with self._registry_lock:
            if not self._registry_registered:
                return
            self._held_paths.discard(self._registry_path)
            self._registry_registered = False

    def close(self) -> None:
        # Both the descriptor and registry entry are released at most once,
        # including when multiple cleanup paths converge after a failure.
        with self._close_lock:
            try:
                if self._fd >= 0:
                    descriptor = self._fd
                    self._fd = -1
                    os.close(descriptor)
            finally:
                self._release_registry()


class PCA9685:
    """PCA9685 PWM controller with instance-owned I2C lifecycle."""

    def __init__(
        self,
        interface: int = SMBUS_INTERFACE,
        address: int = PCA9685_ADDRESS,
        bus_factory: Optional[Callable[[int], object]] = None,
        lock_path: Optional[str] = None,
    ):
        if isinstance(interface, bool) or not isinstance(interface, int) or interface < 0:
            raise ValueError("I2C interface must be a non-negative integer")
        if isinstance(address, bool) or not isinstance(address, int) or not 0x08 <= address <= 0x77:
            raise ValueError("PCA9685 address must be a usable 7-bit I2C address")

        # Import and open the hardware only when an instance is constructed. This
        # keeps module import side-effect free and lets initialization fail closed.
        if bus_factory is None:
            import smbus

            bus_factory = smbus.SMBus

        self.interface = interface
        self.address = address
        if lock_path is None:
            lock_path = f"/var/run/baby_robot_arm/pca9685-i2c{interface}-0x{address:02x}.lock"
        self._actuator_lock = ExclusiveActuatorLock(lock_path)
        try:
            self.bus = bus_factory(interface)
        except Exception:
            self._actuator_lock.close()
            raise
        self._closed = False

        try:
            # Keep every output disabled until configuration is complete.
            self.disable_all_pwm()
            self.bus.write_byte_data(self.address, MODE2, OUTDRV)
            # Address this actuator only. ALLCALL is unnecessary here and would
            # let unrelated group-address traffic alter outputs on a shared bus.
            self.bus.write_byte_data(self.address, MODE1, 0x00)
            time.sleep(0.005)
            mode1 = self.bus.read_byte_data(self.address, MODE1)
            self.bus.write_byte_data(self.address, MODE1, mode1 & ~SLEEP)
            time.sleep(0.005)
        except Exception:
            self._best_effort_disable()
            self.close(disable=False)
            raise

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("PCA9685 bus is closed")

    @staticmethod
    def _validate_pwm_value(value: object, name: str) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 4095:
            raise ValueError(f"{name} must be an integer in [0, 4095]")
        return value

    def set_pwm_freq(self, freq_hz: float) -> None:
        """Set a finite, datasheet-supported PWM frequency."""
        self._require_open()
        frequency = float(freq_hz)
        if not math.isfinite(frequency) or not 24.0 <= frequency <= 1526.0:
            raise ValueError("PWM frequency must be finite and in [24, 1526] Hz")

        prescale_value = 25_000_000.0 / 4096.0 / frequency - 1.0
        prescale = int(math.floor(prescale_value + 0.5))
        if not 3 <= prescale <= 255:
            raise ValueError("PWM frequency produced an invalid prescale")

        old_mode = self.bus.read_byte_data(self.address, MODE1)
        self.bus.write_byte_data(self.address, MODE1, (old_mode & 0x7F) | SLEEP)
        self.bus.write_byte_data(self.address, PRESCALE, prescale)
        self.bus.write_byte_data(self.address, MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, MODE1, old_mode | RESTART)

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        self._require_open()
        if isinstance(channel, bool) or not isinstance(channel, int) or not 0 <= channel <= 15:
            raise ValueError("PWM channel must be an integer in [0, 15]")
        on_value = self._validate_pwm_value(on, "on")
        off_value = self._validate_pwm_value(off, "off")

        # A hardware OE/watchdog remains necessary: these four writes are not an
        # atomic safety boundary if the bus fails partway through.
        self.bus.write_byte_data(self.address, LED0_ON_L + 4 * channel, on_value & 0xFF)
        self.bus.write_byte_data(self.address, LED0_ON_H + 4 * channel, on_value >> 8)
        self.bus.write_byte_data(self.address, LED0_OFF_L + 4 * channel, off_value & 0xFF)
        self.bus.write_byte_data(self.address, LED0_OFF_H + 4 * channel, off_value >> 8)

    def disable_channel(self, channel: int) -> None:
        """Set one channel's FULL_OFF bit before clearing global FULL_OFF."""
        self._require_open()
        if isinstance(channel, bool) or not isinstance(channel, int) or not 0 <= channel <= 15:
            raise ValueError("PWM channel must be an integer in [0, 15]")
        register = LED0_ON_L + 4 * channel
        self.bus.write_byte_data(self.address, register, 0x00)
        self.bus.write_byte_data(self.address, register + 1, 0x00)
        self.bus.write_byte_data(self.address, register + 2, 0x00)
        self.bus.write_byte_data(self.address, register + 3, FULL_OFF)

    def disable_all_pwm(self) -> None:
        """Set the PCA9685 global FULL_OFF bit."""
        self._require_open()
        self.bus.write_byte_data(self.address, ALL_LED_OFF_H, FULL_OFF)

    def enable_all_pwm(self) -> None:
        """Clear FULL_OFF after all channel values have been preloaded."""
        self._require_open()
        self.bus.write_byte_data(self.address, ALL_LED_OFF_H, 0x00)

    def software_reset(self) -> None:
        """Issue the PCA9685 General Call SWRST command to all responders."""
        self._require_open()
        # SWRST is a data byte sent to General Call address 0x00, not register
        # 0x06 on the configured device (0x06 is LED0_ON_L).
        self.bus.write_byte(0x00, 0x06)
        time.sleep(0.005)

    def _best_effort_disable(self) -> None:
        if self._closed:
            return
        try:
            self.bus.write_byte_data(self.address, ALL_LED_OFF_H, FULL_OFF)
        except Exception:
            logger.exception("Unable to disable PCA9685 outputs over I2C")

    def close(self, disable: bool = True) -> None:
        if self._closed:
            return
        if disable:
            self._best_effort_disable()
        try:
            close_method = getattr(self.bus, "close", None)
            if close_method is not None:
                close_method()
        finally:
            self._closed = True
            self._actuator_lock.close()

    def __enter__(self) -> "PCA9685":
        self._require_open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
