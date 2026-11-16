"""Four RP1 PWM0 channels behind the sole native L12 motor writer.

The kernel generates the waveform. No lgpio transmitter, subprocess, file open
or sleep is used by a normal frame write. A separate zero-only watchdog covers
the persistent sysfs PWM state when the control process dies or stops ticking.
"""
from __future__ import annotations

import fcntl
import multiprocessing as mp
from multiprocessing.reduction import DupFd
import os
from pathlib import Path
import re
import select
import signal
import subprocess
import time

from v3.contracts import SafetyDecision
from .gpio_motor import GpioMotorFrameSink, GpioMotorFrameSinkConfig
from .motor_pwm import Drv8871MotorFrame

PIN_CHANNELS = {12: 0, 13: 1, 18: 2, 19: 3}
WATCHDOG_S = 0.25


def find_pwm0(root: Path = Path("/sys/class/pwm")) -> Path:
    """Identify header PWM0 by DT identity, never by pwmchip enumeration."""
    matches = []
    for chip in root.glob("pwmchip*"):
        node = chip / "device/of_node"
        if not (node / "compatible").is_file():
            continue
        if (node / "compatible").read_bytes().split(b"\0")[0] != b"raspberrypi,rp1-pwm":
            continue
        if node.resolve().name == "pwm@98000" and int((chip / "npwm").read_text()) == 4:
            matches.append(chip)
    if len(matches) != 1:
        raise OSError("exactly one four-channel RP1 PWM0 is required")
    return matches[0]


def verify_pinmux() -> None:
    result = subprocess.run(["pinctrl", "get"], capture_output=True, text=True,
                            check=True, timeout=2)
    for pin, channel in PIN_CHANNELS.items():
        if re.search(rf"^{pin}:.*= PWM0_CHAN{channel}$", result.stdout, re.MULTILINE) is None:
            raise OSError(f"GPIO {pin} is not PWM0 channel {channel}; install r2b4-pwm overlay")


def _write_fd(fd: int, value: int) -> None:
    data = str(value).encode("ascii")
    os.lseek(fd, 0, os.SEEK_SET)
    if os.write(fd, data) != len(data):
        raise OSError("short RP1 PWM attribute write")


def _zero_fds(fds) -> None:
    first = None
    for fd in fds:
        try:
            _write_fd(fd, 0)
        except OSError as exc:
            first = first or exc
    if first is not None:
        raise first


def _watchdog(paths, lock_fd, output_lock, heartbeat, tripped, ready, stop, parent_pid):
    """No positive-output input exists on this process boundary."""
    owned_lock = lock_fd.detach()
    fds = []
    parent_fd = os.pidfd_open(parent_pid)
    try:
        for path in paths:
            fds.append(os.open(path, os.O_WRONLY | os.O_CLOEXEC))
        ready.set()
        while not stop.wait(0.02):
            if (os.getppid() != parent_pid or (heartbeat.value and
                    time.monotonic_ns() - heartbeat.value > int(WATCHDOG_S * 1e9))):
                break
        # Latch before waiting: even a temporarily busy parent cannot start
        # another frame. If the parent died holding its lock, still force LOW.
        tripped.value = True
        if not stop.is_set() and os.getppid() == parent_pid:
            # A suspended owner must never wake and overwrite emergency LOW.
            # Pin the process identity with pidfd, then revoke it before zero.
            signal.pidfd_send_signal(parent_fd, signal.SIGKILL)
            select.select([parent_fd], [], [], 0.2)
        acquired = output_lock.acquire(timeout=0.05)
        try:
            _zero_fds(fds)
            time.sleep(0.002)
        finally:
            if acquired:
                output_lock.release()
    finally:
        tripped.value = True
        try:
            _zero_fds(fds)
        finally:
            for fd in fds:
                os.close(fd)
        os.close(owned_lock)
        os.close(parent_fd)


class Rp1MotorFrameSink:
    """One hardware owner; STOP is enabled normal-polarity zero duty.

    Disabled PWM has no portable LOW guarantee. Leave the four channels
    enabled at zero on close, including after watchdog intervention. A future
    owner may adopt only these idle channels after acquiring the same lock.
    """

    def __init__(self, config: GpioMotorFrameSinkConfig, *, sysfs_root=Path("/sys/class/pwm")):
        if set(config.pins) != set(PIN_CHANNELS):
            raise ValueError("RP1 motor wiring requires GPIO 12, 13, 18, 19")
        self._config = config
        self._closed = False
        self._failed = False
        self._fds = {}
        self._last = {pin: 0 for pin in config.pins}
        self._directions = (0, 0)
        self._guard = None
        self._ownership_fd = None
        self._channels = []
        self._period = round(1e9 / config.pwm_frequency_hz)
        verify_pinmux()
        chip = find_pwm0(Path(sysfs_root))
        lock_dir = Path(f"/run/user/{os.getuid()}")
        self._ownership_fd = os.open(lock_dir / "r2b4-rp1-pwm.lock",
                                     os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(self._ownership_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            for pin in config.pins:
                channel = chip / f"pwm{PIN_CHANNELS[pin]}"
                if not channel.exists():
                    (chip / "export").write_text(str(PIN_CHANNELS[pin]))
                    # udev grants gpio-group access asynchronously; startup only.
                    deadline = time.monotonic() + 1.0
                    while not os.access(channel / "duty_cycle", os.W_OK):
                        if time.monotonic() >= deadline:
                            raise OSError("RP1 PWM export permissions unavailable")
                        time.sleep(0.01)
                self._channels.append(channel)
                self._fds[pin] = os.open(channel / "duty_cycle", os.O_WRONLY | os.O_CLOEXEC)
            if any(int((channel / "duty_cycle").read_text()) != 0 for channel in self._channels):
                _zero_fds(self._fds.values())
                raise OSError("RP1 PWM had nonzero duty before ownership; zeroed, startup refused")
            for pin, channel in zip(config.pins, self._channels):
                (channel / "enable").write_text("0")
                (channel / "polarity").write_text("normal")
                (channel / "period").write_text(str(self._period))
                _write_fd(self._fds[pin], 0)
                (channel / "enable").write_text("1")
            self._verify_zero()
            context = mp.get_context("spawn")
            self._output_lock = context.Lock()
            self._heartbeat = context.RawValue("Q", 0)
            self._tripped = context.RawValue("b", False)
            self._stop = context.Event()
            ready = context.Event()
            self._guard = context.Process(target=_watchdog, args=(
                [str(channel / "duty_cycle") for channel in self._channels],
                DupFd(self._ownership_fd), self._output_lock, self._heartbeat,
                self._tripped, ready, self._stop, os.getpid()), name="v3-rp1-zero-watchdog")
            self._guard.start()
            if not ready.wait(3.0) or not self._guard.is_alive():
                raise OSError("RP1 zero watchdog failed to start")
            # Startup is not an ACTIVE deadline. No positive duty exists yet.
        except BaseException:
            self._failed = True
            self.close()
            raise

    @property
    def closed(self):
        return self._closed

    @property
    def failed(self):
        return self._failed

    def _verify_zero(self):
        for channel in self._channels:
            if ((channel / "duty_cycle").read_text().strip() != "0"
                    or (channel / "polarity").read_text().strip() != "normal"
                    or (channel / "enable").read_text().strip() != "1"):
                raise OSError("RP1 PWM zero-state readback failed")

    def write(self, frame: Drv8871MotorFrame):
        if self._closed:
            raise OSError("RP1 motor sink is closed")
        if not isinstance(frame, Drv8871MotorFrame):
            raise TypeError("frame must be Drv8871MotorFrame")
        acquired = self._output_lock.acquire(False)
        try:
            if (not acquired or self._tripped.value or not self._guard.is_alive()
                    or (self._heartbeat.value and time.monotonic_ns() - self._heartbeat.value > int(WATCHDOG_S * 1e9))):
                raise OSError("RP1 motor watchdog expired or unavailable")
            self._heartbeat.value = time.monotonic_ns()
            if frame.safety_decision is SafetyDecision.ALLOW:
                directions = (
                    GpioMotorFrameSink._channel_direction(frame.left, self._config.left),
                    GpioMotorFrameSink._channel_direction(frame.right, self._config.right))
                # A turning wheel must not interrupt the other wheel's PWM.
                reversal = False
                for index, motor in enumerate((self._config.left, self._config.right)):
                    if directions[index] != self._directions[index]:
                        reversal |= directions[index] * self._directions[index] < 0
                        for pin in (motor.in1, motor.in2):
                            _write_fd(self._fds[pin], 0)
                            self._last[pin] = 0
                if reversal:
                    # Complete at least one hardware period at zero before
                    # reversing polarity. Ordinary updates/STOP never sleep.
                    time.sleep(self._period / 1e9)
                duties = (frame.left.in1_duty_cycle, frame.left.in2_duty_cycle,
                          frame.right.in1_duty_cycle, frame.right.in2_duty_cycle)
                for pin, duty in zip(self._config.pins, duties):
                    value = round(self._period * duty / 100)
                    if value != self._last[pin]:
                        _write_fd(self._fds[pin], value)
                        self._last[pin] = value
                self._directions = directions
            else:
                _zero_fds(self._fds.values())
                self._last = dict.fromkeys(self._config.pins, 0)
                self._directions = (0, 0)
            if self._tripped.value:
                raise OSError("RP1 motor watchdog tripped during write")
            # Disarm the deadline only AFTER every requested output is zero.
            # Terminal capture/device cleanup may exceed 250 ms at safe zero.
            # Arm before the next write so a hang mid-activation stays covered.
            self._heartbeat.value = (time.monotonic_ns() if any(self._last.values()) else 0)
        except BaseException:
            self._failed = True
            self._tripped.value = True
            _zero_fds(self._fds.values())
            raise
        finally:
            if acquired:
                self._output_lock.release()

    def close(self):
        if self._closed:
            return
        try:
            _zero_fds(self._fds.values())
            time.sleep(0.002)
            if self._guard is not None:
                self._stop.set()
                self._guard.join(timeout=1.0)
                if self._guard.is_alive() or self._guard.exitcode != 0:
                    self._failed = True
                    raise OSError("RP1 zero watchdog did not close cleanly")
            if len(self._fds) == 4:
                self._verify_zero()
        finally:
            for fd in self._fds.values():
                os.close(fd)
            self._fds.clear()
            if self._ownership_fd is not None:
                os.close(self._ownership_fd)
                self._ownership_fd = None
            self._closed = True
