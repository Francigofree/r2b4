import multiprocessing as mp
import os
from pathlib import Path
import signal
import threading
import time
from types import SimpleNamespace

import pytest

from v3.adapters import rp1_motor as rp1
from v3.adapters.gpio_motor import GpioMotorFrameSinkConfig
from v3.adapters.motor_pwm import MotorChannelPhysicalConfig, PwmDecayMode, plan_final_actuation
from v3.contracts import FinalActuation, SafetyDecision, TickContext


def config():
    return GpioMotorFrameSinkConfig(MotorChannelPhysicalConfig(12, 13, pwm_decay_mode=PwmDecayMode.BRAKE),
                                    MotorChannelPhysicalConfig(18, 19, invert=True, pwm_decay_mode=PwmDecayMode.BRAKE),
                                    pwm_backend="rp1")


def frame(left=0., right=0., decision=SafetyDecision.ALLOW):
    cfg = config()
    return plan_final_actuation(cfg.left, cfg.right, FinalActuation(
        TickContext(1, 20_000_000), left, right, decision is SafetyDecision.ALLOW,
        decision, "CLEAR", None if decision is SafetyDecision.ALLOW else "STOPPED"))


class FakeProcess:
    alive = True
    exitcode = 0

    def start(self): pass
    def is_alive(self): return self.alive
    def join(self, timeout): self.alive = False


class FakeContext:
    def Lock(self): return threading.Lock()
    def Event(self):
        event = threading.Event()
        event.set()
        return event
    def RawValue(self, kind, value): return SimpleNamespace(value=value)
    def Process(self, **kwargs): return FakeProcess()


@pytest.fixture
def sink(tmp_path, monkeypatch):
    chip = tmp_path / "pwmchip73"
    chip.mkdir()
    for channel in range(4):
        directory = chip / f"pwm{channel}"
        directory.mkdir()
        for name, value in {"duty_cycle": "0", "period": "125000", "enable": "1", "polarity": "normal"}.items():
            (directory / name).write_text(value)
    monkeypatch.setattr(rp1, "verify_pinmux", lambda: None)
    monkeypatch.setattr(rp1, "find_pwm0", lambda root: chip)
    monkeypatch.setattr(rp1.mp, "get_context", lambda method: FakeContext())
    monkeypatch.setattr(rp1, "DupFd", lambda fd: None)
    original = rp1._write_fd
    calls = []
    def write(fd, value):
        calls.append((Path(os.readlink(f"/proc/self/fd/{fd}")).parent.name, value))
        original(fd, value)
        os.ftruncate(fd, len(str(value)))
    monkeypatch.setattr(rp1, "_write_fd", write)
    result = rp1.Rp1MotorFrameSink(config())
    calls.clear()
    yield result, calls, chip
    result.close()


def test_hardware_waveform_updates_do_not_restart_other_wheel(sink):
    motor, calls, chip = sink
    motor.write(frame(.3, .3))
    calls.clear()
    motor.write(frame(.4, .3))
    assert calls == [("pwm1", 75000)]
    calls.clear()
    motor.write(frame(-.4, .3))
    assert {channel for channel, value in calls} == {"pwm0", "pwm1"}
    assert calls[:2] == [("pwm0", 0), ("pwm1", 0)]
    assert int((chip / "pwm2/duty_cycle").read_text()) > 0


@pytest.mark.parametrize("decision", [SafetyDecision.STOP, SafetyDecision.FAULT])
def test_hardware_stop_is_zero_enabled_and_restarts_from_zero(sink, decision):
    motor, calls, chip = sink
    motor.write(frame(.3, .3))
    motor.write(frame(decision=decision))
    assert all((chip / f"pwm{i}/duty_cycle").read_text() == "0" for i in range(4))
    assert all((chip / f"pwm{i}/enable").read_text() == "1" for i in range(4))
    motor.write(frame(.2, .2))
    assert any(int((chip / f"pwm{i}/duty_cycle").read_text()) for i in range(4))


def test_watchdog_loss_latches_zero_and_rejects_later_motion(sink):
    motor, calls, chip = sink
    motor.write(frame(.3, .3))
    motor._guard.alive = False
    with pytest.raises(OSError, match="watchdog"):
        motor.write(frame(.4, .4))
    motor._guard.alive = True
    with pytest.raises(OSError, match="watchdog"):
        motor.write(frame(.4, .4))
    assert motor.failed
    assert all((chip / f"pwm{i}/duty_cycle").read_text() == "0" for i in range(4))


def _owner_for_crash_test(directory, ready):
    from multiprocessing.reduction import DupFd
    context = mp.get_context("spawn")
    paths = [str(Path(directory) / str(i)) for i in range(4)]
    lock_fd = os.open(str(Path(directory) / "lock"), os.O_CREAT | os.O_RDWR, 0o600)
    heartbeat = context.RawValue("Q", time.monotonic_ns())
    tripped = context.RawValue("b", False)
    started, stop = context.Event(), context.Event()
    lock = context.Lock()
    guard = context.Process(target=rp1._watchdog, args=(paths, DupFd(lock_fd), lock,
        heartbeat, tripped, started, stop, os.getpid()))
    guard.start()
    assert started.wait(3)
    # Simulate death in the middle of a motor write, holding the semaphore.
    lock.acquire()
    heartbeat.value = time.monotonic_ns()
    ready.set()
    time.sleep(10)


@pytest.mark.parametrize("failure", [signal.SIGKILL, signal.SIGSTOP])
def test_zero_only_watchdog_covers_owner_death_and_hang(tmp_path, failure):
    for index in range(4):
        (tmp_path / str(index)).write_text("9")
    context = mp.get_context("spawn")
    ready = context.Event()
    owner = context.Process(target=_owner_for_crash_test, args=(str(tmp_path), ready))
    owner.start()
    try:
        assert ready.wait(5)
        os.kill(owner.pid, failure)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            if all((tmp_path / str(i)).read_text() == "0" for i in range(4)):
                break
            time.sleep(.01)
        assert all((tmp_path / str(i)).read_text() == "0" for i in range(4))
        owner.join(1)
        assert not owner.is_alive()
    finally:
        if owner.is_alive():
            owner.kill()
            owner.join(2)


def test_pwm_identity_excludes_the_fan_and_chip_number(tmp_path):
    for name, address in (("pwmchip23", "pwm@9c000"), ("pwmchip77", "pwm@98000")):
        chip = tmp_path / name
        node = tmp_path / "dt" / address
        node.mkdir(parents=True)
        (node / "compatible").write_bytes(b"raspberrypi,rp1-pwm\0")
        (chip / "device").mkdir(parents=True)
        (chip / "device/of_node").symlink_to(node)
        (chip / "npwm").write_text("4")
    assert rp1.find_pwm0(tmp_path).name == "pwmchip77"
