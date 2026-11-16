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
    assert motor._heartbeat.value > 0
    motor.write(frame(decision=decision))
    assert motor._heartbeat.value == 0
    assert all((chip / f"pwm{i}/duty_cycle").read_text() == "0" for i in range(4))
    assert all((chip / f"pwm{i}/enable").read_text() == "1" for i in range(4))
    motor.write(frame(.2, .2))
    assert motor._heartbeat.value > 0
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


def test_activation_and_active_stop_arm_before_physical_writes(sink, monkeypatch):
    motor, calls, chip = sink
    original = rp1._write_fd
    heartbeats = []

    def observe(fd, value):
        heartbeats.append(motor._heartbeat.value)
        original(fd, value)

    monkeypatch.setattr(rp1, "_write_fd", observe)
    motor.write(frame(.2, .2))
    assert heartbeats and all(heartbeat > 0 for heartbeat in heartbeats)
    heartbeats.clear()
    motor.write(frame(decision=SafetyDecision.STOP))
    assert heartbeats and all(heartbeat > 0 for heartbeat in heartbeats)
    assert motor._heartbeat.value == 0


def _owner_for_crash_test(directory, ready, armed=True):
    from multiprocessing.reduction import DupFd
    context = mp.get_context("spawn")
    paths = [str(Path(directory) / str(i)) for i in range(4)]
    lock_fd = os.open(str(Path(directory) / "lock"), os.O_CREAT | os.O_RDWR, 0o600)
    heartbeat = context.RawValue("Q", time.monotonic_ns() if armed else 0)
    tripped = context.RawValue("b", False)
    started, stop = context.Event(), context.Event()
    lock = context.Lock()
    guard = context.Process(target=rp1._watchdog, args=(paths, DupFd(lock_fd), lock,
        heartbeat, tripped, started, stop, os.getpid()))
    guard.start()
    assert started.wait(3)
    # Simulate death in the middle of a motor write, holding the semaphore.
    lock.acquire()
    heartbeat.value = time.monotonic_ns() if armed else 0
    ready.set()
    time.sleep(10)


@pytest.mark.parametrize("failure", [signal.SIGKILL, signal.SIGSTOP])
def test_zero_only_watchdog_covers_owner_death_and_hang(tmp_path, failure, capfd):
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
        diagnostic = ""
        diagnostic_deadline = time.monotonic() + 1
        while time.monotonic() < diagnostic_deadline:
            diagnostic += capfd.readouterr().err
            if "RP1_PWM_WATCHDOG" in diagnostic:
                break
            time.sleep(.01)
        expected = "OWNER_EXITED" if failure == signal.SIGKILL else "HEARTBEAT_EXPIRED"
        assert f"RP1_PWM_WATCHDOG reason={expected} parent_pid={owner.pid} " in diagnostic
        assert "heartbeat_age_ns=" in diagnostic
        assert diagnostic.count("RP1_PWM_WATCHDOG") == 1
    finally:
        if owner.is_alive():
            owner.kill()
            owner.join(2)


def test_zero_idle_does_not_kill_owner_during_slow_terminal_cleanup(tmp_path):
    for index in range(4):
        (tmp_path / str(index)).write_text("0")
    context = mp.get_context("spawn")
    ready = context.Event()
    owner = context.Process(target=_owner_for_crash_test, args=(str(tmp_path), ready, False))
    owner.start()
    try:
        assert ready.wait(5)
        time.sleep(rp1.WATCHDOG_S * 2)
        assert owner.is_alive()
        assert all((tmp_path / str(i)).read_text() == "0" for i in range(4))
    finally:
        if owner.is_alive():
            owner.kill()
        owner.join(2)


def _owner_for_slow_zero_write(directory, blocked, release, finished):
    chip = Path(directory)
    original_open = os.open
    lock_path = Path(f"/run/user/{os.getuid()}/r2b4-rp1-pwm.lock")

    def isolated_open(path, *args, **kwargs):
        if Path(path) == lock_path:
            path = chip / "test-ownership.lock"
        return original_open(path, *args, **kwargs)

    # Exercise the real sink and guardian against normal temporary files.
    # Neither the physical PWM device nor its production ownership lock is used.
    rp1.os.open = isolated_open
    rp1.verify_pinmux = lambda: None
    rp1.find_pwm0 = lambda root: chip
    motor = rp1.Rp1MotorFrameSink(config(), sysfs_root=chip)
    original_write = rp1._write_fd

    def slow_zero(fd, value):
        if not blocked.is_set():
            blocked.set()
            if not release.wait(5):
                raise TimeoutError("test did not release redundant STOP write")
        original_write(fd, value)

    rp1._write_fd = slow_zero
    try:
        motor.write(frame(decision=SafetyDecision.STOP))
        finished.set()
    finally:
        rp1._write_fd = original_write
        motor.close()


def test_redundant_zero_stop_can_block_without_killing_idle_owner(tmp_path, capfd):
    for channel in range(4):
        directory = tmp_path / f"pwm{channel}"
        directory.mkdir()
        for name, value in {"duty_cycle": "0", "period": "125000",
                            "enable": "1", "polarity": "normal"}.items():
            (directory / name).write_text(value)
    context = mp.get_context("spawn")
    blocked, release, finished = context.Event(), context.Event(), context.Event()
    owner = context.Process(target=_owner_for_slow_zero_write,
                            args=(str(tmp_path), blocked, release, finished))
    owner.start()
    try:
        assert blocked.wait(10)
        time.sleep(rp1.WATCHDOG_S * 2)
        assert owner.is_alive()
        assert all((tmp_path / f"pwm{i}/duty_cycle").read_text() == "0" for i in range(4))
        release.set()
        assert finished.wait(5)
        owner.join(5)
        assert owner.exitcode == 0
        assert "RP1_PWM_WATCHDOG" not in capfd.readouterr().err
    finally:
        release.set()
        if owner.is_alive():
            owner.kill()
        owner.join(2)


def test_abnormal_guardian_diagnostic_follows_physical_zero(tmp_path, monkeypatch):
    paths = [tmp_path / str(index) for index in range(4)]
    for path in paths:
        path.write_text("9")
    original_write = os.write
    notices = []

    def observe_write(descriptor, data):
        if descriptor == 2:
            assert all(path.read_text() == "0" for path in paths)
            notices.append(data)
            return len(data)
        return original_write(descriptor, data)

    def broken_wait(timeout):
        raise OSError("test watchdog wait failure")

    monkeypatch.setattr(rp1.os, "write", observe_write)
    ownership = os.open(tmp_path / "ownership.lock", os.O_CREAT | os.O_RDWR, 0o600)
    with pytest.raises(OSError, match="test watchdog wait failure"):
        rp1._watchdog([str(path) for path in paths], SimpleNamespace(detach=lambda: ownership),
                      threading.Lock(), SimpleNamespace(value=0), SimpleNamespace(value=False),
                      threading.Event(), SimpleNamespace(wait=broken_wait), os.getpid())
    assert len(notices) == 1
    assert b"reason=WATCHDOG_ERROR" in notices[0]
    assert b"heartbeat_age_ns=disarmed" in notices[0]


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
