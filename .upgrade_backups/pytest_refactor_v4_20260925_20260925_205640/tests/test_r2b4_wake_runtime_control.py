from __future__ import annotations

from r2b4_voice.runtime_control import WakeRuntimeCoordinator, WakeRuntimeOutcome


class _Controller:
    def __init__(self, running=False, fail=False):
        self.running = running
        self.fail = fail
        self.calls = []

    def status(self):
        return {"runtime_running": self.running, "runtime_pid": 77 if self.running else None}

    def runtime_start(self, capture_mode="alap"):
        self.calls.append(("start", capture_mode))
        if self.fail:
            raise RuntimeError("start failed")
        self.running = True
        return 88

    def wait_idle(self, timeout=3.0):
        self.calls.append(("wait_idle", timeout))
        if self.fail:
            raise RuntimeError("not idle")


class _Speaker:
    def __init__(self, fail=False):
        self.fail = fail
        self.calls = 0

    def play_ready(self):
        self.calls += 1
        if self.fail:
            raise RuntimeError("speaker unavailable")
        return "pw-play"


def test_wake_uses_canonical_runtime_start_then_idle_then_ack():
    controller = _Controller()
    speaker = _Speaker()
    result = WakeRuntimeCoordinator(controller, speaker).activate()
    assert result.outcome is WakeRuntimeOutcome.STARTED_READY
    assert result.ready_confirmed is True
    assert result.acknowledged is True
    assert result.runtime_pid == 88
    assert controller.calls == [("start", "alap"), ("wait_idle", 3.0)]
    assert speaker.calls == 1


def test_existing_runtime_is_never_restarted_by_wake():
    controller = _Controller(running=True)
    speaker = _Speaker()
    result = WakeRuntimeCoordinator(controller, speaker).activate()
    assert result.outcome is WakeRuntimeOutcome.ALREADY_RUNNING
    assert controller.calls == []
    assert speaker.calls == 0


def test_start_failure_never_plays_ready_ack():
    controller = _Controller(fail=True)
    speaker = _Speaker()
    result = WakeRuntimeCoordinator(controller, speaker).activate()
    assert result.outcome is WakeRuntimeOutcome.START_FAILED
    assert result.ready_confirmed is False
    assert speaker.calls == 0


def test_speaker_failure_does_not_turn_ready_robot_into_start_failure():
    controller = _Controller()
    speaker = _Speaker(fail=True)
    result = WakeRuntimeCoordinator(controller, speaker).activate()
    assert result.outcome is WakeRuntimeOutcome.STARTED_READY
    assert result.ready_confirmed is True
    assert result.acknowledged is False
    assert result.error and result.error.startswith("speaker ")
