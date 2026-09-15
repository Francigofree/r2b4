import json
from pathlib import Path

import pytest

from v3 import operator_cli
from v3.operator_controller import OperatorController, OperatorError


def _controller(tmp_path: Path) -> OperatorController:
    (tmp_path / "runtime" / "captures").mkdir(parents=True)
    (tmp_path / "conf").mkdir()
    (tmp_path / "conf" / "fizika.json").write_text(
        json.dumps({"nyomtav_szelesseg_m": 0.3557}),
        encoding="utf-8",
    )
    return OperatorController(tmp_path)


def test_wheel_targets_to_twist_uses_canonical_track_width(tmp_path):
    c = _controller(tmp_path)
    v, omega, max_v, max_omega, track = c.wheel_targets_to_twist(0.10, 0.20)
    assert track == pytest.approx(0.3557)
    assert v == pytest.approx(0.15)
    assert omega == pytest.approx((0.20 - 0.10) / 0.3557)
    assert max_v == pytest.approx(0.15)
    assert max_omega <= 1.20


def test_wheel_targets_reject_zero_or_out_of_bounds(tmp_path):
    c = _controller(tmp_path)
    with pytest.raises(OperatorError):
        c.wheel_targets_to_twist(0.0, 0.0)
    with pytest.raises(OperatorError):
        c.wheel_targets_to_twist(0.51, 0.0)


def test_capture_selector_accepts_old_and_new_spelling():
    clean, mode, explicit = operator_cli._extract_capture_selector(
        ["forward", "0.15", "c", "full"]
    )
    assert clean == ["forward", "0.15"]
    assert mode == "full"
    assert explicit is True

    clean, mode, explicit = operator_cli._extract_capture_selector(
        ["roomcruise", "--capture=nincs"]
    )
    assert clean == ["roomcruise"]
    assert mode == "nincs"
    assert explicit is True


def test_legacy_launcher_forms_normalize_to_short_commands():
    assert operator_cli._normalize_legacy(["forward", "start", "0.15"]) == [
        "forward",
        "0.15",
    ]
    assert operator_cli._normalize_legacy(["roomcruise", "start", "nocapture"]) == [
        "roomcruise",
        "--no-trigger",
    ]


def test_operator_api_is_shell_independent(tmp_path):
    c = _controller(tmp_path)
    assert c.root == tmp_path.resolve()
    assert c.status()["runtime_running"] is False


@pytest.fixture
def clock(monkeypatch):
    from v3 import operator_controller as module

    class Clock:
        now = 1.0

        def sleep(self, seconds):
            self.now += seconds

    clock = Clock()
    monkeypatch.setattr(module.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(module.time, "monotonic_ns", lambda: int(clock.now * 1e9))
    monkeypatch.setattr(module.time, "sleep", clock.sleep)
    return clock


def _status(clock, **changes):
    return {
        "state": "RUNNING", "monotonic_ns": int(clock.now * 1e9),
        "ready_for_active": True, "safety_decision": "STOP", "enabled": False,
        "left_output": 0, "right_output": 0, "fault_layer": None,
        "estimate": {"yaw_rad": 0.0}, **changes,
    }


def test_status_does_not_rewrite_stale_runtime_pid_or_create_directories(tmp_path):
    root = tmp_path / "unstarted"
    c = OperatorController(root)
    assert c.status()["runtime_running"] is False
    assert not root.exists()
    c = _controller(tmp_path)
    c.runtime_pid_file.write_text("invalid-pid\n")
    assert c.diagnostics()["runtime_running"] is False
    assert c.runtime_pid_file.read_text() == "invalid-pid\n"


@pytest.mark.parametrize("arguments, method, expected", [
    (["forward", "start", "0.2", "c", "full"], "forward",
     ((0.2,), {"capture": True, "capture_mode": "full"})),
    (["backward", "0.1"], "backward",
     ((0.1,), {"capture": True, "capture_mode": "alap"})),
    (["mozog", "start", "0.1", "-0.1", "nocapture"], "wheels",
     ((0.1, -0.1), {"capture": False, "capture_mode": "alap"})),
    (["wheels", "0.1", "0.2"], "wheels",
     ((0.1, 0.2), {"capture": True, "capture_mode": "alap"})),
    (["roomcruise", "start", "--capture=nincs"], "roomcruise",
     ((), {"capture": True, "capture_mode": "nincs"})),
    (["teleop", "0.15", "-0.2"], "start_teleop",
     ((), {"v_mps": 0.15, "omega_rad_s": -0.2, "max_v_mps": 0.5,
           "max_omega_rad_s": 1.2, "capture": True, "capture_mode": "alap"})),
])
def test_cli_routes_old_and_new_motion_forms_without_hardware(tmp_path, monkeypatch, arguments, method, expected):
    c = _controller(tmp_path)
    calls = []
    monkeypatch.setattr(operator_cli, "OperatorController", lambda **kwargs: c)
    monkeypatch.setattr(c, method, lambda *args, **kwargs: calls.append((args, kwargs)))
    assert operator_cli.main(arguments) == 0
    assert calls == [expected]


@pytest.mark.parametrize("arguments", [["c"], ["c", "bad"], ["c", "full", "--capture=alap"]])
def test_invalid_capture_selector_is_rejected(arguments):
    with pytest.raises(OperatorError):
        operator_cli._extract_capture_selector(arguments)


def test_wait_idle_retries_missing_telemetry_and_accepts_runtime_decision(tmp_path, monkeypatch, clock):
    c = _controller(tmp_path)
    monkeypatch.setattr(c, "_runtime_pid", lambda: 42)
    readings = iter([None, _status(clock)])
    monkeypatch.setattr(c, "_read_status_optional", lambda: next(readings))
    # No command-file coupling and no independent source-health gate.
    c.wait_idle()


@pytest.mark.parametrize("changes", [
    {"monotonic_ns": 0}, {"enabled": True}, {"left_output": 0.1},
    {"safety_decision": "FAULT"}, {"ready_for_active": False},
])
def test_wait_idle_rejects_stale_or_non_idle_status(tmp_path, monkeypatch, clock, changes):
    c = _controller(tmp_path)
    monkeypatch.setattr(c, "_runtime_pid", lambda: 42)
    monkeypatch.setattr(c, "_read_status_optional", lambda: _status(clock, **changes))
    with pytest.raises(OperatorError, match="ready IDLE"):
        c.wait_idle(timeout=0.1)


def test_stop_does_not_signal_unrelated_pid_from_stale_file(tmp_path, monkeypatch):
    from v3 import operator_controller as module
    c = _controller(tmp_path)
    c.command_pid_file.write_text("12345")
    monkeypatch.setattr(c, "_pid_matches", lambda *args: False)
    monkeypatch.setattr(c, "_find_pid", lambda *args: None)
    monkeypatch.setattr(module.os, "kill", lambda *args: pytest.fail("unrelated process signalled"))
    c._stop_control_cli_only()


def test_stop_failure_still_requests_shutdown_when_capture_trigger_fails(tmp_path, monkeypatch):
    c = _controller(tmp_path)
    calls = []

    def fail():
        raise OperatorError("failed")

    monkeypatch.setattr(c, "_stop_command_producers", fail)
    monkeypatch.setattr(c, "_publish_stop", lambda: calls.append("STOP"))
    monkeypatch.setattr(c, "_trigger_movement_capture_if_needed", fail)
    monkeypatch.setattr(c, "_runtime_kill", lambda: calls.append("shutdown"))
    with pytest.raises(OperatorError):
        c.stop()
    assert calls == ["STOP", "shutdown"]


@pytest.mark.parametrize("failure", [OperatorError("no ALLOW"), KeyboardInterrupt()])
def test_motion_start_cleans_up_heartbeat_on_failed_or_interrupted_allow(tmp_path, monkeypatch, failure):
    c = _controller(tmp_path)
    calls = []
    monkeypatch.setattr(c, "ensure_runtime", lambda mode: 42)
    monkeypatch.setattr(c, "stop", lambda **kwargs: calls.append(("stop", kwargs)))
    monkeypatch.setattr(c, "current_capture_mode", lambda: "nincs")
    monkeypatch.setattr(c, "_spawn_control_process", lambda command: calls.append(("spawn", command)) or 43)

    def fail(*args):
        raise failure

    monkeypatch.setattr(c, "_wait_allow", fail)
    with pytest.raises(type(failure)):
        c.forward(capture_mode="nincs")
    assert calls[0] == ("stop", {})
    assert calls[1][1][1:4] == ["-m", "v3.control_cli", "teleop"]
    assert calls[2] == ("stop", {"wait_idle": False})


def test_proba_pause_is_quiet_and_next_phase_refreshes_stop(tmp_path, monkeypatch, clock):
    c = _controller(tmp_path)
    calls = []
    monkeypatch.setattr(c, "_publish_stop", lambda: calls.append("STOP"))
    monkeypatch.setattr(c, "wait_idle", lambda: calls.append("ready"))
    monkeypatch.setattr(c, "_pid_matches", lambda *args: True)
    monkeypatch.setattr(c, "_stop_control_cli_only", lambda: calls.append("stop-producer"))
    monkeypatch.setattr(c, "_spawn_control_process", lambda command: calls.append(command) or 43)
    monkeypatch.setattr(c, "_read_status_optional", lambda: _status(
        clock, safety_decision="ALLOW" if len(calls) >= 3 else "STOP",
        enabled=len(calls) >= 3,
        # Independent degraded capability must not override runtime ALLOW.
        source_health=[{"device_id": "optional", "state": "DEGRADED"}],
    ))
    c._proba_idle(5.0)
    assert clock.now == 6.0
    assert calls == []
    c._run_proba_phase(1, "forward", 0.15, 0.0, 0.0)
    assert calls[:2] == ["STOP", "ready"]
    assert calls[2][1:4] == ["-m", "v3.control_cli", "teleop"]
    assert calls[-2:] == ["stop-producer", "STOP"]
    assert clock.now >= 11.0


def test_proba_runtime_stop_aborts_phase_and_publishes_stop(tmp_path, monkeypatch, clock):
    c = _controller(tmp_path)
    calls = []
    monkeypatch.setattr(c, "_publish_stop", lambda: calls.append("STOP"))
    monkeypatch.setattr(c, "wait_idle", lambda: None)
    monkeypatch.setattr(c, "_pid_matches", lambda *args: True)
    monkeypatch.setattr(c, "_spawn_control_process", lambda command: 43)
    monkeypatch.setattr(c, "_stop_control_cli_only", lambda: calls.append("stop-producer"))
    monkeypatch.setattr(c, "_read_status_optional", lambda: _status(clock, safety_reason="BLOCKED"))
    with pytest.raises(OperatorError, match="motion stopped by safety"):
        c._run_proba_phase(1, "forward", 0.15, 0.0, 0.0)
    assert calls == ["STOP", "stop-producer", "STOP"]


def test_capture_off_skips_test_hub(tmp_path, monkeypatch):
    c = _controller(tmp_path)
    monkeypatch.setattr(c, "_run_test_hub_default", lambda *args: pytest.fail("capture OFF started Test Hub"))
    monkeypatch.setattr(c, "_pid_matches", lambda *args: pytest.fail("capture OFF waited for runtime"))
    c._test_hub_supervisor(tmp_path / "absent.mcap", 42, "nincs")


@pytest.mark.parametrize("mode, capture_args", [
    ("alap", ["--capture-mode", "triggered"]),
    ("full", ["--capture-mode", "append_only"]),
    ("nincs", []),
])
def test_runtime_session_uses_canonical_process_and_capture_mode(tmp_path, monkeypatch, mode, capture_args):
    from v3 import operator_controller as module
    c = _controller(tmp_path)
    calls = []

    class Process:
        pid = 42

        def wait(self):
            return 0

    def spawn(command, **kwargs):
        calls.append((command, kwargs))
        return Process()

    monkeypatch.setattr(module.subprocess, "Popen", spawn)
    monkeypatch.setattr(c, "_test_hub_supervisor", lambda *args: None)
    capture = c.capture_dir / "test.mcap"
    assert c.run_runtime_session(capture, mode) == 0
    command, kwargs = calls[0]
    assert command[:4] == [c.python, "v3_process_runtime.py", "--approval", "native-resident-v3"]
    assert kwargs == {"cwd": c.root}
    if capture_args:
        i = command.index("--capture-mode")
        assert command[i:i + 2] == capture_args
        assert "--capture-path" in command
    else:
        assert len(command) == 4


def test_stop_uses_canonical_cli_defaults(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from v3 import operator_controller as module
    c = _controller(tmp_path)
    calls = []

    def run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(module.subprocess, "run", run)
    c._publish_stop()
    command, kwargs = calls[0]
    assert command[:5] == [c.python, "-m", "v3.control_cli", "stop", "--command-id"]
    assert len(command) == 6  # TTL policy remains in control_cli.
    assert kwargs["cwd"] == c.root


def test_native_capture_finalization_is_detected_and_replayed_by_test_hub(tmp_path):
    from test_v3_mcap_e2e import capture

    c = OperatorController(Path(__file__).resolve().parents[1])
    captured, _ = capture(tmp_path)
    assert captured.complete
    assert c._capture_ready(captured.path)
    c.capture_path_file = tmp_path / "capture_pointer"
    c.capture_path_file.write_text(str(captured.path), encoding="utf-8")
    # Keep this test independent from the repository's transient runtime state.
    c.capture_mode_file = tmp_path / "capture_mode"
    c.capture_mode_file.write_text("alap\n", encoding="utf-8")
    status = c.capture_status()
    assert status['state'] == 'FINALIZED'
    assert status['complete'] is True
    assert status['ticks'] == 12
    assert c._run_test_hub_default(captured.path) == 0
    evidence = c._test_hub_evidence_dir(captured.path)
    agent_view = json.loads((evidence / 'agent_view.json').read_text())
    assert agent_view['replay_status'] == 'MATCH'


@pytest.mark.parametrize('damage', ['loss', 'corruption'])
def test_capture_readiness_rejects_native_integrity_failures(tmp_path, damage):
    from test_v3_mcap_e2e import capture

    c = _controller(tmp_path)
    captured, _ = capture(tmp_path, capacity=4 if damage == 'loss' else 100)
    if damage == 'corruption':
        with captured.path.open('r+b') as stream:
            stream.seek(30)
            value = stream.read(1)
            stream.seek(30)
            stream.write(bytes([value[0] ^ 1]))
    assert not c._capture_ready(captured.path)
