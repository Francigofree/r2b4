from __future__ import annotations

import os
import signal
import time
from types import SimpleNamespace

import pytest

from v3.operator_controller import OperatorController, OperatorError
from v3.mcap_reader import McapReadError


pytestmark = [pytest.mark.contract, pytest.mark.control]


@pytest.mark.parametrize("capture_result", ["PASS", "FAULT", "INCOMPLETE", "MISSING"])
def test_shutdown_allows_tail_then_verifies_capture_after_native_close(tmp_path, monkeypatch, capture_result):
    import v3.operator_controller as module

    events, stopped, verified = [], [], []
    clock = [0.0]
    running = [True]
    controller = OperatorController(tmp_path, event_sink=events.append)
    capture = tmp_path / "bounded.mcap"
    controller.capture_used_file.parent.mkdir(parents=True, exist_ok=True)
    controller.capture_used_file.write_text("origin=movement-stop\n")
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    monkeypatch.setattr(controller, "_transition_sleep", lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    monkeypatch.setattr(controller, "stop", lambda **kw: stopped.append("STOP"))
    monkeypatch.setattr(controller, "_runtime_pid", lambda: 123)
    monkeypatch.setattr(controller, "_pid_matches", lambda *a: running[0])
    monkeypatch.setattr(controller, "current_capture_mode", lambda: "alap")
    monkeypatch.setattr(controller, "current_capture_path", lambda: capture)

    def shutdown(pid, sig):
        assert stopped == ["STOP"] and sig == signal.SIGTERM
        assert 2.0 <= clock[0] <= 4.0
        assert not capture.exists() and not verified
        assert not any(event.kind == "warning" for event in events)
        running[0] = False
        if capture_result != "MISSING":
            capture.write_bytes(b"native finalized artifact")

    def verify(path):
        assert not running[0]
        verified.append(path)
        if capture_result == "INCOMPLETE":
            raise McapReadError("CAPTURE_INCOMPLETE: POST_WINDOW_INCOMPLETE")
        return {"status": capture_result}

    monkeypatch.setattr(module.os, "kill", shutdown)
    monkeypatch.setattr(controller, "_verified_capture_final", verify)
    controller.runtime_stop()
    warnings = [event.message for event in events if event.kind == "warning"]
    if capture_result in {"PASS", "FAULT"}:
        assert not warnings and verified == [capture]
        assert any(f"runtime status {capture_result}" in event.message for event in events)
    elif capture_result == "INCOMPLETE":
        assert len(warnings) == 1 and "POST_WINDOW_INCOMPLETE" in warnings[0]
    else:
        assert len(warnings) == 1 and "not finalized after runtime shutdown" in warnings[0]


@pytest.mark.parametrize("termination", ["SHUTDOWN_SAFE_LOW", "FAULT_SAFE_LOW"])
def test_finite_stop_racing_runtime_shutdown_requires_verified_clean_close(tmp_path, monkeypatch, termination):
    controller = OperatorController(tmp_path)
    monkeypatch.setattr(controller, "_runtime_pid", lambda: 123)
    monkeypatch.setattr(controller, "_read_status_optional", lambda: {
        "state": "STOPPED", "monotonic_ns": time.monotonic_ns(),
        "report": {"status": "PASS" if termination == "SHUTDOWN_SAFE_LOW" else "FAULT",
                   "termination_class": termination, "fault_layer": None}})
    if termination == "SHUTDOWN_SAFE_LOW":
        controller.wait_idle()
    else:
        with pytest.raises(OperatorError, match="inactive outputs"):
            controller.wait_idle()


def test_runtime_disappearance_during_stop_cannot_hide_unverified_close(tmp_path, monkeypatch):
    controller = OperatorController(tmp_path)
    pids = iter((123, None))
    monkeypatch.setattr(controller, "_runtime_pid", lambda: next(pids))
    monkeypatch.setattr(controller, "_read_status_optional", lambda: None)
    with pytest.raises(OperatorError, match="without verified output-owner close"):
        controller.wait_idle()


@pytest.mark.parametrize("replace_command", [False, True])
def test_stop_during_proba_pause_revokes_future_phases_for_interface_callers(tmp_path, monkeypatch, replace_command):
    """Exercise the real sequence/phase loop with hardware edges replaced."""
    import v3.operator_controller as module

    commands = []
    active = False
    now = 1.0
    stopped_during_pause = False

    class Controller(OperatorController):
        def ensure_runtime(self, *args):
            return 123

        def _runtime_pid(self):
            return None

        def _track_width(self):
            return 0.3

        def current_capture_mode(self):
            return "nincs"

        def _wait_allow(self, *args, **kwargs):
            return True

        def _publish_stop(self):
            pass

        def _find_pid(self, args):
            return None

        def _stop_control_cli_only(self):
            nonlocal active
            active = False

        def _spawn_control_process(self, command, **session):
            nonlocal active
            active = True
            commands.append((command, session))
            return 456

        def _pid_matches(self, pid, args):
            return active

        def _runtime_status(self):
            return {
                "monotonic_ns": int(now * 1e9), "state": "RUNNING",
                "safety_decision": "ALLOW", "enabled": True,
            }

    controller = Controller(tmp_path)
    other_client = Controller(tmp_path)

    def sleep(seconds):
        nonlocal now, stopped_during_pause
        now += seconds
        if seconds == 5.0:
            stopped_during_pause = True
            # Same process, different facade: no CLI-name/PID-kill dependency.
            other_client.stop()
            if replace_command:
                other_client.forward(0.1, capture=False, capture_mode="nincs")

    monkeypatch.setattr(module, "time", SimpleNamespace(
        monotonic=lambda: now, monotonic_ns=lambda: int(now * 1e9),
        time_ns=lambda: int(now * 1e9), sleep=sleep,
    ))
    with pytest.raises(OperatorError, match="cancelled"):
        controller.run_proba(capture_mode="nincs")
    assert stopped_during_pause and len(commands) == (2 if replace_command else 1)
    assert active is replace_command
    assert commands[0][1]["session_owner_pid"] == os.getpid()
    assert commands[0][1]["session_watchdog_s"] > 0


def test_command_producer_owner_exit_and_watchdog_stop_before_next_heartbeat(capsys, monkeypatch):
    from v3 import control_cli

    for reason in ("OWNER_EXITED", "WATCHDOG_TIMEOUT"):
        now = 0
        publications = []
        alive = True

        class Client:
            def publish_stop(self, command_id, **kwargs):
                publications.append("STOP")
                return len(publications)

        def sleep(seconds):
            nonlocal now, alive
            now += int(seconds * 1e9)
            alive = False

        monkeypatch.setattr(control_cli, "_pid_alive", lambda pid: alive)
        control_cli._run_active(
            Client(), lambda command_id: publications.append("ACTIVE") or len(publications),
            command_id="test", ttl_ns=200_000_000, heartbeat_ns=100_000_000,
            owner_pid=123 if reason == "OWNER_EXITED" else None,
            max_runtime_s=0.1 if reason == "WATCHDOG_TIMEOUT" else None,
            monotonic_ns=lambda: now, sleep=sleep,
        )
        assert publications == ["ACTIVE", "STOP"]
        assert reason in capsys.readouterr().out


def test_launcher_tool_cannot_bypass_runtime_or_hardware_ownership(tmp_path, monkeypatch):
    from v3 import host_cli

    tools = tmp_path / "tools"
    tools.mkdir()
    (tmp_path / "v3_process_runtime.py").write_text("# never executed\n")
    (tools / "runtime_alias.py").symlink_to(tmp_path / "v3_process_runtime.py")
    runs = []
    monkeypatch.setattr(host_cli, "_run", lambda command, **kwargs: runs.append(command) or 0)
    for name in ("v3_process_runtime", "runtime_alias", "../v3_process_runtime"):
        with pytest.raises(host_cli.HostCliError):
            host_cli.execute("tool", [name], tmp_path)
    monkeypatch.setattr(OperatorController, "status", lambda self: {"runtime_running": True})
    for name in ("r2b4_imu_control_diag", "r2b4_periferial_control_diag", "v3_mcap_measurement"):
        (tools / f"{name}.py").write_text("# never executed\n")
        with pytest.raises(host_cli.HostCliError, match="owns physical hardware"):
            host_cli.execute("tool", [name], tmp_path)
    assert runs == []
