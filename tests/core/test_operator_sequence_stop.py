from __future__ import annotations

import os
from types import SimpleNamespace

import pytest

from v3.operator_controller import OperatorController, OperatorError


pytestmark = [pytest.mark.contract, pytest.mark.control]


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
