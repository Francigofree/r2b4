from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from pathlib import Path

from v3 import control_cli
from v3.adapters.v3_control import V3ControlInterfaceAdapter
from v3.operator_controller import OperatorController


def _controller(tmp_path: Path) -> OperatorController:
    (tmp_path / "runtime" / "captures").mkdir(parents=True)
    (tmp_path / "conf").mkdir()
    (tmp_path / "conf" / "fizika.json").write_text(
        json.dumps({"nyomtav_szelesseg_m": 0.3557}), encoding="utf-8"
    )
    return OperatorController(tmp_path)


def test_active_owner_death_publishes_stop_before_new_heartbeat(monkeypatch, capsys):
    class Client:
        def __init__(self):
            self.stopped = []

        def publish_stop(self, command_id, *, ttl_ns):
            self.stopped.append((command_id, ttl_ns))
            return 7

    client = Client()
    monkeypatch.setattr(control_cli, "_pid_alive", lambda _pid: False)
    publishes = []
    assert control_cli._run_active(
        client, lambda command_id: publishes.append(command_id) or 1,
        command_id="timed-owner", ttl_ns=200_000_000,
        heartbeat_ns=100_000_000, owner_pid=424242,
    ) == 0
    assert publishes == []
    assert client.stopped[0][1] == 200_000_000
    assert json.loads(capsys.readouterr().out)["reason"] == "OWNER_EXITED"


def test_active_watchdog_publishes_stop(capsys):
    class Clock:
        now = 0
        def monotonic_ns(self):
            return self.now
        def sleep(self, seconds):
            self.now += round(seconds * 1e9)

    class Client:
        stopped = False
        def publish_stop(self, _command_id, *, ttl_ns):
            assert ttl_ns == 200_000_000
            self.stopped = True
            return 9

    clock = Clock()
    client = Client()
    publishes = []
    assert control_cli._run_active(
        client, lambda command_id: publishes.append(command_id) or len(publishes),
        command_id="timed-watchdog", ttl_ns=200_000_000,
        heartbeat_ns=100_000_000, max_runtime_s=0.05,
        sleep=clock.sleep, monotonic_ns=clock.monotonic_ns,
    ) == 0
    assert len(publishes) == 1
    assert client.stopped
    assert json.loads(capsys.readouterr().out.splitlines()[-1])["reason"] == "WATCHDOG_TIMEOUT"


def test_live_runtime_status_rejects_stale_file(tmp_path, monkeypatch):
    c = _controller(tmp_path)
    monkeypatch.setattr(c, "_runtime_pid", lambda: 42)
    monkeypatch.setattr(c, "_read_status_optional", lambda: {"state": "RUNNING", "monotonic_ns": 1_000_000_000})
    monkeypatch.setattr("v3.operator_controller.time.monotonic_ns", lambda: 2_000_000_000)
    assert c.live_runtime_status() is None
    monkeypatch.setattr("v3.operator_controller.time.monotonic_ns", lambda: 1_100_000_000)
    assert c.live_runtime_status()["state"] == "RUNNING"


def test_v3_capabilities_do_not_publish_stale_status_as_live():
    class Controller:
        def status(self):
            return {"runtime_running": True, "status": {"state": "RUNNING"}}
        def live_runtime_status(self):
            return None

    caps = V3ControlInterfaceAdapter(Controller()).capabilities()
    assert caps["v3.status"]["available"] is False
    assert caps["v3.safety"]["available"] is False
    assert caps["v3.pose"]["reason"] == "NO_LIVE_RUNTIME_STATUS"


def test_spawn_injects_owner_and_watchdog_before_subcommand(tmp_path, monkeypatch):
    from v3 import operator_controller as module
    c = _controller(tmp_path)
    calls = []

    class Process:
        pid = 123

    monkeypatch.setattr(module.subprocess, "Popen", lambda command, **kwargs: calls.append((command, kwargs)) or Process())
    command = [c.python, "-m", "v3.control_cli", "explore", "--command-id", "x"]
    assert c._spawn_control_process(command, session_owner_pid=77, session_watchdog_s=35.0) == 123
    launched = calls[0][0]
    assert launched[3:7] == ["--owner-pid", "77", "--max-runtime-s", "35.0"]
    assert launched[7] == "explore"


def test_operator_transition_reentrant_and_serialized(tmp_path):
    c1 = _controller(tmp_path)
    c2 = OperatorController(tmp_path)
    entered = threading.Event()
    finished = threading.Event()

    def contender():
        entered.set()
        with c2.operator_transition():
            finished.set()

    with c1.operator_transition():
        with c1.operator_transition():
            thread = threading.Thread(target=contender)
            thread.start()
            assert entered.wait(1.0)
            time.sleep(0.05)
            assert not finished.is_set()
    thread.join(1.0)
    assert finished.is_set()
    assert (tmp_path / "runtime" / ".r2b4_operator.lock").stat().st_mode & 0o777 == 0o600


def test_launcher_unknown_word_cannot_be_shadowed_by_repo_tool(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    fake_root = tmp_path / "fake_repo"
    (fake_root / "tools").mkdir(parents=True)
    (fake_root / "pytest.ini").write_text("[pytest]\n", encoding="utf-8")
    (fake_root / "tools" / "ghost.py").write_text('print("SHADOW_TOOL_RAN")\n', encoding="utf-8")
    env = os.environ.copy()
    env["R2B4_ROOT"] = str(fake_root)
    env["PYTHONPATH"] = str(repo) + os.pathsep + env.get("PYTHONPATH", "")
    result = subprocess.run(
        ["bash", str(repo / "r"), "ghost"], cwd=repo, env=env,
        text=True, capture_output=True, check=False,
    )
    assert result.returncode != 0
    assert "SHADOW_TOOL_RAN" not in result.stdout


def test_launcher_marks_physical_tools_as_exclusive():
    launcher = (Path(__file__).resolve().parents[1] / "r").read_text(encoding="utf-8")
    assert "hardware_guard" in launcher
    assert "v3_sensor_measurement" in launcher
    assert "Direct convenience path for repo tools" not in launcher


def test_v3_adapter_propagates_timed_session_ownership():
    calls = []

    class Controller:
        def forward(self, speed, **kwargs):
            calls.append((speed, kwargs))
            return "ok"

    adapter = V3ControlInterfaceAdapter(Controller())
    assert adapter.execute(
        "v3.command.forward",
        speed_mps=0.2,
        capture=True,
        capture_mode="alap",
        session_owner_pid=55,
        session_watchdog_s=15.0,
    ) == "ok"
    speed, kwargs = calls[0]
    assert speed == 0.2
    assert kwargs["session_owner_pid"] == 55
    assert kwargs["session_watchdog_s"] == 15.0
