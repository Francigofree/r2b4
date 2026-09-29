from __future__ import annotations

import json
import os
from types import SimpleNamespace

from v3 import host_cli, interface_cli, launcher_cli
from v3.operator_controller import OperatorEvent


def test_help_discovery_and_typo_never_start_robot_or_host_tools(monkeypatch, capsys):
    def unexpected(*args, **kwargs):
        raise AssertionError("help/discovery must not start an operation")

    monkeypatch.setattr(interface_cli, "RobotInterface", unexpected)
    monkeypatch.setattr(host_cli, "_run", unexpected)
    monkeypatch.setattr(host_cli, "install", unexpected)
    for args in (
        [], ["help", "fp"], ["f", "--help"], ["help", "rt", "start"],
        ["help", "er2", "stream"], ["help", "cpu"], ["install", "--help"],
        ["help", "test"], ["help", "git"], ["tool", "--help"],
    ):
        assert launcher_cli.main(args) == 0
        assert capsys.readouterr().out

    assert launcher_cli.main(["commands", "--json"]) == 0
    catalog = json.loads(capsys.readouterr().out)
    forward = next(item for item in catalog["robot"] if item["name"] == "forward")
    assert forward["aliases"] == ["f"] and forward["motion"] is True
    assert "r forward" in forward["usage"]
    assert forward["description"]
    assert {item["name"] for item in catalog["local_details"]} == set(catalog["local"])

    assert launcher_cli.main(["statuz"]) == 2
    captured = capsys.readouterr()
    assert captured.out == "" and "r status" in captured.err
    assert launcher_cli.main(["t", "10"]) == 2
    assert "required" in capsys.readouterr().err


def test_host_passthrough_preserves_arguments_aliases_and_exit_code(monkeypatch, capsys):
    calls = []
    monkeypatch.setattr(host_cli, "_run", lambda command, **kwargs: calls.append((command, kwargs["root"])) or 17)
    root = launcher_cli.project_root()
    for args, suffix in (
        (["git", "log", "--format=%s", "--", "a b.py"], ["log", "--format=%s", "--", "a b.py"]),
        (["git", "--help"], ["--help"]),
        (["tests", "-k", "foo or bar"], ["-m", "pytest", "-k", "foo or bar"]),
        (["pytest", "--help"], ["-m", "pytest", "--help"]),
    ):
        assert launcher_cli.main(args) == 17
        command, actual_root = calls.pop()
        assert command[-len(suffix):] == suffix and actual_root == root
    assert capsys.readouterr().out == ""


def test_robot_json_output_and_project_root_reach_the_interface(tmp_path, monkeypatch, capsys):
    (tmp_path / "v3").mkdir()
    (tmp_path / "pytest.ini").touch()
    monkeypatch.setattr("v3.runtime_performance.apply_host_affinity", lambda *args: ())
    monkeypatch.setenv("R2B4_ROOT", str(tmp_path))
    requests = []

    class Interface:
        def __init__(self, *, project_root, event_sink):
            assert project_root == tmp_path
            self.event_sink = event_sink

        def read(self, resource):
            if resource == "operator.status":
                return {"runtime_running": False, "capture_mode": "alap", "capture_hz": 10}
            return {"resource": resource}

        def execute(self, action, **parameters):
            requests.append((action, parameters))
            self.event_sink(OperatorEvent(kind="info", message="operation progress"))
            return {"status": "DONE", "action": action}

        def capabilities(self):
            return {"capabilities": {}}

        def stop(self):
            return {"status": "STOPPED"}

    monkeypatch.setattr(interface_cli, "RobotInterface", Interface)
    for args in (
        ["--json", "s"], ["s", "--json"], ["rt", "status", "--json"],
        ["rt", "diag", "--json"], ["d", "--json"], ["caps", "--json"],
        ["cap", "status", "--json"], ["sys", "--json"], ["x", "--json"],
        ["th", "run", "--json", "--replay", "off"],
    ):
        assert launcher_cli.main(args) == 0
        captured = capsys.readouterr()
        result = json.loads(captured.out)  # Rejects progress text and duplicate documents.
        if "s" in args or args[:2] == ["rt", "status"]:
            assert result["runtime"] == "STOPPED"
        if args[:2] == ["th", "run"]:
            assert "operation progress" in captured.err
            assert result["action"] == "testhub.run"
    assert requests == [("testhub.run", {
        "capture": None, "hz": 5, "replay": "off", "no_sweep": False, "pytest_scope": "off",
    })]


def test_motion_capture_stop_and_runtime_ownership_survive_launcher_changes(monkeypatch, capsys):
    calls = []
    resident = False
    interrupt = False
    now = 0.0

    def sleep(seconds):
        nonlocal now
        if interrupt:
            raise KeyboardInterrupt
        now += seconds

    class Interface:
        def __init__(self, **kwargs):
            pass

        def read(self, resource):
            if resource == "operator.status":
                return {"runtime_running": resident, "capture_mode": "full", "capture_hz": 5}
            assert resource == "testhub.status"
            return {"state": "IDLE"}

        def execute(self, action, **parameters):
            calls.append((action, parameters))
            return {"status": "DONE"}

        def stop(self):
            calls.append(("STOP", {}))

    monkeypatch.setattr(interface_cli, "RobotInterface", Interface)
    monkeypatch.setattr(interface_cli, "time", SimpleNamespace(monotonic=lambda: now, sleep=sleep))
    for resident, interrupt in ((False, False), (True, False), (False, True)):
        calls.clear()
        assert launcher_cli.main(["rc", "0.1", "c", "nincs", "c", "50", "nc", "--json"]) == 0
        captured = capsys.readouterr()
        result = json.loads(captured.out)
        assert result["status"] == ("INTERRUPTED" if interrupt else "FINISHED")
        expected = ["v3.command.explore", "STOP"]
        if not resident:
            expected.append("operator.runtime.stop")
        assert [action for action, _ in calls] == expected
        params = calls[0][1]
        assert params["capture"] is False
        assert params["capture_hz"] == (5 if resident else 50)
        assert params["capture_mode"] == ("full" if resident else "nincs")
        assert params["session_owner_pid"] == os.getpid()
        assert params["session_watchdog_s"] > 0.1
        assert "R2B4:" in captured.err

    calls.clear()
    assert launcher_cli.main(["--json", "f", "0", "0.12", "--capture-mode=nincs"]) == 0
    assert json.loads(capsys.readouterr().out)["status"] == "ACTIVE"
    assert calls == [("v3.command.forward", {
        "speed_mps": 0.12, "capture": True, "capture_mode": "nincs", "capture_hz": 10,
    })]
