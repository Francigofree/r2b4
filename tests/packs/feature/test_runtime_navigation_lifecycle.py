"""Host session lifecycle with simulated services; never starts robot hardware."""
from types import SimpleNamespace
import os

import pytest

from v3.operator_controller import OperatorController


@pytest.mark.parametrize("preexisting", [False, True])
@pytest.mark.parametrize("cleanup_status_error", [False, True])
def test_runtime_supervisor_prepares_navigation_and_releases_only_owned_edges(
    tmp_path, monkeypatch, preexisting, cleanup_status_error,
):
    import v3.operator_controller as module

    calls = []

    class Service:
        def __init__(self, name, pid):
            self.name, self.pid = name, pid
            self.running = preexisting

        def status(self, root):
            if cleanup_status_error and self.name == "nav2" and calls[-1:] == ["resident.close"]:
                raise OSError("status unreadable")
            return {"running": self.running, "state": "RUNNING" if self.running else "STOPPED",
                    "pid": self.pid, "process_start": str(self.pid), "ready": self.running}

        def start(self, root):
            calls.append(self.name + ".start")
            self.running = True
            return self.status(root)

        def stop(self, root):
            calls.append(self.name + ".stop")
            self.running = False

    slam, nav2 = Service("slam", 201), Service("nav2", 202)
    controller = OperatorController(tmp_path)
    monkeypatch.setattr(controller, "_runtime_dependencies", lambda: [("slam", slam), ("nav2", nav2)])

    class Resident:
        pid, returncode = 123, None

        def poll(self):
            return self.returncode

        def wait(self, **kwargs):
            calls.append("resident.close")
            self.returncode = 0
            return 0

    monkeypatch.setattr(module.subprocess, "Popen", lambda *a, **kw: calls.append("resident.start") or Resident())
    monkeypatch.setattr(controller, "_wait_fresh_ready", lambda *a, **kw: calls.append("resident.ready") or True)

    assert controller.run_runtime_session(tmp_path / "capture.mcap", "nincs") == 0
    assert calls == ["slam.start", "resident.start", "resident.ready", "nav2.start", "resident.close"] + (
        [] if preexisting else ([] if cleanup_status_error else ["nav2.stop"]) + ["slam.stop"])


def test_failed_nav2_start_closes_resident_before_releasing_slam(tmp_path, monkeypatch):
    import v3.operator_controller as module

    calls = []
    slam = SimpleNamespace(
        status=lambda root: {"running": False, "pid": 201, "process_start": "1"},
        start=lambda root: {"running": True, "state": "RUNNING", "pid": 201, "process_start": "1"},
        stop=lambda root: calls.append("slam.stop"))

    def failed(root):
        raise RuntimeError("NAV2_LIFECYCLE_INACTIVE")

    nav2 = SimpleNamespace(status=lambda root: {"running": False}, start=failed)
    resident = SimpleNamespace(pid=123, returncode=None)
    resident.poll = lambda: resident.returncode
    resident.terminate = lambda: (calls.append("resident.stop"), setattr(resident, "returncode", -15))
    resident.wait = lambda **kwargs: resident.returncode
    controller = OperatorController(tmp_path)
    monkeypatch.setattr(controller, "_runtime_dependencies", lambda: [("slam", slam), ("nav2", nav2)])
    monkeypatch.setattr(controller, "_wait_fresh_ready", lambda *a, **kw: True)
    monkeypatch.setattr(module.subprocess, "Popen", lambda *a, **kw: resident)
    with pytest.raises(RuntimeError, match="NAV2_LIFECYCLE_INACTIVE"):
        controller.run_runtime_session(tmp_path / "capture.mcap", "nincs")
    assert calls == ["resident.stop", "slam.stop"]


@pytest.mark.parametrize("terminal", ["COMPLETED", "FAILED"])
def test_finite_roomcruise_prepares_dependencies_before_behavior_and_checks_terminal_state(tmp_path, monkeypatch, terminal):
    from v3 import interface_cli, launcher_cli, runtime_performance

    calls = []

    class Interface:
        def read(self, resource):
            if resource == "operator.status":
                return {"runtime_running": False}
            calls.append("behavior.state")
            return {"behavior_id": "own", "lifecycle": terminal, "reason": "TEST_END"}

        def execute(self, action, **parameters):
            calls.append((action, parameters))
            return {"behavior_id": "own", "lifecycle": "STARTING"}

        def stop(self):
            calls.append("stop")

    monkeypatch.setattr(interface_cli, "RobotInterface", lambda **kwargs: Interface())
    monkeypatch.setattr(launcher_cli, "project_root", lambda: tmp_path)
    monkeypatch.setattr(runtime_performance, "apply_host_affinity", lambda *args: None)
    assert launcher_cli.main(["rc", "60", "c", "full"]) == (0 if terminal == "COMPLETED" else 1)
    assert calls == [
        ("operator.runtime.start", {"capture_mode": "full", "capture_hz": 5}),
        ("v3.command.explore", {"capture": True, "capture_mode": "full", "capture_hz": 5,
                                "session_owner_pid": os.getpid(), "session_watchdog_s": 90.0,
                                "max_duration_s": 60.0, "completion_on_duration": True}),
        "behavior.state", "stop", ("operator.runtime.stop", {}),
    ]
