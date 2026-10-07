from __future__ import annotations

import sys
import types

import pytest


@pytest.mark.parametrize("journal_available", [False, True])
def test_agent_route_uses_agent_runner(monkeypatch, tmp_path, capsys, journal_available) -> None:
    from r2b4_orchestration.execution_mode import ExecutionModeSelector
    from r2b4_orchestration import executor

    module = types.ModuleType("r2b4_orchestration.agent_runner")
    module.run_agent_prompt = lambda *args, **kwargs: types.SimpleNamespace(text="agent answer", action_status="NONE")
    monkeypatch.setitem(sys.modules, "r2b4_orchestration.agent_runner", module)
    monkeypatch.setattr(executor, "speak_text", lambda *args, **kwargs: None)
    if not journal_available:
        (tmp_path / "runtime").write_text("not a directory")

    plan = ExecutionModeSelector().select("miért nem kanyarodsz?", source="test")
    assert executor.execute_plan(plan, project_root=tmp_path) == 0
    assert "agent answer" in capsys.readouterr().out


def test_exact_stop_stays_direct_v3(monkeypatch, tmp_path) -> None:
    from r2b4_orchestration.execution_mode import ExecutionModeSelector
    from r2b4_orchestration import executor

    seen = []
    class Interface:
        def __init__(self, **kwargs): pass
        def execute(self, name, **kwargs): seen.append((name, kwargs)); return {}

    v3_pkg = types.ModuleType("v3")
    v3_robot = types.ModuleType("v3.robot_interface")
    v3_robot.RobotInterface = Interface
    monkeypatch.setitem(sys.modules, "v3", v3_pkg)
    monkeypatch.setitem(sys.modules, "v3.robot_interface", v3_robot)
    monkeypatch.setattr(executor, "speak_text", lambda *args, **kwargs: None)

    plan = ExecutionModeSelector().select("stop", source="test")
    assert executor.execute_plan(plan, project_root=tmp_path) == 0
    assert seen == [("v3.command.stop", {})]


@pytest.mark.parametrize("status", ["FAILED:TIMEOUT", "CANCELLED:STOP", "INTERRUPTED:RUNTIME_RESTART",
                                    "ACTIVE:GOAL_RESULT_UNCONFIRMED"])
def test_agent_action_failure_returns_nonzero(monkeypatch, tmp_path, status) -> None:
    from r2b4_orchestration.execution_mode import ExecutionModeSelector
    from r2b4_orchestration import executor

    module = types.ModuleType("r2b4_orchestration.agent_runner")
    module.run_agent_prompt = lambda *args, **kwargs: types.SimpleNamespace(
        text="A kért robotművelet nem fejeződött be.",
        action_status=status,
    )
    monkeypatch.setitem(sys.modules, "r2b4_orchestration.agent_runner", module)
    monkeypatch.setattr(executor, "speak_text", lambda *args, **kwargs: None)

    plan = ExecutionModeSelector().select("menj előre 1,2 m-t", source="test")
    assert executor.execute_plan(plan, project_root=tmp_path) == 2


def test_exact_stop_precedes_failing_route_evidence(monkeypatch, tmp_path, capsys) -> None:
    from r2b4_orchestration.execution_mode import RouteEvidenceJournal
    from r2b4_orchestration import executor
    import v3.robot_interface

    deliveries = []
    class Interface:
        def execute(self, action, **parameters):
            deliveries.append((action, parameters))
    monkeypatch.setattr(v3.robot_interface, "RobotInterface", lambda **_kwargs: Interface())
    monkeypatch.setattr(executor, "speak_text", lambda *args, **kwargs: None)
    (tmp_path / "runtime").write_text("not a directory")
    emit = RouteEvidenceJournal.emit
    events = []
    def check_delivery(journal, event, plan, **fields):
        assert deliveries == [("v3.command.stop", {})]
        events.append(event)
        return emit(journal, event, plan, **fields)
    monkeypatch.setattr(RouteEvidenceJournal, "emit", check_delivery)

    assert executor.execute_text("állj meg!", project_root=tmp_path) == 0
    assert deliveries == [("v3.command.stop", {})]
    assert events == ["ROUTE_SELECTED", "ROUTE_EXECUTION_START", "ROUTE_EXECUTION_COMPLETE"]
    assert "ROUTE_EVIDENCE_WRITE_FAILED" in capsys.readouterr().err


def test_route_journal_reports_evidence_loss_after_storage_recovers(tmp_path, capsys) -> None:
    import json
    from r2b4_orchestration.execution_mode import ExecutionModeSelector, RouteEvidenceJournal

    obstacle = tmp_path / "runtime"
    obstacle.write_text("not a directory")
    journal = RouteEvidenceJournal(tmp_path)
    plan = ExecutionModeSelector().select("stop")
    journal.emit("ROUTE_SELECTED", plan)
    assert journal.evidence_dropped == 1
    assert journal.last_error.startswith("ROUTE_EVIDENCE_WRITE_FAILED:")
    assert "ROUTE_EVIDENCE_WRITE_FAILED" in capsys.readouterr().err

    obstacle.unlink()
    journal.emit("ROUTE_EXECUTION_COMPLETE", plan)
    stored = json.loads(journal.path.read_text())
    assert stored["evidence_dropped"] == 1
    assert stored["event"] == "ROUTE_EXECUTION_COMPLETE"
