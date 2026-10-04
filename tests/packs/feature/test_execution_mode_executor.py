from __future__ import annotations

import sys
import types


def test_agent_route_uses_agent_runner(monkeypatch, tmp_path, capsys) -> None:
    from r2b4_orchestration.execution_mode import ExecutionModeSelector
    from r2b4_orchestration import executor

    module = types.ModuleType("r2b4_orchestration.agent_runner")
    module.run_agent_prompt = lambda *args, **kwargs: types.SimpleNamespace(text="agent answer", action_status="NONE")
    monkeypatch.setitem(sys.modules, "r2b4_orchestration.agent_runner", module)
    monkeypatch.setattr(executor, "speak_text", lambda *args, **kwargs: None)

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


def test_agent_action_failure_returns_nonzero(monkeypatch, tmp_path) -> None:
    from r2b4_orchestration.execution_mode import ExecutionModeSelector
    from r2b4_orchestration import executor

    module = types.ModuleType("r2b4_orchestration.agent_runner")
    module.run_agent_prompt = lambda *args, **kwargs: types.SimpleNamespace(
        text="A kért robotművelet nem fejeződött be.",
        action_status="FAILED:TIMEOUT",
    )
    monkeypatch.setitem(sys.modules, "r2b4_orchestration.agent_runner", module)
    monkeypatch.setattr(executor, "speak_text", lambda *args, **kwargs: None)

    plan = ExecutionModeSelector().select("menj előre 1,2 m-t", source="test")
    assert executor.execute_plan(plan, project_root=tmp_path) == 2

