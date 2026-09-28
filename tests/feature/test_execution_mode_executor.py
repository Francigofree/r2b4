from __future__ import annotations

import sys
import types
from types import SimpleNamespace

import pytest


def test_er2_failure_never_falls_back_to_plain_gemini(monkeypatch, tmp_path) -> None:
    from r2b4_orchestration.execution_mode import ExecutionModeSelector
    from r2b4_orchestration import executor

    plan = ExecutionModeSelector().select("menj előre 1m", source="test")
    plain_called = []

    er2_module = types.ModuleType("r2b4_er2.executor")
    er2_module.run_er2_task = lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("ER2 down"))
    monkeypatch.setitem(sys.modules, "r2b4_er2.executor", er2_module)

    plain_module = types.ModuleType("r2b4_voice.plain_llm")
    plain_module.run_plain_prompt = lambda *args, **kwargs: plain_called.append(True) or 0
    monkeypatch.setitem(sys.modules, "r2b4_voice.plain_llm", plain_module)

    with pytest.raises(RuntimeError, match="ER2 down"):
        executor.execute_plan(plan, project_root=tmp_path)
    assert plain_called == []


def test_host_read_uses_read_only_robot_interface(monkeypatch, tmp_path, capsys) -> None:
    from r2b4_orchestration.execution_mode import ExecutionModeSelector
    from r2b4_orchestration import executor

    calls = []

    class Interface:
        def __init__(self, **kwargs):
            pass

        def read(self, name):
            calls.append(("read", name))
            return {"runtime_running": False}

        def execute(self, *args, **kwargs):
            calls.append(("execute", args, kwargs))
            raise AssertionError("read-only route must not execute")

    v3_pkg = types.ModuleType("v3")
    v3_robot = types.ModuleType("v3.robot_interface")
    v3_robot.RobotInterface = Interface
    monkeypatch.setitem(sys.modules, "v3", v3_pkg)
    monkeypatch.setitem(sys.modules, "v3.robot_interface", v3_robot)
    monkeypatch.setattr(executor, "speak_text", lambda *args, **kwargs: None)

    plan = ExecutionModeSelector().select("mi a robot állapota?", source="test")
    assert executor.execute_plan(plan, project_root=tmp_path) == 0
    assert calls == [("read", "operator.status")]
    assert "V3 jelenleg nem fut" in capsys.readouterr().out


def test_direct_v3_reuses_fresh_action_executor_gate(monkeypatch, tmp_path) -> None:
    from r2b4_orchestration.execution_mode import ExecutionModeSelector
    from r2b4_orchestration import executor

    seen = {}

    class Interface:
        def __init__(self, **kwargs):
            seen["interface_kwargs"] = kwargs

    class Gate:
        def __init__(self, interface, **kwargs):
            seen["gate_kwargs"] = kwargs

        def execute_proposal(self, proposal):
            seen["proposal"] = proposal
            return SimpleNamespace(
                status="EXECUTED",
                executed=True,
                action_name=proposal["name"],
                command_id="cmd-1",
                mission_id="mission-1",
            )

    v3_pkg = types.ModuleType("v3")
    v3_robot = types.ModuleType("v3.robot_interface")
    v3_robot.RobotInterface = Interface
    voice_pkg = types.ModuleType("r2b4_voice")
    action_module = types.ModuleType("r2b4_voice.action_executor")
    action_module.VoiceActionExecutor = Gate
    monkeypatch.setitem(sys.modules, "v3", v3_pkg)
    monkeypatch.setitem(sys.modules, "v3.robot_interface", v3_robot)
    monkeypatch.setitem(sys.modules, "r2b4_voice", voice_pkg)
    monkeypatch.setitem(sys.modules, "r2b4_voice.action_executor", action_module)
    monkeypatch.setattr(executor, "speak_text", lambda *args, **kwargs: None)

    plan = ExecutionModeSelector().select("menj előre", source="test")
    assert executor.execute_plan(plan, project_root=tmp_path) == 0
    assert seen["proposal"] == {"name": "v3.command.forward", "parameters": {}}
    assert seen["gate_kwargs"]["mode"] == "execute"
