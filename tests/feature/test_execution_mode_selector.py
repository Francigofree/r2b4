from __future__ import annotations


def test_exact_stop_is_only_local_fast_path() -> None:
    from r2b4_orchestration.execution_mode import ExecutionMode, ExecutionModeSelector
    plan = ExecutionModeSelector().select("állj meg", source="test")
    assert plan.mode is ExecutionMode.DIRECT_V3
    assert plan.action_name == "v3.command.stop"
    assert plan.requires_v3 is False


def test_simple_motion_goes_to_agent_core() -> None:
    from r2b4_orchestration.execution_mode import ExecutionMode, ExecutionModeSelector
    plan = ExecutionModeSelector().select("menj előre", source="test")
    assert plan.mode is ExecutionMode.AGENT
    assert plan.requires_v3 is False


def test_complex_motion_goes_to_agent_core() -> None:
    from r2b4_orchestration.execution_mode import ExecutionMode, ExecutionModeSelector
    plan = ExecutionModeSelector().select("menj az ajtóhoz és kerüld meg a széket", source="test")
    assert plan.mode is ExecutionMode.AGENT


def test_robot_system_question_goes_to_agent_core() -> None:
    from r2b4_orchestration.execution_mode import ExecutionMode, ExecutionModeSelector
    plan = ExecutionModeSelector().select("miért nem kanyarodtál eleget az előző futásban?", source="test")
    assert plan.mode is ExecutionMode.AGENT


def test_visual_request_goes_to_agent_core() -> None:
    from r2b4_orchestration.execution_mode import ExecutionMode, ExecutionModeSelector
    plan = ExecutionModeSelector().select("mit látsz magad előtt?", source="test")
    assert plan.mode is ExecutionMode.AGENT
