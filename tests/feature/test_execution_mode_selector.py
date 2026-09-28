from __future__ import annotations


def test_default_conversation_stays_gemini() -> None:
    from r2b4_orchestration.execution_mode import ExecutionMode, ExecutionModeSelector

    plan = ExecutionModeSelector().select("Miért kék az ég?", source="test")
    assert plan.mode is ExecutionMode.GEMINI_CHAT
    assert plan.requires_v3 is False


def test_simple_forward_is_direct_v3() -> None:
    from r2b4_orchestration.execution_mode import ExecutionMode, ExecutionModeSelector

    plan = ExecutionModeSelector().select("menj előre", source="test")
    assert plan.mode is ExecutionMode.DIRECT_V3
    assert plan.action_name == "v3.command.forward"
    assert plan.requires_v3 is True


def test_metric_forward_is_er2_not_chat() -> None:
    from r2b4_orchestration.execution_mode import ExecutionMode, ExecutionModeSelector

    plan = ExecutionModeSelector().select("menj előre 1m", source="test")
    assert plan.mode is ExecutionMode.ER2_STREAM
    assert plan.tools is True
    assert plan.requires_v3 is True


def test_exact_turn_is_er2() -> None:
    from r2b4_orchestration.execution_mode import ExecutionMode, ExecutionModeSelector

    plan = ExecutionModeSelector().select("fordulj pontosan 90 fokkal balra", source="test")
    assert plan.mode is ExecutionMode.ER2_STREAM
    assert plan.requires_v3 is True


def test_visual_request_is_observation_and_semantically_v3_free() -> None:
    from r2b4_orchestration.execution_mode import ExecutionMode, ExecutionModeSelector

    plan = ExecutionModeSelector().select("mit látsz magad előtt?", source="test")
    assert plan.mode is ExecutionMode.OBSERVATION
    assert plan.camera is True
    assert plan.tools is False
    assert plan.requires_v3 is False


def test_robot_status_is_host_read() -> None:
    from r2b4_orchestration.execution_mode import ExecutionMode, ExecutionModeSelector

    plan = ExecutionModeSelector().select("mi a robot állapota?", source="test")
    assert plan.mode is ExecutionMode.HOST_READ
    assert plan.capability == "operator.status"
    assert plan.requires_v3 is False


def test_explanation_about_motion_does_not_actuate() -> None:
    from r2b4_orchestration.execution_mode import ExecutionMode, ExecutionModeSelector

    plan = ExecutionModeSelector().select("hogyan menj előre egy métert?", source="test")
    assert plan.mode is ExecutionMode.GEMINI_CHAT


def test_unknown_physical_imperative_never_falls_back_to_chat() -> None:
    from r2b4_orchestration.execution_mode import ExecutionMode, ExecutionModeSelector

    plan = ExecutionModeSelector().select("navigálj az ajtóhoz", source="test")
    assert plan.mode is ExecutionMode.ER2_STREAM


def test_exact_stop_is_canonical_direct_stop() -> None:
    from r2b4_orchestration.execution_mode import ExecutionMode, ExecutionModeSelector

    plan = ExecutionModeSelector().select("állj meg", source="test")
    assert plan.mode is ExecutionMode.DIRECT_V3
    assert plan.action_name == "v3.command.stop"
    assert plan.requires_v3 is False


def test_motion_words_inside_explanation_do_not_actuate() -> None:
    from r2b4_orchestration.execution_mode import ExecutionMode, ExecutionModeSelector

    plan = ExecutionModeSelector().select("mondd el, hogyan menj előre 1 métert biztonságosan", source="test")
    assert plan.mode is ExecutionMode.GEMINI_CHAT
