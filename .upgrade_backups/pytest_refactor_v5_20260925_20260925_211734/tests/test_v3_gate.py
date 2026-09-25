"""Fast high-signal gate intended as the agent's first validation pass."""

from pathlib import Path

import pytest

from v3.composition.native_control import NativeControlComposition
from v3.execution import ExecutionBoundary, IterableInputSource, MemoryOutputSink
from v3.import_guard import validate_v3_imports
from v3.replay import replay_capture
from v3_process_runtime import load_resident_runtime_config
from v3_validation_helpers import (
    RecordingMotorSink,
    control_config,
    create_explore_capture,
    create_fault_capture,
    tick_inputs,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.gate


def test_gate_import_boundaries_are_clean():
    assert validate_v3_imports(PROJECT_ROOT) == ()


def test_gate_canonical_runtime_config_closes_without_opening_hardware():
    runtime = load_resident_runtime_config(PROJECT_ROOT)
    assert runtime.sensor_inputs is not None
    assert runtime.tick_period_ns > 0
    assert (
        runtime.tick_period_ns
        <= runtime.composition.live_control.max_preflight_age_ns
    )


def test_gate_closed_control_preserves_stop_edges():
    motor = RecordingMotorSink()
    output = MemoryOutputSink()
    summary = ExecutionBoundary(
        NativeControlComposition(motor, control_config())
    ).run(IterableInputSource(tick_inputs(3)), output)

    assert summary.tick_count == 3
    assert len(motor.commands) == 3
    assert motor.commands[0].enabled is False
    assert motor.commands[-1].enabled is False


def test_gate_explore_capture_replays_deterministically(tmp_path):
    capture = create_explore_capture(tmp_path, capture_id="gate-explore")
    result = replay_capture(capture, project_root=PROJECT_ROOT)
    assert result["status"] == "MATCH"
    assert result["determinism"]["repeated_trace_match"] is True


def test_gate_fail_closed_capture_remains_replayable(tmp_path):
    capture = create_fault_capture(tmp_path, capture_id="gate-fault")
    result = replay_capture(capture, project_root=PROJECT_ROOT)
    assert result["status"] == "MATCH"
    assert result["capture"]["status"] == "FAULT"
