import ast
from dataclasses import replace
from pathlib import Path

import pytest

from v3.composition.native_control import NativeControlComposition
from v3.contracts import TickContext
from v3.execution import ExecutionBoundary, IterableInputSource, MemoryOutputSink
from v3_validation_helpers import RecordingMotorSink, control_config, tick_inputs


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_execution_boundary_connects_closed_input_production_and_passive_sink():
    inputs = tick_inputs(3)
    motor_sink = RecordingMotorSink()
    output_sink = MemoryOutputSink()

    summary = ExecutionBoundary(
        NativeControlComposition(motor_sink, control_config())
    ).run(IterableInputSource(inputs), output_sink)

    assert summary.tick_count == 3
    assert summary.first_tick_id == 0
    assert summary.last_tick_id == 2
    assert tuple(record.inputs for record in output_sink.records) == inputs
    assert len(motor_sink.commands) == 3
    assert tuple(record.result.final_actuation for record in output_sink.records) == tuple(
        motor_sink.commands
    )


def test_execution_boundary_rejects_a_result_from_another_tick_context():
    inputs = tick_inputs(1)
    production = NativeControlComposition(RecordingMotorSink(), control_config())
    result = production.run_tick(inputs[0])
    wrong = replace(
        result,
        trace=replace(result.trace, context=TickContext(99, 99)),
    )

    class WrongContextProduction:
        def run_tick(self, _inputs):
            return wrong

    with pytest.raises(ValueError, match="context differs"):
        ExecutionBoundary(WrongContextProduction()).run(
            IterableInputSource(inputs),
            MemoryOutputSink(),
        )


def test_offline_validation_modules_import_no_live_hardware_authority():
    forbidden = (
        "v3.adapters.live",
        "v3.runtime",
        "RPi",
        "gpiozero",
        "pigpio",
    )
    for relative in ("v3/execution.py", "v3/capture.py", "v3/replay.py", "v3/test_hub.py"):
        tree = ast.parse((PROJECT_ROOT / relative).read_text(encoding="utf-8"))
        imports = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                imports.append(node.module or "")
        assert not any(
            imported.startswith(prefix)
            for imported in imports
            for prefix in forbidden
        ), relative
