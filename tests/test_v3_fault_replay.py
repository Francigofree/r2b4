from pathlib import Path

import pytest

import v3.composition.native_control as native_control_module
from v3.capture import CaptureSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import LifecycleState, TickContext
from v3.engine import TickExecutionError
from v3.execution import (
    EdgeFaultRecord,
    ExecutionBoundary,
    IterableInputSource,
    WriterFailureRecord,
)
from v3.replay import replay_capture
from v3_validation_helpers import RecordingMotorSink, control_config, tick_inputs


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _raising(*_args, **_kwargs):
    raise RuntimeError("injected production layer failure")


@pytest.mark.parametrize(
    ("layer", "owner_name", "method_name"),
    (
        ("L1", None, "acquire"),
        ("L2", "InputAdmission", "__call__"),
        ("L3", "NativeStateEstimator", "__call__"),
        ("L4", "ShadowWorldModel", "__call__"),
        ("L5", "MissionManager", "evaluate"),
        ("L6", "TrajectoryNavigator", "evaluate"),
        ("L7", None, "select_motion"),
        ("L8", "MotionRealizer", "evaluate"),
        ("L9", "OperationalConstraintLayer", "evaluate"),
        ("L10", "DifferentialDriveKinematics", "__call__"),
        ("L11", "WheelActuatorController", "__call__"),
    ),
)
def test_each_native_upstream_fault_replays_on_the_same_production_layer(
    tmp_path,
    monkeypatch,
    layer,
    owner_name,
    method_name,
):
    if owner_name is None:
        monkeypatch.setattr(native_control_module, method_name, _raising)
    else:
        monkeypatch.setattr(
            getattr(native_control_module, owner_name),
            method_name,
            _raising,
        )
    config = control_config()
    sink = CaptureSink(
        f"fault-{layer}",
        configuration={"resolved_control": config},
    )
    ExecutionBoundary(
        NativeControlComposition(RecordingMotorSink(), config)
    ).run(IterableInputSource((tick_inputs(1)[0],)), sink)
    path = tmp_path / f"{layer}.json"
    sink.finalize("FAULT", path)

    replay = replay_capture(path, project_root=PROJECT_ROOT)

    assert replay["status"] == "MATCH"
    assert replay["first_divergence"] is None
    assert replay["first_live_incident"]["layer"] == layer


@pytest.mark.parametrize(
    ("reason", "fault_layer"),
    (
        ("L0_ERROR", "L0"),
        ("COMMAND_GATEWAY_ERROR", "CommandGateway"),
        ("PREFLIGHT_REQUIRED", "ResidentLiveControl"),
        ("SHUTDOWN_INPUT_ERROR", "L0"),
    ),
)
def test_each_pre_input_closure_fault_replays_as_edge_fault_tick(
    tmp_path,
    reason,
    fault_layer,
):
    config = control_config()
    context = TickContext(0, 1_000_000_000)
    composition = NativeControlComposition(RecordingMotorSink(), config)
    result = composition.run_fault_tick(
        context,
        LifecycleState.FAULT,
        reason,
        fault_layer,
    )
    sink = CaptureSink(
        f"edge-{reason}",
        configuration={"resolved_control": config},
    )
    sink.write(
        EdgeFaultRecord(
            context,
            LifecycleState.FAULT,
            reason,
            fault_layer,
            (),
            result,
        )
    )
    path = tmp_path / f"{reason}.json"
    sink.finalize("FAULT", path)

    replay = replay_capture(path, project_root=PROJECT_ROOT)

    assert replay["status"] == "MATCH"
    assert replay["first_live_incident"]["reason"] == reason


def test_l12_writer_failure_replays_as_the_same_failed_commit(tmp_path):
    class FailingWriter:
        def write(self, _command):
            raise OSError("injected motor edge failure")

    config = control_config()
    inputs = tick_inputs(1)[0]
    with pytest.raises(TickExecutionError) as raised:
        NativeControlComposition(FailingWriter(), config).run_tick(inputs)
    failure = raised.value
    sink = CaptureSink(
        "writer-failure",
        configuration={"resolved_control": config},
    )
    sink.write(
        WriterFailureRecord(
            context=inputs.context,
            lifecycle=inputs.lifecycle,
            reason="MOTOR_WRITER_FAILURE",
            attempted_actuation=failure.attempted_actuation,
            inputs=inputs,
            raw_devices=inputs.raw_devices,
        )
    )
    path = tmp_path / "writer-failure.json"
    sink.finalize("FAULT", path)

    replay = replay_capture(path, project_root=PROJECT_ROOT)

    assert replay["status"] == "MATCH"
    assert replay["execution"]["writer_failure_count"] == 1
    assert replay["physical_root_cause"]["status"] == "PROVEN"
