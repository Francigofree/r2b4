from dataclasses import dataclass
from pathlib import Path

import pytest

from v3.adapters.fake_edges import FakeCommandGateway, FakeHal
from v3.composition import LifecycleTransitionError, StopOnlyComposition
from v3.contracts import (
    ActuatorRequest,
    CommandMode,
    CommandRequest,
    DeviceHealth,
    DeviceHealthState,
    FinalActuation,
    LifecycleState,
    SafetyDecision,
    TickContext,
)
from v3.engine import TickExecutionError
from v3.layers.l12_safety_final import FinalSafetyGate


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _assert_zero(command: FinalActuation) -> None:
    assert command.enabled is False
    assert command.left_output == command.right_output == 0.0
    assert command.safety_decision in (SafetyDecision.STOP, SafetyDecision.FAULT)


def test_v3_l10_is_the_only_v3_wheel_setpoint_producer():
    producers = []
    for path in (PROJECT_ROOT / "v3").rglob("*.py"):
        if "WheelVelocitySetpoint(" in path.read_text(encoding="utf-8"):
            producers.append(path.relative_to(PROJECT_ROOT).as_posix())

    assert producers == ["v3/layers/l10_chassis_control.py"]


def test_complete_stop_only_pipeline_is_typed_sequential_and_never_active():
    hal = FakeHal()
    runtime = StopOnlyComposition(hal, FakeCommandGateway(), hal)

    booting = runtime.tick(1_000)
    runtime.enter_idle()
    idle = runtime.tick(2_000)
    runtime.shutdown()
    shutdown = runtime.tick(3_000)

    assert not hasattr(runtime, "activate")
    assert tuple(record.layer for record in idle.trace.layers) == tuple(
        f"L{index}" for index in range(1, 13)
    )
    assert all(record.output.context == idle.trace.context for record in idle.trace.layers)
    layer_outputs = {record.layer: record.output for record in idle.trace.layers}
    assert layer_outputs["L3"].frame_id == "R2B4_BOOT_ROBOT_MAP"
    assert layer_outputs["L5"].mode is CommandMode.STOP
    assert layer_outputs["L6"].route == ()
    assert layer_outputs["L7"].kind.value == "STOP"
    assert layer_outputs["L8"].requested_v_mps == 0.0
    assert layer_outputs["L8"].requested_omega_rad_s == 0.0
    assert layer_outputs["L9"].allowed_v_mps == 0.0
    assert layer_outputs["L9"].allowed_omega_rad_s == 0.0
    assert layer_outputs["L10"].left_mps == layer_outputs["L10"].right_mps == 0.0
    assert layer_outputs["L11"].left_normalized == 0.0
    assert layer_outputs["L11"].right_normalized == 0.0
    for result in (booting, idle, shutdown):
        _assert_zero(result.final_actuation)
        assert result.final_actuation.reason == "NOT_ACTIVE"
    assert hal.writes == (
        booting.final_actuation,
        idle.final_actuation,
        shutdown.final_actuation,
    )


def test_stop_only_lifecycle_cannot_leave_fault_or_shutdown_for_idle():
    first = StopOnlyComposition(FakeHal(), FakeCommandGateway(), FakeHal())
    first.shutdown()
    with pytest.raises(LifecycleTransitionError):
        first.enter_idle()

    failing_hal = FakeHal(fail_read_ticks=frozenset({0}))
    second = StopOnlyComposition(failing_hal, FakeCommandGateway(), failing_hal)
    second.tick(1_000)
    with pytest.raises(LifecycleTransitionError):
        second.enter_idle()


@pytest.mark.parametrize(
    ("hal", "gateway", "reason", "fault_layer"),
    (
        (FakeHal(fail_read_ticks=frozenset({0})), FakeCommandGateway(), "L0_ERROR", "L0"),
        (
            FakeHal(),
            FakeCommandGateway(fail_ticks=frozenset({0})),
            "COMMAND_GATEWAY_ERROR",
            "CommandGateway",
        ),
    ),
)
def test_edge_snapshot_failure_commits_one_zero_fault(
    hal: FakeHal,
    gateway: FakeCommandGateway,
    reason: str,
    fault_layer: str,
):
    runtime = StopOnlyComposition(hal, gateway, hal)

    result = runtime.tick(1_000)

    _assert_zero(result.final_actuation)
    assert result.final_actuation.safety_decision is SafetyDecision.FAULT
    assert result.final_actuation.reason == reason
    assert result.trace.fault_layer == fault_layer
    assert tuple(record.layer for record in result.trace.layers) == ("L12",)
    assert runtime.lifecycle is LifecycleState.FAULT
    assert hal.writes == (result.final_actuation,)


@dataclass(frozen=True)
class WrongContextGateway:
    def snapshot(self, context: TickContext) -> CommandRequest:
        wrong = TickContext(context.tick_id + 1, context.monotonic_ns)
        return CommandRequest(wrong, "wrong-context", CommandMode.STOP, (), wrong.tick_id)


def test_mismatched_edge_context_fails_closed_before_l1():
    hal = FakeHal()
    runtime = StopOnlyComposition(hal, WrongContextGateway(), hal)

    result = runtime.tick(1_000)

    _assert_zero(result.final_actuation)
    assert result.final_actuation.reason == "INVALID_COMMAND_SNAPSHOT"
    assert result.trace.fault_layer == "CommandGateway"
    assert tuple(record.layer for record in result.trace.layers) == ("L12",)


@pytest.mark.parametrize(
    ("health_state", "reason", "decision"),
    (
        (DeviceHealthState.FAILED, "CRITICAL_DEVICE_FAILED", SafetyDecision.FAULT),
        (DeviceHealthState.UNKNOWN, "CRITICAL_DEVICE_UNKNOWN", SafetyDecision.STOP),
        (DeviceHealthState.DEGRADED, "CRITICAL_DEVICE_DEGRADED", SafetyDecision.STOP),
    ),
)
def test_critical_device_health_never_produces_output(
    health_state: DeviceHealthState,
    reason: str,
    decision: SafetyDecision,
):
    health = (DeviceHealth("fake-motor-driver", health_state, "injected"),)
    hal = FakeHal(device_health=health)
    runtime = StopOnlyComposition(hal, FakeCommandGateway(), hal)

    result = runtime.tick(1_000)

    _assert_zero(result.final_actuation)
    assert result.final_actuation.reason == reason
    assert result.final_actuation.safety_decision is decision
    assert hal.writes == (result.final_actuation,)


def test_missing_and_mismatched_actuator_requests_latch_zero_fault():
    context = TickContext(0, 1_000)
    other_context = TickContext(1, 2_000)

    for request, reason in (
        (None, "MISSING_ACTUATOR_REQUEST"),
        (ActuatorRequest(other_context, 0.0, 0.0), "REQUEST_CONTEXT_MISMATCH"),
    ):
        hal = FakeHal()
        gate = FinalSafetyGate(hal)
        result = gate.finalize(context, request, (), LifecycleState.ACTIVE, None)

        _assert_zero(result)
        assert result.safety_decision is SafetyDecision.FAULT
        assert result.reason == reason
        assert gate.fault_latched is True
        assert hal.writes == (result,)


def test_fault_latch_keeps_later_healthy_request_at_zero():
    first_context = TickContext(0, 1_000)
    second_context = TickContext(1, 2_000)
    hal = FakeHal()
    gate = FinalSafetyGate(hal)

    failed = gate.finalize(first_context, None, (), LifecycleState.ACTIVE, None)
    latched = gate.finalize(
        second_context,
        ActuatorRequest(second_context, 0.5, 0.5),
        (),
        LifecycleState.ACTIVE,
        None,
    )

    _assert_zero(failed)
    _assert_zero(latched)
    assert latched.reason == "FAULT_LATCHED"


def test_writer_failure_is_single_zero_attempt_and_faults_lifecycle():
    hal = FakeHal(fail_write_ticks=frozenset({0}))
    runtime = StopOnlyComposition(hal, FakeCommandGateway(), hal)

    with pytest.raises(TickExecutionError, match="L12"):
        runtime.tick(1_000)

    assert len(hal.writes) == 1
    _assert_zero(hal.writes[0])
    assert runtime.lifecycle is LifecycleState.FAULT
