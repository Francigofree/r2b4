from dataclasses import dataclass
from pathlib import Path
import pytest
from v3.adapters.fake_edges import FakeCommandGateway, FakeHal
from v3.composition import LifecycleTransitionError, StopOnlyComposition
from v3.contracts import ActuatorRequest, CommandMode, CommandRequest, DeviceHealth, DeviceHealthState, FinalActuation, LifecycleState, SafetyDecision, TickContext
from v3.engine import TickExecutionError
from v3.layers.l12_safety_final import FinalSafetyGate
PROJECT_ROOT = Path(__import__('os').environ['R2B4_ROOT']).resolve() if __import__('os').environ.get('R2B4_ROOT') else next((p for p in Path(__file__).resolve().parents if (p / 'conf' / 'hardver.json').is_file() and (p / 'v3').is_dir()), Path.cwd())

def _assert_zero(command: FinalActuation) -> None:
    assert command.enabled is False
    assert command.left_output == command.right_output == 0.0
    assert command.safety_decision in (SafetyDecision.STOP, SafetyDecision.FAULT)

def test_stop_only_lifecycle_cannot_leave_fault_or_shutdown_for_idle():
    first = StopOnlyComposition(FakeHal(), FakeCommandGateway(), FakeHal())
    first.shutdown()
    with pytest.raises(LifecycleTransitionError):
        first.enter_idle()
    failing_hal = FakeHal(fail_read_ticks=frozenset({0}))
    second = StopOnlyComposition(failing_hal, FakeCommandGateway(), failing_hal)
    second.tick(1000)
    with pytest.raises(LifecycleTransitionError):
        second.enter_idle()

@pytest.mark.parametrize(('hal', 'gateway', 'reason', 'fault_layer'), ((FakeHal(fail_read_ticks=frozenset({0})), FakeCommandGateway(), 'L0_ERROR', 'L0'), (FakeHal(), FakeCommandGateway(fail_ticks=frozenset({0})), 'COMMAND_GATEWAY_ERROR', 'CommandGateway')))
def test_edge_snapshot_failure_commits_one_zero_fault(hal: FakeHal, gateway: FakeCommandGateway, reason: str, fault_layer: str):
    runtime = StopOnlyComposition(hal, gateway, hal)
    result = runtime.tick(1000)
    _assert_zero(result.final_actuation)
    assert result.final_actuation.safety_decision is SafetyDecision.FAULT
    assert result.final_actuation.reason == reason
    assert result.trace.fault_layer == fault_layer
    assert tuple((record.layer for record in result.trace.layers)) == ('L12',)
    assert runtime.lifecycle is LifecycleState.FAULT
    assert hal.writes == (result.final_actuation,)

@dataclass(frozen=True)
class WrongContextGateway:

    def snapshot(self, context: TickContext) -> CommandRequest:
        wrong = TickContext(context.tick_id + 1, context.monotonic_ns)
        return CommandRequest(wrong, 'wrong-context', CommandMode.STOP, (), wrong.tick_id)

def test_mismatched_edge_context_fails_closed_before_l1():
    hal = FakeHal()
    runtime = StopOnlyComposition(hal, WrongContextGateway(), hal)
    result = runtime.tick(1000)
    _assert_zero(result.final_actuation)
    assert result.final_actuation.reason == 'INVALID_COMMAND_SNAPSHOT'
    assert result.trace.fault_layer == 'CommandGateway'
    assert tuple((record.layer for record in result.trace.layers)) == ('L12',)

def test_missing_and_mismatched_actuator_requests_latch_zero_fault():
    context = TickContext(0, 1000)
    other_context = TickContext(1, 2000)
    for request, reason in ((None, 'MISSING_ACTUATOR_REQUEST'), (ActuatorRequest(other_context, 0.0, 0.0), 'REQUEST_CONTEXT_MISMATCH')):
        hal = FakeHal()
        gate = FinalSafetyGate(hal)
        result = gate.finalize(context, request, (), LifecycleState.ACTIVE, None)
        _assert_zero(result)
        assert result.safety_decision is SafetyDecision.FAULT
        assert result.reason == reason
        assert gate.fault_latched is True
        assert hal.writes == (result,)

def test_fault_latch_keeps_later_healthy_request_at_zero():
    first_context = TickContext(0, 1000)
    second_context = TickContext(1, 2000)
    hal = FakeHal()
    gate = FinalSafetyGate(hal)
    failed = gate.finalize(first_context, None, (), LifecycleState.ACTIVE, None)
    latched = gate.finalize(second_context, ActuatorRequest(second_context, 0.5, 0.5), (), LifecycleState.ACTIVE, None)
    _assert_zero(failed)
    _assert_zero(latched)
    assert latched.reason == 'FAULT_LATCHED'

def test_writer_failure_is_single_zero_attempt_and_faults_lifecycle():
    hal = FakeHal(fail_write_ticks=frozenset({0}))
    runtime = StopOnlyComposition(hal, FakeCommandGateway(), hal)
    with pytest.raises(TickExecutionError, match='L12'):
        runtime.tick(1000)
    assert len(hal.writes) == 1
    _assert_zero(hal.writes[0])
    assert runtime.lifecycle is LifecycleState.FAULT
