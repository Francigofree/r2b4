"""P0/P1 invariants at the real closure, mission and process boundaries."""
from dataclasses import replace

import pytest

from v3.async_capability import CompletionTiming, WorkerIdentity
from v3.capture import CaptureSink, write_capture
from v3.composition.native_control import NativeControlComposition
from v3.contracts import CommandMode, NavigationStatus, TickContext, Waypoint
from v3.contracts.planner import PlannerCompletion, PlannerInput
from v3.execution import ExecutionRecord
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import (
    InlineTrajectoryRolloutBackend, NavigationConfig, TrajectoryNavigator,
    TrajectoryRolloutComputer,
)
from v3.replay import replay_capture
from test_v3_async_completion_boundary_fix import _explore_values, _retime
from test_v3_async_l6_planner import _estimate, _mission, _world
from test_v3_follow_person import _person
from v3_validation_helpers import (
    PROJECT_ROOT, RecordingMotorSink, configuration_documents, control_config,
)


class TimedBackend(InlineTrajectoryRolloutBackend):
    """Controlled worker/collector clock; real pure planner computation."""

    def __init__(self, config):
        super().__init__(config)
        self.requests = {}
        self.timing = None
        self.deadlines = 0

    def submit(self, request):
        request_id = super().submit(request)
        self.requests[request_id] = request
        return request_id

    def request_identity(self, request_id, source_context):
        return WorkerIdentity(7, request_id, source_context)

    def take_completion(self, request_id, *, visible_ns, transport_timeout_ns):
        if self.timing is None or self.timing.collector_received_ns > visible_ns:
            return None
        result = super().take(request_id)
        if result is None:
            return None
        return PlannerCompletion(self.request_identity(request_id, result.source_context), self.timing, result)

    def note_deadline_missed(self):
        self.deadlines += 1


@pytest.mark.parametrize("completed_ms,late", [(278, False), (300, False), (301, True)])
def test_worker_completion_decides_deadline_even_when_closure_is_later(completed_ms, late):
    config = control_config()
    backend = TimedBackend(config.navigation)
    motor = RecordingMotorSink()
    production = NativeControlComposition(motor, config, trajectory_rollout_backend=backend)
    values = _explore_values()
    try:
        for value in values[:2]:
            production.run_tick(production.close_inputs(value))
        submit_ns = values[1].context.monotonic_ns + 1_000_000
        production.dispatch_pending_planner_request(submit_ns)
        backend.timing = CompletionTiming(submit_ns, submit_ns + 2_000_000,
                                          submit_ns + completed_ms * 1_000_000,
                                          submit_ns + 330_000_000)
        closed = production.close_inputs(_retime(values[2], submit_ns + 335_000_000))
        assert bool(closed.planner_input.error) is late
        result = production.run_tick(closed)
        assert result.trace.fault_layer is None
        assert backend.deadlines == int(late)
        if late:
            assert result.final_actuation.left_output == result.final_actuation.right_output == 0
            assert production._navigation.pending_rollout_request is not None
        else:
            assert closed.planner_input.result is not None
            assert production.checkpoint().navigation.last_replan_ns == values[1].context.monotonic_ns
    finally:
        production.close()


def test_collector_delay_preserves_success_and_closed_tick_immutability():
    config = control_config()
    backend = TimedBackend(config.navigation)
    production = NativeControlComposition(RecordingMotorSink(), config, trajectory_rollout_backend=backend)
    values = _explore_values()
    try:
        for value in values[:2]:
            production.run_tick(production.close_inputs(value))
        submit_ns = values[1].context.monotonic_ns
        production.dispatch_pending_planner_request(submit_ns)
        backend.timing = CompletionTiming(submit_ns, submit_ns, submit_ns + 278_000_000, submit_ns + 330_000_000)
        before = production.close_inputs(_retime(values[2], submit_ns + 320_000_000))
        assert before.planner_input.result is None
        production.run_tick(before)
        after = production.close_inputs(_retime(values[3], submit_ns + 335_000_000))
        assert after.planner_input.result is not None
        assert after.planner_input.error is None
        assert before.planner_input.result is None
    finally:
        production.close()


@pytest.mark.parametrize("mode", [CommandMode.EXPLORE, CommandMode.NAVIGATE, CommandMode.FOLLOW_PERSON])
def test_each_mission_preserves_identity_during_stale_hold_and_fresh_resume(mode):
    config = NavigationConfig()
    navigator = TrajectoryNavigator(config, completion_inputs=True, max_plan_age_ns=350_000_000)
    computer = TrajectoryRolloutComputer(config)
    manager = MissionManager()

    def evaluate(tick, ns, completion=None):
        context = TickContext(tick, ns)
        mission = replace(_mission(manager, context), mode=mode,
                          target_pose=Waypoint(2.0, 0.0) if mode is CommandMode.NAVIGATE else None)
        world = _world(context)
        if mode is CommandMode.FOLLOW_PERSON:
            world = replace(world, obstacle_tracks=(_person("person-1", 2.0, 0.0),))
        return navigator.evaluate(mission, _estimate(context, 0.0), world, completion)

    initial = evaluate(0, 1_000_000_000)
    request = navigator.pending_rollout_request
    seed = evaluate(1, 1_020_000_000, PlannerInput(TickContext(1, 1_020_000_000), request.context, computer.compute(request)))
    assert seed.status is NavigationStatus.ACTIVE
    pending = evaluate(2, 1_100_000_000)
    assert pending.trajectory_candidates == seed.trajectory_candidates
    request = navigator.pending_rollout_request
    held = evaluate(3, 1_360_000_000)
    assert held.reason == "PLANNER_STALE_HOLD"
    assert held.trajectory_candidates == ()
    assert held.mission_id == initial.mission_id
    resumed = evaluate(4, 1_380_000_000, PlannerInput(TickContext(4, 1_380_000_000), request.context, computer.compute(request)))
    assert resumed.status is NavigationStatus.ACTIVE
    assert resumed.trajectory_candidates
    assert resumed.mission_id == initial.mission_id


def test_stale_hold_resume_has_zero_motor_output_and_canonical_replay_match(tmp_path):
    config = control_config()
    backend = TimedBackend(config.navigation)
    motor = RecordingMotorSink()
    production = NativeControlComposition(motor, config, trajectory_rollout_backend=backend)
    sink = CaptureSink("async-continuity", configuration=configuration_documents())
    times = [1_000_000_000, 1_020_000_000, 1_040_000_000, 1_120_000_000,
             1_370_000_000, 1_380_000_000, 1_400_000_000, 1_420_000_000]
    values = _explore_values(len(times))
    results = []
    try:
        for index, (value, ns) in enumerate(zip(values, times)):
            if index == 2:
                backend.timing = CompletionTiming(times[1], times[1], times[1]+10_000_000, times[1]+11_000_000)
            elif index == 6:
                backend.timing = CompletionTiming(times[3], times[3], times[3]+278_000_000, times[3]+279_000_000)
            else:
                backend.timing = None
            closed = production.close_inputs(_retime(value, ns))
            result = production.run_tick(closed)
            assert result.trace.fault_layer is None
            sink.write(ExecutionRecord(closed, result))
            results.append(result)
            production.dispatch_pending_planner_request(ns)
        held = results[5].final_actuation
        assert held.left_output == held.right_output == 0
        resumed = results[6].final_actuation
        assert abs(resumed.left_output) + abs(resumed.right_output) > 0
        path = write_capture(sink.document("PASS"), tmp_path / "async-continuity.json")
        assert replay_capture(path, project_root=PROJECT_ROOT)["status"] == "MATCH"
    finally:
        production.close()
