"""Offline evidence for planner reachability and L6-owned bounded recovery."""
from dataclasses import replace
import math
import pickle
import time

import pytest

from rig import healthy_localization, resolved_config
from v3.contracts import (
    CommandMode, CommandRequest, LOCAL_FRAME_ID, NavigationStatus, ObstacleTrack,
    RobotEstimate, RollingLocalCostmap, TickContext, Waypoint, WorldSnapshot,
)
from v3.contracts.planner import PlannerInput, TrajectoryRolloutRequest
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator, TrajectoryRolloutComputer
from v3.layers.l7_motion_selection import MotionSelector


def _scene(tick=0, *, tracks=(), x=0., revision=1):
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
    estimate = RobotEstimate(context, LOCAL_FRAME_ID, x, 0., 0., 0., 0., (0.,) * 25,
                             localization_quality=healthy_localization())
    world = WorldSnapshot(context, LOCAL_FRAME_ID, revision, tracks, 0,
        RollingLocalCostmap(LOCAL_FRAME_ID, revision, .1, 2.5, (), revision, 0))
    return estimate, world


def _request(*, tracks=(), goals=(Waypoint(.9, 0.),)):
    estimate, world = _scene(tracks=tracks)
    return TrajectoryRolloutRequest(estimate.context, estimate, world, goals[0],
                                    .45, 1.4, (), True, goals[1:])


def test_endpoint_clearance_does_not_admit_blocked_corridor_and_worker_selects_alternate():
    config = resolved_config().runtime.composition.live_control.control.navigation
    goals = (Waypoint(.9, 0.), Waypoint(0., .9), Waypoint(-.9, 0.), Waypoint(0., -.9))
    request = _request(tracks=(ObstacleTrack('wall', .5, 0., .05, 0., 0., 1.),), goals=goals)
    # Both endpoints have ample space, but the first goal's full corridor crosses
    # the wall. The pure worker retains the next footprint-proven goal.
    result = TrajectoryRolloutComputer(config).compute(request)
    assert result.source_context == request.context
    assert result.goal_reachable and result.selected_goal == goals[1]
    assert any(not c.collision and c.progress_viable for c in result.trajectory_candidates)


def test_intermediate_pivot_footprint_can_reject_clear_start_and_endpoint():
    config = resolved_config().runtime.composition.live_control.control.navigation
    bearing = math.radians(70)
    obstacle = ObstacleTrack('corner', .37 * math.cos(bearing), .37 * math.sin(bearing),
                              .01, 0., 0., 1.)
    request = _request(tracks=(obstacle,), goals=(Waypoint(-.9, 0.),))
    result = TrajectoryRolloutComputer(config).compute(request)
    assert not result.goal_reachable
    assert not any(c.progress_viable and not c.collision
                   for c in result.trajectory_candidates if c.candidate_id.startswith('trajectory-'))
    assert all(not c.collision for c in result.trajectory_candidates if c.progress_viable)


def test_selected_alternate_survives_real_spawned_process():
    from v3.adapters.l6_planner_process import ProcessTrajectoryRolloutBackend
    resolved = resolved_config()
    config = resolved.runtime.composition.live_control.control.navigation
    request = _request(tracks=(ObstacleTrack('wall', .5, 0., .05, 0., 0., 1.),),
        goals=(Waypoint(.9, 0.), Waypoint(0., .9), Waypoint(-.9, 0.), Waypoint(0., -.9)))
    expected = TrajectoryRolloutComputer(config).compute(request)
    backend = ProcessTrajectoryRolloutBackend(config, ready_timeout_s=5.,
                                             process_config=resolved.edges.planner_process)
    try:
        request_id = backend.submit(request)
        completion = None
        deadline = time.monotonic() + 5.
        while completion is None and time.monotonic() < deadline:
            completion = backend.take_completion(request_id)
            if completion is None:
                time.sleep(.002)
        assert completion is not None and completion.error is None
        assert completion.identity.source_context == request.context
        assert completion.result == expected
    finally:
        backend.close()


def test_closed_alternate_acceptance_keeps_goal_source_time_and_failed_memory(monkeypatch):
    import v3.layers.l6_navigation as navigation
    config = resolved_config().runtime.composition.live_control.control
    goals = (Waypoint(.9, 0.), Waypoint(0., .9), Waypoint(-.9, 0.), Waypoint(0., -.9))
    monkeypatch.setattr(navigation, '_rank_local_goal_options', lambda *args: goals)
    navigator = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    direct = TrajectoryNavigator(config.navigation,
        async_config=replace(config.async_l6, enabled=False, completion_inputs=False))
    manager = MissionManager(config.mission)
    tracks = (ObstacleTrack('wall', .5, 0., .05, 0., 0., 1.),)
    estimate, world = _scene(tracks=tracks)
    mission = manager.evaluate(CommandRequest(estimate.context, 'alternate', CommandMode.EXPLORE, (), 0))
    direct.evaluate(mission, estimate, world)
    navigator.evaluate(mission, estimate, world)
    request = navigator.pending_rollout_request
    assert request.select_local_goal and request.alternate_goals == ()
    result = TrajectoryRolloutComputer(config.navigation).compute(request)
    assert result.goal_options == goals
    assert direct.checkpoint().trajectory_candidates == result.trajectory_candidates
    estimate, world = _scene(1, tracks=tracks)
    mission = manager.evaluate(CommandRequest(estimate.context, 'alternate', CommandMode.EXPLORE, (), 1))
    plan = navigator.evaluate(mission, estimate, world,
                              PlannerInput(estimate.context, request.context, result))
    assert plan.local_goal == goals[1] == direct.checkpoint().local_goal
    assert plan.goal_selected_ns == request.context.monotonic_ns
    assert plan.motion_validity.source_context in (request.context, estimate.context)
    checkpoint = pickle.loads(pickle.dumps(navigator.checkpoint()))
    assert checkpoint.goal_reachable
    failed_cell = navigator._coverage_key(goals[0].x_m, goals[0].y_m)
    assert any((x, y) == failed_cell for x, y, _until in checkpoint.failed_goals)
    restored = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    restored.restore(checkpoint)
    estimate, world = _scene(2, tracks=tracks)
    mission = manager.evaluate(CommandRequest(estimate.context, 'alternate', CommandMode.EXPLORE, (), 2))
    assert restored.evaluate(mission, estimate, world) == navigator.evaluate(mission, estimate, world)


def test_unreachable_completion_cannot_resume_synthetic_continuation():
    config = resolved_config().runtime.composition.live_control.control
    navigator = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    manager = MissionManager(config.mission)
    estimate, world = _scene()
    mission = manager.evaluate(CommandRequest(estimate.context, 'no-path', CommandMode.EXPLORE, (), 0))
    navigator.evaluate(mission, estimate, world)
    request = navigator.pending_rollout_request
    result = TrajectoryRolloutComputer(config.navigation).compute(request)
    # A negative pure result cannot be bypassed by synthesizing a new motion
    # toward the same endpoint while its replacement is pending.
    result = replace(result, goal_reachable=False,
                     trajectory_candidates=tuple(replace(c, progress_viable=False)
                                                 for c in result.trajectory_candidates))
    tick = math.ceil(navigator._max_plan_age_ns / 2 / 20_000_000)
    estimate, world = _scene(1)
    mission = manager.evaluate(CommandRequest(estimate.context, 'no-path', CommandMode.EXPLORE, (), 1))
    first = navigator.evaluate(mission, estimate, world,
                              PlannerInput(estimate.context, request.context, result))
    estimate, world = _scene(tick)
    mission = manager.evaluate(CommandRequest(estimate.context, 'no-path', CommandMode.EXPLORE, (), tick))
    pending = navigator.evaluate(mission, estimate, world)
    assert first.status is NavigationStatus.IDLE
    assert pending.status is NavigationStatus.IDLE
    assert not pending.trajectory_candidates
    assert MotionSelector(config.motion_selection).evaluate(pending).trajectory is None
    # The no-path budget ends in an explicit HOLD and bounded cooldown.
    tick = math.ceil(config.navigation.local_goal_max_age_ns / 20_000_000) + 1
    estimate, world = _scene(tick)
    mission = manager.evaluate(CommandRequest(estimate.context, 'no-path', CommandMode.EXPLORE, (), tick))
    held = navigator.evaluate(mission, estimate, world)
    assert held.status is NavigationStatus.IDLE
    assert held.reason == 'NO_PATH_HOLD'
    assert navigator.pending_rollout_request is None


def test_recovery_phase_uses_one_fresh_maneuver_and_expires_without_movement(monkeypatch):
    from test_v3_cached_geometry import _escape_scene, _seed_escape
    import v3.layers.l6_navigation as navigation
    config, navigator, intent, _request, _result, first = _seed_escape('pivot')
    phase = navigator.checkpoint()
    candidate = phase.escape_candidate
    assert candidate is not None
    assert len([c for c in first.trajectory_candidates if c.progress_viable and not c.collision]) == 1

    def unexpected(*args, **kwargs):
        pytest.fail('accepted recovery repeated its unlock search')

    monkeypatch.setattr(navigation, '_escape_unlock_potential', unexpected)
    for tick in (2, 5, 10):
        estimate, world = _escape_scene(config.navigation, tick, kind='pivot', revision=tick)
        plan = navigator.evaluate(intent(estimate.context), estimate, world)
        assert plan.status is NavigationStatus.ACTIVE
        viable = [c for c in plan.trajectory_candidates if c.progress_viable and not c.collision]
        assert len(viable) == 1
        assert (viable[0].v_mps, viable[0].omega_rad_s) == (candidate.v_mps, candidate.omega_rad_s)
        assert navigator.checkpoint().escape_started_ns == phase.escape_started_ns
        assert plan.motion_validity.source_context == estimate.context
        assert navigator.pending_rollout_request is None
    expiry_tick = 1 + math.ceil(2 * candidate.horizon_ns / 20_000_000)
    estimate, world = _escape_scene(config.navigation, expiry_tick, kind='pivot', revision=expiry_tick)
    exhausted = navigator.evaluate(intent(estimate.context), estimate, world)
    assert exhausted.status is NavigationStatus.IDLE
    assert exhausted.reason == 'RECOVERY_EXHAUSTED_HOLD'
    assert not exhausted.trajectory_candidates
    assert navigator.checkpoint().escape_candidate is None


def test_explore_progress_survives_continuation_hold_and_counts_cell_entries():
    config = resolved_config().runtime.composition.live_control.control
    navigator = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    manager = MissionManager(config.mission)

    def evaluate(tick, *, x=0., tracks=(), revision=1, completion=None):
        estimate, world = _scene(tick, x=x, tracks=tracks, revision=revision)
        mission = manager.evaluate(CommandRequest(estimate.context, 'coverage', CommandMode.EXPLORE, (), tick))
        return navigator.evaluate(mission, estimate, world, completion)

    evaluate(0)
    request = navigator.pending_rollout_request
    result = TrajectoryRolloutComputer(config.navigation).compute(request)
    normal = evaluate(1, completion=PlannerInput(TickContext(1, 1_020_000_000), request.context, result))
    continuation_tick = math.ceil(navigator._max_plan_age_ns / 2 / 20_000_000)
    continuation = evaluate(continuation_tick)
    hold = evaluate(continuation_tick + 1, tracks=(ObstacleTrack('overlap', 0., 0., .1, 0., 0., 1.),), revision=2)
    assert normal.progress > 0
    assert continuation.reason == 'LOCAL_GUIDANCE_REVALIDATED'
    assert continuation.progress == hold.progress == normal.progress
    assert hold.status is NavigationStatus.IDLE
    assert hold.goal_selection_reason is not None and hold.goal_selected_ns is not None
    assert navigator.checkpoint().coverage == ((0, 0, 1, hold.context.tick_id),)
    evaluate(continuation_tick + 2, x=config.navigation.coverage_cell_size_m * 1.2)
    evaluate(continuation_tick + 3)
    visits = {(x, y): count for x, y, count, _tick in navigator.checkpoint().coverage}
    assert visits == {(0, 0): 2, (1, 0): 1}
