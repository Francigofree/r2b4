"""Room Cruise can back out before a pivot/straight exit becomes reachable."""
from dataclasses import replace
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
from v3.layers.l8_motion_realization import MotionRealizer
from v3.layers.l9_operational_constraints import OperationalConstraintLayer


def _scene(tick=0, *, x=0., rear_blocked=False, open_room=False):
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
    estimate = RobotEstimate(context, LOCAL_FRAME_ID, x, 0., 0., 0., 0., (0.,) * 25,
                             localization_quality=healthy_localization())
    tracks = tuple(ObstacleTrack(f"side-{side}-{index}", .1 + index * .04,
                                 side * .31, .04, 0., 0., 1.)
                   for side in (-1, 1) for index in range(15))
    tracks += (ObstacleTrack("front", .50, 0., .12, 0., 0., 1.),)
    if rear_blocked:
        tracks += (ObstacleTrack("rear", -.30, 0., .08, 0., 0., 1.),)
    if open_room:
        tracks = ()
    world = WorldSnapshot(context, LOCAL_FRAME_ID, 1, tracks, 0,
        RollingLocalCostmap(LOCAL_FRAME_ID, 1, .1, 2.5, (), 1, 0))
    return estimate, world


def _request(estimate, world):
    return TrajectoryRolloutRequest(estimate.context, estimate, world, Waypoint(1., 0.),
                                    .45, 1.4, (), True, select_local_goal=True)


def _viable(result):
    return tuple(c for c in result.trajectory_candidates if c.progress_viable and not c.collision)


def test_no_straight_exit_keeps_a_proved_reverse_without_promoting_the_seed():
    control = resolved_config().runtime.composition.live_control.control
    estimate, world = _scene()
    request = _request(estimate, world)
    result = TrajectoryRolloutComputer(control.navigation).compute(request)

    assert not result.goal_reachable and not result.goal_options and result.reachable_exit_mask == 0
    assert result.selected_goal == request.goal
    viable = _viable(result)
    assert viable and all(c.candidate_id.startswith("escape-") for c in viable)
    assert any(c.v_mps < 0. and c.omega_rad_s == 0. for c in viable)
    assert all(c.min_clearance_m > control.navigation.footprint_safety_margin_m for c in viable)
    # Rejected forward identities still revoke any prior L7 trajectory.
    assert any(c.candidate_id.startswith("trajectory-") and c.v_mps > 0. and c.collision
               for c in result.trajectory_candidates)


def test_front_obstacle_revokes_l7s_previous_forward_path_before_reversing():
    control = resolved_config().runtime.composition.live_control.control
    navigator = TrajectoryNavigator(control.navigation,
        async_config=replace(control.async_l6, enabled=False, completion_inputs=False))
    manager = MissionManager(control.mission)
    selector = MotionSelector(control.motion_selection)
    realizer = MotionRealizer(control.motion_realization)
    limiter = OperationalConstraintLayer(control.operational_constraints)
    forward_id = None
    reverse_seen = False
    for tick in range(16):
        estimate, world = _scene(tick, open_room=tick < 2)
        world = replace(world, map_revision=1 if tick < 2 else 2,
            local_costmap=replace(world.local_costmap, revision=1 if tick < 2 else 2))
        mission = manager.evaluate(CommandRequest(estimate.context, "changing-dead-end",
                                                  CommandMode.EXPLORE, (), tick))
        plan = navigator.evaluate(mission, estimate, world)
        objective = selector.evaluate(plan)
        allowed = limiter.evaluate(realizer.evaluate(objective, estimate, world), estimate)
        if tick == 1:
            assert objective.trajectory is not None and allowed.allowed_v_mps > 0.
            forward_id = objective.trajectory.candidate_id
        if tick >= 2:
            assert allowed.allowed_v_mps <= 0.
        if objective.trajectory is not None and objective.trajectory.v_mps < 0.:
            if not reverse_seen:
                assert any(c.candidate_id == forward_id and c.collision for c in plan.trajectory_candidates)
            reverse_seen |= allowed.allowed_v_mps < 0.
    assert reverse_seen


@pytest.mark.parametrize("closed", [False, True])
def test_reverse_reaches_l9_with_fresh_proof_then_replans_after_backing_out(closed):
    control = resolved_config().runtime.composition.live_control.control
    async_config = control.async_l6 if closed else replace(control.async_l6, enabled=False, completion_inputs=False)
    navigator = TrajectoryNavigator(control.navigation, async_config=async_config)
    computer = TrajectoryRolloutComputer(control.navigation)
    manager = MissionManager(control.mission)
    selector = MotionSelector(control.motion_selection)
    realizer = MotionRealizer(control.motion_realization)
    limiter = OperationalConstraintLayer(control.operational_constraints)
    reverse_seen = normal_seen = False

    for tick in range(126):
        # Two finite reverse horizons clear the side walls. Keep the geometry
        # fixed; only the independently closed local pose advances.
        estimate, world = _scene(tick, x=-min(.52, max(0, tick - 1) * .005))
        pending = navigator.pending_rollout_request
        completion = (PlannerInput(estimate.context, pending.context, computer.compute(pending))
                      if pending is not None else None)
        mission = manager.evaluate(CommandRequest(estimate.context, "dead-end",
                                                  CommandMode.EXPLORE, (), tick))
        plan = navigator.evaluate(mission, estimate, world, completion)
        objective = selector.evaluate(plan)
        motion = realizer.evaluate(objective, estimate, world)
        allowed = limiter.evaluate(motion, estimate)
        candidate = objective.trajectory
        if candidate is not None and candidate.candidate_id.startswith("escape-"):
            assert candidate.v_mps < 0. and candidate.omega_rad_s == 0.
            assert plan.geometry_proof is not None
            assert plan.geometry_proof.geometry_captured_ns == estimate.context.monotonic_ns
            assert plan.geometry_proof.covers(allowed.allowed_v_mps, allowed.allowed_omega_rad_s)
            assert allowed.allowed_v_mps <= 0.
            reverse_seen |= allowed.allowed_v_mps < 0. and not allowed.motion_revoked
        elif candidate is not None and reverse_seen:
            assert navigator.checkpoint().escape_candidate is None
            assert navigator.checkpoint().goal_reachable
            assert plan.status is NavigationStatus.ACTIVE
            assert plan.geometry_proof is not None
            normal_seen |= not allowed.motion_revoked and (
                allowed.allowed_v_mps > 0. or abs(allowed.allowed_omega_rad_s) > 0.)
    assert reverse_seen and normal_seen


@pytest.mark.parametrize("closed", [False, True])
def test_blocked_rear_stays_stopped(closed):
    control = resolved_config().runtime.composition.live_control.control
    async_config = control.async_l6 if closed else replace(control.async_l6, enabled=False, completion_inputs=False)
    navigator = TrajectoryNavigator(control.navigation, async_config=async_config)
    computer = TrajectoryRolloutComputer(control.navigation)
    manager = MissionManager(control.mission)
    selector = MotionSelector(control.motion_selection)
    realizer = MotionRealizer(control.motion_realization)
    limiter = OperationalConstraintLayer(control.operational_constraints)
    for tick in (0, 1):
        estimate, world = _scene(tick, rear_blocked=True)
        pending = navigator.pending_rollout_request
        completion = (PlannerInput(estimate.context, pending.context, computer.compute(pending))
                      if pending is not None else None)
        mission = manager.evaluate(CommandRequest(estimate.context, "blocked-dead-end",
                                                  CommandMode.EXPLORE, (), tick))
        plan = navigator.evaluate(mission, estimate, world, completion)
        allowed = limiter.evaluate(realizer.evaluate(selector.evaluate(plan), estimate, world), estimate)
        assert allowed.allowed_v_mps == allowed.allowed_omega_rad_s == 0.
        assert plan.status is not NavigationStatus.ACTIVE
    result = computer.compute(_request(estimate, world))
    assert not _viable(result) and result.selected_goal is None and not result.goal_reachable


def test_dead_end_recovery_matches_the_spawned_compute_edge():
    from v3.adapters.l6_planner_process import ProcessTrajectoryRolloutBackend

    resolved = resolved_config()
    config = resolved.runtime.composition.live_control.control.navigation
    estimate, world = _scene()
    request = _request(estimate, world)
    expected = TrajectoryRolloutComputer(config).compute(request)
    assert any(c.v_mps < 0. for c in _viable(expected))
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
