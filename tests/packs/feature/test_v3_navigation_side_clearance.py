"""A close parallel wall must not make a proved forward corridor unreachable."""
from dataclasses import replace
import time

import pytest

from rig import healthy_localization, resolved_config
from v3.contracts import (
    CommandMode, CommandRequest, LOCAL_FRAME_ID, NavigationStatus, ObstacleTrack,
    RobotEstimate, RollingLocalCostmap, TickContext, TrajectoryPose, Waypoint, WorldSnapshot,
)
from v3.contracts.planner import PlannerInput, TrajectoryRolloutRequest
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator, TrajectoryRolloutComputer
from v3.layers.l7_motion_selection import MotionSelector
from v3.layers.l8_motion_realization import MotionRealizer
from v3.layers.l9_operational_constraints import OperationalConstraintLayer


def _side_wall_scene(control, tick=0, *, clearance=None):
    nav = control.navigation
    if clearance is None:
        clearance = (nav.footprint_safety_margin_m + nav.local_escape_trigger_clearance_m) / 2
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
    quality = healthy_localization()
    estimate = RobotEstimate(context, LOCAL_FRAME_ID, 0., 0., 0., 0., 0.,
                             (0.,) * 25, localization_quality=quality)
    radius = .04
    wall_y = (nav.footprint_width_m / 2
              + nav.localization_inflation_sigma * quality.local_sigma_m
              + radius + clearance)
    # Overlapping discs form two parallel walls along the whole maneuver.
    # Translation keeps the footprint clearance constant; pivots cannot
    # manufacture progress by turning through either wall.
    tracks = tuple(ObstacleTrack(f"wall-{side}-{index}", index * .04, side * wall_y,
                                radius, 0., 0., 1.)
                   for side in (-1, 1) for index in range(-25, 51))
    world = WorldSnapshot(context, LOCAL_FRAME_ID, 1, tracks, 0,
        RollingLocalCostmap(LOCAL_FRAME_ID, 1, .1, 6., (), 1, 0))
    return estimate, world


def _request(control, estimate, world, *, explore=True):
    constraints = control.mission.default_constraints
    return TrajectoryRolloutRequest(estimate.context, estimate, world,
        Waypoint(control.navigation.explore_local_goal_max_distance_m, 0.),
        constraints.max_v_mps, constraints.max_omega_rad_s, (), explore)


def _straight_at_planning_floor(control, result):
    return next(candidate for candidate in result.trajectory_candidates
                if candidate.candidate_id.startswith("trajectory-")
                and candidate.omega_rad_s == 0.
                and candidate.v_mps == pytest.approx(control.navigation.minimum_planning_speed_mps))


def _front_approach_world(control, estimate, world):
    nav = control.navigation
    radius = .04
    travel = nav.minimum_planning_speed_mps * nav.rollout_horizon_ns / 1e9
    final_clearance = (nav.footprint_safety_margin_m + nav.local_escape_trigger_clearance_m) / 2
    front_x = (nav.footprint_length_m / 2
               + nav.localization_inflation_sigma * estimate.localization_quality.local_sigma_m
               + radius + travel + final_clearance)
    return replace(world, obstacle_tracks=(
        ObstacleTrack("front", front_x, 0., radius, 0., 0., 1.),))


@pytest.mark.parametrize("clearance", [None, .05071963784893588])
def test_parallel_walls_keep_safe_straight_travel_viable(clearance):
    control = resolved_config().runtime.composition.live_control.control
    estimate, world = _side_wall_scene(control, clearance=clearance)
    result = TrajectoryRolloutComputer(control.navigation).compute(_request(control, estimate, world))
    straight = _straight_at_planning_floor(control, result)
    assert result.goal_reachable
    assert (control.navigation.footprint_safety_margin_m < straight.min_clearance_m
            < control.navigation.local_escape_trigger_clearance_m)
    assert straight.progress_viable and not straight.collision
    assert all(sample.y_m == 0. for sample in straight.samples)


@pytest.mark.parametrize("closed", [False, True])
def test_side_corridor_reaches_forward_l7_selection_and_l9_ramp(closed):
    control = resolved_config().runtime.composition.live_control.control
    async_config = (control.async_l6 if closed else
                    replace(control.async_l6, enabled=False, completion_inputs=False))
    navigator = TrajectoryNavigator(control.navigation, async_config=async_config)
    manager = MissionManager(control.mission)
    selector = MotionSelector(control.motion_selection)
    realizer = MotionRealizer(control.motion_realization)
    limiter = OperationalConstraintLayer(control.operational_constraints)

    for tick in (0, 1):
        estimate, world = _side_wall_scene(control, tick)
        pending = navigator.pending_rollout_request
        completion = (PlannerInput(estimate.context, pending.context,
            TrajectoryRolloutComputer(control.navigation).compute(pending))
            if pending is not None else None)
        mission = manager.evaluate(CommandRequest(estimate.context, "side-corridor",
                                                  CommandMode.EXPLORE, (), tick))
        plan = navigator.evaluate(mission, estimate, world, completion)
        objective = selector.evaluate(plan)
        motion = realizer.evaluate(objective, estimate, world)
        allowed = limiter.evaluate(motion, estimate)

    assert plan.status is NavigationStatus.ACTIVE
    assert objective.trajectory is not None and objective.trajectory.v_mps > 0.
    assert objective.trajectory.omega_rad_s == 0.
    assert plan.geometry_proof is not None
    assert motion.stop_reason is None
    assert allowed.allowed_v_mps > 0. and not allowed.motion_revoked
    assert plan.geometry_proof.covers(allowed.allowed_v_mps, allowed.allowed_omega_rad_s)


def test_front_approach_preserves_the_stronger_clearance_gate():
    control = resolved_config().runtime.composition.live_control.control
    nav = control.navigation
    estimate, world = _side_wall_scene(control)
    # The planning-floor straight path ends above the hard footprint margin,
    # while approaching within the unchanged forward escape trigger.
    world = _front_approach_world(control, estimate, world)
    result = TrajectoryRolloutComputer(nav).compute(_request(control, estimate, world, explore=False))
    straight = _straight_at_planning_floor(control, result)
    assert nav.footprint_safety_margin_m < straight.min_clearance_m < nav.local_escape_trigger_clearance_m
    assert straight.collision


@pytest.mark.parametrize("approaching_front", [False, True])
def test_recovery_forward_probe_uses_the_same_approach_policy(approaching_front):
    import v3.layers.l6_navigation as navigation

    control = resolved_config().runtime.composition.live_control.control
    estimate, world = _side_wall_scene(control)
    if approaching_front:
        world = _front_approach_world(control, estimate, world)
    computer = TrajectoryRolloutComputer(control.navigation)
    scene = replace(computer._planning_scene(world),
        uncertainty_m=control.navigation.localization_inflation_sigma
                      * estimate.localization_quality.local_sigma_m)
    start_clearance = navigation._footprint_clearance(0., 0., 0., world, control.navigation, scene)
    potential = navigation._escape_unlock_potential(TrajectoryPose(0., 0., 0., 0),
        _request(control, estimate, world).goal, start_clearance,
        control.navigation.minimum_planning_speed_mps, world, control.navigation,
        scene, forward_only=True)
    assert (potential > 0.) is not approaching_front


def test_starting_inside_hard_margin_rejects_every_recovery_direction():
    control = resolved_config().runtime.composition.live_control.control
    estimate, world = _side_wall_scene(control,
        clearance=control.navigation.footprint_safety_margin_m - .001)
    result = TrajectoryRolloutComputer(control.navigation).compute(_request(control, estimate, world))
    assert all(candidate.collision for candidate in result.trajectory_candidates)
    assert not any(candidate.progress_viable and not candidate.collision
                   for candidate in result.trajectory_candidates)


def test_parallel_corridor_result_matches_the_spawned_planner_edge():
    from v3.adapters.l6_planner_process import ProcessTrajectoryRolloutBackend

    resolved = resolved_config()
    control = resolved.runtime.composition.live_control.control
    estimate, world = _side_wall_scene(control)
    request = _request(control, estimate, world)
    expected = TrajectoryRolloutComputer(control.navigation).compute(request)
    assert not _straight_at_planning_floor(control, expected).collision
    backend = ProcessTrajectoryRolloutBackend(control.navigation, ready_timeout_s=5.,
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
