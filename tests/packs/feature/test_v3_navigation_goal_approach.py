"""A finite goal inside the safety rollout remains reachable through L5-L9."""
from dataclasses import replace

import pytest

from rig import healthy_localization, resolved_config
from v3.contracts import (
    CommandMode, CommandRequest, DataField, GLOBAL_FRAME_ID, LOCAL_FRAME_ID,
    NavigationStatus, ObstacleTrack, Pose2D, QualityState, RobotEstimate,
    RollingLocalCostmap, TickContext, WorldSnapshot,
)
from v3.contracts.planner import PlannerInput
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator, TrajectoryRolloutComputer
from v3.layers.l7_motion_selection import MotionSelector
from v3.layers.l8_motion_realization import MotionRealizer
from v3.layers.l9_operational_constraints import (
    OperationalConstraintLayer, OperationalConstraintsStateCheckpoint,
)


def _scene(tick, x, *, global_quality=QualityState.GOOD, bad=None):
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
    quality = healthy_localization(global_position=global_quality, global_fix_status="ACCEPTED")
    if bad == "local_lost":
        quality = replace(quality, local_translation=QualityState.LOST)
    estimate = RobotEstimate(context, GLOBAL_FRAME_ID, x, 0., 0., .2, 0.,
        (0.,) * 25, local_pose=Pose2D(LOCAL_FRAME_ID, x, 0., 0.),
        global_pose=Pose2D(GLOBAL_FRAME_ID, x, 0., 0.),
        map_to_odom=Pose2D(GLOBAL_FRAME_ID, 0., 0., 0.),
        localization_quality=quality, local_v_mps=.2, local_omega_rad_s=0.)
    obstacles = ((ObstacleTrack("blocked", x + .1, 0., .1, 0., 0., 1.),)
                 if bad == "obstacle" else ())
    world = WorldSnapshot(context, LOCAL_FRAME_ID, tick, obstacles, 0,
        RollingLocalCostmap(LOCAL_FRAME_ID, tick, .1, 2.5, (), tick,
                            300_000_000 if bad == "stale" else 0))
    return estimate, world


def _approach(distance, frame_id=GLOBAL_FRAME_ID, global_quality=QualityState.GOOD):
    config = resolved_config().runtime.composition.live_control.control
    manager = MissionManager(config.mission)
    navigator = TrajectoryNavigator(config.navigation, async_config=config.async_l6)

    def evaluate(tick, *, completion=None, bad=None, x=None, quality=global_quality,
                 accepted_fix=True):
        estimate, world = _scene(tick, 1. - distance if x is None else x,
                                 global_quality=quality, bad=bad)
        if not accepted_fix:
            estimate = replace(estimate, localization_quality=replace(
                estimate.localization_quality, global_fix_status="NO_ADMITTED_FIX"))
        mission = manager.evaluate(CommandRequest(estimate.context, "approach", CommandMode.NAVIGATE,
            (DataField("x_m", 1.), DataField("y_m", 0.), DataField("yaw_rad", 0.),
             DataField("frame_id", frame_id), DataField("goal_tolerance_m", .02),
             DataField("max_v_mps", .2), DataField("max_omega_rad_s", .6)), tick))
        plan = navigator.evaluate(mission, estimate, world, completion)
        return estimate, world, plan

    evaluate(0)
    request = navigator.pending_rollout_request
    result = TrajectoryRolloutComputer(config.navigation).compute(request)
    completion = PlannerInput(TickContext(1, 1_020_000_000), request.context, result)
    return config, evaluate, completion


@pytest.mark.parametrize("distance", [.195, .10, .025])
@pytest.mark.parametrize("frame_id,global_quality", [
    (LOCAL_FRAME_ID, QualityState.LOST), (GLOBAL_FRAME_ID, QualityState.GOOD),
])
def test_fresh_terminal_approach_reaches_inside_rollout_and_waypoint_tolerance(
    distance, frame_id, global_quality,
):
    config, evaluate, completion = _approach(distance, frame_id, global_quality)
    estimate, world, plan = evaluate(1, completion=completion)
    assert plan.status is NavigationStatus.ACTIVE
    assert plan.reason == "LOCAL_GUIDANCE_REVALIDATED"
    assert plan.geometry_proof is not None
    assert all(not c.collision and c.progress_viable for c in plan.trajectory_candidates)
    objective = MotionSelector(config.motion_selection).evaluate(plan)
    motion = MotionRealizer(config.motion_realization).evaluate(objective, estimate, world)
    limiter = OperationalConstraintLayer(config.operational_constraints)
    limiter.restore(OperationalConstraintsStateCheckpoint(TickContext(0, 1_000_000_000), .2, 0.))
    allowed = limiter.evaluate(motion, estimate)
    assert 0. < allowed.allowed_v_mps <= .2
    assert abs(allowed.allowed_omega_rad_s) <= .6
    _, _, arrived = evaluate(2, x=.99)
    assert arrived.status is NavigationStatus.COMPLETE


@pytest.mark.parametrize("bad", ["stale", "obstacle", "local_lost"])
def test_near_goal_never_bypasses_current_geometry_or_localization(bad):
    config, evaluate, completion = _approach(.10)
    estimate, world, plan = evaluate(1, completion=completion, bad=bad)
    objective = MotionSelector(config.motion_selection).evaluate(plan)
    motion = MotionRealizer(config.motion_realization).evaluate(objective, estimate, world)
    allowed = OperationalConstraintLayer(config.operational_constraints).evaluate(motion, estimate)
    assert allowed.allowed_v_mps == allowed.allowed_omega_rad_s == 0.


def test_global_loss_cannot_cross_or_complete_committed_waypoint():
    _, evaluate, completion = _approach(.195)
    _, _, active = evaluate(1, completion=completion)
    assert active.status is NavigationStatus.ACTIVE
    _, _, held = evaluate(2, x=.90, quality=QualityState.LOST)
    assert held.status is NavigationStatus.IDLE
    assert held.reason == "LOCAL_GUIDANCE_UNAVAILABLE"
    assert held.mission_id == active.mission_id
    _, _, unresolved = evaluate(3, x=1., quality=QualityState.LOST)
    assert unresolved.status is not NavigationStatus.COMPLETE


def test_global_arrival_waits_for_independent_fix_and_can_resume_if_prediction_was_wrong():
    _, evaluate, completion = _approach(.195)
    _, _, active = evaluate(1, completion=completion)
    _, _, predicted = evaluate(2, x=.99, accepted_fix=False)
    assert predicted.status is NavigationStatus.IDLE
    assert predicted.reason == "GLOBAL_ARRIVAL_PENDING"
    assert predicted.mission_id == active.mission_id
    # The next independent fix disproves arrival. The mission is still active
    # and can continue toward the same target rather than latching completion.
    _, _, corrected = evaluate(3, x=.96)
    assert corrected.status is NavigationStatus.ACTIVE
    _, _, arrived = evaluate(4, x=.99)
    assert arrived.status is NavigationStatus.COMPLETE
