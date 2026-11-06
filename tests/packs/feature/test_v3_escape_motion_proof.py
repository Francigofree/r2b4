"""Directional escape evidence across geometry, tracking, ramp and final output."""
from dataclasses import replace
import math

import pytest

from rig import healthy_localization, resolved_config
from test_v3_motion_feedback_quality import _warm_native_feedback
from v3.capture_encoding import encode_value
from v3.composition.full_fake import OfflineMotorSink
from v3.contracts import (
    ConstraintCode, LifecycleState, MotionValidity, NavigationPlan,
    NavigationStatus, ObstacleTrack, QualityState, RobotEstimate,
    RollingLocalCostmap, TickContext, Waypoint, WorldSnapshot,
)
from v3.layers.l6_navigation import TrajectoryNavigator
from v3.layers.l7_motion_selection import MotionSelector
from v3.layers.l8_motion_realization import MotionRealizer
from v3.layers.l9_operational_constraints import (
    OperationalConstraintLayer, OperationalConstraintsStateCheckpoint,
)
from v3.layers.l10_chassis_control import DifferentialDriveKinematics
from v3.layers.l12_safety_final import FinalSafetyGate
from v3.replay import _decode_production_value


def escape_scene(tick=30, *, v=0., omega=0., quality=None, obstacles=None):
    config = resolved_config().runtime.composition.live_control.control
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
    estimate = RobotEstimate(context, 'odom', 0., 0., 0., v, omega, (0.,)*25,
        localization_quality=quality or healthy_localization())
    obstacles = (ObstacleTrack('side', .34*math.cos(1.), .34*math.sin(1.),
                               .01, 0., 0., 1.),) if obstacles is None else obstacles
    world = WorldSnapshot(context, 'odom', tick, obstacles, 0,
        RollingLocalCostmap('odom', tick, .02, 2.5, (), tick, 0))
    navigator = TrajectoryNavigator(config.navigation,
        async_config=replace(config.async_l6, enabled=False, completion_inputs=False))
    return config, navigator, estimate, world


def escape_plan(config, navigator, estimate, world, *, reverse=False):
    v = -config.navigation.local_escape_reverse_max_v_mps if reverse else 0.
    omega = 0. if reverse else -config.navigation.wheel_limits.minimum_center_spin_rad_s
    goal = Waypoint(-1., -.5)
    scene = replace(navigator._build_planning_scene(world),
        uncertainty_m=config.navigation.localization_inflation_sigma*estimate.localization_quality.local_sigma_m)
    candidate = navigator._evaluate_trajectory(0, 0, v, omega, estimate, world, scene,
        goal, config.mission.default_constraints.max_v_mps,
        config.mission.default_constraints.max_omega_rad_s,
        config.navigation.clearance_score_cap_m, planning=False)
    candidate = replace(candidate, candidate_id='escape-00-00', progress_viable=True)
    validity = MotionValidity(estimate.context, estimate.context.monotonic_ns+250_000_000,
                              'odom', 'EXPLORE:odom0', geometry_revision=world.local_costmap.revision,
                              geometry_captured_ns=estimate.context.monotonic_ns)
    plan = NavigationPlan(estimate.context, 'escape', (), None, config.mission.default_constraints,
        0., .1, NavigationStatus.ACTIVE, local_goal=goal,
        trajectory_candidates=(candidate,), motion_validity=validity)
    return replace(plan, geometry_proof=navigator._motion_geometry_proof(plan, estimate, world))


def realize(config, plan, estimate, world):
    objective = MotionSelector(config.motion_selection).evaluate(plan)
    motion = MotionRealizer(config.motion_realization).evaluate(objective, estimate, world)
    limiter = OperationalConstraintLayer(config.operational_constraints)
    limiter.restore(OperationalConstraintsStateCheckpoint(
        TickContext(estimate.context.tick_id-1, estimate.context.monotonic_ns-20_000_000), 0., 0.))
    return motion, limiter, limiter.evaluate(motion, estimate)


@pytest.mark.parametrize('reverse', [False, True])
def test_close_escape_has_directional_proof_and_reaches_final_output(reverse):
    config, navigator, estimate, world = escape_scene()
    plan = escape_plan(config, navigator, estimate, world, reverse=reverse)
    assert not plan.trajectory_candidates[0].collision
    proof = plan.geometry_proof
    assert proof is not None and proof.maneuver_bounds is not None
    motion, limiter, allowed = realize(config, plan, estimate, world)
    assert not allowed.motion_revoked
    assert ConstraintCode.LOCAL_CLEARANCE not in allowed.active_constraints
    assert proof.covers(allowed.allowed_v_mps, allowed.allowed_omega_rad_s)
    assert (allowed.allowed_v_mps < 0.) if reverse else (allowed.allowed_omega_rad_s < 0.)
    assert allowed.velocity_transition_until_ns is not None
    # Existing native feedback source and L11, with the sole L12 writer in RAM.
    native, controller = _warm_native_feedback(resolved_config(), None)
    frame = native.frame(30, .25, .25)
    wheels = DifferentialDriveKinematics(config.chassis_control)(allowed)
    request = controller(wheels, frame)
    writer = OfflineMotorSink()
    final = FinalSafetyGate(writer).finalize(estimate.context, request, (), LifecycleState.ACTIVE,
                                            None, wheel_setpoint=wheels)
    assert abs(final.left_output) + abs(final.right_output) > 0.
    assert _decode_production_value(encode_value(plan), NavigationPlan, 'plan') == plan
    # Wrong-way or translating pivot cannot inherit this directional authority.
    if not reverse:
        assert not proof.covers(0., -motion.requested_omega_rad_s)
        assert not proof.covers(.01, motion.requested_omega_rad_s)


@pytest.mark.parametrize('case', ['stale', 'frame', 'uncertainty', 'heading_uncertainty', 'overlap', 'rear', 'moving_track'])
def test_invalid_escape_geometry_has_no_positive_output(case):
    config, navigator, estimate, world = escape_scene()
    reverse = case == 'rear'
    if case == 'stale':
        world = replace(world, local_costmap=replace(world.local_costmap,
            freshness_ns=config.navigation.max_costmap_freshness_ns+1))
    elif case == 'frame':
        world = replace(world, frame_id='other', local_costmap=replace(world.local_costmap, frame_id='other'))
    elif case == 'uncertainty':
        estimate = replace(estimate, localization_quality=healthy_localization(local_sigma_m=.06))
    elif case == 'heading_uncertainty':
        estimate = replace(estimate, localization_quality=healthy_localization(yaw_sigma_rad=.4))
    elif case in ('overlap', 'rear'):
        world = replace(world, obstacle_tracks=(ObstacleTrack('blocked', -.2 if reverse else .1,
                                                             0., .05, 0., 0., 1.),))
    else:
        world = replace(world, obstacle_tracks=(replace(world.obstacle_tracks[0], vy_mps=-1.),))
    plan = escape_plan(config, navigator, estimate, world, reverse=reverse)
    assert plan.geometry_proof is None
    motion, _, allowed = realize(config, plan, estimate, world)
    assert allowed.allowed_v_mps == allowed.allowed_omega_rad_s == 0.
    writer = OfflineMotorSink()
    # Revoked L9 motion goes through the ordinary L10/L11 zero path.
    native, controller = _warm_native_feedback(resolved_config(), None)
    wheels = DifferentialDriveKinematics(config.chassis_control)(allowed)
    final = FinalSafetyGate(writer).finalize(estimate.context,
        controller(wheels, native.frame(30, .25, .25)), (), LifecycleState.ACTIVE, None,
        wheel_setpoint=wheels)
    assert final.left_output == final.right_output == 0.


def test_directional_proof_checks_previous_motion_even_at_approved_endpoint():
    config, navigator, estimate, world = escape_scene()
    plan = escape_plan(config, navigator, estimate, world)
    motion, limiter, _ = realize(config, plan, estimate, world)
    # The new pivot endpoint can be reached in one tick, but the retained
    # prior translating twist lies outside its whole-gap proof.
    approved = motion.approved_velocity
    motion = replace(motion, requested_omega_rad_s=approved.omega_rad_s, transition_allowed=True)
    limiter.restore(OperationalConstraintsStateCheckpoint(
        TickContext(29, estimate.context.monotonic_ns-20_000_000), .0001, approved.omega_rad_s))
    allowed = limiter.evaluate(motion, estimate)
    assert allowed.motion_revoked
    assert allowed.allowed_v_mps == allowed.allowed_omega_rad_s == 0.
    assert ConstraintCode.LOCAL_CLEARANCE in allowed.active_constraints


def test_fresh_phase_proof_covers_the_previous_speed_during_soft_quality_braking():
    config, navigator, estimate, world = escape_scene()
    initial = escape_plan(config, navigator, estimate, world)
    # The owned phase began at a higher speed while quality was GOOD.
    phase = replace(initial.trajectory_candidates[0], omega_rad_s=-1.4)
    navigator._escape_candidate = phase
    estimate = replace(estimate, localization_quality=healthy_localization(
        local_translation=QualityState.DEGRADED))
    plan = replace(initial, geometry_proof=navigator._motion_geometry_proof(initial, estimate, world))
    proof = plan.geometry_proof
    assert proof is not None and proof.maneuver_bounds is not None
    assert proof.covers(0., -1.2)
    motion, limiter, _ = realize(config, plan, estimate, world)
    motion = replace(motion, transition_allowed=True)
    limiter.restore(OperationalConstraintsStateCheckpoint(
        TickContext(29, estimate.context.monotonic_ns-20_000_000), 0., -1.2))
    allowed = limiter.evaluate(motion, estimate)
    assert not allowed.motion_revoked
    assert -1.2 < allowed.allowed_omega_rad_s < motion.approved_velocity.omega_rad_s
    assert proof.covers(allowed.allowed_v_mps, allowed.allowed_omega_rad_s)
    assert allowed.previous_velocity.omega_rad_s == -1.2


def test_stale_or_wrong_scope_proof_cannot_authorize_the_ramp():
    config, navigator, estimate, world = escape_scene()
    plan = escape_plan(config, navigator, estimate, world)
    for proof in (replace(plan.geometry_proof, scope='OTHER'),
                  replace(plan.geometry_proof, context=TickContext(29, estimate.context.monotonic_ns-20_000_000),
                          geometry_captured_ns=estimate.context.monotonic_ns-20_000_000)):
        motion, _, allowed = realize(config, replace(plan, geometry_proof=proof), estimate, world)
        assert allowed.motion_revoked
        assert allowed.allowed_v_mps == allowed.allowed_omega_rad_s == 0.
