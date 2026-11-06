"""Offline acceptance evidence for normal-motion execution geometry.

The fixture has a close side obstacle: its circumscribed free circle cannot
cover a normal forward arc, although the rectangular maneuver is clear.
"""
from dataclasses import replace
import math
import random

import pytest

from rig import healthy_localization, resolved_config
from test_v3_motion_feedback_quality import _warm_native_feedback
from v3.capture_encoding import encode_value
from v3.composition.full_fake import OfflineMotorSink
from v3.contracts import (
    ConstraintCode, LifecycleState, MotionValidity, NavigationPlan,
    NavigationStatus, ObstacleTrack, RobotEstimate, RollingLocalCostmap,
    TickContext, Waypoint, WorldSnapshot,
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


def normal_scene(*, v=.264697, omega=-.267079, obstacles=None):
    config = resolved_config().runtime.composition.live_control.control
    context = TickContext(30, 1_600_000_000)
    estimate = RobotEstimate(context, "odom", 0., 0., 0., v, omega,
        (0.,) * 25, localization_quality=healthy_localization())
    world = WorldSnapshot(context, "odom", 30,
        (ObstacleTrack("side", 0., .4, .01, 0., 0., 1.),)
        if obstacles is None else obstacles, 0,
        RollingLocalCostmap("odom", 30, .02, 2.5, (), 30, 0))
    navigator = TrajectoryNavigator(config.navigation,
        async_config=replace(config.async_l6, enabled=False, completion_inputs=False))
    return config, navigator, estimate, world


def normal_plan(config, navigator, estimate, world, *, v=.264697, omega=-.267079):
    goal = Waypoint(1., 0.)
    scene = replace(navigator._build_planning_scene(world),
        uncertainty_m=config.navigation.localization_inflation_sigma
                      * estimate.localization_quality.local_sigma_m)
    candidate = navigator._evaluate_trajectory(1, 1, v, omega, estimate,
        world, scene, goal, config.mission.default_constraints.max_v_mps,
        config.mission.default_constraints.max_omega_rad_s,
        config.navigation.clearance_score_cap_m, planning=False)
    validity = MotionValidity(estimate.context, estimate.context.monotonic_ns
        + config.navigation.motion_proof_horizon_ns, "odom", "EXPLORE:odom0",
        geometry_revision=world.local_costmap.revision,
        geometry_captured_ns=estimate.context.monotonic_ns)
    plan = NavigationPlan(estimate.context, "normal", (), None,
        config.mission.default_constraints, 0., .1, NavigationStatus.ACTIVE,
        local_goal=goal, trajectory_candidates=(candidate,), motion_validity=validity)
    return replace(plan, geometry_proof=navigator._motion_geometry_proof(plan, estimate, world))


def normal_realization(config, plan, estimate, world, previous):
    objective = MotionSelector(config.motion_selection).evaluate(plan)
    motion = MotionRealizer(config.motion_realization).evaluate(objective, estimate, world)
    limiter = OperationalConstraintLayer(config.operational_constraints)
    limiter.restore(OperationalConstraintsStateCheckpoint(
        TickContext(29, estimate.context.monotonic_ns - 20_000_000), *previous))
    if motion.stop_reason is None:
        motion = replace(motion, transition_allowed=True)
    return motion, limiter.evaluate(motion, estimate)


@pytest.mark.parametrize("v,omega", [(.264697, -.267079), (.3, 0.), (.264697, .267079)])
@pytest.mark.parametrize("previous", ["stationary", "same_motion"])
def test_normal_arc_proof_covers_tracking_and_actual_ramp(v, omega, previous):
    config, navigator, estimate, world = normal_scene(v=v, omega=omega)
    plan = normal_plan(config, navigator, estimate, world, v=v, omega=omega)
    assert not plan.trajectory_candidates[0].collision
    proof = plan.geometry_proof
    assert proof is not None and proof.covers(v, omega)
    previous_velocity = (0., 0.) if previous == "stationary" else (v, omega)
    motion, allowed = normal_realization(config, plan, estimate, world, previous_velocity)
    assert allowed.allowed_v_mps > 0.
    assert not allowed.motion_revoked
    assert ConstraintCode.LOCAL_CLEARANCE not in allowed.active_constraints
    assert proof.covers(motion.requested_v_mps, motion.requested_omega_rad_s)
    assert proof.covers(allowed.allowed_v_mps, allowed.allowed_omega_rad_s)
    assert proof.covers(*previous_velocity)
    assert not proof.covers(-v, omega)
    assert _decode_production_value(encode_value(plan), NavigationPlan, "plan") == plan
    # Physical output remains confined to the sole L12 writer in RAM.
    native, controller = _warm_native_feedback(resolved_config(), None)
    wheels = DifferentialDriveKinematics(config.chassis_control)(allowed)
    writer = OfflineMotorSink()
    final = FinalSafetyGate(writer).finalize(estimate.context,
        controller(wheels, native.frame(30, .25, .25)), (), LifecycleState.ACTIVE,
        None, wheel_setpoint=wheels)
    assert abs(final.left_output) + abs(final.right_output) > 0.


@pytest.mark.parametrize("case", ["stale", "frame", "translation_uncertainty",
    "heading_uncertainty", "overlap", "front", "moving_track"])
def test_invalid_normal_geometry_cannot_authorize_motion(case):
    config, navigator, estimate, world = normal_scene()
    if case == "stale":
        world = replace(world, local_costmap=replace(world.local_costmap,
            freshness_ns=config.navigation.max_costmap_freshness_ns + 1))
    elif case == "frame":
        world = replace(world, frame_id="other",
            local_costmap=replace(world.local_costmap, frame_id="other"))
    elif case == "translation_uncertainty":
        estimate = replace(estimate, localization_quality=healthy_localization(local_sigma_m=.3))
    elif case == "heading_uncertainty":
        estimate = replace(estimate, localization_quality=healthy_localization(yaw_sigma_rad=.6))
    elif case in ("overlap", "front"):
        world = replace(world, obstacle_tracks=(ObstacleTrack("blocked",
            .1 if case == "overlap" else .32, 0., .05, 0., 0., 1.),))
    else:
        world = replace(world, obstacle_tracks=(replace(world.obstacle_tracks[0], vy_mps=-1.),))
    plan = normal_plan(config, navigator, estimate, world)
    motion, allowed = normal_realization(config, plan, estimate, world, (0., 0.))
    assert allowed.allowed_v_mps == allowed.allowed_omega_rad_s == 0.
    assert motion.stop_reason is not None or allowed.motion_revoked
    native, controller = _warm_native_feedback(resolved_config(), None)
    wheels = DifferentialDriveKinematics(config.chassis_control)(allowed)
    final = FinalSafetyGate(OfflineMotorSink()).finalize(estimate.context,
        controller(wheels, native.frame(30, .25, .25)), (), LifecycleState.ACTIVE,
        None, wheel_setpoint=wheels)
    assert final.left_output == final.right_output == 0.


def test_prior_wrong_direction_cannot_inherit_forward_motion_proof():
    config, navigator, estimate, world = normal_scene()
    plan = normal_plan(config, navigator, estimate, world)
    assert plan.geometry_proof is not None and plan.geometry_proof.covers(.264697, -.267079)
    _, allowed = normal_realization(config, plan, estimate, world, (-.3, 0.))
    assert allowed.motion_revoked
    assert allowed.allowed_v_mps == allowed.allowed_omega_rad_s == 0.
    assert ConstraintCode.LOCAL_CLEARANCE in allowed.active_constraints


def test_failed_normal_certificate_cannot_use_unchanged_endpoint_exemption():
    config, navigator, estimate, world = normal_scene(v=.3, omega=0.)
    estimate = replace(estimate,
        localization_quality=healthy_localization(yaw_sigma_rad=.6))
    plan = normal_plan(config, navigator, estimate, world, v=.3, omega=0.)
    # The nominal centerline is clear, but uncertain headings can intersect
    # the side obstacle. An unchanged target still needs current geometry.
    assert not plan.trajectory_candidates[0].collision
    _, allowed = normal_realization(config, plan, estimate, world, (.3, 0.))
    assert allowed.allowed_v_mps == allowed.allowed_omega_rad_s == 0.
    assert allowed.motion_revoked


def test_expanded_normal_box_stays_inside_observed_local_scope():
    config, navigator, estimate, world = normal_scene()
    world = replace(world, local_costmap=replace(world.local_costmap, radius_m=.45))
    proof = normal_plan(config, navigator, estimate, world).geometry_proof
    assert proof is not None and proof.maneuver_bounds is not None
    uncertainty = config.navigation.localization_inflation_sigma * estimate.localization_quality.local_sigma_m
    radius = math.hypot(config.navigation.footprint_length_m / 2 + uncertainty,
        config.navigation.footprint_width_m / 2 + uncertainty)
    max_speed = max(abs(bound) for bound in proof.maneuver_bounds[:2])
    assert (radius + max_speed * proof.horizon_ns / 1e9
        + config.navigation.footprint_safety_margin_m <= world.local_costmap.radius_m)


def test_proved_normal_box_contains_safe_time_varying_paths():
    config, navigator, estimate, world = normal_scene(obstacles=(
        ObstacleTrack("side", 0., .4, .01, 0., -.05, 1.),))
    proof = normal_plan(config, navigator, estimate, world).geometry_proof
    assert proof is not None and proof.maneuver_bounds is not None
    low_v, high_v, low_omega, high_omega = proof.maneuver_bounds
    horizon_s = proof.horizon_ns / 1e9
    dt_s = horizon_s / 250
    inflation = config.navigation.localization_inflation_sigma
    half_length = config.navigation.footprint_length_m / 2 + inflation * estimate.localization_quality.local_sigma_m
    half_width = config.navigation.footprint_width_m / 2 + inflation * estimate.localization_quality.local_sigma_m
    generator = random.Random(2718)
    for trial in range(24):
        x = y = 0.
        yaw = (-1 if trial % 2 else 1) * inflation * estimate.localization_quality.yaw_sigma_rad
        for step in range(251):
            obstacle = world.obstacle_tracks[0]
            dx = obstacle.x_m + obstacle.vx_mps * step * dt_s - x
            dy = obstacle.y_m + obstacle.vy_mps * step * dt_s - y
            local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
            local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
            clearance = math.hypot(max(0., abs(local_x) - half_length),
                max(0., abs(local_y) - half_width)) - obstacle.radius_m
            assert clearance > config.navigation.footprint_safety_margin_m
            if step == 250:
                break
            # Exercise extrema, then arbitrary variations within the same box.
            v = high_v if trial < 8 else generator.uniform(low_v, high_v)
            omega = (low_omega if trial % 2 else high_omega) if trial < 8 else generator.uniform(low_omega, high_omega)
            x += v * dt_s * math.cos(yaw + omega * dt_s / 2)
            y += v * dt_s * math.sin(yaw + omega * dt_s / 2)
            yaw += omega * dt_s
