"""Offline evidence for the capture-derived motion and goal-boundary repairs."""
from dataclasses import replace
import math

import pytest

from rig import healthy_localization, resolved_config
from v3.capture import encode_value
from v3.contracts import (
    CommandMode, CommandRequest, ConstraintCode, LocalMotionProof,
    LocalizationRequirement, MotionIntent, MotionValidity, NavigationPlan,
    NavigationStatus, ObstacleTrack, QualityState, RobotEstimate,
    RobotRelativeGeometry, RollingLocalCostmap, TickContext, TrajectoryEvaluation,
    TrajectoryPose, VelocityTarget, Waypoint, WorldSnapshot,
)
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator
from v3.layers.l7_motion_selection import MotionSelector, select_motion
from v3.layers.l8_motion_realization import MotionRealizer
from v3.layers.l9_operational_constraints import (
    OperationalConstraintLayer, OperationalConstraintsStateCheckpoint,
)
from v3.replay import _decode_production_value


def scene(tick, *, x=0., v=0., omega=0., quality=None):
    context = TickContext(tick, 1_000_000_000 + tick*20_000_000)
    estimate = RobotEstimate(context, 'odom', x, 0., 0., v, omega, (0.,)*25,
                             localization_quality=quality or healthy_localization())
    world = WorldSnapshot(context, 'odom', tick, (), 0,
        RollingLocalCostmap('odom', tick, .1, 2.5, (), tick, 0),
        robot_relative_geometry=RobotRelativeGeometry('ROBOT_BASE', tick,
                                                       context.monotonic_ns, 100, 0))
    return estimate, world


def trajectory(name, v, omega):
    return TrajectoryEvaluation(name, v, omega, 1_000_000_000,
        (TrajectoryPose(v, 0., omega, 1_000_000_000),), False, 1., .8, .8, .8, .8)


def plan(tick, candidates, config, *, displacement=.15):
    context = scene(tick)[0].context
    validity = MotionValidity(TickContext(0, 1_000_000_000), 1_500_000_000,
                              'odom', 'LOCAL_GEOMETRY', LocalizationRequirement(True, True, False))
    proof = LocalMotionProof(context, 'odom', validity.scope, tick,
        context.monotonic_ns, validity.valid_until_ns, 250_000_000, displacement)
    return NavigationPlan(context, 'repair', (), None, config.mission.default_constraints,
                          0., 0., NavigationStatus.ACTIVE, local_goal=Waypoint(1., 0.),
                          trajectory_candidates=tuple(candidates), motion_validity=validity,
                          geometry_proof=proof)


def test_joint_projection_preserves_pivot_and_counter_arc_with_degraded_caps():
    config = resolved_config().runtime.composition.live_control.control
    limits = config.operational_constraints.wheel_limits
    cap = limits.degraded_angular_cap(config.operational_constraints.max_omega_rad_s,
                                      config.operational_constraints.degraded_velocity_scale)
    turn = 1.4*limits.track_width_m/2
    for v, omega, signs in ((turn, 1.4, (0, 1)), (.05899, -1.4, (1, -1)),
                             (-turn, -1.4, (0, -1)), (-.05899, 1.4, (-1, 1))):
        projected = limits.constrain(v, omega, max_v_mps=.45, max_omega_rad_s=cap,
            max_curvature_rad_per_m=config.operational_constraints.max_curvature_rad_per_m)
        assert projected != (0., 0.)
        assert abs(projected[0]) <= abs(v)+1e-12
        assert abs(projected[1]) <= min(abs(omega), cap)+1e-12
        for speed, sign in zip(limits.wheels(*projected), signs):
            if sign == 0:
                assert speed == pytest.approx(0., abs=1e-12)
            else:
                assert limits.minimum_mps-1e-12 <= sign*speed <= limits.maximum_mps+1e-12
    # Physical feasibility is independent of the planner's operating reserve.
    assert limits.constrain(.17, 0.) == (.17, 0.)
    assert limits.constrain(.17, 0., planning=True) == (0., 0.)
    assert limits.constrain(.12, 0.) == (0., 0.)
    assert limits.constrain(0., 1.4, max_omega_rad_s=.2) == (0., 0.)


def test_progress_rejection_has_bounded_motion_but_collision_revokes_its_origin():
    config = resolved_config().runtime.composition.live_control.control
    old = trajectory('same-grid', .3, 0.)
    new = trajectory('new-grid', 0., 1.2)
    for case in ('progress', 'collision', 'reused_id', 'missing_proof', 'wrong_scope', 'stale_proof'):
        selector = MotionSelector(config.motion_selection)
        selector.evaluate(plan(0, (old,), config))
        rejected = replace(old, progress_viable=False, collision=case == 'collision')
        if case == 'reused_id':
            rejected = replace(rejected, v_mps=0., omega_rad_s=-1.2, collision=True)
        current = plan(1, (rejected, new), config)
        if case == 'missing_proof':
            current = replace(current, geometry_proof=None)
        elif case == 'wrong_scope':
            current = replace(current, geometry_proof=replace(current.geometry_proof, scope='OTHER'))
        elif case == 'stale_proof':
            current = replace(current, geometry_proof=plan(0, (old,), config).geometry_proof)
        objective = selector.evaluate(current)
        assert objective.trajectory == new
        assert objective.transition_allowed is (case in ('progress', 'reused_id'))
        # Stateless and stateful selectors carry the same immutable proof.
        assert select_motion(current).geometry_proof == objective.geometry_proof
        estimate, world = scene(1)
        limiter = OperationalConstraintLayer(config.operational_constraints)
        limiter.restore(OperationalConstraintsStateCheckpoint(scene(0)[0].context, .3, 0.))
        allowed = limiter.evaluate(MotionRealizer(config.motion_realization).evaluate(
            objective, estimate, world), estimate)
        if case in ('progress', 'reused_id'):
            assert 0. < allowed.allowed_v_mps < .3
            assert allowed.previous_velocity == VelocityTarget(.3, 0.)
            old_wheels = config.navigation.wheel_limits.wheels(.3, 0.)
            actual = config.navigation.wheel_limits.wheels(allowed.allowed_v_mps, allowed.allowed_omega_rad_s)
            assert max(abs(a-b) for a, b in zip(actual, old_wheels)) <= (
                config.operational_constraints.max_acceleration_mps2*.02 + 1e-12)
        else:
            assert allowed.allowed_v_mps == 0.
            assert allowed.previous_velocity is None
        if case in ('missing_proof', 'wrong_scope', 'stale_proof'):
            assert allowed.motion_revoked
            assert ConstraintCode.LOCAL_CLEARANCE in allowed.active_constraints


def test_proof_checks_actual_previous_motion_and_hard_caps_and_stop():
    config = resolved_config().runtime.composition.live_control.control
    current = plan(1, (trajectory('new', .2, 0.),), config, displacement=.055)
    estimate, _ = scene(1)
    motion = MotionIntent(current.context, .2, 0., 100_000_000, current.constraints,
        transition_allowed=True, localization_requirement=current.motion_validity.localization_requirement,
        geometry_required=True, geometry_proof=current.geometry_proof,
        motion_validity=current.motion_validity, approved_velocity=VelocityTarget(.2, 0.))
    previous = OperationalConstraintsStateCheckpoint(scene(0)[0].context, .3, 0.)
    limiter = OperationalConstraintLayer(config.operational_constraints)
    limiter.restore(previous)
    # Proof covers the selected .2, but not L9's actual prior .3 speed.
    allowed = limiter.evaluate(motion, estimate)
    assert allowed.motion_revoked and allowed.allowed_v_mps == 0.
    limiter.restore(previous)
    motion = replace(motion, geometry_proof=replace(current.geometry_proof, max_displacement_m=.075))
    assert limiter.evaluate(motion, estimate).allowed_v_mps > .2
    limiter.restore(previous)
    tightened = replace(motion, constraints=replace(motion.constraints, max_v_mps=.18))
    assert limiter.evaluate(tightened, estimate).allowed_v_mps <= .18
    limiter.restore(previous)
    stopped = limiter.evaluate(replace(motion, requested_v_mps=0., requested_omega_rad_s=0.,
                                      stop_reason='OPERATOR_STOP', transition_allowed=False), estimate)
    assert stopped.motion_revoked
    assert stopped.allowed_v_mps == stopped.allowed_omega_rad_s == 0.
    assert _decode_production_value(encode_value(current), NavigationPlan, 'plan') == current


def test_explore_goal_handoff_uses_replacement_budget_and_keeps_heading():
    config = resolved_config().runtime.composition.live_control.control
    manager = MissionManager(config.mission)
    navigator = TrajectoryNavigator(config.navigation,
        async_config=replace(config.async_l6, enabled=False, completion_inputs=False))
    def evaluate(tick, **values):
        estimate, world = scene(tick, **values)
        mission = manager.evaluate(CommandRequest(estimate.context, 'handoff', CommandMode.EXPLORE, (), tick))
        return navigator.evaluate(mission, estimate, world)
    first = evaluate(0)
    assert first.goal_selection_reason == 'INITIAL_GOAL'
    assert abs(math.atan2(first.local_goal.y_m, first.local_goal.x_m)) < math.pi/4
    # This distance fell inside the old 1-second collision-horizon handoff,
    # but leaves ample time for the actual bounded replacement completion.
    goal = first.local_goal
    near = evaluate(5, x=goal.x_m-.4, v=.4)
    assert near.local_goal == goal
    assert near.goal_selection_reason == first.goal_selection_reason
    assert near.goal_selected_ns == first.goal_selected_ns
    closer = evaluate(10, x=goal.x_m-.15, v=.4, omega=-.5)
    assert closer.local_goal != goal
    assert closer.goal_selection_reason == 'GOAL_HANDOFF'
    assert closer.goal_selected_ns == closer.context.monotonic_ns
    assert abs(math.atan2(closer.local_goal.y_m, closer.local_goal.x_m-(goal.x_m-.15))) < math.pi/4


def test_current_proof_cannot_certify_rotation_inside_obstacle_and_recovery_direction_latches():
    config = resolved_config().runtime.composition.live_control.control
    manager = MissionManager(config.mission)
    navigator = TrajectoryNavigator(config.navigation,
        async_config=replace(config.async_l6, enabled=False, completion_inputs=False))
    estimate, world = scene(0)
    world = replace(world, obstacle_tracks=(ObstacleTrack('close', .1, 0., .1, 0., 0., 1.),))
    mission = manager.evaluate(CommandRequest(estimate.context, 'recovery', CommandMode.EXPLORE, (), 0))
    assert navigator.evaluate(mission, estimate, world).geometry_proof is None
    restored = None
    for tick, omega in ((1, -.8), (2, .8), (3, 0.)):
        estimate, world = scene(tick, omega=omega,
            quality=healthy_localization(local_translation=QualityState.LOST))
        mission = manager.evaluate(CommandRequest(estimate.context, 'recovery', CommandMode.EXPLORE, (), tick))
        recovery = navigator.evaluate(mission, estimate, world)
        assert recovery.velocity_target.v_mps == 0.
        assert recovery.velocity_target.omega_rad_s < 0.
        if restored is not None:
            assert restored.evaluate(mission, estimate, world) == recovery
        else:
            restored = TrajectoryNavigator(config.navigation, async_config=replace(
                config.async_l6, enabled=False, completion_inputs=False))
            restored.restore(navigator.checkpoint())
