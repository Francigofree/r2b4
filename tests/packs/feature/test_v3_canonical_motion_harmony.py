"""Canonical motion-quality contract for Room Cruise and Follow Person.

Source-first intent:
- exercise the real L5 -> L10 canonical motion chain;
- keep replanning asynchronous, with deterministic delayed completions;
- require motion continuity while a valid accepted trajectory exists;
- derive acceleration bounds from the production config instead of hardcoding
  subjective "smoothness" numbers;
- verify the L10 continuously-realizable wheel-speed floor outside deliberate
  L9 acceleration transitions.

This is an offline pytest. It has no hardware/device access and cannot move the
robot. Physical jerk/traction quality still requires live capture evidence.
"""

from __future__ import annotations

from dataclasses import replace
import math

import pytest

from rig import healthy_localization, resolved_config
from v3.contracts import (
    CommandMode,
    CommandRequest,
    ConstraintCode,
    DataField,
    LOCAL_FRAME_ID,
    NavigationStatus,
    ObstacleTrack,
    QualityState,
    RobotEstimate,
    RobotRelativeGeometry,
    RollingLocalCostmap,
    TickContext,
    WorldSnapshot,
)
from v3.contracts.planner import PlannerInput
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator, TrajectoryRolloutComputer
from v3.layers.l7_motion_selection import MotionSelector
from v3.layers.l8_motion_realization import MotionRealizer
from v3.layers.l9_operational_constraints import OperationalConstraintLayer
from v3.layers.l10_chassis_control import DifferentialDriveKinematics


_TICK_NS = 20_000_000  # 50 Hz canonical control-step used by existing tests.
_EPS = 1e-12


def test_escape_preserves_rejected_forward_evidence_and_revokes_braking_origin():
    import time
    from v3.adapters.l6_planner_process import ProcessTrajectoryRolloutBackend
    config = resolved_config().runtime.composition.live_control.control
    manager = MissionManager(config.mission)
    direct_config = replace(config.async_l6, enabled=False, completion_inputs=False)
    navigator = TrajectoryNavigator(config.navigation, async_config=direct_config)
    selector = MotionSelector(config.motion_selection)
    realizer = MotionRealizer(config.motion_realization)
    limiter = OperationalConstraintLayer(config.operational_constraints)
    for tick in range(8):
        estimate, world = _scene(tick, local_sigma_m=.01, local_state=QualityState.GOOD)
        world = replace(world, obstacle_tracks=())
        command = CommandRequest(estimate.context, 'escape-boundary', CommandMode.NAVIGATE,
            (DataField('x_m', 1.0), DataField('y_m', 0.0), DataField('frame_id', LOCAL_FRAME_ID)), tick)
        mission = manager.evaluate(command)
        plan = navigator.evaluate(mission, estimate, world)
        objective = selector.evaluate(plan)
        allowed = limiter.evaluate(realizer.evaluate(objective, estimate, world), estimate)
    assert objective.trajectory.v_mps > 0 and allowed.allowed_v_mps > 0
    old_id = objective.trajectory.candidate_id
    previous_selection = selector.checkpoint()

    estimate, world = _scene(8, local_sigma_m=.01, local_state=QualityState.GOOD)
    world = replace(world, obstacle_tracks=(ObstacleTrack('wall', .5, 0., .15, 0., 0., 1.),))
    mission = manager.evaluate(replace(command, context=estimate.context, expiry_tick=8))
    direct = TrajectoryNavigator(config.navigation, async_config=direct_config)
    plan = direct.evaluate(mission, estimate, world)
    pending = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    pending.evaluate(mission, estimate, world)
    request = pending.pending_rollout_request
    computed = TrajectoryRolloutComputer(config.navigation).compute(request)
    assert computed.trajectory_candidates == plan.trajectory_candidates
    rejected = next(c for c in plan.trajectory_candidates if c.candidate_id == old_id)
    assert rejected.collision or not rejected.progress_viable
    selected = selector.evaluate(plan)
    assert selected.trajectory.candidate_id.startswith('escape-')
    assert not selected.transition_allowed
    allowed = limiter.evaluate(realizer.evaluate(selected, estimate, world), estimate)
    assert allowed.allowed_v_mps <= 0.0
    assert all(len(c.samples) == 1 for c in computed.trajectory_candidates
               if c.candidate_id.startswith('trajectory-'))

    # The same evidence and source identity survive a real spawned planner.
    resolved = resolved_config()
    backend = ProcessTrajectoryRolloutBackend(config.navigation, ready_timeout_s=5.,
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
        assert completion.result == computed
        assert completion.identity.source_context == request.context
    finally:
        backend.close()

    # Near-equivalent, freshly viable escape choices should avoid a linear
    # reversal when a bounded pivot can open the path. Clearance remains gated.
    alternative_world = replace(world, obstacle_tracks=(
        ObstacleTrack('wall', .55, 0., .15, 0., 0., 1.),))
    alternatives = TrajectoryRolloutComputer(config.navigation).compute(
        replace(request, world=alternative_world)).trajectory_candidates
    pivot = next(c for c in alternatives if not c.collision and c.progress_viable
                 and c.v_mps == 0 and c.omega_rad_s > 0)
    reverse = next(c for c in alternatives if not c.collision and c.progress_viable
                   and c.v_mps < 0)
    clearance = min(pivot.min_clearance_m, reverse.min_clearance_m)
    pivot = replace(pivot, total_score=0., min_clearance_m=clearance)
    reverse = replace(reverse, total_score=config.motion_selection.continuity_score_band / 2,
                      min_clearance_m=clearance)
    selector.restore(previous_selection)
    fresh_plan = replace(plan, trajectory_candidates=(pivot, reverse))
    assert selector.evaluate(fresh_plan).trajectory.candidate_id == pivot.candidate_id


def _scene(tick: int, *, local_sigma_m: float, local_state: QualityState):
    """Deterministic local-frame scene with valid robot-relative geometry."""
    context = TickContext(tick, 1_000_000_000 + tick * _TICK_NS)
    covariance = tuple(
        0.65 if index in (0, 6) else 0.01 if index == 12 else 0.0
        for index in range(25)
    )
    estimate = RobotEstimate(
        context,
        LOCAL_FRAME_ID,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        covariance,
        localization_quality=healthy_localization(
            local_translation=local_state,
            heading=QualityState.GOOD,
            global_position=QualityState.LOST,
            local_sigma_m=local_sigma_m,
            global_sigma_m=0.65,
            lidar_age_ns=0,
        ),
    )
    world = WorldSnapshot(
        context,
        LOCAL_FRAME_ID,
        1,
        (ObstacleTrack("person-1", 1.8, 0.0, 0.2, 0.0, 0.0, 1.0),),
        0,
        RollingLocalCostmap(LOCAL_FRAME_ID, 1, 0.1, 2.5, (), 1, 0),
        robot_relative_geometry=RobotRelativeGeometry(
            "ROBOT_BASE", 1, context.monotonic_ns, 100, 0
        ),
    )
    return estimate, world


def _soft_quality(tick: int) -> tuple[float, QualityState]:
    """Cross the live-observed quality bucket without creating true LOST state."""
    phase = tick % 12
    if phase < 4:
        return 0.03984, QualityState.GOOD
    if phase < 8:
        return 0.04009, QualityState.GOOD
    return 0.061, QualityState.DEGRADED


@pytest.mark.control
@pytest.mark.parametrize(
    "mode",
    (CommandMode.EXPLORE, CommandMode.FOLLOW_PERSON),
    ids=("roomcruise", "follow"),
)
def test_roomcruise_follow_canonical_motion_harmony_survives_background_replans(mode):
    """Valid local motion stays non-zero and rate-bounded across replanning churn."""
    config = resolved_config().runtime.composition.live_control.control

    manager = MissionManager(config.mission)
    navigator = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    computer = TrajectoryRolloutComputer(config.navigation)
    selector = MotionSelector(config.motion_selection)
    realizer = MotionRealizer(config.motion_realization)
    limiter = OperationalConstraintLayer(config.operational_constraints)
    chassis = DifferentialDriveKinematics(config.chassis_control)

    pending_request = None
    previous_allowed = None
    previous_context = None
    moving_samples = 0
    moving_while_replanning_samples = 0
    acceleration_limited_samples = 0
    x = y = yaw = 0.0
    goals = set()

    # Cover startup, multiple moving exploration goals and
    # GOOD -> bucket-crossing -> DEGRADED -> GOOD local-quality changes.
    for tick in range(250 if mode is CommandMode.EXPLORE else 36):
        sigma_m, local_state = _soft_quality(tick)
        estimate, world = _scene(
            tick,
            local_sigma_m=sigma_m,
            local_state=local_state,
        )
        if mode is CommandMode.EXPLORE:
            # Move through intermediate goals with changing local quality.
            # A stationary robot never exercised the live near-goal deadlock.
            if previous_allowed is not None:
                v = previous_allowed.allowed_v_mps
                omega = previous_allowed.allowed_omega_rad_s
                x += v * .02 * math.cos(yaw + omega * .01)
                y += v * .02 * math.sin(yaw + omega * .01)
                yaw += omega * .02
                estimate = replace(estimate, x_m=x, y_m=y, yaw_rad=yaw,
                                   v_mps=v, omega_rad_s=omega)
            world = replace(world, obstacle_tracks=())
        mission = manager.evaluate(
            CommandRequest(
                estimate.context,
                f"motion-harmony-{mode.value}",
                mode,
                (DataField("max_v_mps", .3), DataField("max_omega_rad_s", .6)),
                tick,
            )
        )

        # Complete after 160 ms for moving exploration, 20 ms for Follow.
        # Replanning never blocks this deterministic control chain.
        completion = None
        if (pending_request is not None and
                tick - pending_request.context.tick_id >= (8 if mode is CommandMode.EXPLORE else 1)):
            completion = PlannerInput(
                estimate.context,
                pending_request.context,
                computer.compute(pending_request),
            )

        plan = navigator.evaluate(mission, estimate, world, completion)
        pending_request = navigator.pending_rollout_request
        if plan.local_goal is not None:
            goals.add((plan.local_goal.x_m, plan.local_goal.y_m))

        # Initial async fill has no accepted trajectory yet. From the next tick
        # onward there must be valid retained/current guidance.
        if tick == 0:
            assert pending_request is not None, "L6 did not open the async rollout path"
            continue

        objective = selector.evaluate(plan)
        motion = realizer.evaluate(objective, estimate, world)
        allowed = limiter.evaluate(motion, estimate)
        wheels = chassis(allowed)

        assert plan.status is NavigationStatus.ACTIVE, (
            f"{mode.value}: accepted motion disappeared at tick {tick}: {plan.reason}"
        )
        assert motion.stop_reason is None, (
            f"{mode.value}: L8 inserted an unexpected stop at tick {tick}: "
            f"{motion.stop_reason}"
        )

        if ConstraintCode.ACCELERATION_LIMIT in allowed.active_constraints:
            acceleration_limited_samples += 1

        # L9 owns the acceleration envelope. Consecutive body targets may adapt,
        # but they may not jump farther than the production configuration allows.
        if previous_allowed is not None and previous_context is not None:
            dt_s = (
                estimate.context.monotonic_ns - previous_context.monotonic_ns
            ) / 1_000_000_000.0
            assert dt_s > 0.0
            max_dv = config.operational_constraints.max_acceleration_mps2 * dt_s
            max_domega = (
                config.operational_constraints.max_angular_acceleration_rad_s2 * dt_s
            )
            assert abs(allowed.allowed_v_mps - previous_allowed.allowed_v_mps) <= max_dv + _EPS, (
                f"{mode.value}: linear velocity discontinuity at tick {tick}: "
                f"{previous_allowed.allowed_v_mps:.6f} -> {allowed.allowed_v_mps:.6f} m/s "
                f"(max delta {max_dv:.6f})"
            )
            assert abs(
                allowed.allowed_omega_rad_s - previous_allowed.allowed_omega_rad_s
            ) <= max_domega + _EPS, (
                f"{mode.value}: angular velocity discontinuity at tick {tick}: "
                f"{previous_allowed.allowed_omega_rad_s:.6f} -> "
                f"{allowed.allowed_omega_rad_s:.6f} rad/s "
                f"(max delta {max_domega:.6f})"
            )

        # First accepted sample may legitimately still be exactly zero because
        # L9 has dt=0 on first ownership. After one sequential transition tick,
        # valid Room Cruise / Follow guidance must not create stop-go gaps.
        if tick >= 2:
            body_motion = abs(allowed.allowed_v_mps) + abs(allowed.allowed_omega_rad_s)
            wheel_motion = abs(wheels.left_mps) + abs(wheels.right_mps)
            assert body_motion > _EPS, (
                f"{mode.value}: unintended zero-motion gap at tick {tick} "
                f"while L6 remained ACTIVE"
            )
            assert wheel_motion > _EPS, (
                f"{mode.value}: L10 produced an unintended all-zero wheel target "
                f"at tick {tick}"
            )
            moving_samples += 1

            if pending_request is not None:
                moving_while_replanning_samples += 1
                assert body_motion > _EPS

        # Outside an explicit L9 acceleration transition, L10 must not emit a
        # non-zero wheel target below the physical continuous-speed floor.
        if ConstraintCode.ACCELERATION_LIMIT not in allowed.active_constraints:
            floor = config.chassis_control.minimum_continuous_wheel_speed_mps
            for name, wheel_mps in (
                ("left", wheels.left_mps),
                ("right", wheels.right_mps),
            ):
                if abs(wheel_mps) > _EPS:
                    assert abs(wheel_mps) + _EPS >= floor, (
                        f"{mode.value}: stable {name} wheel target {wheel_mps:.6f} m/s "
                        f"fell below continuous floor {floor:.6f} m/s"
                    )

        previous_allowed = allowed
        previous_context = estimate.context

    assert moving_samples >= 20, "scenario did not exercise enough continuous motion"
    assert moving_while_replanning_samples > 0, (
        "scenario did not exercise motion while an L6 background replan was pending"
    )
    assert acceleration_limited_samples > 0, (
        "scenario did not exercise the L9 acceleration-transition path"
    )
    if mode is CommandMode.EXPLORE:
        assert x > .6, "scenario must travel beyond the first intermediate goal"
        assert len(goals) >= 3, "scenario must exercise rolling goal replacement"
