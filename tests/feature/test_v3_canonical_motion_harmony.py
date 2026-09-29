"""Canonical motion-quality contract for Room Cruise and Follow Person.

Source-first intent:
- exercise the real L5 -> L10 canonical motion chain;
- keep replanning asynchronous, with deterministic one-tick-late completions;
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

import pytest

from rig import healthy_localization, resolved_config
from v3.contracts import (
    CommandMode,
    CommandRequest,
    ConstraintCode,
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

    # Long enough to cover startup, several normal L6 refresh periods and
    # GOOD -> bucket-crossing -> DEGRADED -> GOOD local-quality changes.
    for tick in range(36):
        sigma_m, local_state = _soft_quality(tick)
        estimate, world = _scene(
            tick,
            local_sigma_m=sigma_m,
            local_state=local_state,
        )
        mission = manager.evaluate(
            CommandRequest(
                estimate.context,
                f"motion-harmony-{mode.value}",
                mode,
                (),
                tick,
            )
        )

        # Model the production async boundary deterministically: the worker
        # finishes the previous request one control tick later. Replanning is
        # therefore present, but never blocks the control chain in this test.
        completion = None
        if pending_request is not None:
            completion = PlannerInput(
                estimate.context,
                pending_request.context,
                computer.compute(pending_request),
            )

        plan = navigator.evaluate(mission, estimate, world, completion)
        pending_request = navigator.pending_rollout_request

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
