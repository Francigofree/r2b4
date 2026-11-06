"""Finite terminal yaw stays realizable, capped and replay-checkpointed."""
from dataclasses import replace
import math

import pytest

from rig import healthy_localization, resolved_config
from v3.contracts import (
    CommandMode, CommandRequest, DataField, LOCAL_FRAME_ID, NavigationStatus,
    RobotEstimate, RollingLocalCostmap, TickContext, WorldSnapshot,
)
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator
from v3.layers.l7_motion_selection import MotionSelector
from v3.layers.l8_motion_realization import MotionRealizer


def _scene(tick, yaw):
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
    estimate = RobotEstimate(context, LOCAL_FRAME_ID, 0., 0., yaw, 0., 0.,
        tuple(.01 if index in (0, 6, 12) else 0. for index in range(25)),
        localization_quality=healthy_localization())
    return estimate, WorldSnapshot(context, LOCAL_FRAME_ID, tick, (), 0)


def _mission(manager, estimate, target_yaw, cap):
    return manager.evaluate(CommandRequest(estimate.context, "finite-turn", CommandMode.NAVIGATE,
        tuple(DataField(key, value) for key, value in {
            "x_m": 0., "y_m": 0., "yaw_rad": target_yaw,
            "frame_id": LOCAL_FRAME_ID, "max_omega_rad_s": cap,
        }.items()), estimate.context.tick_id))


@pytest.mark.parametrize("direction,start", ((-1., 0.), (1., 0.), (-1., -3.), (1., 3.)))
def test_finite_yaw_reaches_tolerance_through_calibrated_deadband(direction, start):
    config = resolved_config().runtime.composition.live_control.control
    manager = MissionManager(config.mission)
    navigator = TrajectoryNavigator(config.navigation,
        async_config=replace(config.async_l6, enabled=False, completion_inputs=False))
    selector, realizer = MotionSelector(config.motion_selection), MotionRealizer(config.motion_realization)
    floor = config.motion_realization.wheel_limits.minimum_center_spin_rad_s
    cap = max(floor, config.mission.default_constraints.max_omega_rad_s)
    tolerance = config.mission.default_constraints.yaw_tolerance_rad
    target = math.atan2(math.sin(start + direction * math.pi / 2),
                        math.cos(start + direction * math.pi / 2))
    previous_progress = -1.
    restored = None
    # Includes errors far below floor/heading_gain and just outside tolerance.
    for tick, remaining in enumerate((math.pi / 2, .4, 2 * tolerance, 1.01 * tolerance, .99 * tolerance)):
        yaw = target - direction * remaining
        estimate, world = _scene(tick, yaw)
        mission = _mission(manager, estimate, target, cap)
        plan = navigator.evaluate(mission, estimate, world)
        if restored is not None:
            assert restored.evaluate(mission, estimate, world) == plan
        motion = realizer.evaluate(selector.evaluate(plan), estimate, world)
        if remaining > tolerance:
            assert plan.status is NavigationStatus.ACTIVE
            assert previous_progress <= plan.progress < 1.
            assert motion.requested_v_mps == 0.
            assert floor - 1e-12 <= direction * motion.requested_omega_rad_s <= cap
            assert all(abs(wheel) >= config.motion_realization.wheel_limits.minimum_mps - 1e-12
                for wheel in config.motion_realization.wheel_limits.wheels(0., motion.requested_omega_rad_s))
            assert motion.stop_reason is None
        else:
            assert plan.status is NavigationStatus.COMPLETE and plan.progress == 1.
            assert motion.requested_v_mps == motion.requested_omega_rad_s == 0.
        previous_progress = plan.progress
        if tick == 1:
            restored = TrajectoryNavigator(config.navigation,
                async_config=replace(config.async_l6, enabled=False, completion_inputs=False))
            restored.restore(navigator.checkpoint())


@pytest.mark.parametrize("cap_owner", ("mission", "realization"))
def test_finite_yaw_impossible_cap_is_explicit_hold_without_amplification(cap_owner):
    config = resolved_config().runtime.composition.live_control.control
    floor = config.motion_realization.wheel_limits.minimum_center_spin_rad_s
    if cap_owner == "realization":
        config = replace(config, motion_realization=replace(config.motion_realization,
            max_requested_omega_rad_s=floor * .8))
    manager = MissionManager(config.mission)
    navigator = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    estimate, world = _scene(0, 0.)
    cap = floor * (.8 if cap_owner == "mission" else 1.2)
    mission = _mission(manager, estimate, -math.pi / 2, cap)
    plan = navigator.evaluate(mission, estimate, world)
    motion = MotionRealizer(config.motion_realization).evaluate(
        MotionSelector(config.motion_selection).evaluate(plan), estimate, world)
    assert plan.status is NavigationStatus.ACTIVE and plan.progress == 0.
    assert motion.requested_v_mps == motion.requested_omega_rad_s == 0.
    assert motion.stop_reason == "TERMINAL_YAW_UNREALIZABLE"


@pytest.mark.parametrize("target_x,target_yaw", ((0., 0.), (0., -.1), (.3, 0.), (.3, -.1)))
def test_finite_zero_tolerances_preserve_progress_and_completion(target_x, target_yaw):
    config = resolved_config().runtime.composition.live_control.control
    manager = MissionManager(config.mission)
    navigator = TrajectoryNavigator(config.navigation,
        async_config=replace(config.async_l6, enabled=False, completion_inputs=False))
    estimate, world = _scene(0, 0.)
    world = replace(world, local_costmap=RollingLocalCostmap(LOCAL_FRAME_ID, 0, .1, 2.5, (), 0, 0))
    mission = _mission(manager, estimate, target_yaw, config.mission.default_constraints.max_omega_rad_s)
    mission = replace(mission, target_pose=replace(mission.target_pose, x_m=target_x),
        constraints=replace(mission.constraints, goal_tolerance_m=0., yaw_tolerance_rad=0.))
    plan = navigator.evaluate(mission, estimate, world)
    if target_x == target_yaw == 0.:
        assert plan.status is NavigationStatus.COMPLETE and plan.progress == 1.
    else:
        assert plan.status is NavigationStatus.ACTIVE and plan.progress == 0.
    restored = TrajectoryNavigator(config.navigation,
        async_config=replace(config.async_l6, enabled=False, completion_inputs=False))
    restored.restore(navigator.checkpoint())
    assert restored.evaluate(mission, estimate, world) == plan
