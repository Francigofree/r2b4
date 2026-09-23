from dataclasses import replace

import pytest

from v3.contracts import (
    CommandMode,
    CommandRequest,
    CostmapCell,
    DataField,
    MotionObjectiveKind,
    NavigationStatus,
    ObstacleTrack,
    RobotEstimate,
    RollingLocalCostmap,
    TickContext,
    WorldSnapshot,
)
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import NavigationConfig, TrajectoryNavigator
from v3.layers.l7_motion_selection import select_motion
from v3.layers.l8_motion_realization import MotionRealizationConfig, MotionRealizer


def _estimate(context: TickContext) -> RobotEstimate:
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    return RobotEstimate(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        covariance,
    )


def _world(
    context: TickContext,
    cells: tuple[CostmapCell, ...] = (),
    tracks: tuple[ObstacleTrack, ...] = (),
) -> WorldSnapshot:
    return WorldSnapshot(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        map_revision=1,
        obstacle_tracks=tracks,
        freshness_ns=0,
        local_costmap=RollingLocalCostmap(
            "R2B4_BOOT_ROBOT_MAP",
            revision=1,
            resolution_m=0.1,
            radius_m=2.5,
            occupied_cells=cells,
            source_sequence=1,
            freshness_ns=0,
        ),
    )


def _mission(context: TickContext, mode: CommandMode):
    goal = (
        (DataField("x_m", 1.5), DataField("y_m", 0.0))
        if mode is CommandMode.NAVIGATE
        else (
            DataField("max_v_mps", 0.30),
            DataField("max_omega_rad_s", 0.60),
        )
    )
    return MissionManager().evaluate(
        CommandRequest(
            context,
            f"generic-{mode.value.lower()}",
            mode,
            goal,
            context.tick_id,
        )
    )


@pytest.mark.parametrize("mode", (CommandMode.EXPLORE, CommandMode.NAVIGATE))
def test_explore_and_navigate_share_the_same_generic_trajectory_contract(mode):
    config = NavigationConfig()
    context = TickContext(0, 1_000_000_000)
    estimate = _estimate(context)
    world = _world(context)
    mission = _mission(context, mode)

    first = TrajectoryNavigator(config).evaluate(mission, estimate, world)
    second = TrajectoryNavigator(config).evaluate(mission, estimate, world)

    assert first == second
    assert first.status is NavigationStatus.ACTIVE
    assert first.route == ()
    assert first.local_goal is not None
    assert 30 <= len(first.trajectory_candidates) <= 60

    objective = select_motion(first)
    assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert objective.trajectory is not None
    assert objective.trajectory in first.trajectory_candidates
    assert objective.trajectory.collision is False

    tracking_estimate = replace(
        estimate,
        v_mps=objective.trajectory.v_mps,
        omega_rad_s=objective.trajectory.omega_rad_s,
    )
    realized = MotionRealizer().evaluate(objective, tracking_estimate, world)
    assert realized.requested_v_mps == pytest.approx(
        objective.trajectory.v_mps
    )
    assert realized.requested_omega_rad_s == pytest.approx(
        objective.trajectory.omega_rad_s
    )


def test_obstacle_never_reaches_motion_realization_as_a_colliding_trajectory():
    context = TickContext(0, 1_000_000_000)
    estimate = _estimate(context)
    world = _world(context, (CostmapCell(4, 0, 3),))
    plan = TrajectoryNavigator().evaluate(
        _mission(context, CommandMode.EXPLORE),
        estimate,
        world,
    )

    objective = select_motion(plan)
    assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert objective.trajectory is not None
    assert objective.trajectory.collision is False

    realized = MotionRealizer().evaluate(objective, estimate, world)
    # L8 is a tracking controller: it may add bounded angular correction above
    # the planner's nominal omega. Mission constraints are carried forward for
    # the downstream operational-constraint layer (L9), which owns the clamp.
    assert realized.constraints == plan.constraints
    assert abs(realized.requested_v_mps) <= plan.constraints.max_v_mps + 1e-12
    assert (
        abs(realized.requested_omega_rad_s)
        <= MotionRealizationConfig().max_requested_omega_rad_s + 1e-12
    )


def test_start_collision_fails_closed_at_motion_output():
    context = TickContext(0, 1_000_000_000)
    estimate = _estimate(context)
    world = _world(context, (CostmapCell(0, 0, 1),))
    plan = TrajectoryNavigator().evaluate(
        _mission(context, CommandMode.EXPLORE),
        estimate,
        world,
    )

    objective = select_motion(plan)
    realized = MotionRealizer().evaluate(objective, estimate, world)

    assert objective.kind is MotionObjectiveKind.STOP
    assert objective.selection_reason == "NO_COLLISION_FREE_TRAJECTORY"
    assert realized.requested_v_mps == 0.0
    assert realized.requested_omega_rad_s == 0.0


def test_local_escape_never_drives_forward_into_a_close_front_obstacle():
    context = TickContext(0, 1_000_000_000)
    obstacle = ObstacleTrack(
        track_id="obstacle-front",
        x_m=0.39,
        y_m=0.0,
        radius_m=0.05,
        vx_mps=0.0,
        vy_mps=0.0,
        confidence=1.0,
    )
    estimate = _estimate(context)
    world = _world(context, tracks=(obstacle,))
    plan = TrajectoryNavigator().evaluate(
        _mission(context, CommandMode.NAVIGATE),
        estimate,
        world,
    )

    objective = select_motion(plan)
    assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert objective.trajectory is not None
    assert objective.trajectory.collision is False

    realized = MotionRealizer().evaluate(objective, estimate, world)
    assert realized.requested_v_mps <= 1e-12
    if realized.requested_v_mps < -1e-12:
        assert abs(realized.requested_omega_rad_s) <= 1e-12
    else:
        assert abs(realized.requested_omega_rad_s) > 1e-12


def test_generic_rollout_budget_is_bounded_not_hard_coded_to_one_tuning():
    with pytest.raises(ValueError, match="30 to 60"):
        replace(
            NavigationConfig(),
            rollout_linear_samples=3,
            rollout_angular_samples=3,
        )
