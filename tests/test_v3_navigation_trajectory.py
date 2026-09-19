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
from v3.layers.l8_motion_realization import MotionRealizer


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

    expected_count = config.rollout_linear_samples * config.rollout_angular_samples
    assert first == second
    assert first.status is NavigationStatus.ACTIVE
    assert first.route == ()
    assert first.local_goal is not None
    assert len(first.trajectory_candidates) == expected_count
    assert len({item.candidate_id for item in first.trajectory_candidates}) == expected_count
    assert all(
        len(item.samples) == config.rollout_step_count
        for item in first.trajectory_candidates
    )

    objective = select_motion(first)
    assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert objective.selected_source == "navigation.trajectory"
    assert objective.trajectory is not None
    expected = min(
        (item for item in first.trajectory_candidates if not item.collision),
        key=lambda item: (
            -item.total_score,
            -item.min_clearance_m,
            -item.progress_score,
            -item.novelty_score,
            -item.smoothness_score,
            item.candidate_id,
        ),
    )
    assert objective.trajectory == expected

    tracking_estimate = replace(
        estimate,
        v_mps=expected.v_mps,
        omega_rad_s=expected.omega_rad_s,
    )
    realized = MotionRealizer().evaluate(objective, tracking_estimate, world)
    assert realized.requested_v_mps == pytest.approx(expected.v_mps)
    assert realized.requested_omega_rad_s == pytest.approx(expected.omega_rad_s)


def test_footprint_collision_is_scored_in_l6_and_excluded_only_by_l7():
    context = TickContext(0, 1_000_000_000)
    plan = TrajectoryNavigator().evaluate(
        _mission(context, CommandMode.EXPLORE),
        _estimate(context),
        _world(context, (CostmapCell(4, 0, 3),)),
    )

    colliding = tuple(item for item in plan.trajectory_candidates if item.collision)
    viable = tuple(item for item in plan.trajectory_candidates if not item.collision)
    assert colliding
    assert viable
    assert any(item.v_mps > 0.0 for item in colliding)

    objective = select_motion(plan)
    assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert objective.trajectory is not None
    assert objective.trajectory in viable


def test_start_collision_fails_closed_without_exposing_internal_call_counts():
    config = NavigationConfig()
    context = TickContext(0, 1_000_000_000)
    plan = TrajectoryNavigator(config).evaluate(
        _mission(context, CommandMode.EXPLORE),
        _estimate(context),
        _world(context, (CostmapCell(0, 0, 1),)),
    )

    expected_count = config.rollout_linear_samples * config.rollout_angular_samples
    assert len(plan.trajectory_candidates) == expected_count
    assert all(
        len(item.samples) == config.rollout_step_count
        for item in plan.trajectory_candidates
    )
    assert all(item.collision for item in plan.trajectory_candidates)

    objective = select_motion(plan)
    assert objective.kind is MotionObjectiveKind.STOP
    assert objective.selection_reason == "NO_COLLISION_FREE_TRAJECTORY"


def test_local_escape_uses_pivot_or_short_straight_reverse_when_forward_is_bounded():
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
    config = NavigationConfig()
    plan = TrajectoryNavigator(config).evaluate(
        _mission(context, CommandMode.NAVIGATE),
        _estimate(context),
        _world(context, tracks=(obstacle,)),
    )

    expected_count = config.rollout_linear_samples * config.rollout_angular_samples
    assert len(plan.trajectory_candidates) == expected_count
    assert all(
        item.candidate_id.startswith("escape-")
        for item in plan.trajectory_candidates
    )
    assert any(item.v_mps < 0.0 for item in plan.trajectory_candidates)
    assert all(
        abs(item.omega_rad_s) <= 1e-12
        for item in plan.trajectory_candidates
        if item.v_mps < 0.0
    )

    objective = select_motion(plan)
    assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert objective.trajectory is not None
    assert (
        abs(objective.trajectory.v_mps) > 1e-12
        or abs(objective.trajectory.omega_rad_s) > 1e-12
    )


def test_generic_rollout_budget_is_bounded_not_hard_coded_to_one_tuning():
    with pytest.raises(ValueError, match="30 to 60"):
        replace(
            NavigationConfig(),
            rollout_linear_samples=3,
            rollout_angular_samples=3,
        )
