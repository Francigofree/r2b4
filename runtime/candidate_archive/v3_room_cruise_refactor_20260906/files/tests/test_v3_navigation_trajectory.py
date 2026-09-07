from dataclasses import replace

import pytest

from v3.contracts import (
    CommandMode,
    CommandRequest,
    CostmapCell,
    DataField,
    MotionObjectiveKind,
    NavigationStatus,
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
) -> WorldSnapshot:
    return WorldSnapshot(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        map_revision=1,
        obstacle_tracks=(),
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
        CommandRequest(context, f"generic-{mode.value.lower()}", mode, goal, context.tick_id)
    )


@pytest.mark.parametrize("mode", (CommandMode.EXPLORE, CommandMode.NAVIGATE))
def test_explore_and_navigate_share_the_same_generic_trajectory_contract(mode):
    context = TickContext(0, 1_000_000_000)
    estimate = _estimate(context)
    world = _world(context)
    mission = _mission(context, mode)

    first = TrajectoryNavigator().evaluate(mission, estimate, world)
    second = TrajectoryNavigator().evaluate(mission, estimate, world)

    assert first == second
    assert first.status is NavigationStatus.ACTIVE
    assert first.route == ()
    assert first.local_goal is not None
    assert len(first.trajectory_candidates) == 54
    assert len({item.candidate_id for item in first.trajectory_candidates}) == 54
    assert all(len(item.samples) == 8 for item in first.trajectory_candidates)

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

    realized = MotionRealizer().evaluate(objective, estimate, world)
    assert realized.requested_v_mps == expected.v_mps
    assert realized.requested_omega_rad_s == expected.omega_rad_s


def test_footprint_collision_is_scored_in_l6_and_excluded_only_by_l7():
    context = TickContext(0, 1_000_000_000)
    estimate = _estimate(context)
    world = _world(context, (CostmapCell(4, 0, 3),))
    plan = TrajectoryNavigator().evaluate(
        _mission(context, CommandMode.EXPLORE),
        estimate,
        world,
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


def test_l7_fails_closed_when_every_footprint_rollout_collides():
    context = TickContext(0, 1_000_000_000)
    plan = TrajectoryNavigator().evaluate(
        _mission(context, CommandMode.EXPLORE),
        _estimate(context),
        _world(context, (CostmapCell(0, 0, 1),)),
    )

    assert all(item.collision for item in plan.trajectory_candidates)
    objective = select_motion(plan)
    assert objective.kind is MotionObjectiveKind.STOP
    assert objective.selection_reason == "NO_COLLISION_FREE_TRAJECTORY"


def test_generic_rollout_budget_is_fixed_between_thirty_and_sixty_candidates():
    with pytest.raises(ValueError, match="30 to 60"):
        replace(NavigationConfig(), rollout_linear_samples=3, rollout_angular_samples=3)
