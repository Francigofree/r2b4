from __future__ import annotations

import ast
from dataclasses import replace
from pathlib import Path

import pytest

from v3.composition.native_control import v3_navigation_config_from_mapping
from v3.contracts import (
    CommandMode,
    CommandRequest,
    DataField,
    RobotEstimate,
    RollingLocalCostmap,
    TickContext,
    Waypoint,
    WorldSnapshot,
)
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import (
    AsyncL6PlannerConfig,
    InlineTrajectoryRolloutBackend,
    NavigationConfig,
    TrajectoryNavigator,
    TrajectoryRolloutComputer,
    TrajectoryRolloutRequest,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _estimate(context: TickContext, x_m: float) -> RobotEstimate:
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    return RobotEstimate(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        x_m,
        0.0,
        0.0,
        0.0,
        0.0,
        covariance,
    )


def _world(context: TickContext) -> WorldSnapshot:
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
            occupied_cells=(),
            source_sequence=1,
            freshness_ns=0,
        ),
    )


def _mission(manager: MissionManager, context: TickContext):
    return manager.evaluate(
        CommandRequest(
            context,
            "async-l6-explore",
            CommandMode.EXPLORE,
            (
                DataField("max_v_mps", 0.30),
                DataField("max_omega_rad_s", 0.60),
            ),
            context.tick_id,
        )
    )


def _evaluate(navigator, manager, tick_id: int):
    context = TickContext(tick_id, 1_000_000_000 + tick_id * 20_000_000)
    return navigator.evaluate(
        _mission(manager, context),
        _estimate(context, x_m=0.02 * tick_id),
        _world(context),
    )


def test_pure_rollout_computer_matches_the_canonical_synchronous_l6_kernel():
    config = NavigationConfig()
    context = TickContext(0, 1_000_000_000)
    estimate = _estimate(context, 0.0)
    world = _world(context)
    goal = Waypoint(0.60, 0.0)
    navigator = TrajectoryNavigator(config)
    scene = navigator._build_planning_scene(world)
    expected = navigator._trajectory_rollout(
        estimate,
        world,
        scene,
        goal,
        0.30,
        0.60,
    )
    request = TrajectoryRolloutRequest(
        context=context,
        estimate=estimate,
        world=world,
        goal=goal,
        max_v_mps=0.30,
        max_omega_rad_s=0.60,
        coverage=(),
    )
    actual = TrajectoryRolloutComputer(config).compute(request)
    assert actual.source_context == context
    assert actual.trajectory_candidates == expected


def test_async_latest_plan_handoff_and_pending_checkpoint_restore_are_deterministic():
    config = NavigationConfig()
    original_backend = InlineTrajectoryRolloutBackend(config)
    restored_backend = InlineTrajectoryRolloutBackend(config)
    original = TrajectoryNavigator(
        config,
        rollout_backend=original_backend,
        rollout_release_tick_gap=5,
        max_plan_age_ns=350_000_000,
    )
    original_manager = MissionManager()
    plans = []
    checkpoint = None
    for tick_id in range(6):
        plans.append(_evaluate(original, original_manager, tick_id))
        if tick_id == 5:
            checkpoint = original.checkpoint()

    assert checkpoint is not None
    assert checkpoint.pending_rollout_request is not None
    assert checkpoint.pending_release_tick_id == 10
    assert plans[5].trajectory_candidates == plans[4].trajectory_candidates

    restored = TrajectoryNavigator(
        config,
        rollout_backend=restored_backend,
        rollout_release_tick_gap=5,
        max_plan_age_ns=350_000_000,
    )
    restored.restore(checkpoint)
    restored_manager = MissionManager()
    restored_manager.restore(original_manager.checkpoint())

    for tick_id in range(6, 11):
        original_plan = _evaluate(original, original_manager, tick_id)
        restored_plan = _evaluate(restored, restored_manager, tick_id)
        assert restored_plan == original_plan
        plans.append(original_plan)

    assert plans[10].trajectory_candidates != plans[9].trajectory_candidates
    next_checkpoint = original.checkpoint()
    assert next_checkpoint.pending_rollout_request is not None
    assert next_checkpoint.pending_rollout_request.context.tick_id == 10
    assert next_checkpoint.pending_release_tick_id == 15

    original_backend.close()
    restored_backend.close()


class _NeverReadyBackend:
    def __init__(self):
        self._next_id = 1

    def submit(self, request):
        request_id = self._next_id
        self._next_id += 1
        return request_id

    def take(self, request_id):
        return None

    def abandon(self, request_id):
        pass

    def close(self):
        pass


def test_async_rollout_deadline_miss_fails_closed_as_l6_error():
    config = NavigationConfig()
    navigator = TrajectoryNavigator(
        config,
        rollout_backend=_NeverReadyBackend(),
        rollout_release_tick_gap=5,
        max_plan_age_ns=350_000_000,
    )
    manager = MissionManager()
    for tick_id in range(10):
        _evaluate(navigator, manager, tick_id)
    with pytest.raises(RuntimeError, match="ASYNC_L6_DEADLINE_MISSED"):
        _evaluate(navigator, manager, 10)


def test_async_l6_config_is_explicit_and_defaults_off_for_old_documents():
    base = {
        "v3_navigation": {
            "contract": "R2B4_V3_NAVIGATION_V1",
            "local_perception": {
                "min_range_m": 0.08,
                "max_range_m": 2.5,
                "max_points": 96,
            },
            "rolling_costmap": {
                "resolution_m": 0.1,
                "radius_m": 2.5,
                "max_cell_age_ns": 750000000,
                "max_cells": 1200,
            },
            "exploration": {
                "coverage_cell_size_m": 0.25,
                "coverage_max_cells": 512,
                "local_goal_distance_m": 0.6,
                "local_goal_tolerance_m": 0.15,
                "local_goal_max_age_ns": 8000000000,
                "local_goal_heading_samples": 32,
            },
            "trajectory_rollout": {
                "replan_interval_ns": 100000000,
                "linear_samples": 6,
                "angular_samples": 9,
                "horizon_ns": 800000000,
                "step_count": 8,
                "footprint_length_m": 0.46,
                "footprint_width_m": 0.38,
                "footprint_safety_margin_m": 0.05,
                "clearance_score_cap_m": 1.0,
                "progress_weight": 0.6,
                "clearance_weight": 0.1,
                "smoothness_weight": 0.16,
                "novelty_weight": 0.14,
            },
        }
    }
    old = v3_navigation_config_from_mapping(base)
    assert old.async_l6 == AsyncL6PlannerConfig()
    assert old.async_l6.enabled is False

    enabled = replace(
        old.async_l6,
        enabled=True,
        release_tick_gap=5,
        max_plan_age_ns=350_000_000,
    )
    base["v3_navigation"]["async_l6"] = {
        "enabled": enabled.enabled,
        "release_tick_gap": enabled.release_tick_gap,
        "max_plan_age_ns": enabled.max_plan_age_ns,
    }
    parsed = v3_navigation_config_from_mapping(base)
    assert parsed.async_l6 == enabled


def test_process_adapter_boundary_has_compute_only_dependencies():
    path = PROJECT_ROOT / "v3" / "adapters" / "l6_planner_process.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imported_modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_modules.add(node.module)
    assert imported_modules <= {
        "__future__",
        "contextlib",
        "dataclasses",
        "multiprocessing",
        "os",
        "pathlib",
        "queue",
        "typing",
        "v3.layers.l6_navigation",
        "v3.runtime_performance",
    }
    forbidden = {
        "v3.adapters.gpio_motor",
        "v3.ports",
        "v3.layers.l12_safety_final",
        "v3.adapters.resident_command",
    }
    assert not imported_modules & forbidden
