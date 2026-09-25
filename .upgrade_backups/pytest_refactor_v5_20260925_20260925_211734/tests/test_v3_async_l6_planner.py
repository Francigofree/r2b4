from __future__ import annotations
from v3_config_fixtures import configured

from dataclasses import replace

import pytest

from v3_config_fixtures import navigation_from_control as v3_navigation_config_from_mapping
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
    config = configured(NavigationConfig, )
    context = TickContext(0, 1_000_000_000)
    estimate = _estimate(context, 0.0)
    world = _world(context)
    goal = Waypoint(0.60, 0.0)
    navigator = configured(TrajectoryNavigator, config)
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
    config = configured(NavigationConfig, )
    original_backend = InlineTrajectoryRolloutBackend(config)
    restored_backend = InlineTrajectoryRolloutBackend(config)
    original = configured(TrajectoryNavigator, 
        config,
        rollout_backend=original_backend,
        rollout_release_tick_gap=5,
        max_plan_age_ns=350_000_000,
    )
    original_manager = configured(MissionManager, )
    plans = []
    checkpoint = None
    for tick_id in range(6):
        plans.append(_evaluate(original, original_manager, tick_id))
        if tick_id == 5:
            checkpoint = original.checkpoint()

    assert checkpoint is not None
    assert checkpoint.pending_rollout_request is not None
    assert checkpoint.pending_release_tick_id == 10
    assert checkpoint.pending_release_not_before_ns is None
    assert plans[5].trajectory_candidates == plans[4].trajectory_candidates

    restored = configured(TrajectoryNavigator, 
        config,
        rollout_backend=restored_backend,
        rollout_release_tick_gap=5,
        max_plan_age_ns=350_000_000,
    )
    restored.restore(checkpoint)
    restored_manager = configured(MissionManager, )
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
    assert next_checkpoint.pending_release_not_before_ns is None

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
    config = configured(NavigationConfig, )
    navigator = configured(TrajectoryNavigator, 
        config,
        rollout_backend=_NeverReadyBackend(),
        rollout_release_tick_gap=5,
        max_plan_age_ns=350_000_000,
    )
    manager = configured(MissionManager, )
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
    assert old.async_l6 == configured(AsyncL6PlannerConfig, )
    assert old.async_l6.enabled is False

    enabled = replace(
        old.async_l6,
        enabled=True,
        release_tick_gap=5,
        max_plan_age_ns=350_000_000,
        release_delay_ns=100_000_000,
    )
    base["v3_navigation"]["async_l6"] = {
        "enabled": enabled.enabled,
        "release_tick_gap": enabled.release_tick_gap,
        "max_plan_age_ns": enabled.max_plan_age_ns,
        "release_delay_ns": enabled.release_delay_ns,
    }
    parsed = v3_navigation_config_from_mapping(base)
    assert parsed.async_l6 == enabled


def test_previous_plan_staleness_holds_before_release_tick():
    """The P0 fix must not weaken pre-release fail-closed freshness."""

    config = configured(NavigationConfig, )
    backend = InlineTrajectoryRolloutBackend(config)
    navigator = configured(TrajectoryNavigator, 
        config,
        rollout_backend=backend,
        rollout_release_tick_gap=5,
        max_plan_age_ns=100_000_000,
    )
    manager = configured(MissionManager, )

    for tick_id in range(6):
        _evaluate(navigator, manager, tick_id)

    # Pending result releases at tick 10. At tick 6 the old plan is already
    # 120 ms old, so continuing to drive on it must still fail closed.
    held = _evaluate(navigator, manager, 6)
    assert held.reason == "PLANNER_STALE_HOLD"
    assert not held.trajectory_candidates
    assert held.velocity_target is None

    backend.close()

def _evaluate_at(navigator, manager, tick_id: int, monotonic_ns: int):
    context = TickContext(tick_id, monotonic_ns)
    return navigator.evaluate(
        _mission(manager, context),
        _estimate(context, x_m=0.02 * tick_id),
        _world(context),
    )


def test_time_based_async_handoff_is_stable_under_control_tick_jitter():
    """Reproduce the live failure shape without tying 100 ms to five ticks."""

    config = configured(NavigationConfig, 
        trajectory_replan_interval_ns=100_000_000,
        trajectory_replan_min_tick_gap=5,
    )
    backend = InlineTrajectoryRolloutBackend(config)
    navigator = configured(TrajectoryNavigator, 
        config,
        rollout_backend=backend,
        rollout_release_tick_gap=5,
        rollout_release_delay_ns=100_000_000,
        max_plan_age_ns=350_000_000,
    )
    manager = configured(MissionManager, )

    times = {
        0: 1_000_000_000,
        1: 1_020_000_000,
        2: 1_040_000_000,
        3: 1_060_000_000,
        4: 1_080_000_000,
        5: 1_100_000_000,
        6: 1_130_000_000,
        7: 1_165_000_000,
        8: 1_195_000_000,
        # Four ticks after request, but already 180 ms later.
        9: 1_280_000_000,
    }

    for tick_id in range(6):
        _evaluate_at(navigator, manager, tick_id, times[tick_id])

    pending = navigator.checkpoint()
    assert pending.last_replan_tick_id == 0
    assert pending.pending_rollout_request is not None
    assert pending.pending_rollout_request.context.tick_id == 5
    assert pending.pending_release_tick_id is None
    assert pending.pending_release_not_before_ns == 1_200_000_000

    for tick_id in (6, 7, 8):
        _evaluate_at(navigator, manager, tick_id, times[tick_id])
        assert navigator.checkpoint().last_replan_tick_id == 0

    # Tick 9 is the first closed tick after the 100 ms handoff threshold.
    # The old five-tick scheme would still wait for tick 10.
    _evaluate_at(navigator, manager, 9, times[9])
    after_handoff = navigator.checkpoint()
    assert after_handoff.last_replan_tick_id == 5

    # Because the accepted source snapshot is already >100 ms old, the next
    # async request is scheduled immediately. The historic min-tick-gap must
    # not turn scheduler jitter into another planner-age failure.
    assert after_handoff.pending_rollout_request is not None
    assert after_handoff.pending_rollout_request.context.tick_id == 9
    assert after_handoff.pending_release_tick_id is None
    assert after_handoff.pending_release_not_before_ns == 1_380_000_000

    backend.close()


def test_time_based_async_handoff_budget_must_fit_inside_plan_freshness():
    with pytest.raises(
        ValueError,
        match="release_delay_ns must be shorter than max_plan_age_ns",
    ):
        configured(AsyncL6PlannerConfig, 
            enabled=True,
            release_tick_gap=5,
            max_plan_age_ns=100_000_000,
            release_delay_ns=100_000_000,
        )


class _LateReadyBackend:
    def __init__(self, config):
        self._inner = InlineTrajectoryRolloutBackend(config)
        self._take_calls = 0

    def submit(self, request):
        return self._inner.submit(request)

    def take(self, request_id):
        self._take_calls += 1
        if self._take_calls == 1:
            return None
        return self._inner.take(request_id)

    def abandon(self, request_id):
        self._inner.abandon(request_id)

    def close(self):
        self._inner.close()


def test_late_async_rollout_keeps_fresh_previous_plan():
    config = configured(NavigationConfig, trajectory_replan_interval_ns=100_000_000)
    backend = _LateReadyBackend(config)
    navigator = configured(TrajectoryNavigator, 
        config,
        rollout_backend=backend,
        rollout_release_tick_gap=5,
        rollout_release_delay_ns=100_000_000,
        max_plan_age_ns=350_000_000,
    )
    manager = configured(MissionManager, )

    seed = _evaluate_at(navigator, manager, 0, 1_000_000_000)
    previous_candidates = seed.trajectory_candidates
    _evaluate_at(navigator, manager, 5, 1_100_000_000)

    at_release = _evaluate_at(navigator, manager, 10, 1_200_000_000)
    assert at_release.trajectory_candidates == previous_candidates
    assert navigator.checkpoint().last_replan_ns == 1_000_000_000

    accepted = _evaluate_at(navigator, manager, 11, 1_220_000_000)
    assert accepted.trajectory_candidates != previous_candidates
    assert navigator.checkpoint().last_replan_ns == 1_100_000_000
    backend.close()


def test_async_rollout_holds_when_previous_plan_becomes_stale():
    config = configured(NavigationConfig, trajectory_replan_interval_ns=100_000_000)
    navigator = configured(TrajectoryNavigator, 
        config,
        rollout_backend=_NeverReadyBackend(),
        rollout_release_tick_gap=5,
        rollout_release_delay_ns=100_000_000,
        max_plan_age_ns=350_000_000,
    )
    manager = configured(MissionManager, )

    _evaluate_at(navigator, manager, 0, 1_000_000_000)
    _evaluate_at(navigator, manager, 5, 1_100_000_000)

    _evaluate_at(navigator, manager, 10, 1_200_000_000)

    held = _evaluate_at(navigator, manager, 18, 1_360_000_000)
    assert held.reason == "PLANNER_STALE_HOLD"
    assert not held.trajectory_candidates
    assert held.velocity_target is None

