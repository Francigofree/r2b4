from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from v3.composition.native_control import v3_navigation_config_from_mapping
from v3.contracts import (
    CommandMode,
    CommandRequest,
    DataField,
    NavigationStatus,
    ObstacleTrack,
    RobotEstimate,
    RollingLocalCostmap,
    TickContext,
    WorldSnapshot,
)
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _config():
    raw = json.loads((PROJECT_ROOT / "conf/vezerles.json").read_text(encoding="utf-8"))
    return v3_navigation_config_from_mapping(raw).navigation


def _estimate(context: TickContext, yaw: float = 0.0) -> RobotEstimate:
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    return RobotEstimate(
        context=context,
        frame_id="R2B4_BOOT_ROBOT_MAP",
        x_m=0.0,
        y_m=0.0,
        yaw_rad=yaw,
        v_mps=0.0,
        omega_rad_s=0.0,
        covariance_5x5=covariance,
    )


def _person(track_id: str, x: float, y: float, confidence: float = 0.9) -> ObstacleTrack:
    return ObstacleTrack(
        track_id=track_id,
        x_m=x,
        y_m=y,
        radius_m=0.30,
        vx_mps=0.0,
        vy_mps=0.0,
        confidence=confidence,
    )


def _costmap() -> RollingLocalCostmap:
    return RollingLocalCostmap(
        frame_id="R2B4_BOOT_ROBOT_MAP",
        revision=1,
        resolution_m=0.1,
        radius_m=2.5,
        occupied_cells=(),
        source_sequence=1,
        freshness_ns=0,
    )


def _world(context: TickContext, *tracks: ObstacleTrack) -> WorldSnapshot:
    return WorldSnapshot(
        context=context,
        frame_id="R2B4_BOOT_ROBOT_MAP",
        map_revision=1,
        obstacle_tracks=tuple(tracks),
        freshness_ns=0,
        local_costmap=_costmap(),
    )


def _mission(context: TickContext, command_id: str = "follow-p0"):
    return MissionManager().evaluate(
        CommandRequest(
            context=context,
            command_id=command_id,
            mode=CommandMode.FOLLOW_PERSON,
            goal=(DataField("max_v_mps", 0.15), DataField("max_omega_rad_s", 0.30)),
            expiry_tick=context.tick_id,
        )
    )


def test_p0_config_preserves_follow_person_safety_invariants():
    config = _config()
    assert config.follow_person_lost_hold_ns > 0
    assert (
        0.0
        < config.follow_person_align_tolerance_rad
        < config.follow_person_release_tolerance_rad
        < config.follow_person_pivot_enter_rad
        < math.pi
    )
    assert (
        0.0
        < config.follow_person_min_safe_distance_m
        < (
            config.follow_person_stand_off_m
            - config.follow_person_distance_deadband_m
        )
    )
    assert config.follow_person_hold_release_margin_m > 0.0
    assert config.follow_person_slowdown_distance_m > 0.0
    assert config.follow_person_minimum_follow_speed_mps > 0.0
    assert 0.0 < config.follow_person_heading_min_factor <= 1.0


def test_p0_lost_hold_ignores_other_people_then_uses_bounded_target_recovery():
    nav = TrajectoryNavigator(_config())
    c1 = TickContext(1, 1_000_000_000)
    nav.evaluate(
        _mission(c1),
        _estimate(c1),
        _world(c1, _person("person-a", 1.8, 0.0, 0.95), _person("person-b", 1.5, 0.0, 0.80)),
    )
    c2 = TickContext(2, 1_020_000_000)
    hold = nav.evaluate(
        _mission(c2),
        _estimate(c2),
        _world(c2, _person("person-b", 1.5, 0.0, 0.99)),
    )
    assert hold.status is NavigationStatus.ACTIVE
    assert len(hold.route) == 1

    c3 = TickContext(
        3,
        c2.monotonic_ns + _config().follow_person_lost_hold_ns + 1,
    )
    lost = nav.evaluate(
        _mission(c3),
        _estimate(c3),
        _world(c3, _person("person-b", 1.5, 0.0, 0.99)),
    )
    assert lost.status is NavigationStatus.ACTIVE
    assert len(lost.route) == 1
    assert lost.route[0].yaw_rad == pytest.approx(0.0)

    c4 = TickContext(
        4,
        c2.monotonic_ns
        + _config().follow_person_lost_hold_ns
        + 3_000_000_000,
    )
    expired = nav.evaluate(
        _mission(c4),
        _estimate(c4),
        _world(c4, _person("person-b", 1.5, 0.0, 0.99)),
    )
    assert expired.status is NavigationStatus.INVALIDATED
    assert expired.reason == "PERSON_TARGET_LOST"


def test_p0_same_locked_target_can_return_without_identity_switch():
    nav = TrajectoryNavigator(_config())
    c1 = TickContext(10, 2_000_000_000)
    nav.evaluate(_mission(c1), _estimate(c1), _world(c1, _person("person-a", 1.8, 0.0)))

    c2 = TickContext(11, 2_020_000_000)
    nav.evaluate(_mission(c2), _estimate(c2), _world(c2))

    c3 = TickContext(12, 2_100_000_000)
    recovered = nav.evaluate(
        _mission(c3), _estimate(c3), _world(c3, _person("person-a", 1.8, 0.0))
    )
    assert recovered.status is NavigationStatus.ACTIVE
    assert recovered.route == ()
    assert recovered.trajectory_candidates


def test_p0_medium_heading_error_uses_curved_rollout_not_pivot():
    nav = TrajectoryNavigator(_config())
    c1 = TickContext(20, 3_000_000_000)
    angle = 0.30
    distance = 2.0
    world = _world(
        c1,
        _person("person-a", distance * math.cos(angle), distance * math.sin(angle)),
    )
    plan = nav.evaluate(_mission(c1), _estimate(c1), world)
    assert plan.status is NavigationStatus.ACTIVE
    assert plan.route == ()
    assert plan.trajectory_candidates
    # Heading shaping deliberately lowers the rollout's maximum linear speed.
    max_candidate_v = max(candidate.v_mps for candidate in plan.trajectory_candidates)
    assert 0.0 < max_candidate_v < 0.15


def test_p0_large_heading_error_pivots_until_release_threshold():
    nav = TrajectoryNavigator(_config())
    command_id = "follow-pivot"
    c1 = TickContext(30, 4_000_000_000)
    angle = 0.70
    distance = 2.0
    plan = nav.evaluate(
        _mission(c1, command_id),
        _estimate(c1, yaw=0.0),
        _world(c1, _person("person-a", distance * math.cos(angle), distance * math.sin(angle))),
    )
    assert len(plan.route) == 1
    assert plan.trajectory_candidates == ()

    c2 = TickContext(31, 4_020_000_000)
    still_pivot = nav.evaluate(
        _mission(c2, command_id),
        _estimate(c2, yaw=0.35),
        _world(c2, _person("person-a", distance * math.cos(angle), distance * math.sin(angle))),
    )
    assert len(still_pivot.route) == 1
    assert still_pivot.trajectory_candidates == ()

    c3 = TickContext(32, 4_040_000_000)
    released = nav.evaluate(
        _mission(c3, command_id),
        _estimate(c3, yaw=0.42),
        _world(c3, _person("person-a", distance * math.cos(angle), distance * math.sin(angle))),
    )
    assert released.route == ()
    assert released.trajectory_candidates


def test_p0_standoff_hysteresis_prevents_chatter_and_restarts_motion():
    config = _config()
    nav = TrajectoryNavigator(config)
    command_id = "follow-distance-hysteresis"
    hold_enter = (
        config.follow_person_stand_off_m
        + config.follow_person_distance_deadband_m
    )
    hold_release = hold_enter + config.follow_person_hold_release_margin_m
    inside_hold = 0.5 * (
        config.follow_person_min_safe_distance_m + hold_enter
    )
    inside_hysteresis = (
        hold_enter + 0.5 * config.follow_person_hold_release_margin_m
    )
    beyond_release = hold_release + max(
        0.01,
        0.1 * config.follow_person_hold_release_margin_m,
    )

    c1 = TickContext(40, 5_000_000_000)
    hold = nav.evaluate(
        _mission(c1, command_id),
        _estimate(c1),
        _world(c1, _person("person-a", inside_hold, 0.0)),
    )
    assert len(hold.route) == 1

    c2 = TickContext(41, 5_020_000_000)
    still_hold = nav.evaluate(
        _mission(c2, command_id),
        _estimate(c2),
        _world(c2, _person("person-a", inside_hysteresis, 0.0)),
    )
    assert len(still_hold.route) == 1

    c3 = TickContext(42, 5_040_000_000)
    resumed = nav.evaluate(
        _mission(c3, command_id),
        _estimate(c3),
        _world(c3, _person("person-a", beyond_release, 0.0)),
    )
    assert resumed.route == ()
    assert resumed.trajectory_candidates
    max_candidate_v = max(
        candidate.v_mps for candidate in resumed.trajectory_candidates
    )
    assert 0.0 < max_candidate_v <= 0.15


def test_p0_checkpoint_restore_preserves_target_lock_and_behavior_state():
    config = _config()
    nav = TrajectoryNavigator(config)
    command_id = "follow-checkpoint"

    c1 = TickContext(50, 6_000_000_000)
    nav.evaluate(
        _mission(c1, command_id),
        _estimate(c1),
        _world(c1, _person("person-a", 2.0, 0.0)),
    )
    c2 = TickContext(51, 6_020_000_000)
    nav.evaluate(
        _mission(c2, command_id),
        _estimate(c2),
        _world(c2),
    )
    checkpoint = nav.checkpoint()
    assert checkpoint.follow_person_track_id == "person-a"
    assert checkpoint.follow_person_lost_since_ns == c2.monotonic_ns
    assert checkpoint.follow_person_last_heading_rad == pytest.approx(0.0)

    restored = TrajectoryNavigator(config)
    restored.restore(checkpoint)
    after = restored.checkpoint()
    assert after.follow_person_track_id == "person-a"
    assert after.follow_person_lost_since_ns == c2.monotonic_ns
    assert after.follow_person_pivoting == checkpoint.follow_person_pivoting
    assert after.follow_person_holding == checkpoint.follow_person_holding
    assert after.follow_person_last_heading_rad == checkpoint.follow_person_last_heading_rad
