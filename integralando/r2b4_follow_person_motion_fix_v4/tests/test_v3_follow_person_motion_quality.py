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


def _person(distance_m: float, angle_rad: float = 0.0) -> ObstacleTrack:
    return ObstacleTrack(
        track_id="person-a",
        x_m=distance_m * math.cos(angle_rad),
        y_m=distance_m * math.sin(angle_rad),
        radius_m=0.30,
        vx_mps=0.0,
        vy_mps=0.0,
        confidence=0.95,
    )


def _world(context: TickContext, person: ObstacleTrack) -> WorldSnapshot:
    return WorldSnapshot(
        context=context,
        frame_id="R2B4_BOOT_ROBOT_MAP",
        map_revision=1,
        obstacle_tracks=(person,),
        freshness_ns=0,
        local_costmap=RollingLocalCostmap(
            frame_id="R2B4_BOOT_ROBOT_MAP",
            revision=1,
            resolution_m=0.1,
            radius_m=2.5,
            occupied_cells=(),
            source_sequence=1,
            freshness_ns=0,
        ),
    )


def _mission(
    context: TickContext,
    command_id: str,
    *,
    max_v_mps: float = 0.15,
    max_omega_rad_s: float = 0.30,
):
    return MissionManager().evaluate(
        CommandRequest(
            context=context,
            command_id=command_id,
            mode=CommandMode.FOLLOW_PERSON,
            goal=(
                DataField("max_v_mps", max_v_mps),
                DataField("max_omega_rad_s", max_omega_rad_s),
            ),
            expiry_tick=context.tick_id,
        )
    )


def _max_candidate_v(plan) -> float:
    assert plan.status is NavigationStatus.ACTIVE
    assert plan.route == ()
    assert plan.trajectory_candidates
    return max(candidate.v_mps for candidate in plan.trajectory_candidates)


def test_motion_quality_config_values():
    config = _config()
    assert config.follow_person_align_tolerance_rad == pytest.approx(0.22)
    assert config.follow_person_release_tolerance_rad == pytest.approx(0.30)
    assert config.follow_person_pivot_enter_rad == pytest.approx(0.55)
    assert config.follow_person_hold_release_margin_m == pytest.approx(0.05)
    assert config.follow_person_slowdown_distance_m == pytest.approx(0.18)
    assert config.follow_person_minimum_follow_speed_mps == pytest.approx(0.10)
    assert config.follow_person_heading_min_factor == pytest.approx(0.75)


def test_hold_releases_after_five_centimetres_and_restart_has_speed_floor():
    nav = TrajectoryNavigator(_config())
    command_id = "motion-hold-release"

    c1 = TickContext(1, 1_000_000_000)
    hold = nav.evaluate(
        _mission(c1, command_id),
        _estimate(c1),
        _world(c1, _person(1.10)),
    )
    assert len(hold.route) == 1
    assert nav.checkpoint().follow_person_holding is True

    c2 = TickContext(2, 1_020_000_000)
    still_hold = nav.evaluate(
        _mission(c2, command_id),
        _estimate(c2),
        _world(c2, _person(1.23)),
    )
    assert len(still_hold.route) == 1
    assert nav.checkpoint().follow_person_holding is True

    c3 = TickContext(3, 1_040_000_000)
    resumed = nav.evaluate(
        _mission(c3, command_id),
        _estimate(c3),
        _world(c3, _person(1.26)),
    )
    assert nav.checkpoint().follow_person_holding is False
    max_v = _max_candidate_v(resumed)
    assert 0.10 <= max_v < 0.15


def test_distance_slowdown_recovers_near_full_speed_by_1_36_m():
    nav = TrajectoryNavigator(_config())
    c1 = TickContext(10, 2_000_000_000)
    plan = nav.evaluate(
        _mission(c1, "motion-distance"),
        _estimate(c1),
        _world(c1, _person(1.36)),
    )
    max_v = _max_candidate_v(plan)
    assert max_v >= 0.13
    assert max_v <= 0.15


def test_medium_heading_error_keeps_most_translation_authority():
    nav = TrajectoryNavigator(_config())
    c1 = TickContext(20, 3_000_000_000)
    plan = nav.evaluate(
        _mission(c1, "motion-heading-medium"),
        _estimate(c1),
        _world(c1, _person(2.0, 0.30)),
    )
    max_v = _max_candidate_v(plan)
    assert max_v >= 0.13
    assert max_v <= 0.15
    assert nav.checkpoint().follow_person_pivoting is False


def test_large_nonpivot_heading_error_does_not_crawl():
    nav = TrajectoryNavigator(_config())
    c1 = TickContext(30, 4_000_000_000)
    plan = nav.evaluate(
        _mission(c1, "motion-heading-large"),
        _estimate(c1),
        _world(c1, _person(2.0, 0.50)),
    )
    max_v = _max_candidate_v(plan)
    assert max_v >= 0.11
    assert nav.checkpoint().follow_person_pivoting is False


def test_pivot_threshold_still_stops_translation():
    nav = TrajectoryNavigator(_config())
    c1 = TickContext(40, 5_000_000_000)
    plan = nav.evaluate(
        _mission(c1, "motion-pivot"),
        _estimate(c1),
        _world(c1, _person(2.0, 0.56)),
    )
    assert plan.status is NavigationStatus.ACTIVE
    assert len(plan.route) == 1
    assert plan.trajectory_candidates == ()
    assert nav.checkpoint().follow_person_pivoting is True


def test_minimum_follow_speed_never_overrides_a_lower_operator_limit():
    nav = TrajectoryNavigator(_config())
    c1 = TickContext(50, 6_000_000_000)
    plan = nav.evaluate(
        _mission(c1, "motion-low-limit", max_v_mps=0.08),
        _estimate(c1),
        _world(c1, _person(1.26)),
    )
    max_v = _max_candidate_v(plan)
    assert 0.0 < max_v <= 0.08 + 1e-12
