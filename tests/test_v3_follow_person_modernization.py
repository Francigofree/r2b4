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
    MissionConstraints,
    MotionObjectiveKind,
    NavigationPlan,
    NavigationStatus,
    ObstacleTrack,
    RobotEstimate,
    RollingLocalCostmap,
    TickContext,
    TrajectoryEvaluation,
    TrajectoryPose,
    Waypoint,
    WorldSnapshot,
)
from v3.layers.l4_temporal_tracking import PersonMeasurement, TemporalTrackStore
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator
from v3.layers.l7_motion_selection import MotionSelector


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _navigation_config():
    raw = json.loads((PROJECT_ROOT / "conf/vezerles.json").read_text(encoding="utf-8"))
    return v3_navigation_config_from_mapping(raw).navigation


def test_person_uid_is_not_recycled_after_expiry_and_restore():
    store = TemporalTrackStore(
        alpha=0.6,
        beta=0.2,
        prediction_max_age_ns=500_000_000,
        max_speed_mps=6.0,
    )
    store.associate_people(
        (PersonMeasurement(0.9, 1.0, 0.0),),
        captured_ns=1_000_000_000,
        radius_m=0.30,
        max_association_distance_m=0.75,
    )
    assert store.projected_tracks(1_000_000_000)[0].track_id == "person-1"
    store.expire(2_000_000_000, person_max_age_ns=1, other_max_age_ns=1)
    store.associate_people(
        (PersonMeasurement(0.9, 1.2, 0.0),),
        captured_ns=2_000_000_001,
        radius_m=0.30,
        max_association_distance_m=0.75,
    )
    assert store.projected_tracks(2_000_000_001)[0].track_id == "person-2"

    checkpoint = store.checkpoint()
    restored = TemporalTrackStore(
        alpha=0.6,
        beta=0.2,
        prediction_max_age_ns=500_000_000,
        max_speed_mps=6.0,
    )
    restored.restore(checkpoint)
    restored.expire(3_000_000_000, person_max_age_ns=1, other_max_age_ns=1)
    restored.associate_people(
        (PersonMeasurement(0.9, 1.4, 0.0),),
        captured_ns=3_000_000_001,
        radius_m=0.30,
        max_association_distance_m=0.75,
    )
    assert restored.projected_tracks(3_000_000_001)[0].track_id == "person-3"


def _estimate(context: TickContext) -> RobotEstimate:
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    return RobotEstimate(
        context=context,
        frame_id="R2B4_BOOT_ROBOT_MAP",
        x_m=0.0,
        y_m=0.0,
        yaw_rad=0.0,
        v_mps=0.0,
        omega_rad_s=0.0,
        covariance_5x5=covariance,
    )


def _person(track_id: str, x: float = 2.0, y: float = 0.0) -> ObstacleTrack:
    return ObstacleTrack(
        track_id=track_id,
        x_m=x,
        y_m=y,
        radius_m=0.30,
        vx_mps=0.0,
        vy_mps=0.0,
        confidence=0.9,
    )


def _world(context: TickContext, *tracks: ObstacleTrack) -> WorldSnapshot:
    costmap = RollingLocalCostmap(
        frame_id="R2B4_BOOT_ROBOT_MAP",
        revision=1,
        resolution_m=0.1,
        radius_m=2.5,
        occupied_cells=(),
        source_sequence=1,
        freshness_ns=0,
    )
    return WorldSnapshot(
        context=context,
        frame_id="R2B4_BOOT_ROBOT_MAP",
        map_revision=1,
        obstacle_tracks=tuple(tracks),
        freshness_ns=0,
        local_costmap=costmap,
    )


def _mission(context: TickContext):
    return MissionManager().evaluate(
        CommandRequest(
            context=context,
            command_id="modern-follow",
            mode=CommandMode.FOLLOW_PERSON,
            goal=(DataField("max_v_mps", 0.15), DataField("max_omega_rad_s", 0.30)),
            expiry_tick=context.tick_id,
        )
    )


def test_follow_supervisor_performs_bounded_rotation_only_search():
    config = _navigation_config()
    nav = TrajectoryNavigator(config)
    c1 = TickContext(1, 1_000_000_000)
    nav.evaluate(_mission(c1), _estimate(c1), _world(c1, _person("person-1")))

    c2 = TickContext(2, 1_020_000_000)
    nav.evaluate(_mission(c2), _estimate(c2), _world(c2, _person("person-2")))

    c3 = TickContext(3, c2.monotonic_ns + config.follow_person_lost_hold_ns + 1)
    search_center = nav.evaluate(_mission(c3), _estimate(c3), _world(c3, _person("person-2")))
    assert search_center.status is NavigationStatus.ACTIVE
    assert search_center.route[0].x_m == pytest.approx(0.0)
    assert search_center.route[0].y_m == pytest.approx(0.0)
    assert search_center.route[0].yaw_rad == pytest.approx(0.0)
    assert nav.checkpoint().follow_person_state == "SEARCH"

    c4 = TickContext(4, c3.monotonic_ns + config.follow_person_search_step_ns + 1)
    search_right = nav.evaluate(_mission(c4), _estimate(c4), _world(c4, _person("person-2")))
    assert search_right.route[0].x_m == pytest.approx(0.0)
    assert search_right.route[0].y_m == pytest.approx(0.0)
    assert search_right.route[0].yaw_rad == pytest.approx(config.follow_person_search_sweep_rad)

    c5 = TickContext(5, c4.monotonic_ns + config.follow_person_search_step_ns)
    search_left = nav.evaluate(_mission(c5), _estimate(c5), _world(c5, _person("person-2")))
    assert search_left.route[0].yaw_rad == pytest.approx(-config.follow_person_search_sweep_rad)


def _candidate(candidate_id: str, score: float, omega: float) -> TrajectoryEvaluation:
    horizon_ns = 100_000_000
    return TrajectoryEvaluation(
        candidate_id=candidate_id,
        v_mps=0.12,
        omega_rad_s=omega,
        horizon_ns=horizon_ns,
        samples=(TrajectoryPose(0.01, 0.0, 0.0, horizon_ns),),
        collision=False,
        min_clearance_m=0.6,
        progress_score=0.5,
        smoothness_score=0.5,
        novelty_score=0.5,
        total_score=score,
    )


def _plan(tick: int, candidates: tuple[TrajectoryEvaluation, ...]) -> NavigationPlan:
    context = TickContext(tick, 5_000_000_000 + tick * 20_000_000)
    return NavigationPlan(
        context=context,
        mission_id="follow-motion",
        route=(),
        velocity_target=None,
        constraints=MissionConstraints(0.15, 0.30, 0.30, 0.08, 0.10),
        corridor_radius_m=0.30,
        progress=0.0,
        status=NavigationStatus.ACTIVE,
        local_goal=Waypoint(1.0, 0.0),
        trajectory_candidates=candidates,
    )


def test_l7_continuity_survives_candidate_id_change_and_avoids_sign_flip():
    selector = MotionSelector()
    first = selector.evaluate(
        _plan(1, (_candidate("old-grid", 1.0, -0.20), _candidate("other", 0.9, 0.20)))
    )
    assert first.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert first.trajectory is not None and first.trajectory.omega_rad_s < 0.0

    second = selector.evaluate(
        _plan(
            2,
            (
                _candidate("new-best", 1.000, 0.20),
                _candidate("new-continuous", 0.997, -0.18),
            ),
        )
    )
    assert second.trajectory is not None
    assert second.trajectory.candidate_id == "new-continuous"
    assert second.selection_reason == "CONTINUITY_NEAREST:new-continuous"


def test_follow_search_config_is_explicit_and_bounded():
    config = _navigation_config()
    assert config.follow_person_search_timeout_ns > 0
    assert config.follow_person_search_step_ns > 0
    assert 0.0 < config.follow_person_search_sweep_rad < math.pi
