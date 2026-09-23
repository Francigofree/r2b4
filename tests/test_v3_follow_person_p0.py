from __future__ import annotations

import json
import math
from dataclasses import replace
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
    assert config.follow_person_search_max_duration_ns == 10_000_000_000
    assert config.follow_person_slowdown_distance_m > 0.0
    assert config.follow_person_minimum_follow_speed_mps > 0.0
    assert 0.0 < config.follow_person_heading_min_factor <= 1.0


@pytest.mark.parametrize("cap", [0, True, 1.5, 1_000_000_000])
def test_search_hard_cap_must_be_integer_and_cover_timeout(cap):
    with pytest.raises(ValueError, match="follow_person_search_max_duration_ns"):
        replace(_config(), follow_person_search_max_duration_ns=cap)


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
        + _config().follow_person_search_max_duration_ns + 1,
    )
    expired = nav.evaluate(
        _mission(c4),
        _estimate(c4),
        _world(c4, _person("person-b", 1.5, 0.0, 0.99)),
    )
    assert expired.status is NavigationStatus.INVALIDATED
    assert expired.reason == "PERSON_TARGET_LOST"


@pytest.mark.parametrize("omega", [0.3, 1.2, 1e-300])
def test_search_budget_covers_sweep_at_command_speed_but_is_bounded(omega):
    config = _config()
    nav = TrajectoryNavigator(config)
    first = TickContext(0, 1_000_000_000)
    nav.evaluate(_mission(first), _estimate(first), _world(first, _person("person-a", 2, 0)))
    lost = TickContext(1, 1_020_000_000)
    mission = _mission(lost)
    mission = replace(mission, constraints=replace(mission.constraints, max_omega_rad_s=omega))
    nav.evaluate(mission, _estimate(lost), _world(lost))
    budget = nav.checkpoint().follow_person_search_budget_ns
    expected = min(config.follow_person_search_max_duration_ns,
                   math.ceil(min(10.0, 1.35 / omega) * 1e9)
                   + config.follow_person_search_timeout_ns)
    assert budget == expected
    assert nav.follow_person_evidence.search_budget_ns == budget
    deadline = lost.monotonic_ns + config.follow_person_lost_hold_ns + budget
    late = TickContext(2, deadline + 1)
    plan = nav.evaluate(replace(mission, context=late), _estimate(late), _world(late))
    assert plan.reason == "PERSON_TARGET_LOST"
    assert not plan.route and plan.velocity_target is None


def test_search_completes_both_sides_after_old_timeout_and_does_not_restart():
    config = _config()
    nav = TrajectoryNavigator(config)
    first = TickContext(0, 1_000_000_000)
    nav.evaluate(_mission(first), _estimate(first), _world(first, _person("person-a", 2, 0)))
    lost = TickContext(1, 1_020_000_000)
    nav.evaluate(_mission(lost), _estimate(lost), _world(lost))
    start = lost.monotonic_ns + config.follow_person_lost_hold_ns + 1
    for tick, offset, yaw, phase in (
        (2, 0, 0.0, 0),
        (3, 20_000_000, 0.0, 1),
        (4, 1_800_000_000, 0.45, 2),
        (5, 3_000_000_000, 0.10, 2),
    ):
        c = TickContext(tick, start + offset)
        plan = nav.evaluate(_mission(c), _estimate(c, yaw), _world(c))
        assert plan.status is NavigationStatus.ACTIVE
        assert nav.follow_person_evidence.search_phase == phase
    restored = TrajectoryNavigator(config)
    restored.restore(nav.checkpoint())
    for tick, offset in ((6, 5_200_000_000), (7, 5_220_000_000)):
        c = TickContext(tick, start + offset)
        for navigator in (nav, restored):
            plan = navigator.evaluate(_mission(c), _estimate(c, -0.45), _world(c))
            assert plan.reason == "PERSON_TARGET_LOST"
            assert not plan.route
        assert nav.checkpoint() == restored.checkpoint()
    c = TickContext(8, start + 5_240_000_000)
    nav.evaluate(_mission(c), _estimate(c), _world(c, _person("person-a", 2, 0)))
    assert nav.follow_person_evidence.state == "FOLLOW"
    assert nav.checkpoint().follow_person_search_budget_ns is None


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


def test_world_stale_stops_motion_but_retains_lock_for_low_confidence_recovery():
    from v3.layers.l7_motion_selection import select_motion
    from v3.layers.l8_motion_realization import MotionRealizer

    config = _config()
    nav = TrajectoryNavigator(config)
    first = TickContext(0, 1_000_000_000)
    nav.evaluate(_mission(first), _estimate(first), _world(first, _person("person-a", 2, 0)))
    stale = TickContext(1, 1_020_000_000)
    world = replace(_world(stale), freshness_ns=config.max_world_freshness_ns + 1)
    plan = nav.evaluate(_mission(stale), _estimate(stale), world)
    assert plan.reason == "WORLD_STALE"
    assert not plan.route and not plan.trajectory_candidates and plan.velocity_target is None
    intent = MotionRealizer().evaluate(select_motion(plan), _estimate(stale), world)
    assert intent.requested_v_mps == intent.requested_omega_rad_s == 0.0
    checkpoint = nav.checkpoint()
    assert checkpoint.follow_person_track_id == "person-a"
    assert not checkpoint.trajectory_candidates
    restored = TrajectoryNavigator(config)
    restored.restore(checkpoint)
    fresh = TickContext(2, 1_040_000_000)
    world = _world(fresh, _person("person-a", 2, 0, 0.48), _person("person-b", 1.5, 0, 0.99))
    for navigator in (nav, restored):
        plan = navigator.evaluate(_mission(fresh), _estimate(fresh), world)
        assert plan.status is NavigationStatus.ACTIVE
        assert plan.trajectory_candidates
        assert navigator.follow_person_evidence.locked_target_uid == "person-a"
    assert nav.checkpoint() == restored.checkpoint()


@pytest.mark.parametrize("reset", ["new_command", "stop"])
def test_stale_world_does_not_keep_lock_across_mission_reset(reset):
    from v3.contracts import MissionLifecycle

    nav = TrajectoryNavigator(_config())
    first = TickContext(0, 1_000_000_000)
    nav.evaluate(_mission(first), _estimate(first), _world(first, _person("person-a", 2, 0)))
    stale = TickContext(1, 1_020_000_000)
    mission = _mission(stale, "new-command" if reset == "new_command" else "follow-p0")
    if reset == "stop":
        mission = replace(mission, lifecycle=MissionLifecycle.CANCELLED)
    nav.evaluate(mission, _estimate(stale), replace(_world(stale), freshness_ns=999_000_000))
    assert nav.checkpoint().follow_person_track_id is None
    fresh = TickContext(2, 1_040_000_000)
    nav.evaluate(_mission(fresh, "new-command"), _estimate(fresh),
                 _world(fresh, _person("person-a", 2, 0, 0.48), _person("person-b", 2, 0)))
    assert nav.follow_person_evidence.locked_target_uid == "person-b"


def test_stale_world_discards_closed_rollout_and_requires_a_fresh_request():
    from v3.contracts.planner import PlannerInput
    from v3.layers.l6_navigation import TrajectoryRolloutComputer

    config = _config()
    nav = TrajectoryNavigator(config, completion_inputs=True)
    c0 = TickContext(0, 1_000_000_000)
    nav.evaluate(_mission(c0), _estimate(c0), _world(c0, _person("person-a", 2, 0)), PlannerInput(c0))
    old_request = nav.pending_rollout_request
    assert old_request is not None
    old_result = TrajectoryRolloutComputer(config).compute(old_request)
    c1 = TickContext(1, 1_020_000_000)
    stale = replace(_world(c1), freshness_ns=config.max_world_freshness_ns + 1)
    plan = nav.evaluate(_mission(c1), _estimate(c1), stale, PlannerInput(c1, c0, old_result))
    assert plan.reason == "WORLD_STALE" and not plan.trajectory_candidates
    assert nav.pending_rollout_request is None
    checkpoint = nav.checkpoint()
    restored = TrajectoryNavigator(config, completion_inputs=True)
    restored.restore(checkpoint)
    for navigator in (nav, restored):
        for tick in (2, 3):
            c = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
            plan = navigator.evaluate(_mission(c), _estimate(c),
                _world(c, _person("person-a", 2, 0, 0.48)), PlannerInput(c, c0, old_result))
            assert plan.reason == "PLANNER_PENDING" and not plan.trajectory_candidates
            assert navigator.pending_rollout_request.context.tick_id == 2
        request = navigator.pending_rollout_request
        result = TrajectoryRolloutComputer(config).compute(request)
        c4 = TickContext(4, 1_080_000_000)
        plan = navigator.evaluate(_mission(c4), _estimate(c4),
            _world(c4, _person("person-a", 2, 0, 0.48)), PlannerInput(c4, request.context, result))
        assert plan.status is NavigationStatus.ACTIVE and plan.trajectory_candidates
        assert navigator.follow_person_evidence.locked_target_uid == "person-a"
    assert nav.checkpoint() == restored.checkpoint()


def test_stale_world_does_not_restart_search_or_extend_its_deadline():
    config = _config()
    nav = TrajectoryNavigator(config)
    c0 = TickContext(0, 1_000_000_000)
    nav.evaluate(_mission(c0), _estimate(c0), _world(c0, _person("person-a", 2, 0)))
    c1 = TickContext(1, 1_020_000_000)
    nav.evaluate(_mission(c1), _estimate(c1), _world(c1))
    c2 = TickContext(2, c1.monotonic_ns + config.follow_person_lost_hold_ns + 1)
    nav.evaluate(_mission(c2), _estimate(c2), _world(c2))
    before = nav.checkpoint()
    c3 = TickContext(3, c2.monotonic_ns + 20_000_000)
    nav.evaluate(_mission(c3), _estimate(c3), replace(_world(c3), freshness_ns=999_000_000))
    after = nav.checkpoint()
    assert after.follow_person_search_phase == before.follow_person_search_phase
    assert after.follow_person_lost_since_ns == before.follow_person_lost_since_ns
    assert after.follow_person_search_budget_ns == before.follow_person_search_budget_ns
    c4 = TickContext(4, c1.monotonic_ns + config.follow_person_lost_hold_ns + before.follow_person_search_budget_ns + 1)
    plan = nav.evaluate(_mission(c4), _estimate(c4), _world(c4))
    assert plan.reason == "PERSON_TARGET_LOST"
    assert nav.follow_person_evidence.locked_target_uid == "person-a"


@pytest.mark.parametrize("mission_tolerance", [0.10, 0.04])
def test_search_and_realization_share_completion_tolerance(mission_tolerance):
    from v3.layers.l7_motion_selection import select_motion
    from v3.layers.l8_motion_realization import MotionRealizer

    config = _config()
    nav = TrajectoryNavigator(config)
    first = TickContext(0, 1_000_000_000)
    nav.evaluate(_mission(first), _estimate(first), _world(first, _person("person-a", 2, 0)))
    lost = TickContext(1, 1_020_000_000)
    nav.evaluate(_mission(lost), _estimate(lost), _world(lost))
    effective = min(config.follow_person_search_yaw_tolerance_rad, mission_tolerance)
    start = lost.monotonic_ns + config.follow_person_lost_hold_ns + 1
    for tick, yaw in ((2, 0.09), (3, effective + 0.01), (4, effective - 0.01)):
        c = TickContext(tick, start + (tick - 2) * 20_000_000)
        mission = _mission(c)
        mission = replace(mission, constraints=replace(mission.constraints, yaw_tolerance_rad=mission_tolerance))
        plan = nav.evaluate(mission, _estimate(c, yaw), _world(c))
        assert mission.constraints.yaw_tolerance_rad == mission_tolerance
        assert plan.constraints.yaw_tolerance_rad == effective
        assert nav.follow_person_evidence.search_yaw_tolerance_rad == effective
        assert nav.follow_person_evidence.search_phase == (1 if tick == 4 else 0)
        intent = MotionRealizer().evaluate(select_motion(plan), _estimate(c, yaw), _world(c))
        assert abs(intent.requested_omega_rad_s) > 0.0
