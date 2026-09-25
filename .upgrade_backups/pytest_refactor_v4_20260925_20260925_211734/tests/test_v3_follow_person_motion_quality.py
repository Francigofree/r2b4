from __future__ import annotations
from v3_config_fixtures import configured

import json
import math
from dataclasses import replace
from pathlib import Path

import pytest

from v3_config_fixtures import navigation_from_control as v3_navigation_config_from_mapping
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
from v3.layers.l7_motion_selection import select_motion
from v3.layers.l8_motion_realization import MotionRealizer


@pytest.mark.parametrize("side", [-1, 1])
@pytest.mark.parametrize("closed", [False, True])
def test_follow_does_not_choose_a_pivot_when_safe_translation_is_available(side, closed):
    from v3.contracts.planner import PlannerInput
    from v3.layers.l6_navigation import TrajectoryRolloutComputer

    config = _config()
    nav = configured(TrajectoryNavigator, config, completion_inputs=closed)
    c = TickContext(0, 1_000_000_000)
    estimate = replace(_estimate(c), omega_rad_s=side * 0.15)
    world = _world(c, _person(1.613, side * 0.276))
    plan = nav.evaluate(_mission(c, "follow-continuity"), estimate, world)
    if closed:
        request = nav.pending_rollout_request
        result = TrajectoryRolloutComputer(config).compute(request)
        next_c = TickContext(1, 1_020_000_000)
        plan = nav.evaluate(_mission(next_c, "follow-continuity"),
                            replace(estimate, context=next_c), replace(world, context=next_c),
                            PlannerInput(next_c, c, result))
    selected = select_motion(plan).trajectory
    assert selected is not None and not selected.collision and selected.progress_viable
    assert selected.v_mps > 0.0
    assert selected.omega_rad_s * side > 0.0
    # Keep the complete worker result in owned state for checkpoint/replay.
    assert len(nav.checkpoint().trajectory_candidates) == 54


def test_approach_speed_tapers_to_hold_boundary_even_with_a_cached_plan():
    config = _config()
    nav = configured(TrajectoryNavigator, config)
    boundary = config.follow_person_stand_off_m + config.follow_person_distance_deadband_m
    caps = []
    for tick, remaining in enumerate((0.20, 0.10, 0.04, 0.005, 0.001)):
        c = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
        mission = _mission(c, "follow-arrival")
        plan = nav.evaluate(mission, _estimate(c), _world(c, _person(boundary + remaining)))
        assert plan.trajectory_candidates
        expected = 0.15 * min(1.0, remaining / config.follow_person_slowdown_distance_m)
        assert plan.constraints.max_v_mps == pytest.approx(expected)
        assert mission.constraints.max_v_mps == 0.15
        caps.append(plan.constraints.max_v_mps)
        if tick == 0:
            cached = nav.checkpoint().trajectory_candidates
        else:
            assert nav.checkpoint().trajectory_candidates == cached
    assert all(a > b > 0.0 for a, b in zip(caps, caps[1:]))
    assert caps[-1] < 0.001
    c = TickContext(5, 1_100_000_000)
    world = _world(c, _person(boundary))
    plan = nav.evaluate(_mission(c, "follow-arrival"), _estimate(c), world)
    intent = configured(MotionRealizer, ).evaluate(select_motion(plan), _estimate(c), world)
    assert intent.requested_v_mps == intent.requested_omega_rad_s == 0.0


def test_near_hold_planning_uses_the_same_arrival_boundary_as_the_speed_envelope():
    config = _config()
    c = TickContext(0, 1_000_000_000)
    boundary = config.follow_person_stand_off_m + config.follow_person_distance_deadband_m
    remaining = 0.005
    plan = configured(TrajectoryNavigator, config).evaluate(_mission(c, "follow-near"), _estimate(c),
                                               _world(c, _person(boundary + remaining)))
    assert plan.local_goal.x_m == pytest.approx(remaining)
    selected = select_motion(plan).trajectory
    assert selected is not None and selected.progress_viable
    assert 0.0 < selected.v_mps < 0.01


PROJECT_ROOT = (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))


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
    return configured(MissionManager, ).evaluate(
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


def test_motion_quality_config_preserves_behavioral_ordering():
    config = _config()
    assert (
        0.0
        < config.follow_person_align_tolerance_rad
        < config.follow_person_release_tolerance_rad
        < config.follow_person_pivot_enter_rad
        < math.pi
    )
    assert config.follow_person_hold_release_margin_m > 0.0
    assert config.follow_person_slowdown_distance_m > 0.0
    assert config.follow_person_minimum_follow_speed_mps > 0.0
    assert 0.0 < config.follow_person_heading_min_factor <= 1.0


def test_hold_hysteresis_prevents_chatter_and_restarts_translation():
    config = _config()
    nav = configured(TrajectoryNavigator, config)
    command_id = "motion-hold-release"
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

    c1 = TickContext(1, 1_000_000_000)
    hold = nav.evaluate(
        _mission(c1, command_id),
        _estimate(c1),
        _world(c1, _person(inside_hold)),
    )
    assert hold.reason == "PERSON_DISTANCE_HOLD"

    c2 = TickContext(2, 1_020_000_000)
    still_hold = nav.evaluate(
        _mission(c2, command_id),
        _estimate(c2),
        _world(c2, _person(inside_hysteresis)),
    )
    assert still_hold.reason == "PERSON_DISTANCE_HOLD"

    c3 = TickContext(3, 1_040_000_000)
    resumed = nav.evaluate(
        _mission(c3, command_id),
        _estimate(c3),
        _world(c3, _person(beyond_release)),
    )
    max_v = _max_candidate_v(resumed)
    assert 0.0 < max_v <= 0.15


def test_more_follow_distance_restores_more_translation_authority():
    config = _config()
    hold_enter = (
        config.follow_person_stand_off_m
        + config.follow_person_distance_deadband_m
    )
    hold_release = hold_enter + config.follow_person_hold_release_margin_m
    near_distance = hold_release + max(
        0.01,
        0.1 * config.follow_person_slowdown_distance_m,
    )
    far_distance = (
        hold_enter
        + config.follow_person_slowdown_distance_m
        + max(0.01, config.follow_person_hold_release_margin_m)
    )

    c1 = TickContext(10, 2_000_000_000)
    near = configured(TrajectoryNavigator, config).evaluate(
        _mission(c1, "motion-distance-near"),
        _estimate(c1),
        _world(c1, _person(near_distance)),
    )
    c2 = TickContext(11, 2_020_000_000)
    far = configured(TrajectoryNavigator, config).evaluate(
        _mission(c2, "motion-distance-far"),
        _estimate(c2),
        _world(c2, _person(far_distance)),
    )

    near_max = _max_candidate_v(near)
    far_max = _max_candidate_v(far)
    assert 0.0 < near_max < far_max <= 0.15


def test_medium_heading_error_keeps_translation_available():
    config = _config()
    angle = 0.5 * (
        config.follow_person_align_tolerance_rad
        + config.follow_person_pivot_enter_rad
    )
    nav = configured(TrajectoryNavigator, config)
    c1 = TickContext(20, 3_000_000_000)
    plan = nav.evaluate(
        _mission(c1, "motion-heading-medium"),
        _estimate(c1),
        _world(c1, _person(2.0, angle)),
    )
    max_v = _max_candidate_v(plan)
    assert 0.0 < max_v <= 0.15


def test_large_nonpivot_heading_error_keeps_bounded_translation():
    config = _config()
    angle = config.follow_person_pivot_enter_rad - max(
        0.01,
        0.1
        * (
            config.follow_person_pivot_enter_rad
            - config.follow_person_release_tolerance_rad
        ),
    )
    nav = configured(TrajectoryNavigator, config)
    c1 = TickContext(30, 4_000_000_000)
    plan = nav.evaluate(
        _mission(c1, "motion-heading-large"),
        _estimate(c1),
        _world(c1, _person(2.0, angle)),
    )
    max_v = _max_candidate_v(plan)
    assert (
        min(config.follow_person_minimum_follow_speed_mps, 0.15) - 1e-12
        <= max_v
        <= 0.15
    )


def test_pivot_threshold_stops_translation_and_turns_toward_person():
    config = _config()
    angle = min(
        math.pi - 0.01,
        config.follow_person_pivot_enter_rad + 0.05,
    )
    nav = configured(TrajectoryNavigator, config)
    c1 = TickContext(40, 5_000_000_000)
    world = _world(c1, _person(2.0, angle))
    estimate = _estimate(c1)
    plan = nav.evaluate(
        _mission(c1, "motion-pivot"),
        estimate,
        world,
    )
    motion = configured(MotionRealizer, ).evaluate(
        select_motion(plan),
        estimate,
        world,
    )
    assert plan.status is NavigationStatus.ACTIVE
    assert motion.requested_v_mps == 0.0
    assert motion.requested_omega_rad_s > 0.0


def test_minimum_follow_speed_never_overrides_a_lower_operator_limit():
    config = _config()
    operator_limit = 0.8 * config.follow_person_minimum_follow_speed_mps
    hold_release = (
        config.follow_person_stand_off_m
        + config.follow_person_distance_deadband_m
        + config.follow_person_hold_release_margin_m
    )
    target_distance = hold_release + max(
        0.01,
        0.25 * config.follow_person_slowdown_distance_m,
    )
    nav = configured(TrajectoryNavigator, config)
    c1 = TickContext(50, 6_000_000_000)
    plan = nav.evaluate(
        _mission(c1, "motion-low-limit", max_v_mps=operator_limit),
        _estimate(c1),
        _world(c1, _person(target_distance)),
    )
    max_v = _max_candidate_v(plan)
    assert 0.0 < max_v <= operator_limit + 1e-12
