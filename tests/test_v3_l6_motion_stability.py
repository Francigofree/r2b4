"""Motion regressions motivated by the September 23 sampled live runs."""

from dataclasses import replace
import math

import pytest

from v3.contracts import ObstacleTrack, TickContext, Waypoint
from v3.contracts.planner import TrajectoryRolloutRequest
from v3.layers.l6_navigation import TrajectoryNavigator, TrajectoryRolloutComputer
from v3.layers.l7_motion_selection import MotionSelector
from test_v3_follow_person import _estimate, _mission, _person, _production_navigation, _world


def _rollout(config, goal, *, tracks=()):
    context = TickContext(0, 1_000_000_000)
    request = TrajectoryRolloutRequest(
        context, _estimate(context), _world(context, *tracks), goal, 0.30, 0.60, (),
    )
    nav = TrajectoryNavigator(config)
    direct = nav._trajectory_rollout(
        request.estimate, request.world, nav._build_planning_scene(request.world),
        goal, request.max_v_mps, request.max_omega_rad_s,
    )
    result = TrajectoryRolloutComputer(config).compute(request)
    assert result.source_context == context
    assert result.trajectory_candidates == direct
    return result.trajectory_candidates


def test_productive_heading_alignment_is_rewarded_in_the_ranking():
    candidates = _rollout(_production_navigation(), Waypoint(-0.6, 0.0))
    by_id = {candidate.candidate_id: candidate for candidate in candidates}
    slow = by_id['trajectory-00-05']
    fast = by_id['trajectory-00-08']
    assert fast.progress_potential_score > slow.progress_potential_score > 0.0
    assert fast.total_score > slow.total_score


@pytest.mark.parametrize("side", [-1, 1])
@pytest.mark.parametrize("distance", [0.3, 0.6, 2.0])
def test_goal_alignment_overcomes_wrong_way_smoothness_and_continuity(side, distance):
    from test_v3_progress_viability import _plan
    from v3.layers.l7_motion_selection import MotionSelectionStateCheckpoint

    config = _production_navigation()
    context = TickContext(0, 1_000_000_000)
    bearing = side * 0.507
    goal = Waypoint(distance * math.cos(bearing), distance * math.sin(bearing))
    # Live failure: positive distance progress masks increasing heading error.
    # Exercise both mirror images and goal ranges with wrong-way initial motion.
    estimate = replace(_estimate(context), v_mps=0.10, omega_rad_s=-side * 0.08)
    request = TrajectoryRolloutRequest(context, estimate, _world(context), goal, 0.12, 0.30, ())
    candidates = TrajectoryRolloutComputer(config).compute(request).trajectory_candidates
    nav = TrajectoryNavigator(config)
    assert candidates == nav._trajectory_rollout(
        estimate, request.world, nav._build_planning_scene(request.world), goal, 0.12, 0.30,
    )
    plan = _plan(candidates)
    selector = MotionSelector()
    selector.restore(MotionSelectionStateCheckpoint(plan.mission_id, "previous", 0.12, -side * 0.075))
    chosen = selector.evaluate(plan).trajectory
    assert chosen is not None and chosen.progress_viable and not chosen.collision
    assert chosen.omega_rad_s * side > 0.0
    end = chosen.samples[-1]
    final_error = abs(math.remainder(math.atan2(goal.y_m - end.y_m, goal.x_m - end.x_m) - end.yaw_rad, 2 * math.pi))
    assert final_error < abs(bearing)
    assert math.hypot(goal.x_m - end.x_m, goal.y_m - end.y_m) < distance


def test_clearance_recovery_prefers_retreat_over_unproductive_pivot():
    obstacle = ObstacleTrack('front', 0.39, 0.0, 0.05, 0.0, 0.0, 1.0)
    candidates = _rollout(_production_navigation(), Waypoint(0.6, 0.0), tracks=(obstacle,))
    from test_v3_progress_viability import _plan

    objective = MotionSelector().evaluate(_plan(candidates))
    assert objective.trajectory is not None
    assert objective.trajectory.v_mps < 0.0
    assert objective.trajectory.omega_rad_s == 0.0
    assert objective.trajectory.progress_viable
    assert objective.trajectory.progress_potential_score > 0.0
    assert not objective.trajectory.collision


def test_empty_recovery_cannot_spin_without_predicted_improvement():
    from test_v3_progress_viability import _plan

    candidates = _rollout(_production_navigation(), Waypoint(0.0, 0.0))
    objective = MotionSelector().evaluate(_plan(candidates))
    assert objective.kind.value == 'STOP'
    assert objective.selection_reason == 'NO_PROGRESS_VIABLE_TRAJECTORY'


def test_locked_person_survives_confidence_dip_but_is_not_acquired_at_low_confidence():
    config = _production_navigation()
    nav = TrajectoryNavigator(config)
    c0 = TickContext(0, 1_000_000_000)
    low = _person('person-1', 2.0, 0.0, 0.45703125)
    first = nav.evaluate(_mission(c0), _estimate(c0), _world(c0, low))
    assert first.reason == 'PERSON_TARGET_NOT_AVAILABLE'
    c1 = TickContext(1, 1_020_000_000)
    high = replace(low, confidence=0.60546875)
    nav.evaluate(_mission(c1), _estimate(c1), _world(c1, high))
    restored = TrajectoryNavigator(config)
    restored.restore(nav.checkpoint())
    for tick in range(2, 160):
        context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
        args = (_mission(context), _estimate(context), _world(context, low))
        plan = nav.evaluate(*args)
        assert restored.evaluate(*args) == plan
        assert plan.status.value == 'ACTIVE'
        assert plan.trajectory_candidates
        assert nav.checkpoint().follow_person_lost_since_ns is None


@pytest.mark.parametrize("mode", ["NAVIGATE", "EXPLORE"])
def test_transient_world_staleness_preserves_navigation_progress_and_coverage(mode):
    from v3.contracts import CommandMode

    config = _production_navigation()
    nav = TrajectoryNavigator(config)
    for tick in (0, 1):
        c = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
        mission = replace(_mission(c), mode=CommandMode(mode),
                          target_pose=Waypoint(2.0, 0.0) if mode == "NAVIGATE" else None)
        nav.evaluate(mission, replace(_estimate(c), x_m=tick * 0.3), _world(c))
    before = nav.checkpoint()
    assert before.progress > 0.0 if mode == "NAVIGATE" else before.coverage
    c = TickContext(2, 1_040_000_000)
    plan = nav.evaluate(replace(mission, context=c), _estimate(c),
                        replace(_world(c), freshness_ns=config.max_world_freshness_ns + 1))
    assert plan.reason == "WORLD_STALE"
    after = nav.checkpoint()
    assert after.mission_id == before.mission_id
    assert after.initial_distance_m == before.initial_distance_m
    assert after.progress == before.progress
    assert after.coverage == before.coverage
    assert not after.trajectory_candidates


@pytest.mark.parametrize('tracks', [(), (_person('person-2', 2.0, 0.0),),
                                    (_person('person-1', 2.0, 0.0, 0.44),)])
def test_retention_does_not_bypass_loss_or_switch_identity(tracks):
    config = _production_navigation()
    nav = TrajectoryNavigator(config)
    c0 = TickContext(0, 1_000_000_000)
    nav.evaluate(_mission(c0), _estimate(c0), _world(c0, _person('person-1', 2.0, 0.0)))
    deadline = (1_020_000_000 + config.follow_person_lost_hold_ns
                + config.follow_person_search_max_duration_ns + 1)
    for tick, time_ns in [(1, 1_020_000_000), (2, deadline)]:
        context = TickContext(tick, time_ns)
        plan = nav.evaluate(_mission(context), _estimate(context), _world(context, *tracks))
        assert not plan.trajectory_candidates
        assert nav.checkpoint().follow_person_track_id == 'person-1'
    assert plan.reason == 'PERSON_TARGET_LOST'
