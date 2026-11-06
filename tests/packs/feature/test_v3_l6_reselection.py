"""Offline evidence for recovery goal handoff and bounded L7 alternatives."""
from dataclasses import replace
import pickle

import pytest

from rig import resolved_config
from test_v3_cached_geometry import _escape_scene, _seed_escape
from test_v3_explore_recovery import _scene
from v3.contracts import CommandMode, CommandRequest, NavigationStatus, ObstacleTrack, Waypoint
from v3.contracts.planner import PlannerInput
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator, TrajectoryRolloutComputer
from v3.layers.l7_motion_selection import MotionSelector


def _recovering_explore(*, closed):
    # Use a feasible real reverse maneuver, then restore the reported state:
    # its blocked local endpoint remains cached while recovery is committed.
    control, seeded, intent, _request, _result, first = _seed_escape(clearance=.07)
    assert first.status is NavigationStatus.ACTIVE
    checkpoint = seeded.checkpoint()
    old_goal = checkpoint.escape_goal
    assert old_goal is not None and checkpoint.escape_candidate.v_mps < 0.
    key = seeded._coverage_key(old_goal.x_m, old_goal.y_m)
    async_config = (control.async_l6 if closed else
                    replace(control.async_l6, enabled=False, completion_inputs=False))
    navigator = TrajectoryNavigator(control.navigation, async_config=async_config)
    navigator.restore(replace(checkpoint, goal_reachable=False,
        failed_goals=((*key, checkpoint.escape_started_ns + control.navigation.local_goal_max_age_ns),)))

    def inputs(tick, x):
        estimate, world = _escape_scene(control.navigation, tick, revision=tick, clearance=.07)
        estimate = replace(estimate, x_m=x)
        mission = replace(intent(estimate.context), mode=CommandMode.EXPLORE, target_pose=None)
        return mission, estimate, world

    return control, navigator, old_goal, inputs


def _assert_fresh_proof(plan, estimate, world):
    proof, validity = plan.geometry_proof, plan.motion_validity
    assert plan.status is NavigationStatus.ACTIVE
    assert proof is not None and validity is not None
    assert proof.context == estimate.context
    assert proof.geometry_revision == world.local_costmap.revision
    assert proof.geometry_captured_ns == estimate.context.monotonic_ns - world.local_costmap.freshness_ns
    assert proof.usable_at(estimate.context, validity)
    viable = tuple(c for c in plan.trajectory_candidates if not c.collision and c.progress_viable)
    assert viable and all(proof.covers(c.v_mps, c.omega_rad_s) for c in viable)


@pytest.mark.parametrize("closed", (False, True))
def test_completed_explore_recovery_reselects_instead_of_reusing_unreachable_goal(closed):
    control, navigator, old_goal, inputs = _recovering_explore(closed=closed)
    mission, estimate, world = inputs(2, -.25)
    completed = navigator.evaluate(mission, estimate, world)
    assert navigator.checkpoint().escape_candidate is None
    assert navigator.checkpoint().local_goal != old_goal
    assert completed.local_goal != old_goal
    if closed:
        request = navigator.pending_rollout_request
        assert request is not None and request.select_local_goal
        assert request.goal != old_goal
        assert request.estimate.x_m == estimate.x_m
        mission, estimate, world = inputs(3, -.25)
        completed = navigator.evaluate(mission, estimate, world,
            PlannerInput(estimate.context, request.context,
                         TrajectoryRolloutComputer(control.navigation).compute(request)))
    _assert_fresh_proof(completed, estimate, world)
    assert completed.local_goal != old_goal
    objective = MotionSelector(control.motion_selection).evaluate(completed)
    assert objective.trajectory is not None
    assert not objective.trajectory.candidate_id.startswith("escape-")


@pytest.mark.parametrize("closed", (False, True))
def test_next_exit_is_prepared_during_recovery_without_replacing_owned_maneuver(closed):
    control, navigator, old_goal, inputs = _recovering_explore(closed=closed)
    phase = navigator.checkpoint()
    mission, estimate, world = inputs(6, -.1)
    preparing = navigator.evaluate(mission, estimate, world)
    _assert_fresh_proof(preparing, estimate, world)
    assert preparing.local_goal == old_goal
    assert navigator.checkpoint().escape_candidate == phase.escape_candidate
    assert navigator.checkpoint().escape_origin == phase.escape_origin
    assert navigator.checkpoint().escape_started_ns == phase.escape_started_ns
    request = navigator.pending_rollout_request
    if closed:
        assert request is not None and request.select_local_goal
        assert request.context == estimate.context and request.estimate.x_m == -.1
        result = TrajectoryRolloutComputer(control.navigation).compute(request)
        assert result.goal_reachable and result.selected_goal != old_goal
    else:
        assert navigator.checkpoint().local_goal != old_goal
        assert navigator.checkpoint().goal_reachable

    restored = TrajectoryNavigator(control.navigation, async_config=(control.async_l6 if closed else
        replace(control.async_l6, enabled=False, completion_inputs=False)))
    restored.restore(pickle.loads(pickle.dumps(navigator.checkpoint())))
    mission, estimate, world = inputs(7, -.25)
    completion = PlannerInput(estimate.context, request.context, result) if closed else None
    finished = navigator.evaluate(mission, estimate, world, completion)
    assert restored.evaluate(mission, estimate, world, completion) == finished
    assert navigator.checkpoint().escape_candidate is None
    assert finished.local_goal != old_goal
    _assert_fresh_proof(finished, estimate, world)
    selected = MotionSelector(control.motion_selection).evaluate(finished).trajectory
    assert selected is not None and not selected.candidate_id.startswith("escape-")


def _cached_family(motions):
    control = resolved_config().runtime.composition.live_control.control
    navigator = TrajectoryNavigator(control.navigation,
        async_config=replace(control.async_l6, enabled=False, completion_inputs=False))
    manager = MissionManager(control.mission)
    estimate, world = _scene(0)
    mission = manager.evaluate(CommandRequest(estimate.context, "family", CommandMode.EXPLORE, (), 0))
    navigator.evaluate(mission, estimate, world)
    goal = Waypoint(2., 1.)
    scene = replace(navigator._build_planning_scene(world), uncertainty_m=(
        control.navigation.localization_inflation_sigma * estimate.localization_quality.local_sigma_m))
    candidates = tuple(replace(navigator._evaluate_trajectory(index, 0, v, omega,
        estimate, world, scene, goal, .45, 1.4, 1., planning=False), total_score=1. - index * .01)
        for index, (v, omega) in enumerate(motions))
    assert all(not c.collision and c.progress_viable for c in candidates)
    navigator.restore(replace(navigator.checkpoint(), local_goal=goal,
                             trajectory_candidates=candidates, goal_reachable=True))
    return control, navigator, manager, candidates


def test_changed_geometry_skips_colliding_top_candidate_and_preserves_l7_alternative():
    control, navigator, manager, cached = _cached_family(((.45, 0.), (.4, 1.), (.45, 1.4)))
    estimate, world = _scene(1, revision=2,
        tracks=(ObstacleTrack("new-front", .7, 0., .02, 0., 0., 1.),))
    scene = replace(navigator._build_planning_scene(world), uncertainty_m=(
        control.navigation.localization_inflation_sigma * estimate.localization_quality.local_sigma_m))
    top = cached[0]
    checked_top = navigator._evaluate_trajectory(0, 0, top.v_mps, top.omega_rad_s,
        estimate, world, scene, navigator.checkpoint().local_goal, .45, 1.4, 1., planning=False)
    assert checked_top.collision
    mission = manager.evaluate(CommandRequest(estimate.context, "family", CommandMode.EXPLORE, (), 1))
    plan = navigator.evaluate(mission, estimate, world)
    _assert_fresh_proof(plan, estimate, world)
    assert plan.reason == "LOCAL_GUIDANCE_REVALIDATED"
    assert top.candidate_id not in {c.candidate_id for c in plan.trajectory_candidates}
    assert 1 < len(plan.trajectory_candidates) <= 4
    objective = MotionSelector(control.motion_selection).evaluate(plan)
    assert objective.trajectory in plan.trajectory_candidates
    assert plan.geometry_proof.covers(objective.trajectory.v_mps, objective.trajectory.omega_rad_s)


def test_fresh_family_revalidation_evaluates_at_most_four_proposals(monkeypatch):
    control, navigator, manager, _cached = _cached_family(tuple((.3, index * .05) for index in range(7)))
    calls = []
    evaluate = TrajectoryNavigator._evaluate_trajectory

    def counted(self, *args, **kwargs):
        calls.append((args[2], args[3]))
        return evaluate(self, *args, **kwargs)

    monkeypatch.setattr(TrajectoryNavigator, "_evaluate_trajectory", counted)
    estimate, world = _scene(1, revision=2)
    mission = manager.evaluate(CommandRequest(estimate.context, "family", CommandMode.EXPLORE, (), 1))
    plan = navigator.evaluate(mission, estimate, world)
    _assert_fresh_proof(plan, estimate, world)
    assert len(calls) == 4
    assert 1 < len(plan.trajectory_candidates) <= 4
    assert MotionSelector(control.motion_selection).evaluate(plan).trajectory in plan.trajectory_candidates


@pytest.mark.parametrize("closed", (False, True))
def test_speed_fallback_preserves_faster_selection_when_fresh_geometry_proves_it(closed):
    control, seeded, manager, _cached = _cached_family(
        ((.45, 0.), (.385, .1), (.385, .2), (.32, .3), (.19, 0.)))
    navigator = TrajectoryNavigator(control.navigation, async_config=(control.async_l6 if closed else
        replace(control.async_l6, enabled=False, completion_inputs=False)))
    navigator.restore(pickle.loads(pickle.dumps(seeded.checkpoint())))
    estimate, world = _scene(1, revision=2)
    mission = manager.evaluate(CommandRequest(estimate.context, "family", CommandMode.EXPLORE, (), 1))
    plan = navigator.evaluate(mission, estimate, world)
    _assert_fresh_proof(plan, estimate, world)
    assert len(plan.trajectory_candidates) == 4
    assert any(c.v_mps == .19 and c.omega_rad_s == 0. for c in plan.trajectory_candidates)
    selected = MotionSelector(control.motion_selection).evaluate(plan).trajectory
    assert selected.v_mps >= .32
