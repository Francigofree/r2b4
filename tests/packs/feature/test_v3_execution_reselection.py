"""Fresh negative execution evidence enters bounded L6 reselection."""
from dataclasses import replace
import pickle

import pytest

from rig import healthy_localization
from test_v3_explore_recovery import _scene
from test_v3_l6_reselection import _cached_family, _recovering_explore
from v3.contracts import CommandMode, CommandRequest, NavigationStatus, ObstacleTrack, TickContext
from v3.contracts.planner import PlannerInput
from v3.layers._l6_geometry import _normal_motion_bounds
from v3.layers.l6_navigation import TrajectoryNavigator, TrajectoryRolloutComputer
from v3.layers.l7_motion_selection import MotionSelector
from v3.layers.l8_motion_realization import MotionRealizer
from v3.layers.l9_operational_constraints import (
    OperationalConstraintLayer, OperationalConstraintsStateCheckpoint,
)


def _seed(closed):
    control, seeded, manager, cached = _cached_family(
        ((.45, .35), (.385, .35), (.32, .35), (.32, .5)))
    async_config = (control.async_l6 if closed else
        replace(control.async_l6, enabled=False, completion_inputs=False))
    navigator = TrajectoryNavigator(control.navigation, async_config=async_config)
    navigator.restore(pickle.loads(pickle.dumps(seeded.checkpoint())))
    previous = None
    if closed:
        estimate, world = _scene(0)
        navigator._schedule_or_store_rollout(estimate.context, estimate, world,
            navigator.checkpoint().local_goal, .45, 1.4, allow_exploration_detour=True)
        request = navigator.pending_rollout_request
        previous = (request, TrajectoryRolloutComputer(control.navigation).compute(request))

    def inputs(tick, y):
        estimate, world = _scene(tick, revision=2,
            tracks=(ObstacleTrack("corner", .2, y, .02, 0., 0., 1.),))
        estimate = replace(estimate,
            localization_quality=healthy_localization(local_sigma_m=.01, yaw_sigma_rad=.125))
        mission = manager.evaluate(CommandRequest(estimate.context, "family", CommandMode.EXPLORE, (), tick))
        return mission, estimate, world

    return control, async_config, navigator, cached, previous, inputs


@pytest.mark.parametrize("closed", (False, True))
@pytest.mark.parametrize("y,reason", ((.375, "PLANNER_EXECUTION_HOLD"), (.35, "PLANNER_GEOMETRY_HOLD")))
def test_fresh_failed_family_reselects_without_renewing_progress_or_reviving_old_completion(closed, y, reason):
    control, async_config, navigator, cached, previous, inputs = _seed(closed)
    before = navigator.checkpoint()
    mission, estimate, world = inputs(1, y)
    # This is a real geometric distinction, not a mocked proof rejection:
    # the cached turns cannot be executed, while a straight alternative can.
    scene = replace(navigator._build_planning_scene(world), uncertainty_m=.02)
    alternative = navigator._evaluate_trajectory(9, 0, .19, 0., estimate, world,
        scene, before.local_goal, .45, 1.4, 1., planning=False)
    assert not alternative.collision and alternative.progress_viable
    assert _normal_motion_bounds(alternative, estimate, world, scene, control.navigation,
        control.navigation.motion_proof_horizon_ns, .45, 1.4) is not None
    failed = navigator.evaluate(mission, estimate, world)
    assert failed.status is NavigationStatus.IDLE and failed.reason == reason
    assert failed.geometry_proof is None
    rejected = navigator.checkpoint()
    assert not rejected.goal_reachable
    assert all(not c.progress_viable for c in rejected.trajectory_candidates)
    assert rejected.no_viable_since_ns == estimate.context.monotonic_ns
    assert rejected.pending_rollout_request is None
    assert rejected.explore_progress_started_ns == before.explore_progress_started_ns
    assert rejected.explore_stall_count == before.explore_stall_count
    assert rejected.failed_goals == before.failed_goals

    restored = TrajectoryNavigator(control.navigation, async_config=async_config)
    restored.restore(pickle.loads(pickle.dumps(rejected)))
    mission, estimate, world = inputs(2, y)
    late = (PlannerInput(estimate.context, previous[0].context, previous[1])
            if previous is not None else None)
    assert restored.evaluate(mission, estimate, world, late) == navigator.evaluate(mission, estimate, world, late)
    assert navigator.checkpoint().no_viable_since_ns == rejected.no_viable_since_ns
    assert not navigator.checkpoint().goal_reachable

    # The existing 2*replan-interval hysteresis opens exit selection, without
    # waiting for the 8-second no-progress episode or changing its budget.
    mission, estimate, world = inputs(11, y)
    selected = navigator.evaluate(mission, estimate, world)
    if closed:
        request = navigator.pending_rollout_request
        assert request is not None and request.select_local_goal
        assert request.context.monotonic_ns - rejected.no_viable_since_ns == 2 * control.navigation.trajectory_replan_interval_ns
        result = TrajectoryRolloutComputer(control.navigation).compute(request)
        mission, estimate, world = inputs(12, y)
        selected = navigator.evaluate(mission, estimate, world,
            PlannerInput(estimate.context, request.context, result))
    assert selected.status is NavigationStatus.ACTIVE
    assert selected.geometry_proof is not None
    assert selected.local_goal != before.local_goal
    assert navigator.checkpoint().no_viable_since_ns == rejected.no_viable_since_ns
    assert navigator.checkpoint().explore_progress_started_ns == before.explore_progress_started_ns
    assert navigator.checkpoint().explore_stall_count == before.explore_stall_count
    objective = MotionSelector(control.motion_selection).evaluate(selected)
    assert objective.trajectory is not None
    assert selected.geometry_proof.covers(objective.trajectory.v_mps, objective.trajectory.omega_rad_s)
    motion = MotionRealizer(control.motion_realization).evaluate(objective, estimate, world)
    limiter = OperationalConstraintLayer(control.operational_constraints)
    limiter.restore(OperationalConstraintsStateCheckpoint(TickContext(
        estimate.context.tick_id - 1, estimate.context.monotonic_ns - 20_000_000), 0., 0.))
    allowed = limiter.evaluate(motion, estimate)
    assert abs(allowed.allowed_v_mps) + abs(allowed.allowed_omega_rad_s) > 0.


@pytest.mark.parametrize("case", ("expired", "world_stale", "costmap_stale", "heading_lost", "translation_lost"))
def test_missing_current_execution_evidence_cannot_blacklist_a_goal(case):
    from v3.contracts import QualityState

    control, _, navigator, _, _, inputs = _seed(False)
    mission, estimate, world = inputs(1, .375)
    validity = navigator._guidance_validity(mission, world, trajectory=True, translation=True)
    if case == "expired":
        validity = replace(validity, valid_until_ns=mission.context.monotonic_ns - 1)
    elif case == "world_stale":
        world = replace(world, freshness_ns=control.navigation.max_world_freshness_ns + 1)
    elif case == "costmap_stale":
        world = replace(world, local_costmap=replace(world.local_costmap,
            freshness_ns=control.navigation.max_costmap_freshness_ns + 1))
    else:
        quality = replace(estimate.localization_quality, **{
            "heading" if case == "heading_lost" else "local_translation": QualityState.LOST})
        estimate = replace(estimate, localization_quality=quality)
    before = navigator.checkpoint()
    navigator._reject_unexecutable_explore_family(mission, estimate, world, validity)
    assert navigator.checkpoint() == before


@pytest.mark.parametrize("closed", (False, True))
def test_rejected_owned_reverse_keeps_its_original_bounded_expiry(closed):
    control, navigator, _, inputs = _recovering_explore(closed=closed)
    before = navigator.checkpoint()
    assert before.escape_candidate.v_mps < 0.
    mission, estimate, world = inputs(2, 0.)
    estimate = replace(estimate, localization_quality=replace(
        estimate.localization_quality, yaw_sigma_rad=.4))
    held = navigator.evaluate(mission, estimate, world)
    assert held.status is NavigationStatus.IDLE
    assert held.reason == "PLANNER_EXECUTION_HOLD"
    assert navigator.checkpoint().escape_started_ns == before.escape_started_ns
    assert navigator.checkpoint().escape_candidate == before.escape_candidate
    assert navigator.checkpoint().no_viable_since_ns == before.no_viable_since_ns
    tick = 1 + (2 * before.escape_candidate.horizon_ns + 19_999_999) // 20_000_000
    mission, estimate, world = inputs(tick, 0.)
    expired = navigator.evaluate(mission, estimate, world)
    assert expired.status is NavigationStatus.IDLE
    assert expired.reason == "RECOVERY_EXHAUSTED_HOLD"
    assert navigator.checkpoint().escape_candidate is None
