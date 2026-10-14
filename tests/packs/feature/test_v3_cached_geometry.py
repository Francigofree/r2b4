"""A young cached rollout cannot authorize motion through changed geometry."""
from dataclasses import replace
import pickle

import pytest

from rig import healthy_localization, resolved_config
from v3.contracts import (
    CommandMode, CommandRequest, CostmapCell, DataField, LOCAL_FRAME_ID,
    NavigationStatus, ObstacleTrack, RobotEstimate, RollingLocalCostmap,
    TickContext, WorldSnapshot,
)
from v3.contracts.planner import PlannerInput
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator, TrajectoryRolloutComputer
from v3.layers.l7_motion_selection import MotionSelector


def _scene(tick, *, sigma=.01, blocked=False, rear_wall=False):
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
    estimate = RobotEstimate(context, LOCAL_FRAME_ID, 0., 0., 0., 0., 0., (0.,) * 25,
        localization_quality=replace(healthy_localization(), local_sigma_m=sigma))
    tracks = (ObstacleTrack('person-1', 1.8, 0., .2, 0., 0., 1.),)
    if rear_wall:
        tracks += (ObstacleTrack('rear-wall', -.5, 0., .1, 0., 0., 1.),)
    costmap = RollingLocalCostmap(LOCAL_FRAME_ID, 2 if blocked else 1, .1, 2.5,
        (CostmapCell(0, 0, 1),) if blocked else (), 2 if blocked else 1, tick * 20_000_000)
    return estimate, WorldSnapshot(context, LOCAL_FRAME_ID, 2 if blocked else 1, tracks,
                                   0, costmap)


@pytest.mark.parametrize('mode', (CommandMode.EXPLORE, CommandMode.NAVIGATE, CommandMode.FOLLOW_PERSON))
@pytest.mark.parametrize('change', ('new_obstacle', 'uncertainty'))
def test_young_worker_result_requires_current_geometry_and_restores_guard(mode, change):
    config = resolved_config().runtime.composition.live_control.control
    navigator = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    manager = MissionManager(config.mission)
    selector = MotionSelector(config.motion_selection)

    def mission(context):
        goal = ((DataField('x_m', 1.), DataField('y_m', 0.), DataField('frame_id', LOCAL_FRAME_ID))
                if mode is CommandMode.NAVIGATE else ())
        return manager.evaluate(CommandRequest(context, 'geometry', mode, goal, context.tick_id))

    estimate, world = _scene(0, rear_wall=change == 'uncertainty')
    navigator.evaluate(mission(estimate.context), estimate, world)
    request = navigator.pending_rollout_request
    result = TrajectoryRolloutComputer(config.navigation).compute(request)
    estimate, world = _scene(1, rear_wall=change == 'uncertainty')
    plan = navigator.evaluate(mission(estimate.context), estimate, world,
                              PlannerInput(estimate.context, request.context, result))
    assert plan.status is NavigationStatus.ACTIVE
    assert plan.motion_validity.source_context == request.context
    assert plan.reason != 'LOCAL_GUIDANCE_REVALIDATED'
    selector.evaluate(plan)
    checkpoint = pickle.loads(pickle.dumps(navigator.checkpoint()))
    restored = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    restored.restore(checkpoint)

    estimate, world = _scene(2, blocked=change == 'new_obstacle',
                             sigma=.1 if change == 'uncertainty' else .01,
                             rear_wall=change == 'uncertainty')
    intent = mission(estimate.context)
    changed = navigator.evaluate(intent, estimate, world)
    assert restored.evaluate(intent, estimate, world) == changed
    assert changed.status is NavigationStatus.IDLE
    assert changed.reason == 'PLANNER_GEOMETRY_HOLD'
    assert changed.mission_id == intent.mission_id
    assert not changed.trajectory_candidates
    objective = selector.evaluate(changed)
    assert objective.selection_reason == 'PLANNER_GEOMETRY_HOLD'
    assert objective.trajectory is None and objective.velocity_target is None
    # Revalidation does not rewrite the old result or its source lifetime.
    assert plan.motion_validity.source_context == request.context
    assert result.source_context == request.context


def test_unchanged_geometry_reuses_family_without_renewing_source(monkeypatch):
    config = resolved_config().runtime.composition.live_control.control
    navigator = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    manager = MissionManager(config.mission)

    def evaluate(tick, completion=None):
        estimate, world = _scene(tick)
        mission = manager.evaluate(CommandRequest(estimate.context, 'stable', CommandMode.EXPLORE, (), tick))
        return navigator.evaluate(mission, estimate, world, completion)

    evaluate(0)
    request = navigator.pending_rollout_request
    result = TrajectoryRolloutComputer(config.navigation).compute(request)
    first = evaluate(1, PlannerInput(TickContext(1, 1_020_000_000), request.context, result))

    def unexpected(*args, **kwargs):
        pytest.fail('stable geometry reran a trajectory')

    monkeypatch.setattr(TrajectoryNavigator, '_local_continuation', unexpected)
    second = evaluate(2)
    assert second.trajectory_candidates is first.trajectory_candidates
    assert second.motion_validity == first.motion_validity


def _escape_scene(config, tick, *, kind='reverse', revision=1, clearance=None,
                  sigma=.01, rear_obstacle=False, generation=0, stale=False):
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
    estimate = RobotEstimate(context, LOCAL_FRAME_ID, 0., 0., 0., 0., 0., (0.,) * 25,
        localization_quality=replace(healthy_localization(), local_sigma_m=sigma,
                                     generation=generation))
    if clearance is None:
        clearance = .15 if kind == 'pivot' else .051
    # Keep the physical wall fixed when sigma changes: larger uncertainty must
    # tighten the fresh footprint rather than move the obstacle out of its way.
    front_x = .5 * config.footprint_length_m + .02 + .15 + clearance
    tracks = (ObstacleTrack('front-wall', front_x, 0., .15, 0., 0., 1.),)
    if kind == 'pivot' or rear_obstacle:
        tracks += (ObstacleTrack('rear-wall', -.5, 0., .1, 0., 0., 1.),)
    world = WorldSnapshot(context, LOCAL_FRAME_ID, revision, tracks, 0,
        RollingLocalCostmap(LOCAL_FRAME_ID, revision, .1, 2.5, (), revision,
            config.max_costmap_freshness_ns + 1 if stale else 0))
    return estimate, world


def _seed_escape(kind='reverse', *, clearance=None):
    config = resolved_config().runtime.composition.live_control.control
    navigator = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    manager = MissionManager(config.mission)

    def intent(context):
        return manager.evaluate(CommandRequest(context, 'fresh-escape', CommandMode.NAVIGATE,
            (DataField('x_m', 1.), DataField('y_m', 0.), DataField('frame_id', LOCAL_FRAME_ID)),
            context.tick_id))

    estimate, world = _escape_scene(config.navigation, 0, kind=kind, clearance=clearance)
    navigator.evaluate(intent(estimate.context), estimate, world)
    request = navigator.pending_rollout_request
    result = TrajectoryRolloutComputer(config.navigation).compute(request)
    estimate, world = _escape_scene(config.navigation, 1, kind=kind, clearance=clearance)
    plan = navigator.evaluate(intent(estimate.context), estimate, world,
                              PlannerInput(estimate.context, request.context, result))
    return config, navigator, intent, request, result, plan


@pytest.mark.parametrize('kind', ('reverse', 'pivot'))
def test_fresh_scan_revalidates_existing_reverse_and_pivot_escape(kind):
    config, navigator, intent, request, result, plan = _seed_escape(kind)
    selector = MotionSelector(config.motion_selection)
    old = selector.evaluate(plan)
    assert old.trajectory is not None and old.trajectory.candidate_id.startswith('escape-')
    if kind == 'reverse':
        assert old.trajectory.v_mps < 0.
    else:
        assert old.trajectory.v_mps == 0. and old.trajectory.omega_rad_s != 0.
    restored = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    restored.restore(pickle.loads(pickle.dumps(navigator.checkpoint())))
    estimate, world = _escape_scene(config.navigation, 2, kind=kind, revision=2)
    mission = intent(estimate.context)
    changed = navigator.evaluate(mission, estimate, world)
    assert restored.evaluate(mission, estimate, world) == changed
    assert changed.status is NavigationStatus.ACTIVE
    assert changed.reason == 'LOCAL_GUIDANCE_REVALIDATED'
    assert len(changed.trajectory_candidates) == 1
    fresh = selector.evaluate(changed).trajectory
    assert fresh is not None and not fresh.collision and fresh.progress_viable
    assert (fresh.v_mps, fresh.omega_rad_s) == (old.trajectory.v_mps, old.trajectory.omega_rad_s)
    assert fresh.candidate_id == old.trajectory.candidate_id
    assert changed.motion_validity.source_context == estimate.context
    assert changed.motion_validity.geometry_revision == world.local_costmap.revision
    assert changed.motion_validity.geometry_captured_ns == estimate.context.monotonic_ns
    assert plan.motion_validity.source_context == request.context
    assert result.source_context == request.context


@pytest.mark.parametrize('change', ('rear_obstacle', 'uncertainty', 'stale', 'lineage'))
def test_fresh_escape_rejects_blocked_stale_uncertain_or_changed_lineage(change):
    config, navigator, intent, request, result, plan = _seed_escape()
    selector = MotionSelector(config.motion_selection)
    assert selector.evaluate(plan).trajectory.v_mps < 0.
    estimate, world = _escape_scene(config.navigation, 2, revision=2,
        rear_obstacle=change == 'rear_obstacle', sigma=.1 if change == 'uncertainty' else .01,
        stale=change == 'stale', generation=1 if change == 'lineage' else 0)
    changed = navigator.evaluate(intent(estimate.context), estimate, world)
    assert changed.status is not NavigationStatus.ACTIVE
    objective = selector.evaluate(changed)
    assert objective.trajectory is None and objective.velocity_target is None
    if change in ('rear_obstacle', 'uncertainty'):
        assert changed.reason == 'PLANNER_GEOMETRY_HOLD'
    if change == 'lineage':
        assert not navigator.checkpoint().trajectory_candidates
    assert result.source_context == request.context


def test_escape_revalidation_does_not_relax_original_start_margin_overlap():
    config, navigator, intent, request, result, plan = _seed_escape(clearance=.049)
    assert all(candidate.collision for candidate in result.trajectory_candidates)
    selector = MotionSelector(config.motion_selection)
    assert selector.evaluate(plan).trajectory is None
    estimate, world = _escape_scene(config.navigation, 2, revision=2, clearance=.049)
    changed = navigator.evaluate(intent(estimate.context), estimate, world)
    assert changed.status is NavigationStatus.IDLE
    assert changed.reason == 'PLANNER_GEOMETRY_HOLD'
    assert selector.evaluate(changed).trajectory is None
