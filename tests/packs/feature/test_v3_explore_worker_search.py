"""Local search stays on the compute edge; L6 retains acceptance and budgets."""
from dataclasses import replace
import math
import time

import pytest

from rig import resolved_config
from test_v3_explore_no_progress import _rig
from test_v3_explore_recovery import _scene, _request
from v3.contracts import CommandMode, CommandRequest, NavigationStatus, ObstacleTrack, Waypoint
from v3.contracts.planner import PlannerInput
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator, TrajectoryRolloutComputer


def test_front_obstacle_cannot_hide_reachable_side_exit_behind_top_four():
    import v3.layers.l6_navigation as navigation

    config = resolved_config().runtime.composition.live_control.control.navigation
    request = replace(_request(tracks=(
        ObstacleTrack('front', .5, 0., .05, 0., 0., 1.),)), select_local_goal=True)
    computer = TrajectoryRolloutComputer(config)
    scene = replace(computer._planning_scene(request.world), uncertainty_m=(
        config.localization_inflation_sigma * request.estimate.localization_quality.local_sigma_m))
    endpoint_ranked = navigation._rank_local_goal_options(
        config, request.estimate, request.world.local_costmap, scene, None, {}, {}, 0)
    assert len(endpoint_ranked) == 4
    assert not any(navigation._local_goal_reachable(config, scene, request, goal)
                   for goal in endpoint_ranked)

    result = computer.compute(request)
    assert result.goal_reachable and result.selected_goal not in endpoint_ranked
    assert 0 < len(result.goal_options) <= 4
    assert all(navigation._local_goal_reachable(config, scene, request, goal)
               for goal in result.goal_options)
    assert any(not candidate.collision and candidate.progress_viable
               and candidate.candidate_id.startswith('trajectory-')
               for candidate in result.trajectory_candidates)


def test_exit_mask_tracks_geometry_without_revision_or_failed_goal_noise():
    config = resolved_config().runtime.composition.live_control.control.navigation
    front = (ObstacleTrack('front', .5, 0., .05, 0., 0., 1.),)
    request = replace(_request(tracks=front), select_local_goal=True)
    computer = TrajectoryRolloutComputer(config)
    blocked = computer.compute(request)
    assert blocked.reachable_exit_mask
    assert not blocked.reachable_exit_mask & 1  # Fixed ODOM +X corridor.

    goal = blocked.selected_goal
    cell = (math.floor(goal.x_m / config.coverage_cell_size_m),
            math.floor(goal.y_m / config.coverage_cell_size_m))
    newer_world = replace(request.world, map_revision=37,
        local_costmap=replace(request.world.local_costmap, revision=37, source_sequence=99))
    revised = computer.compute(replace(request, world=newer_world,
        failed_goals=((*cell, request.context.monotonic_ns + 1_000_000_000),)))
    assert revised.selected_goal != blocked.selected_goal
    assert revised.reachable_exit_mask == blocked.reachable_exit_mask

    opened = computer.compute(replace(request,
        world=replace(request.world, obstacle_tracks=())))
    assert opened.reachable_exit_mask & 1
    assert opened.reachable_exit_mask & ~blocked.reachable_exit_mask
    assert not blocked.reachable_exit_mask & ~opened.reachable_exit_mask


def test_probe_ranking_can_only_select_a_previously_unavailable_exit():
    from v3.layers.l6_navigation import _exit_heading_bit

    config = resolved_config().runtime.composition.live_control.control.navigation
    request = replace(_request(tracks=(
        ObstacleTrack('front', .5, 0., .05, 0., 0., 1.),)), select_local_goal=True)
    computer = TrajectoryRolloutComputer(config)
    known = computer.compute(request).reachable_exit_mask
    unchanged = computer.compute(replace(request, excluded_exit_mask=known))
    assert unchanged.reachable_exit_mask == known
    assert not unchanged.goal_reachable and not unchanged.goal_options

    opened = computer.compute(replace(request, excluded_exit_mask=known,
        world=replace(request.world, obstacle_tracks=())))
    assert opened.goal_reachable and opened.goal_options
    assert all(not _exit_heading_bit(request.estimate, goal) & known
               for goal in opened.goal_options)
    assert all(_exit_heading_bit(request.estimate, goal) & opened.reachable_exit_mask
               for goal in opened.goal_options)


def test_closed_tick_only_snapshots_search_and_worker_obeys_failed_cells(monkeypatch):
    import v3.layers.l6_navigation as navigation
    import v3.layers._l6_rollout as rollout
    control, navigator, evaluate = _rig(closed=True)
    rank = navigation._rank_local_goal_options

    def forbidden(*args):
        pytest.fail("control tick performed local goal search")

    monkeypatch.setattr(navigation, '_rank_local_goal_options', forbidden)
    monkeypatch.setattr(rollout, '_rank_local_goal_options', forbidden)
    first = evaluate(0)
    request = navigator.pending_rollout_request
    assert first.status in (NavigationStatus.ACTIVE, NavigationStatus.PENDING)
    assert request.select_local_goal and request.alternate_goals == ()
    monkeypatch.setattr(navigation, '_rank_local_goal_options', rank)
    monkeypatch.setattr(rollout, '_rank_local_goal_options', rank)
    computer = TrajectoryRolloutComputer(control.navigation)
    result = computer.compute(request)
    assert result.goal_reachable and 0 < len(result.goal_options) <= 4
    cell = navigator._coverage_key(result.selected_goal.x_m, result.selected_goal.y_m)
    blocked = replace(request, failed_goals=((*cell, request.context.monotonic_ns + 1_000_000_000),))
    replacement = computer.compute(blocked)
    assert all(navigator._coverage_key(goal.x_m, goal.y_m) != cell
               for goal in replacement.goal_options)


def test_pending_replacement_never_changes_committed_continuation_goal():
    control, navigator, evaluate = _rig(closed=True)
    evaluate(0)
    assert evaluate(1).status is NavigationStatus.ACTIVE
    checkpoint = navigator.checkpoint()
    committed = checkpoint.local_goal
    estimate, world = _scene(2)
    pending = replace(_request(), context=estimate.context, estimate=estimate, world=world,
                      goal=Waypoint(-.9, 0.), select_local_goal=False)
    navigator.restore(replace(checkpoint, pending_rollout_request=pending,
                             pending_goal_selected_ns=estimate.context.monotonic_ns))
    mission = MissionManager(control.mission).evaluate(CommandRequest(
        estimate.context, 'bounded', CommandMode.EXPLORE, (), 2))
    plan = navigator._local_continuation(mission, estimate, world)
    assert plan.status is NavigationStatus.ACTIVE
    assert plan.local_goal == committed and plan.local_goal != pending.goal


@pytest.mark.parametrize('alternating', (False, True))
def test_no_path_and_viability_flips_cannot_renew_measured_progress(alternating):
    control, navigator, evaluate = _rig(closed=True)
    evaluate(0)
    evaluate(1)
    origin = navigator.checkpoint().explore_progress_started_ns
    expiry = math.ceil(control.navigation.local_goal_max_age_ns / 20_000_000)
    manager = MissionManager(control.mission)

    def tick_without_completion(tick):
        estimate, world = _scene(tick, revision=tick)
        mission = manager.evaluate(CommandRequest(estimate.context, 'bounded', CommandMode.EXPLORE, (), tick))
        return navigator.evaluate(mission, estimate, world)

    for tick in range(2, expiry, 5):
        checkpoint = navigator.checkpoint()
        reachable = alternating and tick % 2 == 1
        navigator.restore(replace(checkpoint, goal_reachable=reachable))
        tick_without_completion(tick)
        assert navigator.checkpoint().explore_progress_started_ns == origin
    # At expiry there is an explicit bounded HOLD even if the last result
    # switches between no-path and viable without any measured translation.
    checkpoint = navigator.checkpoint()
    navigator.restore(replace(checkpoint, goal_reachable=alternating))
    held = tick_without_completion(expiry)
    assert held.status is NavigationStatus.IDLE
    assert held.reason == ('NO_PROGRESS_HOLD' if alternating else 'NO_PATH_HOLD')
    assert navigator.checkpoint().explore_stall_count == 1
    assert navigator.pending_rollout_request is None


def test_no_viable_reselection_still_runs_after_its_max_age_boundary():
    control, navigator, evaluate = _rig(closed=True)
    evaluate(0)
    evaluate(1)
    expiry = math.ceil(control.navigation.local_goal_max_age_ns / 20_000_000) + 1
    estimate, world = _scene(expiry)
    checkpoint = navigator.checkpoint()
    navigator.restore(replace(checkpoint,
        trajectory_candidates=tuple(replace(c, progress_viable=False) for c in checkpoint.trajectory_candidates),
        no_viable_since_ns=checkpoint.explore_progress_started_ns,
        # Actual recent translation legitimately renews the execution window;
        # an old unviable goal still needs explicit handling on its own branch.
        explore_progress_started_ns=estimate.context.monotonic_ns - 20_000_000))
    mission = MissionManager(control.mission).evaluate(CommandRequest(
        estimate.context, 'bounded', CommandMode.EXPLORE, (), expiry))
    navigator.evaluate(mission, estimate, world)
    request = navigator.pending_rollout_request
    assert request is not None and request.select_local_goal
    assert navigator.checkpoint().pending_goal_selection_reason == 'NO_VIABLE_PATH'


def test_seed_cannot_retry_a_failed_goal_while_search_is_pending():
    control, navigator, evaluate = _rig(closed=True)
    evaluate(0)
    checkpoint = navigator.checkpoint()
    request = navigator.pending_rollout_request
    cell = navigator._coverage_key(request.goal.x_m, request.goal.y_m)
    navigator.restore(replace(checkpoint,
        failed_goals=((*cell, request.context.monotonic_ns + 1_000_000_000),)))
    estimate, world = _scene(1)
    mission = MissionManager(control.mission).evaluate(CommandRequest(
        estimate.context, 'bounded', CommandMode.EXPLORE, (), 1))
    held = navigator.evaluate(mission, estimate, world)
    assert held.status is NavigationStatus.PENDING
    assert not held.trajectory_candidates


def test_first_failed_rollout_point_is_compact_and_keeps_measurement_lineage():
    control = resolved_config().runtime.composition.live_control.control
    request = _request(tracks=(ObstacleTrack('overlap', 0., 0., .1, 0., 0., 1.),))
    result = TrajectoryRolloutComputer(control.navigation).compute(request)
    assert all(candidate.collision for candidate in result.trajectory_candidates)
    for candidate in result.trajectory_candidates:
        witness = candidate.rejection_witness
        assert witness.candidate_id == candidate.candidate_id
        assert witness.reason == 'FOOTPRINT_CLEARANCE'
        assert witness.time_offset_ns == 0
        assert witness.source_context == request.context
        assert witness.geometry_revision == request.world.local_costmap.revision
        assert witness.geometry_captured_ns == request.context.monotonic_ns
        assert witness.clearance_m <= witness.required_clearance_m


def test_spawned_search_matches_direct_kernel_and_preserves_source():
    from v3.adapters.l6_planner_process import ProcessTrajectoryRolloutBackend
    resolved = resolved_config()
    control, navigator, evaluate = _rig(closed=True)
    evaluate(0)
    request = navigator.pending_rollout_request
    expected = TrajectoryRolloutComputer(control.navigation).compute(request)
    backend = ProcessTrajectoryRolloutBackend(control.navigation, ready_timeout_s=5.,
                                             process_config=resolved.edges.planner_process)
    try:
        request_id = backend.submit(request)
        deadline = time.monotonic() + 5.
        completion = None
        while completion is None and time.monotonic() < deadline:
            completion = backend.take_completion(request_id)
            if completion is None:
                time.sleep(.002)
        assert completion is not None and completion.error is None
        assert completion.identity.source_context == request.context
        assert completion.result == expected
    finally:
        backend.close()
