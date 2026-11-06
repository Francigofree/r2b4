"""Measured EXPLORE stalls cannot stay ACTIVE through viable planner refreshes."""
from dataclasses import replace
import math
import pickle

import pytest

from rig import resolved_config
from test_v3_explore_recovery import _scene
from v3.contracts import CommandMode, CommandRequest, NavigationStatus, ObstacleTrack, Waypoint
from v3.contracts.planner import PlannerInput
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator, TrajectoryRolloutComputer
from v3.layers.l7_motion_selection import MotionSelector


def _rig(*, closed=False):
    control = resolved_config().runtime.composition.live_control.control
    async_config = (control.async_l6 if closed else
                    replace(control.async_l6, enabled=False, completion_inputs=False))
    navigator = TrajectoryNavigator(control.navigation, async_config=async_config)
    manager = MissionManager(control.mission)

    def evaluate(tick, *, x=0., revision=1, generation=0, command_id="bounded", tracks=()):
        estimate, world = _scene(tick, x=x, revision=revision, tracks=tracks)
        estimate = replace(estimate, localization_quality=replace(
            estimate.localization_quality, generation=generation))
        request = navigator.pending_rollout_request
        completion = (PlannerInput(estimate.context, request.context,
            TrajectoryRolloutComputer(control.navigation).compute(request))
            if closed and request is not None else None)
        mission = manager.evaluate(CommandRequest(estimate.context, command_id,
                                                  CommandMode.EXPLORE, (), tick))
        return navigator.evaluate(mission, estimate, world, completion)

    return control, navigator, evaluate


@pytest.mark.parametrize("closed", (False, True))
def test_viable_refreshed_paths_expire_at_measured_stall_boundary(closed):
    control, navigator, evaluate = _rig(closed=closed)
    evaluate(0)
    first = evaluate(1)
    assert first.status is NavigationStatus.ACTIVE
    assert any(not c.collision and c.progress_viable for c in first.trajectory_candidates)
    # Every request/completion is viable, but accepted local pose stays still.
    # Geometry revisions and goal age must not stand in for physical progress.
    expiry = math.ceil(control.navigation.local_goal_max_age_ns / 20_000_000)
    for tick in range(2, expiry, 5):
        plan = evaluate(tick, revision=tick)
        assert plan.status is NavigationStatus.ACTIVE
    blocked_goal = navigator.checkpoint().local_goal
    held = evaluate(expiry, revision=expiry)
    assert held.status is NavigationStatus.IDLE and held.reason == "NO_PROGRESS_HOLD"
    assert held.goal_selection_reason == "NO_PROGRESS"
    assert held.progress == first.progress > 0
    assert navigator.pending_rollout_request is None
    assert not held.trajectory_candidates
    assert MotionSelector(control.motion_selection).evaluate(held).trajectory is None
    cell = navigator._coverage_key(blocked_goal.x_m, blocked_goal.y_m)
    assert any((x, y) == cell for x, y, _until in navigator.checkpoint().failed_goals)


def test_goal_handoff_does_not_renew_stationary_progress_window():
    control, navigator, evaluate = _rig()
    evaluate(0)
    # A retained close endpoint legitimately triggers the ordinary handoff.
    # This representation/lifecycle event must not renew measured progress.
    checkpoint = navigator.checkpoint()
    navigator.restore(replace(checkpoint, local_goal=Waypoint(.01, 0.)))
    handoff = evaluate(5)
    assert handoff.goal_selection_reason == "GOAL_HANDOFF"
    assert navigator.checkpoint().explore_progress_started_ns == checkpoint.explore_progress_started_ns
    expiry = math.ceil(control.navigation.local_goal_max_age_ns / 20_000_000)
    held = evaluate(expiry)
    assert held.reason == "NO_PROGRESS_HOLD"


def test_actual_goal_approach_and_handoff_renew_progress():
    control, navigator, evaluate = _rig()
    first = evaluate(0)
    expiry = math.ceil(control.navigation.local_goal_max_age_ns / 20_000_000)
    x = first.local_goal.x_m - control.navigation.local_goal_tolerance_m / 2
    approached = evaluate(expiry - 1, x=x)
    assert approached.status is NavigationStatus.ACTIVE
    assert approached.goal_selection_reason == "GOAL_HANDOFF"
    assert navigator.checkpoint().explore_stall_count == 0
    after = evaluate(expiry + 1, x=x)
    assert after.status is NavigationStatus.ACTIVE
    assert after.reason != "NO_PROGRESS_HOLD"


def test_retries_exhaust_to_persistent_hold_and_checkpoint_preserves_budget():
    control, navigator, evaluate = _rig()
    evaluate(0)
    window = math.ceil(control.navigation.local_goal_max_age_ns / 20_000_000)
    cooldown = math.ceil(2 * control.navigation.trajectory_replan_interval_ns / 20_000_000)
    tick = 0
    for attempt in range(4):
        tick += window
        held = evaluate(tick, revision=tick)
        assert held.status is NavigationStatus.IDLE
        if attempt < 3:
            assert held.reason == "NO_PROGRESS_HOLD"
            assert evaluate(tick + cooldown - 1).status is NavigationStatus.IDLE
            tick += cooldown
            resumed = evaluate(tick, revision=tick)
            assert resumed.status is NavigationStatus.ACTIVE
            assert resumed.goal_selection_reason == "NO_PROGRESS_RESELECT"
        else:
            assert held.reason == "NO_PROGRESS_EXHAUSTED_HOLD"
    checkpoint = pickle.loads(pickle.dumps(navigator.checkpoint()))
    restored = TrajectoryNavigator(control.navigation,
        async_config=replace(control.async_l6, enabled=False, completion_inputs=False))
    restored.restore(checkpoint)
    for future in (tick + cooldown, tick + window, tick + 10*window):
        estimate, world = _scene(future, revision=future)
        mission = MissionManager(control.mission).evaluate(CommandRequest(
            estimate.context, "bounded", CommandMode.EXPLORE, (), future))
        # The restored mission has the same canonical identity as the original.
        assert mission.mission_id == checkpoint.mission_id
        result = restored.evaluate(mission, estimate, world)
        assert result == evaluate(future, revision=future)
        assert result.reason == "NO_PROGRESS_EXHAUSTED_HOLD"
        assert restored.pending_rollout_request is None
        assert not result.trajectory_candidates
    # Actual local translation can unlock a fresh geometry/planner attempt;
    # merely making the map's publication revision newer cannot.
    resumed = evaluate(tick + 10*window + 1, x=control.navigation.local_goal_tolerance_m * 1.1)
    assert resumed.status is NavigationStatus.ACTIVE
    assert navigator.checkpoint().explore_stall_count == 0


@pytest.mark.parametrize("closed", (False, True))
def test_new_proven_exit_funds_one_bounded_episode_and_restores(closed):
    control, navigator, evaluate = _rig(closed=closed)
    wall = (ObstacleTrack("front-wall", .5, 0., .05, 0., 0., 1.),)
    evaluate(0, tracks=wall)
    assert evaluate(1, tracks=wall).status is NavigationStatus.ACTIVE
    state = navigator.checkpoint()
    assert 0 < state.explore_reachable_exit_mask < (1 << 32) - 1
    known_exits = state.explore_reachable_exit_mask
    # Begin one tick before the last stationary attempt expires, retaining
    # the actual worker-proven exit baseline from this blocked scene.
    navigator.restore(replace(state, explore_stall_count=3))
    window = math.ceil(control.navigation.local_goal_max_age_ns / 20_000_000)
    exhausted = evaluate(window, tracks=wall, revision=window)
    assert exhausted.reason == "NO_PROGRESS_EXHAUSTED_HOLD"
    assert navigator.pending_rollout_request is None

    # A new publication of identical geometry cannot restart motion, even
    # when the slow probe has refreshed every normal rollout candidate.
    before = evaluate(2 * window - 1, tracks=wall, revision=2 * window)
    assert before.status is NavigationStatus.IDLE
    assert navigator.pending_rollout_request is None
    unchanged = evaluate(2 * window, tracks=wall, revision=2 * window + 1)
    if closed:
        assert navigator.pending_rollout_request is not None
        assert navigator.pending_rollout_request.excluded_exit_mask == known_exits
        assert unchanged.reason == "NO_PROGRESS_EXHAUSTED_HOLD"
        unchanged = evaluate(2 * window + 1, tracks=wall, revision=2 * window + 2)
    assert unchanged.reason == "NO_PROGRESS_EXHAUSTED_HOLD"
    assert not navigator.checkpoint().explore_exit_retry_used

    # Removing the actual front obstacle proves previously unavailable ODOM
    # headings. The accepted positive normal path buys exactly one episode.
    opened = evaluate(3 * window, revision=3 * window)
    if closed:
        assert opened.reason == "NO_PROGRESS_EXHAUSTED_HOLD"
        opened = evaluate(3 * window + 1, revision=3 * window + 1)
    assert opened.status is NavigationStatus.ACTIVE
    assert opened.goal_selection_reason == "NEW_EXIT_RETRY"
    bearing = math.atan2(opened.local_goal.y_m, opened.local_goal.x_m)
    chosen_bit = 1 << (round(bearing * 32 / (2 * math.pi)) % 32)
    assert not chosen_bit & known_exits
    assert any(c.v_mps > 0 and c.progress_viable and not c.collision
               and not c.candidate_id.startswith("escape-") for c in opened.trajectory_candidates)
    checkpoint = pickle.loads(pickle.dumps(navigator.checkpoint()))
    assert checkpoint.explore_stall_count == 4
    assert checkpoint.explore_exit_retry_used
    restored = TrajectoryNavigator(control.navigation,
        async_config=(control.async_l6 if closed else
                      replace(control.async_l6, enabled=False, completion_inputs=False)))
    restored.restore(checkpoint)
    estimate, world = _scene(4 * window, revision=4 * window)
    mission = MissionManager(control.mission).evaluate(CommandRequest(
        estimate.context, "bounded", CommandMode.EXPLORE, (), 4 * window))
    held = evaluate(4 * window, revision=4 * window)
    assert restored.evaluate(mission, estimate, world) == held
    assert held.reason == "NO_PROGRESS_EXHAUSTED_HOLD"
    assert navigator.checkpoint().explore_stall_count == 5
    assert evaluate(20 * window, revision=20 * window).status is NavigationStatus.IDLE
    assert navigator.pending_rollout_request is None
    assert navigator.checkpoint().explore_exit_retry_used

    evaluate(20 * window + 1, x=control.navigation.local_goal_tolerance_m * 1.1)
    assert navigator.checkpoint().explore_stall_count == 0
    assert not navigator.checkpoint().explore_exit_retry_used


def test_historical_exhausted_checkpoint_first_probe_only_establishes_baseline():
    control, navigator, evaluate = _rig()
    evaluate(0)
    checkpoint = navigator.checkpoint()
    navigator.restore(replace(checkpoint, explore_stall_count=4,
        explore_retry_after_ns=checkpoint.explore_progress_started_ns,
        explore_reachable_exit_mask=None,
        explore_probe_after_ns=checkpoint.explore_progress_started_ns))
    held = evaluate(1, revision=2)
    assert held.reason == "NO_PROGRESS_EXHAUSTED_HOLD"
    assert navigator.checkpoint().explore_reachable_exit_mask > 0
    assert not navigator.checkpoint().explore_exit_retry_used


@pytest.mark.parametrize("closed", (False, True))
def test_new_side_exit_can_start_with_normal_pivot_but_rotation_cannot_renew_budget(closed):
    control, navigator, evaluate = _rig(closed=closed)
    front = ObstacleTrack("front-wall", .5, 0., .05, 0., 0., 1.)
    side = ObstacleTrack("side-wall", 0., .55, .05, 0., 0., 1.)
    evaluate(0, tracks=(front, side))
    assert evaluate(1, tracks=(front, side)).status is NavigationStatus.ACTIVE
    state = navigator.checkpoint()
    navigator.restore(replace(state, explore_stall_count=3))
    window = math.ceil(control.navigation.local_goal_max_age_ns / 20_000_000)
    assert evaluate(window, tracks=(front, side)).reason == "NO_PROGRESS_EXHAUSTED_HOLD"

    # Removing the side obstacle proves a new corridor while the front wall
    # still rules out every forward rollout. Orienting toward that proved
    # corridor is normal navigation, within the single bonus episode.
    opened = evaluate(2 * window, tracks=(front,), revision=2)
    if closed:
        assert opened.reason == "NO_PROGRESS_EXHAUSTED_HOLD"
        opened = evaluate(2 * window + 1, tracks=(front,), revision=3)
    assert opened.status is NavigationStatus.ACTIVE
    assert opened.goal_selection_reason == "NEW_EXIT_RETRY"
    viable = tuple(c for c in opened.trajectory_candidates if c.progress_viable and not c.collision)
    assert viable and all(c.v_mps == 0. and c.omega_rad_s != 0.
                          and not c.candidate_id.startswith("escape-") for c in viable)
    started = navigator.checkpoint().explore_progress_started_ns

    estimate, world = _scene(2 * window + 2, tracks=(front,), revision=4)
    estimate = replace(estimate, yaw_rad=.2)
    mission = MissionManager(control.mission).evaluate(CommandRequest(
        estimate.context, "bounded", CommandMode.EXPLORE, (), estimate.context.tick_id))
    rotated = navigator.evaluate(mission, estimate, world)
    assert rotated.status is NavigationStatus.ACTIVE
    checkpoint = pickle.loads(pickle.dumps(navigator.checkpoint()))
    assert checkpoint.explore_progress_started_ns == started
    assert checkpoint.explore_stall_count == 4 and checkpoint.explore_exit_retry_used
    restored = TrajectoryNavigator(control.navigation,
        async_config=(control.async_l6 if closed else
                      replace(control.async_l6, enabled=False, completion_inputs=False)))
    restored.restore(checkpoint)
    estimate, world = _scene(3 * window, tracks=(front,), revision=5)
    estimate = replace(estimate, yaw_rad=.4)
    mission = MissionManager(control.mission).evaluate(CommandRequest(
        estimate.context, "bounded", CommandMode.EXPLORE, (), estimate.context.tick_id))
    held = navigator.evaluate(mission, estimate, world)
    assert restored.evaluate(mission, estimate, world) == held
    assert held.reason == "NO_PROGRESS_EXHAUSTED_HOLD"
    assert navigator.checkpoint().explore_stall_count == 5
    assert navigator.checkpoint().explore_exit_retry_used


def test_measured_translation_supersedes_pending_exhausted_probe():
    control, navigator, evaluate = _rig(closed=True)
    evaluate(0)
    evaluate(1)
    checkpoint = navigator.checkpoint()
    navigator.restore(replace(checkpoint, explore_stall_count=4,
        explore_retry_after_ns=checkpoint.explore_progress_started_ns,
        explore_probe_after_ns=checkpoint.explore_progress_started_ns))
    assert evaluate(2).reason == "NO_PROGRESS_EXHAUSTED_HOLD"
    previous = navigator.pending_rollout_request
    assert previous is not None
    assert navigator.checkpoint().pending_goal_selection_reason == "EXHAUSTED_EXIT_PROBE"

    estimate, world = _scene(3, x=control.navigation.local_goal_tolerance_m * 1.1)
    mission = MissionManager(control.mission).evaluate(CommandRequest(
        estimate.context, "bounded", CommandMode.EXPLORE, (), 3))
    navigator.evaluate(mission, estimate, world)
    state = navigator.checkpoint()
    assert state.explore_stall_count == 0 and not state.explore_exit_retry_used
    assert navigator.pending_rollout_request.context == estimate.context
    assert state.pending_goal_selection_reason != "EXHAUSTED_EXIT_PROBE"
    assert not state.trajectory_candidates


def test_unrelated_new_exit_mask_cannot_reopen_a_previously_known_goal():
    control, navigator, evaluate = _rig(closed=True)
    wall = (ObstacleTrack("front-wall", .5, 0., .05, 0., 0., 1.),)
    evaluate(0, tracks=wall)
    evaluate(1, tracks=wall)
    state = navigator.checkpoint()
    assert not state.explore_reachable_exit_mask & 1
    assert state.local_goal is not None
    navigator.restore(replace(state, explore_stall_count=4,
        explore_retry_after_ns=state.explore_progress_started_ns,
        explore_probe_after_ns=state.explore_progress_started_ns))
    evaluate(2, tracks=wall)
    request = navigator.pending_rollout_request
    result = TrajectoryRolloutComputer(control.navigation).compute(request)
    free_result = TrajectoryRolloutComputer(control.navigation).compute(replace(request,
        world=replace(request.world, obstacle_tracks=()), excluded_exit_mask=0))
    normal = tuple(c for c in free_result.trajectory_candidates
                   if c.v_mps > 0 and c.progress_viable and not c.collision)
    assert normal
    # The mask reports a new +X exit, but this completion proposes the same
    # old diagonal goal. That unrelated positive bit cannot fund motion.
    result = replace(result, selected_goal=state.local_goal, goal_options=(state.local_goal,),
        goal_reachable=True, trajectory_candidates=normal,
        reachable_exit_mask=state.explore_reachable_exit_mask | 1)
    estimate, world = _scene(3, tracks=wall)
    mission = MissionManager(control.mission).evaluate(CommandRequest(
        estimate.context, "bounded", CommandMode.EXPLORE, (), 3))
    held = navigator.evaluate(mission, estimate, world,
        PlannerInput(estimate.context, request.context, result))
    assert held.reason == "NO_PROGRESS_EXHAUSTED_HOLD"
    assert not navigator.checkpoint().explore_exit_retry_used
    assert not held.trajectory_candidates


def test_bounded_pivot_phase_finishes_its_own_budget_before_translation_stall():
    from test_v3_cached_geometry import _escape_scene, _seed_escape
    control, navigator, intent, _request, _result, _first = _seed_escape("pivot")
    checkpoint = navigator.checkpoint()
    tick = math.ceil(control.navigation.local_goal_max_age_ns / 20_000_000)
    estimate, world = _escape_scene(control.navigation, tick, kind="pivot", revision=tick)
    # A committed pivot can begin just as the measured translation window
    # expires. It retains its separate bounded odometry-based phase, rather
    # than getting cut off by the translation watcher halfway through a ramp.
    navigator.restore(replace(checkpoint,
        escape_started_ns=estimate.context.monotonic_ns,
        explore_progress_origin=(0., 0.), explore_progress_started_ns=1_000_000_000))

    def mission(context):
        return replace(intent(context), mode=CommandMode.EXPLORE, target_pose=None)

    pivot = navigator.evaluate(mission(estimate.context), estimate, world)
    assert pivot.status is NavigationStatus.ACTIVE
    assert pivot.reason == "LOCAL_GUIDANCE_REVALIDATED"
    assert pivot.trajectory_candidates[0].v_mps == 0.
    assert navigator.checkpoint().explore_progress_started_ns == 1_000_000_000
    expiry = tick + math.ceil(2 * checkpoint.escape_candidate.horizon_ns / 20_000_000)
    estimate, world = _escape_scene(control.navigation, expiry, kind="pivot", revision=expiry)
    expired = navigator.evaluate(mission(estimate.context), estimate, world)
    assert expired.reason == "RECOVERY_EXHAUSTED_HOLD"
    estimate, world = _escape_scene(control.navigation, expiry + 1, kind="pivot", revision=expiry + 1)
    after = navigator.evaluate(mission(estimate.context), estimate, world)
    assert after.reason == "NO_PROGRESS_HOLD"


def test_discontinuous_pose_jump_cannot_unlock_exhausted_progress():
    _control, navigator, evaluate = _rig()
    evaluate(0)
    checkpoint = navigator.checkpoint()
    navigator.restore(replace(checkpoint, explore_stall_count=4,
                             explore_retry_after_ns=checkpoint.explore_progress_started_ns))
    estimate, world = _scene(1, x=10.)
    estimate = replace(estimate, localization_quality=replace(
        estimate.localization_quality, local_pose_continuous=False, pose_discontinuity=True))
    control = resolved_config().runtime.composition.live_control.control
    mission = MissionManager(control.mission).evaluate(CommandRequest(
        estimate.context, "bounded", CommandMode.EXPLORE, (), 1))
    held = navigator.evaluate(mission, estimate, world)
    assert held.reason == "NO_PROGRESS_EXHAUSTED_HOLD"
    assert navigator.checkpoint().explore_stall_count == 4
    assert not held.trajectory_candidates


@pytest.mark.parametrize("change", ("world_stale", "costmap_stale", "frame", "generation"))
def test_progress_observation_never_bypasses_hard_lineage_or_geometry_gates(change):
    control, navigator, evaluate = _rig(closed=True)
    evaluate(0)
    assert evaluate(1).status is NavigationStatus.ACTIVE
    estimate, world = _scene(2, x=control.navigation.local_goal_tolerance_m * 2)
    if change == "world_stale":
        world = replace(world, freshness_ns=control.navigation.max_world_freshness_ns + 1)
    elif change == "costmap_stale":
        world = replace(world, local_costmap=replace(world.local_costmap,
            freshness_ns=control.navigation.max_costmap_freshness_ns + 1))
    elif change == "frame":
        world = replace(world, frame_id="other",
                        local_costmap=replace(world.local_costmap, frame_id="other"))
    else:
        estimate = replace(estimate, localization_quality=replace(
            estimate.localization_quality, generation=1))
    mission = MissionManager(control.mission).evaluate(CommandRequest(
        estimate.context, "bounded", CommandMode.EXPLORE, (), 2))
    plan = navigator.evaluate(mission, estimate, world)
    if change == "generation":
        # The previous generation is revoked. A freshly revalidated single
        # maneuver in the new local frame may resume immediately.
        assert not navigator.checkpoint().trajectory_candidates
        assert navigator.pending_rollout_request.context == estimate.context
        assert plan.motion_validity.source_context == estimate.context
        assert plan.motion_validity.scope.endswith(":odom1")
        return
    assert plan.status is not NavigationStatus.ACTIVE
    assert not plan.trajectory_candidates
    assert MotionSelector(control.motion_selection).evaluate(plan).trajectory is None
