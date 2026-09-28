"""Local motion remains available without global XY corrections."""
from dataclasses import replace

from rig import resolved_config, healthy_localization
from v3.contracts import (
    CommandMode, CommandRequest, ConstraintCode, DataField, MotionIntent,
    LocalizationRequirement, QualityState,
    NavigationStatus, RobotEstimate, RollingLocalCostmap, TickContext, WorldSnapshot,
)
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator
from v3.layers.l7_motion_selection import MotionSelector
from v3.layers.l8_motion_realization import MotionRealizer
from v3.layers.l9_operational_constraints import OperationalConstraintLayer


def _scene(tick, position_variance=0.65, yaw_variance=0.01):
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
    covariance = tuple(position_variance if i in (0, 6) else yaw_variance if i == 12 else 0.0
                       for i in range(25))
    estimate = RobotEstimate(context, "odom", 0, 0, 0, 0, 0, covariance, localization_quality=healthy_localization(
        global_position=QualityState.LOST if position_variance > .25 else QualityState.GOOD,
        heading=QualityState.LOST if yaw_variance > .2 else QualityState.GOOD))
    world = WorldSnapshot(context, "odom", 1, (), 0,
                          RollingLocalCostmap("odom", 1, 0.1, 2.5, (), 1, 0))
    return estimate, world


def _chain():
    config = resolved_config().runtime.composition.live_control.control
    return (
        config, MissionManager(config.mission), TrajectoryNavigator(
            config.navigation, async_config=replace(config.async_l6, enabled=False, completion_inputs=False),
        ),
        MotionSelector(config.motion_selection), MotionRealizer(config.motion_realization),
        OperationalConstraintLayer(config.operational_constraints),
    )


def test_roomcruise_localization_motion_requires_independent_local_quality():
    config, manager, navigator, selector, realizer, limiter = _chain()
    # Global covariance alone never establishes local motion quality.
    for tick, variance in enumerate((0.2465, 0.2517, 0.5017, 0.6497, 0.9, 0.2386)):
        estimate, world = _scene(tick, variance)
        command = CommandRequest(estimate.context, "cruise", CommandMode.EXPLORE, (), tick)
        plan = navigator.evaluate(manager.evaluate(command), estimate, world)
        objective = selector.evaluate(plan)
        motion = realizer.evaluate(objective, estimate, world)
        allowed = limiter.evaluate(motion, estimate)
        assert plan.status is NavigationStatus.ACTIVE
        assert not plan.motion_validity.localization_requirement.global_position
        assert not motion.localization_requirement.global_position
        assert ConstraintCode.LOCALIZATION_DEGRADED not in allowed.active_constraints
        if tick:
            assert abs(allowed.allowed_v_mps) + abs(allowed.allowed_omega_rad_s) > 0

    # An absolute map goal still needs reliable XY, even though it also uses
    # a local rollout. The new mission cannot inherit Room Cruise's exemption.
    estimate, world = _scene(6)
    command = CommandRequest(estimate.context, "navigate", CommandMode.NAVIGATE,
                             (DataField("x_m", 1.0), DataField("y_m", 0.0)), 6)
    plan = navigator.evaluate(manager.evaluate(command), estimate, world)
    motion = realizer.evaluate(selector.evaluate(plan), estimate, world)
    assert plan.reason == "LOCALIZATION_REACQUIRE"
    assert motion.requested_v_mps == 0
    denied = replace(motion, localization_requirement=LocalizationRequirement())
    allowed = limiter.evaluate(denied, estimate)
    assert allowed.active_constraints == (ConstraintCode.LOCALIZATION_DEGRADED,)
    assert allowed.allowed_v_mps == allowed.allowed_omega_rad_s == 0.0


def test_roomcruise_localization_motion_keeps_freshness_heading_and_stop_gates():
    config, manager, navigator, selector, realizer, limiter = _chain()
    estimate, world = _scene(0)
    mission = manager.evaluate(CommandRequest(estimate.context, "cruise", CommandMode.EXPLORE, (), 0))
    plan = navigator.evaluate(mission, estimate, world)
    objective = selector.evaluate(plan)
    # Neither retained guidance nor local XY independence may authorize stale
    # world data, an expired objective, a frame change or uncertain heading.
    for broken in (
        replace(world, freshness_ns=config.motion_realization.max_world_freshness_ns + 1),
        replace(world, frame_id="other", local_costmap=None),
    ):
        motion = realizer.evaluate(objective, estimate, broken)
        assert motion.stop_reason is not None
        assert limiter.evaluate(motion, estimate).allowed_v_mps == 0.0
    late_context = TickContext(1, objective.validity.valid_until_ns + 1)
    expired = replace(objective, context=late_context, expiry_tick=2)
    assert realizer.evaluate(expired, replace(estimate, context=late_context),
                             replace(world, context=late_context)).stop_reason == "OBJECTIVE_EXPIRED"
    bad_heading, _ = _scene(0, yaw_variance=config.estimation.quality.max_yaw_variance + 0.01)
    motion = realizer.evaluate(objective, bad_heading, world)
    assert limiter.evaluate(motion, bad_heading).active_constraints == (ConstraintCode.LOCALIZATION_DEGRADED,)
    assert limiter.evaluate(motion, replace(estimate, context=late_context)).allowed_v_mps == 0.0
    # Missing lineage and callers constructing MotionIntent directly keep the
    # conservative default; no implicit local-motion exemption exists.
    no_lineage = realizer.evaluate(replace(objective, validity=None), estimate, world)
    assert no_lineage.localization_requirement.global_position
    direct = MotionIntent(estimate.context, 0.2, 0.0, 100_000_000, objective.constraints)
    assert limiter.evaluate(direct, estimate).active_constraints == (ConstraintCode.LOCALIZATION_DEGRADED,)
    for broken in (
        replace(world, local_costmap=None),
        replace(world, local_costmap=replace(world.local_costmap,
                freshness_ns=config.navigation.max_costmap_freshness_ns + 1)),
    ):
        invalid = navigator.evaluate(mission, estimate, broken)
        assert invalid.status is NavigationStatus.INVALIDATED
        stopped = realizer.evaluate(selector.evaluate(invalid), estimate, broken)
        assert stopped.stop_reason is not None
        assert limiter.evaluate(stopped, estimate).allowed_v_mps == 0.0
    stopped = manager.evaluate(CommandRequest(estimate.context, "stop", CommandMode.STOP, (), 0))
    motion = realizer.evaluate(selector.evaluate(navigator.evaluate(stopped, estimate, world)), estimate, world)
    allowed = limiter.evaluate(motion, estimate)
    assert allowed.allowed_v_mps == allowed.allowed_omega_rad_s == 0.0


def test_roomcruise_localization_pending_keeps_original_dependency_and_expiry():
    config, manager, navigator, selector, realizer, limiter = _chain()
    estimate, world = _scene(0)
    mission = manager.evaluate(CommandRequest(estimate.context, "cruise", CommandMode.EXPLORE, (), 0))
    plan = navigator.evaluate(mission, estimate, world)
    original = selector.evaluate(plan)
    checkpoint = selector.checkpoint()
    estimate, world = _scene(1)
    pending = replace(plan, context=estimate.context, status=NavigationStatus.PENDING,
                      local_goal=None, trajectory_candidates=(), reason="PLANNER_PENDING")
    retained = selector.evaluate(pending)
    assert retained.validity == original.validity
    assert not realizer.evaluate(retained, estimate, world).localization_requirement.global_position
    restored = MotionSelector(config.motion_selection)
    restored.restore(checkpoint)
    assert restored.evaluate(pending) == retained
    # A tightened dependency invalidates the old objective even when frame and
    # scope match. Pending cannot silently retain less restrictive authority.
    tightened = replace(pending, motion_validity=replace(pending.motion_validity,
                                                       localization_requirement=LocalizationRequirement()))
    restored.restore(checkpoint)
    stopped = realizer.evaluate(restored.evaluate(tightened), estimate, world)
    assert stopped.stop_reason == "PLANNER_PENDING_NO_VALID_OBJECTIVE"
    assert limiter.evaluate(stopped, estimate).allowed_v_mps == 0.0

    # Exercise production L6 completion inputs as well as L7 retention. A
    # timed-out replacement keeps the old trajectory only until its own expiry.
    from v3.contracts import LOCAL_FRAME_ID, ObstacleTrack
    from v3.contracts.planner import PlannerInput
    from v3.layers.l6_navigation import TrajectoryRolloutComputer
    for mode in (CommandMode.EXPLORE, CommandMode.FOLLOW_PERSON, CommandMode.NAVIGATE):
        async_config = replace(config.async_l6, request_timeout_ns=100_000_000)
        navigator = TrajectoryNavigator(config.navigation, async_config=async_config)
        selector = MotionSelector(config.motion_selection)
        computer = TrajectoryRolloutComputer(config.navigation)
        manager = MissionManager(config.mission)
        first_request = replacement = initial_validity = checkpoint = None
        restored = None
        for tick in range(21):
            estimate, world = _scene(tick)
            estimate = replace(estimate, frame_id=LOCAL_FRAME_ID)
            world = replace(world, frame_id=LOCAL_FRAME_ID,
                            local_costmap=replace(world.local_costmap, frame_id=LOCAL_FRAME_ID),
                            obstacle_tracks=(ObstacleTrack("person-1", 1.8, 0.0, .2, 0.0, 0.0, 1.0),))
            goal = (DataField("x_m", 1.0), DataField("y_m", 0.0), DataField("frame_id", LOCAL_FRAME_ID)) if mode is CommandMode.NAVIGATE else ()
            mission = manager.evaluate(CommandRequest(estimate.context, "continuity", mode, goal, tick))
            completion = None
            if tick == 1:
                completion = PlannerInput(estimate.context, first_request.context, computer.compute(first_request))
            elif tick == 11:
                assert replacement is not None
                completion = PlannerInput(estimate.context, replacement.context, error="ASYNC_L6_DEADLINE_MISSED")
            plan = navigator.evaluate(mission, estimate, world, completion)
            if restored is not None:
                assert restored.evaluate(mission, estimate, world, completion) == plan
            motion = realizer.evaluate(selector.evaluate(plan), estimate, world)
            if tick == 0:
                first_request = navigator.pending_rollout_request
                assert first_request is not None
            elif tick <= 17:
                assert plan.status is NavigationStatus.ACTIVE
                assert abs(motion.requested_v_mps) + abs(motion.requested_omega_rad_s) > 0
                if initial_validity is None:
                    initial_validity = plan.motion_validity
                assert plan.motion_validity == initial_validity
            else:
                assert motion.requested_v_mps == motion.requested_omega_rad_s == 0
            if tick == 5:
                replacement = navigator.pending_rollout_request
                assert replacement is not None
            if tick == 10:
                checkpoint = navigator.checkpoint()
                restored = TrajectoryNavigator(config.navigation, async_config=async_config)
                restored.restore(checkpoint)
