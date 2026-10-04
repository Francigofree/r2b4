"""Local motion remains available without global XY corrections."""
from dataclasses import replace
import math

from rig import resolved_config, healthy_localization
from v3.contracts import (
    CommandMode, CommandRequest, ConstraintCode, DataField, MotionIntent,
    LocalizationRequirement, QualityState, RobotRelativeGeometry,
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
    world = WorldSnapshot(
        context, "odom", 1, (), 0,
        RollingLocalCostmap("odom", 1, 0.1, 2.5, (), 1, 0),
        robot_relative_geometry=RobotRelativeGeometry(
            "ROBOT_BASE", 1, context.monotonic_ns, 100, 0
        ),
    )
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
    _, explicit_manager, explicit_navigator, *_ = _chain()
    profile = resolved_config().roomcruise.preferences
    # Global covariance alone never establishes local motion quality.
    for tick, variance in enumerate((0.2465, 0.2517, 0.5017, 0.6497, 0.9, 0.2386)):
        estimate, world = _scene(tick, variance)
        command = CommandRequest(estimate.context, "cruise", CommandMode.EXPLORE, (), tick)
        plan = navigator.evaluate(manager.evaluate(command), estimate, world)
        explicit_plan = explicit_navigator.evaluate(
            explicit_manager.evaluate(replace(command, goal=profile.as_fields())), estimate, world,
        )
        assert explicit_plan == plan
        objective = selector.evaluate(plan)
        motion = realizer.evaluate(objective, estimate, world)
        allowed = limiter.evaluate(motion, estimate)
        assert plan.status is NavigationStatus.ACTIVE
        assert not plan.motion_validity.localization_requirement.global_position
        assert not motion.localization_requirement.global_position
        assert ConstraintCode.LOCALIZATION_DEGRADED not in allowed.active_constraints
        if tick:
            assert abs(allowed.allowed_v_mps) + abs(allowed.allowed_omega_rad_s) > 0

    # Preferences affect the chosen goal through the command, without replacing
    # the shared rollout scorer. The worker gets exactly that immutable goal.
    from v3.layers.l6_navigation import TrajectoryRolloutComputer
    custom = replace(profile, explore_local_goal_min_distance_m=.9, explore_local_goal_max_distance_m=.9)
    custom_estimate, custom_world = _scene(0)
    custom_command = CommandRequest(custom_estimate.context, 'custom', CommandMode.EXPLORE, custom.as_fields(), 0)
    custom_mission = manager.evaluate(custom_command)
    direct_navigator = TrajectoryNavigator(config.navigation, async_config=replace(config.async_l6, enabled=False, completion_inputs=False))
    direct = direct_navigator.evaluate(custom_mission, custom_estimate, custom_world)
    assert direct.local_goal.x_m == .9
    worker_navigator = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    worker_navigator.evaluate(custom_mission, custom_estimate, custom_world)
    request = worker_navigator.pending_rollout_request
    assert request.goal == direct.local_goal
    assert TrajectoryRolloutComputer(config.navigation).compute(request).trajectory_candidates == direct.trajectory_candidates

    # An absolute map goal still needs reliable XY. Recovery may only spin;
    # the new mission cannot inherit Room Cruise's translation exemption.
    estimate, world = _scene(6)
    command = CommandRequest(estimate.context, "navigate", CommandMode.NAVIGATE,
                             (DataField("x_m", 1.0), DataField("y_m", 0.0)), 6)
    plan = navigator.evaluate(manager.evaluate(command), estimate, world)
    motion = realizer.evaluate(selector.evaluate(plan), estimate, world)
    assert plan.reason == "LOCALIZATION_REACQUIRE"
    assert plan.local_goal is None
    assert plan.motion_validity.localization_requirement == LocalizationRequirement(False, True, False)
    assert motion.requested_v_mps == 0
    assert motion.requested_omega_rad_s == config.navigation.localization_recovery_omega_rad_s
    denied = replace(motion, localization_requirement=LocalizationRequirement())
    allowed = limiter.evaluate(denied, estimate)
    assert allowed.allowed_v_mps == allowed.allowed_omega_rad_s == 0.0



def test_localization_recovery_uses_robot_relative_geometry_not_pose_aligned_costmap():
    config, manager, navigator, selector, realizer, limiter = _chain()
    estimate, world = _scene(0)
    estimate = replace(
        estimate,
        localization_quality=replace(
            estimate.localization_quality,
            local_translation=QualityState.LOST,
            heading=QualityState.GOOD,
            local_sigma_m=0.19,
            lidar_age_ns=0,
        ),
    )
    mission = manager.evaluate(
        CommandRequest(estimate.context, "recovery-geometry", CommandMode.EXPLORE, (), 0)
    )

    # Recovery uses its configured realizable spin and fresh robot-relative geometry.
    # A frozen or absent pose-aligned costmap does not authorize translation.
    floor = config.navigation.wheel_limits.minimum_center_spin_rad_s
    recovery_omega = config.navigation.localization_recovery_omega_rad_s
    recovered = navigator.evaluate(mission, estimate, world)
    assert recovered.status is NavigationStatus.ACTIVE
    assert recovered.reason == 'LOCALIZATION_REACQUIRE'
    assert recovered.velocity_target.v_mps == 0
    assert recovered.velocity_target.omega_rad_s == recovery_omega
    assert recovered.constraints.max_omega_rad_s <= mission.constraints.max_omega_rad_s
    for broken_world in (
        replace(world, local_costmap=None),
        replace(
            world,
            local_costmap=replace(
                world.local_costmap,
                freshness_ns=config.navigation.max_costmap_freshness_ns + 1,
            ),
        ),
    ):
        plan = navigator.evaluate(mission, estimate, broken_world)
        assert plan.status is NavigationStatus.ACTIVE
        assert plan.reason == "LOCALIZATION_REACQUIRE"
        motion = realizer.evaluate(selector.evaluate(plan), estimate, broken_world)
        assert motion.requested_v_mps == 0
        assert motion.requested_omega_rad_s == recovery_omega

    # The unchanged L8-L10 chain ramps the spin within angular/wheel budgets.
    from v3.layers.l10_chassis_control import DifferentialDriveKinematics
    kinematics = DifferentialDriveKinematics(config.chassis_control)
    previous = None
    for tick in range(40):
        current, scene = _scene(tick)
        current = replace(current, localization_quality=estimate.localization_quality)
        request = CommandRequest(current.context, 'recovery-geometry', CommandMode.EXPLORE, (), tick)
        plan = navigator.evaluate(manager.evaluate(request), current, scene)
        motion = realizer.evaluate(selector.evaluate(plan), current, scene)
        allowed = limiter.evaluate(motion, current)
        wheels = kinematics(allowed)
        assert allowed.allowed_v_mps == 0
        assert 0 <= allowed.allowed_omega_rad_s <= min(recovery_omega, mission.constraints.max_omega_rad_s,
                                                     config.operational_constraints.max_omega_rad_s)
        if previous is not None:
            for side in ('left_mps', 'right_mps'):
                assert abs(getattr(wheels, side) - getattr(previous, side)) <= config.operational_constraints.max_acceleration_mps2 * .02 + 1e-12
        previous = wheels
    expected_left, expected_right = config.navigation.wheel_limits.wheels(0.0, recovery_omega)
    assert math.isclose(wheels.left_mps, expected_left)
    assert math.isclose(wheels.right_mps, expected_right)
    assert floor <= recovery_omega <= mission.constraints.max_omega_rad_s

    # A cap below the physical floor remains HOLD; neither L6 nor L9 amplifies.
    capped = replace(mission, constraints=replace(mission.constraints, max_omega_rad_s=floor * .95))
    held = navigator.evaluate(capped, estimate, world)
    assert held.reason == 'LOCALIZATION_HOLD'
    limited = OperationalConstraintLayer(replace(config.operational_constraints, max_omega_rad_s=floor * .95))
    motion = realizer.evaluate(selector.evaluate(recovered), estimate, world)
    denied = limited.evaluate(motion, estimate)
    assert denied.allowed_v_mps == denied.allowed_omega_rad_s == 0
    assert ConstraintCode.SPEED_LIMIT in denied.active_constraints
    next_estimate = replace(estimate, context=_scene(1)[0].context)
    denied = limited.evaluate(replace(motion, context=next_estimate.context), next_estimate)
    assert denied.allowed_v_mps == denied.allowed_omega_rad_s == 0

    # The old captured request still cannot turn this physical robot.
    from v3.layers.l6_navigation import TrajectoryNavigator
    legacy = TrajectoryNavigator(replace(config.navigation, localization_recovery_omega_rad_s=.2),
                                 async_config=config.async_l6)
    assert legacy.evaluate(mission, estimate, world).reason == 'LOCALIZATION_HOLD'

    deadline = math.ceil(config.navigation.localization_recovery_timeout_ns / 20_000_000)
    current, scene = _scene(deadline)
    current = replace(current, localization_quality=estimate.localization_quality)
    expired = navigator.evaluate(replace(mission, context=current.context), current, scene)
    assert expired.reason == 'LOCALIZATION_HOLD'

    for quality, scene in (
        (replace(estimate.localization_quality, heading=QualityState.LOST), world),
        (replace(estimate.localization_quality, heading=QualityState.DEGRADED), world),
        (replace(estimate.localization_quality, lidar_age_ns=config.navigation.max_costmap_freshness_ns + 1), world),
        (estimate.localization_quality, replace(world, freshness_ns=config.navigation.max_world_freshness_ns + 1)),
    ):
        _, _, case_navigator, _, _, _ = _chain()
        assert case_navigator.evaluate(mission, replace(estimate, localization_quality=quality), scene).reason == 'LOCALIZATION_HOLD'

    # P1: recovery now depends on fresh pose-independent ROBOT_BASE geometry.
    for geometry in (
        None,
        replace(
            world.robot_relative_geometry,
            freshness_ns=config.navigation.max_costmap_freshness_ns + 1,
        ),
        replace(world.robot_relative_geometry, point_count=0),
    ):
        _, case_manager, case_navigator, _, _, _ = _chain()
        case_mission = case_manager.evaluate(
            CommandRequest(estimate.context, "recovery-geometry", CommandMode.EXPLORE, (), 0)
        )
        held = case_navigator.evaluate(
            case_mission,
            estimate,
            replace(world, robot_relative_geometry=geometry),
        )
        assert held.status is NavigationStatus.IDLE
        assert held.reason == "LOCALIZATION_HOLD"


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
    # timed-out replacement can be bridged only by a new, measured local proof.
    from v3.contracts import LOCAL_FRAME_ID, ObstacleTrack, CostmapCell
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
        for tick in range(24):
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
            if tick == 22:
                world = replace(world, local_costmap=replace(world.local_costmap,
                    revision=2, occupied_cells=(CostmapCell(0, 0, 1),)))
            if tick == 23:
                world = replace(world, local_costmap=replace(world.local_costmap,
                    freshness_ns=config.navigation.max_costmap_freshness_ns+1))
            plan = navigator.evaluate(mission, estimate, world, completion)
            if restored is not None:
                assert restored.evaluate(mission, estimate, world, completion) == plan
            objective = selector.evaluate(plan)
            motion = realizer.evaluate(objective, estimate, world)
            if tick == 0:
                first_request = navigator.pending_rollout_request
                assert first_request is not None
            elif tick < 22:
                assert plan.status is NavigationStatus.ACTIVE
                assert abs(motion.requested_v_mps) + abs(motion.requested_omega_rad_s) > 0
                if initial_validity is None:
                    initial_validity = plan.motion_validity
                if plan.reason == "LOCAL_GUIDANCE_REVALIDATED":
                    assert plan.motion_validity.source_context == estimate.context
                    assert plan.motion_validity.geometry_revision == world.local_costmap.revision
                    assert plan.motion_validity.geometry_captured_ns == estimate.context.monotonic_ns
                    assert objective.transition_allowed
                else:
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


def test_roomcruise_soft_localization_changes_replan_without_zero_motion_gap():
    """Soft quality changes replan in background without revoking local motion."""
    from v3.contracts.planner import PlannerInput
    from v3.layers.l6_navigation import TrajectoryRolloutComputer

    config = resolved_config().runtime.composition.live_control.control
    navigator = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    selector = MotionSelector(config.motion_selection)
    realizer = MotionRealizer(config.motion_realization)
    limiter = OperationalConstraintLayer(config.operational_constraints)
    manager = MissionManager(config.mission)
    computer = TrajectoryRolloutComputer(config.navigation)

    first_request = None
    authority_scope = None
    for tick in range(8):
        estimate, world = _scene(tick)
        if tick < 5:
            sigma_m, local_state = 0.03984, QualityState.GOOD
        elif tick < 7:
            # Reproduces the live-capture 2 cm bucket crossing: 1.992 -> 2.0045.
            sigma_m, local_state = 0.04009, QualityState.GOOD
        else:
            sigma_m, local_state = 0.061, QualityState.DEGRADED
        estimate = replace(
            estimate,
            localization_quality=replace(
                estimate.localization_quality,
                local_sigma_m=sigma_m,
                local_translation=local_state,
                heading=QualityState.GOOD,
            ),
        )
        mission = manager.evaluate(
            CommandRequest(estimate.context, "soft-localization-continuity", CommandMode.EXPLORE, (), tick)
        )
        completion = None
        if tick == 1:
            assert first_request is not None
            completion = PlannerInput(
                estimate.context,
                first_request.context,
                computer.compute(first_request),
            )
        plan = navigator.evaluate(mission, estimate, world, completion)
        if tick == 0:
            assert plan.status is NavigationStatus.ACTIVE
            assert plan.reason == "LOCAL_GUIDANCE_REVALIDATED"
            first_request = navigator.pending_rollout_request
            assert first_request is not None
            continue

        objective = selector.evaluate(plan)
        motion = realizer.evaluate(objective, estimate, world)
        allowed = limiter.evaluate(motion, estimate)
        assert plan.status is NavigationStatus.ACTIVE
        assert motion.stop_reason is None
        assert abs(motion.requested_v_mps) + abs(motion.requested_omega_rad_s) > 0
        if tick >= 2:
            assert abs(allowed.allowed_v_mps) + abs(allowed.allowed_omega_rad_s) > 0
        if authority_scope is None:
            assert plan.motion_validity is not None
            authority_scope = plan.motion_validity.scope
        assert plan.motion_validity is not None
        assert plan.motion_validity.scope == authority_scope
        if tick == 5:
            # A replacement rollout is scheduled at the normal 100 ms cadence,
            # while the accepted trajectory remains ACTIVE.
            assert navigator.pending_rollout_request is not None

    # A true local-translation LOST state still revokes translation and enters
    # bounded localization recovery with independent heading authority.
    estimate, world = _scene(8)
    estimate = replace(
        estimate,
        localization_quality=replace(
            estimate.localization_quality,
            local_sigma_m=0.19,
            local_translation=QualityState.LOST,
            heading=QualityState.GOOD,
        ),
    )
    mission = manager.evaluate(
        CommandRequest(estimate.context, "soft-localization-continuity", CommandMode.EXPLORE, (), 8)
    )
    plan = navigator.evaluate(mission, estimate, world)
    assert plan.reason == "LOCALIZATION_REACQUIRE"
    assert plan.local_goal is None
    assert plan.motion_validity.localization_requirement == LocalizationRequirement(False, True, False)
    motion = realizer.evaluate(selector.evaluate(plan), estimate, world)
    assert motion.requested_v_mps == 0.0
    assert motion.requested_omega_rad_s == config.navigation.localization_recovery_omega_rad_s



def test_global_navigation_motion_continues_only_inside_committed_local_generation():
    from v3.contracts import GLOBAL_FRAME_ID, LOCAL_FRAME_ID, Pose2D
    config, manager, navigator, selector, realizer, limiter = _chain()
    committed_goal = None
    for tick in range(3):
        estimate, world = _scene(tick, position_variance=.01)
        world = replace(world, frame_id=LOCAL_FRAME_ID,
                        local_costmap=replace(world.local_costmap, frame_id=LOCAL_FRAME_ID))
        estimate = replace(estimate, frame_id=LOCAL_FRAME_ID,
            local_pose=Pose2D(LOCAL_FRAME_ID, 0., 0., 0.),
            global_pose=Pose2D(GLOBAL_FRAME_ID, 0., 0., 0.),
            map_to_odom=Pose2D(GLOBAL_FRAME_ID, 0., 0., 0.),
            localization_quality=replace(estimate.localization_quality,
                global_position=QualityState.GOOD if tick == 0 else QualityState.LOST,
                generation=1 if tick == 2 else 0))
        mission = manager.evaluate(CommandRequest(estimate.context, "committed", CommandMode.NAVIGATE,
            (DataField("x_m", 3.0), DataField("y_m", 0.0)), tick))
        plan = navigator.evaluate(mission, estimate, world)
        if tick == 0:
            assert plan.status is NavigationStatus.ACTIVE
            committed_goal = plan.local_goal
        elif tick == 1:
            assert plan.reason == "LOCAL_GUIDANCE_REVALIDATED"
            assert plan.local_goal == committed_goal
            assert not plan.motion_validity.localization_requirement.global_position
            motion = realizer.evaluate(selector.evaluate(plan), estimate, world)
            assert motion.requested_v_mps >= .15
        else:
            # A generation change revokes committed guidance. Only bounded
            # heading recovery is available until global XY is trustworthy.
            assert plan.reason == "LOCALIZATION_REACQUIRE"
            assert plan.local_goal is None
            assert plan.motion_validity.localization_requirement == LocalizationRequirement(False, True, False)
            motion = realizer.evaluate(selector.evaluate(plan), estimate, world)
            assert motion.requested_v_mps == 0.0
            assert motion.requested_omega_rad_s == config.navigation.localization_recovery_omega_rad_s
