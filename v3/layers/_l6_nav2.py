"""Bounded L6 admission of already completed Nav2 controller proposals."""
from __future__ import annotations

from dataclasses import dataclass, replace
import math

from v3.contracts import (CommandMode, GLOBAL_FRAME_ID, LOCAL_FRAME_ID,
    LocalizationRequirement, MissionIntent, MissionLifecycle, MotionValidity,
    NavigationPlan, NavigationStatus, QualityState, RobotEstimate, TickContext,
    VelocitySource, VelocityTarget, Waypoint, WorldSnapshot)
from v3.contracts.navigation import (Nav2NavigationInput, Nav2NavigationRequest,
    Nav2NavigationStatus, Nav2VelocityGuidance)


@dataclass(frozen=True, slots=True)
class Nav2GuidanceState:
    mission_id: str | None = None
    mission_started: TickContext | None = None
    request: Nav2NavigationRequest | None = None
    last_guidance: Nav2VelocityGuidance | None = None
    completed: bool = False
    initial_distance_m: float = 0.0
    bound_goal: Waypoint | None = None
    issued_requests: tuple[Nav2NavigationRequest, ...] = ()
    bound_odom_generation: int | None = None


def accept_nav2_guidance(mission: MissionIntent, estimate: RobotEstimate,
        world: WorldSnapshot, event: Nav2NavigationInput | None,
        state: Nav2GuidanceState, *, max_scan_age_ns: int,
        max_odom_age_ns: int) -> tuple[NavigationPlan, Nav2GuidanceState]:
    """No ROS, transport polling, local mapping, planning or wall clock."""
    now = mission.context.monotonic_ns

    def hold(reason: str, status=NavigationStatus.IDLE):
        return NavigationPlan(mission.context, mission.mission_id, (), None,
            mission.constraints, 0.0, 1.0 if state.completed else 0.0,
            status, reason)

    if state.mission_id != mission.mission_id:
        state = Nav2GuidanceState(mission.mission_id, mission.context)
    if (mission.lifecycle is not MissionLifecycle.ACTIVE
            or mission.mode not in (CommandMode.NAVIGATE, CommandMode.EXPLORE)):
        return hold(mission.stop_reason or "NAV2_INACTIVE"), replace(state, request=None)
    if mission.context != estimate.context or mission.context != world.context:
        return hold("NAV2_CONTEXT_MISMATCH"), replace(state, request=None)
    quality = estimate.localization_quality
    if (quality.local_translation is QualityState.LOST or quality.heading is not QualityState.GOOD
            or quality.global_position is not QualityState.GOOD
            or not quality.local_pose_continuous or quality.pose_discontinuity
            or estimate.map_to_odom is None or estimate.frame_id != LOCAL_FRAME_ID):
        return hold("NAV2_LOCALIZATION_HOLD"), replace(state, request=None)
    goal = state.bound_goal or mission.target_pose
    if mission.mode is CommandMode.NAVIGATE and mission.target_frame_id == LOCAL_FRAME_ID:
        assert goal is not None
        if state.bound_odom_generation is not None and state.bound_odom_generation != quality.generation:
            return hold("NAV2_LOCAL_GOAL_ODOM_RESET"), replace(state, request=None)
        if state.bound_goal is None:
            x, y, yaw = estimate.map_to_odom.apply(goal.x_m, goal.y_m, goal.yaw_rad)
            goal = Waypoint(x, y, yaw)
    request = Nav2NavigationRequest(mission.context, mission.mission_id,
        mission.mission_id, mission.mode, goal, mission.constraints.max_v_mps,
        mission.constraints.max_omega_rad_s, quality.generation, estimate.transform_revision,
        goal_tolerance_m=mission.constraints.goal_tolerance_m,
        yaw_tolerance_rad=mission.constraints.yaw_tolerance_rad)
    issued = tuple(item for item in state.issued_requests
        if item.context.monotonic_ns >= now - 400_000_000)
    state = replace(state, request=request, bound_goal=goal,
        issued_requests=(issued + (request,))[-24:],
        bound_odom_generation=(quality.generation if state.bound_odom_generation is None
            else state.bound_odom_generation))
    if state.completed:
        return hold("NAV2_SUCCEEDED", NavigationStatus.COMPLETE), state
    if event is None or event.error is not None or event.guidance is None:
        return hold("NAV2_UNAVAILABLE" if event is not None and event.error else "NAV2_PENDING"), state
    guidance = event.guidance
    source_request = next((item for item in state.issued_requests
        if item.context == guidance.source_context), None)
    if (event.context != mission.context or guidance.mission_id != mission.mission_id
            or guidance.goal_id != request.goal_id
            or guidance.odom_generation != quality.generation
            or source_request is None
            or guidance.transform_revision != source_request.transform_revision
            or state.mission_started is None
            or guidance.source_context.tick_id < state.mission_started.tick_id
            or guidance.source_context.monotonic_ns < state.mission_started.monotonic_ns):
        return hold("NAV2_IDENTITY_MISMATCH"), state
    previous = state.last_guidance
    if previous is not None:
        if (guidance.worker_generation < previous.worker_generation
                or (guidance.worker_generation == previous.worker_generation
                    and (guidance.command_sequence < previous.command_sequence
                         or guidance.produced_ns < previous.produced_ns
                         or (guidance.command_sequence == previous.command_sequence and guidance != previous)))):
            return hold("NAV2_SUPERSEDED"), state
    state = replace(state, last_guidance=guidance)
    if not guidance.produced_ns <= now < guidance.valid_until_ns:
        return hold("NAV2_STALE"), state
    if guidance.status is Nav2NavigationStatus.SUCCEEDED:
        if mission.mode is CommandMode.NAVIGATE:
            pose = estimate.global_pose
            if (goal is None or pose is None
                    or quality.global_fix_measurement_ns is None
                    or quality.global_fix_last_accepted_ns != quality.global_fix_measurement_ns
                    or quality.global_fix_measurement_ns < state.mission_started.monotonic_ns
                    or quality.global_fix_age_ns > max_scan_age_ns
                    or math.hypot(goal.x_m - pose.x_m, goal.y_m - pose.y_m) > mission.constraints.goal_tolerance_m
                    or (goal.yaw_rad is not None and abs(math.atan2(math.sin(goal.yaw_rad-pose.yaw_rad),
                        math.cos(goal.yaw_rad-pose.yaw_rad))) > mission.constraints.yaw_tolerance_rad)):
                return hold("NAV2_SUCCESS_UNCONFIRMED"), state
            state = replace(state, completed=True)
            return hold("NAV2_SUCCEEDED", NavigationStatus.COMPLETE), state
        return hold("NAV2_EXPLORE_NEXT_GOAL"), state
    if guidance.status is not Nav2NavigationStatus.ACTIVE:
        return hold("NAV2_" + guidance.status.value,
            NavigationStatus.NO_PATH if guidance.status in (Nav2NavigationStatus.NO_PATH, Nav2NavigationStatus.FAILED)
            else NavigationStatus.IDLE), state
    if (now - guidance.scan_measurement_ns > max_scan_age_ns
            or now - guidance.odom_measurement_ns > max_odom_age_ns):
        return hold("NAV2_SENSOR_STALE"), state
    if mission.mode is CommandMode.EXPLORE and (guidance.selected_goal is None or guidance.map_revision <= 0):
        return hold("NAV2_EXPLORE_GOAL_UNAVAILABLE"), state
    # The exact expiry and original producer context survive all downstream
    # layers; receiving the same command in a later tick never renews it.
    until = min(guidance.valid_until_ns - 1,
        guidance.scan_measurement_ns + max_scan_age_ns,
        guidance.odom_measurement_ns + max_odom_age_ns)
    validity = MotionValidity(guidance.source_context, until, LOCAL_FRAME_ID,
        f"NAV2:{mission.mode.value}:odom{quality.generation}:edge{guidance.worker_generation}",
        LocalizationRequirement(True, True, True))
    progress = 0.0
    if mission.mode is CommandMode.NAVIGATE and goal is not None:
        pose = estimate.global_pose
        if pose is not None:
            distance = math.hypot(goal.x_m - pose.x_m, goal.y_m - pose.y_m)
            initial = state.initial_distance_m or distance
            state = replace(state, initial_distance_m=initial)
            progress = 0.0 if initial <= 0 else min(1.0, max(0.0, 1.0 - distance / initial))
    return NavigationPlan(mission.context, mission.mission_id, (),
        VelocityTarget(guidance.v_mps, guidance.omega_rad_s), mission.constraints,
        mission.constraints.corridor_radius_m, progress, NavigationStatus.ACTIVE,
        "NAV2_ACTIVE", motion_validity=validity, velocity_source=VelocitySource.NAV2,
        nav2_goal=guidance.selected_goal or goal, nav2_map_revision=guidance.map_revision,
        nav2_child_goal_id=guidance.child_goal_id), state
