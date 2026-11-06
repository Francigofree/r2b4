"""Pure local-goal search and trajectory rollout for the L6 compute edge.

This module owns no mission, navigation or motion authority. Configuration and
immutable requests enter here; compact typed results return to L6 acceptance.
"""
from __future__ import annotations

import math
from dataclasses import replace
from typing import TYPE_CHECKING

from v3.contracts import (
    ExplorePreferences, NavigationRejectionWitness, RobotEstimate,
    RollingLocalCostmap, TrajectoryEvaluation, TrajectoryPose, Waypoint, WorldSnapshot,
)
from v3.contracts.planner import TrajectoryRolloutRequest, TrajectoryRolloutResult
from v3.layers._l6_geometry import (
    _LocalPlanningScene,
    _MOTION_EPSILON,
    _POINT_CLEARANCE_EPSILON_M,
    _bucket_key,
    _footprint_clearance,
    _footprint_radius,
    _integrate_constant_twist,
    _planning_scene_point_clearance,
    _wrapped_angle,
)

if TYPE_CHECKING:
    from v3.layers.l6_navigation import NavigationConfig


def _speed_clearance_ratio(v_mps: float, config: "NavigationConfig") -> float:
    """Normalize forward speed inside the configured soft-clearance envelope."""
    speed = abs(v_mps)
    low = config.speed_clearance_low_speed_mps
    high = config.speed_clearance_high_speed_mps
    if speed <= low:
        return 0.0
    if speed >= high:
        return 1.0
    return (speed - low) / (high - low)


def _speed_clearance_target_m(v_mps: float, config: "NavigationConfig") -> float:
    ratio = _speed_clearance_ratio(v_mps, config)
    return (
        config.speed_clearance_low_m
        + ratio * (config.speed_clearance_high_m - config.speed_clearance_low_m)
    )


def _trajectory_clearance_score(
    min_clearance_m: float,
    v_mps: float,
    config: "NavigationConfig",
) -> float:
    """Soft barrier: full credit once speed-appropriate clearance is reached.

    L12 remains the hard safety owner. This score only makes L6 prefer a slower
    or wider trajectory before the hard gate needs to intervene.
    """
    bounded = min(config.clearance_score_cap_m, max(0.0, min_clearance_m))
    legacy = bounded / config.clearance_score_cap_m
    if not config.speed_clearance_enabled or v_mps <= _MOTION_EPSILON:
        return legacy
    target = min(config.clearance_score_cap_m, _speed_clearance_target_m(v_mps, config))
    floor = max(config.footprint_safety_margin_m, config.local_escape_trigger_clearance_m)
    if target <= floor + _POINT_CLEARANCE_EPSILON_M:
        return legacy
    ratio = min(1.0, max(0.0, (bounded - floor) / (target - floor)))
    return ratio ** config.speed_clearance_barrier_power


def _trajectory_clearance_penalty(
    clearance_score: float,
    v_mps: float,
    config: "NavigationConfig",
) -> float:
    if not config.speed_clearance_enabled or v_mps <= _MOTION_EPSILON:
        return 0.0
    return _speed_clearance_ratio(v_mps, config) * (1.0 - clearance_score)


def _explore_goal_distances(config: "NavigationConfig", preferences: "ExplorePreferences | None" = None) -> tuple[float, ...]:
    """Far-to-near EXPLORE goal candidates; one sample preserves legacy behavior."""
    count = config.explore_local_goal_distance_samples
    policy = config if preferences is None else preferences
    maximum = policy.explore_local_goal_max_distance_m
    minimum = policy.explore_local_goal_min_distance_m
    if count == 1 or maximum - minimum <= _POINT_CLEARANCE_EPSILON_M:
        return (maximum,)
    step = (maximum - minimum) / (count - 1)
    return tuple(maximum - index * step for index in range(count))


def _rank_local_goal_options(
    config: NavigationConfig, estimate: RobotEstimate, costmap: RollingLocalCostmap,
    scene: _LocalPlanningScene, preferences: ExplorePreferences | None,
    coverage: dict[tuple[int, int], int], failed_goals: dict[tuple[int, int], int],
    goal_handoff_ns: int,
    reachability_request: TrajectoryRolloutRequest | None = None,
    reachability_cache: dict[Waypoint, bool] | None = None,
) -> tuple[Waypoint, ...]:
    # Ranking preferences arrive from the caller; geometry, sampling budget,
    # coverage, lifecycle and feasibility remain owned by generic L6.
    policy = config if preferences is None else preferences
    options: list[tuple[float, float, int, int, Waypoint, float]] = []
    uncertainty = config.localization_inflation_sigma * estimate.localization_quality.local_sigma_m
    tangent = estimate.yaw_rad + estimate.omega_rad_s*goal_handoff_ns/1e9
    footprint_radius = _footprint_radius(config)
    # With a full pivot/corridor proof below, this is only a necessary
    # endpoint prefilter. A circumscribed circle would discard corridors
    # that safely fit the rectangular footprint before that proof can run.
    endpoint_radius = (min(config.footprint_length_m / 2 - abs(config.footprint_center_x_m),
                           config.footprint_width_m / 2) + uncertainty
                       if reachability_request is not None else footprint_radius + uncertainty)
    relevant_clearance_m = max(
        config.clearance_score_cap_m,
        footprint_radius + config.footprint_safety_margin_m + uncertainty,
    ) + _POINT_CLEARANCE_EPSILON_M
    distances = _explore_goal_distances(config, preferences)
    distance_span = max(
        policy.explore_local_goal_max_distance_m
        - policy.explore_local_goal_min_distance_m,
        _POINT_CLEARANCE_EPSILON_M,
    )
    for index in range(config.local_goal_heading_samples):
        offset = _alternating_heading_offset(
            index,
            config.local_goal_heading_samples,
        )
        heading = _wrapped_angle(estimate.yaw_rad + offset)
        for distance_index, distance_m in enumerate(distances):
            goal = Waypoint(
                estimate.x_m + distance_m * math.cos(heading),
                estimate.y_m + distance_m * math.sin(heading),
            )
            if failed_goals.get(_bucket_key(goal.x_m, goal.y_m, config.coverage_cell_size_m), 0) > estimate.context.monotonic_ns:
                continue
            if (reachability_request is not None
                    and reachability_request.excluded_exit_mask & _exit_heading_bit(estimate, goal)):
                continue
            clearance = _planning_scene_point_clearance(
                goal.x_m,
                goal.y_m,
                scene,
                costmap.radius_m,
                relevant_clearance_m,
            )
            if clearance <= endpoint_radius + config.footprint_safety_margin_m:
                continue
            visits = coverage.get(_bucket_key(goal.x_m, goal.y_m, config.coverage_cell_size_m), 0)
            novelty = 1.0 / (1.0 + visits)
            clearance_score = min(
                1.0, clearance / config.clearance_score_cap_m
            )
            forward_preference = 0.5 * (math.cos(offset) + 1.0)
            distance_score = (
                1.0
                if len(distances) == 1
                else (distance_m - policy.explore_local_goal_min_distance_m)
                / distance_span
            )
            score = (
                policy.local_goal_novelty_weight * novelty
                + policy.local_goal_clearance_weight * clearance_score
                + policy.local_goal_forward_weight * forward_preference
                + policy.explore_local_goal_distance_weight * distance_score
            )
            # Distance already has caller-owned preference. Count the
            # additional turn here, avoiding a second short-goal bias.
            maneuver = abs(_wrapped_angle(heading-tangent))
            options.append((score, distance_m, index, distance_index, goal, maneuver))
    if not options:
        return ()
    best_score = max(item[0] for item in options)
    score_band = policy.local_goal_clearance_weight * min(1.,
        (costmap.resolution_m + uncertainty)/config.clearance_score_cap_m)
    ranked = sorted(options, key=lambda item: (
        item[0] < best_score-score_band-1e-12, item[5], -item[0], -item[1], item[2], item[3]))
    # Reachability belongs to the compute edge. Keep searching the ranked
    # family after a blocked corridor instead of letting four similar
    # endpoints discard every usable exit. Only the compact proven family
    # crosses back to L6.
    goals: list[Waypoint] = []
    for item in ranked:
        goal = item[4]
        if (reachability_request is not None
                and not _cached_goal_reachable(
                    config, scene, reachability_request, goal, reachability_cache)):
            continue
        goals.append(goal)
        if len(goals) == 4:
            break
    return tuple(goals)


def _local_goal_reachable(
    config: NavigationConfig, scene: _LocalPlanningScene, request: TrajectoryRolloutRequest,
    goal: Waypoint,
) -> bool:
    """Prove one pivot and the whole straight corridor, only on the planner edge.

    This sufficient local path need not find every possible route. The bounded
    yaw mesh adds its maximum between-sample corner displacement to the footprint;
    translation uses the exact swept rectangle. Endpoint clearance alone never
    admits a goal, and the starting safety margin is preserved.
    """
    estimate, world = request.estimate, request.world
    dx, dy = goal.x_m - estimate.x_m, goal.y_m - estimate.y_m
    distance = math.hypot(dx, dy)
    radius = _footprint_radius(config, scene.uncertainty_m)
    if (world.local_costmap is None
            or distance + radius + config.footprint_safety_margin_m >= world.local_costmap.radius_m):
        return False
    if distance <= config.local_goal_tolerance_m:
        return _footprint_clearance(estimate.x_m, estimate.y_m, estimate.yaw_rad,
                                    world, config, scene) > config.footprint_safety_margin_m
    bearing = math.atan2(dy, dx)
    turn = _wrapped_angle(bearing - estimate.yaw_rad)
    if abs(turn) > _MOTION_EPSILON:
        _v, omega = _constrain_rollout_motion(
            config, 0.0, math.copysign(request.max_omega_rad_s, turn),
            request.max_v_mps, request.max_omega_rad_s)
        if abs(omega) <= _MOTION_EPSILON:
            # A tiny heading drift need not require an unrealizable pivot.
            # Straight travel may reach the goal's acceptance region instead;
            # prove that exact swept corridor below, preserving its heading.
            cosine, sine = math.cos(estimate.yaw_rad), math.sin(estimate.yaw_rad)
            forward = dx * cosine + dy * sine
            lateral = -dx * sine + dy * cosine
            if forward <= 0.0 or abs(lateral) > config.local_goal_tolerance_m:
                return False
            distance = forward
            dx, dy = forward * cosine, forward * sine
            bearing, turn = estimate.yaw_rad, 0.0
    v, _omega = _constrain_rollout_motion(
        config, request.max_v_mps, 0.0, request.max_v_mps, request.max_omega_rad_s)
    if v <= _MOTION_EPSILON:
        return False
    pivot_scene = replace(scene, uncertainty_m=scene.uncertainty_m
                          + radius * abs(turn) / (2 * config.rollout_step_count))
    for step in range(config.rollout_step_count + 1):
        yaw = estimate.yaw_rad + turn * step / config.rollout_step_count
        if _footprint_clearance(estimate.x_m, estimate.y_m, yaw,
                                world, config, pivot_scene) <= config.footprint_safety_margin_m:
            return False
    corridor_config = replace(config, footprint_length_m=config.footprint_length_m + distance)
    return _footprint_clearance(estimate.x_m + dx / 2, estimate.y_m + dy / 2,
                                bearing, world, corridor_config, scene) > config.footprint_safety_margin_m


def _cached_goal_reachable(
    config: NavigationConfig, scene: _LocalPlanningScene, request: TrajectoryRolloutRequest,
    goal: Waypoint, cache: dict[Waypoint, bool] | None,
) -> bool:
    if cache is None:
        return _local_goal_reachable(config, scene, request, goal)
    if goal not in cache:
        cache[goal] = _local_goal_reachable(config, scene, request, goal)
    return cache[goal]


def _exit_heading_bit(estimate: RobotEstimate, goal: Waypoint) -> int:
    """Nearest absolute odometry heading, independent of robot yaw."""
    bearing = math.atan2(goal.y_m - estimate.y_m, goal.x_m - estimate.x_m) % (2 * math.pi)
    index = math.floor(bearing * 32 / (2 * math.pi) + .5) % 32
    return 1 << index


def _reachable_exit_mask(
    config: NavigationConfig, scene: _LocalPlanningScene, request: TrajectoryRolloutRequest,
    cache: dict[Waypoint, bool],
) -> int:
    """Summarize all proved exits in a fixed odometry-frame heading lattice.

    Failed-goal cooldowns and ranking cannot hide an already known opening.
    Revision changes never enter this geometry-only summary. Full corridor
    proofs stay on the worker; only these 32 bits return to L6 retry policy.
    """
    mask = 0
    distances = _explore_goal_distances(config, request.explore_preferences)
    for index in range(32):
        heading = _wrapped_angle(index * 2 * math.pi / 32)
        for distance in distances:
            goal = Waypoint(request.estimate.x_m + distance * math.cos(heading),
                            request.estimate.y_m + distance * math.sin(heading))
            if _cached_goal_reachable(config, scene, request, goal, cache):
                mask |= 1 << index
                break
    return mask


def _compute_goal_rollout(
    config: NavigationConfig, scene: _LocalPlanningScene, request: TrajectoryRolloutRequest,
) -> TrajectoryRolloutResult:
    scene = replace(scene, uncertainty_m=config.localization_inflation_sigma
                    * request.estimate.localization_quality.local_sigma_m)
    options = (request.goal, *request.alternate_goals)
    reachable_exit_mask = 0
    if request.select_local_goal:
        costmap = request.world.local_costmap
        reachability_cache: dict[Waypoint, bool] = {}
        if costmap is not None:
            reachable_exit_mask = _reachable_exit_mask(config, scene, request, reachability_cache)
        options = (() if costmap is None else _rank_local_goal_options(
            config, request.estimate, costmap, scene, request.explore_preferences,
            {(x, y): visits for x, y, visits in request.coverage},
            {(x, y): until for x, y, until in request.failed_goals}, request.goal_handoff_ns,
            request, reachability_cache))
    goal, reachable = request.goal, True
    if request.allow_exploration_detour:
        reachable = request.select_local_goal and bool(options)
        goal = options[0] if options else request.goal
        if not request.select_local_goal:
            for option in options:
                if _local_goal_reachable(config, scene, request, option):
                    goal, reachable = option, True
                    break
    coverage = {(x, y): visits for x, y, visits in request.coverage}
    if request.select_local_goal and not options:
        # A failed pivot/straight exit search does not prove that backing out
        # is impossible. The seed remains unreachable: only the existing
        # footprint-checked escape family may use it as a recovery reference.
        candidates = _compute_rollout_candidates(
            config, request.estimate, request.world, scene, goal,
            request.max_v_mps, request.max_omega_rad_s, coverage,
            goal_reachable=False,
        )
    else:
        candidates = _compute_rollout_candidates(
            config, request.estimate, request.world, scene, goal,
            request.max_v_mps, request.max_omega_rad_s, coverage,
            allow_exploration_detour=request.allow_exploration_detour and reachable,
            goal_reachable=reachable,
        )
    recovery = any(not c.collision and c.progress_viable
                   and c.candidate_id.startswith("escape-") for c in candidates)
    return TrajectoryRolloutResult(request.context, candidates,
        goal if request.allow_exploration_detour and (options or recovery) else None, reachable,
        options if request.select_local_goal else (), reachable_exit_mask)


def _constrain_rollout_motion(
    config: NavigationConfig, v_mps: float, omega_rad_s: float,
    max_v_mps: float, max_omega_rad_s: float, *, planning: bool = True,
) -> tuple[float, float]:
    return config.wheel_limits.constrain(
        v_mps, omega_rad_s, planning=planning,
        max_v_mps=min(max_v_mps, config.operational_max_v_mps),
        max_omega_rad_s=min(max_omega_rad_s, config.operational_max_omega_rad_s),
        max_curvature_rad_per_m=config.operational_max_curvature_rad_per_m,
    )


def _compute_rollout_candidates(
    config: NavigationConfig, estimate: RobotEstimate, world: WorldSnapshot,
    scene: _LocalPlanningScene, goal: Waypoint,
    max_v_mps: float, max_omega_rad_s: float, coverage: dict[tuple[int, int], int],
    *, allow_exploration_detour: bool = False, goal_reachable: bool = True,
) -> tuple[TrajectoryEvaluation, ...]:
    """Evaluate each exact motion once within one immutable request.

    Pose, geometry, uncertainty, coverage and goal remain fixed. Stable grid
    IDs and rejected forward evidence remain visible to L7. Velocities are
    never approximately grouped, and no evaluation is reused across requests.
    """
    start_clearance_m = _footprint_clearance(
        estimate.x_m, estimate.y_m, estimate.yaw_rad, world, config, scene)
    motion_cache: dict[tuple[float, float], TrajectoryEvaluation] = {}
    projection_cache: dict[tuple[float, float], tuple[float, float]] = {}

    def evaluate(linear_index: int, angular_index: int,
                 v_mps: float, omega_rad_s: float) -> TrajectoryEvaluation:
        requested_motion = (v_mps, omega_rad_s)
        motion = projection_cache.get(requested_motion)
        if motion is None:
            motion = _constrain_rollout_motion(
                config, v_mps, omega_rad_s, max_v_mps, max_omega_rad_s)
            projection_cache[requested_motion] = motion
        cached = motion_cache.get(motion)
        candidate_id = f"trajectory-{linear_index:02d}-{angular_index:02d}"
        if cached is None:
            cached = _evaluate_rollout_candidate(
                config, linear_index, angular_index, *motion, estimate, world,
                scene, goal, max_v_mps, max_omega_rad_s, start_clearance_m, coverage)
            motion_cache[motion] = cached
        return cached if cached.candidate_id == candidate_id else replace(
            cached, candidate_id=candidate_id,
            rejection_witness=(replace(cached.rejection_witness, candidate_id=candidate_id)
                               if cached.rejection_witness else None))

    evaluations = [
        evaluate(linear_index, angular_index, v_mps, omega_rad_s)
        for linear_index, angular_index, v_mps, omega_rad_s
        in config.wheel_limits.normal_samples(
            config.rollout_linear_samples, config.rollout_angular_samples,
            max_v_mps, max_omega_rad_s, config.minimum_planning_speed_mps)
    ]
    if allow_exploration_detour:
        evaluations = _exploration_detours(
            evaluations, estimate, world, scene, config, max_v_mps, start_clearance_m)
    if not goal_reachable:
        evaluations = [replace(c, progress_viable=False) for c in evaluations]
    if any(not candidate.collision and candidate.progress_viable
           for candidate in evaluations):
        return tuple(evaluations)

    # The rejected prior IDs remain explicit L7 invalidation evidence. Their
    # terminal samples suffice without transporting a second raw family.
    evaluations = [replace(c, samples=(c.samples[-1],)) for c in evaluations]
    reverse_limit_mps = min(config.local_escape_reverse_max_v_mps, max_v_mps)
    escape_cache: dict[tuple[float, float], TrajectoryEvaluation] = {}
    for linear_index in range(config.rollout_linear_samples):
        v_mps = -config.wheel_limits.linear_sample(
            linear_index, config.rollout_linear_samples, reverse_limit_mps,
            config.minimum_planning_speed_mps)
        for angular_index in range(config.rollout_angular_samples):
            angular_ratio = 2.0 * angular_index / (config.rollout_angular_samples - 1) - 1.0
            omega_rad_s = max_omega_rad_s * angular_ratio if linear_index == 0 else 0.0
            candidate = evaluate(linear_index, angular_index, v_mps, omega_rad_s)
            motion = (candidate.v_mps, candidate.omega_rad_s)
            escape = escape_cache.get(motion)
            candidate_id = f"escape-{linear_index:02d}-{angular_index:02d}"
            if escape is None:
                escape = _as_escape_candidate(
                    candidate, linear_index, angular_index, start_clearance_m,
                    goal, reverse_limit_mps, world, config, scene)
                escape_cache[motion] = escape
            evaluations.append(escape if escape.candidate_id == candidate_id else replace(
                escape, candidate_id=candidate_id,
                rejection_witness=(replace(escape.rejection_witness, candidate_id=candidate_id)
                                   if escape.rejection_witness else None)))
    return tuple(evaluations)


def _evaluate_rollout_candidate(
    config: NavigationConfig, linear_index: int, angular_index: int,
    v_mps: float, omega_rad_s: float, estimate: RobotEstimate,
    world: WorldSnapshot, scene: _LocalPlanningScene, goal: Waypoint,
    max_v_mps: float, max_omega_rad_s: float, start_clearance_m: float,
    coverage: dict[tuple[int, int], int],
) -> TrajectoryEvaluation:
    """Evaluate one already-realizable motion against one immutable scene."""
    step_ns = config.rollout_horizon_ns // config.rollout_step_count
    samples: list[TrajectoryPose] = []
    x_m, y_m, yaw_rad = estimate.x_m, estimate.y_m, estimate.yaw_rad
    min_clearance = start_clearance_m
    start_collision = min_clearance <= config.footprint_safety_margin_m
    collision = start_collision
    candidate_id = f"trajectory-{linear_index:02d}-{angular_index:02d}"
    witness = None

    def rejection(reason, offset, x, y, yaw, clearance, required):
        costmap = world.local_costmap
        return NavigationRejectionWitness(candidate_id, reason, estimate.context,
            costmap.revision if costmap is not None else world.map_revision,
            max(0, estimate.context.monotonic_ns - (costmap.freshness_ns if costmap else world.freshness_ns)),
            world.map_revision, offset, x, y, yaw, max(0.0, clearance), required, scene.uncertainty_m)

    if start_collision:
        witness = rejection("FOOTPRINT_CLEARANCE", 0, x_m, y_m, yaw_rad,
                            start_clearance_m, config.footprint_safety_margin_m)
    speed_witness = None
    for step in range(1, config.rollout_step_count + 1):
        offset_ns = (
            config.rollout_horizon_ns
            if step == config.rollout_step_count
            else step_ns * step
        )
        dt_s = (
            offset_ns - (samples[-1].time_offset_ns if samples else 0)
        ) / 1e9
        x_m, y_m, yaw_rad = _integrate_constant_twist(
            x_m,
            y_m,
            yaw_rad,
            v_mps,
            omega_rad_s,
            dt_s,
        )
        sample = TrajectoryPose(x_m, y_m, yaw_rad, offset_ns)
        samples.append(sample)
        if not start_collision:
            clearance = _footprint_clearance(
                x_m,
                y_m,
                yaw_rad,
                world,
                config,
                scene,
                clearance_limit_m=min_clearance,
            )
            min_clearance = min(min_clearance, clearance)
            if witness is None and clearance <= config.footprint_safety_margin_m:
                witness = rejection("FOOTPRINT_CLEARANCE", offset_ns, x_m, y_m, yaw_rad,
                                    clearance, config.footprint_safety_margin_m)
            if (speed_witness is None and v_mps > _MOTION_EPSILON
                    and clearance <= config.local_escape_trigger_clearance_m
                    and clearance + _POINT_CLEARANCE_EPSILON_M < start_clearance_m):
                speed_witness = rejection("FORWARD_CLEARANCE_DECREASING", offset_ns,
                    x_m, y_m, yaw_rad, clearance, config.local_escape_trigger_clearance_m)
            collision = (
                collision
                or clearance <= config.footprint_safety_margin_m
            )
    if (
        v_mps > _MOTION_EPSILON
        and min_clearance <= config.local_escape_trigger_clearance_m
        and min_clearance + _POINT_CLEARANCE_EPSILON_M < start_clearance_m
    ):
        collision = True
        witness = witness or speed_witness

    start_distance = math.hypot(goal.x_m - estimate.x_m, goal.y_m - estimate.y_m)
    approach_scoring = start_distance <= abs(v_mps) * config.rollout_horizon_ns / 1e9
    score_x, score_y, score_yaw = _goal_progress_pose(estimate, goal, samples, approach_scoring=approach_scoring)
    final_distance = math.hypot(goal.x_m - score_x, goal.y_m - score_y)
    progress = _clamp_signed(
        (start_distance - final_distance) / max(start_distance, 1e-9)
    )
    progress_potential = _trajectory_progress_potential(
        estimate, goal, score_x, score_y, score_yaw, progress,
        max_omega_rad_s * config.rollout_horizon_ns / 1e9,
        approach_scoring=approach_scoring,
    )
    progress_viable = (
        progress_potential + 1e-12 >= config.progress_viability_floor
    )
    linear_change = abs(v_mps - estimate.v_mps) / max(max_v_mps, 1e-9)
    angular_change = abs(omega_rad_s - estimate.omega_rad_s) / max(
        max_omega_rad_s,
        1e-9,
    )
    smoothness = max(
        0.0,
        1.0 - min(1.0, 0.5 * (linear_change + angular_change)),
    )
    novelty = sum(
        1.0
        / (
            1.0
            + coverage.get(_bucket_key(sample.x_m, sample.y_m, config.coverage_cell_size_m), 0)
        )
        for sample in samples
    ) / len(samples)
    bounded_clearance = min(
        config.clearance_score_cap_m,
        max(0.0, min_clearance),
    )
    clearance_score = _trajectory_clearance_score(
        bounded_clearance, v_mps, config
    )
    clearance_penalty = _trajectory_clearance_penalty(
        clearance_score, v_mps, config
    )
    total_score = (
        config.progress_weight * progress_potential
        + config.clearance_weight * clearance_score
        + config.smoothness_weight * smoothness
        + config.novelty_weight * novelty
        - config.speed_clearance_penalty_weight * clearance_penalty
        - config.localization_observability_weight * (1.0-estimate.localization_quality.observability)
        * abs(omega_rad_s) / max(max_omega_rad_s, 1e-9)
    )
    return TrajectoryEvaluation(
        candidate_id=candidate_id,
        v_mps=v_mps,
        omega_rad_s=omega_rad_s,
        horizon_ns=config.rollout_horizon_ns,
        samples=tuple(samples),
        collision=collision,
        min_clearance_m=bounded_clearance,
        progress_score=progress,
        progress_potential_score=progress_potential,
        progress_viable=progress_viable,
        smoothness_score=smoothness,
        novelty_score=novelty,
        total_score=total_score,
        rejection_witness=witness,
    )


def _goal_progress_pose(estimate: RobotEstimate, goal: Waypoint,
                        samples: list[TrajectoryPose], *, approach_scoring: bool) -> tuple[float, float, float]:
    """Score the approach before overshoot, retaining the full safety rollout.

    Receding-horizon execution can complete at the goal before the last predicted
    pose. At the realizable wheel-speed floor a one-second rollout often crosses
    a nearby goal; its reversed goal bearing must not veto the approach. Chord
    projections use the existing bounded samples, with interpolated heading.
    Equal-distance stationary pivots retain their final heading improvement.
    """
    final = samples[-1]
    best = (final.x_m, final.y_m, final.yaw_rad)
    if not approach_scoring:
        return best  # Preserve distant-goal scoring and its constant-time cost.
    best_distance = math.hypot(goal.x_m - final.x_m, goal.y_m - final.y_m)
    previous = (estimate.x_m, estimate.y_m, estimate.yaw_rad)
    for sample in samples:
        dx, dy = sample.x_m - previous[0], sample.y_m - previous[1]
        length_squared = dx * dx + dy * dy
        if length_squared > 1e-18:
            fraction = max(0.0, min(1.0, ((goal.x_m - previous[0]) * dx
                                        + (goal.y_m - previous[1]) * dy) / length_squared))
            x, y = previous[0] + fraction * dx, previous[1] + fraction * dy
            distance = math.hypot(goal.x_m - x, goal.y_m - y)
            if distance + 1e-12 < best_distance:
                yaw = previous[2] + fraction * _wrapped_angle(sample.yaw_rad - previous[2])
                best, best_distance = (x, y, yaw), distance
        previous = (sample.x_m, sample.y_m, sample.yaw_rad)
    return best


def _goal_heading_error(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    goal: Waypoint,
) -> float:
    """Absolute heading error toward the local goal from one predicted pose."""

    dx_m = goal.x_m - x_m
    dy_m = goal.y_m - y_m
    if math.hypot(dx_m, dy_m) <= 1e-9:
        return 0.0
    return abs(_wrapped_angle(math.atan2(dy_m, dx_m) - yaw_rad))


def _trajectory_progress_potential(
    estimate: RobotEstimate,
    goal: Waypoint,
    final_x_m: float,
    final_y_m: float,
    final_yaw_rad: float,
    distance_progress_score: float,
    angular_travel_rad: float,
    *, approach_scoring: bool = False,
) -> float:
    """Combine signed distance and heading improvement toward the local goal.

    Normalize heading by the initial error or one horizon's angular travel,
    whichever is larger. This keeps small errors bounded without diluting a
    speed-limited turn by a full half-circle. Heading deterioration offsets
    distance gain, weighted by the remaining range: at a closest approach the
    bearing is singular even for a tiny lateral miss. Stationary pivots retain
    the full heading criterion; only actual approach reduces that penalty.
    """

    start_heading_error = _goal_heading_error(
        estimate.x_m,
        estimate.y_m,
        estimate.yaw_rad,
        goal,
    )
    final_heading_error = _goal_heading_error(
        final_x_m,
        final_y_m,
        final_yaw_rad,
        goal,
    )
    heading_progress = _clamp_signed(
        (start_heading_error - final_heading_error)
        / max(start_heading_error, angular_travel_rad, 1e-9)
    )
    if approach_scoring and heading_progress < 0 and distance_progress_score > 0:
        heading_progress *= 1.0 - distance_progress_score
    return _clamp_signed(distance_progress_score + heading_progress)


def _exploration_detours(
    candidates: list[TrajectoryEvaluation], estimate: RobotEstimate,
    world: WorldSnapshot, scene: _LocalPlanningScene, config: NavigationConfig,
    max_v_mps: float, start_clearance_m: float,
) -> list[TrajectoryEvaluation]:
    """A local exploration waypoint cannot veto proven normal travel.

    Only after goal progress is exhausted, use safe forward displacement as
    exploration progress. A stationary pivot must open a footprint-checked
    forward next step; spinning alone is not exploration progress.
    """
    if any(not c.collision and c.progress_viable for c in candidates):
        return candidates
    horizon_s = config.rollout_horizon_ns / 1e9
    probe_speed = max(config.wheel_limits.target_floor_mps, config.minimum_planning_speed_mps)
    if probe_speed > max_v_mps:
        probe_speed = 0.0
    result = []
    for candidate in candidates:
        if candidate.collision:
            result.append(candidate)
            continue
        end = candidate.samples[-1]
        potential = 0.0
        if candidate.v_mps > _MOTION_EPSILON:
            potential = min(1.0, math.hypot(end.x_m - estimate.x_m, end.y_m - estimate.y_m)
                            / max(max_v_mps * horizon_s, 1e-9))
        elif abs(candidate.omega_rad_s) > _MOTION_EPSILON:
            reachable_goal = Waypoint(
                end.x_m + probe_speed * horizon_s * math.cos(end.yaw_rad),
                end.y_m + probe_speed * horizon_s * math.sin(end.yaw_rad),
            )
            potential = _escape_unlock_potential(
                end, reachable_goal, start_clearance_m, probe_speed,
                world, config, scene, forward_only=True,
            )
        if potential > _MOTION_EPSILON and potential + 1e-12 >= config.progress_viability_floor:
            candidate = replace(
                candidate, progress_potential_score=potential, progress_viable=True,
                total_score=candidate.total_score
                + config.progress_weight * (potential - candidate.progress_potential_score),
            )
        result.append(candidate)
    return result


def _escape_unlock_potential(
    pivot_end: TrajectoryPose,
    goal: Waypoint,
    start_clearance_m: float,
    probe_speed_mps: float,
    world: WorldSnapshot,
    config: NavigationConfig,
    scene: _LocalPlanningScene,
    *, forward_only: bool = False,
) -> float:
    """Prove that a safe pivot opens one bounded translational next step.

    This is deliberately a two-stage local proof, not a second planner.  It is
    evaluated only after the normal rollout family has no viable trajectory.
    The probe reuses the production footprint model, rollout horizon/step count
    and the already-bounded escape speed.  Forward probes also preserve the
    stronger local-escape trigger used by ordinary forward trajectories.
    """

    if probe_speed_mps <= _MOTION_EPSILON:
        return 0.0

    start_distance_m = math.hypot(goal.x_m - pivot_end.x_m, goal.y_m - pivot_end.y_m)
    step_ns = config.rollout_horizon_ns // config.rollout_step_count
    best_potential = 0.0

    for direction in ((1.0,) if forward_only else (1.0, -1.0)):
        probe_v_mps = direction * probe_speed_mps
        x_m, y_m, yaw_rad = pivot_end.x_m, pivot_end.y_m, pivot_end.yaw_rad
        min_clearance_m = _footprint_clearance(
            x_m, y_m, yaw_rad, world, config, scene,
        )
        blocked = min_clearance_m <= config.footprint_safety_margin_m
        previous_offset_ns = 0

        for step in range(1, config.rollout_step_count + 1):
            offset_ns = (
                config.rollout_horizon_ns
                if step == config.rollout_step_count
                else step_ns * step
            )
            dt_s = (offset_ns - previous_offset_ns) / 1e9
            previous_offset_ns = offset_ns
            x_m, y_m, yaw_rad = _integrate_constant_twist(
                x_m, y_m, yaw_rad, probe_v_mps, 0.0, dt_s,
            )
            clearance_m = _footprint_clearance(
                x_m,
                y_m,
                yaw_rad,
                world,
                config,
                scene,
                clearance_limit_m=min_clearance_m,
            )
            min_clearance_m = min(min_clearance_m, clearance_m)
            if clearance_m <= config.footprint_safety_margin_m:
                blocked = True
                break

        if (
            not blocked
            and probe_v_mps > _MOTION_EPSILON
            and min_clearance_m <= config.local_escape_trigger_clearance_m
            and min_clearance_m + _POINT_CLEARANCE_EPSILON_M < start_clearance_m
        ):
            blocked = True
        if blocked:
            continue

        final_clearance_m = _footprint_clearance(
            x_m, y_m, yaw_rad, world, config, scene,
        )
        clearance_progress = _clamp_signed(
            (final_clearance_m - start_clearance_m) / config.clearance_score_cap_m
        )
        final_distance_m = math.hypot(goal.x_m - x_m, goal.y_m - y_m)
        goal_progress = _clamp_signed(
            (start_distance_m - final_distance_m) / max(start_distance_m, 1e-9)
        )
        best_potential = max(
            best_potential,
            clearance_progress,
            goal_progress,
        )

    return _clamp_signed(best_potential)


def _as_escape_candidate(
    candidate: TrajectoryEvaluation,
    linear_index: int,
    angular_index: int,
    start_clearance_m: float,
    goal: Waypoint,
    probe_speed_mps: float,
    world: WorldSnapshot,
    config: NavigationConfig,
    scene: _LocalPlanningScene,
) -> TrajectoryEvaluation:
    """Score immediate recovery or a bounded pivot-then-translate escape proof.

    Minimum path clearance includes the starting footprint, so it cannot express
    improvement.  Reverse recovery keeps its existing immediate-clearance rule.
    A pure pivot gets no unconditional bonus: it becomes viable only when the
    pivot itself improves clearance or its final pose has a collision-free next
    translational step with enough goal/clearance gain.
    """

    moving = (
        abs(candidate.v_mps) > _MOTION_EPSILON
        or abs(candidate.omega_rad_s) > _MOTION_EPSILON
    )
    end = candidate.samples[-1]
    final_clearance_m = _footprint_clearance(
        end.x_m, end.y_m, end.yaw_rad, world, config, scene,
    )
    clearance_progress = _clamp_signed(
        (final_clearance_m - start_clearance_m) / config.clearance_score_cap_m
    )

    unlock_potential = 0.0
    if (
        not candidate.collision
        and abs(candidate.v_mps) <= _MOTION_EPSILON
        and abs(candidate.omega_rad_s) > _MOTION_EPSILON
        and clearance_progress + 1e-12 < config.progress_viability_floor
    ):
        unlock_potential = _escape_unlock_potential(
            end,
            goal,
            start_clearance_m,
            probe_speed_mps,
            world,
            config,
            scene,
        )

    escape_potential = max(clearance_progress, unlock_potential)
    viable = (
        moving
        and escape_potential > _MOTION_EPSILON
        and escape_potential + 1e-12 >= config.progress_viability_floor
    )
    escape_score = (
        config.progress_weight * escape_potential
        + config.clearance_weight * candidate.min_clearance_m / config.clearance_score_cap_m
        + config.smoothness_weight * candidate.smoothness_score
        + config.novelty_weight * candidate.novelty_score
    )
    return replace(
        candidate,
        candidate_id=f"escape-{linear_index:02d}-{angular_index:02d}",
        rejection_witness=(replace(candidate.rejection_witness,
            candidate_id=f"escape-{linear_index:02d}-{angular_index:02d}")
            if candidate.rejection_witness else None),
        total_score=escape_score,
        progress_potential_score=escape_potential,
        progress_viable=viable,
    )


def _alternating_heading_offset(index: int, count: int) -> float:
    if index == 0:
        return 0.0
    step = 2.0 * math.pi / count
    rank = (index + 1) // 2
    sign = 1.0 if index % 2 else -1.0
    return sign * rank * step


def _clamp_signed(value: float) -> float:
    return min(1.0, max(-1.0, value))

