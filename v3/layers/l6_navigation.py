# R2B4_FOLLOW_PERSON_P0_V2_20260923
"""L6 generic deterministic local navigation and progress ownership."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import Enum
from typing import Protocol

from v3.contracts.planner import PlannerInput, TrajectoryRolloutRequest, TrajectoryRolloutResult, GlobalPlanStatus
from v3.contracts.temporal import source_is_stale
from v3.wheel_motion import WheelMotionLimits

from v3.contracts import (
    CommandMode,
    DeviceHealthState,
    ExplorePreferences,
    MissionIntent,
    MissionLifecycle,
    MotionValidity,
    LocalMotionProof,
    NavigationPlan,
    NavigationRejectionWitness,
    NavigationStatus,
    RobotEstimate,
    LocalizationRequirement, QualityState, LOCAL_FRAME_ID, VelocityTarget,
    RollingLocalCostmap,
    TickContext,
    TrackEstimateStatus,
    TrajectoryEvaluation,
    TrajectoryPose,
    Waypoint,
    WorldSnapshot,
)


from v3.layers._l6_geometry import (
    _POINT_CLEARANCE_EPSILON_M,
    _MOTION_EPSILON,
    _ObstacleDisc,
    _StaticPlanningIndex,
    _LocalPlanningScene,
    _wrapped_angle,
    _integrate_constant_twist,
    _costmap_cache_key,
    _bucket_key,
    _build_static_planning_index,
    _scene_from_world,
    _static_obstacles_in_bounds,
    _planning_scene_point_clearance,
    _costmap_point_clearance,
    _rectangle_disc_clearance,
    _footprint_radius,
    _motion_sweep_hull,
    _motion_sweep_point_distance,
    _normal_motion_bounds,
    _escape_motion_bounds,
    _footprint_clearance,
)
from v3.layers._l6_rollout import (
    _speed_clearance_ratio,
    _speed_clearance_target_m,
    _trajectory_clearance_score,
    _trajectory_clearance_penalty,
    _explore_goal_distances,
    _rank_local_goal_options,
    _exit_heading_bit,
    _local_goal_reachable,
    _compute_goal_rollout,
    _constrain_rollout_motion,
    _compute_rollout_candidates,
    _evaluate_rollout_candidate,
    _goal_progress_pose,
    _goal_heading_error,
    _trajectory_progress_potential,
    _exploration_detours,
    _escape_unlock_potential,
    _as_escape_candidate,
    _alternating_heading_offset,
    _clamp_signed,
)


_FAILED_LOCAL_GOAL_CAPACITY = 4


class _RolloutDisposition(str, Enum):
    NONE = "NONE"
    ACCEPTED = "ACCEPTED"
    HOLD = "HOLD"


class _FollowPersonState(str, Enum):
    """L6-owned target lifecycle; no new numbered architectural layer."""

    ACQUIRE = "ACQUIRE"
    FOLLOW = "FOLLOW"
    OCCLUDED_HOLD = "OCCLUDED_HOLD"
    SEARCH = "SEARCH"
    LOST = "LOST"

@dataclass(frozen=True, slots=True)
class FollowPersonEvidence:
    """Passive L6 follow state for capture/Test Hub; never feeds control."""

    state: str
    locked_target_uid: str | None
    target_visible: bool
    target_confidence: float | None
    target_distance_m: float | None
    target_bearing_rad: float | None
    target_missing_age_ms: float | None
    person_candidate_count: int
    search_phase: int | None
    search_target_yaw_rad: float | None
    acquisition_min_confidence: float
    retention_min_confidence: float
    stand_off_m: float
    distance_deadband_m: float
    min_safe_distance_m: float
    lost_hold_ns: int
    search_timeout_ns: int
    search_sweep_rad: float
    search_yaw_tolerance_rad: float
    search_budget_ns: int | None = None
    target_estimate_status: str | None = None
    target_measurement_age_ms: float | None = None


@dataclass(frozen=True, slots=True)
class NavigationConfig:
    localization_recovery_timeout_ns: int
    # Live config derives the smallest centered spin from calibrated wheel
    # limits. Keep captured numeric values for exact historical replay.
    localization_recovery_omega_rad_s: float
    localization_degraded_speed_scale: float
    localization_inflation_sigma: float
    localization_observability_weight: float
    max_world_freshness_ns: int
    obstacle_confidence_floor: float
    max_costmap_freshness_ns: int
    trajectory_replan_interval_ns: int
    # Preserve the nominal 10 Hz replan cadence at a 50 Hz control rate,
    # while guaranteeing cheap control ticks after an over-budget replan.
    trajectory_replan_min_tick_gap: int
    coverage_cell_size_m: float
    coverage_max_cells: int
    local_goal_distance_m: float
    local_goal_tolerance_m: float
    local_goal_max_age_ns: int
    local_goal_heading_samples: int
    rollout_linear_samples: int
    rollout_angular_samples: int
    rollout_horizon_ns: int
    rollout_step_count: int
    footprint_length_m: float
    footprint_width_m: float
    footprint_safety_margin_m: float
    clearance_score_cap_m: float
    progress_weight: float
    clearance_weight: float
    smoothness_weight: float
    novelty_weight: float
    # Minimum predicted normalized improvement required for a normal trajectory.
    # Progress may be translational or angular alignment toward the local goal.
    progress_viability_floor: float
    face_person_min_confidence: float
    face_person_align_tolerance_rad: float
    face_person_release_tolerance_rad: float
    follow_person_min_confidence: float
    follow_person_align_tolerance_rad: float
    follow_person_release_tolerance_rad: float
    follow_person_stand_off_m: float
    follow_person_distance_deadband_m: float
    follow_person_min_safe_distance_m: float
    follow_person_lost_hold_ns: int
    # Recovery is bounded, rotation-only and deterministic. The first search
    # step looks at the last known heading before sweeping either side.
    follow_person_search_timeout_ns: int
    follow_person_search_sweep_rad: float
    # Compatibility field only; search advancement is now yaw-completion-driven.
    follow_person_search_step_ns: int
    follow_person_search_yaw_tolerance_rad: float
    follow_person_pivot_enter_rad: float
    follow_person_hold_release_margin_m: float
    follow_person_slowdown_distance_m: float
    follow_person_minimum_follow_speed_mps: float
    follow_person_heading_min_factor: float
    # None preserves the acquisition threshold for configs without retention tuning.
    follow_person_retention_min_confidence: float | None
    # Search allows ideal travel time plus search_timeout_ns settling/stall slack,
    # under this hard cap even for very small operator angular-speed limits.
    follow_person_search_max_duration_ns: int

    planning_bucket_size_m: float
    local_escape_trigger_clearance_m: float
    local_escape_reverse_max_v_mps: float
    local_goal_novelty_weight: float
    local_goal_clearance_weight: float
    local_goal_forward_weight: float
    wheel_limits: WheelMotionLimits = WheelMotionLimits()
    minimum_planning_speed_mps: float = 0.15
    # Derived from the existing L9 policy / control-gap owner by the resolver.
    # Infinity defaults decode historical configs predating this forward edge.
    operational_max_v_mps: float = math.inf
    operational_max_omega_rad_s: float = math.inf
    operational_max_curvature_rad_per_m: float = math.inf
    motion_proof_horizon_ns: int = 250_000_000
    # Soft navigation envelope. L12 remains the independent hard safety gate.
    speed_clearance_enabled: bool = False
    speed_clearance_low_speed_mps: float = 0.20
    speed_clearance_high_speed_mps: float = 0.40
    speed_clearance_low_m: float = 0.18
    speed_clearance_high_m: float = 0.30
    speed_clearance_barrier_power: float = 2.0
    speed_clearance_penalty_weight: float = 0.0
    # EXPLORE-only adaptive waypoint range; NAVIGATE/FOLLOW keep local_goal_distance_m.
    explore_local_goal_min_distance_m: float = 0.60
    explore_local_goal_max_distance_m: float = 0.60
    explore_local_goal_distance_samples: int = 1
    explore_local_goal_distance_weight: float = 0.0
    # Axle-frame rectangle center; zero decodes historical centered captures.
    footprint_center_x_m: float = 0.0

    def __post_init__(self) -> None:
        if (type(self.motion_proof_horizon_ns) is not int or self.motion_proof_horizon_ns <= 0
                or any(math.isnan(cap) or cap <= 0 for cap in (
                    self.operational_max_v_mps, self.operational_max_omega_rad_s,
                    self.operational_max_curvature_rad_per_m))):
            raise ValueError("invalid derived navigation motion envelope")
        if self.localization_recovery_timeout_ns <= 0 or not 0 < self.localization_degraded_speed_scale <= 1:
            raise ValueError("invalid localization recovery limits")
        omega = self.localization_recovery_omega_rad_s
        if (isinstance(omega, bool) or not isinstance(omega, (int, float))
                or not math.isfinite(omega) or omega <= 0):
            raise ValueError("localization recovery omega must be finite and positive")
        if not 0 < self.localization_inflation_sigma <= 5:
            raise ValueError("invalid localization preservation limits")
        for name in (
            "max_world_freshness_ns",
            "max_costmap_freshness_ns",
            "local_goal_max_age_ns",
        ):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        if (
            not isinstance(self.obstacle_confidence_floor, (int, float))
            or isinstance(self.obstacle_confidence_floor, bool)
            or not math.isfinite(self.obstacle_confidence_floor)
            or not 0.0 <= self.obstacle_confidence_floor <= 1.0
        ):
            raise ValueError("obstacle_confidence_floor must be in [0, 1]")
        for name in (
            "coverage_cell_size_m",
            "local_goal_distance_m",
            "local_goal_tolerance_m",
            "footprint_length_m",
            "footprint_width_m",
            "clearance_score_cap_m",
            "minimum_planning_speed_mps",
        ):
            value = getattr(self, name)
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value <= 0.0
            ):
                raise ValueError(f"{name} must be positive")
        if (isinstance(self.footprint_center_x_m, bool)
                or not isinstance(self.footprint_center_x_m, (int, float))
                or not math.isfinite(self.footprint_center_x_m)
                or abs(self.footprint_center_x_m) > self.footprint_length_m / 2):
            raise ValueError("footprint center must keep the drive axle inside the body")
        if self.minimum_planning_speed_mps > self.wheel_limits.maximum_mps:
            raise ValueError("planner minimum exceeds calibrated maximum")
        if type(self.speed_clearance_enabled) is not bool:
            raise ValueError("speed_clearance_enabled must be boolean")
        for name in (
            "speed_clearance_low_speed_mps",
            "speed_clearance_high_speed_mps",
            "speed_clearance_low_m",
            "speed_clearance_high_m",
            "speed_clearance_barrier_power",
            "explore_local_goal_min_distance_m",
            "explore_local_goal_max_distance_m",
        ):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0.0
            ):
                raise ValueError(f"{name} must be finite and positive")
        if self.speed_clearance_high_speed_mps <= self.speed_clearance_low_speed_mps:
            raise ValueError("speed-clearance high speed must exceed low speed")
        if self.speed_clearance_high_m < self.speed_clearance_low_m:
            raise ValueError("speed-clearance high-speed clearance must not shrink")
        if self.speed_clearance_enabled and self.speed_clearance_low_m <= self.local_escape_trigger_clearance_m:
            raise ValueError("soft speed-clearance envelope must stay above the forward escape trigger")
        if self.speed_clearance_barrier_power < 1.0:
            raise ValueError("speed-clearance barrier power must be >= 1")
        if (
            isinstance(self.speed_clearance_penalty_weight, bool)
            or not isinstance(self.speed_clearance_penalty_weight, (int, float))
            or not math.isfinite(self.speed_clearance_penalty_weight)
            or self.speed_clearance_penalty_weight < 0.0
        ):
            raise ValueError("speed_clearance_penalty_weight must be finite and non-negative")
        if self.explore_local_goal_min_distance_m > self.explore_local_goal_max_distance_m:
            raise ValueError("EXPLORE local-goal minimum distance exceeds maximum")
        if (
            not isinstance(self.explore_local_goal_distance_samples, int)
            or isinstance(self.explore_local_goal_distance_samples, bool)
            or self.explore_local_goal_distance_samples <= 0
        ):
            raise ValueError("explore_local_goal_distance_samples must be a positive integer")
        if (
            isinstance(self.explore_local_goal_distance_weight, bool)
            or not isinstance(self.explore_local_goal_distance_weight, (int, float))
            or not math.isfinite(self.explore_local_goal_distance_weight)
            or self.explore_local_goal_distance_weight < 0.0
        ):
            raise ValueError("explore_local_goal_distance_weight must be finite and non-negative")
        if (
            not isinstance(self.footprint_safety_margin_m, (int, float))
            or isinstance(self.footprint_safety_margin_m, bool)
            or not math.isfinite(self.footprint_safety_margin_m)
            or self.footprint_safety_margin_m < 0.0
        ):
            raise ValueError("footprint_safety_margin_m cannot be negative")
        for name in (
            "coverage_max_cells",
            "local_goal_heading_samples",
            "rollout_linear_samples",
            "rollout_angular_samples",
            "rollout_horizon_ns",
            "rollout_step_count",
            "trajectory_replan_interval_ns",
            "trajectory_replan_min_tick_gap",
        ):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        candidate_count = self.rollout_linear_samples * self.rollout_angular_samples
        if not 30 <= candidate_count <= 60:
            raise ValueError("trajectory rollout must contain 30 to 60 candidates")
        if self.rollout_linear_samples < 2 or self.rollout_angular_samples < 3:
            raise ValueError("rollout axes do not provide enough samples")
        if self.rollout_horizon_ns < self.rollout_step_count:
            raise ValueError("rollout horizon must provide positive sample intervals")
        weights = (
            self.progress_weight,
            self.clearance_weight,
            self.smoothness_weight,
            self.novelty_weight,
        )
        if any(
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or value < 0.0
            or not math.isfinite(value)
            for value in weights
        ):
            raise ValueError("trajectory score weights must be finite and non-negative")
        if sum(weights) <= 0.0:
            raise ValueError("at least one trajectory score weight must be positive")
        local_goal_weights = (
            self.local_goal_novelty_weight,
            self.local_goal_clearance_weight,
            self.local_goal_forward_weight,
            self.explore_local_goal_distance_weight,
        )
        if any(
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or value < 0.0
            or not math.isfinite(value)
            for value in local_goal_weights
        ):
            raise ValueError("EXPLORE local-goal weights must be finite and non-negative")
        if sum(local_goal_weights) <= 0.0:
            raise ValueError("at least one EXPLORE local-goal weight must be positive")
        if (
            not isinstance(self.progress_viability_floor, (int, float))
            or isinstance(self.progress_viability_floor, bool)
            or not math.isfinite(self.progress_viability_floor)
            or not 0.0 <= self.progress_viability_floor <= 1.0
        ):
            raise ValueError("progress_viability_floor must be finite and in [0, 1]")
        if (
            not isinstance(self.face_person_min_confidence, (int, float))
            or isinstance(self.face_person_min_confidence, bool)
            or not math.isfinite(self.face_person_min_confidence)
            or not 0.0 <= self.face_person_min_confidence <= 1.0
        ):
            raise ValueError("face_person_min_confidence must be in [0, 1]")
        if (
            not isinstance(self.follow_person_min_confidence, (int, float))
            or isinstance(self.follow_person_min_confidence, bool)
            or not math.isfinite(self.follow_person_min_confidence)
            or not 0.0 <= self.follow_person_min_confidence <= 1.0
        ):
            raise ValueError("follow_person_min_confidence must be in [0, 1]")
        retention = self.follow_person_retention_min_confidence
        if retention is not None and (
            isinstance(retention, bool)
            or not isinstance(retention, (int, float))
            or not math.isfinite(retention)
            or not 0.0 < retention <= self.follow_person_min_confidence
        ):
            raise ValueError("follow_person_retention_min_confidence must be in (0, acquisition confidence]")
        for name in (
            "face_person_align_tolerance_rad",
            "face_person_release_tolerance_rad",
        ):
            value = getattr(self, name)
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value <= 0.0
                or value >= math.pi
            ):
                raise ValueError(f"{name} must be in (0, pi)")
        if self.face_person_release_tolerance_rad <= self.face_person_align_tolerance_rad:
            raise ValueError(
                "face_person_release_tolerance_rad must exceed align tolerance"
            )
        for name in (
            "follow_person_align_tolerance_rad",
            "follow_person_release_tolerance_rad",
        ):
            value = getattr(self, name)
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value <= 0.0
                or value >= math.pi
            ):
                raise ValueError(f"{name} must be in (0, pi)")
        if self.follow_person_release_tolerance_rad <= self.follow_person_align_tolerance_rad:
            raise ValueError(
                "follow_person_release_tolerance_rad must exceed align tolerance"
            )
        for name in (
            "follow_person_stand_off_m",
            "follow_person_min_safe_distance_m",
        ):
            value = getattr(self, name)
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value <= 0.0
            ):
                raise ValueError(f"{name} must be positive")
        if (
            not isinstance(self.follow_person_distance_deadband_m, (int, float))
            or isinstance(self.follow_person_distance_deadband_m, bool)
            or not math.isfinite(self.follow_person_distance_deadband_m)
            or self.follow_person_distance_deadband_m < 0.0
        ):
            raise ValueError("follow_person_distance_deadband_m cannot be negative")
        if (
            self.follow_person_min_safe_distance_m
            >= self.follow_person_stand_off_m - self.follow_person_distance_deadband_m
        ):
            raise ValueError(
                "follow person safe distance must stay below the stand-off deadband"
            )
        if (
            not isinstance(self.follow_person_lost_hold_ns, int)
            or isinstance(self.follow_person_lost_hold_ns, bool)
            or self.follow_person_lost_hold_ns <= 0
        ):
            raise ValueError("follow_person_lost_hold_ns must be a positive integer")
        for name in (
            "follow_person_search_timeout_ns",
            "follow_person_search_step_ns",
            "follow_person_search_max_duration_ns",
        ):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if self.follow_person_search_max_duration_ns < self.follow_person_search_timeout_ns:
            raise ValueError("follow_person_search_max_duration_ns must cover search timeout")
        if (
            not isinstance(self.follow_person_search_sweep_rad, (int, float))
            or isinstance(self.follow_person_search_sweep_rad, bool)
            or not math.isfinite(self.follow_person_search_sweep_rad)
            or not 0.0 < self.follow_person_search_sweep_rad < math.pi
        ):
            raise ValueError("follow_person_search_sweep_rad must be in (0, pi)")
        if (
            not isinstance(self.follow_person_search_yaw_tolerance_rad, (int, float))
            or isinstance(self.follow_person_search_yaw_tolerance_rad, bool)
            or not math.isfinite(self.follow_person_search_yaw_tolerance_rad)
            or not 0.0 < self.follow_person_search_yaw_tolerance_rad < math.pi
        ):
            raise ValueError("follow_person_search_yaw_tolerance_rad must be in (0, pi)")
        if (
            not isinstance(self.follow_person_pivot_enter_rad, (int, float))
            or isinstance(self.follow_person_pivot_enter_rad, bool)
            or not math.isfinite(self.follow_person_pivot_enter_rad)
            or self.follow_person_pivot_enter_rad <= 0.0
            or self.follow_person_pivot_enter_rad >= math.pi
        ):
            raise ValueError("follow_person_pivot_enter_rad must be in (0, pi)")
        if self.follow_person_pivot_enter_rad <= self.follow_person_release_tolerance_rad:
            raise ValueError(
                "follow_person_pivot_enter_rad must exceed release tolerance"
            )
        for name in (
            "follow_person_hold_release_margin_m",
            "follow_person_slowdown_distance_m",
            "follow_person_minimum_follow_speed_mps",
        ):
            value = getattr(self, name)
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value <= 0.0
            ):
                raise ValueError(f"{name} must be positive")
        if (
            not isinstance(self.follow_person_heading_min_factor, (int, float))
            or isinstance(self.follow_person_heading_min_factor, bool)
            or not math.isfinite(self.follow_person_heading_min_factor)
            or not 0.0 < self.follow_person_heading_min_factor <= 1.0
        ):
            raise ValueError("follow_person_heading_min_factor must be in (0, 1]")


@dataclass(frozen=True, slots=True)
class NavigationStateCheckpoint:
    mission_id: str | None
    initial_distance_m: float
    progress: float
    completed: bool
    coverage: tuple[tuple[int, int, int, int], ...]
    local_goal: Waypoint | None
    goal_selected_ns: int
    last_replan_ns: int | None
    last_replan_tick_id: int | None
    trajectory_candidates: tuple[TrajectoryEvaluation, ...]
    pending_rollout_request: TrajectoryRolloutRequest | None = None
    pending_goal_selected_ns: int | None = None
    pending_release_tick_id: int | None = None
    pending_release_not_before_ns: int | None = None
    face_person_track_id: str | None = None
    face_person_aligned: bool = False
    follow_person_track_id: str | None = None
    follow_person_lost_since_ns: int | None = None
    follow_person_pivoting: bool = False
    follow_person_holding: bool = False
    follow_person_last_heading_rad: float | None = None
    follow_person_state: str = _FollowPersonState.ACQUIRE.value
    follow_person_search_phase: int | None = None
    follow_person_search_target_yaw_rad: float | None = None
    follow_person_search_budget_ns: int | None = None
    localization_recovery_started_ns: int | None = None
    localization_recovery_mission_id: str | None = None
    localization_generation: int = -1
    global_transform_revision: int = -1
    localization_envelope: tuple[str, str, int] | None = None
    person_capability_failed_mission_id: str | None = None
    localization_recovery_direction: int = 0
    last_maneuver_direction: int = 0
    goal_selection_reason: str | None = None
    pending_goal_selection_reason: str | None = None
    initial_yaw_error_rad: float = 0.0
    trajectory_geometry_key: tuple[str, int, int, float] | None = None
    trajectory_map_revision: int | None = None
    trajectory_local_sigma_m: float | None = None
    coverage_last_cell: tuple[int, int] | None = None
    escape_candidate: TrajectoryEvaluation | None = None
    escape_goal: Waypoint | None = None
    escape_started_ns: int | None = None
    escape_origin: tuple[float, float, float] | None = None
    failed_goals: tuple[tuple[int, int, int], ...] = ()
    no_viable_since_ns: int | None = None
    goal_reachable: bool = True
    explore_progress_origin: tuple[float, float] | None = None
    explore_progress_started_ns: int | None = None
    explore_stall_count: int = 0
    explore_retry_after_ns: int | None = None
    explore_reachable_exit_mask: int | None = None
    explore_exit_retry_used: bool = False
    explore_probe_after_ns: int | None = None


class TrajectoryRolloutBackend(Protocol):
    """Authority-free compute port injected by the composition root."""

    def submit(self, request: TrajectoryRolloutRequest) -> int: ...
    def take(self, request_id: int) -> TrajectoryRolloutResult | None: ...
    def abandon(self, request_id: int) -> None: ...
    def close(self) -> None: ...


@dataclass(frozen=True, slots=True)
class AsyncL6PlannerConfig:
    """Deterministic handoff policy; process placement is not layer state."""

    enabled: bool
    completion_inputs: bool
    request_timeout_ns: int
    # Legacy replay compatibility. New production handoffs use release_delay_ns.
    release_tick_gap: int
    max_plan_age_ns: int
    release_delay_ns: int | None
    # Missing delivery is a separate hard failure, not a computation deadline.
    transport_timeout_ns: int

    def __post_init__(self) -> None:
        if type(self.completion_inputs) is not bool:
            raise TypeError("completion_inputs must be bool")
        if type(self.request_timeout_ns) is not int or self.request_timeout_ns <= 0:
            raise ValueError("request_timeout_ns must be positive integer")
        if type(self.transport_timeout_ns) is not int or self.transport_timeout_ns <= self.request_timeout_ns:
            raise ValueError("transport_timeout_ns must exceed request_timeout_ns")
        if type(self.enabled) is not bool:
            raise TypeError("enabled must be bool")
        for value, name in (
            (self.release_tick_gap, "release_tick_gap"),
            (self.max_plan_age_ns, "max_plan_age_ns"),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if self.release_delay_ns is not None:
            if (
                not isinstance(self.release_delay_ns, int)
                or isinstance(self.release_delay_ns, bool)
                or self.release_delay_ns <= 0
            ):
                raise ValueError("release_delay_ns must be a positive integer or None")
            if self.release_delay_ns >= self.max_plan_age_ns:
                raise ValueError(
                    "release_delay_ns must be shorter than max_plan_age_ns"
                )


class InlineTrajectoryRolloutBackend:
    """Pure replay/test backend; L6 owns deterministic result visibility."""

    __slots__ = ("_closed", "_computer", "_next_id", "_results")

    def __init__(self, config: NavigationConfig) -> None:
        if not isinstance(config, NavigationConfig):
            raise TypeError("config must be NavigationConfig")
        self._computer = TrajectoryRolloutComputer(config)
        self._next_id = 1
        self._results: dict[int, TrajectoryRolloutResult] = {}
        self._closed = False

    def submit(self, request: TrajectoryRolloutRequest) -> int:
        if self._closed:
            raise RuntimeError("trajectory rollout backend is closed")
        if not isinstance(request, TrajectoryRolloutRequest):
            raise TypeError("request must be TrajectoryRolloutRequest")
        request_id = self._next_id
        self._next_id += 1
        self._results[request_id] = self._computer.compute(request)
        return request_id

    def take(self, request_id: int) -> TrajectoryRolloutResult | None:
        if self._closed:
            raise RuntimeError("trajectory rollout backend is closed")
        return self._results.pop(request_id, None)

    def abandon(self, request_id: int) -> None:
        self._results.pop(request_id, None)

    def close(self) -> None:
        self._results.clear()
        self._closed = True


class TrajectoryNavigator:
    """Own reusable trajectory evaluation plus deterministic exploration state."""

    __slots__ = (
        "_localization_recovery_direction", "_last_maneuver_direction",
        "_goal_selection_reason", "_pending_goal_selection_reason",
        "_localization_recovery_started_ns", "_localization_recovery_mission_id",
        "_localization_generation", "_global_transform_revision", "_localization_envelope",
        "_completed",
        "_completion_inputs",
        "_closed_completion",
        "_request_timeout_ns",
        "_config",
        "_face_person_aligned",
        "_face_person_track_id",
        "_follow_person_holding",
        "_follow_person_lost_since_ns",
        "_follow_person_pivoting",
        "_follow_person_state",
        "_follow_person_search_phase",
        "_follow_person_search_target_yaw_rad",
        "_follow_person_search_budget_ns",
        "_follow_person_evidence",
        "_follow_person_track_id",
        "_follow_person_last_heading_rad",
        "_coverage",
        "_goal_selected_ns",
        "_initial_distance_m",
        "_initial_yaw_error_rad",
        "_last_replan_ns",
        "_last_replan_tick_id",
        "_local_goal",
        "_max_plan_age_ns",
        "_mission_id",
        "_person_capability_failed_mission_id",
        "_pending_goal_selected_ns",
        "_pending_release_not_before_ns",
        "_pending_release_tick_id",
        "_pending_rollout_id",
        "_pending_rollout_request",
        "_progress",
        "_rollout_backend",
        "_rollout_release_delay_ns",
        "_rollout_release_tick_gap",
        "_static_planning_index",
        "_trajectory_candidates",
        "_trajectory_geometry_key",
        "_trajectory_map_revision",
        "_trajectory_local_sigma_m",
        "_coverage_last_cell", "_escape_candidate", "_escape_goal",
        "_escape_started_ns", "_escape_origin", "_failed_goals", "_no_viable_since_ns",
        "_goal_reachable",
        "_explore_progress_origin", "_explore_progress_started_ns",
        "_explore_stall_count", "_explore_retry_after_ns",
        "_explore_reachable_exit_mask", "_explore_exit_retry_used", "_explore_probe_after_ns",
    )

    def __init__(
        self,
        config: NavigationConfig,
        *,
        rollout_backend: TrajectoryRolloutBackend | None = None,
        async_config: AsyncL6PlannerConfig,
    ) -> None:
        rollout_release_tick_gap = async_config.release_tick_gap
        rollout_release_delay_ns = async_config.release_delay_ns
        max_plan_age_ns = async_config.max_plan_age_ns
        completion_inputs = async_config.completion_inputs
        request_timeout_ns = async_config.request_timeout_ns
        if rollout_backend is not None:
            for method_name in ("submit", "take", "abandon", "close"):
                if not callable(getattr(rollout_backend, method_name, None)):
                    raise TypeError(
                        "rollout_backend must implement submit/take/abandon/close"
                    )
        for value, name in (
            (rollout_release_tick_gap, "rollout_release_tick_gap"),
            (max_plan_age_ns, "max_plan_age_ns"),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if rollout_release_delay_ns is not None:
            if (
                not isinstance(rollout_release_delay_ns, int)
                or isinstance(rollout_release_delay_ns, bool)
                or rollout_release_delay_ns <= 0
            ):
                raise ValueError(
                    "rollout_release_delay_ns must be a positive integer or None"
                )
            if rollout_release_delay_ns >= max_plan_age_ns:
                raise ValueError(
                    "rollout_release_delay_ns must be shorter than max_plan_age_ns"
                )
        self._localization_recovery_started_ns = None
        self._localization_recovery_mission_id = None
        self._localization_recovery_direction = self._last_maneuver_direction = 0
        self._goal_selection_reason = self._pending_goal_selection_reason = None
        self._localization_generation = self._global_transform_revision = -1
        self._localization_envelope = None
        self._completion_inputs = completion_inputs
        self._closed_completion: PlannerInput | None = None
        self._request_timeout_ns = request_timeout_ns
        if completion_inputs and rollout_backend is not None:
            raise ValueError("closed-input navigation cannot own a worker backend")
        self._config = config
        self._rollout_backend = rollout_backend
        self._rollout_release_tick_gap = rollout_release_tick_gap
        self._rollout_release_delay_ns = rollout_release_delay_ns
        self._max_plan_age_ns = max_plan_age_ns
        self._mission_id: str | None = None
        self._person_capability_failed_mission_id: str | None = None
        self._initial_distance_m = 0.0
        self._initial_yaw_error_rad = 0.0
        self._progress = 0.0
        self._completed = False
        self._coverage: dict[tuple[int, int], tuple[int, int]] = {}
        self._coverage_last_cell: tuple[int, int] | None = None
        self._escape_candidate: TrajectoryEvaluation | None = None
        self._escape_goal: Waypoint | None = None
        self._escape_started_ns: int | None = None
        self._escape_origin: tuple[float, float, float] | None = None
        self._failed_goals: dict[tuple[int, int], int] = {}
        self._no_viable_since_ns: int | None = None
        self._goal_reachable = True
        self._explore_progress_origin: tuple[float, float] | None = None
        self._explore_progress_started_ns: int | None = None
        self._explore_stall_count = 0
        self._explore_retry_after_ns: int | None = None
        self._explore_reachable_exit_mask: int | None = 0
        self._explore_exit_retry_used = False
        self._explore_probe_after_ns: int | None = None
        self._local_goal: Waypoint | None = None
        self._goal_selected_ns = 0
        self._last_replan_ns: int | None = None
        self._last_replan_tick_id: int | None = None
        self._pending_rollout_id: int | None = None
        self._pending_rollout_request: TrajectoryRolloutRequest | None = None
        self._pending_goal_selected_ns: int | None = None
        self._pending_release_tick_id: int | None = None
        self._pending_release_not_before_ns: int | None = None
        self._static_planning_index: _StaticPlanningIndex | None = None
        self._trajectory_candidates: tuple[TrajectoryEvaluation, ...] = ()
        self._trajectory_geometry_key: tuple[str, int, int, float] | None = None
        self._trajectory_map_revision: int | None = None
        self._trajectory_local_sigma_m: float | None = None
        self._face_person_track_id: str | None = None
        self._face_person_aligned = False
        self._follow_person_track_id: str | None = None
        self._follow_person_lost_since_ns: int | None = None
        self._follow_person_pivoting = False
        self._follow_person_holding = False
        self._follow_person_last_heading_rad: float | None = None
        self._follow_person_state = _FollowPersonState.ACQUIRE
        self._follow_person_search_phase: int | None = None
        self._follow_person_search_target_yaw_rad: float | None = None
        self._follow_person_search_budget_ns: int | None = None
        self._follow_person_evidence: FollowPersonEvidence | None = None

    def checkpoint(self) -> NavigationStateCheckpoint:
        return NavigationStateCheckpoint(
            self._mission_id,
            self._initial_distance_m,
            self._progress,
            self._completed,
            tuple(
                (x_index, y_index, visits, tick_id)
                for (x_index, y_index), (visits, tick_id) in sorted(
                    self._coverage.items()
                )
            ),
            self._local_goal,
            self._goal_selected_ns,
            self._last_replan_ns,
            self._last_replan_tick_id,
            self._trajectory_candidates,
            self._pending_rollout_request,
            self._pending_goal_selected_ns,
            self._pending_release_tick_id,
            self._pending_release_not_before_ns,
            self._face_person_track_id,
            self._face_person_aligned,
            self._follow_person_track_id,
            self._follow_person_lost_since_ns,
            self._follow_person_pivoting,
            self._follow_person_holding,
            self._follow_person_last_heading_rad,
            self._follow_person_state.value,
            self._follow_person_search_phase,
            self._follow_person_search_target_yaw_rad,
            self._follow_person_search_budget_ns,
            self._localization_recovery_started_ns, self._localization_recovery_mission_id,
            self._localization_generation, self._global_transform_revision, self._localization_envelope,
            self._person_capability_failed_mission_id,
            self._localization_recovery_direction, self._last_maneuver_direction,
            self._goal_selection_reason, self._pending_goal_selection_reason,
            self._initial_yaw_error_rad,
            self._trajectory_geometry_key,
            self._trajectory_map_revision,
            self._trajectory_local_sigma_m,
            self._coverage_last_cell, self._escape_candidate, self._escape_goal,
            self._escape_started_ns, self._escape_origin,
            tuple((x, y, until) for (x, y), until in sorted(self._failed_goals.items())),
            self._no_viable_since_ns,
            self._goal_reachable,
            self._explore_progress_origin, self._explore_progress_started_ns,
            self._explore_stall_count, self._explore_retry_after_ns,
            self._explore_reachable_exit_mask, self._explore_exit_retry_used,
            self._explore_probe_after_ns,
        )

    def restore(self, checkpoint: NavigationStateCheckpoint) -> None:
        if not isinstance(checkpoint, NavigationStateCheckpoint):
            raise TypeError("checkpoint must be NavigationStateCheckpoint")
        self._localization_recovery_started_ns = checkpoint.localization_recovery_started_ns
        self._localization_recovery_mission_id = checkpoint.localization_recovery_mission_id
        self._localization_generation = checkpoint.localization_generation
        self._global_transform_revision = checkpoint.global_transform_revision
        self._localization_envelope = checkpoint.localization_envelope
        self._person_capability_failed_mission_id = checkpoint.person_capability_failed_mission_id
        self._localization_recovery_direction = checkpoint.localization_recovery_direction
        self._last_maneuver_direction = checkpoint.last_maneuver_direction
        self._goal_selection_reason = checkpoint.goal_selection_reason
        self._abandon_pending_rollout()
        self._mission_id = checkpoint.mission_id
        self._initial_distance_m = checkpoint.initial_distance_m
        self._initial_yaw_error_rad = checkpoint.initial_yaw_error_rad
        self._progress = checkpoint.progress
        self._completed = checkpoint.completed
        self._coverage = {
            (x_index, y_index): (visits, tick_id)
            for x_index, y_index, visits, tick_id in checkpoint.coverage
        }
        self._local_goal = checkpoint.local_goal
        self._goal_selected_ns = checkpoint.goal_selected_ns
        self._last_replan_ns = checkpoint.last_replan_ns
        self._last_replan_tick_id = checkpoint.last_replan_tick_id
        self._trajectory_candidates = checkpoint.trajectory_candidates
        self._trajectory_geometry_key = checkpoint.trajectory_geometry_key
        self._trajectory_map_revision = checkpoint.trajectory_map_revision
        self._trajectory_local_sigma_m = checkpoint.trajectory_local_sigma_m
        self._coverage_last_cell = checkpoint.coverage_last_cell
        self._escape_candidate = checkpoint.escape_candidate
        self._escape_goal = checkpoint.escape_goal
        self._escape_started_ns = checkpoint.escape_started_ns
        self._escape_origin = checkpoint.escape_origin
        self._failed_goals = {(x, y): until for x, y, until in checkpoint.failed_goals}
        self._no_viable_since_ns = checkpoint.no_viable_since_ns
        self._goal_reachable = checkpoint.goal_reachable
        self._explore_progress_origin = checkpoint.explore_progress_origin
        self._explore_progress_started_ns = checkpoint.explore_progress_started_ns
        self._explore_stall_count = checkpoint.explore_stall_count
        self._explore_retry_after_ns = checkpoint.explore_retry_after_ns
        self._explore_reachable_exit_mask = checkpoint.explore_reachable_exit_mask
        self._explore_exit_retry_used = checkpoint.explore_exit_retry_used
        self._explore_probe_after_ns = checkpoint.explore_probe_after_ns
        self._face_person_track_id = checkpoint.face_person_track_id
        self._face_person_aligned = checkpoint.face_person_aligned
        self._follow_person_track_id = checkpoint.follow_person_track_id
        self._follow_person_lost_since_ns = checkpoint.follow_person_lost_since_ns
        self._follow_person_pivoting = checkpoint.follow_person_pivoting
        self._follow_person_holding = checkpoint.follow_person_holding
        self._follow_person_last_heading_rad = checkpoint.follow_person_last_heading_rad
        self._follow_person_state = _FollowPersonState(checkpoint.follow_person_state)
        self._follow_person_search_phase = checkpoint.follow_person_search_phase
        self._follow_person_search_target_yaw_rad = checkpoint.follow_person_search_target_yaw_rad
        self._follow_person_search_budget_ns = checkpoint.follow_person_search_budget_ns
        self._follow_person_evidence = None
        # Derived acceleration state is deliberately not part of replay authority.
        self._static_planning_index = None
        pending = checkpoint.pending_rollout_request
        if pending is not None and self._completion_inputs:
            self._pending_rollout_id = pending.context.tick_id + 1
            self._pending_rollout_request = pending
            self._pending_goal_selected_ns = checkpoint.pending_goal_selected_ns
            self._pending_goal_selection_reason = checkpoint.pending_goal_selection_reason
            return
        if pending is not None:
            backend = self._rollout_backend
            if backend is None:
                raise RuntimeError("async navigation checkpoint requires rollout backend")
            release_tick_id = checkpoint.pending_release_tick_id
            release_not_before_ns = checkpoint.pending_release_not_before_ns
            if (release_tick_id is None) == (release_not_before_ns is None):
                raise RuntimeError(
                    "async navigation checkpoint must contain exactly one release criterion"
                )
            if (
                release_not_before_ns is not None
                and release_not_before_ns <= pending.context.monotonic_ns
            ):
                raise RuntimeError(
                    "async navigation checkpoint release time is not after request time"
                )
            request_id = backend.submit(pending)
            self._pending_rollout_id = request_id
            self._pending_rollout_request = pending
            self._pending_goal_selected_ns = checkpoint.pending_goal_selected_ns
            self._pending_goal_selection_reason = checkpoint.pending_goal_selection_reason
            self._pending_release_tick_id = release_tick_id
            self._pending_release_not_before_ns = release_not_before_ns

    def evaluate(
        self,
        mission: MissionIntent,
        estimate: RobotEstimate,
        world: WorldSnapshot,
        planner_input: PlannerInput | None = None,
    ) -> NavigationPlan:
        estimate = estimate.in_local_frame()
        if mission.lifecycle is MissionLifecycle.ACTIVE and mission.context == estimate.context == world.context:
            if mission.mode in (CommandMode.FACE_PERSON, CommandMode.FOLLOW_PERSON):
                # Closed capability health revokes vision motion before target
                # prediction, SEARCH or localization-recovery guidance can run.
                if (
                    self._person_capability_failed_mission_id == mission.mission_id
                    or world.person_detection_state is DeviceHealthState.FAILED
                ):
                    if self._person_capability_failed_mission_id != mission.mission_id:
                        self._reset()
                        self._person_capability_failed_mission_id = mission.mission_id
                    self._clear_trajectory_plan()
                    return self._inactive(mission, NavigationStatus.INVALIDATED, "PERSON_CAPABILITY_FAILED")
                if world.person_detection_state in {DeviceHealthState.UNKNOWN, DeviceHealthState.DEGRADED}:
                    self._clear_trajectory_plan()
                    return self._inactive(mission, NavigationStatus.IDLE, "PERSON_CAPABILITY_UNAVAILABLE")
            recovery = self._localization_plan(mission, estimate, world)
            if recovery is not None:
                return recovery
        else:
            self._localization_recovery_started_ns = None
            self._localization_recovery_mission_id = None
        plan = self._evaluate(mission, estimate, world, planner_input)
        if (plan.status is NavigationStatus.ACTIVE and self._escape_candidate is None
                and plan.local_goal is not None):
            viable = tuple(c for c in plan.trajectory_candidates if not c.collision and c.progress_viable)
            if viable and all(c.candidate_id.startswith("escape-") for c in viable):
                # Recovery is one L6-owned bounded maneuver; subsequent planner
                # replacements cannot reverse its direction or replace its goal.
                self._escape_candidate = min(viable, key=lambda c: (-c.total_score, c.candidate_id))
                self._escape_goal = plan.local_goal
                self._escape_started_ns = mission.context.monotonic_ns
                self._escape_origin = (estimate.x_m, estimate.y_m, estimate.yaw_rad)
                if mission.mode is CommandMode.EXPLORE:
                    self._remember_failed_goal(plan.local_goal, mission.context.monotonic_ns)
                rejected = tuple(replace(c, samples=(c.samples[-1],))
                                 for c in plan.trajectory_candidates
                                 if c.collision or not c.progress_viable)
                plan = (self._local_continuation(mission, estimate, world,
                            max_v_mps=plan.constraints.max_v_mps)
                        or self._inactive(mission, NavigationStatus.IDLE, "RECOVERY_GEOMETRY_HOLD"))
                if plan.status is NavigationStatus.ACTIVE:
                    # One freshly revalidated maneuver owns the phase; compact
                    # negative evidence still revokes the previous normal path.
                    plan = replace(plan, trajectory_candidates=rejected + plan.trajectory_candidates)
        if (plan.status is NavigationStatus.ACTIVE and plan.trajectory_candidates
                and plan.motion_validity is None
                and self._cached_geometry_changed(estimate, world)):
            # A young worker result is not proof against a newer obstacle map
            # or larger localization footprint. Recheck a bounded family;
            # a failed check must not fall back to the old candidate family.
            plan = (self._local_continuation(mission, estimate, world,
                        max_v_mps=plan.constraints.max_v_mps)
                    or self._inactive(mission, NavigationStatus.IDLE,
                                      "PLANNER_GEOMETRY_HOLD"))
        if (plan.status is NavigationStatus.IDLE and plan.reason == "PLANNER_GEOMETRY_HOLD"
                and plan.rejection_witness is not None and self._last_replan_ns is not None
                and any(not c.collision and c.progress_viable for c in self._trajectory_candidates)):
            self._reject_unexecutable_explore_family(mission, estimate, world,
                self._guidance_validity(mission, world, trajectory=True, translation=True))
        if plan.status is NavigationStatus.ACTIVE and plan.motion_validity is None:
            plan = replace(plan, motion_validity=self._guidance_validity(
                mission, world, trajectory=bool(plan.trajectory_candidates),
                translation=(bool(plan.trajectory_candidates)
                    or (plan.velocity_target is not None and abs(plan.velocity_target.v_mps) > 1e-12)
                    or bool(plan.route and math.hypot(plan.route[0].x_m-estimate.x_m,
                               plan.route[0].y_m-estimate.y_m) > mission.constraints.goal_tolerance_m)),
            ))
        if plan.status in (NavigationStatus.ACTIVE, NavigationStatus.PENDING):
            proof_validity = plan.motion_validity
            proof = self._motion_geometry_proof(plan, estimate, world)
            viable = tuple(c for c in plan.trajectory_candidates if not c.collision and c.progress_viable)
            rejected_family = plan.status is NavigationStatus.ACTIVE and bool(viable)
            if proof is None and plan.status is NavigationStatus.ACTIVE and len(viable) > 1:
                # A free circle cannot certify every candidate in this family.
                # Revalidate the bounded family, as at the geometry-change
                # boundary; no worker search or downstream authority is read.
                rejected = tuple(replace(c, samples=(c.samples[-1],))
                                 for c in plan.trajectory_candidates if c.collision or not c.progress_viable)
                plan = (self._local_continuation(mission, estimate, world,
                            max_v_mps=plan.constraints.max_v_mps)
                        or self._inactive(mission, NavigationStatus.IDLE, "PLANNER_EXECUTION_HOLD"))
                if plan.status is NavigationStatus.ACTIVE:
                    plan = replace(plan, trajectory_candidates=rejected + plan.trajectory_candidates)
                    proof = self._motion_geometry_proof(plan, estimate, world)
            if proof is None and plan.status is NavigationStatus.ACTIVE and viable:
                # An unproved realizable transition is an explicit HOLD, not
                # an ACTIVE goal whose motion is repeatedly rejected in L9.
                candidate = viable[0]
                costmap = world.local_costmap
                witness = NavigationRejectionWitness(candidate.candidate_id,
                    "EXECUTION_PROOF_UNAVAILABLE", mission.context,
                    costmap.revision, max(0, mission.context.monotonic_ns-costmap.freshness_ns),
                    world.map_revision, 0, estimate.x_m, estimate.y_m, estimate.yaw_rad,
                    candidate.min_clearance_m, self._config.footprint_safety_margin_m,
                    self._config.localization_inflation_sigma * estimate.localization_quality.local_sigma_m)
                plan = replace(self._inactive(mission, NavigationStatus.IDLE, "PLANNER_EXECUTION_HOLD"),
                               rejection_witness=witness)
            if proof is None and rejected_family:
                self._reject_unexecutable_explore_family(mission, estimate, world, proof_validity)
            if proof is not None and proof.maneuver_bounds is not None and plan.trajectory_candidates:
                covered = tuple(
                    c for c in plan.trajectory_candidates
                    if c.collision or not c.progress_viable or proof.covers(c.v_mps, c.omega_rad_s))
                if len(covered) != len(plan.trajectory_candidates):
                    plan = replace(plan, trajectory_candidates=covered)
            plan = replace(plan, geometry_proof=proof)
        if plan.status is NavigationStatus.ACTIVE:
            omega = (plan.velocity_target.omega_rad_s if plan.velocity_target is not None
                     else estimate.omega_rad_s)
            if abs(omega) > 1e-3 and estimate.localization_quality.heading is QualityState.GOOD:
                self._last_maneuver_direction = 1 if omega > 0 else -1
        if mission.mode is CommandMode.EXPLORE and self._mission_id == mission.mission_id:
            plan = replace(plan, progress=self._progress,
                goal_selection_reason=self._goal_selection_reason,
                goal_selected_ns=self._goal_selected_ns)
        return plan

    def _reject_unexecutable_explore_family(
        self, mission: MissionIntent, estimate: RobotEstimate,
        world: WorldSnapshot, validity: MotionValidity | None,
    ) -> None:
        """A fresh negative proof must enter the existing bounded exit search."""
        quality, costmap = estimate.localization_quality, world.local_costmap
        if (mission.mode is not CommandMode.EXPLORE or self._escape_candidate is not None
                or self._mission_id != mission.mission_id or validity is None
                or not validity.usable_at(mission.context) or costmap is None
                or costmap.frame_id != world.frame_id or world.frame_id != estimate.frame_id
                or world.freshness_ns > self._config.max_world_freshness_ns
                or costmap.freshness_ns > self._config.max_costmap_freshness_ns
                or quality.local_translation is QualityState.LOST
                or quality.heading is QualityState.LOST
                or not quality.local_pose_continuous or quality.pose_discontinuity):
            return
        # A pending result for this endpoint only proves its old nominal
        # family; it cannot undo the current execution rejection. Keep the
        # existing hysteresis and measured-progress budgets, rather than
        # renewing either when a new worker result or goal arrives.
        self._abandon_pending_rollout()
        self._goal_reachable = False
        self._trajectory_candidates = tuple(replace(c, progress_viable=False)
                                             for c in self._trajectory_candidates)
        if self._no_viable_since_ns is None:
            self._no_viable_since_ns = mission.context.monotonic_ns

    def _motion_geometry_proof(
        self, plan: NavigationPlan, estimate: RobotEstimate, world: WorldSnapshot,
    ) -> LocalMotionProof | None:
        validity, costmap = plan.motion_validity, world.local_costmap
        quality = estimate.localization_quality
        now = plan.context.monotonic_ns
        if (validity is None or not validity.usable_at(plan.context) or costmap is None
                or costmap.frame_id != world.frame_id or world.frame_id != estimate.frame_id
                or costmap.freshness_ns > self._config.max_costmap_freshness_ns
                or world.freshness_ns > self._config.max_world_freshness_ns
                or quality.local_translation is QualityState.LOST
                or quality.heading is QualityState.LOST
                or not quality.local_pose_continuous or quality.pose_discontinuity):
            return None
        horizon_ns = self._config.motion_proof_horizon_ns
        uncertainty = self._config.localization_inflation_sigma * quality.local_sigma_m
        robot_radius = _footprint_radius(self._config, uncertainty)
        margin = robot_radius + self._config.footprint_safety_margin_m
        # Only the displacement needed under the hard speed cap is queried.
        speed_cap = min(plan.constraints.max_v_mps, self._config.operational_max_v_mps)
        bound = speed_cap * horizon_ns / 1e9
        scene = self._build_planning_scene(world)
        clearance = min(costmap.radius_m, _planning_scene_point_clearance(
            estimate.x_m, estimate.y_m, scene, costmap.radius_m, margin + bound))
        for obstacle in world.obstacle_tracks:
            if obstacle.confidence >= self._config.obstacle_confidence_floor:
                clearance = min(clearance, math.hypot(obstacle.x_m-estimate.x_m, obstacle.y_m-estimate.y_m)
                    - obstacle.radius_m - math.hypot(obstacle.vx_mps, obstacle.vy_mps)*horizon_ns/1e9)
        captured_ns = now - costmap.freshness_ns
        until = min(validity.valid_until_ns,
                    captured_ns + self._config.max_costmap_freshness_ns,
                    now + self._config.max_world_freshness_ns - world.freshness_ns)
        candidates = tuple(c for c in plan.trajectory_candidates
                           if not c.collision and c.progress_viable)
        displacement = max(0., min(bound, clearance-margin))
        required_speed = max((abs(c.v_mps) for c in candidates), default=0.)
        if self._escape_candidate is not None:
            required_speed = max(required_speed, abs(self._escape_candidate.v_mps))
        if clearance >= margin and required_speed * horizon_ns / 1e9 <= displacement:
            return LocalMotionProof(plan.context, world.frame_id, validity.scope,
                costmap.revision, captured_ns, until, horizon_ns, displacement)
        # An insufficient circle also needs a directional certificate, even
        # when its radius is positive. Try at most four revalidated proposals;
        # the caller retains only candidates covered by the resulting bound.
        if not 1 <= len(candidates) <= 4:
            return None
        scene = replace(scene, uncertainty_m=uncertainty)
        phase = self._escape_candidate
        for candidate in candidates:
            if phase is not None:
                # A soft speed cap can reduce the new target before L9 has finished
                # braking the same owned phase. Prove its original family afresh,
                # so the previous allowed twist can remain inside the gap envelope.
                if (abs(candidate.v_mps) <= _MOTION_EPSILON and abs(phase.v_mps) <= _MOTION_EPSILON
                        and candidate.omega_rad_s * phase.omega_rad_s > 0.0):
                    candidate = replace(candidate, omega_rad_s=math.copysign(
                        max(abs(candidate.omega_rad_s), abs(phase.omega_rad_s)), candidate.omega_rad_s))
                elif (candidate.v_mps < 0.0 and phase.v_mps < 0.0
                        and abs(candidate.omega_rad_s) <= _MOTION_EPSILON
                        and abs(phase.omega_rad_s) <= _MOTION_EPSILON):
                    candidate = replace(candidate, v_mps=min(candidate.v_mps, phase.v_mps))
            bounds = (_escape_motion_bounds(candidate, estimate, world, scene, self._config, horizon_ns)
                      if candidate.candidate_id.startswith("escape-") else None)
            if bounds is None:
                bounds = _normal_motion_bounds(candidate, estimate, world, scene,
                    self._config, horizon_ns, min(plan.constraints.max_v_mps, self._config.operational_max_v_mps),
                    min(plan.constraints.max_omega_rad_s, self._config.operational_max_omega_rad_s))
            if bounds is not None:
                return LocalMotionProof(plan.context, world.frame_id, validity.scope,
                    costmap.revision, captured_ns, until, horizon_ns, 0.0, bounds)
        return None

    def _goal_handoff_ns(self) -> int:
        # Completion-input mode has no release_delay authority. The next replan
        # plus the bounded request lifetime is the actual replacement budget.
        return self._config.trajectory_replan_interval_ns + min(self._request_timeout_ns, self._max_plan_age_ns)

    def _guidance_validity(
        self, mission: MissionIntent, world: WorldSnapshot, *, trajectory: bool = False,
        translation: bool | None = None,
    ) -> MotionValidity:
        # Cached guidance keeps its computation time. L7 alone retains the
        # selected objective; a pending request cannot refresh that objective.
        source = mission.context
        if trajectory:
            assert self._last_replan_ns is not None and self._last_replan_tick_id is not None
            source = TickContext(self._last_replan_tick_id, self._last_replan_ns)
        until_ns = source.monotonic_ns + self._max_plan_age_ns
        if trajectory:
            until_ns = min(until_ns, source.monotonic_ns + min(
                candidate.horizon_ns for candidate in self._trajectory_candidates
            ))
        scope = mission.mode.value
        target_uid = (self._follow_person_track_id if mission.mode is CommandMode.FOLLOW_PERSON
                      else self._face_person_track_id if mission.mode is CommandMode.FACE_PERSON
                      else None)
        if target_uid is not None:
            scope += ":" + target_uid
            track = next((item for item in world.obstacle_tracks if item.track_id == target_uid), None)
            # SEARCH has its own current guidance. FOLLOW prediction must not
            # renew the underlying person's measurement lifetime.
            if track is not None and track.usable_at(mission.context.monotonic_ns):
                if track.prediction_valid_until_ns is not None:
                    until_ns = min(until_ns, track.prediction_valid_until_ns)
        # Motion authority follows odometry lineage, not soft localization quality.
        # GOOD/DEGRADED and sigma changes are handled as dynamic motion evidence.
        scope += f":odom{self._localization_generation}"
        global_target = mission.mode is CommandMode.NAVIGATE and mission.target_frame_id != LOCAL_FRAME_ID
        if global_target:
            scope += f":map{self._global_transform_revision}"
        return MotionValidity(
            source, until_ns, world.frame_id, scope,
            localization_requirement=LocalizationRequirement(
                (mission.mode not in (CommandMode.FACE_PERSON, CommandMode.STOP)) if translation is None else translation,
                mission.mode is not CommandMode.STOP, global_target and not trajectory,
            ),
        )

    def _localization_plan(self, mission, estimate, world):
        if self._completed and self._mission_id == mission.mission_id:
            return None
        if (mission.mode is CommandMode.TELEOP and mission.velocity_target is not None
                and mission.velocity_target.v_mps == mission.velocity_target.omega_rad_s == 0.0):
            return None
        quality = estimate.localization_quality
        envelope = (quality.local_translation.value, quality.heading.value, math.ceil(quality.local_sigma_m/.02))
        if envelope != self._localization_envelope:
            # Soft localization changes may trigger the normal periodic replan, but
            # must not revoke a still-valid cached trajectory. LOST and lineage
            # changes remain hard invalidation gates below; L9 gates discontinuity.
            self._localization_envelope = envelope
        global_target = mission.mode is CommandMode.NAVIGATE and mission.target_frame_id != LOCAL_FRAME_ID
        if self._localization_generation != quality.generation:
            self._clear_trajectory_plan()
            self._local_goal = None
            self._coverage.clear()
            self._coverage_last_cell = None
            self._progress = 0.0
            self._failed_goals.clear()
            self._clear_explore_progress()
            self._localization_generation = quality.generation
        if (global_target and quality.global_position is QualityState.GOOD
                and self._global_transform_revision != estimate.transform_revision):
            self._clear_trajectory_plan()
            self._local_goal = None
        if not global_target or quality.global_position is QualityState.GOOD:
            self._global_transform_revision = estimate.transform_revision
        translation_needed = mission.mode not in (CommandMode.FACE_PERSON, CommandMode.STOP)
        if mission.mode is CommandMode.TELEOP and mission.velocity_target is not None:
            translation_needed = abs(mission.velocity_target.v_mps) > 1e-12
        if (mission.mode is CommandMode.NAVIGATE and mission.target_frame_id == LOCAL_FRAME_ID
                and mission.target_pose is not None
                and math.hypot(mission.target_pose.x_m-estimate.x_m,
                               mission.target_pose.y_m-estimate.y_m) <= mission.constraints.goal_tolerance_m):
            translation_needed = False
        needs_recovery = ((translation_needed and quality.local_translation is QualityState.LOST)
                          or quality.heading is QualityState.LOST
                          or (global_target and quality.global_position is not QualityState.GOOD
                              and (self._mission_id != mission.mission_id or self._local_goal is None)))
        if not needs_recovery:
            if self._localization_recovery_started_ns is not None:
                self._clear_trajectory_plan()
            self._localization_recovery_started_ns = None
            self._localization_recovery_mission_id = None
            self._localization_recovery_direction = 0
            return None
        self._clear_trajectory_plan()
        now = mission.context.monotonic_ns
        if self._localization_recovery_mission_id != mission.mission_id:
            self._localization_recovery_started_ns = now
            self._localization_recovery_mission_id = mission.mission_id
            self._localization_recovery_direction = (
                (1 if estimate.omega_rad_s > 0 else -1) if abs(estimate.omega_rad_s) > 1e-3
                else (self._last_maneuver_direction if self._mission_id == mission.mission_id else 0) or 1)
        elapsed = now - self._localization_recovery_started_ns
        geometry = world.robot_relative_geometry
        # Recovery must not depend on the pose-aligned costmap: L4 intentionally
        # freezes that map when local translation is LOST.  Instead require a
        # fresh ROBOT_BASE scan token.  Translation remains zero and L12 still
        # independently owns directional raw-LiDAR collision safety.
        if (mission.mode is CommandMode.TELEOP
                or elapsed >= self._config.localization_recovery_timeout_ns
                or quality.heading is not QualityState.GOOD
                or quality.lidar_age_ns > self._config.max_costmap_freshness_ns
                or estimate.frame_id != world.frame_id
                or world.freshness_ns > self._config.max_world_freshness_ns
                or geometry is None
                or geometry.point_count <= 0
                or geometry.freshness_ns > self._config.max_costmap_freshness_ns):
            return self._inactive(mission, NavigationStatus.IDLE, "LOCALIZATION_HOLD")
        # Recovery must itself be realizable without increasing the authorized
        # angular speed. A mission cap below the calibrated spin floor is HOLD.
        recovery_omega = min(mission.constraints.max_omega_rad_s,
                             self._config.localization_recovery_omega_rad_s)
        _, recovery_omega = self._config.wheel_limits.constrain(0.0, recovery_omega)
        recovery_omega *= self._localization_recovery_direction
        if recovery_omega == 0.0:
            return self._inactive(mission, NavigationStatus.IDLE, "LOCALIZATION_HOLD")
        # No translation; L12 independently verifies the raw side clearances.
        return NavigationPlan(
            context=mission.context, mission_id=mission.mission_id, route=(),
            velocity_target=VelocityTarget(0.0, recovery_omega),
            constraints=replace(mission.constraints, max_omega_rad_s=min(mission.constraints.max_omega_rad_s,
                                                  self._config.localization_recovery_omega_rad_s)),
            corridor_radius_m=0.0, progress=self._progress, status=NavigationStatus.ACTIVE,
            reason="LOCALIZATION_REACQUIRE",
            motion_validity=MotionValidity(mission.context, now+self._config.rollout_horizon_ns,
                world.frame_id, f"REACQUIRE:{quality.generation}", LocalizationRequirement(False, True, False)),
        )

    def _evaluate(
        self,
        mission: MissionIntent,
        estimate: RobotEstimate,
        world: WorldSnapshot,
        planner_input: PlannerInput | None = None,
    ) -> NavigationPlan:
        if planner_input is not None and planner_input.context != mission.context:
            raise ValueError("planner input context mismatch")
        self._closed_completion = planner_input
        self._follow_person_evidence = None
        if mission.context != estimate.context or mission.context != world.context:
            return self._inactive(mission, NavigationStatus.INVALIDATED, "CONTEXT_MISMATCH")
        if mission.lifecycle is not MissionLifecycle.ACTIVE:
            self._reset()
            status = (
                NavigationStatus.INVALIDATED
                if mission.lifecycle is MissionLifecycle.FAILED
                else NavigationStatus.IDLE
            )
            return self._inactive(mission, status, mission.stop_reason)
        if estimate.frame_id != world.frame_id:
            self._reset()
            return self._inactive(mission, NavigationStatus.INVALIDATED, "FRAME_MISMATCH")
        if world.freshness_ns > self._config.max_world_freshness_ns:
            # Transient input invalidity revokes motion, not mission identity.
            # Never reuse a cached/pending rollout after the stale interval.
            if self._mission_id != mission.mission_id:
                self._reset()
            else:
                self._clear_trajectory_plan()
            return self._inactive(mission, NavigationStatus.INVALIDATED, "WORLD_STALE")

        if self._escape_candidate is not None and self._mission_id == mission.mission_id:
            self._accept_pending_rollout(mission.context)
            if mission.mode is CommandMode.EXPLORE:
                self._mark_coverage(estimate.x_m, estimate.y_m, mission.context.tick_id)
                self._progress = min(1.0, len(self._coverage) / self._config.coverage_max_cells)
                self._observe_explore_progress(estimate)
            recovery = self._active_escape_plan(mission, estimate, world)
            if recovery is not None:
                return recovery

        if (mission.mode is CommandMode.NAVIGATE and mission.target_frame_id != LOCAL_FRAME_ID
                and estimate.localization_quality.global_position is not QualityState.GOOD):
            # The committed odometry-frame segment survives map uncertainty.
            # Do not transform a new global goal or falsely complete the mission
            # at this intermediate waypoint. Only fresh local proof can move.
            continuation = self._local_continuation(mission, estimate, world)
            return continuation or self._inactive(mission, NavigationStatus.IDLE, "LOCAL_GUIDANCE_UNAVAILABLE")

        if mission.mode is CommandMode.TELEOP:
            self._reset()
            return NavigationPlan(
                context=mission.context,
                mission_id=mission.mission_id,
                route=(),
                velocity_target=mission.velocity_target,
                constraints=mission.constraints,
                corridor_radius_m=0.0,
                progress=0.0,
                status=NavigationStatus.ACTIVE,
            )
        if mission.mode is CommandMode.FACE_PERSON:
            return self._face_person_plan(mission, estimate, world)
        if mission.mode is CommandMode.FOLLOW_PERSON:
            return self._follow_person_plan(mission, estimate, world)
        if mission.mode is CommandMode.EXPLORE:
            return self._exploration_plan(mission, estimate, world)

        target = mission.target_pose
        if target is None:
            self._reset()
            return self._inactive(mission, NavigationStatus.INVALIDATED, "TARGET_MISSING")

        if mission.target_frame_id != LOCAL_FRAME_ID:
            if estimate.map_to_odom is None:
                self._clear_trajectory_plan()
                return self._inactive(
                    mission,
                    NavigationStatus.INVALIDATED,
                    "GLOBAL_TRANSFORM_MISSING",
                )
            target = Waypoint(*estimate.map_to_odom.inverse().apply(target.x_m, target.y_m, target.yaw_rad))
        distance_m = math.hypot(target.x_m - estimate.x_m, target.y_m - estimate.y_m)
        yaw_error_rad = (0.0 if target.yaw_rad is None else
                         abs(_wrapped_angle(target.yaw_rad - estimate.yaw_rad)))
        if self._mission_id != mission.mission_id:
            self._reset()
            self._mission_id = mission.mission_id
            self._initial_distance_m = max(distance_m, mission.constraints.goal_tolerance_m)
            self._initial_yaw_error_rad = max(yaw_error_rad, mission.constraints.yaw_tolerance_rad)
            self._progress = 0.0
            self._completed = False
        if self._completed:
            return self._complete_plan(mission)
        progress = (1.0 if distance_m == 0.0 else
                    max(0.0, 1.0 - distance_m / self._initial_distance_m)
                    if self._initial_distance_m > 0.0 else 0.0)
        if target.yaw_rad is not None:
            initial_yaw_error_rad = max(self._initial_yaw_error_rad, mission.constraints.yaw_tolerance_rad)
            yaw_progress = (1.0 if yaw_error_rad == 0.0 else
                            max(0.0, 1.0 - yaw_error_rad / initial_yaw_error_rad)
                            if initial_yaw_error_rad > 0.0 else 0.0)
            progress = min(progress, yaw_progress)
        # Only the explicit completion branch may report 100%; a rotation-only
        # mission has positional arrival throughout its still-active yaw phase.
        self._progress = min(math.nextafter(1.0, 0.0), max(self._progress, progress))
        yaw_reached = yaw_error_rad <= mission.constraints.yaw_tolerance_rad
        if distance_m <= mission.constraints.goal_tolerance_m and yaw_reached:
            # A wheel-propagated map pose can cross the arrival envelope
            # between independent fixes. Hold until L3 accepts a current fix;
            # prediction alone must not latch a global mission as complete.
            if (mission.target_frame_id != LOCAL_FRAME_ID
                    and estimate.localization_quality.global_fix_status != "ACCEPTED"):
                return self._inactive(mission, NavigationStatus.IDLE, "GLOBAL_ARRIVAL_PENDING")
            self._completed = True
            self._progress = 1.0
            return self._complete_plan(mission)

        if distance_m <= mission.constraints.goal_tolerance_m:
            # Terminal yaw alignment uses heading and direct L12 safety, not
            # pose-aligned occupancy. An unavailable derived map must not
            # revoke this rotation-only guidance.
            self._clear_trajectory_plan()
            return NavigationPlan(
                context=mission.context, mission_id=mission.mission_id,
                route=(target,), velocity_target=None, constraints=mission.constraints,
                corridor_radius_m=mission.constraints.corridor_radius_m,
                progress=self._progress, status=NavigationStatus.ACTIVE,
            )

        costmap = world.local_costmap
        if costmap is None:
            self._clear_trajectory_plan()
            return self._inactive(
                mission,
                NavigationStatus.INVALIDATED,
                "LOCAL_COSTMAP_MISSING",
            )
        if costmap.freshness_ns > self._config.max_costmap_freshness_ns:
            self._clear_trajectory_plan()
            return self._inactive(
                mission,
                NavigationStatus.INVALIDATED,
                "LOCAL_COSTMAP_STALE",
            )
        disposition = self._accept_pending_rollout(mission.context)
        if self._replan_due(
            mission.context.monotonic_ns,
            mission.context.tick_id,
        ):
            local_distance = min(distance_m, self._config.local_goal_distance_m)
            heading = math.atan2(target.y_m - estimate.y_m, target.x_m - estimate.x_m)
            local_goal = Waypoint(
                estimate.x_m + local_distance * math.cos(heading),
                estimate.y_m + local_distance * math.sin(heading),
                target.yaw_rad if local_distance == distance_m else None,
            )
            self._schedule_or_store_rollout(
                mission.context,
                estimate,
                world,
                local_goal,
                mission.constraints.max_v_mps,
                mission.constraints.max_omega_rad_s,
                global_goal=(mission.target_pose if mission.target_frame_id != LOCAL_FRAME_ID else None),
            )
        if disposition is _RolloutDisposition.HOLD or self._cached_plan_stale(mission.context):
            return (self._local_continuation(mission, estimate, world)
                    or self._inactive(mission, NavigationStatus.IDLE, "PLANNER_STALE_HOLD"))
        if (self._last_replan_ns is not None
                and mission.context.monotonic_ns-self._last_replan_ns >= self._max_plan_age_ns // 2):
            continuation = self._local_continuation(mission, estimate, world)
            if continuation is not None:
                return continuation
        local_goal = self._local_goal
        candidates = self._trajectory_candidates
        if local_goal is None or not candidates:
            if self._completion_inputs and self._pending_rollout_request is not None:
                continuation = self._local_continuation(mission, estimate, world)
                if continuation is not None:
                    return continuation
                return self._inactive(
                    mission, NavigationStatus.PENDING, "PLANNER_PENDING",
                    motion_validity=self._guidance_validity(mission, world),
                )
            raise RuntimeError("trajectory plan cache is empty after replanning")
        return NavigationPlan(
            context=mission.context,
            mission_id=mission.mission_id,
            route=(),
            velocity_target=None,
            constraints=mission.constraints,
            corridor_radius_m=mission.constraints.corridor_radius_m,
            progress=self._progress,
            status=NavigationStatus.ACTIVE,
            local_goal=local_goal,
            trajectory_candidates=candidates,
        )

    def _inactive(
        self,
        mission: MissionIntent,
        status: NavigationStatus,
        reason: str | None,
        *,
        motion_validity: MotionValidity | None = None,
    ) -> NavigationPlan:
        if status is NavigationStatus.INVALIDATED and reason is None:
            reason = "MISSION_INVALID"
        return NavigationPlan(
            context=mission.context,
            mission_id=mission.mission_id,
            route=(),
            velocity_target=None,
            constraints=mission.constraints,
            corridor_radius_m=0.0,
            progress=(self._progress if self._mission_id == mission.mission_id else 0.0),
            status=status,
            reason=reason,
            motion_validity=motion_validity,
        )

    @staticmethod
    def _complete_plan(mission: MissionIntent) -> NavigationPlan:
        return NavigationPlan(
            context=mission.context,
            mission_id=mission.mission_id,
            route=(),
            velocity_target=None,
            constraints=mission.constraints,
            corridor_radius_m=mission.constraints.corridor_radius_m,
            progress=1.0,
            status=NavigationStatus.COMPLETE,
        )

    def _reset(self) -> None:
        self._abandon_pending_rollout()
        self._mission_id = None
        self._person_capability_failed_mission_id = None
        self._initial_distance_m = 0.0
        self._initial_yaw_error_rad = 0.0
        self._progress = 0.0
        self._completed = False
        self._coverage.clear()
        self._coverage_last_cell = None
        self._failed_goals.clear()
        self._no_viable_since_ns = None
        self._goal_reachable = True
        self._clear_explore_progress()
        self._clear_escape_phase()
        self._local_goal = None
        self._goal_selected_ns = 0
        self._goal_selection_reason = None
        self._last_maneuver_direction = 0
        self._last_replan_ns = None
        self._last_replan_tick_id = None
        self._trajectory_candidates = ()
        self._trajectory_geometry_key = None
        self._trajectory_map_revision = None
        self._trajectory_local_sigma_m = None
        self._static_planning_index = None
        self._face_person_track_id = None
        self._face_person_aligned = False
        self._follow_person_track_id = None
        self._follow_person_lost_since_ns = None
        self._follow_person_pivoting = False
        self._follow_person_holding = False
        self._follow_person_last_heading_rad = None
        self._follow_person_state = _FollowPersonState.ACQUIRE
        self._follow_person_search_phase = None
        self._follow_person_search_target_yaw_rad = None
        self._follow_person_search_budget_ns = None
        self._follow_person_evidence = None

    def _face_person_plan(
        self,
        mission: MissionIntent,
        estimate: RobotEstimate,
        world: WorldSnapshot,
    ) -> NavigationPlan:
        if self._mission_id != mission.mission_id:
            self._reset()
            self._mission_id = mission.mission_id

        eligible = tuple(
            track
            for track in world.obstacle_tracks
            if track.track_id.startswith("person-")
            and track.confidence >= self._config.face_person_min_confidence
            and track.usable_at(mission.context.monotonic_ns)
        )
        selected = next(
            (
                track
                for track in eligible
                if track.track_id == self._face_person_track_id
            ),
            None,
        )
        if selected is None and eligible:
            selected = min(
                eligible,
                key=lambda track: (
                    -track.confidence,
                    math.hypot(track.x_m - estimate.x_m, track.y_m - estimate.y_m),
                    track.track_id,
                ),
            )
            self._face_person_track_id = selected.track_id
            self._face_person_aligned = False

        if selected is None:
            self._face_person_track_id = None
            self._face_person_aligned = False
            return self._inactive(
                mission,
                NavigationStatus.INVALIDATED,
                "PERSON_TARGET_NOT_AVAILABLE",
            )

        dx = selected.x_m - estimate.x_m
        dy = selected.y_m - estimate.y_m
        if math.hypot(dx, dy) <= 1e-9:
            desired_yaw = estimate.yaw_rad
            heading_error = 0.0
        else:
            desired_yaw = math.atan2(dy, dx)
            heading_error = _wrapped_angle(desired_yaw - estimate.yaw_rad)

        absolute_error = abs(heading_error)
        if self._face_person_aligned:
            if absolute_error > self._config.face_person_release_tolerance_rad:
                self._face_person_aligned = False
        elif absolute_error <= self._config.face_person_align_tolerance_rad:
            self._face_person_aligned = True

        target_yaw = estimate.yaw_rad if self._face_person_aligned else desired_yaw
        return NavigationPlan(
            context=mission.context,
            mission_id=mission.mission_id,
            route=(Waypoint(estimate.x_m, estimate.y_m, target_yaw),),
            velocity_target=None,
            constraints=mission.constraints,
            corridor_radius_m=0.0,
            progress=0.0,
            status=NavigationStatus.ACTIVE,
        )


    @property
    def follow_person_evidence(self) -> FollowPersonEvidence | None:
        """Return passive evidence produced by the latest FOLLOW_PERSON tick."""
        return self._follow_person_evidence

    def _follow_person_plan(
        self,
        mission: MissionIntent,
        estimate: RobotEstimate,
        world: WorldSnapshot,
    ) -> NavigationPlan:
        plan = self._follow_person_plan_impl(mission, estimate, world)
        self._follow_person_evidence = self._build_follow_person_evidence(mission, estimate, world)
        return plan

    def _build_follow_person_evidence(
        self,
        mission: MissionIntent,
        estimate: RobotEstimate,
        world: WorldSnapshot,
    ) -> FollowPersonEvidence:
        locked_uid = self._follow_person_track_id
        retention = self._config.follow_person_retention_min_confidence
        if retention is None:
            retention = self._config.follow_person_min_confidence
        selected = next(
            (
                track
                for track in world.obstacle_tracks
                if locked_uid is not None
                and track.track_id == locked_uid
                and track.confidence >= retention
            ),
            None,
        )
        target_distance_m = target_bearing_rad = target_confidence = None
        if selected is not None:
            dx = selected.x_m - estimate.x_m
            dy = selected.y_m - estimate.y_m
            target_distance_m = math.hypot(dx, dy)
            desired_yaw = estimate.yaw_rad if target_distance_m <= 1e-9 else math.atan2(dy, dx)
            target_bearing_rad = _wrapped_angle(desired_yaw - estimate.yaw_rad)
            target_confidence = selected.confidence
        missing_age_ms = None
        if self._follow_person_lost_since_ns is not None:
            missing_age_ms = max(0.0, (mission.context.monotonic_ns - self._follow_person_lost_since_ns) / 1e6)
        return FollowPersonEvidence(
            state=self._follow_person_state.value,
            locked_target_uid=locked_uid,
            target_visible=selected is not None and selected.estimate_status is TrackEstimateStatus.OBSERVED,
            target_confidence=target_confidence,
            target_distance_m=target_distance_m,
            target_bearing_rad=target_bearing_rad,
            target_missing_age_ms=missing_age_ms,
            person_candidate_count=sum(1 for track in world.obstacle_tracks if track.track_id.startswith("person-")),
            search_phase=self._follow_person_search_phase if self._follow_person_state is _FollowPersonState.SEARCH else None,
            search_target_yaw_rad=self._follow_person_search_target_yaw_rad if self._follow_person_state is _FollowPersonState.SEARCH else None,
            acquisition_min_confidence=self._config.follow_person_min_confidence,
            retention_min_confidence=retention,
            stand_off_m=self._config.follow_person_stand_off_m,
            distance_deadband_m=self._config.follow_person_distance_deadband_m,
            min_safe_distance_m=self._config.follow_person_min_safe_distance_m,
            lost_hold_ns=self._config.follow_person_lost_hold_ns,
            search_timeout_ns=self._config.follow_person_search_timeout_ns,
            search_sweep_rad=self._config.follow_person_search_sweep_rad,
            search_yaw_tolerance_rad=min(
                self._config.follow_person_search_yaw_tolerance_rad,
                mission.constraints.yaw_tolerance_rad,
            ),
            search_budget_ns=self._follow_person_search_budget_ns,
            target_estimate_status=None if selected is None else selected.estimate_status.value,
            target_measurement_age_ms=(
                None if selected is None or selected.measurement_monotonic_ns is None
                else (mission.context.monotonic_ns - selected.measurement_monotonic_ns) / 1e6
            ),
        )

    def _follow_person_plan_impl(
        self,
        mission: MissionIntent,
        estimate: RobotEstimate,
        world: WorldSnapshot,
    ) -> NavigationPlan:
        if self._mission_id != mission.mission_id:
            self._reset()
            self._mission_id = mission.mission_id

        retention_confidence = self._config.follow_person_retention_min_confidence
        if retention_confidence is None:
            retention_confidence = self._config.follow_person_min_confidence
        eligible = tuple(
            track
            for track in world.obstacle_tracks
            if track.track_id.startswith("person-")
            and (mission.target_track_id is None or track.track_id == mission.target_track_id)
            and track.usable_at(mission.context.monotonic_ns)
            and track.confidence >= (
                retention_confidence
                if track.track_id == self._follow_person_track_id
                else self._config.follow_person_min_confidence
            )
        )

        # Acquire exactly once per FOLLOW_PERSON mission. Another visible person
        # may not silently replace the locked target. A new command is re-acquire.
        if self._follow_person_track_id is None:
            if not eligible:
                self._follow_person_state = _FollowPersonState.ACQUIRE
                self._clear_trajectory_plan()
                return self._inactive(
                    mission,
                    NavigationStatus.INVALIDATED,
                    "PERSON_TARGET_NOT_AVAILABLE",
                )
            selected = min(
                eligible,
                key=lambda track: (
                    -track.confidence,
                    math.hypot(track.x_m - estimate.x_m, track.y_m - estimate.y_m),
                    track.track_id,
                ),
            )
            self._follow_person_track_id = selected.track_id
            self._follow_person_lost_since_ns = None
            self._follow_person_search_phase = None
            self._follow_person_search_target_yaw_rad = None
            self._follow_person_pivoting = False
            self._follow_person_holding = False
            self._follow_person_state = _FollowPersonState.FOLLOW
            self._clear_trajectory_plan()
        else:
            selected = next(
                (
                    track
                    for track in eligible
                    if track.track_id == self._follow_person_track_id
                ),
                None,
            )
            if selected is None:
                if self._follow_person_lost_since_ns is None:
                    self._follow_person_lost_since_ns = mission.context.monotonic_ns
                    self._follow_person_search_phase = None
                    self._follow_person_search_target_yaw_rad = None
                    self._follow_person_search_budget_ns = self._follow_search_budget(
                        mission, estimate,
                    )
                    self._clear_trajectory_plan()
                lost_ns = mission.context.monotonic_ns - self._follow_person_lost_since_ns
                if lost_ns <= self._config.follow_person_lost_hold_ns:
                    self._follow_person_state = _FollowPersonState.OCCLUDED_HOLD
                    return self._inactive(mission, NavigationStatus.IDLE, "PERSON_OCCLUDED_HOLD")

                search_elapsed_ns = lost_ns - self._config.follow_person_lost_hold_ns
                if (
                    self._follow_person_last_heading_rad is not None
                    and self._follow_person_state is not _FollowPersonState.LOST
                    and search_elapsed_ns <= (
                        self._follow_person_search_budget_ns
                        if self._follow_person_search_budget_ns is not None
                        else self._config.follow_person_search_timeout_ns
                    )
                ):
                    # Stable target yaw until the physical yaw reaches it. Overall
                    # search timeout remains the bounded failure exit.
                    self._follow_person_state = _FollowPersonState.SEARCH
                    entered_search = self._follow_person_search_phase is None
                    if entered_search:
                        # Preserve the established recovery contract: the first
                        # SEARCH output is the last observed target heading.
                        # Advancement is evaluated only on a later tick, so a
                        # robot already aligned with center cannot skip CENTER.
                        self._follow_person_search_phase = 0
                        self._follow_person_search_target_yaw_rad = (
                            self._follow_person_last_heading_rad
                        )
                    assert self._follow_person_search_target_yaw_rad is not None
                    search_yaw = self._follow_person_search_target_yaw_rad
                    search_tolerance = min(
                        self._config.follow_person_search_yaw_tolerance_rad,
                        mission.constraints.yaw_tolerance_rad,
                    )
                    if (
                        not entered_search
                        and abs(_wrapped_angle(search_yaw - estimate.yaw_rad))
                        <= search_tolerance
                    ):
                        if self._follow_person_search_phase == 2:
                            self._follow_person_state = _FollowPersonState.LOST
                            self._follow_person_search_target_yaw_rad = None
                            return self._inactive(
                                mission, NavigationStatus.INVALIDATED, "PERSON_TARGET_LOST",
                            )
                        next_phase = self._follow_person_search_phase + 1
                        search_offset_rad = (
                            self._config.follow_person_search_sweep_rad
                            if next_phase % 2
                            else -self._config.follow_person_search_sweep_rad
                        )
                        self._follow_person_search_phase = next_phase
                        search_yaw = _wrapped_angle(
                            self._follow_person_last_heading_rad + search_offset_rad
                        )
                        self._follow_person_search_target_yaw_rad = search_yaw
                    return NavigationPlan(
                        context=mission.context,
                        mission_id=mission.mission_id,
                        route=(Waypoint(estimate.x_m, estimate.y_m, search_yaw),),
                        velocity_target=None,
                        constraints=replace(mission.constraints, yaw_tolerance_rad=search_tolerance),
                        corridor_radius_m=0.0,
                        progress=0.0,
                        status=NavigationStatus.ACTIVE,
                    )

                self._follow_person_state = _FollowPersonState.LOST
                self._follow_person_search_target_yaw_rad = None
                return self._inactive(
                    mission,
                    NavigationStatus.INVALIDATED,
                    "PERSON_TARGET_LOST",
                )

        assert selected is not None
        self._follow_person_lost_since_ns = None
        self._follow_person_search_phase = None
        self._follow_person_search_target_yaw_rad = None
        self._follow_person_search_budget_ns = None
        self._follow_person_state = _FollowPersonState.FOLLOW

        dx = selected.x_m - estimate.x_m
        dy = selected.y_m - estimate.y_m
        distance_m = math.hypot(dx, dy)
        if distance_m <= 1e-9:
            desired_yaw = estimate.yaw_rad
            heading_error = 0.0
        else:
            desired_yaw = math.atan2(dy, dx)
            heading_error = _wrapped_angle(desired_yaw - estimate.yaw_rad)
        self._follow_person_last_heading_rad = desired_yaw

        if distance_m <= self._config.follow_person_min_safe_distance_m:
            self._follow_person_pivoting = False
            self._follow_person_holding = True
            self._clear_trajectory_plan()
            return self._inactive(
                mission,
                NavigationStatus.INVALIDATED,
                "PERSON_TOO_CLOSE",
            )

        absolute_error = abs(heading_error)
        if self._follow_person_pivoting:
            if absolute_error <= self._config.follow_person_release_tolerance_rad:
                self._follow_person_pivoting = False
        elif absolute_error >= self._config.follow_person_pivot_enter_rad:
            self._follow_person_pivoting = True

        # Large errors still pivot in place. Medium errors stay in the rollout,
        # so the robot bends toward the person instead of stop-turn-start.
        if self._follow_person_pivoting:
            self._clear_trajectory_plan()
            return NavigationPlan(
                context=mission.context,
                mission_id=mission.mission_id,
                route=(Waypoint(estimate.x_m, estimate.y_m, desired_yaw),),
                velocity_target=None,
                constraints=mission.constraints,
                corridor_radius_m=0.0,
                progress=0.0,
                status=NavigationStatus.ACTIVE,
            )

        hold_enter_m = (
            self._config.follow_person_stand_off_m
            + self._config.follow_person_distance_deadband_m
        )
        hold_release_m = hold_enter_m + self._config.follow_person_hold_release_margin_m
        if self._follow_person_holding:
            if distance_m >= hold_release_m:
                self._follow_person_holding = False
        elif distance_m <= hold_enter_m:
            self._follow_person_holding = True

        if self._follow_person_holding:
            self._clear_trajectory_plan()
            # Stand-off arrival is ordinary zero guidance, not authority
            # revocation. Keep it ACTIVE so L7 retains temporal ownership and
            # L8/L9 can perform bounded deceleration/restart. Too-close,
            # missing-target and safety paths remain fail-closed.
            return NavigationPlan(
                context=mission.context,
                mission_id=mission.mission_id,
                route=(Waypoint(estimate.x_m, estimate.y_m, estimate.yaw_rad),),
                velocity_target=None,
                constraints=mission.constraints,
                corridor_radius_m=0.0,
                progress=0.0,
                status=NavigationStatus.ACTIVE,
                reason="PERSON_DISTANCE_HOLD",
            )

        costmap = world.local_costmap
        if costmap is None:
            self._clear_trajectory_plan()
            return self._inactive(
                mission,
                NavigationStatus.INVALIDATED,
                "LOCAL_COSTMAP_MISSING",
            )
        if costmap.freshness_ns > self._config.max_costmap_freshness_ns:
            self._clear_trajectory_plan()
            return self._inactive(
                mission,
                NavigationStatus.INVALIDATED,
                "LOCAL_COSTMAP_STALE",
            )

        # Steering slowdown must retain a realizable planner speed inside the
        # mission cap. Distance braking still tends continuously to zero at HOLD.
        max_follow_v_mps = mission.constraints.max_v_mps
        minimum_follow_v_mps = min(
            max(
                self._config.follow_person_minimum_follow_speed_mps,
                self._config.minimum_planning_speed_mps,
                self._config.wheel_limits.target_floor_mps,
            ),
            max_follow_v_mps,
        )

        distance_ratio = min(
            1.0,
            max(
                0.0,
                (distance_m - hold_enter_m)
                / self._config.follow_person_slowdown_distance_m,
            ),
        )
        distance_cap_mps = max_follow_v_mps * distance_ratio

        if absolute_error <= self._config.follow_person_align_tolerance_rad:
            heading_factor = 1.0
        else:
            heading_span = max(
                self._config.follow_person_pivot_enter_rad
                - self._config.follow_person_align_tolerance_rad,
                1e-9,
            )
            heading_ratio = min(
                1.0,
                max(
                    0.0,
                    (
                        absolute_error
                        - self._config.follow_person_align_tolerance_rad
                    )
                    / heading_span,
                ),
            )
            heading_factor = 1.0 - (
                1.0 - self._config.follow_person_heading_min_factor
            ) * heading_ratio

        heading_cap_mps = max_follow_v_mps * heading_factor
        follow_max_v_mps = min(
            distance_cap_mps,
            max(minimum_follow_v_mps, heading_cap_mps),
        )

        disposition = self._accept_pending_rollout(mission.context)
        if self._replan_due(
            mission.context.monotonic_ns,
            mission.context.tick_id,
        ):
            travel_m = min(
                distance_m - hold_enter_m,
                self._config.local_goal_distance_m,
            )
            local_goal = Waypoint(
                estimate.x_m + travel_m * math.cos(desired_yaw),
                estimate.y_m + travel_m * math.sin(desired_yaw),
            )
            self._schedule_or_store_rollout(
                mission.context,
                estimate,
                world,
                local_goal,
                follow_max_v_mps,
                mission.constraints.max_omega_rad_s,
            )

        if disposition is _RolloutDisposition.HOLD or self._cached_plan_stale(mission.context):
            return (self._local_continuation(mission, estimate, world, max_v_mps=follow_max_v_mps)
                    or self._inactive(mission, NavigationStatus.IDLE, "PLANNER_STALE_HOLD"))
        if (self._last_replan_ns is not None
                and mission.context.monotonic_ns-self._last_replan_ns >= self._max_plan_age_ns // 2):
            continuation = self._local_continuation(mission, estimate, world, max_v_mps=follow_max_v_mps)
            if continuation is not None:
                return continuation
        local_goal = self._local_goal
        candidates = self._trajectory_candidates
        if local_goal is None or not candidates:
            if self._completion_inputs and self._pending_rollout_request is not None:
                continuation = self._local_continuation(mission, estimate, world, max_v_mps=follow_max_v_mps)
                if continuation is not None:
                    return continuation
                return self._inactive(
                    replace(mission, constraints=replace(mission.constraints, max_v_mps=follow_max_v_mps)),
                    NavigationStatus.PENDING, "PLANNER_PENDING",
                    motion_validity=self._guidance_validity(mission, world),
                )
            raise RuntimeError("follow-person trajectory cache is empty after replanning")
        # FOLLOW already chose translation over the explicit PIVOT behavior.
        # Keep L7's choice within that intent when a safe, productive forward
        # path exists. Preserve recovery/pivot options when none exists.
        if any(
            candidate.v_mps > _MOTION_EPSILON
            and candidate.progress_score > 0.0
            and candidate.progress_viable
            and not candidate.collision
            for candidate in candidates
        ):
            candidates = tuple(
                candidate for candidate in candidates
                if abs(candidate.v_mps) > _MOTION_EPSILON
            )
        return NavigationPlan(
            context=mission.context,
            mission_id=mission.mission_id,
            route=(),
            velocity_target=None,
            # Reapply the current arrival envelope even while a fresh cached
            # trajectory or async completion still carries an older speed.
            constraints=replace(mission.constraints, max_v_mps=follow_max_v_mps),
            corridor_radius_m=mission.constraints.corridor_radius_m,
            progress=0.0,
            status=NavigationStatus.ACTIVE,
            local_goal=local_goal,
            trajectory_candidates=candidates,
        )

    def _follow_search_budget(
        self, mission: MissionIntent, estimate: RobotEstimate,
    ) -> int:
        heading = self._follow_person_last_heading_rad
        if heading is None:
            return self._config.follow_person_search_timeout_ns
        # CENTER -> +SWEEP -> -SWEEP. Use wrapped angular distances, including
        # sweeps larger than pi/2, and the actual command's angular-speed cap.
        sweep = self._config.follow_person_search_sweep_rad
        travel_rad = (
            abs(_wrapped_angle(heading - estimate.yaw_rad))
            + sweep + abs(_wrapped_angle(2 * sweep))
        )
        cap = self._config.follow_person_search_max_duration_ns
        travel_s = min(cap / 1e9, travel_rad / mission.constraints.max_omega_rad_s)
        return min(cap, math.ceil(travel_s * 1e9) + self._config.follow_person_search_timeout_ns)

    def _exploration_plan(
        self,
        mission: MissionIntent,
        estimate: RobotEstimate,
        world: WorldSnapshot,
    ) -> NavigationPlan:
        costmap = world.local_costmap
        if costmap is None:
            self._clear_trajectory_plan()
            return self._inactive(
                mission,
                NavigationStatus.INVALIDATED,
                "LOCAL_COSTMAP_MISSING",
            )
        if costmap.freshness_ns > self._config.max_costmap_freshness_ns:
            self._clear_trajectory_plan()
            return self._inactive(
                mission,
                NavigationStatus.INVALIDATED,
                "LOCAL_COSTMAP_STALE",
            )
        if self._mission_id != mission.mission_id:
            self._reset()
            self._mission_id = mission.mission_id

        disposition = self._accept_pending_rollout(mission.context)
        self._mark_coverage(estimate.x_m, estimate.y_m, mission.context.tick_id)
        self._progress = min(1.0, len(self._coverage) / self._config.coverage_max_cells)
        now = mission.context.monotonic_ns
        self._failed_goals = {cell: until for cell, until in self._failed_goals.items() if until > now}
        no_progress = self._explore_no_progress_plan(mission, estimate, world)
        if no_progress is not None:
            return no_progress
        goal = self._local_goal
        if self._replan_due(
            mission.context.monotonic_ns,
            mission.context.tick_id,
        ):
            scene: _LocalPlanningScene | None = None
            goal_selected_ns: int | None = None
            approach_speed = 0.0
            if goal is not None:
                bearing = math.atan2(goal.y_m - estimate.y_m, goal.x_m - estimate.x_m)
                approach_speed = max(0.0, estimate.v_mps * math.cos(bearing - estimate.yaw_rad))
            goal_reason = None
            alternate_goals: tuple[Waypoint, ...] = ()
            if goal is None:
                goal_reason = ("NO_PROGRESS_RESELECT" if self._explore_stall_count else "INITIAL_GOAL")
            elif not self._goal_reachable or (self._trajectory_candidates and not any(
                    not c.collision and c.progress_viable for c in self._trajectory_candidates)):
                if self._no_viable_since_ns is None:
                    self._no_viable_since_ns = now
                if now - self._no_viable_since_ns >= 2 * self._config.trajectory_replan_interval_ns:
                    self._remember_failed_goal(goal, now)
                    goal_reason = "NO_VIABLE_PATH"
            elif mission.context.monotonic_ns - self._goal_selected_ns >= self._config.local_goal_max_age_ns:
                goal_reason = "GOAL_MAX_AGE"
            elif math.hypot(goal.x_m-estimate.x_m, goal.y_m-estimate.y_m) <= (
                    self._config.local_goal_tolerance_m + approach_speed*self._goal_handoff_ns()/1e9):
                goal_reason = "GOAL_HANDOFF"
            if goal_reason is not None:
                # Collision lookahead is not the waypoint replacement lead.
                # Keep an approaching, viable goal until its bounded handoff.
                # The worker ranks pure local options. Until acceptance, retain
                # the committed endpoint. A first seed only supplies a proposal
                # for the independent bounded continuation proof.
                distance = (mission.explore_preferences or self._config).explore_local_goal_max_distance_m
                goal = goal or Waypoint(estimate.x_m + distance * math.cos(estimate.yaw_rad),
                                        estimate.y_m + distance * math.sin(estimate.yaw_rad))
                goal_selected_ns = mission.context.monotonic_ns
            assert goal is not None
            self._schedule_or_store_rollout(
                mission.context,
                estimate,
                world,
                goal,
                mission.constraints.max_v_mps,
                mission.constraints.max_omega_rad_s,
                scene=scene,
                goal_selected_ns=goal_selected_ns,
                goal_selection_reason=goal_reason,
                allow_exploration_detour=True,
                alternate_goals=alternate_goals,
                select_local_goal=goal_reason is not None,
                explore_preferences=mission.explore_preferences,
            )
        if disposition is _RolloutDisposition.HOLD or self._cached_plan_stale(mission.context):
            return (self._local_continuation(mission, estimate, world)
                    or self._inactive(mission, NavigationStatus.IDLE, "PLANNER_STALE_HOLD"))
        if (self._last_replan_ns is not None
                and mission.context.monotonic_ns-self._last_replan_ns >= self._max_plan_age_ns // 2):
            continuation = self._local_continuation(mission, estimate, world)
            if continuation is not None:
                return continuation
        goal = self._local_goal
        candidates = self._trajectory_candidates
        if goal is None or not candidates:
            if not self._goal_reachable:
                return self._inactive(mission, NavigationStatus.IDLE, "NO_PATH_HOLD")
            if self._completion_inputs and self._pending_rollout_request is not None:
                continuation = self._local_continuation(mission, estimate, world)
                if continuation is not None:
                    return continuation
                return self._inactive(
                    mission, NavigationStatus.PENDING, "PLANNER_PENDING",
                    motion_validity=self._guidance_validity(mission, world),
                )
            raise RuntimeError("trajectory plan cache is empty after replanning")
        if not self._goal_reachable and not any(
                not c.collision and c.progress_viable for c in candidates):
            return self._inactive(mission, NavigationStatus.IDLE, "NO_PATH_HOLD")
        return NavigationPlan(
            context=mission.context,
            mission_id=mission.mission_id,
            route=(),
            velocity_target=None,
            constraints=mission.constraints,
            corridor_radius_m=0.0,
            progress=self._progress,
            status=NavigationStatus.ACTIVE,
            local_goal=goal,
            trajectory_candidates=candidates,
        )

    def _clear_explore_progress(self) -> None:
        self._explore_progress_origin = None
        self._explore_progress_started_ns = None
        self._explore_stall_count = 0
        self._explore_retry_after_ns = None
        self._explore_reachable_exit_mask = 0
        self._explore_exit_retry_used = False
        self._explore_probe_after_ns = None

    def _observe_explore_progress(self, estimate: RobotEstimate) -> None:
        """Credit measured local translation, never a goal or planner refresh."""
        quality = estimate.localization_quality
        if (quality.local_translation is QualityState.LOST
                or not quality.local_pose_continuous or quality.pose_discontinuity):
            return
        origin = self._explore_progress_origin
        if (origin is None or math.hypot(estimate.x_m-origin[0], estimate.y_m-origin[1])
                >= self._config.local_goal_tolerance_m):
            if self._explore_retry_after_ns is not None:
                # Measured motion supersedes an exhausted stationary probe;
                # its old pose/result cannot become the next active cache.
                self._clear_trajectory_plan()
                self._local_goal = None
                self._goal_reachable = True
            self._explore_progress_origin = (estimate.x_m, estimate.y_m)
            self._explore_progress_started_ns = estimate.context.monotonic_ns
            self._explore_stall_count = 0
            self._explore_retry_after_ns = None
            self._no_viable_since_ns = None
            self._explore_reachable_exit_mask = 0
            self._explore_exit_retry_used = False
            self._explore_probe_after_ns = None

    def _explore_no_progress_plan(
        self, mission: MissionIntent, estimate: RobotEstimate, world: WorldSnapshot,
    ) -> NavigationPlan | None:
        # Candidate viability predicts a path; only local odometry proves that
        # a repeatedly published path actually moves. Handoffs and fresh worker
        # results cannot renew this window. A committed escape has its own
        # measured completion/2*horizon budget and bypasses this expiry check.
        self._observe_explore_progress(estimate)
        now = mission.context.monotonic_ns
        retry_after = self._explore_retry_after_ns
        if retry_after is not None:
            prefix = "NO_PATH" if self._goal_selection_reason == "NO_PATH" else "NO_PROGRESS"
            if self._explore_stall_count >= _FAILED_LOCAL_GOAL_CAPACITY:
                # Exhaustion revokes motion, but a slow pure worker probe can
                # discover a genuinely new exit. Revisions, ranking changes
                # and expired failed-goal cells alone cannot arm another try.
                probe_after = self._explore_probe_after_ns
                if probe_after is None:
                    self._explore_probe_after_ns = now + self._config.local_goal_max_age_ns
                elif (not self._explore_exit_retry_used and now >= probe_after
                        and self._pending_rollout_request is None):
                    self._explore_probe_after_ns = now + self._config.local_goal_max_age_ns
                    distance = (mission.explore_preferences or self._config).explore_local_goal_max_distance_m
                    seed = Waypoint(estimate.x_m + distance * math.cos(estimate.yaw_rad),
                                    estimate.y_m + distance * math.sin(estimate.yaw_rad))
                    self._schedule_or_store_rollout(
                        mission.context, estimate, world, seed,
                        mission.constraints.max_v_mps, mission.constraints.max_omega_rad_s,
                        goal_selected_ns=now, goal_selection_reason="EXHAUSTED_EXIT_PROBE",
                        allow_exploration_detour=True, select_local_goal=True,
                        explore_preferences=mission.explore_preferences,
                    )
                    if self._explore_retry_after_ns is None:
                        return None
                return self._inactive(mission, NavigationStatus.IDLE, prefix + "_EXHAUSTED_HOLD")
            if now < retry_after:
                return self._inactive(mission, NavigationStatus.IDLE, prefix + "_HOLD")
            self._explore_retry_after_ns = None
            self._explore_progress_started_ns = now
            self._local_goal = None
            self._goal_reachable = True
            return None
        no_path = not self._goal_reachable
        if no_path and self._no_viable_since_ns is None:
            self._no_viable_since_ns = now
        started = self._explore_progress_started_ns
        # Neither a no-path result nor a momentarily viable replacement is
        # physical progress. They cannot renew either bounded episode.
        blocked_since = self._no_viable_since_ns if no_path else None
        elapsed = max(now-started if started is not None else 0,
                      now-blocked_since if blocked_since is not None else 0)
        if elapsed < self._config.local_goal_max_age_ns:
            return None
        goal = self._local_goal
        if goal is None and self._pending_rollout_request is not None:
            goal = self._pending_rollout_request.goal
        if goal is not None:
            self._remember_failed_goal(goal, now)
        self._clear_trajectory_plan()
        self._goal_reachable = False
        self._goal_selection_reason = "NO_PATH" if no_path else "NO_PROGRESS"
        self._explore_stall_count += 1
        # Reuse the existing no-viable hysteresis and failed-goal capacity.
        # One new footprint-proven exit may fund one extra bounded episode.
        # Only measured translation/new lineage resets the overall budget.
        self._explore_retry_after_ns = now + 2*self._config.trajectory_replan_interval_ns
        if self._explore_stall_count >= _FAILED_LOCAL_GOAL_CAPACITY:
            self._explore_probe_after_ns = now + self._config.local_goal_max_age_ns
        prefix = "NO_PATH" if no_path else "NO_PROGRESS"
        reason = prefix + ("_EXHAUSTED_HOLD"
                           if self._explore_stall_count >= _FAILED_LOCAL_GOAL_CAPACITY else "_HOLD")
        return self._inactive(mission, NavigationStatus.IDLE, reason)

    def _local_continuation(
        self, mission: MissionIntent, estimate: RobotEstimate, world: WorldSnapshot,
        *, max_v_mps: float | None = None,
    ) -> NavigationPlan | None:
        """Prove at most four short proposals against this tick's geometry.

        A stale rollout is only a proposal here. It supplies no actuation
        authority: the complete footprint is tested anew, from the current
        odometry pose. This bounded family check cannot run a search
        or manufacture a new global goal while the worker is pending.
        """
        quality = estimate.localization_quality
        costmap = world.local_costmap
        if (self._mission_id != mission.mission_id or costmap is None
                or quality.local_translation is QualityState.LOST
                or quality.heading is QualityState.LOST
                or not quality.local_pose_continuous or quality.pose_discontinuity
                or world.freshness_ns > self._config.max_world_freshness_ns
                or costmap.freshness_ns > self._config.max_costmap_freshness_ns):
            self._clear_escape_phase()
            return None
        goal = self._escape_goal or self._local_goal
        if self._escape_candidate is None and not self._goal_reachable:
            return None
        if (self._escape_candidate is None and self._pending_rollout_request is not None
                and goal is None):
            # An explicit seed still needs an independent fresh local proof.
            # Search proposals cannot replace a committed endpoint here.
            goal = self._pending_rollout_request.goal
            if (mission.mode is CommandMode.EXPLORE
                    and self._failed_goals.get(self._coverage_key(goal.x_m, goal.y_m), 0)
                    > mission.context.monotonic_ns):
                return None
        if goal is None:
            return None
        distance = math.hypot(goal.x_m-estimate.x_m, goal.y_m-estimate.y_m)
        # A resolvable NAVIGATE target is closed-loop: _evaluate checks its
        # actual arrival on every tick. The rolling waypoint tolerance must
        # not strand that mission before its tighter completion tolerance.
        # Without a global fix only the committed local segment is available;
        # retain its original endpoint bounds until the target can be resolved.
        navigating_to_target = (mission.mode is CommandMode.NAVIGATE
            and self._escape_candidate is None
            and (mission.target_frame_id == LOCAL_FRAME_ID
                 or quality.global_position is QualityState.GOOD))
        arrival_tolerance = (min(self._config.local_goal_tolerance_m,
                                 mission.constraints.goal_tolerance_m)
                             if navigating_to_target else self._config.local_goal_tolerance_m)
        if distance <= arrival_tolerance:
            return None
        v_cap = min(mission.constraints.max_v_mps,
                    mission.constraints.max_v_mps if max_v_mps is None else max_v_mps)
        omega_cap = mission.constraints.max_omega_rad_s
        if quality.local_translation is QualityState.DEGRADED or quality.heading is QualityState.DEGRADED:
            v_cap = self._config.wheel_limits.degraded_linear_cap(
                v_cap, omega_cap, self._config.localization_degraded_speed_scale)
            omega_cap = self._config.wheel_limits.degraded_angular_cap(
                omega_cap, self._config.localization_degraded_speed_scale)
        if v_cap < max(self._config.wheel_limits.minimum_mps, self._config.minimum_planning_speed_mps):
            return None
        proposals = tuple(c for c in self._trajectory_candidates
                          if not c.collision and c.progress_viable
                          and (c.v_mps > 0
                               or (c.v_mps < 0 and c.candidate_id.startswith("escape-"))
                               or (abs(c.v_mps) <= _MOTION_EPSILON
                                               and abs(c.omega_rad_s) > _MOTION_EPSILON)))
        if self._escape_candidate is not None:
            proposals = (self._escape_candidate,)
        elif proposals:
            # Keep direction and speed alternatives within the four-proposal
            # proof budget. A faster nominal rollout can need a wider execution
            # sweep; it must not discard an existing slower straight proposal.
            # L7 retains the choice among the freshly proved alternatives.
            ranked = sorted(proposals, key=lambda c: (-c.total_score, c.candidate_id))
            family = [ranked[0]]
            fallback = min(ranked, key=lambda c: (abs(c.omega_rad_s), abs(c.v_mps),
                                                 -c.total_score, c.candidate_id))
            if (fallback.v_mps, fallback.omega_rad_s) != (family[0].v_mps, family[0].omega_rad_s):
                family.append(fallback)
            for direction in (-1, 0, 1):
                if len(family) == 4:
                    break
                alternative = next((c for c in ranked
                    if (0 if abs(c.omega_rad_s) <= _MOTION_EPSILON
                        else (1 if c.omega_rad_s > 0.0 else -1)) == direction), None)
                if alternative is not None and all(
                        (alternative.v_mps, alternative.omega_rad_s) != (c.v_mps, c.omega_rad_s)
                        for c in family):
                    family.append(alternative)
            for c in ranked[1:]:
                if len(family) == 4:
                    break
                if all((c.v_mps, c.omega_rad_s) != (p.v_mps, p.omega_rad_s) for p in family):
                    family.append(c)
            proposals = tuple(family)
        else:
            proposals = (None,)
        scene = replace(self._build_planning_scene(world),
                        uncertainty_m=self._config.localization_inflation_sigma * quality.local_sigma_m)
        clearance = _footprint_clearance(estimate.x_m, estimate.y_m, estimate.yaw_rad,
                                         world, self._config, scene)
        checked = []
        witness = None
        for proposal in proposals:
            if proposal is not None:
                v, omega = proposal.v_mps, proposal.omega_rad_s
            else:
                heading_error = _wrapped_angle(math.atan2(goal.y_m-estimate.y_m,
                                                           goal.x_m-estimate.x_m)-estimate.yaw_rad)
                v = 0.0 if abs(heading_error) > math.pi / 4 else v_cap
                omega = (math.copysign(omega_cap, heading_error) if v == 0.0
                         else max(-omega_cap, min(omega_cap, heading_error)))
            v, omega = self._config.wheel_limits.constrain(v, omega,
                max_v_mps=min(v_cap, self._config.operational_max_v_mps),
                max_omega_rad_s=min(omega_cap, self._config.operational_max_omega_rad_s),
                max_curvature_rad_per_m=self._config.operational_max_curvature_rad_per_m)
            if abs(v) <= _MOTION_EPSILON and abs(omega) <= _MOTION_EPSILON:
                continue
            candidate = self._evaluate_trajectory(0, 0, v, omega, estimate, world, scene,
                                                 goal, v_cap, omega_cap, clearance, planning=False)
            candidate = replace(candidate, candidate_id=(proposal.candidate_id if proposal else "local-continuation"))
            if candidate.collision:
                if witness is None and candidate.rejection_witness is not None:
                    witness = replace(candidate.rejection_witness, candidate_id=candidate.candidate_id)
                continue
            if proposal is not None and proposal.candidate_id.startswith("escape-"):
                if self._escape_candidate is not None:
                    candidate = replace(candidate,
                        progress_potential_score=proposal.progress_potential_score,
                        progress_viable=True, total_score=(
                            self._config.progress_weight * proposal.progress_potential_score
                            + self._config.clearance_weight * candidate.min_clearance_m / self._config.clearance_score_cap_m
                            + self._config.smoothness_weight * candidate.smoothness_score
                            + self._config.novelty_weight * candidate.novelty_score))
                else:
                    candidate = _as_escape_candidate(
                        candidate, 0, 0, clearance, goal,
                        min(self._config.local_escape_reverse_max_v_mps, v_cap),
                        world, self._config, scene,
                    )
                    candidate = replace(candidate, candidate_id=proposal.candidate_id)
            elif not candidate.progress_viable and mission.mode is CommandMode.EXPLORE:
                candidate = _exploration_detours(
                    [candidate], estimate, world, scene, self._config, v_cap, clearance)[0]
            if not candidate.progress_viable:
                continue
            if (not navigating_to_target and mission.mode is not CommandMode.EXPLORE
                    and candidate.v_mps * candidate.horizon_ns / 1e9 > distance):
                continue
            checked.append(candidate)
        if not checked:
            if witness is not None:
                self._clear_escape_phase()
                return replace(self._inactive(mission, NavigationStatus.IDLE, "PLANNER_GEOMETRY_HOLD"),
                               rejection_witness=witness)
            return None
        validity = self._guidance_validity(mission, world)
        until = min(validity.valid_until_ns,
                    mission.context.monotonic_ns + self._config.max_world_freshness_ns-world.freshness_ns,
                    mission.context.monotonic_ns + self._config.max_costmap_freshness_ns-costmap.freshness_ns)
        validity = replace(validity, valid_until_ns=until,
                           localization_requirement=LocalizationRequirement(True, True, False),
                           geometry_revision=costmap.revision,
                           geometry_captured_ns=mission.context.monotonic_ns-costmap.freshness_ns)
        return NavigationPlan(
            mission.context, mission.mission_id, (), None,
            replace(mission.constraints, max_v_mps=min(mission.constraints.max_v_mps,
                mission.constraints.max_v_mps if max_v_mps is None else max_v_mps)),
            0.0, self._progress, NavigationStatus.ACTIVE, "LOCAL_GUIDANCE_REVALIDATED",
            goal, tuple(checked), validity,
        )

    def _active_escape_plan(self, mission, estimate, world) -> NavigationPlan | None:
        candidate, origin = self._escape_candidate, self._escape_origin
        if candidate is None or origin is None or self._escape_started_ns is None:
            return None
        horizon_s = candidate.horizon_ns / 1e9
        distance = math.hypot(estimate.x_m-origin[0], estimate.y_m-origin[1])
        turn = abs(_wrapped_angle(estimate.yaw_rad-origin[2]))
        complete = (distance >= abs(candidate.v_mps)*horizon_s
                    if abs(candidate.v_mps) > _MOTION_EPSILON
                    else turn >= min(math.pi, abs(candidate.omega_rad_s)*horizon_s))
        if complete:
            prepared = (mission.mode is CommandMode.EXPLORE
                        and self._local_goal is not None
                        and self._local_goal != self._escape_goal
                        and self._goal_reachable
                        and not self._cached_plan_stale(mission.context)
                        and any(not c.collision and c.progress_viable
                                and not c.candidate_id.startswith("escape-")
                                for c in self._trajectory_candidates))
            self._clear_escape_phase()
            if not prepared:
                if mission.mode is CommandMode.EXPLORE:
                    # The blocked endpoint cannot survive its recovery. A
                    # pending search already uses the latest recovery pose;
                    # let it complete without promoting the old endpoint.
                    self._local_goal = None
                    self._goal_reachable = True
                    self._goal_selection_reason = "RECOVERY_RESELECT"
                self._trajectory_candidates = ()
            return None
        if mission.context.monotonic_ns-self._escape_started_ns >= 2*candidate.horizon_ns:
            if self._escape_goal is not None:
                self._remember_failed_goal(self._escape_goal, mission.context.monotonic_ns)
            self._clear_escape_phase()
            self._trajectory_candidates = tuple(replace(c, progress_viable=False)
                                                 for c in self._trajectory_candidates)
            return self._inactive(mission, NavigationStatus.IDLE, "RECOVERY_EXHAUSTED_HOLD")
        if (mission.mode is CommandMode.EXPLORE
                and self._replan_due(mission.context.monotonic_ns, mission.context.tick_id)):
            # Pure next-exit search overlaps the owned maneuver. It cannot
            # change that maneuver's direction, endpoint or completion budget.
            self._schedule_or_store_rollout(
                mission.context, estimate, world, self._escape_goal,
                mission.constraints.max_v_mps, mission.constraints.max_omega_rad_s,
                goal_selected_ns=mission.context.monotonic_ns,
                goal_selection_reason="RECOVERY_RESELECT", allow_exploration_detour=True,
                select_local_goal=True, explore_preferences=mission.explore_preferences)
        return (self._local_continuation(mission, estimate, world)
                or self._inactive(mission, NavigationStatus.IDLE, "RECOVERY_GEOMETRY_HOLD"))

    def _clear_escape_phase(self) -> None:
        self._escape_candidate = self._escape_goal = None
        self._escape_started_ns = self._escape_origin = None

    def _remember_failed_goal(self, goal: Waypoint, now: int) -> None:
        self._failed_goals[self._coverage_key(goal.x_m, goal.y_m)] = now + self._config.local_goal_max_age_ns
        if len(self._failed_goals) > _FAILED_LOCAL_GOAL_CAPACITY:
            del self._failed_goals[min(self._failed_goals, key=lambda cell: (self._failed_goals[cell], cell))]

    def _build_planning_scene(self, world: WorldSnapshot) -> _LocalPlanningScene:
        costmap = world.local_costmap
        static_index: _StaticPlanningIndex | None = None
        if costmap is not None:
            cache_key = _costmap_cache_key(costmap)
            cached = self._static_planning_index
            if cached is None or cached.cache_key != cache_key:
                cached = _build_static_planning_index(costmap, self._config.planning_bucket_size_m)
                self._static_planning_index = cached
            static_index = cached
        dynamic_obstacles = tuple(
            _ObstacleDisc(obstacle.x_m, obstacle.y_m, obstacle.radius_m)
            for obstacle in world.obstacle_tracks
            if obstacle.confidence >= self._config.obstacle_confidence_floor
        )
        return _LocalPlanningScene(static_index, dynamic_obstacles)

    def _replan_due(self, monotonic_ns: int, tick_id: int) -> bool:
        if self._pending_rollout_id is not None:
            pending = self._pending_rollout_request
            if not self._completion_inputs or pending is None or not source_is_stale(
                monotonic_ns, pending.context.monotonic_ns, self._max_plan_age_ns
            ):
                return False
            self._abandon_pending_rollout()
            return True
        previous_ns = self._last_replan_ns
        previous_tick_id = self._last_replan_tick_id
        if (
            previous_ns is None
            or previous_tick_id is None
            or (not self._trajectory_candidates and self._goal_reachable)
        ):
            return True
        if monotonic_ns - previous_ns < self._config.trajectory_replan_interval_ns:
            return False
        # New async production timing is monotonic-time based. A pending request
        # already bounds work to one rollout, so a second tick-count gate would
        # only turn runtime jitter into planner latency. Legacy tick-mode replay
        # and the synchronous planner retain the historic cooldown.
        if self._completion_inputs or (self._rollout_backend is not None and self._rollout_release_delay_ns is not None):
            return True
        return (
            tick_id - previous_tick_id
            >= self._config.trajectory_replan_min_tick_gap
        )

    def _schedule_or_store_rollout(
        self,
        context: TickContext,
        estimate: RobotEstimate,
        world: WorldSnapshot,
        goal: Waypoint,
        max_v_mps: float,
        max_omega_rad_s: float,
        *,
        scene: _LocalPlanningScene | None = None,
        goal_selected_ns: int | None = None,
        goal_selection_reason: str | None = None,
        allow_exploration_detour: bool = False,
        alternate_goals: tuple[Waypoint, ...] = (),
        select_local_goal: bool = False,
        explore_preferences: ExplorePreferences | None = None,
        global_goal: Waypoint | None = None,
    ) -> None:
        if estimate.localization_quality.local_translation is QualityState.DEGRADED or estimate.localization_quality.heading is QualityState.DEGRADED:
            max_v_mps = self._config.wheel_limits.degraded_linear_cap(
                max_v_mps, max_omega_rad_s, self._config.localization_degraded_speed_scale,
            )
            max_omega_rad_s = self._config.wheel_limits.degraded_angular_cap(
                max_omega_rad_s, self._config.localization_degraded_speed_scale)
        request = TrajectoryRolloutRequest(
            context=context, estimate=estimate, world=world, goal=goal,
            max_v_mps=max_v_mps, max_omega_rad_s=max_omega_rad_s,
            coverage=tuple((x, y, visits) for (x, y), (visits, _tick) in sorted(self._coverage.items())),
            allow_exploration_detour=allow_exploration_detour, alternate_goals=alternate_goals,
            select_local_goal=select_local_goal, explore_preferences=explore_preferences,
            failed_goals=tuple((x, y, until) for (x, y), until in sorted(self._failed_goals.items())),
            goal_handoff_ns=self._goal_handoff_ns(),
            excluded_exit_mask=((self._explore_reachable_exit_mask or 0)
                                if goal_selection_reason == "EXHAUSTED_EXIT_PROBE" else 0),
            global_goal=global_goal,
        )
        backend = self._rollout_backend
        # Direct/replay mode computes the identical kernel synchronously.
        # Production completion-input mode only closes this immutable request;
        # its initial continuation gets a separate bounded current proof.
        if not self._completion_inputs and (backend is None or not self._trajectory_candidates):
            planning_scene = scene or self._build_planning_scene(world)
            result = _compute_goal_rollout(self._config, planning_scene, request)
            self._store_rollout_result(request, result, goal_selected_ns, goal_selection_reason)
            return
        if self._pending_rollout_id is not None:
            raise RuntimeError("async trajectory rollout is already pending")
        self._pending_goal_selection_reason = goal_selection_reason
        if self._completion_inputs:
            self._pending_rollout_id = context.tick_id + 1
            self._pending_rollout_request = request
            self._pending_goal_selected_ns = goal_selected_ns
            return
        request_id = backend.submit(request)
        if not isinstance(request_id, int) or isinstance(request_id, bool) or request_id <= 0:
            raise RuntimeError("async rollout backend returned invalid request id")
        self._pending_rollout_id = request_id
        self._pending_rollout_request = request
        self._pending_goal_selected_ns = goal_selected_ns
        if self._rollout_release_delay_ns is None:
            # Legacy capture/replay mode: preserve the historical tick gate.
            self._pending_release_tick_id = (
                context.tick_id + self._rollout_release_tick_gap
            )
            self._pending_release_not_before_ns = None
        else:
            # Production mode: one canonical monotonic-time authority. The
            # first closed tick at/after this threshold owns the handoff.
            self._pending_release_tick_id = None
            self._pending_release_not_before_ns = (
                context.monotonic_ns + self._rollout_release_delay_ns
            )

    @property
    def pending_rollout_request(self) -> TrajectoryRolloutRequest | None:
        return self._pending_rollout_request

    def _accept_closed_rollout(
        self,
        context: TickContext,
    ) -> _RolloutDisposition:
        request = self._pending_rollout_request
        if request is None:
            return _RolloutDisposition.HOLD if self._cached_plan_stale(context) else _RolloutDisposition.NONE

        event = self._closed_completion
        if (
            event is None
            or event.request_context is None
            or event.request_context.tick_id < request.context.tick_id
        ):
            if self._trajectory_candidates and self._cached_plan_stale(context):
                return _RolloutDisposition.HOLD
            return _RolloutDisposition.NONE

        if event.request_context != request.context:
            raise RuntimeError("ASYNC_L6_SOURCE_CONTEXT_MISMATCH")
        if event.error is not None:
            if event.error == "ASYNC_L6_DEADLINE_MISSED":
                # Failure of the replacement does not revoke an independently
                # valid previous trajectory. Its original expiry still applies.
                self._abandon_pending_rollout()
                if self._trajectory_candidates and not self._cached_plan_stale(context):
                    return _RolloutDisposition.NONE
                return _RolloutDisposition.HOLD
            raise RuntimeError(f"ASYNC_L6_WORKER_FAILED:{event.error}")

        result = event.result
        if result is None or result.source_context != request.context:
            raise RuntimeError("ASYNC_L6_SOURCE_CONTEXT_MISMATCH")
        if source_is_stale(context.monotonic_ns, request.context.monotonic_ns, self._max_plan_age_ns):
            self._abandon_pending_rollout()
            return _RolloutDisposition.HOLD

        self._store_rollout_result(request, result, self._pending_goal_selected_ns,
                                   self._pending_goal_selection_reason)
        self._abandon_pending_rollout()
        return _RolloutDisposition.ACCEPTED

    def _accept_pending_rollout(
        self,
        context: TickContext,
    ) -> _RolloutDisposition:
        if self._completion_inputs:
            return self._accept_closed_rollout(context)

        request_id = self._pending_rollout_id
        if request_id is None:
            return _RolloutDisposition.NONE
        request = self._pending_rollout_request
        release_tick_id = self._pending_release_tick_id
        release_not_before_ns = self._pending_release_not_before_ns
        backend = self._rollout_backend
        if (
            request is None
            or backend is None
            or (release_tick_id is None) == (release_not_before_ns is None)
        ):
            raise RuntimeError("async rollout pending state is incomplete")

        source_context = request.context
        if release_not_before_ns is not None:
            before_handoff = context.monotonic_ns < release_not_before_ns
        else:
            assert release_tick_id is not None
            before_handoff = context.tick_id < release_tick_id
            if context.tick_id > release_tick_id:
                raise RuntimeError("ASYNC_L6_RELEASE_TICK_MISSED")

        if before_handoff:
            if self._trajectory_candidates and self._cached_plan_stale(context):
                return _RolloutDisposition.HOLD
            return _RolloutDisposition.NONE

        result = backend.take(request_id)
        if result is None:
            if release_not_before_ns is None:
                raise RuntimeError("ASYNC_L6_DEADLINE_MISSED")
            if self._trajectory_candidates and self._cached_plan_stale(context):
                return _RolloutDisposition.HOLD
            return _RolloutDisposition.NONE

        if result.source_context != source_context:
            raise RuntimeError("ASYNC_L6_SOURCE_CONTEXT_MISMATCH")
        if source_is_stale(context.monotonic_ns, result.source_context.monotonic_ns, self._max_plan_age_ns):
            self._abandon_pending_rollout()
            return _RolloutDisposition.HOLD

        selected_ns = self._pending_goal_selected_ns
        selection_reason = self._pending_goal_selection_reason
        self._pending_rollout_id = None
        self._pending_rollout_request = None
        self._pending_goal_selected_ns = None
        self._pending_goal_selection_reason = None
        self._pending_release_tick_id = None
        self._pending_release_not_before_ns = None
        self._store_rollout_result(request, result, selected_ns, selection_reason)
        return _RolloutDisposition.ACCEPTED

    def _store_rollout_result(
        self, request: TrajectoryRolloutRequest, result: TrajectoryRolloutResult,
        selected_ns: int | None, selection_reason: str | None,
    ) -> None:
        if request.select_local_goal:
            previous_exits = self._explore_reachable_exit_mask
            self._explore_reachable_exit_mask = (previous_exits or 0) | result.reachable_exit_mask
            if selection_reason == "EXHAUSTED_EXIT_PROBE":
                # A restored historical checkpoint has no exit baseline. Its
                # first probe records one, without granting motion authority.
                new_exits = (result.reachable_exit_mask & ~previous_exits
                             if previous_exits is not None else 0)
                selected_exit = (_exit_heading_bit(request.estimate, result.selected_goal)
                                 if result.selected_goal is not None else 0)
                normal_path = any(not c.collision and c.progress_viable
                                  and c.v_mps >= 0.0
                                  and (c.v_mps > _MOTION_EPSILON
                                       or abs(c.omega_rad_s) > _MOTION_EPSILON)
                                  and not c.candidate_id.startswith("escape-")
                                  for c in result.trajectory_candidates)
                if (self._explore_exit_retry_used or not selected_exit & new_exits
                        or not result.goal_reachable or result.selected_goal is None
                        or not normal_path):
                    return
                # This is a single extra episode, not measured progress. The
                # ordinary fresh geometry/execution proof still gates output.
                self._explore_exit_retry_used = True
                self._explore_retry_after_ns = None
                self._explore_progress_started_ns = request.context.monotonic_ns
                self._no_viable_since_ns = None
                selection_reason = "NEW_EXIT_RETRY"
        goal = result.selected_goal or request.goal
        options = result.goal_options if request.select_local_goal else (request.goal, *request.alternate_goals)
        guidance = result.global_guidance
        if guidance is not None:
            if (request.global_goal is None or guidance.source_context != request.context
                    or guidance.frame_id != request.world.frame_id
                    or guidance.odom_generation != request.estimate.localization_quality.generation
                    or guidance.transform_revision != request.estimate.transform_revision):
                raise RuntimeError("ASYNC_L6_GLOBAL_GUIDANCE_LINEAGE_MISMATCH")
            if guidance.status is GlobalPlanStatus.READY:
                if (guidance.local_goal != result.selected_goal
                        or math.hypot(goal.x_m-request.estimate.x_m, goal.y_m-request.estimate.y_m)
                        > self._config.local_goal_distance_m + 1e-6):
                    raise RuntimeError("ASYNC_L6_GLOBAL_GUIDANCE_GOAL_MISMATCH")
                options = (goal,)
            elif any(candidate.progress_viable for candidate in result.trajectory_candidates):
                raise RuntimeError("ASYNC_L6_GLOBAL_GUIDANCE_FAILURE_WITH_MOTION")
        recovery_without_exit = False
        if request.select_local_goal and not options:
            viable = tuple(c for c in result.trajectory_candidates if not c.collision and c.progress_viable)
            recovery_without_exit = (result.selected_goal == request.goal
                and bool(viable) and all(c.candidate_id.startswith("escape-") for c in viable))
            if result.goal_reachable or (result.selected_goal is not None and not recovery_without_exit):
                raise RuntimeError("ASYNC_L6_SELECTED_GOAL_MISMATCH")
            if not recovery_without_exit:
                self._goal_reachable = False
                self._goal_selection_reason = "NO_REACHABLE_GOAL"
                if self._no_viable_since_ns is None:
                    self._no_viable_since_ns = request.context.monotonic_ns
                self._last_replan_ns = request.context.monotonic_ns
                self._last_replan_tick_id = request.context.tick_id
                self._trajectory_candidates = ()
                return
            # This is not a reachable endpoint or another retry budget. The
            # existing owned escape phase and fresh execution proof gate it.
            selection_reason = "NO_REACHABLE_GOAL_RECOVERY"
        if goal not in options and not recovery_without_exit:
            raise RuntimeError("ASYNC_L6_SELECTED_GOAL_MISMATCH")
        if request.select_local_goal and not recovery_without_exit:
            policy = request.explore_preferences or self._config
            distance = math.hypot(goal.x_m-request.estimate.x_m, goal.y_m-request.estimate.y_m)
            failed = {(x, y): until for x, y, until in request.failed_goals}
            if (distance > policy.explore_local_goal_max_distance_m + 1e-9
                    or distance < policy.explore_local_goal_min_distance_m - 1e-9
                    or failed.get(self._coverage_key(goal.x_m, goal.y_m), 0) > request.context.monotonic_ns):
                raise RuntimeError("ASYNC_L6_SELECTED_GOAL_MISMATCH")
        if not result.goal_reachable and not request.allow_exploration_detour:
            raise RuntimeError("ASYNC_L6_GOAL_REACHABILITY_MISMATCH")
        self._goal_reachable = result.goal_reachable
        if selected_ns is not None:
            self._goal_selected_ns = selected_ns
            self._goal_selection_reason = selection_reason
        if request.allow_exploration_detour:
            rejected = options[:options.index(goal)] if result.goal_reachable else options
            for failed_goal in rejected:
                self._remember_failed_goal(failed_goal, request.context.monotonic_ns)
            if not result.goal_reachable:
                self._goal_selection_reason = ("NO_REACHABLE_GOAL_RECOVERY"
                    if recovery_without_exit else "NO_REACHABLE_GOAL")
                if self._no_viable_since_ns is None:
                    self._no_viable_since_ns = request.context.monotonic_ns
        self._store_trajectory_plan(
            request.context.monotonic_ns, request.context.tick_id,
            goal, result.trajectory_candidates, estimate=request.estimate, world=request.world)

    def _cached_plan_stale(self, context: TickContext) -> bool:
        return bool(
            self._trajectory_candidates
            and self._last_replan_ns is not None
            and source_is_stale(context.monotonic_ns, self._last_replan_ns, self._max_plan_age_ns)
        )

    def _abandon_pending_rollout(self) -> None:
        request_id = self._pending_rollout_id
        backend = self._rollout_backend
        if request_id is not None and backend is not None:
            backend.abandon(request_id)
        self._pending_rollout_id = None
        self._pending_rollout_request = None
        self._pending_goal_selected_ns = None
        self._pending_goal_selection_reason = None
        self._pending_release_tick_id = None
        self._pending_release_not_before_ns = None

    def _store_trajectory_plan(
        self,
        monotonic_ns: int,
        tick_id: int,
        local_goal: Waypoint,
        candidates: tuple[TrajectoryEvaluation, ...],
        *,
        estimate: RobotEstimate,
        world: WorldSnapshot,
    ) -> None:
        self._last_replan_ns = monotonic_ns
        self._last_replan_tick_id = tick_id
        self._local_goal = local_goal
        self._trajectory_candidates = candidates
        self._trajectory_geometry_key = (
            _costmap_cache_key(world.local_costmap) if world.local_costmap is not None else None
        )
        self._trajectory_map_revision = world.map_revision
        self._trajectory_local_sigma_m = estimate.localization_quality.local_sigma_m

    def _cached_geometry_changed(self, estimate: RobotEstimate, world: WorldSnapshot) -> bool:
        costmap = world.local_costmap
        return (costmap is None
                or self._trajectory_geometry_key != _costmap_cache_key(costmap)
                or self._trajectory_map_revision != world.map_revision
                or self._trajectory_local_sigma_m is None
                or estimate.localization_quality.local_sigma_m > self._trajectory_local_sigma_m
                or (self._last_replan_ns != estimate.context.monotonic_ns and any(
                    obstacle.confidence >= self._config.obstacle_confidence_floor
                    and (obstacle.vx_mps != 0.0 or obstacle.vy_mps != 0.0)
                    for obstacle in world.obstacle_tracks)))

    def _clear_trajectory_plan(self) -> None:
        self._abandon_pending_rollout()
        self._last_replan_ns = None
        self._last_replan_tick_id = None
        self._trajectory_candidates = ()
        self._trajectory_geometry_key = None
        self._trajectory_map_revision = None
        self._trajectory_local_sigma_m = None
        self._clear_escape_phase()
        self._no_viable_since_ns = None

    def _coverage_key(self, x_m: float, y_m: float) -> tuple[int, int]:
        size = self._config.coverage_cell_size_m
        return math.floor(x_m / size), math.floor(y_m / size)

    def _mark_coverage(self, x_m: float, y_m: float, tick_id: int) -> None:
        key = self._coverage_key(x_m, y_m)
        previous_visits = self._coverage.get(key, (0, 0))[0]
        self._coverage[key] = previous_visits + int(key != self._coverage_last_cell), tick_id
        self._coverage_last_cell = key
        if len(self._coverage) > self._config.coverage_max_cells:
            remove = min(
                self._coverage,
                key=lambda item: (self._coverage[item][1], self._coverage[item][0], item),
            )
            del self._coverage[remove]

    def _choose_local_goal(
        self,
        estimate: RobotEstimate,
        costmap: RollingLocalCostmap,
        scene: _LocalPlanningScene,
        preferences: "ExplorePreferences | None" = None,
    ) -> Waypoint:
        options = self._local_goal_options(estimate, costmap, scene, preferences)
        return options[0] if options else Waypoint(estimate.x_m, estimate.y_m, estimate.yaw_rad)

    def _local_goal_options(
        self, estimate: RobotEstimate, costmap: RollingLocalCostmap,
        scene: _LocalPlanningScene, preferences: "ExplorePreferences | None" = None,
    ) -> tuple[Waypoint, ...]:
        return _rank_local_goal_options(
            self._config, estimate, costmap, scene, preferences,
            {key: visits for key, (visits, _tick) in self._coverage.items()},
            self._failed_goals, self._goal_handoff_ns(),
        )

    def _trajectory_rollout(
        self, estimate: RobotEstimate, world: WorldSnapshot, scene: _LocalPlanningScene,
        goal: Waypoint, max_v_mps: float, max_omega_rad_s: float,
        *, allow_exploration_detour: bool = False,
    ) -> tuple[TrajectoryEvaluation, ...]:
        scene = replace(scene, uncertainty_m=self._config.localization_inflation_sigma * estimate.localization_quality.local_sigma_m)
        return _compute_rollout_candidates(
            self._config, estimate, world, scene, goal, max_v_mps, max_omega_rad_s,
            {key: visits for key, (visits, _tick_id) in self._coverage.items()},
            allow_exploration_detour=allow_exploration_detour,
        )

    def _evaluate_trajectory(
        self, linear_index: int, angular_index: int, v_mps: float, omega_rad_s: float,
        estimate: RobotEstimate, world: WorldSnapshot, scene: _LocalPlanningScene,
        goal: Waypoint, max_v_mps: float, max_omega_rad_s: float,
        start_clearance_m: float, *, planning: bool = True,
    ) -> TrajectoryEvaluation:
        v_mps, omega_rad_s = _constrain_rollout_motion(
            self._config, v_mps, omega_rad_s, max_v_mps, max_omega_rad_s,
            planning=planning,
        )
        return _evaluate_rollout_candidate(
            self._config, linear_index, angular_index, v_mps, omega_rad_s,
            estimate, world, scene, goal, max_v_mps, max_omega_rad_s,
            start_clearance_m,
            {key: visits for key, (visits, _tick_id) in self._coverage.items()},
        )


class TrajectoryRolloutComputer:
    """Pure rollout kernel; owns only config and a derived static-index cache."""

    __slots__ = ("_config", "_static_planning_index")

    def __init__(self, config: NavigationConfig) -> None:
        if not isinstance(config, NavigationConfig):
            raise TypeError("config must be NavigationConfig")
        self._config = config
        self._static_planning_index: _StaticPlanningIndex | None = None

    def _planning_scene(self, world: WorldSnapshot) -> _LocalPlanningScene:
        static_index: _StaticPlanningIndex | None = None
        costmap = world.local_costmap
        if costmap is not None:
            cache_key = _costmap_cache_key(costmap)
            cached = self._static_planning_index
            if cached is None or cached.cache_key != cache_key:
                cached = _build_static_planning_index(costmap, self._config.planning_bucket_size_m)
                self._static_planning_index = cached
            static_index = cached
        dynamic_obstacles = tuple(
            _ObstacleDisc(obstacle.x_m, obstacle.y_m, obstacle.radius_m)
            for obstacle in world.obstacle_tracks
            if obstacle.confidence >= self._config.obstacle_confidence_floor
        )
        return _LocalPlanningScene(static_index, dynamic_obstacles)

    def compute(self, request: TrajectoryRolloutRequest) -> TrajectoryRolloutResult:
        if not isinstance(request, TrajectoryRolloutRequest):
            raise TypeError("request must be TrajectoryRolloutRequest")
        return _compute_goal_rollout(self._config, self._planning_scene(request.world), request)


def hold_position(
    mission: MissionIntent,
    estimate: RobotEstimate,
    world: WorldSnapshot,
) -> NavigationPlan:
    """Preserve the deliberately inert behavior of existing STOP-only roots."""

    return NavigationPlan(
        context=mission.context,
        mission_id=mission.mission_id,
        route=(),
        velocity_target=None,
        constraints=mission.constraints,
        corridor_radius_m=0.0,
        progress=0.0,
        status=NavigationStatus.IDLE,
    )


__all__ = [
    "NavigationConfig",
    "NavigationStateCheckpoint",
    "TrajectoryNavigator",
    "hold_position",
]
