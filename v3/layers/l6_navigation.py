# R2B4_FOLLOW_PERSON_P0_V2_20260923
"""L6 generic deterministic local navigation and progress ownership."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import Enum
from typing import Protocol

from v3.contracts.planner import PlannerInput, TrajectoryRolloutRequest, TrajectoryRolloutResult
from v3.contracts.async_runtime import source_is_stale

from v3.contracts import (
    CommandMode,
    MissionIntent,
    MissionLifecycle,
    NavigationPlan,
    NavigationStatus,
    RobotEstimate,
    RollingLocalCostmap,
    TickContext,
    TrajectoryEvaluation,
    TrajectoryPose,
    Waypoint,
    WorldSnapshot,
)


_PLANNING_BUCKET_SIZE_M = 0.50
_POINT_CLEARANCE_EPSILON_M = 1e-9
# This is footprint-to-obstacle clearance, not LiDAR-origin range. 0.12 m
# corresponds approximately to the production L12 front envelope after robot
# half-length/cell radius are accounted for. It gives L6 room to replan before
# L12 has to reject the same forward command repeatedly.
_LOCAL_ESCAPE_TRIGGER_CLEARANCE_M = 0.12
_LOCAL_ESCAPE_REVERSE_MAX_V_MPS = 0.12
_MOTION_EPSILON = 1e-9


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



@dataclass(frozen=True, slots=True)
class NavigationConfig:
    max_world_freshness_ns: int = 250_000_000
    obstacle_confidence_floor: float = 0.5
    max_costmap_freshness_ns: int = 250_000_000
    trajectory_replan_interval_ns: int = 100_000_000
    # Preserve the nominal 10 Hz replan cadence at a 50 Hz control rate,
    # while guaranteeing cheap control ticks after an over-budget replan.
    trajectory_replan_min_tick_gap: int = 5
    coverage_cell_size_m: float = 0.35
    coverage_max_cells: int = 256
    local_goal_distance_m: float = 1.00
    local_goal_tolerance_m: float = 0.20
    local_goal_max_age_ns: int = 8_000_000_000
    local_goal_heading_samples: int = 16
    rollout_linear_samples: int = 6
    rollout_angular_samples: int = 9
    rollout_horizon_ns: int = 1_200_000_000
    rollout_step_count: int = 8
    footprint_length_m: float = 0.46
    footprint_width_m: float = 0.38
    footprint_safety_margin_m: float = 0.05
    clearance_score_cap_m: float = 0.75
    progress_weight: float = 0.38
    clearance_weight: float = 0.25
    smoothness_weight: float = 0.15
    novelty_weight: float = 0.22
    # Minimum predicted normalized improvement required for a normal trajectory.
    # Progress may be translational or angular alignment toward the local goal.
    progress_viability_floor: float = 0.02
    face_person_min_confidence: float = 0.60
    face_person_align_tolerance_rad: float = 0.10
    face_person_release_tolerance_rad: float = 0.16
    follow_person_min_confidence: float = 0.60
    follow_person_align_tolerance_rad: float = 0.22
    follow_person_release_tolerance_rad: float = 0.30
    follow_person_stand_off_m: float = 1.05
    follow_person_distance_deadband_m: float = 0.15
    follow_person_min_safe_distance_m: float = 0.75
    follow_person_lost_hold_ns: int = 400_000_000
    # Recovery is bounded, rotation-only and deterministic. The first search
    # step looks at the last known heading before sweeping either side.
    follow_person_search_timeout_ns: int = 2_000_000_000
    follow_person_search_sweep_rad: float = 0.45
    # Compatibility field only; search advancement is now yaw-completion-driven.
    follow_person_search_step_ns: int = 400_000_000
    follow_person_search_yaw_tolerance_rad: float = 0.08
    follow_person_pivot_enter_rad: float = 0.55
    follow_person_hold_release_margin_m: float = 0.05
    follow_person_slowdown_distance_m: float = 0.18
    follow_person_minimum_follow_speed_mps: float = 0.10
    follow_person_heading_min_factor: float = 0.75
    # None preserves the acquisition threshold for configs without retention tuning.
    follow_person_retention_min_confidence: float | None = None

    def __post_init__(self) -> None:
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
        ):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
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


class TrajectoryRolloutBackend(Protocol):
    """Authority-free compute port injected by the composition root."""

    def submit(self, request: TrajectoryRolloutRequest) -> int: ...
    def take(self, request_id: int) -> TrajectoryRolloutResult | None: ...
    def abandon(self, request_id: int) -> None: ...
    def close(self) -> None: ...


@dataclass(frozen=True, slots=True)
class AsyncL6PlannerConfig:
    """Deterministic handoff policy; process placement is not layer state."""

    enabled: bool = False
    completion_inputs: bool = False
    request_timeout_ns: int = 300_000_000
    # Legacy replay compatibility. New production handoffs use release_delay_ns.
    release_tick_gap: int = 5
    max_plan_age_ns: int = 350_000_000
    release_delay_ns: int | None = None
    # Missing delivery is a separate hard failure, not a computation deadline.
    transport_timeout_ns: int = 2_000_000_000

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


@dataclass(frozen=True, slots=True)
class _ObstacleDisc:
    x_m: float
    y_m: float
    radius_m: float


@dataclass(frozen=True, slots=True)
class _StaticPlanningIndex:
    cache_key: tuple[str, int, int, float]
    bucket_size_m: float
    cell_radius_m: float
    costmap_radius_m: float
    buckets: dict[tuple[int, int], tuple[_ObstacleDisc, ...]]


@dataclass(frozen=True, slots=True)
class _LocalPlanningScene:
    static_index: _StaticPlanningIndex | None
    dynamic_obstacles: tuple[_ObstacleDisc, ...]


class TrajectoryNavigator:
    """Own reusable trajectory evaluation plus deterministic exploration state."""

    __slots__ = (
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
        "_follow_person_evidence",
        "_follow_person_track_id",
        "_follow_person_last_heading_rad",
        "_coverage",
        "_goal_selected_ns",
        "_initial_distance_m",
        "_last_replan_ns",
        "_last_replan_tick_id",
        "_local_goal",
        "_max_plan_age_ns",
        "_mission_id",
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
    )

    def __init__(
        self,
        config: NavigationConfig = NavigationConfig(),
        *,
        rollout_backend: TrajectoryRolloutBackend | None = None,
        rollout_release_tick_gap: int = 5,
        rollout_release_delay_ns: int | None = None,
        max_plan_age_ns: int = 350_000_000,
        completion_inputs: bool = False,
        request_timeout_ns: int = 300_000_000,
    ) -> None:
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
        self._initial_distance_m = 0.0
        self._progress = 0.0
        self._completed = False
        self._coverage: dict[tuple[int, int], tuple[int, int]] = {}
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
        )

    def restore(self, checkpoint: NavigationStateCheckpoint) -> None:
        if not isinstance(checkpoint, NavigationStateCheckpoint):
            raise TypeError("checkpoint must be NavigationStateCheckpoint")
        self._abandon_pending_rollout()
        self._mission_id = checkpoint.mission_id
        self._initial_distance_m = checkpoint.initial_distance_m
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
        self._follow_person_evidence = None
        # Derived acceleration state is deliberately not part of replay authority.
        self._static_planning_index = None
        pending = checkpoint.pending_rollout_request
        if pending is not None and self._completion_inputs:
            self._pending_rollout_id = pending.context.tick_id + 1
            self._pending_rollout_request = pending
            self._pending_goal_selected_ns = checkpoint.pending_goal_selected_ns
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
            self._pending_release_tick_id = release_tick_id
            self._pending_release_not_before_ns = release_not_before_ns

    def evaluate(
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
            self._reset()
            return self._inactive(mission, NavigationStatus.INVALIDATED, "WORLD_STALE")

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

        distance_m = math.hypot(target.x_m - estimate.x_m, target.y_m - estimate.y_m)
        if self._mission_id != mission.mission_id:
            self._reset()
            self._mission_id = mission.mission_id
            self._initial_distance_m = max(distance_m, mission.constraints.goal_tolerance_m)
            self._progress = 0.0
            self._completed = False
        if self._completed:
            return self._complete_plan(mission)
        progress = min(1.0, max(0.0, 1.0 - distance_m / self._initial_distance_m))
        self._progress = max(self._progress, progress)
        yaw_reached = target.yaw_rad is None or abs(
            _wrapped_angle(target.yaw_rad - estimate.yaw_rad)
        ) <= mission.constraints.yaw_tolerance_rad
        if distance_m <= mission.constraints.goal_tolerance_m and yaw_reached:
            self._completed = True
            self._progress = 1.0
            return self._complete_plan(mission)

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
            )
        if disposition is _RolloutDisposition.HOLD or self._cached_plan_stale(mission.context):
            return self._inactive(mission, NavigationStatus.IDLE, "PLANNER_STALE_HOLD")
        local_goal = self._local_goal
        candidates = self._trajectory_candidates
        if local_goal is None or not candidates:
            if self._completion_inputs and self._pending_rollout_request is not None:
                return self._inactive(mission, NavigationStatus.IDLE, "PLANNER_PENDING")
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
            progress=0.0,
            status=status,
            reason=reason,
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
        self._initial_distance_m = 0.0
        self._progress = 0.0
        self._completed = False
        self._coverage.clear()
        self._local_goal = None
        self._goal_selected_ns = 0
        self._last_replan_ns = None
        self._last_replan_tick_id = None
        self._trajectory_candidates = ()
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
            target_visible=selected is not None,
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
            search_yaw_tolerance_rad=self._config.follow_person_search_yaw_tolerance_rad,
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
                    self._clear_trajectory_plan()
                lost_ns = mission.context.monotonic_ns - self._follow_person_lost_since_ns
                if lost_ns <= self._config.follow_person_lost_hold_ns:
                    self._follow_person_state = _FollowPersonState.OCCLUDED_HOLD
                    return NavigationPlan(
                        context=mission.context,
                        mission_id=mission.mission_id,
                        route=(Waypoint(estimate.x_m, estimate.y_m, estimate.yaw_rad),),
                        velocity_target=None,
                        constraints=mission.constraints,
                        corridor_radius_m=0.0,
                        progress=0.0,
                        status=NavigationStatus.ACTIVE,
                    )

                search_elapsed_ns = lost_ns - self._config.follow_person_lost_hold_ns
                if (
                    self._follow_person_last_heading_rad is not None
                    and search_elapsed_ns <= self._config.follow_person_search_timeout_ns
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
                    if (
                        not entered_search
                        and abs(_wrapped_angle(search_yaw - estimate.yaw_rad))
                        <= self._config.follow_person_search_yaw_tolerance_rad
                    ):
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
                        constraints=mission.constraints,
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
            return NavigationPlan(
                context=mission.context,
                mission_id=mission.mission_id,
                route=(Waypoint(estimate.x_m, estimate.y_m, estimate.yaw_rad),),
                velocity_target=None,
                constraints=mission.constraints,
                corridor_radius_m=0.0,
                progress=0.0,
                status=NavigationStatus.ACTIVE,
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

        # FOLLOW motion shaping keeps HOLD, distance approach and heading response
        # independent. HOLD is the only behavior-level zero-motion state here.
        # The minimum speed is a rollout-envelope floor, not a forced command:
        # rollout still contains v=0 and downstream L7-L12 remain authoritative.
        max_follow_v_mps = mission.constraints.max_v_mps
        minimum_follow_v_mps = min(
            self._config.follow_person_minimum_follow_speed_mps,
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
        distance_cap_mps = min(
            max_follow_v_mps,
            max(
                minimum_follow_v_mps,
                max_follow_v_mps * distance_ratio,
            ),
        )

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
            max_follow_v_mps,
            max(
                minimum_follow_v_mps,
                min(distance_cap_mps, heading_cap_mps),
            ),
        )

        disposition = self._accept_pending_rollout(mission.context)
        if self._replan_due(
            mission.context.monotonic_ns,
            mission.context.tick_id,
        ):
            travel_m = min(
                distance_m - self._config.follow_person_stand_off_m,
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
            return self._inactive(mission, NavigationStatus.IDLE, "PLANNER_STALE_HOLD")
        local_goal = self._local_goal
        candidates = self._trajectory_candidates
        if local_goal is None or not candidates:
            if self._completion_inputs and self._pending_rollout_request is not None:
                return self._inactive(mission, NavigationStatus.IDLE, "PLANNER_PENDING")
            raise RuntimeError("follow-person trajectory cache is empty after replanning")
        return NavigationPlan(
            context=mission.context,
            mission_id=mission.mission_id,
            route=(),
            velocity_target=None,
            constraints=mission.constraints,
            corridor_radius_m=mission.constraints.corridor_radius_m,
            progress=0.0,
            status=NavigationStatus.ACTIVE,
            local_goal=local_goal,
            trajectory_candidates=candidates,
        )

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
        goal = self._local_goal
        if self._replan_due(
            mission.context.monotonic_ns,
            mission.context.tick_id,
        ):
            scene: _LocalPlanningScene | None = None
            goal_selected_ns: int | None = None
            if (
                goal is None
                or math.hypot(goal.x_m - estimate.x_m, goal.y_m - estimate.y_m)
                <= self._config.local_goal_tolerance_m
                or mission.context.monotonic_ns - self._goal_selected_ns
                >= self._config.local_goal_max_age_ns
            ):
                scene = self._build_planning_scene(world)
                goal = self._choose_local_goal(estimate, costmap, scene)
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
            )
        if disposition is _RolloutDisposition.HOLD or self._cached_plan_stale(mission.context):
            return self._inactive(mission, NavigationStatus.IDLE, "PLANNER_STALE_HOLD")
        goal = self._local_goal
        candidates = self._trajectory_candidates
        if goal is None or not candidates:
            if self._completion_inputs and self._pending_rollout_request is not None:
                return self._inactive(mission, NavigationStatus.IDLE, "PLANNER_PENDING")
            raise RuntimeError("trajectory plan cache is empty after replanning")
        return NavigationPlan(
            context=mission.context,
            mission_id=mission.mission_id,
            route=(),
            velocity_target=None,
            constraints=mission.constraints,
            corridor_radius_m=0.0,
            progress=min(1.0, len(self._coverage) / self._config.coverage_max_cells),
            status=NavigationStatus.ACTIVE,
            local_goal=goal,
            trajectory_candidates=candidates,
        )

    def _build_planning_scene(self, world: WorldSnapshot) -> _LocalPlanningScene:
        costmap = world.local_costmap
        static_index: _StaticPlanningIndex | None = None
        if costmap is not None:
            cache_key = _costmap_cache_key(costmap)
            cached = self._static_planning_index
            if cached is None or cached.cache_key != cache_key:
                cached = _build_static_planning_index(costmap)
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
            or not self._trajectory_candidates
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
    ) -> None:
        backend = self._rollout_backend
        # Every new mission gets one synchronous seed. This avoids an ACTIVE
        # planner-warmup STOP and moves all recurring heavy replans off CPU3.
        if not self._completion_inputs and (backend is None or not self._trajectory_candidates):
            planning_scene = scene or self._build_planning_scene(world)
            candidates = self._trajectory_rollout(
                estimate,
                world,
                planning_scene,
                goal,
                max_v_mps,
                max_omega_rad_s,
            )
            if goal_selected_ns is not None:
                self._goal_selected_ns = goal_selected_ns
            self._store_trajectory_plan(
                context.monotonic_ns,
                context.tick_id,
                goal,
                candidates,
            )
            return
        if self._pending_rollout_id is not None:
            raise RuntimeError("async trajectory rollout is already pending")
        request = TrajectoryRolloutRequest(
            context=context,
            estimate=estimate,
            world=world,
            goal=goal,
            max_v_mps=max_v_mps,
            max_omega_rad_s=max_omega_rad_s,
            coverage=tuple(
                (x_index, y_index, visits)
                for (x_index, y_index), (visits, _tick_id) in sorted(
                    self._coverage.items()
                )
            ),
        )
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
                self._clear_trajectory_plan()
                return _RolloutDisposition.HOLD
            raise RuntimeError(f"ASYNC_L6_WORKER_FAILED:{event.error}")

        result = event.result
        if result is None or result.source_context != request.context:
            raise RuntimeError("ASYNC_L6_SOURCE_CONTEXT_MISMATCH")
        if source_is_stale(context.monotonic_ns, request.context.monotonic_ns, self._max_plan_age_ns):
            self._clear_trajectory_plan()
            return _RolloutDisposition.HOLD

        if self._pending_goal_selected_ns is not None:
            self._goal_selected_ns = self._pending_goal_selected_ns
        self._store_trajectory_plan(
            request.context.monotonic_ns,
            request.context.tick_id,
            request.goal,
            result.trajectory_candidates,
        )
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
        goal = request.goal
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
            self._clear_trajectory_plan()
            return _RolloutDisposition.HOLD

        selected_ns = self._pending_goal_selected_ns
        self._pending_rollout_id = None
        self._pending_rollout_request = None
        self._pending_goal_selected_ns = None
        self._pending_release_tick_id = None
        self._pending_release_not_before_ns = None
        if selected_ns is not None:
            self._goal_selected_ns = selected_ns
        self._store_trajectory_plan(
            result.source_context.monotonic_ns,
            result.source_context.tick_id,
            goal,
            result.trajectory_candidates,
        )
        return _RolloutDisposition.ACCEPTED

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
        self._pending_release_tick_id = None
        self._pending_release_not_before_ns = None

    def _store_trajectory_plan(
        self,
        monotonic_ns: int,
        tick_id: int,
        local_goal: Waypoint,
        candidates: tuple[TrajectoryEvaluation, ...],
    ) -> None:
        self._last_replan_ns = monotonic_ns
        self._last_replan_tick_id = tick_id
        self._local_goal = local_goal
        self._trajectory_candidates = candidates

    def _clear_trajectory_plan(self) -> None:
        self._abandon_pending_rollout()
        self._last_replan_ns = None
        self._last_replan_tick_id = None
        self._trajectory_candidates = ()

    def _coverage_key(self, x_m: float, y_m: float) -> tuple[int, int]:
        size = self._config.coverage_cell_size_m
        return math.floor(x_m / size), math.floor(y_m / size)

    def _mark_coverage(self, x_m: float, y_m: float, tick_id: int) -> None:
        key = self._coverage_key(x_m, y_m)
        previous_visits = self._coverage.get(key, (0, 0))[0]
        self._coverage[key] = previous_visits + 1, tick_id
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
    ) -> Waypoint:
        options: list[tuple[float, int, Waypoint]] = []
        footprint_radius = 0.5 * math.hypot(
            self._config.footprint_length_m,
            self._config.footprint_width_m,
        )
        relevant_clearance_m = max(
            self._config.clearance_score_cap_m,
            footprint_radius + self._config.footprint_safety_margin_m,
        ) + _POINT_CLEARANCE_EPSILON_M
        for index in range(self._config.local_goal_heading_samples):
            offset = _alternating_heading_offset(
                index,
                self._config.local_goal_heading_samples,
            )
            heading = _wrapped_angle(estimate.yaw_rad + offset)
            goal = Waypoint(
                estimate.x_m + self._config.local_goal_distance_m * math.cos(heading),
                estimate.y_m + self._config.local_goal_distance_m * math.sin(heading),
            )
            clearance = _planning_scene_point_clearance(
                goal.x_m,
                goal.y_m,
                scene,
                costmap.radius_m,
                relevant_clearance_m,
            )
            if clearance <= footprint_radius + self._config.footprint_safety_margin_m:
                continue
            visits = self._coverage.get(self._coverage_key(goal.x_m, goal.y_m), (0, 0))[0]
            novelty = 1.0 / (1.0 + visits)
            clearance_score = min(1.0, clearance / self._config.clearance_score_cap_m)
            forward_preference = 0.5 * (math.cos(offset) + 1.0)
            score = 0.65 * novelty + 0.25 * clearance_score + 0.10 * forward_preference
            options.append((score, index, goal))
        if not options:
            return Waypoint(estimate.x_m, estimate.y_m, estimate.yaw_rad)
        return max(options, key=lambda item: (item[0], -item[1]))[2]

    def _trajectory_rollout(
        self,
        estimate: RobotEstimate,
        world: WorldSnapshot,
        scene: _LocalPlanningScene,
        goal: Waypoint,
        max_v_mps: float,
        max_omega_rad_s: float,
    ) -> tuple[TrajectoryEvaluation, ...]:
        start_clearance_m = _footprint_clearance(
            estimate.x_m,
            estimate.y_m,
            estimate.yaw_rad,
            world,
            self._config,
            scene,
        )
        evaluations: list[TrajectoryEvaluation] = []
        for linear_index in range(self._config.rollout_linear_samples):
            v_mps = max_v_mps * linear_index / (self._config.rollout_linear_samples - 1)
            for angular_index in range(self._config.rollout_angular_samples):
                angular_ratio = (
                    2.0 * angular_index / (self._config.rollout_angular_samples - 1) - 1.0
                )
                omega_rad_s = max_omega_rad_s * angular_ratio
                evaluations.append(
                    self._evaluate_trajectory(
                        linear_index,
                        angular_index,
                        v_mps,
                        omega_rad_s,
                        estimate,
                        world,
                        scene,
                        goal,
                        max_v_mps,
                        max_omega_rad_s,
                        start_clearance_m,
                    )
                )
        # Normal navigation is useful only if at least one collision-free
        # candidate can improve distance or heading toward the local goal. Keep the
        # full 54-candidate family for diagnostics/L7 ranking once that invariant holds.
        if any(
            not candidate.collision and candidate.progress_viable
            for candidate in evaluations
        ):
            return tuple(evaluations)

        evaluations = []
        reverse_limit_mps = min(_LOCAL_ESCAPE_REVERSE_MAX_V_MPS, max_v_mps)
        for linear_index in range(self._config.rollout_linear_samples):
            v_mps = (
                0.0
                if linear_index == 0
                else -reverse_limit_mps
                * linear_index
                / (self._config.rollout_linear_samples - 1)
            )
            for angular_index in range(self._config.rollout_angular_samples):
                # Reverse recovery is straight at every angular index. Reuse its
                # immutable geometry while preserving the bounded family and IDs.
                if linear_index > 0 and angular_index > 0:
                    evaluations.append(replace(
                        evaluations[-1],
                        candidate_id=f"escape-{linear_index:02d}-{angular_index:02d}",
                    ))
                    continue
                angular_ratio = (
                    2.0 * angular_index
                    / (self._config.rollout_angular_samples - 1)
                    - 1.0
                )
                omega_rad_s = (
                    max_omega_rad_s * angular_ratio
                    if linear_index == 0
                    else 0.0
                )
                candidate = self._evaluate_trajectory(
                    linear_index,
                    angular_index,
                    v_mps,
                    omega_rad_s,
                    estimate,
                    world,
                    scene,
                    goal,
                    max_v_mps,
                    max_omega_rad_s,
                    start_clearance_m,
                )
                evaluations.append(
                    _as_escape_candidate(
                        candidate,
                        linear_index,
                        angular_index,
                        start_clearance_m,
                        world,
                        self._config,
                        scene,
                    )
                )
        return tuple(evaluations)

    def _evaluate_trajectory(
        self,
        linear_index: int,
        angular_index: int,
        v_mps: float,
        omega_rad_s: float,
        estimate: RobotEstimate,
        world: WorldSnapshot,
        scene: _LocalPlanningScene,
        goal: Waypoint,
        max_v_mps: float,
        max_omega_rad_s: float,
        start_clearance_m: float,
    ) -> TrajectoryEvaluation:
        step_ns = self._config.rollout_horizon_ns // self._config.rollout_step_count
        samples: list[TrajectoryPose] = []
        x_m, y_m, yaw_rad = estimate.x_m, estimate.y_m, estimate.yaw_rad
        min_clearance = start_clearance_m
        start_collision = min_clearance <= self._config.footprint_safety_margin_m
        collision = start_collision
        for step in range(1, self._config.rollout_step_count + 1):
            offset_ns = (
                self._config.rollout_horizon_ns
                if step == self._config.rollout_step_count
                else step_ns * step
            )
            dt_s = (offset_ns - (samples[-1].time_offset_ns if samples else 0)) / 1e9
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
                    self._config,
                    scene,
                )
                min_clearance = min(min_clearance, clearance)
                collision = collision or clearance <= self._config.footprint_safety_margin_m

        if (
            v_mps > _MOTION_EPSILON
            and min_clearance <= _LOCAL_ESCAPE_TRIGGER_CLEARANCE_M
        ):
            collision = True

        start_distance = math.hypot(goal.x_m - estimate.x_m, goal.y_m - estimate.y_m)
        final_distance = math.hypot(goal.x_m - x_m, goal.y_m - y_m)
        progress = _clamp_signed(
            (start_distance - final_distance) / max(start_distance, 1e-9)
        )
        progress_potential = _trajectory_progress_potential(
            estimate, goal, x_m, y_m, yaw_rad, progress
        )
        progress_viable = (
            progress_potential + 1e-12 >= self._config.progress_viability_floor
        )
        linear_change = abs(v_mps - estimate.v_mps) / max(max_v_mps, 1e-9)
        angular_change = abs(omega_rad_s - estimate.omega_rad_s) / max(
            max_omega_rad_s,
            1e-9,
        )
        smoothness = max(0.0, 1.0 - min(1.0, 0.5 * (linear_change + angular_change)))
        novelty = sum(
            1.0
            / (
                1.0
                + self._coverage.get(
                    self._coverage_key(sample.x_m, sample.y_m),
                    (0, 0),
                )[0]
            )
            for sample in samples
        ) / len(samples)
        bounded_clearance = min(
            self._config.clearance_score_cap_m,
            max(0.0, min_clearance),
        )
        clearance_score = bounded_clearance / self._config.clearance_score_cap_m
        total_score = (
            self._config.progress_weight * progress_potential
            + self._config.clearance_weight * clearance_score
            + self._config.smoothness_weight * smoothness
            + self._config.novelty_weight * novelty
        )
        return TrajectoryEvaluation(
            candidate_id=f"trajectory-{linear_index:02d}-{angular_index:02d}",
            v_mps=v_mps,
            omega_rad_s=omega_rad_s,
            horizon_ns=self._config.rollout_horizon_ns,
            samples=tuple(samples),
            collision=collision,
            min_clearance_m=bounded_clearance,
            progress_score=progress,
            progress_potential_score=progress_potential,
            progress_viable=progress_viable,
            smoothness_score=smoothness,
            novelty_score=novelty,
            total_score=total_score,
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
                cached = _build_static_planning_index(costmap)
                self._static_planning_index = cached
            static_index = cached
        dynamic_obstacles = tuple(
            _ObstacleDisc(obstacle.x_m, obstacle.y_m, obstacle.radius_m)
            for obstacle in world.obstacle_tracks
            if obstacle.confidence >= self._config.obstacle_confidence_floor
        )
        return _LocalPlanningScene(static_index, dynamic_obstacles)

    def _coverage_key(self, x_m: float, y_m: float) -> tuple[int, int]:
        size = self._config.coverage_cell_size_m
        return math.floor(x_m / size), math.floor(y_m / size)

    def compute(self, request: TrajectoryRolloutRequest) -> TrajectoryRolloutResult:
        if not isinstance(request, TrajectoryRolloutRequest):
            raise TypeError("request must be TrajectoryRolloutRequest")
        config = self._config
        estimate = request.estimate
        world = request.world
        goal = request.goal
        scene = self._planning_scene(world)
        coverage = {
            (x_index, y_index): visits
            for x_index, y_index, visits in request.coverage
        }
        start_clearance_m = _footprint_clearance(
            estimate.x_m,
            estimate.y_m,
            estimate.yaw_rad,
            world,
            config,
            scene,
        )
        evaluations: list[TrajectoryEvaluation] = []
        for linear_index in range(config.rollout_linear_samples):
            v_mps = (
                request.max_v_mps
                * linear_index
                / (config.rollout_linear_samples - 1)
            )
            for angular_index in range(config.rollout_angular_samples):
                angular_ratio = (
                    2.0 * angular_index / (config.rollout_angular_samples - 1) - 1.0
                )
                omega_rad_s = request.max_omega_rad_s * angular_ratio
                evaluations.append(
                    self._evaluate_candidate(
                        linear_index,
                        angular_index,
                        v_mps,
                        omega_rad_s,
                        estimate,
                        world,
                        scene,
                        goal,
                        request.max_v_mps,
                        request.max_omega_rad_s,
                        start_clearance_m,
                        coverage,
                    )
                )
        if not any(
            not candidate.collision and candidate.progress_viable
            for candidate in evaluations
        ):
            evaluations = []
            reverse_limit_mps = min(
                _LOCAL_ESCAPE_REVERSE_MAX_V_MPS,
                request.max_v_mps,
            )
            for linear_index in range(config.rollout_linear_samples):
                v_mps = (
                    0.0
                    if linear_index == 0
                    else -reverse_limit_mps
                    * linear_index
                    / (config.rollout_linear_samples - 1)
                )
                for angular_index in range(config.rollout_angular_samples):
                    if linear_index > 0 and angular_index > 0:
                        evaluations.append(replace(
                            evaluations[-1],
                            candidate_id=f"escape-{linear_index:02d}-{angular_index:02d}",
                        ))
                        continue
                    angular_ratio = (
                        2.0 * angular_index
                        / (config.rollout_angular_samples - 1)
                        - 1.0
                    )
                    omega_rad_s = (
                        request.max_omega_rad_s * angular_ratio
                        if linear_index == 0
                        else 0.0
                    )
                    candidate = self._evaluate_candidate(
                        linear_index,
                        angular_index,
                        v_mps,
                        omega_rad_s,
                        estimate,
                        world,
                        scene,
                        goal,
                        request.max_v_mps,
                        request.max_omega_rad_s,
                        start_clearance_m,
                        coverage,
                    )
                    evaluations.append(
                        _as_escape_candidate(
                            candidate,
                            linear_index,
                            angular_index,
                            start_clearance_m,
                            world,
                            config,
                            scene,
                        )
                    )
        return TrajectoryRolloutResult(request.context, tuple(evaluations))

    def _evaluate_candidate(
        self,
        linear_index: int,
        angular_index: int,
        v_mps: float,
        omega_rad_s: float,
        estimate: RobotEstimate,
        world: WorldSnapshot,
        scene: _LocalPlanningScene,
        goal: Waypoint,
        max_v_mps: float,
        max_omega_rad_s: float,
        start_clearance_m: float,
        coverage: dict[tuple[int, int], int],
    ) -> TrajectoryEvaluation:
        config = self._config
        step_ns = config.rollout_horizon_ns // config.rollout_step_count
        samples: list[TrajectoryPose] = []
        x_m, y_m, yaw_rad = estimate.x_m, estimate.y_m, estimate.yaw_rad
        min_clearance = start_clearance_m
        start_collision = min_clearance <= config.footprint_safety_margin_m
        collision = start_collision
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
                )
                min_clearance = min(min_clearance, clearance)
                collision = (
                    collision
                    or clearance <= config.footprint_safety_margin_m
                )
        if (
            v_mps > _MOTION_EPSILON
            and min_clearance <= _LOCAL_ESCAPE_TRIGGER_CLEARANCE_M
        ):
            collision = True

        start_distance = math.hypot(goal.x_m - estimate.x_m, goal.y_m - estimate.y_m)
        final_distance = math.hypot(goal.x_m - x_m, goal.y_m - y_m)
        progress = _clamp_signed(
            (start_distance - final_distance) / max(start_distance, 1e-9)
        )
        progress_potential = _trajectory_progress_potential(
            estimate, goal, x_m, y_m, yaw_rad, progress
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
                + coverage.get(self._coverage_key(sample.x_m, sample.y_m), 0)
            )
            for sample in samples
        ) / len(samples)
        bounded_clearance = min(
            config.clearance_score_cap_m,
            max(0.0, min_clearance),
        )
        clearance_score = bounded_clearance / config.clearance_score_cap_m
        total_score = (
            config.progress_weight * progress_potential
            + config.clearance_weight * clearance_score
            + config.smoothness_weight * smoothness
            + config.novelty_weight * novelty
        )
        return TrajectoryEvaluation(
            candidate_id=f"trajectory-{linear_index:02d}-{angular_index:02d}",
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
        )


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
) -> float:
    """Predict generic local progress without conflating it with quality score.

    A trajectory is useful when it either gets closer to the local goal or turns
    the robot meaningfully toward it. This admits productive in-place pivots while
    rejecting stationary/nonproductive local optima.
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
        (start_heading_error - final_heading_error) / math.pi
    )
    return max(distance_progress_score, heading_progress)


def _as_escape_candidate(
    candidate: TrajectoryEvaluation,
    linear_index: int,
    angular_index: int,
    start_clearance_m: float,
    world: WorldSnapshot,
    config: NavigationConfig,
    scene: _LocalPlanningScene,
) -> TrajectoryEvaluation:
    """Make recovery viability and quality depend on predicted clearance gain.

    Minimum path clearance includes the starting footprint, so it cannot express
    improvement. Keep it for collision safety and score the final clearance gain
    separately. A pivot earns no unconditional bonus for remaining in place.
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
    viable = (
        moving
        and clearance_progress > _MOTION_EPSILON
        and clearance_progress + 1e-12 >= config.progress_viability_floor
    )
    escape_score = (
        config.progress_weight * clearance_progress
        + config.clearance_weight * candidate.min_clearance_m / config.clearance_score_cap_m
        + config.smoothness_weight * candidate.smoothness_score
        + config.novelty_weight * candidate.novelty_score
    )
    return replace(
        candidate,
        candidate_id=f"escape-{linear_index:02d}-{angular_index:02d}",
        total_score=escape_score,
        progress_potential_score=clearance_progress,
        progress_viable=viable,
    )


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


def _wrapped_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _alternating_heading_offset(index: int, count: int) -> float:
    if index == 0:
        return 0.0
    step = 2.0 * math.pi / count
    rank = (index + 1) // 2
    sign = 1.0 if index % 2 else -1.0
    return sign * rank * step


def _integrate_constant_twist(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    v_mps: float,
    omega_rad_s: float,
    dt_s: float,
) -> tuple[float, float, float]:
    next_yaw = _wrapped_angle(yaw_rad + omega_rad_s * dt_s)
    if abs(omega_rad_s) <= 1e-12:
        return (
            x_m + v_mps * math.cos(yaw_rad) * dt_s,
            y_m + v_mps * math.sin(yaw_rad) * dt_s,
            next_yaw,
        )
    radius_m = v_mps / omega_rad_s
    return (
        x_m + radius_m * (math.sin(next_yaw) - math.sin(yaw_rad)),
        y_m - radius_m * (math.cos(next_yaw) - math.cos(yaw_rad)),
        next_yaw,
    )


def _clamp_signed(value: float) -> float:
    return min(1.0, max(-1.0, value))


def _costmap_cache_key(costmap: RollingLocalCostmap) -> tuple[str, int, int, float]:
    return (
        costmap.frame_id,
        costmap.revision,
        costmap.source_sequence,
        costmap.resolution_m,
    )


def _bucket_key(x_m: float, y_m: float, bucket_size_m: float) -> tuple[int, int]:
    return math.floor(x_m / bucket_size_m), math.floor(y_m / bucket_size_m)


def _build_static_planning_index(costmap: RollingLocalCostmap) -> _StaticPlanningIndex:
    cell_radius_m = costmap.resolution_m / math.sqrt(2.0)
    bucket_lists: dict[tuple[int, int], list[_ObstacleDisc]] = {}
    for cell in costmap.occupied_cells:
        obstacle = _ObstacleDisc(
            (cell.grid_x + 0.5) * costmap.resolution_m,
            (cell.grid_y + 0.5) * costmap.resolution_m,
            cell_radius_m,
        )
        key = _bucket_key(obstacle.x_m, obstacle.y_m, _PLANNING_BUCKET_SIZE_M)
        bucket_lists.setdefault(key, []).append(obstacle)
    return _StaticPlanningIndex(
        cache_key=_costmap_cache_key(costmap),
        bucket_size_m=_PLANNING_BUCKET_SIZE_M,
        cell_radius_m=cell_radius_m,
        costmap_radius_m=costmap.radius_m,
        buckets={key: tuple(items) for key, items in bucket_lists.items()},
    )


def _scene_from_world(
    world: WorldSnapshot,
    config: NavigationConfig,
) -> _LocalPlanningScene:
    static_index = (
        _build_static_planning_index(world.local_costmap)
        if world.local_costmap is not None
        else None
    )
    return _LocalPlanningScene(
        static_index,
        tuple(
            _ObstacleDisc(obstacle.x_m, obstacle.y_m, obstacle.radius_m)
            for obstacle in world.obstacle_tracks
            if obstacle.confidence >= config.obstacle_confidence_floor
        ),
    )


def _static_obstacles_in_bounds(
    static_index: _StaticPlanningIndex,
    min_x_m: float,
    max_x_m: float,
    min_y_m: float,
    max_y_m: float,
):
    bucket_size = static_index.bucket_size_m
    min_bucket_x = math.floor(min_x_m / bucket_size)
    max_bucket_x = math.floor(max_x_m / bucket_size)
    min_bucket_y = math.floor(min_y_m / bucket_size)
    max_bucket_y = math.floor(max_y_m / bucket_size)
    for bucket_x in range(min_bucket_x, max_bucket_x + 1):
        for bucket_y in range(min_bucket_y, max_bucket_y + 1):
            yield from static_index.buckets.get((bucket_x, bucket_y), ())


def _planning_scene_point_clearance(
    x_m: float,
    y_m: float,
    scene: _LocalPlanningScene,
    empty_default_m: float,
    relevant_clearance_m: float,
) -> float:
    static_index = scene.static_index
    if static_index is None or not static_index.buckets:
        return empty_default_m
    search_radius_m = relevant_clearance_m + static_index.cell_radius_m
    minimum = relevant_clearance_m
    for obstacle in _static_obstacles_in_bounds(
        static_index,
        x_m - search_radius_m,
        x_m + search_radius_m,
        y_m - search_radius_m,
        y_m + search_radius_m,
    ):
        clearance = max(
            0.0,
            math.hypot(obstacle.x_m - x_m, obstacle.y_m - y_m)
            - obstacle.radius_m,
        )
        if clearance < minimum:
            minimum = clearance
            if minimum <= 0.0:
                return 0.0
    return minimum


def _costmap_point_clearance(
    x_m: float,
    y_m: float,
    costmap: RollingLocalCostmap,
) -> float:
    if not costmap.occupied_cells:
        return costmap.radius_m
    half_diagonal = costmap.resolution_m / math.sqrt(2.0)
    return max(
        0.0,
        min(
            math.hypot(
                (cell.grid_x + 0.5) * costmap.resolution_m - x_m,
                (cell.grid_y + 0.5) * costmap.resolution_m - y_m,
            )
            - half_diagonal
            for cell in costmap.occupied_cells
        ),
    )


def _rectangle_disc_clearance(
    robot_x_m: float,
    robot_y_m: float,
    half_length_m: float,
    half_width_m: float,
    yaw_cos: float,
    yaw_sin: float,
    obstacle: _ObstacleDisc,
) -> float:
    dx = obstacle.x_m - robot_x_m
    dy = obstacle.y_m - robot_y_m
    local_x = yaw_cos * dx + yaw_sin * dy
    local_y = -yaw_sin * dx + yaw_cos * dy
    outside_x = max(0.0, abs(local_x) - half_length_m)
    outside_y = max(0.0, abs(local_y) - half_width_m)
    return max(0.0, math.hypot(outside_x, outside_y) - obstacle.radius_m)


def _footprint_clearance(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    world: WorldSnapshot,
    config: NavigationConfig,
    scene: _LocalPlanningScene | None = None,
) -> float:
    if scene is None:
        scene = _scene_from_world(world, config)

    half_length = 0.5 * config.footprint_length_m
    half_width = 0.5 * config.footprint_width_m
    yaw_cos = math.cos(yaw_rad)
    yaw_sin = math.sin(yaw_rad)
    minimum = config.clearance_score_cap_m

    # Static costmap: broad-phase buckets first, then the unchanged exact
    # rotated-rectangle/disc clearance. The AABB reach is conservative, so
    # cells that could reduce the capped result cannot be omitted.
    static_index = scene.static_index
    if static_index is not None and static_index.buckets:
        padding_m = static_index.cell_radius_m + config.clearance_score_cap_m
        extent_x_m = (
            abs(yaw_cos) * half_length
            + abs(yaw_sin) * half_width
            + padding_m
        )
        extent_y_m = (
            abs(yaw_sin) * half_length
            + abs(yaw_cos) * half_width
            + padding_m
        )
        for obstacle in _static_obstacles_in_bounds(
            static_index,
            x_m - extent_x_m,
            x_m + extent_x_m,
            y_m - extent_y_m,
            y_m + extent_y_m,
        ):
            clearance = _rectangle_disc_clearance(
                x_m,
                y_m,
                half_length,
                half_width,
                yaw_cos,
                yaw_sin,
                obstacle,
            )
            if clearance < minimum:
                minimum = clearance
                if minimum <= 0.0:
                    return 0.0

    # Dynamic tracks intentionally stay separate: their population is small
    # and a later P1 can predict each track at TrajectoryPose.time_offset_ns
    # without changing the static spatial index or introducing another planner.
    for obstacle in scene.dynamic_obstacles:
        clearance = _rectangle_disc_clearance(
            x_m,
            y_m,
            half_length,
            half_width,
            yaw_cos,
            yaw_sin,
            obstacle,
        )
        if clearance < minimum:
            minimum = clearance
            if minimum <= 0.0:
                return 0.0

    return minimum


__all__ = [
    "NavigationConfig",
    "NavigationStateCheckpoint",
    "TrajectoryNavigator",
    "hold_position",
]
