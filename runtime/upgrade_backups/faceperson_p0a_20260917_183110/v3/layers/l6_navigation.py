"""L6 generic deterministic local navigation and progress ownership."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

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


@dataclass(frozen=True, slots=True)
class TrajectoryRolloutRequest:
    """Immutable pure-computation snapshot handed to the rollout worker."""

    context: TickContext
    estimate: RobotEstimate
    world: WorldSnapshot
    goal: Waypoint
    max_v_mps: float
    max_omega_rad_s: float
    coverage: tuple[tuple[int, int, int], ...]

    def __post_init__(self) -> None:
        if self.estimate.context != self.context or self.world.context != self.context:
            raise ValueError("rollout request inputs must share one TickContext")
        if not isinstance(self.goal, Waypoint):
            raise TypeError("goal must be Waypoint")
        for name in ("max_v_mps", "max_omega_rad_s"):
            value = getattr(self, name)
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value < 0.0
            ):
                raise ValueError(f"{name} must be finite and non-negative")
        if any(
            not isinstance(item, tuple)
            or len(item) != 3
            or any(not isinstance(value, int) or isinstance(value, bool) for value in item)
            or item[2] < 0
            for item in self.coverage
        ):
            raise ValueError("coverage must contain integer (x, y, visits) tuples")


@dataclass(frozen=True, slots=True)
class TrajectoryRolloutResult:
    source_context: TickContext
    trajectory_candidates: tuple[TrajectoryEvaluation, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.source_context, TickContext):
            raise TypeError("source_context must be TickContext")
        if (
            not isinstance(self.trajectory_candidates, tuple)
            or not self.trajectory_candidates
            or any(
                not isinstance(item, TrajectoryEvaluation)
                for item in self.trajectory_candidates
            )
        ):
            raise ValueError("trajectory_candidates must be a non-empty tuple")


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
    # Legacy replay compatibility. New production handoffs use release_delay_ns.
    release_tick_gap: int = 5
    max_plan_age_ns: int = 350_000_000
    release_delay_ns: int | None = None

    def __post_init__(self) -> None:
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
        "_config",
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
        # Derived acceleration state is deliberately not part of replay authority.
        self._static_planning_index = None
        pending = checkpoint.pending_rollout_request
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
    ) -> NavigationPlan:
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
        self._accept_pending_rollout(mission.context)
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
        local_goal = self._local_goal
        candidates = self._trajectory_candidates
        if local_goal is None or not candidates:
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

        self._accept_pending_rollout(mission.context)
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
        goal = self._local_goal
        candidates = self._trajectory_candidates
        if goal is None or not candidates:
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
            return False
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
        if self._rollout_backend is not None and self._rollout_release_delay_ns is not None:
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
        if backend is None or not self._trajectory_candidates:
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

    def _accept_pending_rollout(self, context: TickContext) -> bool:
        request_id = self._pending_rollout_id
        if request_id is None:
            return False
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
            # Until the deterministic handoff boundary the previously accepted
            # plan remains authoritative, so its age is still safety-critical.
            if (
                self._last_replan_ns is not None
                and context.monotonic_ns - self._last_replan_ns > self._max_plan_age_ns
            ):
                raise RuntimeError("ASYNC_L6_PLAN_STALE")
            return False

        # At the deterministic handoff boundary inspect the replacement first.
        # Its immutable source snapshot, not scheduler jitter, defines freshness.
        result = backend.take(request_id)
        if result is None:
            raise RuntimeError("ASYNC_L6_DEADLINE_MISSED")
        if result.source_context != source_context:
            raise RuntimeError("ASYNC_L6_SOURCE_CONTEXT_MISMATCH")
        if (
            context.monotonic_ns - result.source_context.monotonic_ns
            > self._max_plan_age_ns
        ):
            raise RuntimeError("ASYNC_L6_PLAN_STALE")
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
        return True

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

        start_distance = math.hypot(goal.x_m - estimate.x_m, goal.y_m - estimate.y_m)
        final_distance = math.hypot(goal.x_m - x_m, goal.y_m - y_m)
        progress = _clamp_signed(
            (start_distance - final_distance) / max(start_distance, 1e-9)
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
            self._config.progress_weight * progress
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

        start_distance = math.hypot(goal.x_m - estimate.x_m, goal.y_m - estimate.y_m)
        final_distance = math.hypot(goal.x_m - x_m, goal.y_m - y_m)
        progress = _clamp_signed(
            (start_distance - final_distance) / max(start_distance, 1e-9)
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
            config.progress_weight * progress
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
            smoothness_score=smoothness,
            novelty_score=novelty,
            total_score=total_score,
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
