"""L6 generic deterministic local navigation and progress ownership."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import (
    CommandMode,
    MissionIntent,
    MissionLifecycle,
    NavigationPlan,
    NavigationStatus,
    RobotEstimate,
    RollingLocalCostmap,
    TrajectoryEvaluation,
    TrajectoryPose,
    Waypoint,
    WorldSnapshot,
)


@dataclass(frozen=True, slots=True)
class NavigationConfig:
    max_world_freshness_ns: int = 250_000_000
    obstacle_confidence_floor: float = 0.5
    max_costmap_freshness_ns: int = 250_000_000
    trajectory_replan_interval_ns: int = 100_000_000
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
    trajectory_candidates: tuple[TrajectoryEvaluation, ...]


class TrajectoryNavigator:
    """Own reusable trajectory evaluation plus deterministic exploration state."""

    __slots__ = (
        "_completed",
        "_config",
        "_coverage",
        "_goal_selected_ns",
        "_initial_distance_m",
        "_last_replan_ns",
        "_local_goal",
        "_mission_id",
        "_progress",
        "_trajectory_candidates",
    )

    def __init__(self, config: NavigationConfig = NavigationConfig()) -> None:
        self._config = config
        self._mission_id: str | None = None
        self._initial_distance_m = 0.0
        self._progress = 0.0
        self._completed = False
        self._coverage: dict[tuple[int, int], tuple[int, int]] = {}
        self._local_goal: Waypoint | None = None
        self._goal_selected_ns = 0
        self._last_replan_ns: int | None = None
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
            self._trajectory_candidates,
        )

    def restore(self, checkpoint: NavigationStateCheckpoint) -> None:
        if not isinstance(checkpoint, NavigationStateCheckpoint):
            raise TypeError("checkpoint must be NavigationStateCheckpoint")
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
        self._trajectory_candidates = checkpoint.trajectory_candidates

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
        if self._replan_due(mission.context.monotonic_ns):
            local_distance = min(distance_m, self._config.local_goal_distance_m)
            heading = math.atan2(target.y_m - estimate.y_m, target.x_m - estimate.x_m)
            local_goal = Waypoint(
                estimate.x_m + local_distance * math.cos(heading),
                estimate.y_m + local_distance * math.sin(heading),
                target.yaw_rad if local_distance == distance_m else None,
            )
            self._store_trajectory_plan(
                mission.context.monotonic_ns,
                local_goal,
                self._trajectory_rollout(
                    estimate,
                    world,
                    local_goal,
                    mission.constraints.max_v_mps,
                    mission.constraints.max_omega_rad_s,
                ),
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
        self._mission_id = None
        self._initial_distance_m = 0.0
        self._progress = 0.0
        self._completed = False
        self._coverage.clear()
        self._local_goal = None
        self._goal_selected_ns = 0
        self._last_replan_ns = None
        self._trajectory_candidates = ()

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

        self._mark_coverage(estimate.x_m, estimate.y_m, mission.context.tick_id)
        goal = self._local_goal
        if self._replan_due(mission.context.monotonic_ns):
            if (
                goal is None
                or math.hypot(goal.x_m - estimate.x_m, goal.y_m - estimate.y_m)
                <= self._config.local_goal_tolerance_m
                or mission.context.monotonic_ns - self._goal_selected_ns
                >= self._config.local_goal_max_age_ns
            ):
                goal = self._choose_local_goal(estimate, costmap)
                self._goal_selected_ns = mission.context.monotonic_ns
            self._store_trajectory_plan(
                mission.context.monotonic_ns,
                goal,
                self._trajectory_rollout(
                    estimate,
                    world,
                    goal,
                    mission.constraints.max_v_mps,
                    mission.constraints.max_omega_rad_s,
                ),
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

    def _replan_due(self, monotonic_ns: int) -> bool:
        previous = self._last_replan_ns
        return (
            previous is None
            or not self._trajectory_candidates
            or monotonic_ns - previous >= self._config.trajectory_replan_interval_ns
        )

    def _store_trajectory_plan(
        self,
        monotonic_ns: int,
        local_goal: Waypoint,
        candidates: tuple[TrajectoryEvaluation, ...],
    ) -> None:
        self._last_replan_ns = monotonic_ns
        self._local_goal = local_goal
        self._trajectory_candidates = candidates

    def _clear_trajectory_plan(self) -> None:
        self._last_replan_ns = None
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
    ) -> Waypoint:
        options: list[tuple[float, int, Waypoint]] = []
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
            clearance = _costmap_point_clearance(goal.x_m, goal.y_m, costmap)
            footprint_radius = 0.5 * math.hypot(
                self._config.footprint_length_m,
                self._config.footprint_width_m,
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
                clearance = _footprint_clearance(x_m, y_m, yaw_rad, world, self._config)
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


def _footprint_clearance(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    world: WorldSnapshot,
    config: NavigationConfig,
) -> float:
    half_length = 0.5 * config.footprint_length_m
    half_width = 0.5 * config.footprint_width_m
    yaw_cos = math.cos(yaw_rad)
    yaw_sin = math.sin(yaw_rad)
    clearances: list[float] = []

    def rectangle_clearance(
        obstacle_x: float,
        obstacle_y: float,
        obstacle_radius: float,
    ) -> float:
        dx = obstacle_x - x_m
        dy = obstacle_y - y_m
        local_x = yaw_cos * dx + yaw_sin * dy
        local_y = -yaw_sin * dx + yaw_cos * dy
        outside_x = max(0.0, abs(local_x) - half_length)
        outside_y = max(0.0, abs(local_y) - half_width)
        return max(0.0, math.hypot(outside_x, outside_y) - obstacle_radius)

    costmap = world.local_costmap
    if costmap is not None:
        cell_radius = costmap.resolution_m / math.sqrt(2.0)
        for cell in costmap.occupied_cells:
            clearances.append(
                rectangle_clearance(
                    (cell.grid_x + 0.5) * costmap.resolution_m,
                    (cell.grid_y + 0.5) * costmap.resolution_m,
                    cell_radius,
                )
            )
    for obstacle in world.obstacle_tracks:
        if obstacle.confidence >= config.obstacle_confidence_floor:
            clearances.append(
                rectangle_clearance(
                    obstacle.x_m,
                    obstacle.y_m,
                    obstacle.radius_m,
                )
            )
    return min(clearances, default=config.clearance_score_cap_m)


__all__ = [
    "NavigationConfig",
    "NavigationStateCheckpoint",
    "TrajectoryNavigator",
    "hold_position",
]
