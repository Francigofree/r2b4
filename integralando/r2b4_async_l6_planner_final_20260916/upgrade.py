#!/usr/bin/env python3
from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path

EXPECTED_BASE = "c183965fc4d702d91e809471c685d6515d8d3811"
MARKER = "R2B4_ASYNC_L6_PLANNER_V1"
PACKAGE_DIR = Path(__file__).resolve().parent
BACKUP_DIR = PACKAGE_DIR / "backup"

NEW_ASYNC_MODULE = (PACKAGE_DIR / "l6_planner_process.py.new").read_text(encoding="utf-8")
NEW_TEST = (PACKAGE_DIR / "test_v3_async_l6_planner.py.new").read_text(encoding="utf-8")


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly 1 source anchor, found {count}")
    return text.replace(old, new, 1)


def load(repo: Path, relative: str) -> str:
    path = repo / relative
    if not path.is_file():
        raise RuntimeError(f"missing target: {relative}")
    return path.read_text(encoding="utf-8")


def save(repo: Path, relative: str, text: str) -> None:
    path = repo / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def backup(repo: Path, relative: str) -> None:
    source = repo / relative
    target = BACKUP_DIR / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    if source.exists():
        shutil.copy2(source, target)
    else:
        target.with_suffix(target.suffix + ".ABSENT").write_text("\n", encoding="utf-8")


def git_head(repo: Path) -> str | None:
    try:
        return subprocess.check_output(
            ["git", "-C", str(repo), "rev-parse", "HEAD"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def patch_l6(text: str) -> str:
    text = replace_once(
        text,
        "import math\nfrom dataclasses import dataclass\n",
        "import math\nfrom dataclasses import dataclass\nfrom typing import Protocol\n",
        "L6 Protocol import",
    )
    text = replace_once(
        text,
        "    RollingLocalCostmap,\n    TrajectoryEvaluation,\n",
        "    RollingLocalCostmap,\n    TickContext,\n    TrajectoryEvaluation,\n",
        "L6 TickContext import",
    )
    checkpoint_anchor = '''@dataclass(frozen=True, slots=True)
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
'''
    request_contract = checkpoint_anchor.replace(
        "    trajectory_candidates: tuple[TrajectoryEvaluation, ...]\n",
        "    trajectory_candidates: tuple[TrajectoryEvaluation, ...]\n"
        "    pending_rollout_request: TrajectoryRolloutRequest | None = None\n"
        "    pending_goal_selected_ns: int | None = None\n"
        "    pending_release_tick_id: int | None = None\n",
    ) + '''

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
'''
    text = replace_once(text, checkpoint_anchor, request_contract, "L6 rollout contract")

    old_init = '''    __slots__ = (
        "_completed",
        "_config",
        "_coverage",
        "_goal_selected_ns",
        "_initial_distance_m",
        "_last_replan_ns",
        "_last_replan_tick_id",
        "_local_goal",
        "_mission_id",
        "_progress",
        "_static_planning_index",
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
        self._last_replan_tick_id: int | None = None
        self._static_planning_index: _StaticPlanningIndex | None = None
        self._trajectory_candidates: tuple[TrajectoryEvaluation, ...] = ()
'''
    new_init = '''    __slots__ = (
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
        "_pending_release_tick_id",
        "_pending_rollout_id",
        "_pending_rollout_request",
        "_progress",
        "_rollout_backend",
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
        self._config = config
        self._rollout_backend = rollout_backend
        self._rollout_release_tick_gap = rollout_release_tick_gap
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
        self._static_planning_index: _StaticPlanningIndex | None = None
        self._trajectory_candidates: tuple[TrajectoryEvaluation, ...] = ()
'''
    text = replace_once(text, old_init, new_init, "TrajectoryNavigator async state")

    text = replace_once(
        text,
        '''            self._last_replan_tick_id,
            self._trajectory_candidates,
        )
''',
        '''            self._last_replan_tick_id,
            self._trajectory_candidates,
            self._pending_rollout_request,
            self._pending_goal_selected_ns,
            self._pending_release_tick_id,
        )
''',
        "L6 checkpoint pending request",
    )
    text = replace_once(
        text,
        '''    def restore(self, checkpoint: NavigationStateCheckpoint) -> None:
        if not isinstance(checkpoint, NavigationStateCheckpoint):
            raise TypeError("checkpoint must be NavigationStateCheckpoint")
        self._mission_id = checkpoint.mission_id
''',
        '''    def restore(self, checkpoint: NavigationStateCheckpoint) -> None:
        if not isinstance(checkpoint, NavigationStateCheckpoint):
            raise TypeError("checkpoint must be NavigationStateCheckpoint")
        self._abandon_pending_rollout()
        self._mission_id = checkpoint.mission_id
''',
        "L6 restore pending clear",
    )
    text = replace_once(
        text,
        '''        # Derived acceleration state is deliberately not part of replay authority.
        self._static_planning_index = None
''',
        '''        # Derived acceleration state is deliberately not part of replay authority.
        self._static_planning_index = None
        pending = checkpoint.pending_rollout_request
        if pending is not None:
            backend = self._rollout_backend
            if backend is None:
                raise RuntimeError("async navigation checkpoint requires rollout backend")
            release_tick_id = checkpoint.pending_release_tick_id
            if release_tick_id is None:
                raise RuntimeError("async navigation checkpoint lacks release tick")
            request_id = backend.submit(pending)
            self._pending_rollout_id = request_id
            self._pending_rollout_request = pending
            self._pending_goal_selected_ns = checkpoint.pending_goal_selected_ns
            self._pending_release_tick_id = release_tick_id
''',
        "L6 restore pending request",
    )

    old_nav_replan = '''        if self._replan_due(
            mission.context.monotonic_ns,
            mission.context.tick_id,
        ):
            scene = self._build_planning_scene(world)
            local_distance = min(distance_m, self._config.local_goal_distance_m)
            heading = math.atan2(target.y_m - estimate.y_m, target.x_m - estimate.x_m)
            local_goal = Waypoint(
                estimate.x_m + local_distance * math.cos(heading),
                estimate.y_m + local_distance * math.sin(heading),
                target.yaw_rad if local_distance == distance_m else None,
            )
            self._store_trajectory_plan(
                mission.context.monotonic_ns,
                mission.context.tick_id,
                local_goal,
                self._trajectory_rollout(
                    estimate,
                    world,
                    scene,
                    local_goal,
                    mission.constraints.max_v_mps,
                    mission.constraints.max_omega_rad_s,
                ),
            )
'''
    new_nav_replan = '''        self._accept_pending_rollout(mission.context)
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
'''
    text = replace_once(text, old_nav_replan, new_nav_replan, "NAVIGATE async replan")

    text = replace_once(
        text,
        '''    def _reset(self) -> None:
        self._mission_id = None
''',
        '''    def _reset(self) -> None:
        self._abandon_pending_rollout()
        self._mission_id = None
''',
        "L6 reset pending clear",
    )

    old_explore = '''        self._mark_coverage(estimate.x_m, estimate.y_m, mission.context.tick_id)
        goal = self._local_goal
        if self._replan_due(
            mission.context.monotonic_ns,
            mission.context.tick_id,
        ):
            scene = self._build_planning_scene(world)
            if (
                goal is None
                or math.hypot(goal.x_m - estimate.x_m, goal.y_m - estimate.y_m)
                <= self._config.local_goal_tolerance_m
                or mission.context.monotonic_ns - self._goal_selected_ns
                >= self._config.local_goal_max_age_ns
            ):
                goal = self._choose_local_goal(estimate, costmap, scene)
                self._goal_selected_ns = mission.context.monotonic_ns
            self._store_trajectory_plan(
                mission.context.monotonic_ns,
                mission.context.tick_id,
                goal,
                self._trajectory_rollout(
                    estimate,
                    world,
                    scene,
                    goal,
                    mission.constraints.max_v_mps,
                    mission.constraints.max_omega_rad_s,
                ),
            )
'''
    new_explore = '''        self._accept_pending_rollout(mission.context)
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
'''
    text = replace_once(text, old_explore, new_explore, "EXPLORE async replan")

    text = replace_once(
        text,
        '''    def _replan_due(self, monotonic_ns: int, tick_id: int) -> bool:
        previous_ns = self._last_replan_ns
''',
        '''    def _replan_due(self, monotonic_ns: int, tick_id: int) -> bool:
        if self._pending_rollout_id is not None:
            return False
        previous_ns = self._last_replan_ns
''',
        "L6 pending replan gate",
    )

    helper_anchor = '''    def _store_trajectory_plan(
        self,
        monotonic_ns: int,
        tick_id: int,
        local_goal: Waypoint,
        candidates: tuple[TrajectoryEvaluation, ...],
    ) -> None:
'''
    helpers = '''    def _schedule_or_store_rollout(
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
        self._pending_release_tick_id = context.tick_id + self._rollout_release_tick_gap

    def _accept_pending_rollout(self, context: TickContext) -> bool:
        request_id = self._pending_rollout_id
        if request_id is None:
            return False
        request = self._pending_rollout_request
        release_tick_id = self._pending_release_tick_id
        backend = self._rollout_backend
        if request is None or release_tick_id is None or backend is None:
            raise RuntimeError("async rollout pending state is incomplete")
        source_context = request.context
        goal = request.goal
        if (
            self._last_replan_ns is not None
            and context.monotonic_ns - self._last_replan_ns > self._max_plan_age_ns
        ):
            raise RuntimeError("ASYNC_L6_PLAN_STALE")
        if context.tick_id < release_tick_id:
            return False
        if context.tick_id > release_tick_id:
            raise RuntimeError("ASYNC_L6_RELEASE_TICK_MISSED")
        result = backend.take(request_id)
        if result is None:
            raise RuntimeError("ASYNC_L6_DEADLINE_MISSED")
        if result.source_context != source_context:
            raise RuntimeError("ASYNC_L6_SOURCE_CONTEXT_MISMATCH")
        selected_ns = self._pending_goal_selected_ns
        self._pending_rollout_id = None
        self._pending_rollout_request = None
        self._pending_goal_selected_ns = None
        self._pending_release_tick_id = None
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

    def close(self) -> None:
        self._abandon_pending_rollout()
        backend = self._rollout_backend
        if backend is not None:
            backend.close()

'''
    text = replace_once(text, helper_anchor, helpers + helper_anchor, "L6 async helpers")

    text = replace_once(
        text,
        '''    def _clear_trajectory_plan(self) -> None:
        self._last_replan_ns = None
''',
        '''    def _clear_trajectory_plan(self) -> None:
        self._abandon_pending_rollout()
        self._last_replan_ns = None
''',
        "L6 clear pending rollout",
    )

    computer = '''

class TrajectoryRolloutComputer:
    """Authority-free rollout kernel with only a derived static-index cache."""

    __slots__ = ("_navigator",)

    def __init__(self, config: NavigationConfig) -> None:
        if not isinstance(config, NavigationConfig):
            raise TypeError("config must be NavigationConfig")
        self._navigator = TrajectoryNavigator(config)

    def compute(self, request: TrajectoryRolloutRequest) -> TrajectoryRolloutResult:
        if not isinstance(request, TrajectoryRolloutRequest):
            raise TypeError("request must be TrajectoryRolloutRequest")
        navigator = self._navigator
        navigator._coverage = {
            (x_index, y_index): (visits, request.context.tick_id)
            for x_index, y_index, visits in request.coverage
        }
        scene = navigator._build_planning_scene(request.world)
        return TrajectoryRolloutResult(
            request.context,
            navigator._trajectory_rollout(
                request.estimate,
                request.world,
                scene,
                request.goal,
                request.max_v_mps,
                request.max_omega_rad_s,
            ),
        )
'''
    text = replace_once(
        text,
        "\n\ndef hold_position(\n",
        computer + "\n\ndef hold_position(\n",
        "L6 rollout computer",
    )
    return text


def patch_native_control(text: str) -> str:
    text = replace_once(
        text,
        '''from v3.layers.l6_navigation import (
    NavigationConfig,
    NavigationStateCheckpoint,
    TrajectoryNavigator,
)
''',
        '''from v3.layers.l6_navigation import (
    NavigationConfig,
    NavigationStateCheckpoint,
    TrajectoryNavigator,
)
from .async_l6_planner import (
    AsyncL6PlannerConfig,
    InlineTrajectoryRolloutBackend,
)
''',
        "native control async imports",
    )
    text = replace_once(
        text,
        '''    world_model: WorldModelConfig
    navigation: NavigationConfig

    def __post_init__(self) -> None:
''',
        '''    world_model: WorldModelConfig
    navigation: NavigationConfig
    async_l6: AsyncL6PlannerConfig = AsyncL6PlannerConfig()

    def __post_init__(self) -> None:
''',
        "V3NavigationConfig async field",
    )
    text = replace_once(
        text,
        '''        if not isinstance(self.navigation, NavigationConfig):
            raise TypeError("navigation must be NavigationConfig")
''',
        '''        if not isinstance(self.navigation, NavigationConfig):
            raise TypeError("navigation must be NavigationConfig")
        if not isinstance(self.async_l6, AsyncL6PlannerConfig):
            raise TypeError("async_l6 must be AsyncL6PlannerConfig")
''',
        "V3NavigationConfig async validation",
    )
    text = replace_once(
        text,
        '''    exploration = _mapping(root.get("exploration"), "v3_navigation.exploration")
    rollout = _mapping(root.get("trajectory_rollout"), "v3_navigation.trajectory_rollout")
''',
        '''    exploration = _mapping(root.get("exploration"), "v3_navigation.exploration")
    rollout = _mapping(root.get("trajectory_rollout"), "v3_navigation.trajectory_rollout")
    async_value = root.get("async_l6")
    async_mapping = (
        {}
        if async_value is None
        else _mapping(async_value, "v3_navigation.async_l6")
    )
    async_enabled = async_mapping.get("enabled", False)
    if type(async_enabled) is not bool:
        raise ValueError("v3_navigation.async_l6.enabled must be bool")
    async_l6 = AsyncL6PlannerConfig(
        enabled=async_enabled,
        release_tick_gap=_positive_int(
            async_mapping.get("release_tick_gap", 5),
            "v3_navigation.async_l6.release_tick_gap",
        ),
        max_plan_age_ns=_positive_int(
            async_mapping.get("max_plan_age_ns", 350_000_000),
            "v3_navigation.async_l6.max_plan_age_ns",
        ),
    )
''',
        "parse async_l6 config",
    )
    text = replace_once(
        text,
        '''    return V3NavigationConfig(
        local_perception_min_range_m=local_min_range_m,
        local_perception_max_range_m=local_max_range_m,
        local_perception_max_points=local_max_points,
        world_model=world_model,
        navigation=navigation,
    )
''',
        '''    return V3NavigationConfig(
        local_perception_min_range_m=local_min_range_m,
        local_perception_max_range_m=local_max_range_m,
        local_perception_max_points=local_max_points,
        world_model=world_model,
        navigation=navigation,
        async_l6=async_l6,
    )
''',
        "return async_l6 config",
    )
    text = replace_once(
        text,
        '''    world_model: WorldModelConfig = WorldModelConfig()
    mission: MissionConfig = MissionConfig()
    navigation: NavigationConfig = NavigationConfig()
    motion_realization: MotionRealizationConfig = MotionRealizationConfig()
''',
        '''    world_model: WorldModelConfig = WorldModelConfig()
    mission: MissionConfig = MissionConfig()
    navigation: NavigationConfig = NavigationConfig()
    async_l6: AsyncL6PlannerConfig = AsyncL6PlannerConfig()
    motion_realization: MotionRealizationConfig = MotionRealizationConfig()
''',
        "control config async field",
    )
    text = replace_once(
        text,
        '''            ("mission", self.mission, MissionConfig),
            ("navigation", self.navigation, NavigationConfig),
            (
                "motion_realization",
''',
        '''            ("mission", self.mission, MissionConfig),
            ("navigation", self.navigation, NavigationConfig),
            ("async_l6", self.async_l6, AsyncL6PlannerConfig),
            (
                "motion_realization",
''',
        "control config async validation",
    )
    text = replace_once(
        text,
        '''    def __init__(
        self,
        motor_writer: object,
        config: NativeControlCompositionConfig,
    ) -> None:
''',
        '''    def __init__(
        self,
        motor_writer: object,
        config: NativeControlCompositionConfig,
        *,
        trajectory_rollout_backend: object | None = None,
    ) -> None:
''',
        "NativeControl constructor backend",
    )
    text = replace_once(
        text,
        '''        world_model = ShadowWorldModel(config.world_model)
        mission = MissionManager(config.mission)
        navigation = TrajectoryNavigator(config.navigation)
        motion_realization = MotionRealizer(config.motion_realization)
''',
        '''        world_model = ShadowWorldModel(config.world_model)
        mission = MissionManager(config.mission)
        if config.async_l6.enabled:
            backend = trajectory_rollout_backend
            if backend is None:
                backend = InlineTrajectoryRolloutBackend(config.navigation)
            navigation = TrajectoryNavigator(
                config.navigation,
                rollout_backend=backend,
                rollout_release_tick_gap=config.async_l6.release_tick_gap,
                max_plan_age_ns=config.async_l6.max_plan_age_ns,
            )
        else:
            if trajectory_rollout_backend is not None:
                raise ValueError("trajectory rollout backend requires async_l6.enabled")
            navigation = TrajectoryNavigator(config.navigation)
        motion_realization = MotionRealizer(config.motion_realization)
''',
        "NativeControl async navigator wiring",
    )
    text = replace_once(
        text,
        '''    def run_tick(self, inputs: TickInputs) -> TickResult:
        if not isinstance(inputs, TickInputs):
            raise TypeError("inputs must be TickInputs")
        return self._engine.run_tick(inputs)

    def run_fault_tick(
''',
        '''    def run_tick(self, inputs: TickInputs) -> TickResult:
        if not isinstance(inputs, TickInputs):
            raise TypeError("inputs must be TickInputs")
        return self._engine.run_tick(inputs)

    def close(self) -> None:
        self._navigation.close()

    def run_fault_tick(
''',
        "NativeControl close",
    )
    return text


def patch_resident_live(text: str) -> str:
    text = replace_once(
        text,
        '''        config: ResidentLiveControlConfig,
        *,
        auxiliary_sources: tuple[LiveDeviceSource, ...] = (),
    ) -> None:
''',
        '''        config: ResidentLiveControlConfig,
        *,
        auxiliary_sources: tuple[LiveDeviceSource, ...] = (),
        trajectory_rollout_backend: object | None = None,
    ) -> None:
''',
        "ResidentLive backend argument",
    )
    text = replace_once(
        text,
        '''        self._command_gateway = command_gateway
        self._control = NativeControlComposition(motor_writer, config.control)
        self._config = config
''',
        '''        self._command_gateway = command_gateway
        self._control = NativeControlComposition(
            motor_writer,
            config.control,
            trajectory_rollout_backend=trajectory_rollout_backend,
        )
        self._config = config
''',
        "ResidentLive backend wiring",
    )
    text = replace_once(
        text,
        '''    def checkpoint(self) -> NativeControlStateCheckpoint:
        return self._control.checkpoint()

    def _preflight_is_fresh_for(self, context: TickContext) -> bool:
''',
        '''    def checkpoint(self) -> NativeControlStateCheckpoint:
        return self._control.checkpoint()

    def close(self) -> None:
        self._control.close()

    def _preflight_is_fresh_for(self, context: TickContext) -> bool:
''',
        "ResidentLive close",
    )
    return text


def patch_resident_physical(text: str) -> str:
    text = replace_once(
        text,
        '''        config: ResidentPhysicalControlConfig,
        *,
        auxiliary_sources: tuple[LiveDeviceSource, ...] = (),
    ) -> None:
''',
        '''        config: ResidentPhysicalControlConfig,
        *,
        auxiliary_sources: tuple[LiveDeviceSource, ...] = (),
        trajectory_rollout_backend: object | None = None,
    ) -> None:
''',
        "ResidentPhysical backend argument",
    )
    text = replace_once(
        text,
        '''                config.live_control,
                auxiliary_sources=auxiliary_sources,
            )
''',
        '''                config.live_control,
                auxiliary_sources=auxiliary_sources,
                trajectory_rollout_backend=trajectory_rollout_backend,
            )
''',
        "ResidentPhysical backend wiring",
    )
    text = replace_once(
        text,
        '''        try:
            self._motor_output.close()
        finally:
            self._shutdown = True
''',
        '''        try:
            self._motor_output.close()
        finally:
            try:
                self._live_control.close()
            finally:
                self._shutdown = True
''',
        "ResidentPhysical planner close",
    )
    return text


def patch_runtime(text: str) -> str:
    text = replace_once(
        text,
        '''    auxiliary_sources: tuple[LiveDeviceSource, ...] = (),
    stop_requested: Callable[[], bool],
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
    tick_observer: Callable[[TickResult], None] | None = None,
    readiness_observer: Callable[[TickResult, bool], None] | None = None,
    record_observer: Callable[[CaptureRecord], None] | None = None,
    timing_enabled: bool = False,
) -> ResidentRuntimeReport:
''',
        '''    auxiliary_sources: tuple[LiveDeviceSource, ...] = (),
    stop_requested: Callable[[], bool],
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
    tick_observer: Callable[[TickResult], None] | None = None,
    readiness_observer: Callable[[TickResult, bool], None] | None = None,
    record_observer: Callable[[CaptureRecord], None] | None = None,
    timing_enabled: bool = False,
    trajectory_rollout_backend: object | None = None,
) -> ResidentRuntimeReport:
''',
        "runtime backend argument",
    )
    text = replace_once(
        text,
        '''        config.composition,
        auxiliary_sources=auxiliary_sources,
    )
''',
        '''        config.composition,
        auxiliary_sources=auxiliary_sources,
        trajectory_rollout_backend=trajectory_rollout_backend,
    )
''',
        "runtime backend wiring",
    )
    text = replace_once(
        text,
        '''    record_observer: Callable[[CaptureRecord], None] | None = None,
    timing_enabled: bool = False,
) -> ResidentRuntimeReport:
    """Run the resident path and always close the sole concrete input owner."""
''',
        '''    record_observer: Callable[[CaptureRecord], None] | None = None,
    timing_enabled: bool = False,
    trajectory_rollout_backend: object | None = None,
) -> ResidentRuntimeReport:
    """Run the resident path and always close the sole concrete input owner."""
''',
        "owned runtime backend argument",
    )
    text = replace_once(
        text,
        '''            record_observer=record_observer,
            timing_enabled=timing_enabled,
        )
''',
        '''            record_observer=record_observer,
            timing_enabled=timing_enabled,
            trajectory_rollout_backend=trajectory_rollout_backend,
        )
''',
        "owned runtime backend pass-through",
    )
    return text


def patch_hardware_runtime(text: str) -> str:
    text = replace_once(
        text,
        '''from v3.composition.live_inputs import (
    LiveInputComposition,
    LiveInputCompositionConfig,
)
''',
        '''from v3.composition.live_inputs import (
    LiveInputComposition,
    LiveInputCompositionConfig,
)
from v3.adapters.l6_planner_process import ProcessTrajectoryRolloutBackend
''',
        "hardware async planner import",
    )
    old = '''    try:
        def observe(result: TickResult) -> None:
            owner.publish_tick_result(result)
            if raw_lidar_observer is not None:
                raw_lidar_observer(owner.inputs.raw_lidar_snapshot())
            if tick_observer is not None:
                tick_observer(result)

        return run_owned_resident_physical_control(
            owner.inputs,
            command_gateway,
            motor_gpio_backend,
            config,
            stop_requested=stop_requested,
            monotonic_ns=monotonic_ns,
            sleep=sleep,
            tick_observer=observe,
            readiness_observer=readiness_observer,
            record_observer=record_observer,
            timing_enabled=bool(
                affinity_config is not None and affinity_config.enabled
            ),
        )
    finally:
        owner.close()
'''
    new = '''    rollout_backend = None
    async_l6 = config.composition.live_control.control.async_l6
    if async_l6.enabled:
        affinity = affinity_config or RuntimeAffinityConfig(enabled=False)
        rollout_backend = ProcessTrajectoryRolloutBackend(
            config.composition.live_control.control.navigation,
            worker_cpu=(affinity.io_cpu if affinity.enabled else None),
            strict_affinity=(affinity.strict if affinity.enabled else False),
        )
    try:
        def observe(result: TickResult) -> None:
            owner.publish_tick_result(result)
            if raw_lidar_observer is not None:
                raw_lidar_observer(owner.inputs.raw_lidar_snapshot())
            if tick_observer is not None:
                tick_observer(result)

        return run_owned_resident_physical_control(
            owner.inputs,
            command_gateway,
            motor_gpio_backend,
            config,
            stop_requested=stop_requested,
            monotonic_ns=monotonic_ns,
            sleep=sleep,
            tick_observer=observe,
            readiness_observer=readiness_observer,
            record_observer=record_observer,
            timing_enabled=bool(
                affinity_config is not None and affinity_config.enabled
            ),
            trajectory_rollout_backend=rollout_backend,
        )
    finally:
        if rollout_backend is not None:
            rollout_backend.close()
        owner.close()
'''
    return replace_once(text, old, new, "hardware process planner wiring")


def patch_bounded_config(text: str) -> str:
    return replace_once(
        text,
        '''        {
            "world_model": navigation_config.world_model,
            "navigation": navigation_config.navigation,
        }
''',
        '''        {
            "world_model": navigation_config.world_model,
            "navigation": navigation_config.navigation,
            "async_l6": navigation_config.async_l6,
        }
''',
        "bounded config async propagation",
    )


def patch_replay(text: str) -> str:
    return replace_once(
        text,
        '''        {
            "world_model": navigation.world_model,
            "navigation": navigation.navigation,
        }
''',
        '''        {
            "world_model": navigation.world_model,
            "navigation": navigation.navigation,
            "async_l6": navigation.async_l6,
        }
''',
        "replay compatibility async propagation",
    )


def patch_validation_helpers(text: str) -> str:
    return replace_once(
        text,
        '''        world_model=navigation.world_model,
        navigation=navigation.navigation,
    )
''',
        '''        world_model=navigation.world_model,
        navigation=navigation.navigation,
        async_l6=navigation.async_l6,
    )
''',
        "test helper async config propagation",
    )


def patch_architecture(text: str) -> str:
    old = '''A composition root egyetlen `TickEngine`-t futtat. A motor-döntést befolyásoló folyamatban nincs rétegenkénti thread, sleep, falióra, rejtett I/O vagy modulglobális mutable state.
'''
    new = '''A composition root egyetlen `TickEngine`-t futtat. A motor-döntést befolyásoló authority és layer-state nem költözhet rétegenkénti threadbe/processzbe, és továbbra sincs layer-oldali sleep, falióra, rejtett I/O vagy modulglobális mutable state. Authority nélküli, composition root által injektált **pure computation worker** használható drága, determinisztikus részszámításhoz (például L6 trajectory rollout), ha kizárólag lezárt immutable snapshotból számol, nem birtokol mission/navigation/safety/motor state-et, és az eredményt az owning layer csak előre meghatározott tick-határon veheti át. Worker-hiány, deadline-miss, context-eltérés vagy túl öreg elfogadott terv fail-closed hiba; a checkpoint az esetleges pending immutable rollout-kérést és annak release tickjét is rögzíti, replay pedig ezt a kérést pure módon újraszámolja és ugyanazon a handoff ticken teszi láthatóvá.
'''
    return replace_once(text, old, new, "V3 async pure-compute exception")


def patch_control_json(text: str) -> str:
    root = json.loads(text)
    nav = root.get("v3_navigation")
    if not isinstance(nav, dict):
        raise RuntimeError("control config lacks v3_navigation object")
    current = nav.get("async_l6")
    desired = {
        "enabled": True,
        "release_tick_gap": 5,
        "max_plan_age_ns": 350000000,
    }
    if current not in (None, desired):
        raise RuntimeError(f"v3_navigation.async_l6 already contains unexpected value: {current!r}")
    nav["async_l6"] = desired
    return json.dumps(root, indent=2, ensure_ascii=False) + "\n"


# ---------------------------------------------------------------------------
# Boundary-clean overrides: scheduling/pure kernel stay in L6; multiprocessing
# stays at the adapter/runtime edge. Existing architecture tests remain valid.
# ---------------------------------------------------------------------------

_BASE_PATCH_L6 = patch_l6


def patch_l6(text: str) -> str:
    text = _BASE_PATCH_L6(text)
    protocol = '''class TrajectoryRolloutBackend(Protocol):
    """Authority-free compute port injected by the composition root."""

    def submit(self, request: TrajectoryRolloutRequest) -> int: ...
    def take(self, request_id: int) -> TrajectoryRolloutResult | None: ...
    def abandon(self, request_id: int) -> None: ...
    def close(self) -> None: ...
'''
    policy = protocol + '''

@dataclass(frozen=True, slots=True)
class AsyncL6PlannerConfig:
    """Deterministic handoff policy; process placement is not layer state."""

    enabled: bool = False
    release_tick_gap: int = 5
    max_plan_age_ns: int = 350_000_000

    def __post_init__(self) -> None:
        if type(self.enabled) is not bool:
            raise TypeError("enabled must be bool")
        for value, name in (
            (self.release_tick_gap, "release_tick_gap"),
            (self.max_plan_age_ns, "max_plan_age_ns"),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")


class InlineTrajectoryRolloutBackend:
    """Pure replay/test backend; compute now, reveal only on L6 release tick."""

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
'''
    text = replace_once(text, protocol, policy, "L6 async policy and inline backend")

    old_computer = '''

class TrajectoryRolloutComputer:
    """Authority-free rollout kernel with only a derived static-index cache."""

    __slots__ = ("_navigator",)

    def __init__(self, config: NavigationConfig) -> None:
        if not isinstance(config, NavigationConfig):
            raise TypeError("config must be NavigationConfig")
        self._navigator = TrajectoryNavigator(config)

    def compute(self, request: TrajectoryRolloutRequest) -> TrajectoryRolloutResult:
        if not isinstance(request, TrajectoryRolloutRequest):
            raise TypeError("request must be TrajectoryRolloutRequest")
        navigator = self._navigator
        navigator._coverage = {
            (x_index, y_index): (visits, request.context.tick_id)
            for x_index, y_index, visits in request.coverage
        }
        scene = navigator._build_planning_scene(request.world)
        return TrajectoryRolloutResult(
            request.context,
            navigator._trajectory_rollout(
                request.estimate,
                request.world,
                scene,
                request.goal,
                request.max_v_mps,
                request.max_omega_rad_s,
            ),
        )
'''
    new_computer = '''

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
'''
    text = replace_once(text, old_computer, new_computer, "pure L6 rollout computer")

    text = replace_once(
        text,
        '''    def close(self) -> None:
        self._abandon_pending_rollout()
        backend = self._rollout_backend
        if backend is not None:
            backend.close()

''',
        '',
        "L6 does not own backend lifecycle",
    )
    return text


_BASE_PATCH_NATIVE_CONTROL = patch_native_control


def patch_native_control(text: str) -> str:
    text = _BASE_PATCH_NATIVE_CONTROL(text)
    text = replace_once(
        text,
        '''from v3.layers.l6_navigation import (
    NavigationConfig,
    NavigationStateCheckpoint,
    TrajectoryNavigator,
)
from .async_l6_planner import (
    AsyncL6PlannerConfig,
    InlineTrajectoryRolloutBackend,
)
''',
        '''from v3.layers.l6_navigation import (
    AsyncL6PlannerConfig,
    InlineTrajectoryRolloutBackend,
    NavigationConfig,
    NavigationStateCheckpoint,
    TrajectoryNavigator,
)
''',
        "native control keeps process/runtime imports out",
    )
    text = replace_once(
        text,
        '''        "_navigation",
        "_operational_constraints",
        "_world_model",
''',
        '''        "_navigation",
        "_operational_constraints",
        "_rollout_backend",
        "_world_model",
''',
        "native control backend resource slot",
    )
    text = replace_once(
        text,
        '''        else:
            if trajectory_rollout_backend is not None:
                raise ValueError("trajectory rollout backend requires async_l6.enabled")
            navigation = TrajectoryNavigator(config.navigation)
        motion_realization = MotionRealizer(config.motion_realization)
''',
        '''        else:
            if trajectory_rollout_backend is not None:
                raise ValueError("trajectory rollout backend requires async_l6.enabled")
            backend = None
            navigation = TrajectoryNavigator(config.navigation)
        motion_realization = MotionRealizer(config.motion_realization)
''',
        "native control disabled backend state",
    )
    text = replace_once(
        text,
        '''        self._navigation = navigation
        self._motion_realization = motion_realization
''',
        '''        self._navigation = navigation
        self._rollout_backend = backend
        self._motion_realization = motion_realization
''',
        "native control owns injected compute backend",
    )
    text = replace_once(
        text,
        '''    def close(self) -> None:
        self._navigation.close()
''',
        '''    def close(self) -> None:
        backend = self._rollout_backend
        self._rollout_backend = None
        if backend is not None:
            backend.close()
''',
        "native control closes compute backend",
    )
    return text


def patch_architecture(text: str) -> str:
    old = '''A composition root egyetlen `TickEngine`-t futtat. A motor-döntést befolyásoló folyamatban nincs rétegenkénti thread, sleep, falióra, rejtett I/O vagy modulglobális mutable state.
'''
    new = '''A composition root egyetlen `TickEngine`-t futtat. A motor-döntést befolyásoló **authority és owned layer-state** nem költözhet rétegenkénti threadbe/processzbe, és layer-kódban továbbra sincs sleep, falióra, rejtett I/O vagy modulglobális mutable state. Drága, determinisztikus **pure computation** külön worker-processzbe tehető kizárólag a runtime/adapter szélen, composition-root által injektált typed compute-port mögött. A worker csak lezárt immutable snapshotból számolhat; nem birtokolhat command-, mission-, navigation-, lifecycle-, safety-, motor- vagy GPIO-authorityt. Az owning layer az eredményt csak előre meghatározott tick-határon fogadhatja el; worker-hiány, deadline-miss, context-eltérés vagy túl öreg elfogadott terv fail-closed hiba. A checkpointnak az esetleges pending immutable kérést és determinisztikus release tickjét is rögzítenie kell, hogy replay ugyanazt a pure számítást ugyanazon a handoff ticken tegye láthatóvá.
'''
    return replace_once(text, old, new, "V3 pure compute worker boundary")

def main() -> int:
    repo = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    if not (repo / "v3").is_dir():
        raise RuntimeError(f"not an R2B4 repo: {repo}")
    if (repo / "v3/adapters/l6_planner_process.py").exists():
        existing = (repo / "v3/adapters/l6_planner_process.py").read_text(encoding="utf-8")
        if MARKER in existing or "class ProcessTrajectoryRolloutBackend" in existing:
            print("already installed: asynchronous L6 planner")
            return 0
        raise RuntimeError("v3/adapters/l6_planner_process.py already exists with unknown content")

    head = git_head(repo)
    if head is not None and head != EXPECTED_BASE:
        print(f"WARNING: package was built for {EXPECTED_BASE}, current HEAD is {head}")
        print("Source anchors will still prevent a blind/partial install.")

    targets = [
        "v3/layers/l6_navigation.py",
        "v3/composition/native_control.py",
        "v3/composition/resident_live_control.py",
        "v3/composition/resident_physical_control.py",
        "v3_runtime.py",
        "v3_hardware_runtime.py",
        "v3_bounded_config.py",
        "v3/replay.py",
        "tests/v3_validation_helpers.py",
        "conf/vezerles.json",
        "STRUKTURALIS_RETEGEK_V3.md",
        "v3/adapters/l6_planner_process.py",
        "tests/test_v3_async_l6_planner.py",
    ]
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    for relative in targets:
        backup(repo, relative)

    originals = {
        relative: (repo / relative).read_text(encoding="utf-8")
        for relative in targets
        if (repo / relative).is_file()
    }
    try:
        save(repo, "v3/layers/l6_navigation.py", patch_l6(load(repo, "v3/layers/l6_navigation.py")))
        save(repo, "v3/adapters/l6_planner_process.py", f'# {MARKER}\n' + NEW_ASYNC_MODULE)
        save(repo, "v3/composition/native_control.py", patch_native_control(load(repo, "v3/composition/native_control.py")))
        save(repo, "v3/composition/resident_live_control.py", patch_resident_live(load(repo, "v3/composition/resident_live_control.py")))
        save(repo, "v3/composition/resident_physical_control.py", patch_resident_physical(load(repo, "v3/composition/resident_physical_control.py")))
        save(repo, "v3_runtime.py", patch_runtime(load(repo, "v3_runtime.py")))
        save(repo, "v3_hardware_runtime.py", patch_hardware_runtime(load(repo, "v3_hardware_runtime.py")))
        save(repo, "v3_bounded_config.py", patch_bounded_config(load(repo, "v3_bounded_config.py")))
        save(repo, "v3/replay.py", patch_replay(load(repo, "v3/replay.py")))
        save(repo, "tests/v3_validation_helpers.py", patch_validation_helpers(load(repo, "tests/v3_validation_helpers.py")))
        save(repo, "conf/vezerles.json", patch_control_json(load(repo, "conf/vezerles.json")))
        save(repo, "STRUKTURALIS_RETEGEK_V3.md", patch_architecture(load(repo, "STRUKTURALIS_RETEGEK_V3.md")))
        save(repo, "tests/test_v3_async_l6_planner.py", NEW_TEST)
        compile_targets = [
            "v3/layers/l6_navigation.py",
            "v3/adapters/l6_planner_process.py",
            "v3/composition/native_control.py",
            "v3/composition/resident_live_control.py",
            "v3/composition/resident_physical_control.py",
            "v3_runtime.py",
            "v3_hardware_runtime.py",
            "v3_bounded_config.py",
            "v3/replay.py",
            "tests/test_v3_async_l6_planner.py",
        ]
        subprocess.run(
            [sys.executable, "-m", "py_compile", *[str(repo / item) for item in compile_targets]],
            check=True,
        )
    except BaseException:
        for relative, content in originals.items():
            save(repo, relative, content)
        for relative in targets:
            if relative not in originals:
                path = repo / relative
                if path.exists():
                    path.unlink()
        raise

    print("installed: asynchronous L6 planner / deterministic latest-plan handoff")
    print("next: bash validate_upgrade.sh", repo)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
