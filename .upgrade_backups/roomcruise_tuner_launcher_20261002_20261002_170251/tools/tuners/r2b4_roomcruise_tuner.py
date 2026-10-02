#!/usr/bin/env python3
"""Deterministic, non-actuating RoomCruise tuner for the R2B4 V3 stack.

The tuner loads the canonical production configuration through ConfigResolver,
creates validated in-memory navigation variants, and evaluates them through the
headless L5-L9 MissionNavigationComposition.  It never opens hardware, starts
the resident runtime, writes the production config, or owns command/safety
runtime authority.

Default search is coordinate-descent rather than a Cartesian grid.  This keeps
planner work bounded on Raspberry Pi while still testing the coupled parameters
that control RoomCruise turning authority and straight-line bias.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import subprocess
import sys
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Iterable, Sequence

PROJECT_ROOT = Path(__file__).resolve().parents[2]
RESULT_SCHEMA = "R2B4_ROOMCRUISE_TUNER_RESULT_V1"
TUNER_VERSION = 1

# Source-first baseline of the current operator -> control_cli -> EXPLORE path.
# The values are intentionally represented as candidate inputs rather than
# production config authority because v3/control_cli.py currently owns them.
DEFAULT_COMMAND_MAX_V_MPS = 0.30
DEFAULT_COMMAND_MAX_OMEGA_RAD_S = 0.60

_EPS = 1e-9


@dataclass(frozen=True, slots=True)
class RectObstacle:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def __post_init__(self) -> None:
        if not self.x_min < self.x_max or not self.y_min < self.y_max:
            raise ValueError("invalid rectangle obstacle")


@dataclass(frozen=True, slots=True)
class Scenario:
    name: str
    bounds: tuple[float, float, float, float]
    start: tuple[float, float, float]
    obstacles: tuple[RectObstacle, ...] = ()
    localization_state: str = "GOOD"

    def __post_init__(self) -> None:
        x_min, x_max, y_min, y_max = self.bounds
        if not x_min < x_max or not y_min < y_max:
            raise ValueError("invalid scenario bounds")
        x, y, _ = self.start
        if not x_min < x < x_max or not y_min < y < y_max:
            raise ValueError("scenario start must be inside bounds")
        if self.localization_state not in {"GOOD", "DEGRADED"}:
            raise ValueError("unsupported localization state")


@dataclass(frozen=True, slots=True)
class Candidate:
    command_max_v_mps: float
    command_max_omega_rad_s: float
    minimum_planning_speed_mps: float
    rollout_linear_samples: int
    rollout_angular_samples: int
    localization_degraded_speed_scale: float
    local_goal_novelty_weight: float
    local_goal_clearance_weight: float
    local_goal_forward_weight: float
    smoothness_weight: float
    novelty_weight: float
    localization_observability_weight: float

    def __post_init__(self) -> None:
        for name in (
            "command_max_v_mps",
            "command_max_omega_rad_s",
            "minimum_planning_speed_mps",
            "localization_degraded_speed_scale",
            "local_goal_novelty_weight",
            "local_goal_clearance_weight",
            "local_goal_forward_weight",
            "smoothness_weight",
            "novelty_weight",
            "localization_observability_weight",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite numeric")
        if self.command_max_v_mps <= 0.0 or self.command_max_omega_rad_s <= 0.0:
            raise ValueError("command envelope must be positive")
        if self.minimum_planning_speed_mps <= 0.0:
            raise ValueError("minimum planning speed must be positive")
        if not 0.0 < self.localization_degraded_speed_scale <= 1.0:
            raise ValueError("degraded scale must be in (0,1]")
        if self.rollout_linear_samples < 2 or self.rollout_angular_samples < 3:
            raise ValueError("rollout axes are too small")
        count = self.rollout_linear_samples * self.rollout_angular_samples
        if not 30 <= count <= 60:
            raise ValueError("rollout candidate count must stay in [30, 60]")
        for name in (
            "local_goal_novelty_weight",
            "local_goal_clearance_weight",
            "local_goal_forward_weight",
            "smoothness_weight",
            "novelty_weight",
            "localization_observability_weight",
        ):
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} cannot be negative")

    @property
    def key(self) -> tuple[object, ...]:
        return tuple(asdict(self).items())


@dataclass(frozen=True, slots=True)
class ScenarioMetrics:
    scenario: str
    ticks: int
    moving_ratio: float
    curved_ratio: float
    straight_ratio: float
    pivot_ratio: float
    reverse_ratio: float
    stopped_ratio: float
    heading_bin_ratio: float
    unique_coverage_cells: int
    distance_m: float
    steering_reversals_per_m: float
    no_progress_events: int
    p05_clearance_m: float
    minimum_clearance_m: float
    collision: bool
    score: float


@dataclass(frozen=True, slots=True)
class CandidateResult:
    candidate: Candidate
    score: float
    hard_fail: bool
    scenarios: tuple[ScenarioMetrics, ...]


@dataclass(frozen=True, slots=True)
class _Axis:
    name: str
    variants: tuple[dict[str, object], ...]


def scenarios() -> tuple[Scenario, ...]:
    """Deterministic living-room-like geometry used for every candidate."""
    room = (-2.5, 2.5, -2.0, 2.0)
    return (
        Scenario("open_room", room, (-1.55, -0.55, 0.0)),
        Scenario(
            "coffee_table",
            room,
            (-1.70, -0.85, 0.10),
            (
                RectObstacle(-0.45, 0.45, -0.35, 0.35),
                RectObstacle(1.35, 2.10, -1.85, -1.25),
            ),
        ),
        Scenario(
            "asymmetric_opening",
            room,
            (-1.75, 0.0, 0.0),
            (
                RectObstacle(-0.10, 0.35, 0.45, 1.95),
                RectObstacle(-0.10, 0.35, -1.95, -0.75),
                RectObstacle(1.35, 2.25, -0.25, 0.45),
            ),
        ),
        Scenario(
            "chair_cluster",
            room,
            (-1.65, -1.05, 0.25),
            (
                RectObstacle(-0.20, 0.20, -0.20, 0.20),
                RectObstacle(0.65, 0.95, 0.35, 0.70),
                RectObstacle(0.75, 1.05, -0.75, -0.40),
                RectObstacle(-0.85, -0.55, 0.65, 1.00),
            ),
        ),
        Scenario(
            "degraded_opening",
            room,
            (-1.75, 0.0, 0.0),
            (
                RectObstacle(-0.05, 0.35, 0.50, 1.95),
                RectObstacle(-0.05, 0.35, -1.95, -0.65),
            ),
            localization_state="DEGRADED",
        ),
    )


def _baseline_candidate(control, command_max_v: float, command_max_omega: float) -> Candidate:
    nav = control.navigation
    return Candidate(
        command_max_v_mps=command_max_v,
        command_max_omega_rad_s=command_max_omega,
        minimum_planning_speed_mps=nav.minimum_planning_speed_mps,
        rollout_linear_samples=nav.rollout_linear_samples,
        rollout_angular_samples=nav.rollout_angular_samples,
        localization_degraded_speed_scale=nav.localization_degraded_speed_scale,
        local_goal_novelty_weight=nav.local_goal_novelty_weight,
        local_goal_clearance_weight=nav.local_goal_clearance_weight,
        local_goal_forward_weight=nav.local_goal_forward_weight,
        smoothness_weight=nav.smoothness_weight,
        novelty_weight=nav.novelty_weight,
        localization_observability_weight=nav.localization_observability_weight,
    )


def _axis_specs(baseline: Candidate) -> tuple[_Axis, ...]:
    """Bounded source-first search axes; values outside production envelopes are absent."""
    return (
        _Axis("command_max_v_mps", tuple({"command_max_v_mps": v} for v in _unique(
            baseline.command_max_v_mps, 0.35, 0.45))),
        _Axis("command_max_omega_rad_s", tuple({"command_max_omega_rad_s": v} for v in _unique(
            baseline.command_max_omega_rad_s, 0.90, 1.20))),
        _Axis("minimum_planning_speed_mps", tuple({"minimum_planning_speed_mps": v} for v in _unique(
            baseline.minimum_planning_speed_mps, 0.17, 0.19))),
        _Axis("rollout_shape", tuple(
            {"rollout_linear_samples": linear, "rollout_angular_samples": angular}
            for linear, angular in _unique_pairs(
                (baseline.rollout_linear_samples, baseline.rollout_angular_samples),
                (5, 11),
            )
        )),
        _Axis("localization_degraded_speed_scale", tuple(
            {"localization_degraded_speed_scale": v}
            for v in _unique(baseline.localization_degraded_speed_scale, 0.55, 0.70)
        )),
        _Axis("local_goal_forward_weight", tuple({"local_goal_forward_weight": v} for v in _unique(
            baseline.local_goal_forward_weight, 0.10, 0.05))),
        _Axis("local_goal_novelty_weight", tuple({"local_goal_novelty_weight": v} for v in _unique(
            baseline.local_goal_novelty_weight, 0.55, 0.65))),
        _Axis("local_goal_clearance_weight", tuple({"local_goal_clearance_weight": v} for v in _unique(
            baseline.local_goal_clearance_weight, 0.15))),
        _Axis("smoothness_weight", tuple({"smoothness_weight": v} for v in _unique(
            baseline.smoothness_weight, 0.15, 0.12))),
        _Axis("novelty_weight", tuple({"novelty_weight": v} for v in _unique(
            baseline.novelty_weight, 0.15, 0.18))),
        _Axis("localization_observability_weight", tuple(
            {"localization_observability_weight": v}
            for v in _unique(baseline.localization_observability_weight, 0.12, 0.08)
        )),
    )


def _unique(*values: float) -> tuple[float, ...]:
    result: list[float] = []
    for value in values:
        if not any(abs(value - previous) <= 1e-12 for previous in result):
            result.append(float(value))
    return tuple(result)


def _unique_pairs(*values: tuple[int, int]) -> tuple[tuple[int, int], ...]:
    result: list[tuple[int, int]] = []
    for value in values:
        if value not in result:
            result.append(value)
    return tuple(result)


def _candidate_with(candidate: Candidate, changes: dict[str, object]) -> Candidate:
    return replace(candidate, **changes)


def _git_head(root: Path) -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=root, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _load_control(root: Path):
    from v3.config import ConfigResolver

    return ConfigResolver.for_project(root).resolve().runtime.composition.live_control.control


def _composition_for(control, candidate: Candidate):
    from v3.composition.mission_navigation import MissionNavigationComposition

    nav = replace(
        control.navigation,
        minimum_planning_speed_mps=candidate.minimum_planning_speed_mps,
        rollout_linear_samples=candidate.rollout_linear_samples,
        rollout_angular_samples=candidate.rollout_angular_samples,
        localization_degraded_speed_scale=candidate.localization_degraded_speed_scale,
        local_goal_novelty_weight=candidate.local_goal_novelty_weight,
        local_goal_clearance_weight=candidate.local_goal_clearance_weight,
        local_goal_forward_weight=candidate.local_goal_forward_weight,
        smoothness_weight=candidate.smoothness_weight,
        novelty_weight=candidate.novelty_weight,
        localization_observability_weight=candidate.localization_observability_weight,
    )
    # Synchronous non-actuating planner: same L5-L9 semantics, no worker/process
    # timing noise and no runtime edge. release_delay is irrelevant without a
    # backend, but None keeps the synchronous path explicit.
    async_config = replace(
        control.async_l6,
        enabled=False,
        completion_inputs=False,
        release_delay_ns=None,
    )
    return MissionNavigationComposition(
        mission_config=control.mission,
        navigation_config=nav,
        async_config=async_config,
        selection_config=control.motion_selection,
        motion_config=control.motion_realization,
        constraints_config=control.operational_constraints,
    ), nav


def _rasterized_cells(scenario: Scenario, resolution_m: float):
    from v3.contracts import CostmapCell

    x_min, x_max, y_min, y_max = scenario.bounds
    wall = resolution_m * 0.75
    gx0 = math.floor(x_min / resolution_m)
    gx1 = math.ceil(x_max / resolution_m)
    gy0 = math.floor(y_min / resolution_m)
    gy1 = math.ceil(y_max / resolution_m)
    cells = []
    for gx in range(gx0, gx1 + 1):
        x = gx * resolution_m
        for gy in range(gy0, gy1 + 1):
            y = gy * resolution_m
            occupied = (
                abs(x - x_min) <= wall
                or abs(x - x_max) <= wall
                or abs(y - y_min) <= wall
                or abs(y - y_max) <= wall
                or any(
                    obstacle.x_min - wall <= x <= obstacle.x_max + wall
                    and obstacle.y_min - wall <= y <= obstacle.y_max + wall
                    for obstacle in scenario.obstacles
                )
            )
            if occupied:
                cells.append(CostmapCell(gx, gy, 8))
    return tuple(cells)


def _rect_clearance(x: float, y: float, obstacle: RectObstacle) -> float:
    dx = max(obstacle.x_min - x, 0.0, x - obstacle.x_max)
    dy = max(obstacle.y_min - y, 0.0, y - obstacle.y_max)
    if dx > 0.0 or dy > 0.0:
        return math.hypot(dx, dy)
    return -min(x - obstacle.x_min, obstacle.x_max - x, y - obstacle.y_min, obstacle.y_max - y)


def _physical_clearance(scenario: Scenario, x: float, y: float, radius: float) -> float:
    x_min, x_max, y_min, y_max = scenario.bounds
    wall_clearance = min(x - x_min, x_max - x, y - y_min, y_max - y) - radius
    if not scenario.obstacles:
        return wall_clearance
    furniture = min(_rect_clearance(x, y, obstacle) - radius for obstacle in scenario.obstacles)
    return min(wall_clearance, furniture)


def _percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(math.floor((len(ordered) - 1) * fraction))))
    return ordered[index]


def simulate_candidate(
    control,
    candidate: Candidate,
    scenario: Scenario,
    *,
    ticks: int,
    dt_s: float = 0.02,
) -> ScenarioMetrics:
    """Run one deterministic closed-loop, non-actuating L5-L9 RoomCruise scenario."""
    from v3.contracts import (
        CommandMode,
        CommandRequest,
        DataField,
        LOCAL_FRAME_ID,
        LocalizationQuality,
        QualityState,
        RobotEstimate,
        RobotRelativeGeometry,
        RollingLocalCostmap,
        TickContext,
        WorldSnapshot,
    )
    from v3.composition.mission_navigation import MissionNavigationInputs

    if ticks <= 0:
        raise ValueError("ticks must be positive")
    composition, nav = _composition_for(control, candidate)
    resolution = control.world_model.local_costmap_resolution_m
    occupied = _rasterized_cells(scenario, resolution)
    if len(occupied) > control.world_model.local_costmap_max_cells:
        raise ValueError(
            f"scenario {scenario.name} produces {len(occupied)} occupied cells, "
            f"above production max {control.world_model.local_costmap_max_cells}"
        )

    x, y, yaw = scenario.start
    v = omega = 0.0
    now_ns = 1_000_000_000
    dt_ns = int(round(dt_s * 1e9))
    coverage: set[tuple[int, int]] = set()
    headings: set[int] = set()
    clearances: list[float] = []
    moving = curved = straight = pivot = reverse = stopped = 0
    distance = 0.0
    steering_reversals = 0
    previous_turn_sign = 0
    no_progress_events = 0
    collision = False
    robot_radius = 0.5 * math.hypot(nav.footprint_length_m, nav.footprint_width_m) + nav.footprint_safety_margin_m

    quality_state = QualityState[scenario.localization_state]
    quality = LocalizationQuality(
        local_translation=quality_state,
        heading=QualityState.GOOD,
        global_position=QualityState.GOOD,
        local_sigma_m=0.025 if quality_state is QualityState.GOOD else 0.075,
        global_sigma_m=0.025,
        yaw_sigma_rad=0.02,
        encoder_age_ns=0,
        imu_age_ns=0,
        lidar_age_ns=0,
        global_fix_age_ns=0,
        local_pose_continuous=True,
        observability=0.85 if quality_state is QualityState.GOOD else 0.45,
    )

    for tick in range(ticks):
        if tick:
            x += v * dt_s * math.cos(yaw + 0.5 * omega * dt_s)
            y += v * dt_s * math.sin(yaw + 0.5 * omega * dt_s)
            yaw = math.atan2(math.sin(yaw + omega * dt_s), math.cos(yaw + omega * dt_s))
            now_ns += dt_ns
            distance += abs(v) * dt_s

        clearance = _physical_clearance(scenario, x, y, robot_radius)
        clearances.append(clearance)
        if clearance < 0.0:
            collision = True
            break

        context = TickContext(tick, now_ns)
        covariance = [0.0] * 25
        covariance[0] = covariance[6] = quality.local_sigma_m ** 2
        covariance[12] = quality.yaw_sigma_rad ** 2
        estimate = RobotEstimate(
            context,
            LOCAL_FRAME_ID,
            x,
            y,
            yaw,
            v,
            omega,
            tuple(covariance),
            localization_quality=quality,
        )
        costmap = RollingLocalCostmap(
            LOCAL_FRAME_ID,
            1,
            resolution,
            control.world_model.local_costmap_radius_m,
            occupied,
            tick + 1,
            0,
        )
        world = WorldSnapshot(
            context,
            LOCAL_FRAME_ID,
            1,
            (),
            0,
            costmap,
            robot_relative_geometry=RobotRelativeGeometry(
                "ROBOT_BASE", tick + 1, now_ns, max(1, len(occupied)), 0
            ),
        )
        command = CommandRequest(
            context,
            "roomcruise-tuner",
            CommandMode.EXPLORE,
            (
                DataField("max_v_mps", candidate.command_max_v_mps),
                DataField("max_omega_rad_s", candidate.command_max_omega_rad_s),
            ),
            tick + 1,
        )
        trace = composition.run_tick(MissionNavigationInputs(context, command, estimate, world))
        v = trace.constrained.allowed_v_mps
        omega = trace.constrained.allowed_omega_rad_s

        active = abs(v) + abs(omega) > 1e-6
        if active:
            moving += 1
        else:
            stopped += 1
        if v > 0.02 and abs(omega) > 0.10:
            curved += 1
        elif v > 0.02:
            straight += 1
        if abs(v) <= 0.02 and abs(omega) > 0.10:
            pivot += 1
        if v < -0.02:
            reverse += 1

        turn_sign = 1 if omega >= 0.05 else -1 if omega <= -0.05 else 0
        if turn_sign and previous_turn_sign and turn_sign != previous_turn_sign:
            steering_reversals += 1
        if turn_sign:
            previous_turn_sign = turn_sign

        coverage.add((math.floor(x / nav.coverage_cell_size_m), math.floor(y / nav.coverage_cell_size_m)))
        heading_bin = int(((yaw + math.pi) % (2.0 * math.pi)) / (2.0 * math.pi) * 16.0) % 16
        headings.add(heading_bin)
        reason = trace.navigation.reason or trace.objective.selection_reason
        if reason and "NO_PROGRESS" in reason:
            no_progress_events += 1

    completed_ticks = max(1, len(clearances))
    moving_denominator = max(1, moving)
    forward_denominator = max(1, curved + straight)
    moving_ratio = moving / completed_ticks
    curved_ratio = curved / forward_denominator
    straight_ratio = straight / forward_denominator
    pivot_ratio = pivot / completed_ticks
    reverse_ratio = reverse / completed_ticks
    stopped_ratio = stopped / completed_ticks
    heading_ratio = len(headings) / 16.0
    p05 = _percentile(clearances, 0.05)
    minimum_clearance = min(clearances) if clearances else -1.0
    chatter = steering_reversals / max(distance, 0.25)

    curve_balance = max(0.0, 1.0 - abs(curved_ratio - 0.55) / 0.55)
    coverage_score = min(1.0, len(coverage) / 16.0)
    clearance_score = min(1.0, max(0.0, p05) / 0.35)
    straight_excess = max(0.0, straight_ratio - 0.45)
    no_progress_ratio = no_progress_events / completed_ticks
    score = (
        2.00 * curve_balance
        + 1.60 * coverage_score
        + 0.65 * heading_ratio
        + 0.55 * moving_ratio
        + 0.45 * clearance_score
        - 1.40 * reverse_ratio
        - 1.80 * stopped_ratio
        - 1.10 * straight_excess
        - 0.18 * chatter
        - 2.00 * no_progress_ratio
    )
    if collision:
        score = -1000.0 - completed_ticks / max(1, ticks)

    return ScenarioMetrics(
        scenario=scenario.name,
        ticks=completed_ticks,
        moving_ratio=moving_ratio,
        curved_ratio=curved_ratio,
        straight_ratio=straight_ratio,
        pivot_ratio=pivot_ratio,
        reverse_ratio=reverse_ratio,
        stopped_ratio=stopped_ratio,
        heading_bin_ratio=heading_ratio,
        unique_coverage_cells=len(coverage),
        distance_m=distance,
        steering_reversals_per_m=chatter,
        no_progress_events=no_progress_events,
        p05_clearance_m=p05,
        minimum_clearance_m=minimum_clearance,
        collision=collision,
        score=score,
    )


def evaluate_candidate(
    control,
    candidate: Candidate,
    selected_scenarios: Sequence[Scenario],
    *,
    ticks: int,
) -> CandidateResult:
    metrics = tuple(
        simulate_candidate(control, candidate, scenario, ticks=ticks)
        for scenario in selected_scenarios
    )
    hard_fail = any(item.collision for item in metrics)
    score = statistics.fmean(item.score for item in metrics)
    if hard_fail:
        score -= 1000.0
    return CandidateResult(candidate, score, hard_fail, metrics)


def tune(
    control,
    baseline: Candidate,
    selected_scenarios: Sequence[Scenario],
    *,
    ticks: int,
    passes: int,
) -> tuple[CandidateResult, tuple[CandidateResult, ...], tuple[dict[str, object], ...]]:
    if passes <= 0:
        raise ValueError("passes must be positive")
    cache: dict[tuple[object, ...], CandidateResult] = {}
    history: list[dict[str, object]] = []

    def measured(candidate: Candidate) -> CandidateResult:
        result = cache.get(candidate.key)
        if result is None:
            result = evaluate_candidate(control, candidate, selected_scenarios, ticks=ticks)
            cache[candidate.key] = result
        return result

    current = baseline
    current_result = measured(current)
    for pass_index in range(passes):
        changed = False
        for axis in _axis_specs(baseline):
            variants = [_candidate_with(current, changes) for changes in axis.variants]
            if all(item.key != current.key for item in variants):
                variants.append(current)
            results = [measured(item) for item in variants]
            winner = max(
                results,
                key=lambda result: (
                    not result.hard_fail,
                    result.score,
                    -abs(result.candidate.command_max_v_mps - baseline.command_max_v_mps),
                    repr(result.candidate.key),
                ),
            )
            history.append(
                {
                    "pass": pass_index + 1,
                    "axis": axis.name,
                    "winner_score": winner.score,
                    "winner": asdict(winner.candidate),
                    "evaluated": len(results),
                }
            )
            if winner.candidate.key != current.key:
                current = winner.candidate
                current_result = winner
                changed = True
        if not changed:
            break
    ranked = tuple(sorted(cache.values(), key=lambda item: (item.hard_fail, -item.score, repr(item.candidate.key))))
    return current_result, ranked, tuple(history)


def _config_patch(baseline: Candidate, candidate: Candidate) -> dict[str, object]:
    nav = {}
    for name in (
        "minimum_planning_speed_mps",
        "rollout_linear_samples",
        "rollout_angular_samples",
        "localization_degraded_speed_scale",
        "local_goal_novelty_weight",
        "local_goal_clearance_weight",
        "local_goal_forward_weight",
        "smoothness_weight",
        "novelty_weight",
        "localization_observability_weight",
    ):
        old = getattr(baseline, name)
        new = getattr(candidate, name)
        if old != new:
            nav[name] = new
    return {"layers": {"navigation": nav}} if nav else {}


def _command_patch(baseline: Candidate, candidate: Candidate) -> dict[str, object]:
    result = {}
    if candidate.command_max_v_mps != baseline.command_max_v_mps:
        result["explore_default_max_v_mps"] = candidate.command_max_v_mps
    if candidate.command_max_omega_rad_s != baseline.command_max_omega_rad_s:
        result["explore_default_max_omega_rad_s"] = candidate.command_max_omega_rad_s
    return result


def emit_config(root: Path, patch: dict[str, object], destination: Path) -> None:
    source = (root / "conf" / "vezerles.json").resolve()
    destination = destination.resolve()
    if destination == source:
        raise ValueError("tuner refuses to overwrite production conf/vezerles.json")
    value = json.loads(source.read_text(encoding="utf-8"))
    nav_changes = patch.get("layers", {}).get("navigation", {}) if patch else {}
    if nav_changes:
        value["layers"]["navigation"].update(nav_changes)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _jsonable_result(result: CandidateResult) -> dict[str, object]:
    return {
        "candidate": asdict(result.candidate),
        "score": result.score,
        "hard_fail": result.hard_fail,
        "scenarios": [asdict(item) for item in result.scenarios],
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Non-actuating source-first RoomCruise tuner over the canonical L5-L9 stack"
    )
    parser.add_argument("--project-root", type=Path, default=PROJECT_ROOT)
    parser.add_argument("--profile", choices=("quick", "full"), default="quick")
    parser.add_argument("--passes", type=int)
    parser.add_argument("--ticks", type=int)
    parser.add_argument("--scenarios", help="comma-separated scenario names")
    parser.add_argument("--output", type=Path, default=Path("/tmp/r2b4_roomcruise_tuner.json"))
    parser.add_argument("--emit-config", type=Path)
    parser.add_argument("--top", type=int, default=8)
    parser.add_argument("--baseline-command-max-v", type=float, default=DEFAULT_COMMAND_MAX_V_MPS)
    parser.add_argument("--baseline-command-max-omega", type=float, default=DEFAULT_COMMAND_MAX_OMEGA_RAD_S)
    parser.add_argument("--list-scenarios", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    root = args.project_root.resolve()
    if args.list_scenarios:
        for scenario in scenarios():
            print(scenario.name)
        return 0
    if not (root / "AGENTS.md").is_file() or not (root / "conf" / "vezerles.json").is_file():
        raise SystemExit(f"not an R2B4 project root: {root}")

    control = _load_control(root)
    baseline = _baseline_candidate(
        control,
        float(args.baseline_command_max_v),
        float(args.baseline_command_max_omega),
    )
    available = {scenario.name: scenario for scenario in scenarios()}
    if args.scenarios:
        names = tuple(name.strip() for name in args.scenarios.split(",") if name.strip())
        unknown = sorted(set(names) - available.keys())
        if unknown:
            raise SystemExit(f"unknown scenarios: {', '.join(unknown)}")
        selected = tuple(available[name] for name in names)
    elif args.profile == "quick":
        selected = tuple(available[name] for name in ("open_room", "coffee_table", "asymmetric_opening", "degraded_opening"))
    else:
        selected = scenarios()

    ticks = args.ticks if args.ticks is not None else (220 if args.profile == "quick" else 500)
    passes = args.passes if args.passes is not None else (1 if args.profile == "quick" else 2)
    if ticks <= 0 or passes <= 0 or args.top <= 0:
        raise SystemExit("ticks, passes and top must be positive")

    baseline_result = evaluate_candidate(control, baseline, selected, ticks=ticks)
    winner, ranked, history = tune(
        control,
        baseline,
        selected,
        ticks=ticks,
        passes=passes,
    )
    patch = _config_patch(baseline, winner.candidate)
    command_patch = _command_patch(baseline, winner.candidate)
    payload = {
        "schema": RESULT_SCHEMA,
        "tuner_version": TUNER_VERSION,
        "project_root": str(root),
        "git_head": _git_head(root),
        "non_actuating": True,
        "profile": args.profile,
        "ticks_per_scenario": ticks,
        "passes": passes,
        "scenario_names": [item.name for item in selected],
        "baseline": _jsonable_result(baseline_result),
        "winner": _jsonable_result(winner),
        "delta_score": winner.score - baseline_result.score,
        "config_patch": patch,
        "command_ingress_patch": command_patch,
        "command_ingress_note": (
            "RoomCruise operator currently reaches v3.control_cli explore defaults; "
            "command envelope recommendations are source changes, not vezerles.json fields."
        ),
        "ranking": [_jsonable_result(item) for item in ranked[: args.top]],
        "search_history": list(history),
        "limitations": [
            "Synthetic L5-L9 tuning does not replace live MCAP/evidence acceptance.",
            "L0-L4 sensor/localization dynamics and L10-L12 actuator/safety dynamics are not simulated here.",
            "The tuner never changes speed_map or encoder reliability authority.",
        ],
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if args.emit_config is not None:
        emit_config(root, patch, args.emit_config)
    print(json.dumps({
        "schema": RESULT_SCHEMA,
        "status": "PASS" if not winner.hard_fail else "NO_SAFE_WINNER",
        "baseline_score": baseline_result.score,
        "winner_score": winner.score,
        "delta_score": winner.score - baseline_result.score,
        "output": str(args.output),
        "config_patch": patch,
        "command_ingress_patch": command_patch,
    }, sort_keys=True))
    return 0 if not winner.hard_fail else 1


if __name__ == "__main__":
    raise SystemExit(main())
