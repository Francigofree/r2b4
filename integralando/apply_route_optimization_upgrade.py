#!/usr/bin/env python3
"""Transactional R2B4 speed-aware route optimization upgrade.

Source authority:
  Francigofree/r2b4
  expected main HEAD: aa882aa3a2202b665ae0be54858336e8cc1099c0

Scope:
  - L6 speed-aware soft-clearance trajectory scoring
  - EXPLORE-only adaptive local-goal distance
  - L7 clearance-guarded temporal continuity
  - strict vezerles.json parameters
  - replay migration preserving historical config semantics
  - focused regression tests

No L12 safety threshold or L9 platform limit is relaxed.
"""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import textwrap
import time
from pathlib import Path

EXPECTED_HEAD = "aa882aa3a2202b665ae0be54858336e8cc1099c0"
TARGETS = (
    "v3/layers/l6_navigation.py",
    "v3/layers/l7_motion_selection.py",
    "v3/replay.py",
    "conf/vezerles.json",
)
NEW_TEST = "tests/core/test_v3_route_optimization_config.py"


def run(*args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        args,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if check and result.returncode != 0:
        print(f"COMMAND FAILED ({result.returncode}): {' '.join(args)}", file=sys.stderr)
        if result.stdout:
            print("--- stdout ---", file=sys.stderr)
            print(result.stdout, file=sys.stderr, end="" if result.stdout.endswith("\n") else "\n")
        if result.stderr:
            print("--- stderr ---", file=sys.stderr)
            print(result.stderr, file=sys.stderr, end="" if result.stderr.endswith("\n") else "\n")
        raise subprocess.CalledProcessError(
            result.returncode, args, output=result.stdout, stderr=result.stderr
        )
    return result


def git(*args: str, check: bool = True) -> str:
    return run("git", *args, check=check).stdout.strip()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one anchor, found {count}")
    return text.replace(old, new, 1)


def replace_method(text: str, start_name: str, next_name: str, new_body: str, label: str) -> str:
    pattern = re.compile(
        rf"\n    def {re.escape(start_name)}\([\s\S]*?(?=\n    def {re.escape(next_name)}\()"
    )
    matches = list(pattern.finditer(text))
    if len(matches) != 1:
        raise RuntimeError(f"{label}: expected one method block, found {len(matches)}")
    normalized = textwrap.dedent(new_body).strip("\n")
    replacement = "\n" + textwrap.indent(normalized, "    ") + "\n"
    return text[: matches[0].start()] + replacement + text[matches[0].end() :]


def patch_l6(text: str) -> str:
    text = replace_once(
        text,
        "_MOTION_EPSILON = 1e-9\n",
        r'''_MOTION_EPSILON = 1e-9


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


def _explore_goal_distances(config: "NavigationConfig") -> tuple[float, ...]:
    """Far-to-near EXPLORE goal candidates; one sample preserves legacy behavior."""
    count = config.explore_local_goal_distance_samples
    maximum = config.explore_local_goal_max_distance_m
    minimum = config.explore_local_goal_min_distance_m
    if count == 1 or maximum - minimum <= _POINT_CLEARANCE_EPSILON_M:
        return (maximum,)
    step = (maximum - minimum) / (count - 1)
    return tuple(maximum - index * step for index in range(count))
''',
        "l6:soft-clearance-helpers",
    )

    text = replace_once(
        text,
        '''    wheel_limits: WheelMotionLimits = WheelMotionLimits()\n    minimum_planning_speed_mps: float = 0.15\n''',
        '''    wheel_limits: WheelMotionLimits = WheelMotionLimits()\n    minimum_planning_speed_mps: float = 0.15\n    # Soft navigation envelope. L12 remains the independent hard safety gate.\n    speed_clearance_enabled: bool = False\n    speed_clearance_low_speed_mps: float = 0.20\n    speed_clearance_high_speed_mps: float = 0.40\n    speed_clearance_low_m: float = 0.18\n    speed_clearance_high_m: float = 0.30\n    speed_clearance_barrier_power: float = 2.0\n    speed_clearance_penalty_weight: float = 0.0\n    # EXPLORE-only adaptive waypoint range; NAVIGATE/FOLLOW keep local_goal_distance_m.\n    explore_local_goal_min_distance_m: float = 0.60\n    explore_local_goal_max_distance_m: float = 0.60\n    explore_local_goal_distance_samples: int = 1\n    explore_local_goal_distance_weight: float = 0.0\n''',
        "l6:config-fields",
    )

    text = replace_once(
        text,
        '''        if self.minimum_planning_speed_mps > self.wheel_limits.maximum_mps:\n            raise ValueError("planner minimum exceeds calibrated maximum")\n        if (\n            not isinstance(self.footprint_safety_margin_m, (int, float))\n''',
        '''        if self.minimum_planning_speed_mps > self.wheel_limits.maximum_mps:\n            raise ValueError("planner minimum exceeds calibrated maximum")\n        if type(self.speed_clearance_enabled) is not bool:\n            raise ValueError("speed_clearance_enabled must be boolean")\n        for name in (\n            "speed_clearance_low_speed_mps",\n            "speed_clearance_high_speed_mps",\n            "speed_clearance_low_m",\n            "speed_clearance_high_m",\n            "speed_clearance_barrier_power",\n            "explore_local_goal_min_distance_m",\n            "explore_local_goal_max_distance_m",\n        ):\n            value = getattr(self, name)\n            if (\n                isinstance(value, bool)\n                or not isinstance(value, (int, float))\n                or not math.isfinite(value)\n                or value <= 0.0\n            ):\n                raise ValueError(f"{name} must be finite and positive")\n        if self.speed_clearance_high_speed_mps <= self.speed_clearance_low_speed_mps:\n            raise ValueError("speed-clearance high speed must exceed low speed")\n        if self.speed_clearance_high_m < self.speed_clearance_low_m:\n            raise ValueError("speed-clearance high-speed clearance must not shrink")\n        if self.speed_clearance_enabled and self.speed_clearance_low_m <= self.local_escape_trigger_clearance_m:\n            raise ValueError("soft speed-clearance envelope must stay above the forward escape trigger")\n        if self.speed_clearance_barrier_power < 1.0:\n            raise ValueError("speed-clearance barrier power must be >= 1")\n        if (\n            isinstance(self.speed_clearance_penalty_weight, bool)\n            or not isinstance(self.speed_clearance_penalty_weight, (int, float))\n            or not math.isfinite(self.speed_clearance_penalty_weight)\n            or self.speed_clearance_penalty_weight < 0.0\n        ):\n            raise ValueError("speed_clearance_penalty_weight must be finite and non-negative")\n        if self.explore_local_goal_min_distance_m > self.explore_local_goal_max_distance_m:\n            raise ValueError("EXPLORE local-goal minimum distance exceeds maximum")\n        if (\n            not isinstance(self.explore_local_goal_distance_samples, int)\n            or isinstance(self.explore_local_goal_distance_samples, bool)\n            or self.explore_local_goal_distance_samples <= 0\n        ):\n            raise ValueError("explore_local_goal_distance_samples must be a positive integer")\n        if (\n            isinstance(self.explore_local_goal_distance_weight, bool)\n            or not isinstance(self.explore_local_goal_distance_weight, (int, float))\n            or not math.isfinite(self.explore_local_goal_distance_weight)\n            or self.explore_local_goal_distance_weight < 0.0\n        ):\n            raise ValueError("explore_local_goal_distance_weight must be finite and non-negative")\n        if (\n            not isinstance(self.footprint_safety_margin_m, (int, float))\n''',
        "l6:config-validation",
    )

    text = replace_once(
        text,
        '''        if sum(weights) <= 0.0:\n            raise ValueError("at least one trajectory score weight must be positive")\n        if (\n            not isinstance(self.progress_viability_floor, (int, float))\n''',
        '''        if sum(weights) <= 0.0:\n            raise ValueError("at least one trajectory score weight must be positive")\n        local_goal_weights = (\n            self.local_goal_novelty_weight,\n            self.local_goal_clearance_weight,\n            self.local_goal_forward_weight,\n            self.explore_local_goal_distance_weight,\n        )\n        if any(\n            not isinstance(value, (int, float))\n            or isinstance(value, bool)\n            or value < 0.0\n            or not math.isfinite(value)\n            for value in local_goal_weights\n        ):\n            raise ValueError("EXPLORE local-goal weights must be finite and non-negative")\n        if sum(local_goal_weights) <= 0.0:\n            raise ValueError("at least one EXPLORE local-goal weight must be positive")\n        if (\n            not isinstance(self.progress_viability_floor, (int, float))\n''',
        "l6:goal-weight-validation",
    )

    text = replace_method(
        text,
        "_choose_local_goal",
        "_trajectory_rollout",
        r'''
    def _choose_local_goal(
        self,
        estimate: RobotEstimate,
        costmap: RollingLocalCostmap,
        scene: _LocalPlanningScene,
    ) -> Waypoint:
        options: list[tuple[float, float, int, int, Waypoint]] = []
        footprint_radius = 0.5 * math.hypot(
            self._config.footprint_length_m,
            self._config.footprint_width_m,
        )
        relevant_clearance_m = max(
            self._config.clearance_score_cap_m,
            footprint_radius + self._config.footprint_safety_margin_m,
        ) + _POINT_CLEARANCE_EPSILON_M
        distances = _explore_goal_distances(self._config)
        distance_span = max(
            self._config.explore_local_goal_max_distance_m
            - self._config.explore_local_goal_min_distance_m,
            _POINT_CLEARANCE_EPSILON_M,
        )
        for index in range(self._config.local_goal_heading_samples):
            offset = _alternating_heading_offset(
                index,
                self._config.local_goal_heading_samples,
            )
            heading = _wrapped_angle(estimate.yaw_rad + offset)
            for distance_index, distance_m in enumerate(distances):
                goal = Waypoint(
                    estimate.x_m + distance_m * math.cos(heading),
                    estimate.y_m + distance_m * math.sin(heading),
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
                visits = self._coverage.get(
                    self._coverage_key(goal.x_m, goal.y_m), (0, 0)
                )[0]
                novelty = 1.0 / (1.0 + visits)
                clearance_score = min(
                    1.0, clearance / self._config.clearance_score_cap_m
                )
                forward_preference = 0.5 * (math.cos(offset) + 1.0)
                distance_score = (
                    1.0
                    if len(distances) == 1
                    else (distance_m - self._config.explore_local_goal_min_distance_m)
                    / distance_span
                )
                score = (
                    self._config.local_goal_novelty_weight * novelty
                    + self._config.local_goal_clearance_weight * clearance_score
                    + self._config.local_goal_forward_weight * forward_preference
                    + self._config.explore_local_goal_distance_weight * distance_score
                )
                options.append((score, distance_m, index, distance_index, goal))
        if not options:
            return Waypoint(estimate.x_m, estimate.y_m, estimate.yaw_rad)
        # Equal-quality options prefer the longer smooth segment, then the
        # canonical alternating heading order.
        return max(
            options,
            key=lambda item: (item[0], item[1], -item[2], -item[3]),
        )[4]
''',
        "l6:adaptive-explore-goal",
    )

    text = replace_once(
        text,
        '''        clearance_score = bounded_clearance / self._config.clearance_score_cap_m\n        total_score = (\n            self._config.progress_weight * progress_potential\n            + self._config.clearance_weight * clearance_score\n            + self._config.smoothness_weight * smoothness\n''',
        '''        clearance_score = _trajectory_clearance_score(\n            bounded_clearance, v_mps, self._config\n        )\n        clearance_penalty = _trajectory_clearance_penalty(\n            clearance_score, v_mps, self._config\n        )\n        total_score = (\n            self._config.progress_weight * progress_potential\n            + self._config.clearance_weight * clearance_score\n            + self._config.smoothness_weight * smoothness\n''',
        "l6:live-clearance-score",
    )
    text = replace_once(
        text,
        '''            + self._config.novelty_weight * novelty\n            - self._config.localization_observability_weight * (1.0-estimate.localization_quality.observability)\n''',
        '''            + self._config.novelty_weight * novelty\n            - self._config.speed_clearance_penalty_weight * clearance_penalty\n            - self._config.localization_observability_weight * (1.0-estimate.localization_quality.observability)\n''',
        "l6:live-clearance-penalty",
    )

    text = replace_once(
        text,
        '''        clearance_score = bounded_clearance / config.clearance_score_cap_m\n        total_score = (\n            config.progress_weight * progress_potential\n            + config.clearance_weight * clearance_score\n            + config.smoothness_weight * smoothness\n''',
        '''        clearance_score = _trajectory_clearance_score(\n            bounded_clearance, v_mps, config\n        )\n        clearance_penalty = _trajectory_clearance_penalty(\n            clearance_score, v_mps, config\n        )\n        total_score = (\n            config.progress_weight * progress_potential\n            + config.clearance_weight * clearance_score\n            + config.smoothness_weight * smoothness\n''',
        "l6:pure-clearance-score",
    )
    text = replace_once(
        text,
        '''            + config.novelty_weight * novelty\n            - config.localization_observability_weight * (1.0-estimate.localization_quality.observability)\n''',
        '''            + config.novelty_weight * novelty\n            - config.speed_clearance_penalty_weight * clearance_penalty\n            - config.localization_observability_weight * (1.0-estimate.localization_quality.observability)\n''',
        "l6:pure-clearance-penalty",
    )
    return text


def patch_l7(text: str) -> str:
    text = replace_once(
        text,
        '''    continuity_score_band: float\n    reversal_min_omega_rad_s: float\n''',
        '''    continuity_score_band: float\n    reversal_min_omega_rad_s: float\n    # Continuity may not retain a materially tighter path than another\n    # near-best candidate. Large legacy default disables the guard.\n    continuity_clearance_drop_tolerance_m: float = 1_000_000.0\n''',
        "l7:config-field",
    )
    text = replace_once(
        text,
        '''        if (\n            isinstance(reversal, bool)\n            or not isinstance(reversal, (int, float))\n            or not math.isfinite(reversal)\n            or reversal < 0.0\n        ):\n            raise ValueError("reversal_min_omega_rad_s must be finite and non-negative")\n''',
        '''        if (\n            isinstance(reversal, bool)\n            or not isinstance(reversal, (int, float))\n            or not math.isfinite(reversal)\n            or reversal < 0.0\n        ):\n            raise ValueError("reversal_min_omega_rad_s must be finite and non-negative")\n        clearance_drop = self.continuity_clearance_drop_tolerance_m\n        if (\n            isinstance(clearance_drop, bool)\n            or not isinstance(clearance_drop, (int, float))\n            or not math.isfinite(clearance_drop)\n            or clearance_drop < 0.0\n        ):\n            raise ValueError(\n                "continuity_clearance_drop_tolerance_m must be finite and non-negative"\n            )\n''',
        "l7:config-validation",
    )
    text = replace_once(
        text,
        '''                near_best = tuple(\n                    candidate\n                    for candidate in viable\n                    if candidate.total_score + _SCORE_EPSILON\n                    >= best.total_score - band\n                )\n                selected = min(\n                    near_best,\n''',
        '''                near_best = tuple(\n                    candidate\n                    for candidate in viable\n                    if candidate.total_score + _SCORE_EPSILON\n                    >= best.total_score - band\n                )\n                near_best = _clearance_guarded_near_best(\n                    near_best,\n                    self._config.continuity_clearance_drop_tolerance_m,\n                )\n                selected = min(\n                    near_best,\n''',
        "l7:clearance-guard-use",
    )
    text = replace_once(
        text,
        '''\ndef _command_continuity_key(\n''',
        r'''
def _clearance_guarded_near_best(
    candidates: tuple[TrajectoryEvaluation, ...],
    clearance_drop_tolerance_m: float,
) -> tuple[TrajectoryEvaluation, ...]:
    """Keep continuity inside both score and clearance-equivalence bands."""
    if not candidates:
        return ()
    best_clearance = max(candidate.min_clearance_m for candidate in candidates)
    floor = best_clearance - clearance_drop_tolerance_m
    guarded = tuple(
        candidate
        for candidate in candidates
        if candidate.min_clearance_m + _SCORE_EPSILON >= floor
    )
    return guarded or candidates


def _command_continuity_key(
''',
        "l7:clearance-guard-helper",
    )
    return text


def patch_replay(text: str) -> str:
    return replace_once(
        text,
        '''        if (\n            migrated.get("__type__") == "MotionSelectionConfig"\n            and "reversal_min_omega_rad_s" not in migrated\n        ):\n            migrated["reversal_min_omega_rad_s"] = 0.05\n        kind = migrated.get("__type__")\n''',
        '''        if (\n            migrated.get("__type__") == "MotionSelectionConfig"\n            and "reversal_min_omega_rad_s" not in migrated\n        ):\n            migrated["reversal_min_omega_rad_s"] = 0.05\n        if migrated.get("__type__") == "MotionSelectionConfig":\n            # Historical L7 had no clearance guard. Preserve that behavior for\n            # immutable old captures instead of applying the new live tuning.\n            migrated.setdefault("continuity_clearance_drop_tolerance_m", 1_000_000.0)\n        kind = migrated.get("__type__")\n        if kind == "NavigationConfig":\n            # These fields did not exist in historical resolved snapshots.\n            # Disable the new soft envelope and collapse adaptive EXPLORE goals\n            # to the captured legacy local_goal_distance_m.\n            legacy_goal_distance = migrated.get("local_goal_distance_m", 0.6)\n            migrated.setdefault("speed_clearance_enabled", False)\n            migrated.setdefault("speed_clearance_low_speed_mps", 0.20)\n            migrated.setdefault("speed_clearance_high_speed_mps", 0.40)\n            migrated.setdefault("speed_clearance_low_m", 0.18)\n            migrated.setdefault("speed_clearance_high_m", 0.30)\n            migrated.setdefault("speed_clearance_barrier_power", 2.0)\n            migrated.setdefault("speed_clearance_penalty_weight", 0.0)\n            migrated.setdefault("explore_local_goal_min_distance_m", legacy_goal_distance)\n            migrated.setdefault("explore_local_goal_max_distance_m", legacy_goal_distance)\n            migrated.setdefault("explore_local_goal_distance_samples", 1)\n            migrated.setdefault("explore_local_goal_distance_weight", 0.0)\n''',
        "replay:legacy-route-config",
    )


def patch_control_json(text: str) -> str:
    text = replace_once(
        text,
        '''      "local_goal_distance_m": 0.6,\n      "local_goal_tolerance_m": 0.15,\n''',
        '''      "local_goal_distance_m": 0.6,\n      "explore_local_goal_min_distance_m": 0.45,\n      "explore_local_goal_max_distance_m": 0.9,\n      "explore_local_goal_distance_samples": 4,\n      "explore_local_goal_distance_weight": 0.15,\n      "local_goal_tolerance_m": 0.15,\n''',
        "config:adaptive-goal",
    )
    text = replace_once(
        text,
        '''      "rollout_horizon_ns": 800000000,\n      "rollout_step_count": 8,\n      "footprint_safety_margin_m": 0.05,\n      "clearance_score_cap_m": 1.0,\n      "progress_weight": 0.6,\n      "clearance_weight": 0.1,\n      "smoothness_weight": 0.16,\n      "novelty_weight": 0.14,\n''',
        '''      "rollout_horizon_ns": 1000000000,\n      "rollout_step_count": 10,\n      "footprint_safety_margin_m": 0.05,\n      "clearance_score_cap_m": 1.0,\n      "speed_clearance_enabled": true,\n      "speed_clearance_low_speed_mps": 0.2,\n      "speed_clearance_high_speed_mps": 0.4,\n      "speed_clearance_low_m": 0.18,\n      "speed_clearance_high_m": 0.3,\n      "speed_clearance_barrier_power": 2.0,\n      "speed_clearance_penalty_weight": 0.18,\n      "progress_weight": 0.46,\n      "clearance_weight": 0.24,\n      "smoothness_weight": 0.22,\n      "novelty_weight": 0.08,\n''',
        "config:rollout-soft-clearance",
    )
    text = replace_once(
        text,
        '''      "local_goal_novelty_weight": 0.65,\n      "local_goal_clearance_weight": 0.25,\n      "local_goal_forward_weight": 0.1,\n''',
        '''      "local_goal_novelty_weight": 0.4,\n      "local_goal_clearance_weight": 0.35,\n      "local_goal_forward_weight": 0.1,\n''',
        "config:goal-weights",
    )
    text = replace_once(
        text,
        '''    "motion_selection": {\n      "continuity_score_band": 0.005,\n      "reversal_min_omega_rad_s": 0.05\n    }\n''',
        '''    "motion_selection": {\n      "continuity_score_band": 0.02,\n      "reversal_min_omega_rad_s": 0.05,\n      "continuity_clearance_drop_tolerance_m": 0.04\n    }\n''',
        "config:l7-continuity",
    )
    return text


TEST_CONTENT = r'''from __future__ import annotations

import json
from pathlib import Path

import pytest

from v3.config import ConfigResolver
from v3.contracts import TrajectoryEvaluation, TrajectoryPose
from v3.layers import l6_navigation as l6
from v3.layers import l7_motion_selection as l7
from v3.replay import _migrate_legacy_resolved_config_snapshot

ROOT = next(
    (
        path
        for path in Path(__file__).resolve().parents
        if (path / "conf" / "hardver.json").is_file() and (path / "v3").is_dir()
    ),
    Path.cwd(),
)


def _resolved():
    conf = ROOT / "conf"
    documents = [
        json.loads((conf / name).read_text(encoding="utf-8"))
        for name in ("hardver.json", "fizika.json", "speed_map.json", "vezerles.json")
    ]
    return ConfigResolver.from_documents(*documents)


def _candidate(candidate_id: str, *, clearance: float, score: float) -> TrajectoryEvaluation:
    horizon_ns = 100_000_000
    return TrajectoryEvaluation(
        candidate_id=candidate_id,
        v_mps=0.3,
        omega_rad_s=0.0,
        horizon_ns=horizon_ns,
        samples=(TrajectoryPose(0.03, 0.0, 0.0, horizon_ns),),
        collision=False,
        min_clearance_m=clearance,
        progress_score=0.5,
        progress_potential_score=0.5,
        progress_viable=True,
        smoothness_score=0.5,
        novelty_score=0.5,
        total_score=score,
    )


def test_production_route_tuning_is_explicit_config_authority():
    resolved = _resolved()
    control = resolved.runtime.composition.live_control.control
    nav = control.navigation
    selection = control.motion_selection

    assert nav.speed_clearance_enabled is True
    assert nav.speed_clearance_low_speed_mps == 0.2
    assert nav.speed_clearance_high_speed_mps == 0.4
    assert nav.speed_clearance_low_m == 0.18
    assert nav.speed_clearance_high_m == 0.3
    assert nav.speed_clearance_penalty_weight == 0.18
    assert nav.rollout_horizon_ns == 1_000_000_000
    assert nav.rollout_step_count == 10
    assert l6._explore_goal_distances(nav) == pytest.approx((0.9, 0.75, 0.6, 0.45))
    assert selection.continuity_score_band == 0.02
    assert selection.continuity_clearance_drop_tolerance_m == 0.04


def test_speed_clearance_soft_barrier_penalizes_fast_tight_path_first():
    nav = _resolved().runtime.composition.live_control.control.navigation
    clearance = nav.speed_clearance_low_m
    slow_score = l6._trajectory_clearance_score(clearance, 0.2, nav)
    fast_score = l6._trajectory_clearance_score(clearance, 0.4, nav)
    slow_penalty = l6._trajectory_clearance_penalty(slow_score, 0.2, nav)
    fast_penalty = l6._trajectory_clearance_penalty(fast_score, 0.4, nav)
    assert slow_score == 1.0
    assert 0.0 <= fast_score < slow_score
    assert slow_penalty == 0.0
    assert fast_penalty > 0.0
    assert l6._trajectory_clearance_score(nav.speed_clearance_high_m, 0.4, nav) == 1.0


def test_l7_continuity_guard_does_not_keep_materially_tighter_near_best_path():
    tight = _candidate("tight", clearance=0.30, score=1.0)
    wide = _candidate("wide", clearance=0.45, score=0.99)
    guarded = l7._clearance_guarded_near_best((tight, wide), 0.04)
    assert guarded == (wide,)
    assert l7._clearance_guarded_near_best((tight, wide), 1_000_000.0) == (tight, wide)


def test_historical_capture_migration_disables_new_route_policy():
    old_l7 = _migrate_legacy_resolved_config_snapshot(
        {"__type__": "MotionSelectionConfig", "continuity_score_band": 0.005}
    )
    assert old_l7["reversal_min_omega_rad_s"] == 0.05
    assert old_l7["continuity_clearance_drop_tolerance_m"] == 1_000_000.0

    old_l6 = _migrate_legacy_resolved_config_snapshot(
        {"__type__": "NavigationConfig", "local_goal_distance_m": 0.6}
    )
    assert old_l6["speed_clearance_enabled"] is False
    assert old_l6["speed_clearance_penalty_weight"] == 0.0
    assert old_l6["explore_local_goal_min_distance_m"] == 0.6
    assert old_l6["explore_local_goal_max_distance_m"] == 0.6
    assert old_l6["explore_local_goal_distance_samples"] == 1
'''


def patch_map() -> dict[str, callable]:
    return {
        "v3/layers/l6_navigation.py": patch_l6,
        "v3/layers/l7_motion_selection.py": patch_l7,
        "v3/replay.py": patch_replay,
        "conf/vezerles.json": patch_control_json,
    }


def preflight(root: Path, *, allow_head_mismatch: bool) -> dict[str, str]:
    if not (root / ".git").exists():
        raise RuntimeError(f"{root}: not a git worktree")
    head = git("rev-parse", "HEAD")
    if head != EXPECTED_HEAD and not allow_head_mismatch:
        raise RuntimeError(
            f"HEAD mismatch: expected {EXPECTED_HEAD}, found {head}. "
            "Re-run source-first against the new HEAD instead of forcing this package."
        )
    dirty = []
    for relative in TARGETS:
        if git("status", "--porcelain", "--", relative):
            dirty.append(relative)
    if (root / NEW_TEST).exists() or git("status", "--porcelain", "--", NEW_TEST):
        dirty.append(NEW_TEST)
    if dirty:
        raise RuntimeError("refusing to overwrite dirty/existing targets: " + ", ".join(dirty))

    patched: dict[str, str] = {}
    for relative, patcher in patch_map().items():
        source = (root / relative).read_text(encoding="utf-8")
        patched[relative] = patcher(source)
        if patched[relative] == source:
            raise RuntimeError(f"{relative}: patch produced no change")
    return patched


def restore_backup(root: Path, backup: Path) -> None:
    for relative in TARGETS:
        saved = backup / relative
        if saved.is_file():
            target = root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(saved, target)
    test_saved = backup / NEW_TEST
    target_test = root / NEW_TEST
    if test_saved.is_file():
        target_test.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(test_saved, target_test)
    elif target_test.exists():
        target_test.unlink()


def apply(root: Path, *, allow_head_mismatch: bool, check_only: bool) -> int:
    root = root.resolve()
    if Path.cwd().resolve() != root:
        raise RuntimeError("run this installer with --root pointing at the repository root")
    patched = preflight(root, allow_head_mismatch=allow_head_mismatch)
    print(f"preflight PASS: HEAD={git('rev-parse', 'HEAD')}")
    print("anchors PASS: " + ", ".join((*TARGETS, NEW_TEST)))
    if check_only:
        print("check-only: no files changed")
        return 0

    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup = root / ".upgrade_backups" / f"route_optimization_{stamp}"
    for relative in TARGETS:
        source = root / relative
        saved = backup / relative
        saved.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, saved)
    if (root / NEW_TEST).exists():
        saved = backup / NEW_TEST
        saved.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root / NEW_TEST, saved)

    try:
        for relative, content in patched.items():
            (root / relative).write_text(content, encoding="utf-8")
        test_path = root / NEW_TEST
        test_path.parent.mkdir(parents=True, exist_ok=True)
        test_path.write_text(TEST_CONTENT, encoding="utf-8")

        run(
            sys.executable,
            "-m",
            "py_compile",
            "v3/layers/l6_navigation.py",
            "v3/layers/l7_motion_selection.py",
            "v3/replay.py",
            NEW_TEST,
        )
        run(
            sys.executable,
            "-m",
            "pytest",
            "-q",
            NEW_TEST,
            "tests/core/test_v3_config_p0_authority.py",
            "tests/core/test_v3_planner_escape_deadlock.py",
            "tests/feature/test_v3_canonical_motion_harmony.py",
            "tests/feature/test_v3_roomcruise_localization_motion.py",
        )
    except Exception:
        print(f"validation failed; restoring {backup}", file=sys.stderr)
        restore_backup(root, backup)
        raise

    print(f"upgrade PASS; backup={backup}")
    print("changed files:")
    print(git("status", "--short", "--", *TARGETS, NEW_TEST))
    print("recommended next step: run ./r test full, then a bounded Room Cruise live A/B capture")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=".")
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument(
        "--allow-head-mismatch",
        action="store_true",
        help="allow only when anchors were independently re-reviewed against the new HEAD",
    )
    parser.add_argument(
        "--rollback",
        metavar="BACKUP_DIR",
        help="restore a backup created by this installer and exit",
    )
    args = parser.parse_args()
    root = Path(args.root).resolve()
    if args.rollback:
        restore_backup(root, Path(args.rollback).resolve())
        print(f"rollback restored from {Path(args.rollback).resolve()}")
        return 0
    return apply(
        root,
        allow_head_mismatch=args.allow_head_mismatch,
        check_only=args.check_only,
    )


if __name__ == "__main__":
    raise SystemExit(main())
