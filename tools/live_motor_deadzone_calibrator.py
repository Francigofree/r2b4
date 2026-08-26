#!/usr/bin/env python3
"""Direct-PWM dead-zone sweep and monotonic four-direction speed-map calibration."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
import uuid
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import ensure_agent_system_prompt_loaded
from tools.lidar_1m_step import STATUS_PATH, _read_json, _safe_stop_best_effort, _send_command_checked
from tools.live_motion_measurement_validator import (
    _manual_reposition_suspected,
    _wait_measurement_ready_after_reset,
)
from tools.live_motor_feedforward_calibrator import (
    ARM_PATH,
    DEFAULT_SPEEDS,
    SPEED_MAP_PATH,
    _append_jsonl,
    _map_pwm,
    _run_pulse,
    _wait_calibration_ready,
    _write_json_atomic,
)

AGENT_TESTS_DIR = test_artifacts_dir()
LATEST_RESULT = AGENT_TESTS_DIR / "latest_motor_deadzone_calibration.json"
LATEST_SAMPLES = AGENT_TESTS_DIR / "latest_motor_deadzone_calibration_samples.jsonl"
LATEST_BACKUP = AGENT_TESTS_DIR / "speed_map_before_deadzone_calibration.json"
LATEST_CANDIDATE = AGENT_TESTS_DIR / "candidate_wheel_speed_map.json"
DEFAULT_PWM_POINTS = (0.05, 0.12, 0.22, 0.35, 0.065, 0.16, 0.28, 0.08, 0.095)


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except Exception:
        return float(default)
    return out if math.isfinite(out) else float(default)


def _is_stable_side(row: Dict[str, Any], side: str) -> bool:
    stability = dict((row.get("stability") or {}).get(side) or {})
    onset = (row.get("stability") or {}).get(f"onset_{side}_s")
    actual = abs(_safe_float((row.get("actual_mps") or {}).get(side), 0.0))
    cv = stability.get("coefficient_of_variation")
    return bool(
        row.get("direct_executor_observed", False)
        and row.get("pi_disabled_observed", False)
        and not row.get("pi_violation_seen", False)
        and not (row.get("faults") or [])
        and not bool(row.get("encoder_blocking_anomaly_seen", False))
        and set(row.get("encoder_reliability_health_seen") or []) <= {"OK"}
        and _safe_float(row.get("encoder_reliability_trust_min"), 0.0) >= 0.35
        and "CALIBRATION_DIRECT_PWM"
        in set(row.get("encoder_observation_context_seen") or [])
        and onset is not None
        and float(onset) <= 1.2
        and actual >= 0.015
        and _safe_float(stability.get("moving_sample_ratio"), 0.0) >= 0.65
        and (cv is None or float(cv) <= 0.60)
        and int(stability.get("dropout_transitions", 0) or 0) <= 2
        and int(stability.get("wrong_direction_samples", 0) or 0) == 0
    )


def _first_repeat_response_gate(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "ok": True,
        "high_pwm_min": 0.33,
        "high_speed_min_mps": 0.18,
        "speed_span_min_mps": 0.12,
        "curves": {},
        "failures": [],
    }
    repeat_rows = [row for row in rows if int(row.get("repeat", 0) or 0) == 1]
    for direction in ("forward", "reverse"):
        sign = 1.0 if direction == "forward" else -1.0
        direction_rows = [row for row in repeat_rows if row.get("direction") == direction]
        for side in ("left", "right"):
            key = f"{side}_{direction}"
            ordered = sorted(
                direction_rows,
                key=lambda row: abs(
                    _safe_float((row.get("commanded_pwm") or {}).get(side))
                ),
            )
            high = ordered[-1] if ordered else {}
            low = next(
                (
                    row
                    for row in ordered
                    if sign * _safe_float((row.get("actual_mps") or {}).get(side)) >= 0.015
                ),
                ordered[0] if ordered else {},
            )
            high_pwm = abs(_safe_float((high.get("median_output_pwm") or {}).get(side)))
            high_speed = sign * _safe_float((high.get("actual_mps") or {}).get(side))
            low_speed = max(
                0.0,
                sign * _safe_float((low.get("actual_mps") or {}).get(side)),
            )
            span = high_speed - low_speed
            curve_ok = bool(
                high_pwm >= result["high_pwm_min"]
                and high_speed >= result["high_speed_min_mps"]
                and span >= result["speed_span_min_mps"]
                and not high.get("faults")
                and high.get("pi_disabled_observed", False)
                and not high.get("pi_violation_seen", False)
            )
            result["curves"][key] = {
                "ok": curve_ok,
                "high_commanded_pwm": abs(
                    _safe_float((high.get("commanded_pwm") or {}).get(side))
                ),
                "high_output_pwm": high_pwm,
                "high_speed_mps": high_speed,
                "low_speed_mps": low_speed,
                "speed_span_mps": span,
            }
            if not curve_ok:
                result["failures"].append(key)
    result["ok"] = not result["failures"]
    return result


def _pav(values: List[float], weights: List[int]) -> List[float]:
    blocks: List[List[float]] = []
    for index, (value, weight) in enumerate(zip(values, weights)):
        blocks.append([float(index), float(index), float(value) * int(weight), float(weight)])
        while len(blocks) >= 2:
            left = blocks[-2]
            right = blocks[-1]
            if left[2] / left[3] <= right[2] / right[3] + 1e-12:
                break
            blocks[-2:] = [[left[0], right[1], left[2] + right[2], left[3] + right[3]]]
    out = [0.0] * len(values)
    for start, end, total, weight in blocks:
        for index in range(int(start), int(end) + 1):
            out[index] = float(total / weight)
    return out


def _build_curve(rows: Iterable[Dict[str, Any]], direction: str, side: str) -> Dict[str, Any]:
    grouped: Dict[float, List[Dict[str, Any]]] = {}
    for row in rows:
        if str(row.get("direction")) != direction:
            continue
        pwm = round(abs(_safe_float((row.get("commanded_pwm") or {}).get(side))), 5)
        grouped.setdefault(pwm, []).append(row)
    points: List[Dict[str, Any]] = []
    sign = 1.0 if direction == "forward" else -1.0
    for pwm in sorted(grouped):
        group = grouped[pwm]
        stable_rows = [row for row in group if _is_stable_side(row, side)]
        speeds = [
            max(0.0, sign * _safe_float((row.get("actual_mps") or {}).get(side)))
            for row in stable_rows
        ]
        stable = len(stable_rows) >= 2
        points.append(
            {
                "pwm": float(pwm),
                "repeats": len(group),
                "stable_repeats": len(stable_rows),
                "stable": bool(stable),
                "median_speed_mps": statistics.median(speeds) if speeds else 0.0,
                "speed_cv": (
                    statistics.pstdev(speeds) / statistics.mean(speeds)
                    if len(speeds) >= 2 and statistics.mean(speeds) > 0.003
                    else None
                ),
            }
        )
    stable_points = [point for point in points if point["stable"]]
    if len(stable_points) < 3:
        raise RuntimeError(f"insufficient_stable_pwm_points:{direction}:{side}:{len(stable_points)}")
    isotonic = _pav(
        [float(point["median_speed_mps"]) for point in stable_points],
        [int(point["stable_repeats"]) for point in stable_points],
    )
    for point, speed in zip(stable_points, isotonic):
        point["isotonic_speed_mps"] = float(speed)
    return {
        "direction": direction,
        "side": side,
        "points": points,
        "stable_points": stable_points,
        "min_stable_pwm": float(stable_points[0]["pwm"]),
        "min_stable_speed_mps": float(stable_points[0]["isotonic_speed_mps"]),
        "max_stable_pwm": float(stable_points[-1]["pwm"]),
        "max_stable_speed_mps": float(stable_points[-1]["isotonic_speed_mps"]),
    }


def _curve_speed_at_pwm(curve: Dict[str, Any], pwm: float) -> float:
    points = list(curve.get("stable_points") or [])
    target = abs(float(pwm))
    if target <= float(points[0]["pwm"]):
        return float(points[0]["isotonic_speed_mps"])
    if target >= float(points[-1]["pwm"]):
        return float(points[-1]["isotonic_speed_mps"])
    for left, right in zip(points, points[1:]):
        p0, p1 = float(left["pwm"]), float(right["pwm"])
        if p0 <= target <= p1:
            ratio = (target - p0) / max(1e-9, p1 - p0)
            return float(left["isotonic_speed_mps"]) + ratio * (
                float(right["isotonic_speed_mps"]) - float(left["isotonic_speed_mps"])
            )
    return float(points[-1]["isotonic_speed_mps"])


def _curve_pwm_for_speed(curve: Dict[str, Any], target_speed: float) -> Tuple[float, bool]:
    points = list(curve.get("stable_points") or [])
    target = abs(float(target_speed))
    min_speed = float(points[0]["isotonic_speed_mps"])
    max_speed = float(points[-1]["isotonic_speed_mps"])
    if target <= min_speed:
        return float(points[0]["pwm"]), bool(target >= min_speed * 0.80)
    if target >= max_speed:
        return float(points[-1]["pwm"]), bool(target <= max_speed * 1.20)
    for left, right in zip(points, points[1:]):
        v0, v1 = float(left["isotonic_speed_mps"]), float(right["isotonic_speed_mps"])
        if v0 <= target <= v1:
            if v1 - v0 <= 0.002:
                return float(right["pwm"]), True
            ratio = (target - v0) / (v1 - v0)
            return float(left["pwm"]) + ratio * (float(right["pwm"]) - float(left["pwm"])), True
    return float(points[-1]["pwm"]), False


def _build_candidate(old_map: Dict[str, Any], curves: Dict[str, Dict[str, Any]]) -> Tuple[Dict[str, Any], List[str]]:
    candidate = {
        "schema": "R2B4_WHEEL_SPEED_MAP_V2",
        "map_state": "CANDIDATE",
        "hardware": old_map.get("hardware", "DFRobot KIT0085"),
        "calibration_state": "LIVE_DIRECT_PWM_DEADZONE_MONOTONIC_CANDIDATE_2026_07_16",
        "calibration_evidence": "latest_motor_deadzone_calibration.json anomaly-free stable samples only",
        "interpolation": "linear",
        "curves": {},
    }
    unreachable: List[str] = []
    for direction in ("forward", "reverse"):
        for side in ("left", "right"):
            curve = curves[f"{side}_{direction}"]
            previous = 0.0
            rows: List[Dict[str, float]] = []
            for speed in DEFAULT_SPEEDS:
                pwm, reachable = _curve_pwm_for_speed(curves[f"{side}_{direction}"], speed)
                pwm = max(float(pwm), previous + (0.002 if previous else 0.0))
                pwm = round(min(0.35, pwm), 4)
                rows.append({"speed_mps": float(speed), "pwm": float(pwm)})
                previous = pwm
                if not reachable:
                    unreachable.append(f"{side}_{direction}:{speed:.2f}")
            last_unstable = max(
                (
                    float(point["pwm"])
                    for point in list(curve.get("points") or [])
                    if not bool(point.get("stable", False))
                    and float(point["pwm"]) < float(curve["min_stable_pwm"])
                ),
                default=None,
            )
            candidate["curves"][f"{side}_{direction}"] = {
                "wheel": side,
                "direction": direction,
                "startup_pwm": float(curve["min_stable_pwm"]),
                "maintenance_pwm": float(curve["min_stable_pwm"]),
                "dead_zone_pwm": float(curve["min_stable_pwm"]),
                "last_unstable_pwm": last_unstable,
                "minimum_stable_speed_mps": float(curve["min_stable_speed_mps"]),
                "stable_source_point_count": len(curve.get("stable_points") or []),
                "points": rows,
            }
    return candidate, unreachable


def _build_refit_candidate(
    candidate: Dict[str, Any],
    validation_rows: List[Dict[str, Any]],
) -> Dict[str, Any]:
    refit = {
        "schema": "R2B4_WHEEL_SPEED_MAP_V2",
        "map_state": "CANDIDATE",
        "hardware": candidate.get("hardware", "DFRobot KIT0085"),
        "calibration_state": "LIVE_DIRECT_PWM_STARTUP_MAINTENANCE_REFIT_CANDIDATE_2026_07_16",
        "calibration_evidence": (
            "latest_motor_deadzone_calibration.json differential-PWM validation residuals"
        ),
        "interpolation": "linear",
        "curves": {},
    }
    for direction in ("forward", "reverse"):
        sign = 1.0 if direction == "forward" else -1.0
        for side in ("left", "right"):
            key = f"{side}_{direction}"
            source_curve = dict((candidate.get("curves") or {}).get(key) or {})
            source_points = {
                round(float(point["speed_mps"]), 4): float(point["pwm"])
                for point in list(source_curve.get("points") or [])
            }
            rows: List[Dict[str, float]] = []
            previous = 0.0
            stable_pwm_groups: Dict[float, List[Dict[str, Any]]] = {}
            for row in validation_rows:
                if row.get("direction") != direction or not _is_stable_side(row, side):
                    continue
                pwm = round(
                    abs(_safe_float((row.get("commanded_pwm") or {}).get(side))),
                    4,
                )
                stable_pwm_groups.setdefault(pwm, []).append(row)
            for speed in DEFAULT_SPEEDS:
                speed_key = round(float(speed), 4)
                original_pwm = float(source_points[speed_key])
                stable_rows = [
                    row
                    for row in validation_rows
                    if row.get("direction") == direction
                    and abs(_safe_float(row.get("target_speed_mps")) - speed) <= 1e-6
                    and _is_stable_side(row, side)
                ]
                corrected_pwm = original_pwm
                if len(stable_rows) >= 2:
                    actual = statistics.median(
                        max(
                            0.0,
                            sign
                            * _safe_float((row.get("actual_mps") or {}).get(side)),
                        )
                        for row in stable_rows
                    )
                    ratio = max(0.75, min(1.35, float(speed) / max(0.005, actual)))
                    corrected_pwm = original_pwm * (ratio ** 0.70)
                corrected_pwm = max(
                    corrected_pwm,
                    previous + (0.002 if previous else 0.0),
                )
                corrected_pwm = round(min(0.35, corrected_pwm), 4)
                rows.append({"speed_mps": float(speed), "pwm": corrected_pwm})
                previous = corrected_pwm

            proven_startup_pwm = min(
                (
                    pwm
                    for pwm, pwm_rows in stable_pwm_groups.items()
                    if len(pwm_rows) >= 2
                ),
                default=rows[0]["pwm"],
            )
            proven_startup_pwm = max(float(proven_startup_pwm), float(rows[0]["pwm"]))
            refit["curves"][key] = {
                "wheel": side,
                "direction": direction,
                "startup_pwm": round(min(0.35, proven_startup_pwm), 4),
                "maintenance_pwm": float(rows[0]["pwm"]),
                "dead_zone_pwm": float(rows[0]["pwm"]),
                "maintenance_source": "differential_pwm_residual_refit",
                "startup_source": "lowest_two_repeat_stable_differential_pwm",
                "points": rows,
            }
    return refit


def _build_bracket_candidate(
    active_map: Dict[str, Any],
    source_rows: List[Dict[str, Any]],
) -> Dict[str, Any]:
    initial_rows = [
        row for row in source_rows if row.get("stage") == "candidate_validation"
    ]
    aggressive_rows = [
        row for row in source_rows if row.get("stage") == "refit_validation"
    ]
    sweep_rows = [row for row in source_rows if row.get("stage") == "pwm_sweep"]
    candidate = {
        "schema": "R2B4_WHEEL_SPEED_MAP_V2",
        "map_state": "CANDIDATE",
        "hardware": active_map.get("hardware", "DFRobot KIT0085"),
        "calibration_state": "LIVE_DIRECT_PWM_TWO_SIDED_BRACKET_CANDIDATE_2026_07_16",
        "calibration_evidence": (
            "initial and aggressive differential-PWM validation measurements"
        ),
        "interpolation": "linear",
        "curves": {},
    }

    def group_stats(
        rows: List[Dict[str, Any]],
        direction: str,
        side: str,
        speed: float,
    ) -> Tuple[float, float, bool]:
        sign = 1.0 if direction == "forward" else -1.0
        group = [
            row
            for row in rows
            if row.get("direction") == direction
            and abs(_safe_float(row.get("target_speed_mps")) - speed) <= 1e-6
        ]
        pwm = statistics.median(
            abs(_safe_float((row.get("commanded_pwm") or {}).get(side)))
            for row in group
        )
        actual = statistics.median(
            max(
                0.0,
                sign * _safe_float((row.get("actual_mps") or {}).get(side)),
            )
            for row in group
        )
        stable = sum(_is_stable_side(row, side) for row in group) >= 2
        return float(pwm), float(actual), bool(stable)

    for direction in ("forward", "reverse"):
        for side in ("left", "right"):
            points: List[Dict[str, float]] = []
            previous = 0.0
            for speed in DEFAULT_SPEEDS:
                p0, a0, stable0 = group_stats(
                    initial_rows, direction, side, float(speed)
                )
                p1, a1, stable1 = group_stats(
                    aggressive_rows, direction, side, float(speed)
                )
                target = float(speed)
                bracketed = bool(
                    stable0
                    and stable1
                    and abs(a1 - a0) >= 0.008
                    and min(a0, a1) <= target <= max(a0, a1)
                )
                if bracketed:
                    ratio = (target - a0) / (a1 - a0)
                    pwm = p0 + ratio * (p1 - p0)
                elif stable0 or stable1:
                    choices = []
                    if stable0:
                        choices.append((abs(a0 - target), p0))
                    if stable1:
                        choices.append((abs(a1 - target), p1))
                    pwm = min(choices, key=lambda item: item[0])[1]
                else:
                    failed_pwm = max(p0, p1)
                    stable_sweep_pwms = []
                    grouped_sweep: Dict[float, List[Dict[str, Any]]] = {}
                    for row in sweep_rows:
                        if row.get("direction") != direction:
                            continue
                        sweep_pwm = round(
                            abs(
                                _safe_float(
                                    (row.get("commanded_pwm") or {}).get(side)
                                )
                            ),
                            4,
                        )
                        grouped_sweep.setdefault(sweep_pwm, []).append(row)
                    for sweep_pwm, pwm_rows in grouped_sweep.items():
                        if (
                            sweep_pwm >= failed_pwm
                            and len(pwm_rows) >= 3
                            and all(_is_stable_side(row, side) for row in pwm_rows)
                        ):
                            stable_sweep_pwms.append(sweep_pwm)
                    pwm = min(stable_sweep_pwms, default=failed_pwm)
                pwm = max(float(pwm), previous + (0.002 if previous else 0.0))
                pwm = round(min(0.35, pwm), 4)
                points.append({"speed_mps": target, "pwm": pwm})
                previous = pwm

            low_aggressive = [
                row
                for row in aggressive_rows
                if row.get("direction") == direction
                and abs(_safe_float(row.get("target_speed_mps")) - 0.05) <= 1e-6
            ]
            startup_pwm = statistics.median(
                abs(
                    _safe_float(
                        (row.get("startup_commanded_pwm") or {}).get(
                            side,
                            (row.get("commanded_pwm") or {}).get(side),
                        )
                    )
                )
                for row in low_aggressive
            )
            startup_pwm = round(
                min(0.35, max(float(points[0]["pwm"]), float(startup_pwm))),
                4,
            )
            candidate["curves"][f"{side}_{direction}"] = {
                "wheel": side,
                "direction": direction,
                "startup_pwm": startup_pwm,
                "maintenance_pwm": float(points[0]["pwm"]),
                "dead_zone_pwm": float(points[0]["pwm"]),
                "maintenance_source": "two_sided_live_bracket_or_stable_sweep_fallback",
                "startup_source": "latest_low_speed_startup_phase",
                "points": points,
            }
    return candidate


def _model_score(speed_map: Dict[str, Any], curves: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    errors: List[float] = []
    groups: Dict[str, List[float]] = {}
    for direction in ("forward", "reverse"):
        for side in ("left", "right"):
            key = f"{side}_{direction}"
            for speed in DEFAULT_SPEEDS:
                pwm = _map_pwm(speed_map, direction, speed, side)
                actual = _curve_speed_at_pwm(curves[key], pwm)
                error = abs(actual - speed) / max(0.01, speed)
                errors.append(error)
                groups.setdefault(key, []).append(error)
    return {
        "median_relative_error": statistics.median(errors),
        "mean_relative_error": statistics.mean(errors),
        "max_group_median_relative_error": max(statistics.median(values) for values in groups.values()),
        "group_median_relative_error": {key: statistics.median(values) for key, values in groups.items()},
    }


def _validation_score(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    errors: List[float] = []
    groups: Dict[str, List[float]] = {}
    unstable = 0
    wrong_direction = 0
    for row in rows:
        direction = str(row["direction"])
        sign = 1.0 if direction == "forward" else -1.0
        target = abs(float(row["target_speed_mps"]))
        for side in ("left", "right"):
            actual = sign * _safe_float((row.get("actual_mps") or {}).get(side))
            error = abs(actual - target) / max(0.01, target)
            errors.append(error)
            groups.setdefault(f"{side}_{direction}", []).append(error)
            if not _is_stable_side(row, side):
                unstable += 1
            if actual < -0.003:
                wrong_direction += 1
    return {
        "median_relative_error": statistics.median(errors) if errors else math.inf,
        "mean_relative_error": statistics.mean(errors) if errors else math.inf,
        "max_group_median_relative_error": max(
            (statistics.median(values) for values in groups.values()), default=math.inf
        ),
        "group_median_relative_error": {
            key: statistics.median(values) for key, values in groups.items()
        },
        "unstable_side_count": int(unstable),
        "wrong_direction_count": int(wrong_direction),
        "fault_count": sum(len(row.get("faults") or []) for row in rows),
    }


def _pause_and_reanchor(args: argparse.Namespace, label: str) -> Dict[str, Any]:
    print(f"DEADZONE pause={args.pause_s:.1f}s after={label}", flush=True)
    deadline = time.monotonic() + max(0.0, float(args.pause_s))
    while time.monotonic() < deadline:
        time.sleep(min(0.5, max(0.01, deadline - time.monotonic())))
    status = _read_json(STATUS_PATH) or {}
    reposition = _manual_reposition_suspected(status)
    event = {
        "label": label,
        "manual_reposition": reposition,
        "reset_required": True,
        "reset_reason": "mandatory_after_manual_reposition_window",
        "ok": True,
    }
    try:
        event["reset_command"] = _send_command_checked("reset_pos", token=args.token, timeout_s=8.0)
        ready = _wait_measurement_ready_after_reset(args.ready_timeout_s, stable_samples=5)
        event["measurement_ready"] = ready
        if not ready.get("ok", False):
            raise RuntimeError("measurement_not_ready_after_reset")
    except Exception as exc:
        event["ok"] = False
        event["error"] = str(exc)
        return event
    try:
        _wait_calibration_ready(args.ready_timeout_s)
    except Exception as exc:
        event["ok"] = False
        event["error"] = str(exc)
    return event


def _run_matrix(
    *,
    stage: str,
    repeats: int,
    points: Iterable[float],
    args: argparse.Namespace,
    nonce: str,
    candidate: Dict[str, Any] | None = None,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    rows: List[Dict[str, Any]] = []
    pauses: List[Dict[str, Any]] = []
    point_list = list(points)
    total = int(repeats) * len(point_list) * 2
    index = 0
    for repeat in range(1, int(repeats) + 1):
        for point in point_list:
            for direction in ("forward", "reverse"):
                index += 1
                if candidate is None:
                    left_pwm = right_pwm = float(point)
                    target_speed = max(0.03, min(0.15, float(point) * 0.50))
                else:
                    target_speed = float(point)
                    left_pwm = _map_pwm(candidate, direction, target_speed, "left")
                    right_pwm = _map_pwm(candidate, direction, target_speed, "right")
                startup_left_pwm = abs(float(left_pwm))
                startup_right_pwm = abs(float(right_pwm))
                startup_duration_s = 0.0
                if candidate is not None:
                    direction_key = "forward" if direction == "forward" else "reverse"
                    candidate_curves = dict(candidate.get("curves") or {})
                    startup_left_pwm = max(
                        startup_left_pwm,
                        abs(
                            _safe_float(
                                (candidate_curves.get(f"left_{direction_key}") or {}).get(
                                    "startup_pwm"
                                ),
                                startup_left_pwm,
                            )
                        ),
                    )
                    startup_right_pwm = max(
                        startup_right_pwm,
                        abs(
                            _safe_float(
                                (candidate_curves.get(f"right_{direction_key}") or {}).get(
                                    "startup_pwm"
                                ),
                                startup_right_pwm,
                            )
                        ),
                    )
                    startup_duration_s = max(0.0, float(args.startup_duration_s))
                print(
                    f"DEADZONE phase={index}/{total} stage={stage} repeat={repeat} "
                    f"direction={direction} point={point:.3f} pwm={left_pwm:.4f}/{right_pwm:.4f}",
                    flush=True,
                )
                row = _run_pulse(
                    stage=stage,
                    repeat=repeat,
                    target_speed=target_speed,
                    direction=direction,
                    left_pwm=left_pwm,
                    right_pwm=right_pwm,
                    nonce=nonce,
                    token=args.token,
                    stabilization_s=args.stabilization_s,
                    measurement_s=args.measurement_s,
                    poll_s=args.poll_s,
                    max_phase_distance_m=args.max_phase_distance_m,
                    sample_path=LATEST_SAMPLES,
                    sample_metadata={
                        "sweep_pwm": None if candidate is not None else float(point),
                    },
                    startup_left_pwm=startup_left_pwm,
                    startup_right_pwm=startup_right_pwm,
                    startup_duration_s=startup_duration_s,
                )
                rows.append(row)
                if index < total:
                    pause = _pause_and_reanchor(args, f"{stage}_{repeat}_{direction}_{point:.3f}")
                    pauses.append(pause)
                    if not pause.get("ok", False):
                        raise RuntimeError(f"pause_not_ready:{pause.get('error', '')}")
            if (
                stage == "pwm_sweep"
                and repeat == 1
                and abs(float(point) - max(point_list)) <= 1e-9
            ):
                response_gate = _first_repeat_response_gate(rows)
                print(
                    "DEADZONE response_gate="
                    f"{'PASS' if response_gate['ok'] else 'FAIL'} "
                    f"failures={','.join(response_gate['failures'])}",
                    flush=True,
                )
                if not response_gate["ok"]:
                    raise RuntimeError(
                        "insufficient_pwm_speed_response:"
                        + ",".join(response_gate["failures"])
                    )
    return rows, pauses


def _run_refit(args: argparse.Namespace) -> Dict[str, Any]:
    source_result_path = latest_artifact_path("latest_motor_deadzone_calibration.json")
    source_samples_path = latest_artifact_path("latest_motor_deadzone_calibration_samples.jsonl")
    if not source_result_path.exists() or not source_samples_path.exists():
        raise RuntimeError("refit_source_artifact_missing")
    previous_result = json.loads(source_result_path.read_text(encoding="utf-8"))
    source_rows = [
        json.loads(raw)
        for raw in source_samples_path.read_text(encoding="utf-8").splitlines()
        if raw.strip()
    ]
    sweep_rows = [row for row in source_rows if row.get("stage") == "pwm_sweep"]
    prior_validation = [
        row for row in source_rows if row.get("stage") == "candidate_validation"
    ]
    aggressive_validation = [
        row for row in source_rows if row.get("stage") == "refit_validation"
    ]
    if len(sweep_rows) != 54 or len(prior_validation) != 24:
        raise RuntimeError(
            f"refit_source_phase_count_invalid:{len(sweep_rows)}:{len(prior_validation)}"
        )
    bracket_mode = bool(args.reuse_latest_bracket)
    if bracket_mode and len(aggressive_validation) != 24:
        raise RuntimeError(
            f"bracket_source_phase_count_invalid:{len(aggressive_validation)}"
        )
    if bool(previous_result.get("candidate_qualified", False)):
        raise RuntimeError("refit_source_candidate_already_qualified")

    old_map = json.loads(SPEED_MAP_PATH.read_text(encoding="utf-8"))
    _write_json_atomic(LATEST_BACKUP, old_map)
    if bracket_mode:
        candidate = _build_bracket_candidate(old_map, source_rows)
    else:
        source_candidate = dict(previous_result.get("candidate_map") or {})
        candidate = _build_refit_candidate(source_candidate, prior_validation)
    _write_json_atomic(LATEST_CANDIDATE, candidate)

    curves: Dict[str, Dict[str, Any]] = {}
    for direction in ("forward", "reverse"):
        for side in ("left", "right"):
            curves[f"{side}_{direction}"] = _build_curve(sweep_rows, direction, side)
    before_score = _model_score(old_map, curves)

    nonce = uuid.uuid4().hex
    _write_json_atomic(
        ARM_PATH,
        {
            "purpose": "motor_feedforward_calibration",
            "nonce": nonce,
            "issued_at": time.time(),
            "expires_at": time.time() + max(900.0, float(args.arm_duration_s)),
            "max_abs_pwm": 0.35,
        },
    )
    validation_rows: List[Dict[str, Any]] = []
    pauses: List[Dict[str, Any]] = []
    error = ""
    qualified = False
    try:
        _safe_stop_best_effort(args.token)
        _wait_calibration_ready(args.ready_timeout_s)
        transition_pause = _pause_and_reanchor(args, "refit_candidate_written")
        pauses.append(transition_pause)
        if not transition_pause.get("ok", False):
            raise RuntimeError("refit_transition_not_ready")
        validation_rows, validation_pauses = _run_matrix(
            stage="bracket_validation" if bracket_mode else "refit_validation",
            repeats=args.validation_repeats,
            points=DEFAULT_SPEEDS,
            args=args,
            nonce=nonce,
            candidate=candidate,
        )
        pauses.extend(validation_pauses)
        after_score = _validation_score(validation_rows)
        qualified = bool(
            after_score["median_relative_error"] + 0.03
            < before_score["median_relative_error"]
            and after_score["mean_relative_error"] < before_score["mean_relative_error"]
            and after_score["max_group_median_relative_error"]
            <= before_score["max_group_median_relative_error"] + 0.10
            and after_score["unstable_side_count"] == 0
            and after_score["wrong_direction_count"] == 0
            and after_score["fault_count"] == 0
        )
    except Exception as exc:
        error = str(exc)
    finally:
        _safe_stop_best_effort(args.token)
        try:
            ARM_PATH.unlink()
        except FileNotFoundError:
            pass

    after_score = _validation_score(validation_rows)
    completed = bool(
        not error
        and len(validation_rows)
        == int(args.validation_repeats) * len(DEFAULT_SPEEDS) * 2
    )
    result = {
        "test_name": "motor_deadzone_calibration_live",
        "success": bool(completed),
        "status": "PASS" if completed else "FAIL",
        "calibration_outcome": (
            "CANDIDATE_QUALIFIED"
            if qualified
            else ("CANDIDATE_REJECTED" if completed else "INCOMPLETE")
        ),
        "candidate_qualified": bool(qualified),
        "candidate_kept": False,
        "candidate_activation_allowed": False,
        "error": error,
        "refit_from_latest_validation": not bracket_mode,
        "bracket_from_two_live_candidates": bracket_mode,
        "startup_duration_s": float(args.startup_duration_s),
        "reused_sweep_phase_count": len(sweep_rows),
        "reused_prior_validation_phase_count": len(prior_validation),
        "reused_aggressive_validation_phase_count": len(aggressive_validation),
        "validation_phase_count": len(validation_rows),
        "before_score": before_score,
        "prior_after_score": previous_result.get("after_score") or {},
        "after_score": after_score,
        "candidate_map": candidate,
        "active_speed_map": old_map,
        "pause_failures": [event for event in pauses if not event.get("ok", False)],
        "artifacts": {
            "result": str(LATEST_RESULT.relative_to(PROJECT_ROOT)),
            "samples": str(LATEST_SAMPLES.relative_to(PROJECT_ROOT)),
            "backup": str(LATEST_BACKUP.relative_to(PROJECT_ROOT)),
            "candidate": str(LATEST_CANDIDATE.relative_to(PROJECT_ROOT)),
        },
    }
    _write_json_atomic(LATEST_RESULT, result)
    return result


def run(args: argparse.Namespace) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    if bool(args.reuse_latest_refit or args.reuse_latest_bracket):
        return _run_refit(args)
    points: List[float] = []
    for raw_value in args.pwm_points:
        value = round(abs(float(raw_value)), 5)
        if value not in points:
            points.append(value)
    if not points or points[-1] > 0.35 or points[0] < 0.04:
        raise ValueError("pwm_points_out_of_bounds")
    old_map = json.loads(SPEED_MAP_PATH.read_text(encoding="utf-8"))
    _write_json_atomic(LATEST_BACKUP, old_map)
    LATEST_SAMPLES.parent.mkdir(parents=True, exist_ok=True)
    LATEST_SAMPLES.write_text("", encoding="utf-8")
    nonce = uuid.uuid4().hex
    _write_json_atomic(
        ARM_PATH,
        {
            "purpose": "motor_feedforward_calibration",
            "nonce": nonce,
            "issued_at": time.time(),
            "expires_at": time.time() + max(1800.0, float(args.arm_duration_s)),
            "max_abs_pwm": 0.35,
        },
    )
    sweep_rows: List[Dict[str, Any]] = []
    validation_rows: List[Dict[str, Any]] = []
    pauses: List[Dict[str, Any]] = []
    curves: Dict[str, Dict[str, Any]] = {}
    candidate: Dict[str, Any] = {}
    unreachable: List[str] = []
    response_gate: Dict[str, Any] = {}
    qualified = False
    error = ""
    try:
        _safe_stop_best_effort(args.token)
        _wait_calibration_ready(args.ready_timeout_s)
        sweep_rows, sweep_pauses = _run_matrix(
            stage="pwm_sweep",
            repeats=args.sweep_repeats,
            points=points,
            args=args,
            nonce=nonce,
        )
        pauses.extend(sweep_pauses)
        response_gate = _first_repeat_response_gate(sweep_rows)
        for direction in ("forward", "reverse"):
            for side in ("left", "right"):
                curves[f"{side}_{direction}"] = _build_curve(sweep_rows, direction, side)
        candidate, unreachable = _build_candidate(old_map, curves)
        _write_json_atomic(LATEST_CANDIDATE, candidate)
        before_score = _model_score(old_map, curves)
        transition_pause = _pause_and_reanchor(args, "candidate_written")
        pauses.append(transition_pause)
        if not transition_pause.get("ok", False):
            raise RuntimeError("candidate_transition_not_ready")
        validation_rows, validation_pauses = _run_matrix(
            stage="candidate_validation",
            repeats=args.validation_repeats,
            points=DEFAULT_SPEEDS,
            args=args,
            nonce=nonce,
            candidate=candidate,
        )
        pauses.extend(validation_pauses)
        after_score = _validation_score(validation_rows)
        qualified = bool(
            after_score["median_relative_error"] + 0.03 < before_score["median_relative_error"]
            and after_score["mean_relative_error"] < before_score["mean_relative_error"]
            and after_score["max_group_median_relative_error"]
            <= before_score["max_group_median_relative_error"] + 0.10
            and after_score["unstable_side_count"] == 0
            and after_score["wrong_direction_count"] == 0
            and after_score["fault_count"] == 0
            and not unreachable
            and response_gate.get("ok", False)
        )
    except Exception as exc:
        error = str(exc)
    finally:
        _safe_stop_best_effort(args.token)
        try:
            ARM_PATH.unlink()
        except FileNotFoundError:
            pass

    before_score = _model_score(old_map, curves) if curves else {}
    after_score = _validation_score(validation_rows)
    completed = bool(
        not error
        and len(sweep_rows) == int(args.sweep_repeats) * len(points) * 2
        and len(validation_rows) == int(args.validation_repeats) * len(DEFAULT_SPEEDS) * 2
    )
    result = {
        "test_name": "motor_deadzone_calibration_live",
        "success": bool(completed),
        "status": "PASS" if completed else "FAIL",
        "calibration_outcome": "CANDIDATE_QUALIFIED" if qualified else ("CANDIDATE_REJECTED" if completed else "INCOMPLETE"),
        "candidate_qualified": bool(qualified),
        "candidate_kept": False,
        "candidate_activation_allowed": False,
        "error": error,
        "pwm_points": points,
        "sweep_repeats": int(args.sweep_repeats),
        "validation_repeats": int(args.validation_repeats),
        "max_abs_pwm": 0.35,
        "max_phase_distance_m": float(args.max_phase_distance_m),
        "pause_s": float(args.pause_s),
        "curves": curves,
        "unreachable_targets": unreachable,
        "first_repeat_response_gate": response_gate,
        "before_score": before_score,
        "after_score": after_score,
        "candidate_map": candidate,
        "active_speed_map": old_map,
        "sweep_phase_count": len(sweep_rows),
        "validation_phase_count": len(validation_rows),
        "pause_failures": [event for event in pauses if not event.get("ok", False)],
        "artifacts": {
            "result": str(LATEST_RESULT.relative_to(PROJECT_ROOT)),
            "samples": str(LATEST_SAMPLES.relative_to(PROJECT_ROOT)),
            "backup": str(LATEST_BACKUP.relative_to(PROJECT_ROOT)),
            "candidate": str(LATEST_CANDIDATE.relative_to(PROJECT_ROOT)),
        },
    }
    _write_json_atomic(LATEST_RESULT, result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pwm-points", nargs="+", type=float, default=list(DEFAULT_PWM_POINTS))
    parser.add_argument("--sweep-repeats", type=int, default=3)
    parser.add_argument("--validation-repeats", type=int, default=2)
    parser.add_argument("--stabilization-s", type=float, default=0.6)
    parser.add_argument("--measurement-s", type=float, default=1.8)
    parser.add_argument("--pause-s", type=float, default=10.0)
    parser.add_argument("--poll-s", type=float, default=0.08)
    parser.add_argument("--max-phase-distance-m", type=float, default=1.5)
    parser.add_argument("--ready-timeout-s", type=float, default=90.0)
    parser.add_argument("--arm-duration-s", type=float, default=2400.0)
    parser.add_argument("--startup-duration-s", type=float, default=0.35)
    parser.add_argument("--reuse-latest-refit", action="store_true")
    parser.add_argument("--reuse-latest-bracket", action="store_true")
    parser.add_argument("--token", default="GUI_DEFAULT")
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()
    result = run(args)
    print(
        "MOTOR_DEADZONE_CAL|"
        f"status={result.get('status')}|outcome={result.get('calibration_outcome')}|"
        f"qualified={result.get('candidate_qualified')}|error={result.get('error', '')}",
        flush=True,
    )
    return 0 if result.get("success", False) else 1


if __name__ == "__main__":
    raise SystemExit(main())
