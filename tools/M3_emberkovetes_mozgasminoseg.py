#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Live Human Follow v2 movement-quality measurement and validation."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import ensure_agent_system_prompt_loaded  # noqa: E402
from tools import person_follow_camera_live as live  # noqa: E402


AGENT_TESTS_DIR = test_artifacts_dir()
RESULT_PATH = AGENT_TESTS_DIR / "latest_M3_emberkovetes_mozgasminoseg.json"
SUMMARY_PATH = AGENT_TESTS_DIR / "latest_M3_emberkovetes_mozgasminoseg_summary.json"
SAMPLES_PATH = AGENT_TESTS_DIR / "M3_emberkovetes_mozgasminoseg_samples.jsonl"
INCIDENT_PATH = AGENT_TESTS_DIR / "latest_M3_emberkovetes_mozgasminoseg_incident.json"
BASE_RESULT_PATH = AGENT_TESTS_DIR / "latest_M3_emberkovetes_mozgasminoseg_follow_base.json"
BASE_SUMMARY_PATH = AGENT_TESTS_DIR / "latest_M3_emberkovetes_mozgasminoseg_follow_base_summary.json"


DEFAULT_THRESHOLDS: Dict[str, float] = {
    "min_total_samples": 120,
    "min_follow_samples": 40,
    "min_moving_samples": 25,
    "min_straight_samples": 20,
    "min_arc_samples": 15,
    "target_visible_ratio_min": 0.75,
    "target_lock_ratio_min": 0.65,
    "target_bearing_abs_p90_max_deg": 25.0,
    "target_distance_error_abs_p90_max_m": 0.35,
    "target_distance_std_max_m": 0.18,
    "linear_tracking_relative_p90_max": 0.20,
    "omega_tracking_relative_p90_max": 0.20,
    "wheel_tracking_abs_p90_max_mps": 0.025,
    "wheel_tracking_relative_p90_max": 0.35,
    "pwm_step_p95_max": 0.08,
    "pwm_dropout_ratio_max": 0.03,
    "pwm_sign_reversal_max": 0,
    "wheel_stop_start_ratio_max": 0.05,
    "straight_oscillation_amplitude_p90_max_rad_s": 0.10,
    "straight_oscillation_frequency_max_hz": 1.50,
    "arc_residual_amplitude_p90_max_rad_s": 0.15,
    "arc_wrong_direction_ratio_max": 0.05,
    "primitive_mismatch_ratio_max": 0.15,
    "target_steering_alignment_ratio_min": 0.75,
    "loop_expected_hz": 50.0,
    "loop_frequency_p10_min_hz": 40.0,
    "loop_frequency_below_45_ratio_max": 0.10,
    "loop_dt_p95_max_s": 0.030,
    "loop_jitter_p95_max_s": 0.012,
    "sensor_rate_disagreement_p90_max_rad_s": 0.20,
    "sensor_endpoint_heading_disagreement_max_deg": 15.0,
    "min_heading_change_for_endpoint_gate_deg": 8.0,
    "moving_v_min_mps": 0.020,
    "moving_omega_min_rad_s": 0.040,
    "linear_tracking_command_min_mps": 0.030,
    "omega_tracking_command_min_rad_s": 0.060,
    "low_speed_track_ref_max_mps": 0.015,
    "low_speed_track_ref_min_mps": 0.003,
    "low_speed_track_min_samples": 8,
    "low_speed_track_error_abs_p90_max_mps": 0.035,
    "low_speed_track_overshoot_ratio_max": 0.10,
    "centered_target_max_deg": 12.0,
    "target_steering_min_deg": 10.0,
    "straight_command_omega_max_rad_s": 0.035,
    "arc_command_omega_min_rad_s": 0.050,
    "oscillation_deadband_rad_s": 0.025,
    "pwm_active_min": 0.01,
    "wheel_ref_active_min_mps": 0.025,
    "encoder_quantized_pulses_max": 1,
}


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return float(default)
    return result if math.isfinite(result) else float(default)


def _json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(_json_safe(payload), ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")
    tmp.replace(path)


def _write_jsonl(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(_json_safe(row), ensure_ascii=False, separators=(",", ":")) + "\n")
    tmp.replace(path)


def _read_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    try:
        with path.open("r", encoding="utf-8") as handle:
            for raw in handle:
                try:
                    row = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                if isinstance(row, dict):
                    rows.append(row)
    except FileNotFoundError:
        pass
    return rows


def _percentile(values: Iterable[float], fraction: float) -> Optional[float]:
    finite = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not finite:
        return None
    position = (len(finite) - 1) * min(1.0, max(0.0, float(fraction)))
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return float(finite[lower])
    weight = position - lower
    return float(finite[lower] * (1.0 - weight) + finite[upper] * weight)


def _mean(values: Iterable[float]) -> Optional[float]:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    return float(statistics.fmean(finite)) if finite else None


def _std(values: Iterable[float]) -> Optional[float]:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    return float(statistics.pstdev(finite)) if len(finite) > 1 else (0.0 if finite else None)


def _correlation(left: Sequence[float], right: Sequence[float]) -> Optional[float]:
    pairs = [(float(a), float(b)) for a, b in zip(left, right) if math.isfinite(float(a)) and math.isfinite(float(b))]
    if len(pairs) < 4:
        return None
    xs, ys = zip(*pairs)
    mx = statistics.fmean(xs)
    my = statistics.fmean(ys)
    numerator = sum((x - mx) * (y - my) for x, y in pairs)
    denominator = math.sqrt(sum((x - mx) ** 2 for x in xs) * sum((y - my) ** 2 for y in ys))
    return float(numerator / denominator) if denominator > 1e-12 else None


def _angle_delta_deg(a: float, b: float) -> float:
    return float((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def _ratio(count: int, total: int) -> Optional[float]:
    return float(count) / float(total) if total > 0 else None


def _compact_number(value: Any) -> Any:
    if isinstance(value, float):
        return round(value, 6) if math.isfinite(value) else None
    return value


def _evidence(samples: Sequence[Dict[str, Any]], indices: Iterable[int], limit: int = 6) -> List[Dict[str, Any]]:
    fields = (
        "sample_index",
        "elapsed_s",
        "m3_causes",
        "adaptive_target_angle_deg",
        "adaptive_target_dist_m",
        "requested_v_mps",
        "requested_omega_rad_s",
        "actual_linear_mps",
        "actual_omega_rad_s",
        "target_track_left_mps",
        "target_track_right_mps",
        "actual_track_left_mps",
        "actual_track_right_mps",
        "pwm_left",
        "pwm_right",
        "watchdog_freq_hz",
        "straight_hold_correction_rad_s",
        "localization_gate_mode",
        "stop_reason",
        "active_route",
    )
    output: List[Dict[str, Any]] = []
    seen = set()
    for index in indices:
        if index in seen or index < 0 or index >= len(samples):
            continue
        seen.add(index)
        sample = samples[index]
        output.append({key: _compact_number(sample.get(key)) for key in fields if key in sample})
        if len(output) >= limit:
            break
    return output


def _gate(
    status: str,
    *,
    observed: Any,
    requirement: str,
    required: bool = True,
    evidence: Optional[List[Dict[str, Any]]] = None,
    reason: str = "",
) -> Dict[str, Any]:
    return {
        "status": str(status),
        "required": bool(required),
        "observed": observed,
        "requirement": str(requirement),
        "reason": str(reason),
        "evidence": list(evidence or []),
    }


def _max_counter_delta(samples: Sequence[Dict[str, Any]], key: str) -> int:
    values = [int(sample.get(key, 0) or 0) for sample in samples]
    return max(0, max(values) - min(values)) if values else 0


def _meaningful_sign_changes(values: Sequence[float], deadband: float) -> int:
    signs: List[int] = []
    for value in values:
        sign = 1 if value > deadband else (-1 if value < -deadband else 0)
        if sign and (not signs or sign != signs[-1]):
            signs.append(sign)
    return max(0, len(signs) - 1)


def _episode_count(indices: Sequence[int]) -> int:
    episodes = 0
    previous: Optional[int] = None
    for index in sorted(set(int(value) for value in indices)):
        if previous is None or index != previous + 1:
            episodes += 1
        previous = index
    return episodes


def _oscillation_metrics(samples: Sequence[Dict[str, Any]], indices: Sequence[int], deadband: float) -> Dict[str, Any]:
    residuals = [_safe_float(samples[index].get("m3_residual_omega_rad_s"), 0.0) for index in indices]
    median = _percentile(residuals, 0.50)
    centered = [value - float(median or 0.0) for value in residuals]
    sign_changes = _meaningful_sign_changes(centered, deadband)
    duration = 0.0
    if len(indices) > 1:
        duration = max(
            0.0,
            _safe_float(samples[indices[-1]].get("elapsed_s"), 0.0)
            - _safe_float(samples[indices[0]].get("elapsed_s"), 0.0),
        )
    return {
        "sample_count": len(indices),
        "bias_rad_s": median,
        "amplitude_p90_rad_s": _percentile((abs(value) for value in centered), 0.90),
        "amplitude_peak_rad_s": _percentile((abs(value) for value in centered), 1.0),
        "sign_change_count": sign_changes,
        "approx_frequency_hz": float(sign_changes / (2.0 * duration)) if duration > 0.0 else None,
        "duration_s": duration,
    }


def _classify_samples(samples: Sequence[Dict[str, Any]], thresholds: Dict[str, float]) -> List[Dict[str, Any]]:
    enriched: List[Dict[str, Any]] = []
    previous: Optional[Dict[str, Any]] = None
    expected_dt = 1.0 / max(1.0, thresholds["loop_expected_hz"])
    for index, original in enumerate(samples):
        sample = dict(original)
        requested_v = _safe_float(sample.get("requested_v_mps", sample.get("expected_v")), 0.0)
        requested_omega = _safe_float(sample.get("requested_omega_rad_s", sample.get("expected_omega")), 0.0)
        actual_omega = _safe_float(
            sample.get("actual_omega_rad_s"),
            math.radians(_safe_float(sample.get("actual_angular_dps"), 0.0)),
        )
        target_angle = _safe_float(sample.get("adaptive_target_angle_deg"), 0.0)
        usable = bool(sample.get("target_camera_usable", False))
        locked = bool(sample.get("target_camera_lock_confirmed", False))
        moving = bool(
            abs(requested_v) >= thresholds["moving_v_min_mps"]
            or abs(requested_omega) >= thresholds["moving_omega_min_rad_s"]
        )
        target_steering = bool(
            usable
            and locked
            and abs(target_angle) >= thresholds["target_steering_min_deg"]
            and abs(requested_omega) >= thresholds["straight_command_omega_max_rad_s"]
        )
        target_steering_aligned = bool(
            target_steering
            and target_angle * requested_omega < 0.0
        )
        straight = bool(
            usable
            and locked
            and abs(requested_v) >= thresholds["linear_tracking_command_min_mps"]
            and abs(requested_omega) <= thresholds["straight_command_omega_max_rad_s"]
            and abs(target_angle) <= thresholds["centered_target_max_deg"]
        )
        arc = bool(
            usable
            and locked
            and abs(requested_v) >= thresholds["moving_v_min_mps"]
            and abs(requested_omega) >= thresholds["arc_command_omega_min_rad_s"]
        )
        heading_correction = bool(
            sample.get("straight_hold_active", False)
            and abs(_safe_float(sample.get("straight_hold_correction_rad_s"), 0.0)) > 0.005
        )
        period = _safe_float(sample.get("watchdog_period_s"), 0.0)
        loop_slow = bool(period > 1.0 / max(1.0, thresholds["loop_frequency_p10_min_hz"]))
        left_ref = _safe_float(sample.get("wheel_loop_left_ref_mps", sample.get("target_track_left_mps")), 0.0)
        right_ref = _safe_float(sample.get("wheel_loop_right_ref_mps", sample.get("target_track_right_mps")), 0.0)
        left_meas = _safe_float(sample.get("wheel_loop_left_meas_mps", sample.get("actual_track_left_mps")), 0.0)
        right_meas = _safe_float(sample.get("wheel_loop_right_meas_mps", sample.get("actual_track_right_mps")), 0.0)
        track_error_abs = max(abs(left_meas - left_ref), abs(right_meas - right_ref))
        low_speed_track_exec = bool(
            str(sample.get("motion_execution_mode") or sample.get("execution_mode") or "").strip().upper()
            == "TRACK_EXEC"
            and max(abs(left_ref), abs(right_ref)) >= thresholds["low_speed_track_ref_min_mps"]
            and max(abs(left_ref), abs(right_ref)) <= thresholds["low_speed_track_ref_max_mps"]
        )
        low_speed_track_overshoot = bool(
            low_speed_track_exec
            and track_error_abs > thresholds["low_speed_track_error_abs_p90_max_mps"]
        )
        pwm_step_left = 0.0
        pwm_step_right = 0.0
        refs_stable = False
        if previous is not None:
            pwm_step_left = abs(_safe_float(sample.get("pwm_left"), 0.0) - _safe_float(previous.get("pwm_left"), 0.0))
            pwm_step_right = abs(_safe_float(sample.get("pwm_right"), 0.0) - _safe_float(previous.get("pwm_right"), 0.0))
            refs_stable = bool(
                abs(left_ref - _safe_float(previous.get("wheel_loop_left_ref_mps", previous.get("target_track_left_mps")), 0.0)) < 0.008
                and abs(right_ref - _safe_float(previous.get("wheel_loop_right_ref_mps", previous.get("target_track_right_mps")), 0.0)) < 0.008
            )
        quantized = bool(
            moving
            and refs_stable
            and max(abs(int(sample.get("encoder_pulses_left", 0) or 0)), abs(int(sample.get("encoder_pulses_right", 0) or 0)))
            <= int(thresholds["encoder_quantized_pulses_max"])
            and max(pwm_step_left, pwm_step_right) > 0.02
        )
        residual = actual_omega - requested_omega
        causes: List[str] = []
        if target_steering:
            causes.append("TARGET_REQUIRED_STEERING")
        if straight and abs(residual) > thresholds["oscillation_deadband_rad_s"]:
            causes.append("ROBOT_STRAIGHT_RESIDUAL")
        if heading_correction:
            causes.append("HEADING_ESTIMATOR_CORRECTION")
        if quantized:
            causes.append("WHEEL_LOOP_ENCODER_QUANTIZATION")
        if low_speed_track_overshoot:
            causes.append("LOW_SPEED_TRACK_OVERSHOOT")
        if loop_slow:
            causes.append("CONTROL_LOOP_SLOW")
        if loop_slow and max(pwm_step_left, pwm_step_right) > 0.02:
            causes.append("LOOP_JITTER_PWM_ASSOCIATION")
        sample.update(
            {
                "sample_index": index,
                "m3_follow_locked": locked,
                "m3_moving": moving,
                "m3_target_required_steering": target_steering,
                "m3_target_steering_aligned": target_steering_aligned,
                "m3_straight_window": straight,
                "m3_arc_window": arc,
                "m3_heading_estimator_correction": heading_correction,
                "m3_wheel_quantization_suspect": quantized,
                "m3_low_speed_track_exec": low_speed_track_exec,
                "m3_low_speed_track_overshoot": low_speed_track_overshoot,
                "m3_track_error_abs_mps": track_error_abs,
                "m3_loop_slow": loop_slow,
                "m3_residual_omega_rad_s": residual,
                "m3_loop_jitter_s": abs(period - expected_dt) if period > 0.0 else None,
                "m3_pwm_step_left": pwm_step_left,
                "m3_pwm_step_right": pwm_step_right,
                "m3_causes": causes,
            }
        )
        enriched.append(sample)
        previous = sample
    return enriched


def _sensor_endpoint_metrics(samples: Sequence[Dict[str, Any]], moving_indices: Sequence[int]) -> Dict[str, Any]:
    if len(moving_indices) < 2:
        return {"available": False, "reason": "insufficient_moving_samples"}
    first = samples[moving_indices[0]]
    last = samples[moving_indices[-1]]
    sources = {
        "imu": (first.get("imu_heading_deg"), last.get("imu_heading_deg")),
        "ekf": (first.get("pose_theta_deg"), last.get("pose_theta_deg")),
        "lidar": (first.get("lidar_heading_deg"), last.get("lidar_heading_deg")),
    }
    changes: Dict[str, float] = {}
    for name, (start, end) in sources.items():
        try:
            start_f = float(start)
            end_f = float(end)
        except (TypeError, ValueError):
            continue
        if math.isfinite(start_f) and math.isfinite(end_f):
            changes[name] = _angle_delta_deg(end_f, start_f)
    pairs: Dict[str, float] = {}
    names = sorted(changes)
    for left_index, left in enumerate(names):
        for right in names[left_index + 1 :]:
            pairs[f"{left}_vs_{right}"] = abs(changes[left] - changes[right])
    return {
        "available": len(changes) >= 2,
        "heading_change_deg": changes,
        "pair_disagreement_deg": pairs,
        "max_pair_disagreement_deg": max(pairs.values()) if pairs else None,
        "max_observed_heading_change_deg": max((abs(value) for value in changes.values()), default=0.0),
        "start_sample_index": moving_indices[0],
        "end_sample_index": moving_indices[-1],
    }


def analyze_samples(
    raw_samples: Sequence[Dict[str, Any]],
    thresholds: Optional[Dict[str, float]] = None,
    *,
    base_result: Optional[Dict[str, Any]] = None,
) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    limits = dict(DEFAULT_THRESHOLDS)
    if thresholds:
        limits.update({key: float(value) for key, value in thresholds.items() if key in limits})
    samples = _classify_samples(raw_samples, limits)
    total = len(samples)
    follow_indices = [i for i, sample in enumerate(samples) if str(sample.get("state") or "") == "FOLLOW"]
    if not follow_indices:
        follow_indices = list(range(total))
    visible_indices = [i for i in follow_indices if bool(samples[i].get("target_camera_visible", False))]
    usable_indices = [i for i in follow_indices if bool(samples[i].get("target_camera_usable", False))]
    locked_indices = [i for i in follow_indices if bool(samples[i].get("target_camera_lock_confirmed", False))]
    moving_indices = [i for i, sample in enumerate(samples) if bool(sample.get("m3_moving", False))]
    straight_indices = [i for i, sample in enumerate(samples) if bool(sample.get("m3_straight_window", False))]
    arc_indices = [i for i, sample in enumerate(samples) if bool(sample.get("m3_arc_window", False))]
    target_steering_indices = [i for i, sample in enumerate(samples) if bool(sample.get("m3_target_required_steering", False))]
    heading_correction_indices = [i for i, sample in enumerate(samples) if bool(sample.get("m3_heading_estimator_correction", False))]
    quantization_indices = [i for i, sample in enumerate(samples) if bool(sample.get("m3_wheel_quantization_suspect", False))]
    slow_loop_indices = [i for i, sample in enumerate(samples) if bool(sample.get("m3_loop_slow", False))]

    visible_ratio = _ratio(len(visible_indices), len(follow_indices))
    lock_ratio = _ratio(len(locked_indices), len(follow_indices))
    loss_count = _max_counter_delta(samples, "target_camera_lost_count")
    relock_count = _max_counter_delta(samples, "target_camera_relock_count")
    relock_ratio = float(relock_count / loss_count) if loss_count > 0 else None
    bearing_values = [abs(_safe_float(samples[i].get("adaptive_target_angle_deg"), 0.0)) for i in usable_indices]
    distances = [
        _safe_float(samples[i].get("adaptive_target_dist_m", samples[i].get("target_camera_distance_used_m")), 0.0)
        for i in usable_indices
    ]
    distances = [value for value in distances if value > 0.05]
    target_distance = _percentile(
        (_safe_float(samples[i].get("follow_tuning_target_distance_m"), 0.0) for i in follow_indices),
        0.50,
    )
    if not target_distance or target_distance <= 0.0:
        target_distance = 1.0
    distance_errors = [abs(value - target_distance) for value in distances]

    linear_indices = [
        i for i in moving_indices
        if abs(_safe_float(samples[i].get("requested_v_mps"), 0.0)) >= limits["linear_tracking_command_min_mps"]
    ]
    omega_indices = [
        i for i in moving_indices
        if abs(_safe_float(samples[i].get("requested_omega_rad_s"), 0.0)) >= limits["omega_tracking_command_min_rad_s"]
    ]
    linear_relative_errors = [
        abs(_safe_float(samples[i].get("actual_linear_mps"), 0.0) - _safe_float(samples[i].get("requested_v_mps"), 0.0))
        / abs(_safe_float(samples[i].get("requested_v_mps"), 0.0))
        for i in linear_indices
    ]
    omega_relative_errors = [
        abs(_safe_float(samples[i].get("actual_omega_rad_s"), 0.0) - _safe_float(samples[i].get("requested_omega_rad_s"), 0.0))
        / abs(_safe_float(samples[i].get("requested_omega_rad_s"), 0.0))
        for i in omega_indices
    ]
    wheel_abs_errors: List[float] = []
    wheel_relative_errors: List[float] = []
    for index in moving_indices:
        for side in ("left", "right"):
            reference = _safe_float(
                samples[index].get(f"wheel_loop_{side}_ref_mps", samples[index].get(f"target_track_{side}_mps")),
                0.0,
            )
            measured = _safe_float(
                samples[index].get(f"wheel_loop_{side}_meas_mps", samples[index].get(f"actual_track_{side}_mps")),
                0.0,
            )
            if abs(reference) >= limits["wheel_ref_active_min_mps"]:
                error = abs(measured - reference)
                wheel_abs_errors.append(error)
                wheel_relative_errors.append(error / abs(reference))

    pwm_steps = [
        max(_safe_float(samples[i].get("m3_pwm_step_left"), 0.0), _safe_float(samples[i].get("m3_pwm_step_right"), 0.0))
        for i in moving_indices if i > 0
    ]
    dropout_indices = [
        i for i in moving_indices
        if (
            abs(_safe_float(samples[i].get("target_track_left_mps"), 0.0)) >= limits["wheel_ref_active_min_mps"]
            and abs(_safe_float(samples[i].get("pwm_left"), 0.0)) < limits["pwm_active_min"]
        )
        or (
            abs(_safe_float(samples[i].get("target_track_right_mps"), 0.0)) >= limits["wheel_ref_active_min_mps"]
            and abs(_safe_float(samples[i].get("pwm_right"), 0.0)) < limits["pwm_active_min"]
        )
    ]
    wheel_stop_indices = [
        i for i in moving_indices
        if max(
            abs(_safe_float(samples[i].get("target_track_left_mps"), 0.0)),
            abs(_safe_float(samples[i].get("target_track_right_mps"), 0.0)),
        ) >= limits["wheel_ref_active_min_mps"]
        and max(
            abs(_safe_float(samples[i].get("actual_track_left_mps"), 0.0)),
            abs(_safe_float(samples[i].get("actual_track_right_mps"), 0.0)),
        ) < 0.008
        and max(
            abs(_safe_float(samples[i].get("pwm_left"), 0.0)),
            abs(_safe_float(samples[i].get("pwm_right"), 0.0)),
        ) >= limits["pwm_active_min"]
    ]
    pwm_sign_reversals = 0
    reversal_indices: List[int] = []
    for index in moving_indices:
        if index <= 0:
            continue
        previous = samples[index - 1]
        current = samples[index]
        for side in ("left", "right"):
            before = _safe_float(previous.get(f"pwm_{side}"), 0.0)
            after = _safe_float(current.get(f"pwm_{side}"), 0.0)
            ref = _safe_float(current.get(f"target_track_{side}_mps"), 0.0)
            if before * after < 0.0 and abs(ref) >= limits["wheel_ref_active_min_mps"]:
                pwm_sign_reversals += 1
                reversal_indices.append(index)

    straight_oscillation = _oscillation_metrics(samples, straight_indices, limits["oscillation_deadband_rad_s"])
    arc_oscillation = _oscillation_metrics(samples, arc_indices, limits["oscillation_deadband_rad_s"])
    arc_wrong_indices = [
        i for i in arc_indices
        if _safe_float(samples[i].get("requested_omega_rad_s"), 0.0)
        * _safe_float(samples[i].get("actual_omega_rad_s"), 0.0) < 0.0
    ]
    primitive_mismatch_indices = [
        i for i in moving_indices
        if str(samples[i].get("turn_primitive_requested") or "")
        and str(samples[i].get("turn_primitive_actual") or "")
        and str(samples[i].get("turn_primitive_requested")) != str(samples[i].get("turn_primitive_actual"))
    ]
    low_speed_track_indices = [
        i for i in moving_indices
        if bool(samples[i].get("m3_low_speed_track_exec", False))
    ]
    low_speed_track_overshoot_indices = [
        i for i in low_speed_track_indices
        if bool(samples[i].get("m3_low_speed_track_overshoot", False))
    ]
    low_speed_track_errors = [
        _safe_float(samples[i].get("m3_track_error_abs_mps"), 0.0)
        for i in low_speed_track_indices
    ]
    target_alignment_indices = [
        i for i in target_steering_indices
        if bool(samples[i].get("m3_target_steering_aligned", False))
    ]

    periods = [_safe_float(sample.get("watchdog_period_s"), 0.0) for sample in samples]
    periods = [value for value in periods if value > 0.0]
    frequencies = [_safe_float(sample.get("watchdog_freq_hz"), 0.0) for sample in samples]
    frequencies = [value for value in frequencies if value > 0.0]
    expected_dt = 1.0 / max(1.0, limits["loop_expected_hz"])
    jitters = [abs(value - expected_dt) for value in periods]
    below_45_ratio = _ratio(sum(1 for value in frequencies if value < 45.0), len(frequencies))
    slow_residual = [abs(_safe_float(samples[i].get("m3_residual_omega_rad_s"), 0.0)) for i in slow_loop_indices]
    normal_residual = [
        abs(_safe_float(sample.get("m3_residual_omega_rad_s"), 0.0))
        for sample in samples if not bool(sample.get("m3_loop_slow", False))
    ]

    rate_disagreements: List[float] = []
    rate_disagreement_indices: List[int] = []
    for index in moving_indices:
        values = [
            _safe_float(samples[index].get("imu_gyro_z_rad_s"), math.nan),
            _safe_float(samples[index].get("ekf_omega_rad_s"), math.nan),
            _safe_float(samples[index].get("encoder_yaw_rate_rad_s"), math.nan),
        ]
        finite = [value for value in values if math.isfinite(value)]
        if len(finite) >= 2:
            disagreement = max(finite) - min(finite)
            rate_disagreements.append(disagreement)
            if disagreement > limits["sensor_rate_disagreement_p90_max_rad_s"]:
                rate_disagreement_indices.append(index)
    endpoint = _sensor_endpoint_metrics(samples, moving_indices)

    forbidden_indices = [
        i for i, sample in enumerate(samples)
        if bool(sample.get("direct_motor_bypass", False))
        or bool(sample.get("used_legacy_generic_planner", False))
        or bool(sample.get("service_motion_active", False))
        or bool(sample.get("actuator_service_active", False))
        or str(sample.get("active_motion_type") or "") == "set_motor_pwm"
    ]
    route_bad_indices = [
        i for i in moving_indices
        if str(samples[i].get("active_route") or "") != live.HUMAN_FOLLOW_V2_ROUTE
    ]
    safety_indices = [
        i for i, sample in enumerate(samples)
        if not bool(sample.get("safety_allow", True))
        or bool(sample.get("watchdog_stop_triggered", False))
        or str(sample.get("last_emergency_reason") or "")
        or str(sample.get("safety_limiting_reason") or "")
    ]
    failsafe_events = max((int(sample.get("global_motion_failsafe_events", 0) or 0) for sample in samples), default=0)
    localization_contradiction_indices = [
        i for i, sample in enumerate(samples)
        if (
            str(sample.get("localization_gate_mode") or "").upper() == "LOST"
            and (
                bool(sample.get("localization_gate_allow_motion", False))
                or _safe_float(sample.get("localization_gate_trust"), 0.0) >= 0.99
            )
        )
        or not bool(sample.get("localization_truth_consistent", True))
        or (
            str(sample.get("localization_truth_state") or "").upper() == "LOST"
            and bool(sample.get("localization_truth_allow_motion", False))
        )
    ]
    contract_indices = [
        i for i, sample in enumerate(samples)
        if bool(sample.get("primitive_contract_violation", False))
        or bool(sample.get("control_execution_contract_violation", False))
    ]

    metrics: Dict[str, Any] = {
        "coverage": {
            "total_samples": total,
            "follow_samples": len(follow_indices),
            "moving_samples": len(moving_indices),
            "straight_samples": len(straight_indices),
            "arc_samples": len(arc_indices),
            "duration_s": (
                _safe_float(samples[-1].get("elapsed_s"), 0.0) - _safe_float(samples[0].get("elapsed_s"), 0.0)
                if len(samples) > 1 else 0.0
            ),
        },
        "target": {
            "visible_ratio": visible_ratio,
            "usable_ratio": _ratio(len(usable_indices), len(follow_indices)),
            "lock_ratio": lock_ratio,
            "loss_count": loss_count,
            "relock_count": relock_count,
            "relock_ratio_after_loss": relock_ratio,
            "bearing_abs_p50_deg": _percentile(bearing_values, 0.50),
            "bearing_abs_p90_deg": _percentile(bearing_values, 0.90),
            "distance_target_m": target_distance,
            "distance_error_abs_p50_m": _percentile(distance_errors, 0.50),
            "distance_error_abs_p90_m": _percentile(distance_errors, 0.90),
            "distance_std_m": _std(distances),
            "required_steering_samples": len(target_steering_indices),
            "steering_alignment_ratio": _ratio(len(target_alignment_indices), len(target_steering_indices)),
        },
        "motion_tracking": {
            "linear_samples": len(linear_indices),
            "linear_relative_error_p50": _percentile(linear_relative_errors, 0.50),
            "linear_relative_error_p90": _percentile(linear_relative_errors, 0.90),
            "omega_samples": len(omega_indices),
            "omega_relative_error_p50": _percentile(omega_relative_errors, 0.50),
            "omega_relative_error_p90": _percentile(omega_relative_errors, 0.90),
            "wheel_error_abs_p50_mps": _percentile(wheel_abs_errors, 0.50),
            "wheel_error_abs_p90_mps": _percentile(wheel_abs_errors, 0.90),
            "wheel_error_relative_p90": _percentile(wheel_relative_errors, 0.90),
            "primitive_mismatch_ratio": _ratio(len(primitive_mismatch_indices), len(moving_indices)),
            "low_speed_track_samples": len(low_speed_track_indices),
            "low_speed_track_error_abs_p50_mps": _percentile(low_speed_track_errors, 0.50),
            "low_speed_track_error_abs_p90_mps": _percentile(low_speed_track_errors, 0.90),
            "low_speed_track_overshoot_count": len(low_speed_track_overshoot_indices),
            "low_speed_track_overshoot_ratio": _ratio(
                len(low_speed_track_overshoot_indices),
                len(low_speed_track_indices),
            ),
        },
        "pwm": {
            "step_p50": _percentile(pwm_steps, 0.50),
            "step_p95": _percentile(pwm_steps, 0.95),
            "dropout_count": len(dropout_indices),
            "dropout_ratio": _ratio(len(dropout_indices), len(moving_indices)),
            "sign_reversal_count": pwm_sign_reversals,
            "wheel_stop_start_sample_count": len(wheel_stop_indices),
            "wheel_stop_start_episode_count": _episode_count(wheel_stop_indices),
            "wheel_stop_start_ratio": _ratio(len(wheel_stop_indices), len(moving_indices)),
        },
        "oscillation": {
            "straight": straight_oscillation,
            "arc": arc_oscillation,
            "arc_wrong_direction_ratio": _ratio(len(arc_wrong_indices), len(arc_indices)),
        },
        "control_loop": {
            "expected_hz": limits["loop_expected_hz"],
            "frequency_p10_hz": _percentile(frequencies, 0.10),
            "frequency_p50_hz": _percentile(frequencies, 0.50),
            "frequency_below_45_ratio": below_45_ratio,
            "dt_p50_s": _percentile(periods, 0.50),
            "dt_p95_s": _percentile(periods, 0.95),
            "jitter_p95_s": _percentile(jitters, 0.95),
            "slow_sample_count": len(slow_loop_indices),
            "residual_omega_abs_slow_p50_rad_s": _percentile(slow_residual, 0.50),
            "residual_omega_abs_normal_p50_rad_s": _percentile(normal_residual, 0.50),
            "slow_loop_pwm_association_count": sum(
                1 for sample in samples if "LOOP_JITTER_PWM_ASSOCIATION" in list(sample.get("m3_causes") or [])
            ),
        },
        "estimator_relationship": {
            "sensor_rate_disagreement_p90_rad_s": _percentile(rate_disagreements, 0.90),
            "endpoint": endpoint,
            "heading_correction_samples": len(heading_correction_indices),
            "heading_error_correction_correlation": _correlation(
                [_safe_float(samples[i].get("straight_hold_heading_error_deg"), 0.0) for i in heading_correction_indices],
                [_safe_float(samples[i].get("straight_hold_correction_rad_s"), 0.0) for i in heading_correction_indices],
            ),
        },
        "cause_separation": {
            "target_required_steering_samples": len(target_steering_indices),
            "robot_straight_residual_samples": sum(
                1 for sample in samples if "ROBOT_STRAIGHT_RESIDUAL" in list(sample.get("m3_causes") or [])
            ),
            "heading_estimator_correction_samples": len(heading_correction_indices),
            "wheel_loop_encoder_quantization_samples": len(quantization_indices),
            "control_loop_slow_samples": len(slow_loop_indices),
        },
        "integrity": {
            "forbidden_path_samples": len(forbidden_indices),
            "wrong_route_moving_samples": len(route_bad_indices),
            "safety_event_samples": len(safety_indices),
            "failsafe_events": failsafe_events,
            "localization_contradiction_samples": len(localization_contradiction_indices),
            "execution_contract_violation_samples": len(contract_indices),
        },
    }

    gates: Dict[str, Dict[str, Any]] = {}
    enough_total = total >= int(limits["min_total_samples"])
    gates["sample_coverage"] = _gate(
        "PASS" if enough_total and len(follow_indices) >= int(limits["min_follow_samples"]) else "INCONCLUSIVE",
        observed=metrics["coverage"],
        requirement=f"total >= {int(limits['min_total_samples'])}, follow >= {int(limits['min_follow_samples'])}",
        reason="not_enough_live_samples" if not enough_total else "",
    )

    def numeric_gate(name: str, value: Optional[float], limit: float, relation: str, *, required: bool = True) -> None:
        if value is None:
            status = "INCONCLUSIVE"
        elif relation == "max":
            status = "PASS" if value <= limit else "FAIL"
        else:
            status = "PASS" if value >= limit else "FAIL"
        gates[name] = _gate(status, observed=value, requirement=f"{relation} {limit}", required=required)

    numeric_gate("target_visibility", visible_ratio, limits["target_visible_ratio_min"], "min")
    numeric_gate("target_lock", lock_ratio, limits["target_lock_ratio_min"], "min")
    numeric_gate("target_centering", metrics["target"]["bearing_abs_p90_deg"], limits["target_bearing_abs_p90_max_deg"], "max")
    numeric_gate("following_distance_accuracy", metrics["target"]["distance_error_abs_p90_m"], limits["target_distance_error_abs_p90_max_m"], "max")
    numeric_gate("following_distance_stability", metrics["target"]["distance_std_m"], limits["target_distance_std_max_m"], "max")
    if loss_count > 0:
        numeric_gate("relock_after_loss", relock_ratio, 1.0, "min")
    else:
        gates["relock_after_loss"] = _gate(
            "INCONCLUSIVE", observed={"loss_count": 0, "relock_count": relock_count},
            requirement="relock for every observed loss", required=False, reason="no_target_loss_observed",
        )
    numeric_gate("target_steering_alignment", metrics["target"]["steering_alignment_ratio"], limits["target_steering_alignment_ratio_min"], "min")

    numeric_gate("linear_speed_tracking", metrics["motion_tracking"]["linear_relative_error_p90"], limits["linear_tracking_relative_p90_max"], "max")
    numeric_gate("angular_speed_tracking", metrics["motion_tracking"]["omega_relative_error_p90"], limits["omega_tracking_relative_p90_max"], "max")
    wheel_abs = metrics["motion_tracking"]["wheel_error_abs_p90_mps"]
    wheel_rel = metrics["motion_tracking"]["wheel_error_relative_p90"]
    if wheel_abs is None or wheel_rel is None:
        wheel_status = "INCONCLUSIVE"
    else:
        wheel_status = "PASS" if wheel_abs <= limits["wheel_tracking_abs_p90_max_mps"] and wheel_rel <= limits["wheel_tracking_relative_p90_max"] else "FAIL"
    gates["wheel_speed_tracking"] = _gate(
        wheel_status,
        observed={"abs_p90_mps": wheel_abs, "relative_p90": wheel_rel},
        requirement=(
            f"abs p90 <= {limits['wheel_tracking_abs_p90_max_mps']} and "
            f"relative p90 <= {limits['wheel_tracking_relative_p90_max']}"
        ),
    )
    numeric_gate("primitive_classification", metrics["motion_tracking"]["primitive_mismatch_ratio"], limits["primitive_mismatch_ratio_max"], "max")
    low_speed_track_count = int(metrics["motion_tracking"]["low_speed_track_samples"])
    low_speed_track_p90 = metrics["motion_tracking"]["low_speed_track_error_abs_p90_mps"]
    low_speed_track_ratio = metrics["motion_tracking"]["low_speed_track_overshoot_ratio"]
    if low_speed_track_count < int(limits["low_speed_track_min_samples"]):
        gates["low_speed_track_fidelity"] = _gate(
            "INCONCLUSIVE",
            observed=metrics["motion_tracking"],
            requirement=f"samples >= {int(limits['low_speed_track_min_samples'])}",
            required=False,
            reason="low_speed_track_window_not_exercised",
        )
    else:
        low_speed_track_ok = bool(
            _safe_float(low_speed_track_p90, math.inf) <= limits["low_speed_track_error_abs_p90_max_mps"]
            and _safe_float(low_speed_track_ratio, math.inf) <= limits["low_speed_track_overshoot_ratio_max"]
        )
        gates["low_speed_track_fidelity"] = _gate(
            "PASS" if low_speed_track_ok else "FAIL",
            observed={
                "sample_count": low_speed_track_count,
                "error_abs_p90_mps": low_speed_track_p90,
                "overshoot_ratio": low_speed_track_ratio,
            },
            requirement=(
                f"error p90 <= {limits['low_speed_track_error_abs_p90_max_mps']} m/s and "
                f"overshoot ratio <= {limits['low_speed_track_overshoot_ratio_max']}"
            ),
        )

    numeric_gate("pwm_smoothness", metrics["pwm"]["step_p95"], limits["pwm_step_p95_max"], "max")
    numeric_gate("pwm_stop_start", metrics["pwm"]["dropout_ratio"], limits["pwm_dropout_ratio_max"], "max")
    numeric_gate("pwm_sign_reversal", float(pwm_sign_reversals), limits["pwm_sign_reversal_max"], "max")
    numeric_gate(
        "wheel_stop_start",
        metrics["pwm"]["wheel_stop_start_ratio"],
        limits["wheel_stop_start_ratio_max"],
        "max",
    )

    if len(straight_indices) < int(limits["min_straight_samples"]):
        gates["straight_oscillation"] = _gate(
            "INCONCLUSIVE", observed=straight_oscillation,
            requirement=f"samples >= {int(limits['min_straight_samples'])}", reason="straight_window_not_exercised",
        )
    else:
        straight_ok = bool(
            _safe_float(straight_oscillation.get("amplitude_p90_rad_s"), math.inf)
            <= limits["straight_oscillation_amplitude_p90_max_rad_s"]
            and _safe_float(straight_oscillation.get("approx_frequency_hz"), math.inf)
            <= limits["straight_oscillation_frequency_max_hz"]
        )
        gates["straight_oscillation"] = _gate(
            "PASS" if straight_ok else "FAIL", observed=straight_oscillation,
            requirement=(
                f"amplitude p90 <= {limits['straight_oscillation_amplitude_p90_max_rad_s']} rad/s and "
                f"frequency <= {limits['straight_oscillation_frequency_max_hz']} Hz"
            ),
        )
    if len(arc_indices) < int(limits["min_arc_samples"]):
        gates["arc_motion_quality"] = _gate(
            "INCONCLUSIVE", observed={**arc_oscillation, "wrong_direction_ratio": metrics["oscillation"]["arc_wrong_direction_ratio"]},
            requirement=f"samples >= {int(limits['min_arc_samples'])}", reason="arc_window_not_exercised",
        )
    else:
        arc_ok = bool(
            _safe_float(arc_oscillation.get("amplitude_p90_rad_s"), math.inf)
            <= limits["arc_residual_amplitude_p90_max_rad_s"]
            and _safe_float(metrics["oscillation"]["arc_wrong_direction_ratio"], math.inf)
            <= limits["arc_wrong_direction_ratio_max"]
        )
        gates["arc_motion_quality"] = _gate(
            "PASS" if arc_ok else "FAIL",
            observed={**arc_oscillation, "wrong_direction_ratio": metrics["oscillation"]["arc_wrong_direction_ratio"]},
            requirement=(
                f"residual p90 <= {limits['arc_residual_amplitude_p90_max_rad_s']} rad/s and "
                f"wrong direction <= {limits['arc_wrong_direction_ratio_max']}"
            ),
        )

    loop_values = metrics["control_loop"]
    if not periods or not frequencies:
        loop_status = "INCONCLUSIVE"
    else:
        loop_status = "PASS" if (
            _safe_float(loop_values["frequency_p10_hz"], 0.0) >= limits["loop_frequency_p10_min_hz"]
            and _safe_float(loop_values["frequency_below_45_ratio"], 1.0) <= limits["loop_frequency_below_45_ratio_max"]
            and _safe_float(loop_values["dt_p95_s"], math.inf) <= limits["loop_dt_p95_max_s"]
            and _safe_float(loop_values["jitter_p95_s"], math.inf) <= limits["loop_jitter_p95_max_s"]
        ) else "FAIL"
    gates["control_loop_cadence"] = _gate(
        loop_status, observed=loop_values,
        requirement="50 Hz target; p10 >= 40 Hz, below-45 ratio <= 0.10, dt p95 <= 30 ms, jitter p95 <= 12 ms",
        evidence=_evidence(samples, slow_loop_indices),
    )

    numeric_gate("sensor_rate_consistency", metrics["estimator_relationship"]["sensor_rate_disagreement_p90_rad_s"], limits["sensor_rate_disagreement_p90_max_rad_s"], "max")
    endpoint_change = _safe_float(endpoint.get("max_observed_heading_change_deg"), 0.0)
    endpoint_disagreement = endpoint.get("max_pair_disagreement_deg")
    if not endpoint.get("available") or endpoint_change < limits["min_heading_change_for_endpoint_gate_deg"]:
        gates["sensor_endpoint_heading"] = _gate(
            "INCONCLUSIVE", observed=endpoint,
            requirement=f"heading excitation >= {limits['min_heading_change_for_endpoint_gate_deg']} deg then disagreement <= {limits['sensor_endpoint_heading_disagreement_max_deg']} deg",
            reason="insufficient_heading_excitation",
        )
    else:
        numeric_gate("sensor_endpoint_heading", endpoint_disagreement, limits["sensor_endpoint_heading_disagreement_max_deg"], "max")
        gates["sensor_endpoint_heading"]["observed"] = endpoint

    integrity_specs = {
        "forbidden_paths": (forbidden_indices, "no service, legacy, or direct-PWM sample"),
        "normal_v2_route": (route_bad_indices, "every moving sample uses human_follow_v2"),
        "safety_and_failsafe": (safety_indices + ([0] if failsafe_events else []), "no safety, emergency, watchdog-stop, or failsafe event"),
        "localization_consistency": (localization_contradiction_indices, "no LOST/allow-motion/trust contradiction"),
        "execution_contract": (contract_indices, "no execution-mode or primitive contract violation"),
    }
    for name, (indices, requirement) in integrity_specs.items():
        gates[name] = _gate(
            "PASS" if not indices else "FAIL", observed={"violation_count": len(indices)},
            requirement=requirement, evidence=_evidence(samples, indices),
        )

    base_status = str((base_result or {}).get("status") or "")
    if base_result is None:
        gates["base_human_follow_v2"] = _gate(
            "INCONCLUSIVE", observed=None, requirement="base Human Follow v2 run result is available", reason="missing_base_result",
        )
    else:
        gates["base_human_follow_v2"] = _gate(
            "PASS" if base_status == "PASS" else "FAIL",
            observed={"status": base_status, "errors": list((base_result or {}).get("errors") or [])[:12]},
            requirement="existing strict Human Follow v2 gate passes",
        )

    evidence_map = {
        "linear_speed_tracking": linear_indices,
        "angular_speed_tracking": omega_indices,
        "wheel_speed_tracking": moving_indices,
        "primitive_classification": primitive_mismatch_indices,
        "low_speed_track_fidelity": sorted(
            low_speed_track_indices,
            key=lambda i: _safe_float(samples[i].get("m3_track_error_abs_mps"), 0.0),
            reverse=True,
        ),
        "pwm_smoothness": sorted(moving_indices, key=lambda i: max(_safe_float(samples[i].get("m3_pwm_step_left"), 0.0), _safe_float(samples[i].get("m3_pwm_step_right"), 0.0)), reverse=True),
        "pwm_stop_start": dropout_indices,
        "pwm_sign_reversal": reversal_indices,
        "wheel_stop_start": wheel_stop_indices,
        "straight_oscillation": sorted(straight_indices, key=lambda i: abs(_safe_float(samples[i].get("m3_residual_omega_rad_s"), 0.0)), reverse=True),
        "arc_motion_quality": sorted(arc_indices, key=lambda i: abs(_safe_float(samples[i].get("m3_residual_omega_rad_s"), 0.0)), reverse=True),
        "sensor_rate_consistency": rate_disagreement_indices,
        "sensor_endpoint_heading": [endpoint.get("start_sample_index", -1), endpoint.get("end_sample_index", -1)],
        "target_steering_alignment": target_steering_indices,
    }
    for name, indices in evidence_map.items():
        if name in gates and not gates[name].get("evidence"):
            gates[name]["evidence"] = _evidence(samples, indices)

    required_gates = [gate for gate in gates.values() if bool(gate.get("required", True))]
    if any(gate.get("status") == "FAIL" for gate in required_gates):
        status = "FAIL"
    elif any(gate.get("status") == "INCONCLUSIVE" for gate in required_gates):
        status = "INCONCLUSIVE"
    else:
        status = "PASS"
    failed = [name for name, gate in gates.items() if gate.get("status") == "FAIL"]
    inconclusive = [name for name, gate in gates.items() if gate.get("status") == "INCONCLUSIVE"]
    plain_summary = _plain_summary(status, metrics, failed, inconclusive)
    result = {
        "schema": "M3_EMBERKOVETES_MOZGASMINOSEG_V1",
        "status": status,
        "success": status == "PASS",
        "generated_ts": time.time(),
        "thresholds": limits,
        "metrics": metrics,
        "gates": gates,
        "failed_gates": failed,
        "inconclusive_gates": inconclusive,
        "plain_summary_hu": plain_summary,
        "base_follow": {
            "status": base_status or "MISSING",
            "errors": list((base_result or {}).get("errors") or []),
        },
    }
    return result, samples


def _plain_summary(status: str, metrics: Dict[str, Any], failed: Sequence[str], inconclusive: Sequence[str]) -> str:
    target = metrics.get("target") or {}
    oscillation = metrics.get("oscillation") or {}
    loop = metrics.get("control_loop") or {}
    visible = target.get("visible_ratio")
    straight = (oscillation.get("straight") or {}).get("amplitude_p90_rad_s")
    arc = (oscillation.get("arc") or {}).get("amplitude_p90_rad_s")
    parts = [f"Eredmény: {status}."]
    if visible is not None:
        parts.append(f"A célpont a mérési idő {100.0 * float(visible):.0f}%-ában volt látható.")
    if straight is not None:
        parts.append(f"Az egyenesmeneti saját irányingadozás jellemző amplitúdója {float(straight):.3f} rad/s volt.")
    if arc is not None:
        parts.append(f"Az íves mozgás maradék irányingadozása {float(arc):.3f} rad/s volt.")
    if loop.get("frequency_p50_hz") is not None:
        parts.append(f"A vezérlési ciklus középértéke {float(loop['frequency_p50_hz']):.1f} Hz volt az elvárt 50 Hz helyett/mellett.")
    if failed:
        parts.append("Biztosan hibás területek: " + ", ".join(failed) + ".")
    if inconclusive:
        parts.append("További mozgásminta kell ezekhez: " + ", ".join(inconclusive) + ".")
    return " ".join(parts)


def _load_thresholds(path: str) -> Dict[str, float]:
    if not path:
        return {}
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("thresholds JSON must contain an object")
    unknown = sorted(set(payload) - set(DEFAULT_THRESHOLDS))
    if unknown:
        raise ValueError(f"unknown threshold keys: {unknown}")
    return {str(key): float(value) for key, value in payload.items()}


def _incident_payload(result: Dict[str, Any], samples: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    relevant_names = list(result.get("failed_gates") or []) + list(result.get("inconclusive_gates") or [])
    relevant_samples: List[Dict[str, Any]] = []
    seen = set()
    for name in relevant_names:
        gate = dict((result.get("gates") or {}).get(name) or {})
        for sample in list(gate.get("evidence") or []):
            index = sample.get("sample_index")
            if index in seen:
                continue
            seen.add(index)
            relevant_samples.append(sample)
            if len(relevant_samples) >= 30:
                break
    return {
        "schema": "M3_EMBERKOVETES_MOZGASMINOSEG_INCIDENT_V1",
        "needed": result.get("status") != "PASS",
        "status": result.get("status"),
        "failed_gates": list(result.get("failed_gates") or []),
        "inconclusive_gates": list(result.get("inconclusive_gates") or []),
        "relevant_samples": relevant_samples,
        "sample_artifact": str(SAMPLES_PATH.relative_to(PROJECT_ROOT)),
        "plain_summary_hu": result.get("plain_summary_hu"),
        "total_samples": len(samples),
    }


def build_summary(result: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "schema": result["schema"],
        "status": result["status"],
        "success": result["success"],
        "test_name": result["test_name"],
        "plain_summary_hu": result["plain_summary_hu"],
        "failed_gates": result["failed_gates"],
        "inconclusive_gates": result["inconclusive_gates"],
        "coverage": result["metrics"]["coverage"],
        "target": result["metrics"]["target"],
        "motion_tracking": result["metrics"]["motion_tracking"],
        "oscillation": result["metrics"]["oscillation"],
        "control_loop": result["metrics"]["control_loop"],
        "integrity": result["metrics"]["integrity"],
        "artifact_paths": result["artifact_paths"],
    }


def write_artifacts(result: Dict[str, Any], samples: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    summary = build_summary(result)
    _write_jsonl(SAMPLES_PATH, samples)
    _write_json(RESULT_PATH, result)
    _write_json(SUMMARY_PATH, summary)
    _write_json(INCIDENT_PATH, _incident_payload(result, samples))
    return summary


def run(args: argparse.Namespace) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    threshold_overrides = _load_thresholds(str(args.thresholds_json or ""))
    live.RESULT_PATH = BASE_RESULT_PATH
    live.SUMMARY_PATH = BASE_SUMMARY_PATH
    live.HISTORY_PATH = SAMPLES_PATH
    base_result = live.run(args)
    raw_samples = _read_jsonl(SAMPLES_PATH)
    result, samples = analyze_samples(raw_samples, threshold_overrides, base_result=base_result)
    result.update(
        {
            "test_name": str(args.test_name),
            "duration_s": float(args.duration_s),
            "sample_rate_hz": float(args.sample_rate_hz),
            "artifact_paths": {
                "result": str(RESULT_PATH.relative_to(PROJECT_ROOT)),
                "summary": str(SUMMARY_PATH.relative_to(PROJECT_ROOT)),
                "samples": str(SAMPLES_PATH.relative_to(PROJECT_ROOT)),
                "incident": str(INCIDENT_PATH.relative_to(PROJECT_ROOT)),
                "base_follow_result": str(BASE_RESULT_PATH.relative_to(PROJECT_ROOT)),
            },
        }
    )
    write_artifacts(result, samples)
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="M3 live Human Follow v2 movement-quality validation.")
    parser.add_argument("--test-name", default="M3_emberkovetes_mozgasminoseg")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--sample-rate-hz", type=float, default=10.0)
    parser.add_argument("--speed-scale", type=float, default=0.8)
    parser.add_argument("--follow-distance-m", type=float, default=1.0)
    parser.add_argument("--search-pivot-omega-rad-s", type=float, default=0.08)
    parser.add_argument("--control-mode", default="UNIFIED")
    parser.add_argument("--fresh-target-omega-max-rad-s", type=float, default=None)
    parser.add_argument("--omega-p90-max-rad-s", type=float, default=None)
    parser.add_argument("--command-delta-omega-p90-max-rad-s", type=float, default=None)
    parser.add_argument("--command-delta-omega-max-rad-s", type=float, default=None)
    parser.add_argument("--status-timeout-s", type=float, default=5.0)
    parser.add_argument("--token", default="GUI_DEFAULT")
    parser.add_argument("--thresholds-json", default="")
    parser.add_argument("--no-strict-v2", dest="strict_v2", action="store_false")
    parser.set_defaults(strict_v2=True)
    parser.add_argument("--compact", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    result = run(args)
    output = result if not args.compact else {
        "status": result["status"],
        "plain_summary_hu": result["plain_summary_hu"],
        "failed_gates": result["failed_gates"],
        "inconclusive_gates": result["inconclusive_gates"],
    }
    print(json.dumps(output, ensure_ascii=False))
    return 0 if result.get("status") == "PASS" else (2 if result.get("status") == "INCONCLUSIVE" else 1)


if __name__ == "__main__":
    raise SystemExit(main())
