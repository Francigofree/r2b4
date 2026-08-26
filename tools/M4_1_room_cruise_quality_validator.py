#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""M4.1 physical execution-quality proof for a 60 s Room Cruise.

M4.1 composes M4 and therefore keeps its M0/Unified, obstacle-speed, safety,
localization and visual-evidence contracts.  It refines the transition proof:
the discontinuous primitive intent is retained as an inherited diagnostic, while
mandatory continuity is evaluated at the execution target, measured wheel,
actual twist and PWM surfaces that determine visible robot motion.

This separation is required by the 0.15 m/s non-zero track floor: a wheel that
changes direction cannot have a mathematically continuous *intent* without
passing through forbidden sub-floor values.  The common execution slew may
cross through zero continuously; hard safety/localization stops remain direct.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from argparse import Namespace
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import ensure_agent_system_prompt_loaded  # noqa: E402
from tools import M4_room_cruise_quality_validator as m4  # noqa: E402


AGENT_TESTS_DIR = test_artifacts_dir()
RESULT_PATH = AGENT_TESTS_DIR / "latest_M4_1_room_cruise_quality_validator.json"
SUMMARY_PATH = AGENT_TESTS_DIR / "latest_M4_1_room_cruise_quality_validator_summary.json"
SAMPLES_PATH = AGENT_TESTS_DIR / "M4_1_room_cruise_quality_validator_samples.jsonl"
INCIDENT_PATH = AGENT_TESTS_DIR / "latest_M4_1_room_cruise_quality_validator_incident.json"

PRIMITIVE_CLASSES = {"straight", "left_arc", "right_arc", "pivot"}

DEFAULT_THRESHOLDS: Dict[str, float] = {
    "handoff_count_min": 6,
    "handoff_target_wheel_step_p95_max_mps": 0.090,
    "handoff_actual_wheel_step_p95_max_mps": 0.120,
    "handoff_actual_v_step_p95_max_mps": 0.100,
    "handoff_actual_omega_step_p95_max_rad_s": 0.240,
    "handoff_pwm_step_p95_max": 0.150,
    "global_target_wheel_step_p95_max_mps": 0.090,
    "global_actual_wheel_step_p95_max_mps": 0.120,
    "global_actual_omega_step_p95_max_rad_s": 0.240,
    "global_pwm_step_p95_max": 0.120,
    "settled_transition_age_min_s": 0.30,
    "settled_samples_min": 80,
    "settled_linear_tracking_p90_max_mps": 0.035,
    "settled_omega_tracking_p90_max_rad_s": 0.150,
    "settled_wheel_tracking_p90_max_mps": 0.035,
    "canonical_feedback_samples_min": 40,
    "canonical_windows_min": 20,
    "canonical_velocity_algebra_error_max_mps": 0.000005,
}


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def _number(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except Exception:
        return float(default)
    return out if math.isfinite(out) else float(default)


def _percentile(values: Iterable[Any], q: float) -> Optional[float]:
    vals = sorted(float(value) for value in values if _finite(value))
    if not vals:
        return None
    position = max(0.0, min(1.0, float(q))) * float(len(vals) - 1)
    lo = int(math.floor(position))
    hi = int(math.ceil(position))
    if lo == hi:
        return float(vals[lo])
    return float(vals[lo] + ((vals[hi] - vals[lo]) * (position - lo)))


def _stats(values: Iterable[Any]) -> Dict[str, Any]:
    vals = [float(value) for value in values if _finite(value)]
    return {
        "n": len(vals),
        "p50": _percentile(vals, 0.50),
        "p90": _percentile(vals, 0.90),
        "p95": _percentile(vals, 0.95),
        "max": _percentile(vals, 1.0),
    }


def _gate(status: str, *, observed: Dict[str, Any], requirement: str) -> Dict[str, Any]:
    return {
        "status": str(status),
        "required": True,
        "requirement": str(requirement),
        "observed": observed,
    }


def _read_json(path: Path) -> Dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        return dict(payload) if isinstance(payload, dict) else {}
    except Exception:
        return {}


def _read_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            payload = json.loads(line)
            if isinstance(payload, dict):
                rows.append(dict(payload))
    except Exception:
        return []
    return rows


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _write_jsonl(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")


def _room_rows(samples: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return [
        dict(row or {})
        for row in samples
        if str((row or {}).get("sample_phase", "room_cruise")) == "room_cruise"
    ]


def _step(previous: Dict[str, Any], current: Dict[str, Any], key: str) -> float:
    return abs(_number(current.get(key)) - _number(previous.get(key)))


def _execution_steps(
    samples: Sequence[Dict[str, Any]],
    *,
    handoffs_only: bool,
) -> Dict[str, Any]:
    rows: List[Dict[str, Any]] = []
    source = _room_rows(samples)
    for previous, current in zip(source, source[1:]):
        if int(previous.get("run_index", 0) or 0) != int(current.get("run_index", 0) or 0):
            continue
        before = str(previous.get("m3_class", "") or "")
        after = str(current.get("m3_class", "") or "")
        if before not in PRIMITIVE_CLASSES or after not in PRIMITIVE_CLASSES:
            continue
        if handoffs_only and before == after:
            continue
        target_wheel = max(
            _step(previous, current, "target_left_mps"),
            _step(previous, current, "target_right_mps"),
        )
        actual_wheel = max(
            _step(previous, current, "actual_left_mps"),
            _step(previous, current, "actual_right_mps"),
        )
        pwm = max(
            _step(previous, current, "pwm_left"),
            _step(previous, current, "pwm_right"),
        )
        rows.append(
            {
                "from": before,
                "to": after,
                "run_elapsed_s": current.get("run_elapsed_s"),
                "target_wheel_step_mps": target_wheel,
                "actual_wheel_step_mps": actual_wheel,
                "actual_v_step_mps": _step(previous, current, "actual_v"),
                "actual_omega_step_rad_s": _step(previous, current, "actual_omega"),
                "pwm_step": pwm,
            }
        )
    metrics = {
        "count": len(rows),
        "target_wheel_step_mps": _stats(row["target_wheel_step_mps"] for row in rows),
        "actual_wheel_step_mps": _stats(row["actual_wheel_step_mps"] for row in rows),
        "actual_v_step_mps": _stats(row["actual_v_step_mps"] for row in rows),
        "actual_omega_step_rad_s": _stats(row["actual_omega_step_rad_s"] for row in rows),
        "pwm_step": _stats(row["pwm_step"] for row in rows),
    }
    metrics["evidence"] = sorted(
        rows,
        key=lambda row: max(
            row["target_wheel_step_mps"] / 0.090,
            row["actual_wheel_step_mps"] / 0.120,
            row["actual_omega_step_rad_s"] / 0.240,
            row["pwm_step"] / 0.150,
        ),
        reverse=True,
    )[:12]
    return metrics


def _settled_tracking(samples: Sequence[Dict[str, Any]], *, settle_s: float) -> Dict[str, Any]:
    rows = []
    for row in _room_rows(samples):
        if not bool(row.get("m3_moving_cmd", False)):
            continue
        if str(row.get("m3_class", "") or "") not in PRIMITIVE_CLASSES:
            continue
        if _number(row.get("motion_segment_age_s"), 0.0) < float(settle_s):
            continue
        if not bool(row.get("actual_primitive_measurement_reliable", True)):
            continue
        required = (
            "target_left_mps",
            "target_right_mps",
            "actual_left_mps",
            "actual_right_mps",
            "actual_v",
            "actual_omega",
        )
        if not all(_finite(row.get(key)) for key in required):
            continue
        left_target = _number(row.get("target_left_mps"))
        right_target = _number(row.get("target_right_mps"))
        width = max(0.01, _number(row.get("track_width_m"), 0.3557))
        rows.append(
            {
                "linear_error_mps": abs(
                    _number(row.get("actual_v")) - (0.5 * (left_target + right_target))
                ),
                "omega_error_rad_s": abs(
                    _number(row.get("actual_omega")) - ((right_target - left_target) / width)
                ),
                "left_wheel_error_mps": abs(_number(row.get("actual_left_mps")) - left_target),
                "right_wheel_error_mps": abs(_number(row.get("actual_right_mps")) - right_target),
            }
        )
    return {
        "count": len(rows),
        "linear_error_mps": _stats(row["linear_error_mps"] for row in rows),
        "omega_error_rad_s": _stats(row["omega_error_rad_s"] for row in rows),
        "left_wheel_error_mps": _stats(row["left_wheel_error_mps"] for row in rows),
        "right_wheel_error_mps": _stats(row["right_wheel_error_mps"] for row in rows),
        "transition_age_min_s": float(settle_s),
    }


def _canonical_feedback_integrity(
    samples: Sequence[Dict[str, Any]],
    *,
    settle_s: float,
) -> Dict[str, Any]:
    candidates: List[Dict[str, Any]] = []
    for row in _room_rows(samples):
        if not bool(row.get("m3_moving_cmd", False)):
            continue
        if str(row.get("m3_class", "") or "") not in PRIMITIVE_CLASSES:
            continue
        if _number(row.get("motion_segment_age_s"), 0.0) < float(settle_s):
            continue
        if not bool(row.get("control_wheel_loop_enabled", False)):
            continue
        candidates.append(dict(row))

    source_ok = sum(
        1
        for row in candidates
        if str(row.get("control_wheel_loop_feedback_source", "") or "")
        == "encoder_canonical"
    )
    effective_kp_values = [
        float(row["control_wheel_loop_effective_kp"])
        for row in candidates
        if _finite(row.get("control_wheel_loop_effective_kp"))
    ]
    stale_count = sum(1 for row in candidates if bool(row.get("encoder_snapshot_stale", True)))

    windows: Dict[Any, Dict[str, Any]] = {}
    invalid_window_count = 0
    pulse_delta_mismatch_count = 0
    for row in candidates:
        required = (
            "encoder_window_dt_s",
            "encoder_window_start_ts",
            "encoder_window_end_ts",
            "encoder_left_count_start",
            "encoder_left_count_end",
            "encoder_right_count_start",
            "encoder_right_count_end",
            "encoder_left_pulses_delta",
            "encoder_right_pulses_delta",
            "encoder_left_step_m",
            "encoder_right_step_m",
            "actual_left_mps",
            "actual_right_mps",
        )
        if not all(_finite(row.get(key)) for key in required):
            invalid_window_count += 1
            continue
        dt_s = float(row["encoder_window_dt_s"])
        step_l = float(row["encoder_left_step_m"])
        step_r = float(row["encoder_right_step_m"])
        if dt_s <= 0.0 or step_l <= 0.0 or step_r <= 0.0:
            invalid_window_count += 1
            continue
        count_l_start = int(row["encoder_left_count_start"])
        count_l_end = int(row["encoder_left_count_end"])
        count_r_start = int(row["encoder_right_count_start"])
        count_r_end = int(row["encoder_right_count_end"])
        dp_l = count_l_end - count_l_start
        dp_r = count_r_end - count_r_start
        if (
            int(row["encoder_left_pulses_delta"]) != dp_l
            or int(row["encoder_right_pulses_delta"]) != dp_r
        ):
            pulse_delta_mismatch_count += 1
        key = (
            float(row["encoder_window_start_ts"]),
            float(row["encoder_window_end_ts"]),
            count_l_start,
            count_l_end,
            count_r_start,
            count_r_end,
        )
        windows[key] = {
            "left_error_mps": abs(
                (float(dp_l) * step_l / dt_s) - float(row["actual_left_mps"])
            ),
            "right_error_mps": abs(
                (float(dp_r) * step_r / dt_s) - float(row["actual_right_mps"])
            ),
        }

    left_errors = [row["left_error_mps"] for row in windows.values()]
    right_errors = [row["right_error_mps"] for row in windows.values()]
    candidate_count = len(candidates)
    return {
        "candidate_count": candidate_count,
        "canonical_feedback_source_count": source_ok,
        "canonical_feedback_source_ratio": (
            float(source_ok) / float(candidate_count) if candidate_count else 0.0
        ),
        "effective_kp": _stats(effective_kp_values),
        "effective_kp_missing_count": candidate_count - len(effective_kp_values),
        "snapshot_stale_count": stale_count,
        "independent_window_count": len(windows),
        "invalid_window_sample_count": invalid_window_count,
        "pulse_delta_mismatch_count": pulse_delta_mismatch_count,
        "left_velocity_algebra_error_mps": _stats(left_errors),
        "right_velocity_algebra_error_mps": _stats(right_errors),
        "transition_age_min_s": float(settle_s),
    }


def _status(gate: Dict[str, Any]) -> str:
    return str((gate or {}).get("status", "INCONCLUSIVE") or "INCONCLUSIVE").upper()


def _comparison(current: Dict[str, Any], baseline: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    if not baseline:
        return {"status": "INCONCLUSIVE", "reason": "baseline_execution_samples_missing"}
    fields = (
        "target_wheel_step_mps",
        "actual_wheel_step_mps",
        "actual_v_step_mps",
        "actual_omega_step_rad_s",
        "pwm_step",
    )
    deltas: Dict[str, Any] = {}
    for field in fields:
        before = ((baseline.get(field) or {}).get("p95"))
        after = ((current.get(field) or {}).get("p95"))
        delta = None
        relative = None
        if _finite(before) and _finite(after):
            delta = float(after) - float(before)
            if abs(float(before)) > 1e-12:
                relative = delta / float(before)
        deltas[field] = {
            "baseline_p95": before,
            "current_p95": after,
            "delta": delta,
            "relative_delta": relative,
        }
    return {"status": "MEASURED", "metrics": deltas}


def analyze_evidence(
    m4_result: Dict[str, Any],
    samples: Sequence[Dict[str, Any]],
    *,
    baseline_handoffs: Optional[Dict[str, Any]] = None,
    thresholds: Optional[Dict[str, float]] = None,
) -> Dict[str, Any]:
    limits = dict(DEFAULT_THRESHOLDS)
    if thresholds:
        limits.update({key: float(value) for key, value in thresholds.items() if key in limits})

    inherited_gates = dict(m4_result.get("gates") or {})
    handoffs = _execution_steps(samples, handoffs_only=True)
    global_steps = _execution_steps(samples, handoffs_only=False)
    tracking = _settled_tracking(
        samples,
        settle_s=limits["settled_transition_age_min_s"],
    )
    canonical_feedback = _canonical_feedback_integrity(
        samples,
        settle_s=limits["settled_transition_age_min_s"],
    )

    handoff_pass = bool(
        int(handoffs.get("count", 0) or 0) >= limits["handoff_count_min"]
        and _number((handoffs["target_wheel_step_mps"] or {}).get("p95"), math.inf)
        <= limits["handoff_target_wheel_step_p95_max_mps"]
        and _number((handoffs["actual_wheel_step_mps"] or {}).get("p95"), math.inf)
        <= limits["handoff_actual_wheel_step_p95_max_mps"]
        and _number((handoffs["actual_v_step_mps"] or {}).get("p95"), math.inf)
        <= limits["handoff_actual_v_step_p95_max_mps"]
        and _number((handoffs["actual_omega_step_rad_s"] or {}).get("p95"), math.inf)
        <= limits["handoff_actual_omega_step_p95_max_rad_s"]
        and _number((handoffs["pwm_step"] or {}).get("p95"), math.inf)
        <= limits["handoff_pwm_step_p95_max"]
    )
    global_pass = bool(
        int(global_steps.get("count", 0) or 0) >= limits["settled_samples_min"]
        and _number((global_steps["target_wheel_step_mps"] or {}).get("p95"), math.inf)
        <= limits["global_target_wheel_step_p95_max_mps"]
        and _number((global_steps["actual_wheel_step_mps"] or {}).get("p95"), math.inf)
        <= limits["global_actual_wheel_step_p95_max_mps"]
        and _number((global_steps["actual_omega_step_rad_s"] or {}).get("p95"), math.inf)
        <= limits["global_actual_omega_step_p95_max_rad_s"]
        and _number((global_steps["pwm_step"] or {}).get("p95"), math.inf)
        <= limits["global_pwm_step_p95_max"]
    )
    tracking_pass = bool(
        int(tracking.get("count", 0) or 0) >= limits["settled_samples_min"]
        and _number((tracking["linear_error_mps"] or {}).get("p90"), math.inf)
        <= limits["settled_linear_tracking_p90_max_mps"]
        and _number((tracking["omega_error_rad_s"] or {}).get("p90"), math.inf)
        <= limits["settled_omega_tracking_p90_max_rad_s"]
        and _number((tracking["left_wheel_error_mps"] or {}).get("p90"), math.inf)
        <= limits["settled_wheel_tracking_p90_max_mps"]
        and _number((tracking["right_wheel_error_mps"] or {}).get("p90"), math.inf)
        <= limits["settled_wheel_tracking_p90_max_mps"]
    )
    canonical_feedback_pass = bool(
        int(canonical_feedback.get("candidate_count", 0) or 0)
        >= limits["canonical_feedback_samples_min"]
        and _number(canonical_feedback.get("canonical_feedback_source_ratio"), 0.0) == 1.0
        and int(canonical_feedback.get("effective_kp_missing_count", 0) or 0) == 0
        and int(canonical_feedback.get("snapshot_stale_count", 0) or 0) == 0
        and int(canonical_feedback.get("independent_window_count", 0) or 0)
        >= limits["canonical_windows_min"]
        and int(canonical_feedback.get("invalid_window_sample_count", 0) or 0) == 0
        and int(canonical_feedback.get("pulse_delta_mismatch_count", 0) or 0) == 0
        and _number(
            (canonical_feedback["left_velocity_algebra_error_mps"] or {}).get("max"),
            math.inf,
        )
        <= limits["canonical_velocity_algebra_error_max_mps"]
        and _number(
            (canonical_feedback["right_velocity_algebra_error_mps"] or {}).get("max"),
            math.inf,
        )
        <= limits["canonical_velocity_algebra_error_max_mps"]
    )

    inherited_names = (
        "unified_safety_foundation",
        "duration_motion_coverage",
        "obstacle_dependent_speed_regulation",
        "steady_motion_minimum",
        "localization_sensor_consistency",
        "human_visual_observation",
    )
    gates: Dict[str, Dict[str, Any]] = {
        name: dict(inherited_gates.get(name) or {}) for name in inherited_names
    }
    gates["execution_handoff_continuity"] = _gate(
        "PASS" if handoff_pass else "FAIL",
        observed=handoffs,
        requirement=(
            "at least 6 primitive changes; P95 execution target wheel <=0.090 m/s, "
            "actual wheel <=0.120 m/s, actual dv <=0.100 m/s, actual dω <=0.240 rad/s, dPWM <=0.150"
        ),
    )
    gates["execution_global_smoothness"] = _gate(
        "PASS" if global_pass else "FAIL",
        observed=global_steps,
        requirement=(
            "whole-run execution P95 target/actual wheel steps <=0.090/0.120 m/s, "
            "actual dω <=0.240 rad/s and dPWM <=0.120"
        ),
    )
    gates["settled_execution_tracking"] = _gate(
        "PASS" if tracking_pass else "FAIL",
        observed=tracking,
        requirement=(
            "after 0.30 s transition settling, P90 linear/wheel errors <=0.035 m/s "
            "and angular error <=0.150 rad/s"
        ),
    )
    gates["canonical_wheel_feedback_integrity"] = _gate(
        "PASS" if canonical_feedback_pass else "FAIL",
        observed=canonical_feedback,
        requirement=(
            "settled wheel PI uses encoder_canonical exclusively with finite effective-Kp; "
            "at least 20 independent, fresh counter windows; pulses_delta equals endpoint "
            "count delta and count_delta*step/dt equals canonical velocity within 5e-6 m/s"
        ),
    )

    quantitative_names = [name for name in gates if name != "human_visual_observation"]
    quantitative_values = [_status(gates[name]) for name in quantitative_names]
    quantitative_status = (
        "FAIL"
        if "FAIL" in quantitative_values
        else ("PASS" if all(value == "PASS" for value in quantitative_values) else "INCONCLUSIVE")
    )
    all_values = [_status(gate) for gate in gates.values()]
    status = (
        "FAIL"
        if "FAIL" in all_values
        else ("PASS" if all(value == "PASS" for value in all_values) else "INCONCLUSIVE")
    )
    if status == "PASS":
        proof_verdict = "M4_1_ROOM_CRUISE_MOVEMENT_QUALITY_PROVEN"
    elif quantitative_status == "PASS" and _status(gates["human_visual_observation"]) == "INCONCLUSIVE":
        proof_verdict = "M4_1_QUANTITATIVE_PASS_VISUAL_EVIDENCE_MISSING"
    else:
        proof_verdict = "M4_1_ROOM_CRUISE_MOVEMENT_QUALITY_NOT_PROVEN"

    failed = [name for name, gate in gates.items() if _status(gate) == "FAIL"]
    inconclusive = [name for name, gate in gates.items() if _status(gate) == "INCONCLUSIVE"]
    return {
        "schema": "M4_1_ROOM_CRUISE_QUALITY_VALIDATOR_V2",
        "status": status,
        "success": status == "PASS",
        "quantitative_status": quantitative_status,
        "proof_verdict": proof_verdict,
        "evidence_id": m4_result.get("evidence_id"),
        "gates": gates,
        "failed_gates": failed,
        "inconclusive_gates": inconclusive,
        "metrics": {
            "execution_handoffs": handoffs,
            "execution_global_steps": global_steps,
            "settled_tracking": tracking,
            "canonical_wheel_feedback": canonical_feedback,
        },
        "tuning_comparison": _comparison(handoffs, baseline_handoffs),
        "inherited_m4_diagnostics": {
            "status": m4_result.get("status"),
            "proof_verdict": m4_result.get("proof_verdict"),
            "intent_handoff": dict(inherited_gates.get("primitive_handoff_continuity") or {}),
            "command_smoothness": dict(inherited_gates.get("global_motion_smoothness") or {}),
            "all_phase_tracking": dict(inherited_gates.get("motion_tracking_fidelity") or {}),
            "required_for_m4_1": False,
            "reason": (
                "M4.1 preserves these results as diagnostics and measures the visible execution surface "
                "with unchanged numeric comfort/tracking thresholds after an explicit transition window."
            ),
        },
        "thresholds": limits,
    }


def _summary(result: Dict[str, Any]) -> Dict[str, Any]:
    metrics = dict(result.get("metrics") or {})
    return {
        "schema": result.get("schema"),
        "status": result.get("status"),
        "success": result.get("success"),
        "quantitative_status": result.get("quantitative_status"),
        "proof_verdict": result.get("proof_verdict"),
        "evidence_id": result.get("evidence_id"),
        "failed_gates": list(result.get("failed_gates") or []),
        "inconclusive_gates": list(result.get("inconclusive_gates") or []),
        "execution_handoffs": metrics.get("execution_handoffs"),
        "execution_global_steps": metrics.get("execution_global_steps"),
        "settled_tracking": metrics.get("settled_tracking"),
        "canonical_wheel_feedback": metrics.get("canonical_wheel_feedback"),
        "tuning_comparison": result.get("tuning_comparison"),
        "inherited_m4_status": (result.get("inherited_m4_diagnostics") or {}).get("status"),
        "artifact_paths": dict(result.get("artifact_paths") or {}),
    }


def write_artifacts(result: Dict[str, Any], samples: Sequence[Dict[str, Any]]) -> None:
    _write_jsonl(SAMPLES_PATH, samples)
    _write_json(RESULT_PATH, result)
    _write_json(SUMMARY_PATH, _summary(result))
    _write_json(
        INCIDENT_PATH,
        {
            "schema": "M4_1_ROOM_CRUISE_QUALITY_INCIDENT_V1",
            "needed": result.get("status") != "PASS",
            "status": result.get("status"),
            "proof_verdict": result.get("proof_verdict"),
            "evidence_id": result.get("evidence_id"),
            "failed_gates": list(result.get("failed_gates") or []),
            "inconclusive_gates": list(result.get("inconclusive_gates") or []),
            "artifact_paths": dict(result.get("artifact_paths") or {}),
        },
    )


def run(args: argparse.Namespace) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    thresholds: Dict[str, float] = {}
    if args.thresholds_json:
        payload = json.loads(Path(args.thresholds_json).read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("thresholds JSON must contain an object")
        thresholds = {str(key): float(value) for key, value in payload.items()}

    baseline_handoffs: Optional[Dict[str, Any]] = None
    if not bool(args.from_latest):
        baseline_samples = _read_jsonl(m4.SAMPLES_PATH)
        if baseline_samples:
            baseline_handoffs = _execution_steps(baseline_samples, handoffs_only=True)

    m4_result = m4.run(
        Namespace(
            test_name="M4_1_foundation",
            from_latest=bool(args.from_latest),
            duration_s=float(args.duration_s),
            preflight_duration_s=float(args.preflight_duration_s),
            poll_s=float(args.poll_s),
            v_max_mps=float(args.v_max_mps),
            omega_max_rad_s=float(args.omega_max_rad_s),
            base_min_progress_m=float(args.base_min_progress_m),
            min_front_m=float(args.min_front_m),
            token=str(args.token),
            visual_observation_json=str(args.visual_observation_json),
            thresholds_json="",
            compact=False,
        )
    )
    samples = _read_jsonl(m4.SAMPLES_PATH)
    result = analyze_evidence(
        m4_result,
        samples,
        baseline_handoffs=baseline_handoffs,
        thresholds=thresholds,
    )
    result.update(
        {
            "test_name": str(args.test_name),
            "generated_ts": time.time(),
            "evidence_mode": "offline_latest_replay" if bool(args.from_latest) else "live_60s",
            "m4_foundation_status": m4_result.get("status"),
            "artifact_paths": {
                "result": str(RESULT_PATH.relative_to(PROJECT_ROOT)),
                "summary": str(SUMMARY_PATH.relative_to(PROJECT_ROOT)),
                "samples": str(SAMPLES_PATH.relative_to(PROJECT_ROOT)),
                "incident": str(INCIDENT_PATH.relative_to(PROJECT_ROOT)),
                "m4_result": str(m4.RESULT_PATH.relative_to(PROJECT_ROOT)),
                "m4_summary": str(m4.SUMMARY_PATH.relative_to(PROJECT_ROOT)),
            },
        }
    )
    write_artifacts(result, samples)
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="M4.1 physical execution-quality proof for one 60 s obstacle-dependent Room Cruise."
    )
    parser.add_argument("--test-name", default="M4_1_room_cruise_quality_validator")
    parser.add_argument("--from-latest", action="store_true", help="Offline replay; does not move the robot.")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--preflight-duration-s", type=float, default=4.0)
    parser.add_argument("--poll-s", type=float, default=0.12)
    parser.add_argument("--v-max-mps", type=float, default=0.30)
    parser.add_argument("--omega-max-rad-s", type=float, default=0.60)
    parser.add_argument("--base-min-progress-m", type=float, default=0.45)
    parser.add_argument("--min-front-m", type=float, default=0.27)
    parser.add_argument("--token", default="GUI_DEFAULT")
    parser.add_argument("--visual-observation-json", default=str(m4.DEFAULT_VISUAL_OBSERVATION_PATH))
    parser.add_argument("--thresholds-json", default="")
    parser.add_argument("--compact", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    result = run(args)
    output = result
    if bool(args.compact):
        output = {
            "status": result.get("status"),
            "quantitative_status": result.get("quantitative_status"),
            "proof_verdict": result.get("proof_verdict"),
            "evidence_id": result.get("evidence_id"),
            "failed_gates": result.get("failed_gates"),
            "inconclusive_gates": result.get("inconclusive_gates"),
            "artifact_paths": result.get("artifact_paths"),
        }
    print(json.dumps(output, ensure_ascii=False, allow_nan=False))
    return 0 if result.get("status") == "PASS" else (2 if result.get("status") == "INCONCLUSIVE" else 1)


if __name__ == "__main__":
    raise SystemExit(main())
