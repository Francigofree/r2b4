#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""M4 60 s Room Cruise movement-quality proof validator.

M4 composes the existing M0 + Unified live runner with independent proof gates
for obstacle-dependent speed regulation and primitive-handoff continuity.  It
does not command motors directly and it does not replace any lower-level gate.

The human-perception claim is intentionally separate from telemetry: a full
M4 PASS requires a structured on-site observation.  Without it the best
possible verdict is ``QUANTITATIVE_PASS_VISUAL_EVIDENCE_MISSING``.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from argparse import Namespace
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import resolve_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import ensure_agent_system_prompt_loaded  # noqa: E402
from tools import M3_room_cruise_unified_validator as m3_unified  # noqa: E402


AGENT_TESTS_DIR = test_artifacts_dir()
RESULT_PATH = AGENT_TESTS_DIR / "latest_M4_room_cruise_quality_validator.json"
SUMMARY_PATH = AGENT_TESTS_DIR / "latest_M4_room_cruise_quality_validator_summary.json"
SAMPLES_PATH = AGENT_TESTS_DIR / "M4_room_cruise_quality_validator_samples.jsonl"
INCIDENT_PATH = AGENT_TESTS_DIR / "latest_M4_room_cruise_quality_validator_incident.json"
DEFAULT_VISUAL_OBSERVATION_PATH = AGENT_TESTS_DIR / "M4_human_visual_observation.json"

DEFAULT_THRESHOLDS: Dict[str, float] = {
    "duration_min_s": 58.0,
    "duration_max_s": 66.0,
    "progress_min_m": 0.45,
    "moving_samples_min": 160,
    "straight_samples_min": 18,
    "left_arc_samples_min": 12,
    "right_arc_samples_min": 12,
    "pivot_samples_min": 8,
    "avoidance_samples_min": 8,
    "open_front_min_m": 1.25,
    "near_front_min_m": 0.65,
    "near_front_max_m": 1.05,
    "speed_band_samples_min": 20,
    "open_speed_p50_min_mps": 0.22,
    "near_open_speed_delta_min_mps": 0.04,
    "clearance_monotonic_tolerance_mps": 0.03,
    "steady_speed_min_mps": 0.145,
    "steady_samples_min": 20,
    "transition_settle_s": 0.30,
    "handoff_count_min": 6,
    "handoff_v_step_p95_max_mps": 0.055,
    "handoff_omega_step_p95_max_rad_s": 0.24,
    "handoff_pwm_step_p95_max": 0.15,
    "global_v_step_p95_max_mps": 0.055,
    "global_omega_step_p95_max_rad_s": 0.24,
    "global_pwm_step_p95_max": 0.12,
    "linear_tracking_p90_max_mps": 0.035,
    "omega_tracking_p90_max_rad_s": 0.15,
    "wheel_tracking_p90_max_mps": 0.035,
}

PRIMITIVE_CLASSES = {"straight", "left_arc", "right_arc", "pivot"}


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
    pos = max(0.0, min(1.0, float(q))) * float(len(vals) - 1)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return float(vals[lo])
    return float(vals[lo] + ((vals[hi] - vals[lo]) * (pos - lo)))


def _stats(values: Iterable[Any]) -> Dict[str, Any]:
    vals = [float(value) for value in values if _finite(value)]
    return {
        "n": len(vals),
        "min": _percentile(vals, 0.0),
        "p10": _percentile(vals, 0.10),
        "p50": _percentile(vals, 0.50),
        "p90": _percentile(vals, 0.90),
        "p95": _percentile(vals, 0.95),
        "max": _percentile(vals, 1.0),
    }


def _gate(status: str, *, observed: Dict[str, Any], requirement: str, evidence: Optional[List[Dict[str, Any]]] = None) -> Dict[str, Any]:
    return {
        "status": str(status),
        "required": True,
        "requirement": str(requirement),
        "observed": observed,
        "evidence": list(evidence or []),
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
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _write_jsonl(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")


def _m3_gate(m3_result: Dict[str, Any], gate_name: str) -> str:
    gate = dict((((m3_result.get("m3_room_cruise") or {}).get("gates") or {}).get(gate_name) or {}))
    return str(gate.get("status", "INCONCLUSIVE") or "INCONCLUSIVE").upper()


def _runtime_gate(m3_result: Dict[str, Any], gate_name: str) -> str:
    gate = dict((((m3_result.get("runtime_validation") or {}).get("gates") or {}).get(gate_name) or {}))
    return str(gate.get("status", "INCONCLUSIVE") or "INCONCLUSIVE").upper()


def _primitive_handoffs(samples: Sequence[Dict[str, Any]]) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    rows: List[Dict[str, Any]] = []
    for previous, current in zip(samples, samples[1:]):
        if int(previous.get("run_index", 0) or 0) != int(current.get("run_index", 0) or 0):
            continue
        before = str(previous.get("m3_class", "") or "")
        after = str(current.get("m3_class", "") or "")
        if before not in PRIMITIVE_CLASSES or after not in PRIMITIVE_CLASSES or before == after:
            continue
        rows.append(
            {
                "from": before,
                "to": after,
                "run_elapsed_s": current.get("run_elapsed_s"),
                "v_step_mps": abs(_number(current.get("resolved_v")) - _number(previous.get("resolved_v"))),
                "omega_step_rad_s": abs(_number(current.get("resolved_omega")) - _number(previous.get("resolved_omega"))),
                "pwm_step": max(
                    abs(_number(current.get("pwm_left")) - _number(previous.get("pwm_left"))),
                    abs(_number(current.get("pwm_right")) - _number(previous.get("pwm_right"))),
                ),
            }
        )
    metrics = {
        "count": len(rows),
        "v_step_mps": _stats(row["v_step_mps"] for row in rows),
        "omega_step_rad_s": _stats(row["omega_step_rad_s"] for row in rows),
        "pwm_step": _stats(row["pwm_step"] for row in rows),
    }
    evidence = sorted(
        rows,
        key=lambda row: max(row["v_step_mps"] / 0.055, row["omega_step_rad_s"] / 0.24, row["pwm_step"] / 0.15),
        reverse=True,
    )[:12]
    return metrics, evidence


def _observer_gate(observer: Optional[Dict[str, Any]], *, evidence_id: str) -> Dict[str, Any]:
    if not observer:
        return _gate(
            "INCONCLUSIVE",
            observed={"provided": False},
            requirement=(
                "structured observation linked to this evidence_id: observed_full_run=true, "
                "primitive_transition_noticeability_max_5<=1, abrupt_motion_events=0"
            ),
        )
    full_run = bool(observer.get("observed_full_run", False))
    observer_id = str(observer.get("observer_id", "") or "").strip()
    observed_evidence_id = str(observer.get("evidence_id", "") or "").strip()
    noticeability_raw = observer.get("primitive_transition_noticeability_max_5")
    abrupt_events_raw = observer.get("abrupt_motion_events")
    noticeability = _number(noticeability_raw, math.inf)
    abrupt_events = int(_number(abrupt_events_raw, math.inf)) if _finite(abrupt_events_raw) else None
    status = (
        "PASS"
        if (
            full_run
            and observer_id
            and observed_evidence_id == str(evidence_id)
            and noticeability <= 1.0
            and abrupt_events == 0
        )
        else "FAIL"
    )
    return _gate(
        status,
        observed={
            "provided": True,
            "expected_evidence_id": str(evidence_id),
            "observed_evidence_id": observed_evidence_id,
            "observer_id": observer_id,
            "observed_full_run": full_run,
            "primitive_transition_noticeability_max_5": noticeability if math.isfinite(noticeability) else None,
            "abrupt_motion_events": abrupt_events,
            "video_reference": str(observer.get("video_reference", "") or ""),
        },
        requirement=(
            "observation belongs to this evidence ID; full 60 s observed; transition noticeability <=1/5; no abrupt event"
        ),
    )


def analyze_evidence(
    m3_result: Dict[str, Any],
    samples: Sequence[Dict[str, Any]],
    *,
    observer: Optional[Dict[str, Any]] = None,
    thresholds: Optional[Dict[str, float]] = None,
) -> Dict[str, Any]:
    limits = dict(DEFAULT_THRESHOLDS)
    if thresholds:
        limits.update({key: float(value) for key, value in thresholds.items() if key in limits})
    rows = [dict(row or {}) for row in samples if str((row or {}).get("sample_phase", "room_cruise")) == "room_cruise"]
    foundation_generated_ts = _number(m3_result.get("generated_ts"), 0.0)
    evidence_id = f"m3-foundation:{foundation_generated_ts:.6f}"
    m3_metrics = dict((m3_result.get("m3_room_cruise") or {}).get("metrics") or {})
    coverage = dict(m3_metrics.get("coverage") or {})
    behavior = dict(m3_metrics.get("behavior") or {})
    safety = dict(m3_metrics.get("safety") or {})
    tracking = dict(m3_metrics.get("motion_tracking") or {})
    smoothness = dict(m3_metrics.get("smoothness") or {})
    wheel_direction = dict(m3_metrics.get("wheel_direction") or {})
    run_summaries = list(coverage.get("run_summaries") or [])
    duration_s = _number((run_summaries[0] if run_summaries else {}).get("duration_s"), 0.0)
    progress_m = _number((behavior.get("progress_per_run_m") or [0.0])[0], 0.0)

    translational = [
        row for row in rows
        if bool(row.get("m3_moving_cmd", False))
        and str(row.get("m3_class", "") or "") != "pivot"
        and abs(_number(row.get("resolved_v"))) >= 0.05
        and _finite(row.get("front_m"))
    ]
    open_speeds = [
        abs(_number(row.get("resolved_v"))) for row in translational
        if _number(row.get("front_m")) >= limits["open_front_min_m"]
    ]
    near_speeds = [
        abs(_number(row.get("resolved_v"))) for row in translational
        if limits["near_front_min_m"] <= _number(row.get("front_m")) <= limits["near_front_max_m"]
    ]
    open_stats = _stats(open_speeds)
    near_stats = _stats(near_speeds)
    speed_delta = (
        float(open_stats["p50"]) - float(near_stats["p50"])
        if open_stats.get("p50") is not None and near_stats.get("p50") is not None
        else None
    )
    clearance_bands = []
    for label, low, high in (
        ("near_low", limits["near_front_min_m"], 0.85),
        ("near_high", 0.85, limits["near_front_max_m"]),
        ("open", limits["open_front_min_m"], math.inf),
    ):
        values = [
            abs(_number(row.get("resolved_v"))) for row in translational
            if low <= _number(row.get("front_m")) < high
        ]
        clearance_bands.append(
            {
                "name": label,
                "front_min_m": low,
                "front_max_m": high if math.isfinite(high) else None,
                "speed": _stats(values),
            }
        )
    band_medians = [band["speed"]["p50"] for band in clearance_bands if band["speed"]["n"] >= limits["speed_band_samples_min"]]
    monotonic = all(
        float(later) + limits["clearance_monotonic_tolerance_mps"] >= float(earlier)
        for earlier, later in zip(band_medians, band_medians[1:])
    )

    steady_speeds = [
        abs(_number(row.get("resolved_v"))) for row in rows
        if bool(row.get("m3_moving_cmd", False))
        and str(row.get("m3_class", "") or "") != "pivot"
        and abs(_number(row.get("resolved_v"))) > 1e-6
        and _number(row.get("motion_segment_age_s")) >= limits["transition_settle_s"]
    ]
    steady_stats = _stats(steady_speeds)
    handoff_metrics, handoff_evidence = _primitive_handoffs(rows)

    runtime_foundation_names = (
        "base_room_cruise_v2",
        "ssot_contract",
        "safety_runtime",
        "control_loop_timing",
        "software_performance",
        "peripheral_runtime_health",
    )
    foundation_statuses = {name: _runtime_gate(m3_result, name) for name in runtime_foundation_names}
    preflight_status = str((m3_result.get("preflight") or {}).get("status", "INCONCLUSIVE") or "INCONCLUSIVE")
    foundation_values = [preflight_status.upper(), *foundation_statuses.values()]
    foundation_status = "FAIL" if "FAIL" in foundation_values else ("PASS" if all(v == "PASS" for v in foundation_values) else "INCONCLUSIVE")

    wheel_observed = {
        name: {
            "sample_count": int((data or {}).get("sample_count", 0) or 0),
            "error_abs_p90_mps": (data or {}).get("error_abs_p90_mps"),
            "wrong_sign_ratio": (data or {}).get("wrong_sign_ratio"),
        }
        for name, data in wheel_direction.items()
    }
    wheel_fail = any(
        int(data.get("sample_count", 0) or 0) >= 8
        and (
            _number(data.get("error_abs_p90_mps"), math.inf) > limits["wheel_tracking_p90_max_mps"]
            or _number(data.get("wrong_sign_ratio"), math.inf) > 0.0
        )
        for data in wheel_observed.values()
    )

    gates: Dict[str, Dict[str, Any]] = {
        "unified_safety_foundation": _gate(
            foundation_status,
            observed={"preflight": preflight_status, "runtime_gates": foundation_statuses, "safety": safety},
            requirement="M0, base Room Cruise, UNIFIED SSOT, safety, timing, software and peripheral gates all PASS",
        ),
        "duration_motion_coverage": _gate(
            "PASS" if (
                limits["duration_min_s"] <= duration_s <= limits["duration_max_s"]
                and progress_m >= limits["progress_min_m"]
                and int(coverage.get("moving_samples", 0) or 0) >= limits["moving_samples_min"]
                and int(coverage.get("straight_samples", 0) or 0) >= limits["straight_samples_min"]
                and int(coverage.get("left_arc_samples", 0) or 0) >= limits["left_arc_samples_min"]
                and int(coverage.get("right_arc_samples", 0) or 0) >= limits["right_arc_samples_min"]
                and int(coverage.get("pivot_samples", 0) or 0) >= limits["pivot_samples_min"]
                and int(coverage.get("obstacle_avoidance_samples", 0) or 0) >= limits["avoidance_samples_min"]
            ) else "FAIL",
            observed={"duration_s": duration_s, "progress_m": progress_m, "coverage": coverage},
            requirement="58-66 s run with progress and straight/left-arc/right-arc/pivot/avoidance coverage",
        ),
        "obstacle_dependent_speed_regulation": _gate(
            "PASS" if (
                open_stats["n"] >= limits["speed_band_samples_min"]
                and near_stats["n"] >= limits["speed_band_samples_min"]
                and _number(open_stats.get("p50")) >= limits["open_speed_p50_min_mps"]
                and speed_delta is not None
                and float(speed_delta) >= limits["near_open_speed_delta_min_mps"]
                and monotonic
            ) else "FAIL",
            observed={
                "open_speed": open_stats,
                "near_speed": near_stats,
                "open_minus_near_p50_mps": speed_delta,
                "clearance_bands": clearance_bands,
                "monotonic_with_tolerance": monotonic,
            },
            requirement="open-space speed >=0.22 m/s and at least 0.04 m/s above obstacle-near median; clearance bands monotonic",
        ),
        "steady_motion_minimum": _gate(
            "PASS" if (
                steady_stats["n"] >= limits["steady_samples_min"]
                and steady_stats.get("min") is not None
                and float(steady_stats["min"]) >= limits["steady_speed_min_mps"]
            ) else "FAIL",
            observed=steady_stats,
            requirement="settled non-pivot forward/reverse command is zero or at least 0.145 m/s (0.15 m/s nominal floor)",
        ),
        "primitive_handoff_continuity": _gate(
            "PASS" if (
                handoff_metrics["count"] >= limits["handoff_count_min"]
                and _number(handoff_metrics["v_step_mps"].get("p95"), math.inf) <= limits["handoff_v_step_p95_max_mps"]
                and _number(handoff_metrics["omega_step_rad_s"].get("p95"), math.inf) <= limits["handoff_omega_step_p95_max_rad_s"]
                and _number(handoff_metrics["pwm_step"].get("p95"), math.inf) <= limits["handoff_pwm_step_p95_max"]
            ) else "FAIL",
            observed=handoff_metrics,
            evidence=handoff_evidence,
            requirement="primitive-change P95: dv<=0.055 m/s, dω<=0.24 rad/s, dPWM<=0.15",
        ),
        "global_motion_smoothness": _gate(
            "PASS" if (
                _number(smoothness.get("velocity_step_p95_mps"), math.inf) <= limits["global_v_step_p95_max_mps"]
                and _number(smoothness.get("omega_step_p95_rad_s"), math.inf) <= limits["global_omega_step_p95_max_rad_s"]
                and _number(smoothness.get("pwm_step_p95"), math.inf) <= limits["global_pwm_step_p95_max"]
            ) else "FAIL",
            observed=smoothness,
            requirement="whole-run P95 command and motor-output steps stay below comfort limits",
        ),
        "motion_tracking_fidelity": _gate(
            "PASS" if (
                _number(tracking.get("linear_abs_error_p90_mps"), math.inf) <= limits["linear_tracking_p90_max_mps"]
                and _number(tracking.get("omega_abs_error_p90_rad_s"), math.inf) <= limits["omega_tracking_p90_max_rad_s"]
                and not wheel_fail
            ) else "FAIL",
            observed={"motion_tracking": tracking, "wheel_direction": wheel_observed},
            requirement="linear/angular command tracking and each sufficiently sampled wheel direction meet existing M3 limits",
        ),
        "localization_sensor_consistency": _gate(
            _m3_gate(m3_result, "localization_sensor_consistency"),
            observed=dict(m3_metrics.get("localization") or {}),
            requirement="same-snapshot yaw rate and endpoint sensor/EKF consistency gate PASS",
        ),
        "human_visual_observation": _observer_gate(observer, evidence_id=evidence_id),
    }
    quantitative_names = [name for name in gates if name != "human_visual_observation"]
    quantitative_statuses = [str(gates[name]["status"]) for name in quantitative_names]
    quantitative_status = "FAIL" if "FAIL" in quantitative_statuses else ("PASS" if all(v == "PASS" for v in quantitative_statuses) else "INCONCLUSIVE")
    all_statuses = [str(gate["status"]) for gate in gates.values()]
    status = "FAIL" if "FAIL" in all_statuses else ("PASS" if all(v == "PASS" for v in all_statuses) else "INCONCLUSIVE")
    if status == "PASS":
        proof_verdict = "M4_ROOM_CRUISE_QUALITY_PROVEN"
    elif quantitative_status == "PASS" and gates["human_visual_observation"]["status"] == "INCONCLUSIVE":
        proof_verdict = "QUANTITATIVE_PASS_VISUAL_EVIDENCE_MISSING"
    else:
        proof_verdict = "M4_ROOM_CRUISE_QUALITY_NOT_PROVEN"
    failed = [name for name, gate in gates.items() if gate["status"] == "FAIL"]
    inconclusive = [name for name, gate in gates.items() if gate["status"] == "INCONCLUSIVE"]
    return {
        "schema": "M4_ROOM_CRUISE_QUALITY_VALIDATOR_V1",
        "status": status,
        "success": status == "PASS",
        "quantitative_status": quantitative_status,
        "proof_verdict": proof_verdict,
        "evidence_id": evidence_id,
        "gates": gates,
        "failed_gates": failed,
        "inconclusive_gates": inconclusive,
        "metrics": {
            "duration_s": duration_s,
            "progress_m": progress_m,
            "obstacle_speed_regulation": gates["obstacle_dependent_speed_regulation"]["observed"],
            "primitive_handoffs": handoff_metrics,
            "smoothness": smoothness,
            "tracking": tracking,
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
        "duration_s": metrics.get("duration_s"),
        "progress_m": metrics.get("progress_m"),
        "obstacle_speed_regulation": metrics.get("obstacle_speed_regulation"),
        "primitive_handoffs": metrics.get("primitive_handoffs"),
        "artifact_paths": dict(result.get("artifact_paths") or {}),
    }


def write_artifacts(result: Dict[str, Any], samples: Sequence[Dict[str, Any]]) -> None:
    _write_jsonl(SAMPLES_PATH, samples)
    _write_json(RESULT_PATH, result)
    _write_json(SUMMARY_PATH, _summary(result))
    _write_json(
        INCIDENT_PATH,
        {
            "schema": "M4_ROOM_CRUISE_QUALITY_INCIDENT_V1",
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

    if bool(args.from_latest):
        m3_result_path = resolve_artifact_path("logs/latest/latest_M3_room_cruise_unified_validator.json")
        m3_summary_path = resolve_artifact_path("logs/latest/latest_M3_room_cruise_unified_validator_summary.json")
        m3_samples_path = resolve_artifact_path("logs/latest/M3_room_cruise_unified_validator_samples.jsonl")
        m3_result = _read_json(m3_result_path)
    else:
        m3_result = m3_unified.run(
            Namespace(
                test_name="M4_room_cruise_quality_foundation",
                preflight_only=False,
                preflight_duration_s=float(args.preflight_duration_s),
                preflight_poll_s=0.15,
                duration_s=float(args.duration_s),
                poll_s=float(args.poll_s),
                v_max_mps=float(args.v_max_mps),
                omega_max_rad_s=float(args.omega_max_rad_s),
                base_min_progress_m=float(args.base_min_progress_m),
                min_front_m=float(args.min_front_m),
                token=str(args.token),
                disable_camera=True,
                camera_settle_s=0.4,
                run_on_preflight_fail=False,
                thresholds_json="",
                compact=False,
            )
        )
        m3_result_path = m3_unified.RESULT_PATH
        m3_summary_path = m3_unified.SUMMARY_PATH
        m3_samples_path = m3_unified.SAMPLES_PATH
    samples = _read_jsonl(m3_samples_path)
    observer = _read_json(Path(args.visual_observation_json)) if args.visual_observation_json else None
    result = analyze_evidence(m3_result, samples, observer=observer, thresholds=thresholds)
    result.update(
        {
            "test_name": str(args.test_name),
            "generated_ts": time.time(),
            "evidence_mode": "offline_latest_replay" if bool(args.from_latest) else "live_60s",
            "m3_foundation_status": m3_result.get("status"),
            "artifact_paths": {
                "result": str(RESULT_PATH.relative_to(PROJECT_ROOT)),
                "summary": str(SUMMARY_PATH.relative_to(PROJECT_ROOT)),
                "samples": str(SAMPLES_PATH.relative_to(PROJECT_ROOT)),
                "incident": str(INCIDENT_PATH.relative_to(PROJECT_ROOT)),
                "m3_foundation_result": str(m3_result_path.relative_to(PROJECT_ROOT)),
                "m3_foundation_summary": str(m3_summary_path.relative_to(PROJECT_ROOT)),
                "m3_foundation_samples": str(m3_samples_path.relative_to(PROJECT_ROOT)),
            },
        }
    )
    write_artifacts(result, samples)
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="M4 obstacle-dependent 60 s Room Cruise quality proof validator.")
    parser.add_argument("--test-name", default="M4_room_cruise_quality_validator")
    parser.add_argument("--from-latest", action="store_true", help="Offline replay of the latest M3 Unified evidence; does not move the robot.")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--preflight-duration-s", type=float, default=4.0)
    parser.add_argument("--poll-s", type=float, default=0.12)
    parser.add_argument("--v-max-mps", type=float, default=0.30)
    parser.add_argument("--omega-max-rad-s", type=float, default=0.60)
    parser.add_argument("--base-min-progress-m", type=float, default=0.45)
    parser.add_argument("--min-front-m", type=float, default=0.27)
    parser.add_argument("--token", default="GUI_DEFAULT")
    parser.add_argument("--visual-observation-json", default=str(DEFAULT_VISUAL_OBSERVATION_PATH))
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
