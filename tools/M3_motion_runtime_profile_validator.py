#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""M3 runtime slow-tick profiler for no-motion and bounded pivot windows."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from middleware.peripheral_usage import read_peripherals, set_peripheral_enabled  # noqa: E402
from project_rules.bootstrap_guard import ensure_agent_system_prompt_loaded  # noqa: E402
from tools import M3_motion_primitive_validator as primitive  # noqa: E402
from tools.M3_emberkovetes_mozgasminoseg import _gate, _json_safe, _ratio, _safe_float, _write_json, _write_jsonl  # noqa: E402
from tools.lidar_1m_step import STATUS_PATH, _get_pose, _read_json  # noqa: E402


RUNTIME_DIR = PROJECT_ROOT / "runtime"
AGENT_TESTS_DIR = test_artifacts_dir()
RESULT_PATH = AGENT_TESTS_DIR / "latest_M3_motion_runtime_profile_validator.json"
SUMMARY_PATH = AGENT_TESTS_DIR / "latest_M3_motion_runtime_profile_validator_summary.json"
SAMPLES_PATH = AGENT_TESTS_DIR / "M3_motion_runtime_profile_validator_samples.jsonl"
INCIDENT_PATH = AGENT_TESTS_DIR / "latest_M3_motion_runtime_profile_validator_incident.json"


DEFAULT_THRESHOLDS: Dict[str, float] = {
    "duration_s": 4.0,
    "poll_s": 0.08,
    "loop_frequency_p10_min_hz": 40.0,
    "loop_below_45_ratio_max": 0.15,
    "loop_budget_p95_max_ms": 40.0,
    "slow_tick_ratio_max": 0.35,
    "logger_queue_depth_max": 256.0,
    "logger_flush_p95_max_ms": 140.0,
}


def _max_counter_delta(samples: Sequence[Dict[str, Any]], key: str) -> int:
    values = [int(sample.get(key, 0) or 0) for sample in samples]
    return max(0, max(values) - min(values)) if values else 0


def _counter_map_delta(samples: Sequence[Dict[str, Any]], key: str) -> Dict[str, int]:
    sample_list = [dict(sample or {}) for sample in samples]
    names = {
        str(name)
        for sample in sample_list
        for name in dict(sample.get(key) or {}).keys()
    }
    out: Dict[str, int] = {}
    for name in sorted(names):
        values = [int(dict(sample.get(key) or {}).get(name, 0) or 0) for sample in sample_list]
        delta = max(0, max(values) - min(values)) if values else 0
        if delta > 0:
            out[name] = int(delta)
    return out


def _counter_map_max(samples: Sequence[Dict[str, Any]], key: str) -> Dict[str, int]:
    sample_list = [dict(sample or {}) for sample in samples]
    names = {
        str(name)
        for sample in sample_list
        for name in dict(sample.get(key) or {}).keys()
    }
    return {
        name: max(int(dict(sample.get(key) or {}).get(name, 0) or 0) for sample in sample_list)
        for name in sorted(names)
    }


def _collect_no_motion_samples(*, duration_s: float, poll_s: float, token: str) -> List[Dict[str, Any]]:
    prepare = primitive._prepare_run_start_state(token=str(token), stop_timeout_s=5.0)
    status = dict(prepare.get("status") or _read_json(STATUS_PATH) or {})
    start_pose = _get_pose(status)
    samples: List[Dict[str, Any]] = []
    start = time.monotonic()
    deadline = start + max(0.2, float(duration_s))
    while time.monotonic() <= deadline:
        row_status = _read_json(STATUS_PATH)
        if row_status:
            samples.append(
                primitive._sample_status(
                    row_status,
                    phase="no_motion",
                    start_pose=start_pose,
                    expected_sign=1.0,
                    t_rel_s=max(0.0, time.monotonic() - start),
                )
            )
        time.sleep(max(0.02, float(poll_s)))
    return samples


def _analyze_phase(samples: Sequence[Dict[str, Any]], *, phase: str, thresholds: Dict[str, float]) -> Dict[str, Any]:
    sample_list = [dict(sample or {}) for sample in samples]
    watchdog = [_safe_float(s.get("watchdog_freq_hz"), math.nan) for s in sample_list]
    finite_watchdog = [v for v in watchdog if math.isfinite(float(v))]
    loop_below_ratio = _ratio(sum(1 for v in finite_watchdog if float(v) < 45.0), len(finite_watchdog))
    loop_budget = [_safe_float(s.get("loop_budget_total_ema_ms"), math.nan) for s in sample_list]
    logger_depth = [_safe_float(s.get("logger_queue_depth"), math.nan) for s in sample_list]
    logger_flush = [_safe_float(s.get("logger_flush_duration_ms"), math.nan) for s in sample_list]
    slow_tick_delta, observed_tick_delta, slow_ratio = primitive._slow_tick_window(sample_list)
    coobserved_deltas = _counter_map_delta(sample_list, "slow_coobserved_category_counts")
    primary_timing_deltas = _counter_map_delta(sample_list, "slow_primary_timing_class_counts")
    dominant_phase_deltas = _counter_map_delta(
        sample_list,
        "slow_dominant_processing_phase_counts",
    )
    phase_spike_deltas = _counter_map_delta(sample_list, "slow_phase_spike_counts")
    category_combination_deltas = _counter_map_delta(
        sample_list,
        "slow_category_combination_counts",
    )
    counter_semantics = next(
        (
            dict(sample.get("slow_counter_semantics") or {})
            for sample in reversed(sample_list)
            if sample.get("slow_counter_semantics")
        ),
        {},
    )
    slow = {
        "slow_tick_delta": int(slow_tick_delta),
        "observed_tick_delta": int(observed_tick_delta),
        "slow_lidar_spike_delta": _max_counter_delta(sample_list, "slow_lidar_spike_count"),
        "slow_resolver_spike_delta": _max_counter_delta(sample_list, "slow_resolver_spike_count"),
        "slow_io_event_delta": _max_counter_delta(sample_list, "slow_io_event_count"),
        "slow_gc_delta": _max_counter_delta(sample_list, "slow_gc_count"),
        "slow_scheduler_delay_delta": _max_counter_delta(
            sample_list,
            "slow_scheduler_delay_count",
        ),
        "slow_unattributed_spike_delta": _max_counter_delta(
            sample_list,
            "slow_unattributed_spike_count",
        ),
        "slow_none_delta": _max_counter_delta(sample_list, "slow_none_count"),
        "slow_multi_label_delta": _max_counter_delta(sample_list, "slow_multi_label_count"),
        "slow_scheduler_delay_coobserved_delta": int(
            coobserved_deltas.get("scheduler_delay_observed", 0)
        ),
        "slow_uninstrumented_processing_coobserved_delta": int(
            coobserved_deltas.get("uninstrumented_processing_present", 0)
        ),
        "coobserved_slow_tick_deltas": dict(coobserved_deltas),
        "category_combination_deltas": dict(category_combination_deltas),
        "primary_timing_class_deltas": dict(primary_timing_deltas),
        "dominant_processing_phase_deltas": dict(dominant_phase_deltas),
        "phase_spike_coobserved_deltas": dict(phase_spike_deltas),
        "phase_max_us": _counter_map_max(sample_list, "slow_phase_max_us"),
        "phase_gc_pause_max_us": _counter_map_max(
            sample_list,
            "slow_phase_gc_pause_max_us",
        ),
        "counter_semantics": dict(counter_semantics),
        "max_gc_pause_us": max([int(s.get("slow_max_gc_pause_us", 0) or 0) for s in sample_list] or [0]),
        "max_scheduler_delay_us": max(
            [int(s.get("slow_max_scheduler_delay_us", 0) or 0) for s in sample_list] or [0]
        ),
        "max_unattributed_processing_us": max(
            [int(s.get("slow_max_unattributed_processing_us", 0) or 0) for s in sample_list] or [0]
        ),
        "min_phase_coverage_ratio": min(
            [
                float(s.get("slow_min_phase_coverage_ratio"))
                for s in sample_list
                if math.isfinite(_safe_float(s.get("slow_min_phase_coverage_ratio"), math.nan))
            ]
            or [math.nan]
        ),
    }
    watchdog_summary = primitive._summary(watchdog)
    loop_summary = primitive._summary(loop_budget)
    logger_depth_summary = primitive._summary(logger_depth)
    logger_flush_summary = primitive._summary(logger_flush)
    gate = _gate(
        "PASS"
        if (
            (watchdog_summary.get("p10") is None or float(watchdog_summary.get("p10")) >= thresholds["loop_frequency_p10_min_hz"])
            and (loop_below_ratio is None or float(loop_below_ratio) <= thresholds["loop_below_45_ratio_max"])
            and (loop_summary.get("p95") is None or float(loop_summary.get("p95")) <= thresholds["loop_budget_p95_max_ms"])
            and (logger_depth_summary.get("max") is None or float(logger_depth_summary.get("max")) <= thresholds["logger_queue_depth_max"])
            and (logger_flush_summary.get("p95") is None or float(logger_flush_summary.get("p95")) <= thresholds["logger_flush_p95_max_ms"])
            and slow_ratio <= thresholds["slow_tick_ratio_max"]
        )
        else "FAIL",
        observed={
            "sample_count": len(sample_list),
            "watchdog_frequency_hz": watchdog_summary,
            "loop_below_45_ratio": loop_below_ratio,
            "loop_budget_total_ema_ms": loop_summary,
            "logger_queue_depth": logger_depth_summary,
            "logger_flush_duration_ms": logger_flush_summary,
            **slow,
            "slow_tick_ratio": slow_ratio,
        },
        requirement="runtime loop/logger/slow-tick counters stay bounded in this phase",
    )
    return {
        "phase": str(phase),
        "status": str(gate.get("status")),
        "success": gate.get("status") == "PASS",
        "gates": {"runtime_profile": gate},
        "failed_gates": ["runtime_profile"] if gate.get("status") == "FAIL" else [],
        "metrics": dict(gate.get("observed") or {}),
    }


def _aggregate_profile_verdict(
    phase_results: Sequence[Dict[str, Any]],
    pivot_primitive_validation: Dict[str, Any] | None,
) -> Dict[str, Any]:
    failed: List[str] = []
    inconclusive: List[str] = []
    for phase in phase_results:
        phase_name = str(phase.get("phase") or "unknown")
        phase_failed = list(phase.get("failed_gates") or [])
        phase_inconclusive = list(phase.get("inconclusive_gates") or [])
        if str(phase.get("status") or "").upper() == "FAIL" and not phase_failed:
            phase_failed = ["runtime_profile"]
        if str(phase.get("status") or "").upper() == "INCONCLUSIVE" and not phase_inconclusive:
            phase_inconclusive = ["runtime_profile"]
        failed.extend(f"{phase_name}:{name}" for name in phase_failed)
        inconclusive.extend(f"{phase_name}:{name}" for name in phase_inconclusive)

    pivot = dict(pivot_primitive_validation or {})
    if pivot:
        pivot_status = str(pivot.get("status") or "INCONCLUSIVE").upper()
        pivot_failed = list(pivot.get("failed_gates") or [])
        pivot_inconclusive = list(pivot.get("inconclusive_gates") or [])
        if pivot_status == "FAIL" and not pivot_failed:
            pivot_failed = ["primitive_validation"]
        elif pivot_status != "PASS" and not pivot_failed and not pivot_inconclusive:
            pivot_inconclusive = ["primitive_validation"]
        failed.extend(f"pivot_primitive:{name}" for name in pivot_failed)
        inconclusive.extend(f"pivot_primitive:{name}" for name in pivot_inconclusive)

    failed = list(dict.fromkeys(failed))
    inconclusive = list(dict.fromkeys(inconclusive))
    status = "FAIL" if failed else ("INCONCLUSIVE" if inconclusive else "PASS")
    return {
        "status": status,
        "success": status == "PASS",
        "failed_gates": failed,
        "inconclusive_gates": inconclusive,
    }


def _compact_pivot_primitive_validation(case_result: Dict[str, Any]) -> Dict[str, Any]:
    result = dict(case_result or {})
    metrics = dict(result.get("metrics") or {})
    gates = dict(result.get("gates") or {})
    failed_gates = list(result.get("failed_gates") or [])
    inconclusive_gates = list(result.get("inconclusive_gates") or [])
    relevant_gate_names = list(dict.fromkeys(failed_gates + inconclusive_gates))
    raw = dict(result.get("raw") or {})
    active = primitive._active_motion_samples(
        list(raw.get("motion_samples") or []) + list(raw.get("settle_samples") or [])
    )

    metric_names = (
        "active_sample_count",
        "active_unique_status_frame_count",
        "active_observation_span_s",
        "directional_progress_max_deg",
        "primitive_actual_expected_ratio",
        "actual_classifier_eligible_sample_count",
        "actual_classifier_unique_status_frame_count",
        "actual_classifier_coverage_ratio",
        "actual_contract_violation_samples",
        "actual_contract_violation_unique_status_frame_count",
        "slow_tick_delta",
        "slow_observed_tick_delta",
        "slow_tick_ratio",
    )
    return {
        "status": result.get("status"),
        "failed_gates": failed_gates,
        "inconclusive_gates": inconclusive_gates,
        "metrics": {key: metrics.get(key) for key in metric_names},
        "gate_details": {
            name: {
                "status": dict(gates.get(name) or {}).get("status"),
                "observed": dict(gates.get(name) or {}).get("observed"),
                "requirement": dict(gates.get(name) or {}).get("requirement"),
            }
            for name in relevant_gate_names
        },
        "mismatch_examples": _pivot_mismatch_examples(active),
    }


def _pivot_mismatch_examples(
    active_samples: Sequence[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    examples: List[Dict[str, Any]] = []
    seen_versions = set()
    for sample in active_samples:
        actual = str(sample.get("turn_primitive_actual") or "").upper()
        if not bool(sample.get("primitive_contract_violation", False)) and actual == primitive.EXPECTED_PIVOT_PRIMITIVE:
            continue
        status_version = int(sample.get("status_version", 0) or 0)
        version_key = ("version", status_version) if status_version > 0 else ("sample", len(examples))
        if version_key in seen_versions:
            continue
        seen_versions.add(version_key)
        examples.append(
            {
                "t_rel_s": _safe_float(sample.get("t_rel_s"), math.nan),
                "status_version": status_version,
                "turn_primitive_actual": actual or "UNKNOWN",
                "primitive_contract_violation": bool(
                    sample.get("primitive_contract_violation", False)
                ),
                "actual_v_mps": _safe_float(sample.get("actual_v"), math.nan),
                "actual_omega_rad_s": _safe_float(sample.get("actual_omega"), math.nan),
                "actual_left_mps": _safe_float(sample.get("actual_left_mps"), math.nan),
                "actual_right_mps": _safe_float(sample.get("actual_right_mps"), math.nan),
                "measurement_ready": bool(
                    sample.get("actual_primitive_measurement_ready", False)
                ),
                "measurement_reliable": bool(
                    sample.get("actual_primitive_measurement_reliable", False)
                ),
            }
        )
        if len(examples) >= 4:
            break
    return examples


def _augment_pivot_from_saved_samples(result: Dict[str, Any]) -> Dict[str, Any]:
    out = dict(result or {})
    pivot = dict(out.get("pivot_primitive_validation") or {})
    if not pivot or not SAMPLES_PATH.exists():
        return out
    samples: List[Dict[str, Any]] = []
    with SAMPLES_PATH.open("r", encoding="utf-8") as handle:
        for line in handle:
            text = line.strip()
            if not text:
                continue
            sample = dict(json.loads(text) or {})
            if str(sample.get("profile_phase") or "") == "pivot":
                samples.append(sample)
    active = primitive._active_motion_samples(samples)
    classifier = [
        sample for sample in active if primitive._actual_classifier_eligible(sample)
    ]
    violations = [
        sample
        for sample in active
        if bool(sample.get("primitive_contract_violation", False))
    ]
    active_observation = primitive._observation_metrics(active)
    classifier_observation = primitive._observation_metrics(classifier)
    violation_observation = primitive._observation_metrics(violations)
    metrics = dict(pivot.get("metrics") or {})
    metrics.update(
        {
            "active_sample_count": len(active),
            "active_unique_status_frame_count": active_observation[
                "unique_status_frame_count"
            ],
            "active_observation_span_s": active_observation["observation_span_s"],
            "directional_progress_max_deg": max(
                [
                    _safe_float(sample.get("directional_progress_deg"), 0.0)
                    for sample in samples
                ]
                or [0.0]
            ),
            "actual_classifier_eligible_sample_count": len(classifier),
            "actual_classifier_unique_status_frame_count": classifier_observation[
                "unique_status_frame_count"
            ],
            "actual_contract_violation_samples": len(violations),
            "actual_contract_violation_unique_status_frame_count": violation_observation[
                "unique_status_frame_count"
            ],
        }
    )
    pivot["metrics"] = metrics
    pivot["mismatch_examples"] = _pivot_mismatch_examples(active)
    out["pivot_primitive_validation"] = pivot
    return out


def _apply_profile_verdict(result: Dict[str, Any]) -> Dict[str, Any]:
    out = dict(result or {})
    verdict = _aggregate_profile_verdict(
        list(out.get("phases") or []),
        dict(out.get("pivot_primitive_validation") or {}),
    )
    out.update(verdict)
    failed = list(verdict["failed_gates"])
    inconclusive = list(verdict["inconclusive_gates"])
    details = []
    if failed:
        details.append(f"Hibak: {', '.join(failed)}")
    if inconclusive:
        details.append(f"Nem eldönthető: {', '.join(inconclusive)}")
    out["plain_summary_hu"] = (
        f"Runtime profil: {verdict['status']}. "
        + ("; ".join(details) if details else "Hibák nincsenek.")
    )
    return out


def build_summary(result: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "schema": result.get("schema"),
        "status": result.get("status"),
        "success": bool(result.get("success", False)),
        "test_name": result.get("test_name"),
        "plain_summary_hu": result.get("plain_summary_hu"),
        "failed_gates": list(result.get("failed_gates") or []),
        "inconclusive_gates": list(result.get("inconclusive_gates") or []),
        "phases": [
            {
                "phase": phase.get("phase"),
                "status": phase.get("status"),
                "failed_gates": list(phase.get("failed_gates") or []),
                "inconclusive_gates": list(phase.get("inconclusive_gates") or []),
                "metrics": phase.get("metrics"),
            }
            for phase in list(result.get("phases") or [])
        ],
        "pivot_primitive_validation": dict(result.get("pivot_primitive_validation") or {}),
        "artifact_paths": dict(result.get("artifact_paths") or {}),
        "reassessment": dict(result.get("reassessment") or {}),
    }


def run(args: argparse.Namespace) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    thresholds = dict(DEFAULT_THRESHOLDS)
    thresholds["duration_s"] = float(args.duration_s)
    thresholds["poll_s"] = float(args.poll_s)
    original_peripherals = read_peripherals(runtime_dir=RUNTIME_DIR, use_cache=False)
    camera_disable = {"requested": bool(args.disable_camera), "original_camera": bool(original_peripherals.get("camera", False))}
    if bool(args.disable_camera):
        camera_disable["peripherals_after_disable"] = set_peripheral_enabled("camera", False, runtime_dir=RUNTIME_DIR)
        time.sleep(max(0.0, float(args.camera_settle_s)))

    modes = [item.strip() for item in str(args.mode or "").split(",") if item.strip()]
    if not modes:
        modes = ["no_motion"]
    all_samples: List[Dict[str, Any]] = []
    phase_results: List[Dict[str, Any]] = []
    pivot_primitive_validation: Dict[str, Any] = {}
    if "no_motion" in modes:
        samples = _collect_no_motion_samples(duration_s=float(args.duration_s), poll_s=float(args.poll_s), token=str(args.token))
        for sample in samples:
            row = dict(sample)
            row["profile_phase"] = "no_motion"
            all_samples.append(row)
        phase_results.append(_analyze_phase(samples, phase="no_motion", thresholds=thresholds))
    if "pivot" in modes:
        pivot_thresholds = dict(primitive.DEFAULT_THRESHOLDS)
        pivot_thresholds.update({"target_angle_deg": float(args.pivot_target_angle_deg), "poll_s": float(args.poll_s)})
        case = primitive.PrimitiveCase("pivot_left", left_mps=-float(args.pivot_track_speed_mps), right_mps=float(args.pivot_track_speed_mps))
        case_result = primitive.run_pivot_case(case, token=str(args.token), thresholds=pivot_thresholds)
        pivot_primitive_validation = _compact_pivot_primitive_validation(case_result)
        raw = dict(case_result.get("raw") or {})
        pivot_samples = list(raw.get("motion_samples") or []) + list(raw.get("settle_samples") or [])
        for sample in pivot_samples:
            row = dict(sample)
            row["profile_phase"] = "pivot"
            all_samples.append(row)
        phase_results.append(_analyze_phase(pivot_samples, phase="pivot", thresholds=thresholds))

    result = {
        "schema": "M3_MOTION_RUNTIME_PROFILE_VALIDATOR_V2",
        "test_name": str(args.test_name),
        "generated_ts": time.time(),
        "camera_disable": camera_disable,
        "thresholds": thresholds,
        "phases": phase_results,
        "pivot_primitive_validation": pivot_primitive_validation,
        "artifact_paths": {
            "result": str(RESULT_PATH.relative_to(PROJECT_ROOT)),
            "summary": str(SUMMARY_PATH.relative_to(PROJECT_ROOT)),
            "samples": str(SAMPLES_PATH.relative_to(PROJECT_ROOT)),
            "incident": str(INCIDENT_PATH.relative_to(PROJECT_ROOT)),
        },
    }
    result = _apply_profile_verdict(result)
    status = str(result.get("status") or "FAIL")
    failed = list(result.get("failed_gates") or [])
    inconclusive = list(result.get("inconclusive_gates") or [])
    _write_jsonl(SAMPLES_PATH, all_samples)
    _write_json(RESULT_PATH, result)
    _write_json(SUMMARY_PATH, build_summary(result))
    _write_json(
        INCIDENT_PATH,
        {
            "schema": "M3_MOTION_RUNTIME_PROFILE_INCIDENT_V2",
            "needed": status != "PASS",
            "status": status,
            "failed_gates": failed,
            "inconclusive_gates": inconclusive,
            "artifact_paths": result["artifact_paths"],
        },
    )
    return result


def reassess_existing_result() -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    with RESULT_PATH.open("r", encoding="utf-8") as handle:
        original = dict(json.load(handle) or {})
    input_status = str(original.get("status") or "")
    result = _apply_profile_verdict(_augment_pivot_from_saved_samples(original))
    result["schema"] = "M3_MOTION_RUNTIME_PROFILE_VALIDATOR_V2"
    result["reassessment"] = {
        "mode": "OFFLINE_EXISTING_ARTIFACT",
        "input_status": input_status,
        "reassessed_ts": time.time(),
        "motion_executed": False,
    }
    _write_json(RESULT_PATH, result)
    _write_json(SUMMARY_PATH, build_summary(result))
    _write_json(
        INCIDENT_PATH,
        {
            "schema": "M3_MOTION_RUNTIME_PROFILE_INCIDENT_V2",
            "needed": result.get("status") != "PASS",
            "status": result.get("status"),
            "failed_gates": list(result.get("failed_gates") or []),
            "inconclusive_gates": list(result.get("inconclusive_gates") or []),
            "artifact_paths": dict(result.get("artifact_paths") or {}),
            "reassessment": dict(result.get("reassessment") or {}),
        },
    )
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="M3 runtime slow-tick profiler.")
    parser.add_argument("--test-name", default="M3_motion_runtime_profile_validator")
    parser.add_argument("--mode", default="no_motion")
    parser.add_argument("--duration-s", type=float, default=DEFAULT_THRESHOLDS["duration_s"])
    parser.add_argument("--poll-s", type=float, default=DEFAULT_THRESHOLDS["poll_s"])
    parser.add_argument("--pivot-target-angle-deg", type=float, default=20.0)
    parser.add_argument("--pivot-track-speed-mps", type=float, default=0.035)
    parser.add_argument("--token", default="GUI_DEFAULT")
    parser.add_argument("--disable-camera", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--camera-settle-s", type=float, default=0.4)
    parser.add_argument("--reassess-existing", action="store_true")
    parser.add_argument("--compact", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    result = reassess_existing_result() if bool(args.reassess_existing) else run(args)
    output = result
    if bool(args.compact):
        output = {
            "status": result.get("status"),
            "plain_summary_hu": result.get("plain_summary_hu"),
            "failed_gates": result.get("failed_gates"),
            "inconclusive_gates": result.get("inconclusive_gates"),
            "artifact_paths": result.get("artifact_paths"),
            "reassessment": result.get("reassessment"),
        }
    print(json.dumps(_json_safe(output), ensure_ascii=False))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
