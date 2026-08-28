#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Single-trial straight-line validation helper for LIDAR_FIRST odometry.

Design goals:
- use the normal runtime command bus only (`runtime/commands.jsonl`)
- clear an existing FAILSAFE with the minimal correct reset (`strong_reset`)
- keep pose SSOT in the EKF while allowing KIT0085 encoder fusion in LIDAR_FIRST mode
- verify live status + live raw LIDAR + forward clearance before motion
- command straight forward motion with `set_twist` keepalive
- stop with zero twist instead of the emergency-stop `stop` command
- return one structured result for a human-in-the-loop validation workflow
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.command_bus import get_latest_command_status  # noqa: E402
from log.runtime_debug import load_log_switches  # noqa: E402
from log.log_paths import test_artifacts_dir  # noqa: E402

from tools.runtime_status_client import get_runtime_status_client


STATUS_PATH = Path("runtime/status.json")
CURRENT_POSE_PATH = Path("runtime/current_pose.json")
LIDAR_SCAN_PATH = Path("runtime/lidar_scan.json")
LIDAR_FULL_SCAN_PATH = Path("runtime/lidar_scan_full.json")
LIDAR_SCAN_SUBSCRIBER_PATH = Path("runtime/lidar_scan_subscriber.json")
COMMANDS_PATH = Path("runtime/commands.jsonl")
RAW_LOG_PATH = test_artifacts_dir() / "lidar_first_straight_trials_raw.jsonl"

DEFAULT_TOKEN = "GUI_DEFAULT"
DEFAULT_MOTION_SOURCE = "STATE"
DEFAULT_TARGET_DISTANCE_M = 1.0
DEFAULT_V_MPS = 0.15
DEFAULT_MOVE_TIMEOUT_S = 20.0
DEFAULT_STOP_TIMEOUT_S = 8.0
DEFAULT_KEEPALIVE_INTERVAL_S = 0.15
DEFAULT_POLL_S = 0.05
DEFAULT_LIDAR_STALE_S = 1.0
DEFAULT_STATUS_STALE_S = 3.0
DEFAULT_HEADING_ABORT_DEG = 15.0
DEFAULT_RUNTIME_STALE_STOP_S = 0.60
DEFAULT_START_GATE_TIMEOUT_S = 6.0
DEFAULT_START_GATE_MIN_STATUS_INCREMENTS = 3
DEFAULT_START_GATE_MIN_SCAN_INCREMENTS = 3
DEFAULT_START_GATE_MIN_FRESH_SCAN_STREAK = 3
DEFAULT_START_GATE_FRESHNESS_RESERVE_S = 0.22
DEFAULT_START_GATE_MIN_CONFIDENCE = 0.20
RAW_SAFETY_SOURCE = "PARENT_CURRENT_RAW_SCAN"
STRAIGHT_CORRIDOR_FULL_RAW_SOURCE = "FULL_CURRENT_RAW_SCAN_STRAIGHT_CORRIDOR"
DEFAULT_STRAIGHT_CORRIDOR_HALF_WIDTH_M = 0.30
DEFAULT_FULL_SCAN_FRESHNESS_S = 0.75

_RUNTIME_STATUS_CLIENT = get_runtime_status_client()
_RUNTIME_STATUS_MIN_POLL_S = max(0.05, float(os.environ.get("R2B4_RUNTIME_STATUS_MIN_POLL_S", "0.10")))


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return float(default)
        return float(value)
    except Exception:
        return float(default)


def _is_finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def _canonical_turn_primitives(status: Dict[str, Any]) -> Dict[str, str]:
    st = dict(status or {})
    motion_command = dict(st.get("motion_command") or {})
    turn_primitives = dict(st.get("turn_primitives") or motion_command.get("turn_primitives") or {})
    turn_semantics = dict(motion_command.get("turn_semantics") or {})
    requested = dict(turn_semantics.get("requested") or {})
    limited = dict(turn_semantics.get("limited") or {})
    executed = dict(turn_semantics.get("executed") or {})
    actual = dict(turn_semantics.get("actual") or {})

    def _norm(value: Any, fallback: str = "UNKNOWN") -> str:
        raw = str(value or "").strip().upper()
        return raw if raw else str(fallback)

    return {
        "requested": _norm(
            requested.get(
                "turn_primitive",
                motion_command.get(
                    "turn_primitive_requested",
                    turn_primitives.get("requested", st.get("turn_primitive_requested", "")),
                ),
            )
        ),
        "limited": _norm(
            limited.get(
                "turn_primitive",
                motion_command.get(
                    "turn_primitive_limited",
                    turn_primitives.get("limited", st.get("turn_primitive_limited", "")),
                ),
            )
        ),
        "executed": _norm(
            executed.get(
                "turn_primitive",
                motion_command.get(
                    "turn_primitive_executed",
                    turn_primitives.get("executed", st.get("turn_primitive_executed", "")),
                ),
            )
        ),
        "actual": _norm(
            actual.get(
                "turn_primitive",
                motion_command.get(
                    "turn_primitive_actual",
                    turn_primitives.get("actual", st.get("turn_primitive_actual", "")),
                ),
            )
        ),
    }


def _normalize_trial_identity(trial: Any) -> tuple[int, str]:
    label = str(trial or "").strip() or "1"
    try:
        return int(label), label
    except Exception:
        return 1, label


def _positive_int_or_none(value: Any) -> Optional[int]:
    try:
        out = int(value)
    except (TypeError, ValueError):
        return None
    return out if out > 0 else None


def _extract_lidar_observation(status: Dict[str, Any]) -> Dict[str, Any]:
    """Return the exact ID lineage of the LIDAR evidence in one status snapshot."""
    lidar_odom = dict((status or {}).get("lidar_odom_status") or {})
    observation = {
        "raw_scan_id": _positive_int_or_none(lidar_odom.get("raw_scan_id")),
        "matcher_result_id": _positive_int_or_none(lidar_odom.get("matcher_result_id")),
        "candidate_id": _positive_int_or_none(lidar_odom.get("candidate_id")),
        "candidate_source_raw_scan_id": _positive_int_or_none(
            lidar_odom.get("candidate_source_raw_scan_id")
        ),
        "lidar_odometry_measurement_id": _positive_int_or_none(
            lidar_odom.get("lidar_odometry_measurement_id")
        ),
        "measurement_source_matcher_result_id": _positive_int_or_none(
            lidar_odom.get("measurement_source_matcher_result_id")
        ),
        "measurement_source_raw_scan_id": _positive_int_or_none(
            lidar_odom.get("measurement_source_raw_scan_id")
        ),
        "ekf_input_lidar_odometry_measurement_id": _positive_int_or_none(
            lidar_odom.get("ekf_input_lidar_odometry_measurement_id")
        ),
        "ekf_last_processed_lidar_odometry_measurement_id": _positive_int_or_none(
            lidar_odom.get("ekf_last_processed_lidar_odometry_measurement_id")
        ),
        "ekf_last_applied_lidar_odometry_measurement_id": _positive_int_or_none(
            lidar_odom.get("ekf_last_applied_lidar_odometry_measurement_id")
        ),
        "ekf_duplicate_measurement_rejected_total": int(
            max(0, _safe_float(lidar_odom.get("ekf_duplicate_measurement_rejected_total"), 0.0))
        ),
        "ekf_missing_measurement_id_rejected_total": int(
            max(0, _safe_float(lidar_odom.get("ekf_missing_measurement_id_rejected_total"), 0.0))
        ),
        "rejected_duplicate_matcher_result": int(
            max(0, _safe_float(lidar_odom.get("rejected_duplicate_matcher_result"), 0.0))
        ),
        "rejected_duplicate_raw_scan": int(
            max(0, _safe_float(lidar_odom.get("rejected_duplicate_raw_scan"), 0.0))
        ),
    }
    errors: List[str] = []
    matcher_id = observation["matcher_result_id"]
    candidate_id = observation["candidate_id"]
    if matcher_id is not None and candidate_id is not None and matcher_id != candidate_id:
        errors.append(f"candidate_matcher_id_mismatch:{candidate_id}!={matcher_id}")
    measurement_id = observation["lidar_odometry_measurement_id"]
    if measurement_id is not None:
        if observation["measurement_source_matcher_result_id"] is None:
            errors.append(f"measurement_source_matcher_result_id_missing:{measurement_id}")
        if observation["measurement_source_raw_scan_id"] is None:
            errors.append(f"measurement_source_raw_scan_id_missing:{measurement_id}")
    if bool(lidar_odom.get("applied", False)):
        input_id = observation["ekf_input_lidar_odometry_measurement_id"]
        applied_id = observation["ekf_last_applied_lidar_odometry_measurement_id"]
        if measurement_id is None:
            errors.append("applied_measurement_id_missing")
        if input_id is None:
            errors.append("applied_ekf_input_measurement_id_missing")
        if applied_id is None:
            errors.append("applied_ekf_last_applied_measurement_id_missing")
        if measurement_id is not None and input_id is not None and measurement_id != input_id:
            errors.append(f"applied_input_id_mismatch:{measurement_id}!={input_id}")
        if input_id is not None and applied_id is not None and input_id != applied_id:
            errors.append(f"applied_last_id_mismatch:{input_id}!={applied_id}")
    observation["lineage_errors"] = errors
    return observation


def _summarize_lidar_observation_rows(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Aggregate confidence/freshness once per odometry measurement, never per poll."""
    observations: Dict[int, Dict[str, Any]] = {}
    applied_ids = set()
    raw_ids = set()
    matcher_ids = set()
    errors: List[str] = []
    missing_applied_ids = 0
    applied_status_samples = 0

    for row in list(rows or []):
        obs = dict(row.get("lidar_observation") or {})
        errors.extend(str(item) for item in list(obs.get("lineage_errors") or []) if str(item))
        raw_id = _positive_int_or_none(obs.get("raw_scan_id"))
        matcher_id = _positive_int_or_none(obs.get("matcher_result_id"))
        measurement_id = _positive_int_or_none(obs.get("lidar_odometry_measurement_id"))
        if raw_id is not None:
            raw_ids.add(raw_id)
        if matcher_id is not None:
            matcher_ids.add(matcher_id)

        applied = bool(row.get("lidar_odom_applied", False))
        if applied:
            applied_status_samples += 1
            if measurement_id is None:
                missing_applied_ids += 1
            else:
                applied_ids.add(measurement_id)

        if measurement_id is None:
            continue
        confidence = _safe_float(row.get("lidar_odom_latest_confidence"), math.nan)
        age_s = _safe_float(row.get("lidar_odom_latest_age_s"), math.nan)
        signature = (
            _positive_int_or_none(obs.get("measurement_source_matcher_result_id")),
            _positive_int_or_none(obs.get("measurement_source_raw_scan_id")),
            float(confidence) if math.isfinite(float(confidence)) else None,
        )
        previous = observations.get(measurement_id)
        if previous is not None:
            if tuple(previous["signature"]) != signature:
                errors.append(f"measurement_id_reused_with_changed_payload:{measurement_id}")
            continue
        observations[measurement_id] = {
            "signature": signature,
            "confidence": float(confidence) if math.isfinite(float(confidence)) else None,
            "age_s": float(age_s) if math.isfinite(float(age_s)) else None,
            "healthy": bool(row.get("lidar_localization_healthy", False)),
        }

    confidence_values = [
        float(item["confidence"])
        for item in observations.values()
        if item.get("confidence") is not None
    ]
    age_values = [
        float(item["age_s"])
        for item in observations.values()
        if item.get("age_s") is not None
    ]
    return {
        "poll_samples": int(len(rows or [])),
        "applied_status_samples": int(applied_status_samples),
        "applied_samples": int(len(applied_ids)),
        "applied_measurement_ids": sorted(int(item) for item in applied_ids),
        "applied_missing_measurement_id_samples": int(missing_applied_ids),
        "unique_lidar_odometry_measurements": int(len(observations)),
        "lidar_odometry_measurement_ids": sorted(int(item) for item in observations),
        "unique_matcher_result_observations": int(len(matcher_ids)),
        "unique_raw_scan_observations": int(len(raw_ids)),
        "localization_healthy_samples": int(
            sum(1 for item in observations.values() if bool(item.get("healthy", False)))
        ),
        "confidence_values": confidence_values,
        "age_values": age_values,
        "observation_contract_errors": list(dict.fromkeys(errors)),
    }


def _extract_truth_basis(status: Dict[str, Any]) -> Dict[str, Any]:
    st = dict(status or {})
    truth_src = dict(st.get("truth_basis") or {})
    lidar_odom = dict(st.get("lidar_odom_status") or {})
    lidar_observation = _extract_lidar_observation(st)
    motion_command = dict(st.get("motion_command") or {})
    turn_primitives = _canonical_turn_primitives(st)

    def _finite_or_none(value: Any) -> Optional[float]:
        out = _safe_float(value, math.nan)
        if not math.isfinite(float(out)):
            return None
        return float(out)

    motion_actual_ssot = str(
        truth_src.get("motion_actual_ssot")
        or st.get("motion_actual_ssot")
        or "EKF_POSE_ODOMETRY_SSOT"
    ).strip().upper()
    encoder_pose_active_samples = int(
        max(
            0,
            _safe_float(
                truth_src.get(
                    "encoder_pose_active_samples",
                    st.get(
                        "encoder_pose_active_samples",
                        1 if bool(st.get("encoder_pose_fusion_active", False)) else 0,
                    ),
                ),
                0.0,
            ),
        )
    )
    lidar_latest_age_s = _finite_or_none(
        truth_src.get("lidar_odom_latest_age_s", lidar_odom.get("latest_age_s"))
    )
    lidar_latest_confidence = _finite_or_none(
        truth_src.get("lidar_odom_latest_confidence", lidar_odom.get("latest_confidence"))
    )
    lidar_accepted_total = int(
        max(
            0,
            _safe_float(
                truth_src.get("lidar_odom_accepted_total", lidar_odom.get("accepted", 0)),
                0.0,
            ),
        )
    )
    lidar_total_samples = int(
        max(
            0,
            _safe_float(
                truth_src.get("lidar_odom_total_samples", lidar_odom.get("total", 0)),
                0.0,
            ),
        )
    )
    lidar_applied_samples = int(
        max(
            0,
            max(
                _safe_float(truth_src.get("lidar_odom_applied_samples"), 0.0),
                _safe_float(truth_src.get("lidar_odom_accepted_total"), 0.0),
                _safe_float(lidar_odom.get("accepted"), 0.0),
                1.0 if bool(lidar_odom.get("applied", False)) else 0.0,
            ),
        )
    )
    lidar_delivery_status = str(
        truth_src.get("lidar_odom_delivery_status", lidar_odom.get("delivery_status", ""))
        or ""
    ).strip().lower()
    lidar_get_odometry_result = str(
        truth_src.get("lidar_odom_get_odometry_result", lidar_odom.get("get_odometry_result", ""))
        or ""
    ).strip().lower()
    turn_primitive_source = dict(
        truth_src.get("turn_primitive_source")
        or st.get("turn_primitive_source")
        or motion_command.get("turn_primitive_source")
        or {}
    )
    track_idle_transition_contract = dict(
        truth_src.get("track_idle_transition_contract")
        or st.get("track_idle_transition_contract")
        or motion_command.get("track_idle_transition_contract")
        or {}
    )
    out_truth_basis = {
        "motion_actual_ssot": str(motion_actual_ssot or "EKF_POSE_ODOMETRY_SSOT"),
        "odometry_mode": str(
            truth_src.get("odometry_mode", st.get("odometry_mode", "LIDAR_FIRST"))
        ).strip().upper(),
        "encoder_pose_active_samples": int(encoder_pose_active_samples),
        "lidar_odom_applied_samples": int(lidar_applied_samples),
        "lidar_odom_accepted_total": int(lidar_accepted_total),
        "lidar_odom_total_samples": int(lidar_total_samples),
        "lidar_odom_delivery_status": str(lidar_delivery_status),
        "lidar_odom_get_odometry_result": str(lidar_get_odometry_result),
        "lidar_odom_latest_age_s": lidar_latest_age_s,
        "lidar_odom_latest_confidence": lidar_latest_confidence,
        "lidar_observation": dict(lidar_observation),
        "arc_inner_track_min_mps": _finite_or_none(
            truth_src.get(
                "arc_inner_track_min_mps",
                motion_command.get("arc_inner_track_min_mps", st.get("arc_inner_track_min_mps")),
            )
        ),
        "arc_track_ratio": _finite_or_none(
            truth_src.get(
                "arc_track_ratio",
                motion_command.get("arc_track_ratio", st.get("arc_track_ratio")),
            )
        ),
        "arc_pivot_like_samples": int(
            max(
                0,
                _safe_float(
                    truth_src.get(
                        "arc_pivot_like_samples",
                        motion_command.get("arc_pivot_like_samples", st.get("arc_pivot_like_samples", 0)),
                    ),
                    0.0,
                ),
            )
        ),
        "arc_inner_track_positive_ratio": _finite_or_none(
            truth_src.get(
                "arc_inner_track_positive_ratio",
                motion_command.get("arc_inner_track_positive_ratio", st.get("arc_inner_track_positive_ratio")),
            )
        ),
        "arc_sample_count": int(
            max(
                0,
                _safe_float(
                    truth_src.get(
                        "arc_sample_count",
                        motion_command.get("arc_sample_count", st.get("arc_sample_count", 0)),
                    ),
                    0.0,
                ),
            )
        ),
        "turn_primitive_source": dict(turn_primitive_source),
        "track_idle_transition_contract": dict(track_idle_transition_contract),
    }
    return {
        "motion_actual_ssot": str(motion_actual_ssot or "EKF_POSE_ODOMETRY_SSOT"),
        "truth_basis": out_truth_basis,
        "lidar_odom_applied": bool(lidar_odom.get("applied", False)),
        "lidar_odom_latest_age_s": lidar_latest_age_s,
        "lidar_odom_latest_confidence": lidar_latest_confidence,
        "lidar_observation": dict(lidar_observation),
        "lidar_odom_status": {
            "status": str(lidar_odom.get("status", "") or ""),
            "odometry_status": str(lidar_odom.get("odometry_status", "") or ""),
            "ekf_status": str(lidar_odom.get("ekf_status", "") or ""),
            "applied": bool(lidar_odom.get("applied", False)),
            "latest_age_s": lidar_latest_age_s,
            "latest_confidence": lidar_latest_confidence,
            "accepted": int(max(0, _safe_float(lidar_odom.get("accepted"), 0.0))),
            "total": int(max(0, _safe_float(lidar_odom.get("total"), 0.0))),
            **dict(lidar_observation),
        },
        "encoder_pose_active_samples": int(encoder_pose_active_samples),
        "arc_inner_track_min_mps": out_truth_basis.get("arc_inner_track_min_mps"),
        "arc_track_ratio": out_truth_basis.get("arc_track_ratio"),
        "arc_pivot_like_samples": out_truth_basis.get("arc_pivot_like_samples"),
        "arc_inner_track_positive_ratio": out_truth_basis.get("arc_inner_track_positive_ratio"),
        "arc_sample_count": out_truth_basis.get("arc_sample_count"),
        "execution_mode": str(
            st.get("motion_execution_mode", motion_command.get("execution_mode", ""))
            or ""
        ).strip().upper(),
        "turn_primitive_requested": str(turn_primitives["requested"]),
        "turn_primitive_limited": str(turn_primitives["limited"]),
        "turn_primitive_executed": str(turn_primitives["executed"]),
        "turn_primitive_actual": str(turn_primitives["actual"]),
        "turn_primitive_source": dict(turn_primitive_source),
        "track_idle_transition_contract": dict(track_idle_transition_contract),
        "turn_primitives": dict(turn_primitives),
    }


def _read_json(path: Path) -> Dict[str, Any]:
    try:
        resolved = Path(path).resolve()
        if resolved in {
            STATUS_PATH.resolve(),
            CURRENT_POSE_PATH.resolve(),
            LIDAR_SCAN_PATH.resolve(),
        }:
            return _RUNTIME_STATUS_CLIENT.read_json(
                resolved,
                min_poll_interval_s=_RUNTIME_STATUS_MIN_POLL_S,
            )
    except Exception:
        pass
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.tmp")
    tmp.write_text(
        json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(tmp, path)


def _append_jsonl(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(payload, ensure_ascii=False) + "\n")


def _make_cmd_id(prefix: str = "lidar1m") -> str:
    return f"{prefix}_{int(time.time() * 1000)}_{int(time.perf_counter() * 1_000_000) % 1_000_000}"


def _append_command(cmd_type: str, *, token: str, **kwargs: Any) -> str:
    cmd_id = _make_cmd_id()
    payload = {
        "type": str(cmd_type),
        "token": str(token),
        "ts": time.time(),
        "cmd_id": cmd_id,
    }
    payload.update(kwargs)
    _append_jsonl(COMMANDS_PATH, payload)
    return cmd_id


def _latest_command_status(cmd_id: str, max_lines: int = 8000) -> Optional[Dict[str, Any]]:
    return get_latest_command_status(str(cmd_id), max_lines=max_lines)


def _wait_command_terminal(cmd_id: str, *, timeout_s: float, poll_s: float = 0.05) -> Dict[str, Any]:
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    last_row: Dict[str, Any] = {}
    while time.monotonic() <= deadline:
        row = _latest_command_status(cmd_id)
        if row:
            last_row = row
            state = str(row.get("state", "")).strip().lower()
            if state in ("effective", "failed"):
                return row
        time.sleep(max(0.01, poll_s))
    out = dict(last_row)
    out.setdefault("cmd_id", cmd_id)
    out.setdefault("state", "failed")
    out.setdefault("reason", "command_status_timeout")
    return out


def _send_command_checked(cmd_type: str, *, token: str, timeout_s: float = 4.0, **kwargs: Any) -> Dict[str, Any]:
    sent_wall = time.time()
    sent_mono = time.monotonic()
    cmd_id = _append_command(cmd_type, token=token, **kwargs)
    status = _wait_command_terminal(cmd_id, timeout_s=timeout_s)
    if str(status.get("state", "")).strip().lower() != "effective":
        reason = str(status.get("reason", "") or status.get("error_code", "") or "command_failed")
        raise RuntimeError(f"Command '{cmd_type}' failed ({reason}), cmd_id={cmd_id}")
    return {
        "cmd_id": cmd_id,
        "cmd_type": str(cmd_type),
        "sent_ts_wall": float(sent_wall),
        "duration_s": float(time.monotonic() - sent_mono),
        "status": status,
    }


def _status_version(status: Dict[str, Any]) -> int:
    try:
        return int(status.get("status_version", -1))
    except Exception:
        return -1


def _wait_for_status(timeout_s: float = 5.0) -> Dict[str, Any]:
    deadline = time.monotonic() + max(0.1, timeout_s)
    while time.monotonic() <= deadline:
        st = _read_json(STATUS_PATH)
        if st:
            return st
        time.sleep(0.05)
    raise RuntimeError(f"Status not available: {STATUS_PATH}")


def _wait_for_status_progress(min_increments: int = 2, timeout_s: float = 4.0) -> Dict[str, Any]:
    st0 = _wait_for_status(timeout_s=max(0.5, timeout_s))
    v0 = _status_version(st0)
    increments = 0
    deadline = time.monotonic() + max(0.5, timeout_s)
    last = st0
    while time.monotonic() <= deadline:
        st = _read_json(STATUS_PATH)
        if st:
            last = st
            v = _status_version(st)
            if v > v0:
                increments += 1
                v0 = v
                if increments >= max(1, int(min_increments)):
                    return st
        time.sleep(0.05)
    raise RuntimeError("status_version not progressing. Controller main loop appears inactive.")


def _wait_for_lidar_scan_progress(min_increments: int = 2, timeout_s: float = 2.5) -> Dict[str, Any]:
    scan0 = _read_json(LIDAR_SCAN_PATH)
    ts0 = _safe_float(scan0.get("ts"), math.nan)
    if not _is_finite(ts0):
        raise RuntimeError("runtime/lidar_scan.json not available or missing ts.")
    increments = 0
    deadline = time.monotonic() + max(0.5, timeout_s)
    last_ts = ts0
    last = scan0
    while time.monotonic() <= deadline:
        scan = _read_json(LIDAR_SCAN_PATH)
        ts = _safe_float(scan.get("ts"), math.nan)
        if _is_finite(ts):
            last = scan
            if ts > last_ts:
                increments += 1
                last_ts = ts
                if increments >= max(1, int(min_increments)):
                    return scan
        time.sleep(0.05)
    raise RuntimeError("Raw LIDAR scan timestamp is not progressing.")


def _resolve_runtime_lidar_stale_stop_s(status: Dict[str, Any]) -> float:
    st = dict(status or {})
    safety = dict(st.get("safety") or {})
    lidar_stale = dict(safety.get("lidar_stale") or {})
    threshold = _safe_float(
        lidar_stale.get("threshold_s", safety.get("lidar_stale_sec", DEFAULT_RUNTIME_STALE_STOP_S)),
        DEFAULT_RUNTIME_STALE_STOP_S,
    )
    if not _is_finite(threshold):
        return float(DEFAULT_RUNTIME_STALE_STOP_S)
    return max(0.10, float(threshold))


def _wait_for_start_gate(
    *,
    min_status_increments: int = DEFAULT_START_GATE_MIN_STATUS_INCREMENTS,
    min_scan_increments: int = DEFAULT_START_GATE_MIN_SCAN_INCREMENTS,
    min_fresh_scan_streak: int = DEFAULT_START_GATE_MIN_FRESH_SCAN_STREAK,
    stale_stop_s: float = DEFAULT_RUNTIME_STALE_STOP_S,
    freshness_reserve_s: float = DEFAULT_START_GATE_FRESHNESS_RESERVE_S,
    min_confidence: float = DEFAULT_START_GATE_MIN_CONFIDENCE,
    timeout_s: float = DEFAULT_START_GATE_TIMEOUT_S,
    poll_s: float = 0.05,
) -> Dict[str, Any]:
    st0 = _wait_for_status(timeout_s=max(0.5, timeout_s))
    scan0 = _read_json(LIDAR_SCAN_PATH)
    scan_ts = _safe_float(scan0.get("ts"), math.nan)
    if not _is_finite(scan_ts):
        raise RuntimeError("runtime/lidar_scan.json missing ts for start gate.")

    status_ver = _status_version(st0)
    status_increments = 0
    scan_increments = 0
    fresh_scan_streak = 0
    last_status = st0
    last_scan = scan0
    latest_diag: Dict[str, Any] = {}
    required_max_age_s = max(0.05, float(stale_stop_s) - max(0.0, float(freshness_reserve_s)))
    deadline = time.monotonic() + max(0.5, timeout_s)

    while time.monotonic() <= deadline:
        now_mono = time.monotonic()
        st = _read_json(STATUS_PATH)
        if st:
            last_status = st
            ver = _status_version(st)
            if ver > status_ver:
                status_increments += 1
                status_ver = ver

        scan = _read_json(LIDAR_SCAN_PATH)
        scan_progressed = False
        scan_ts_now = _safe_float(scan.get("ts"), math.nan)
        if _is_finite(scan_ts_now):
            last_scan = scan
            if scan_ts_now > scan_ts:
                scan_increments += 1
                scan_progressed = True
                scan_ts = scan_ts_now

        st_time = _safe_float(last_status.get("time"), math.nan)
        status_age_s = (now_mono - st_time) if _is_finite(st_time) else math.inf
        scan_age_s = (now_mono - float(scan_ts)) if _is_finite(scan_ts) else math.inf
        lidar_odom = dict(last_status.get("lidar_odom_status") or {})
        lidar_latest_age_s = _safe_float(lidar_odom.get("latest_age_s"), math.inf)
        lidar_latest_conf = _safe_float(lidar_odom.get("latest_confidence"), math.nan)
        lidar_latest_age_ok = _is_finite(lidar_latest_age_s) and float(lidar_latest_age_s) <= required_max_age_s
        confidence_ok = _is_finite(lidar_latest_conf) and float(lidar_latest_conf) >= float(min_confidence)
        status_age_ok = _is_finite(status_age_s) and float(status_age_s) <= required_max_age_s
        scan_age_ok = _is_finite(scan_age_s) and float(scan_age_s) <= required_max_age_s

        fresh_sample = bool(scan_progressed and status_age_ok and scan_age_ok and lidar_latest_age_ok and confidence_ok)
        if fresh_sample:
            fresh_scan_streak += 1
        elif scan_progressed:
            fresh_scan_streak = 0

        latest_diag = {
            "status_increments": int(status_increments),
            "scan_increments": int(scan_increments),
            "fresh_scan_streak": int(fresh_scan_streak),
            "required_status_increments": int(max(1, int(min_status_increments))),
            "required_scan_increments": int(max(1, int(min_scan_increments))),
            "required_fresh_scan_streak": int(max(1, int(min_fresh_scan_streak))),
            "status_age_s": round(float(status_age_s), 4) if _is_finite(status_age_s) else None,
            "scan_age_s": round(float(scan_age_s), 4) if _is_finite(scan_age_s) else None,
            "lidar_latest_age_s": round(float(lidar_latest_age_s), 4) if _is_finite(lidar_latest_age_s) else None,
            "lidar_latest_confidence": round(float(lidar_latest_conf), 4) if _is_finite(lidar_latest_conf) else None,
            "stale_stop_s": round(float(stale_stop_s), 4),
            "freshness_reserve_s": round(float(freshness_reserve_s), 4),
            "required_max_age_s": round(float(required_max_age_s), 4),
            "status_age_ok": bool(status_age_ok),
            "scan_age_ok": bool(scan_age_ok),
            "lidar_latest_age_ok": bool(lidar_latest_age_ok),
            "confidence_ok": bool(confidence_ok),
        }

        if (
            status_increments >= max(1, int(min_status_increments))
            and scan_increments >= max(1, int(min_scan_increments))
            and fresh_scan_streak >= max(1, int(min_fresh_scan_streak))
        ):
            return {
                "status": dict(last_status),
                "lidar_scan": dict(last_scan),
                "diagnostics": dict(latest_diag),
            }

        time.sleep(max(0.01, poll_s))

    raise RuntimeError(
        "Start gate failed: "
        f"status_increments={latest_diag.get('status_increments', 0)} "
        f"scan_increments={latest_diag.get('scan_increments', 0)} "
        f"fresh_scan_streak={latest_diag.get('fresh_scan_streak', 0)} "
        f"required_max_age_s={latest_diag.get('required_max_age_s')} "
        f"status_age_s={latest_diag.get('status_age_s')} "
        f"scan_age_s={latest_diag.get('scan_age_s')} "
        f"lidar_latest_age_s={latest_diag.get('lidar_latest_age_s')} "
        f"lidar_latest_confidence={latest_diag.get('lidar_latest_confidence')}"
    )


def _get_pose(status: Dict[str, Any]) -> Dict[str, float]:
    pose = status.get("pose", {}) or {}
    return {
        "x": float(_safe_float(pose.get("x"), 0.0)),
        "y": float(_safe_float(pose.get("y"), 0.0)),
        "theta": float(_safe_float(pose.get("theta"), 0.0)),
        "theta_deg": float(_safe_float(pose.get("theta_deg"), math.degrees(_safe_float(pose.get("theta"), 0.0)))),
        "v": float(_safe_float(pose.get("v"), 0.0)),
    }


def _pose_distance(a: Dict[str, float], b: Dict[str, float]) -> float:
    return math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))


def _normalize_angle_rad(rad: float) -> float:
    out = float(rad)
    while out >= math.pi:
        out -= 2.0 * math.pi
    while out < -math.pi:
        out += 2.0 * math.pi
    return out


def _normalize_angle_deg(deg: float) -> float:
    out = float(deg)
    while out >= 180.0:
        out -= 360.0
    while out < -180.0:
        out += 360.0
    return out


def _is_stopped(status: Dict[str, Any], *, pwm_eps: float = 0.03, vel_eps: float = 0.03) -> bool:
    pwm = status.get("pwm", {}) or {}
    pwm_l = abs(_safe_float(pwm.get("left"), 0.0))
    pwm_r = abs(_safe_float(pwm.get("right"), 0.0))
    v_l = abs(_safe_float(status.get("v_l_raw"), 0.0))
    v_r = abs(_safe_float(status.get("v_r_raw"), 0.0))
    state = str(status.get("state", "")).strip().upper()
    return state in ("IDLE", "FAILSAFE", "CALIBRATING") and pwm_l <= pwm_eps and pwm_r <= pwm_eps and v_l <= vel_eps and v_r <= vel_eps


def _wait_until_stopped(*, timeout_s: float, poll_s: float = DEFAULT_POLL_S, stale_status_s: float = DEFAULT_STATUS_STALE_S) -> Dict[str, Any]:
    deadline = time.monotonic() + max(0.1, timeout_s)
    last_ver = -1
    last_change = time.monotonic()
    last_status: Dict[str, Any] = {}
    while time.monotonic() <= deadline:
        st = _read_json(STATUS_PATH)
        if st:
            last_status = st
            ver = _status_version(st)
            if ver != last_ver:
                last_ver = ver
                last_change = time.monotonic()
            if _is_stopped(st):
                return st
        if time.monotonic() - last_change > stale_status_s:
            raise RuntimeError("Status stream stale while waiting for stop.")
        time.sleep(max(0.01, poll_s))
    raise TimeoutError(f"Robot did not stop within {timeout_s:.1f}s.")


def _ensure_idle_and_stopped(token: str, *, timeout_s: float) -> Dict[str, Any]:
    stop_cmd = _send_command_checked(
        "set_twist",
        token=token,
        timeout_s=4.0,
        v=0.0,
        omega=0.0,
        motion_source=DEFAULT_MOTION_SOURCE,
    )
    st = _wait_until_stopped(timeout_s=timeout_s)
    state = str(st.get("state", "")).strip().upper()
    if state != "IDLE":
        raise RuntimeError(f"Robot state is not IDLE after zero-twist stop: {state}")
    return {"status": st, "stop_cmd": stop_cmd}


def _safe_stop_best_effort(token: str) -> None:
    try:
        _append_command(
            "set_twist",
            token=token,
            v=0.0,
            omega=0.0,
            motion_source=DEFAULT_MOTION_SOURCE,
        )
    except Exception:
        pass
    try:
        _wait_until_stopped(timeout_s=4.0)
    except Exception:
        pass


def _sample_forward_clearance(sample_s: float = 1.2, poll_s: float = 0.05) -> Dict[str, Any]:
    deadline = time.monotonic() + max(0.2, sample_s)
    min_dists: List[float] = []
    clearance_sources: Dict[str, int] = {}
    raw_safety_sources: Dict[str, int] = {}
    raw_safety_scan_ids = set()
    invalid_raw_safety_source_count = 0
    blocked_front_count = 0
    blocked_back_count = 0
    samples = 0
    while time.monotonic() <= deadline:
        st = _read_json(STATUS_PATH)
        lidar = dict(st.get("lidar") or {})
        raw_source = str(lidar.get("raw_safety_source", "") or "")
        raw_safety_sources[raw_source or "MISSING"] = (
            int(raw_safety_sources.get(raw_source or "MISSING", 0)) + 1
        )
        raw_scan_id = int(_safe_float(lidar.get("raw_safety_raw_scan_id"), 0))
        raw_source_valid = bool(
            raw_source == RAW_SAFETY_SOURCE and raw_scan_id > 0
        )
        if raw_source_valid:
            raw_safety_scan_ids.add(raw_scan_id)
        else:
            invalid_raw_safety_source_count += 1
        clearance_source = ""
        min_d = math.nan
        if raw_source_valid:
            for key in ("min_dist_narrow", "min_dist"):
                candidate = _safe_float(lidar.get(key), math.nan)
                if _is_finite(candidate):
                    min_d = float(candidate)
                    clearance_source = key
                    break
        if _is_finite(min_d):
            min_dists.append(float(min_d))
            clearance_sources[clearance_source] = (
                int(clearance_sources.get(clearance_source, 0)) + 1
            )
        if raw_source_valid and bool(lidar.get("blocked_front", False)):
            blocked_front_count += 1
        if raw_source_valid and bool(lidar.get("blocked_back", False)):
            blocked_back_count += 1
        samples += 1
        time.sleep(max(0.01, poll_s))
    return {
        "samples": int(samples),
        "blocked_front_count": int(blocked_front_count),
        "blocked_back_count": int(blocked_back_count),
        "min_dist_min_m": (float(min(min_dists)) if min_dists else None),
        "min_dist_median_m": (float(statistics.median(min_dists)) if min_dists else None),
        "clearance_sources": dict(sorted(clearance_sources.items())),
        "raw_safety_sources": dict(sorted(raw_safety_sources.items())),
        "raw_safety_unique_scan_ids": len(raw_safety_scan_ids),
        "invalid_raw_safety_source_count": int(
            invalid_raw_safety_source_count
        ),
    }


def _straight_corridor_clearance_from_scan(
    scan_data: Any,
    *,
    half_width_m: float = DEFAULT_STRAIGHT_CORRIDOR_HALF_WIDTH_M,
) -> Dict[str, Any]:
    """Return forward X clearance inside a rectangular straight swept path."""
    corridor_half_width_m = max(0.05, float(half_width_m))
    nearest_x_m: Optional[float] = None
    nearest: Dict[str, float] = {}
    valid_point_count = 0
    corridor_point_count = 0
    for point in scan_data if isinstance(scan_data, list) else ():
        try:
            angle_deg = float(point["angle"]) % 360.0
            range_m = float(point["dist"]) / 1000.0
        except (KeyError, TypeError, ValueError):
            continue
        if not math.isfinite(angle_deg) or not math.isfinite(range_m) or range_m <= 0.0:
            continue
        valid_point_count += 1
        angle_rad = math.radians(angle_deg)
        forward_x_m = range_m * math.cos(angle_rad)
        lateral_y_m = range_m * math.sin(angle_rad)
        if forward_x_m <= 0.0 or abs(lateral_y_m) > corridor_half_width_m:
            continue
        corridor_point_count += 1
        if nearest_x_m is None or forward_x_m < nearest_x_m:
            nearest_x_m = float(forward_x_m)
            nearest = {
                "angle_deg": float(angle_deg),
                "range_m": float(range_m),
                "forward_x_m": float(forward_x_m),
                "lateral_y_m": float(lateral_y_m),
            }
    return {
        "source": STRAIGHT_CORRIDOR_FULL_RAW_SOURCE,
        "corridor_half_width_m": float(corridor_half_width_m),
        "valid_point_count": int(valid_point_count),
        "corridor_point_count": int(corridor_point_count),
        "min_forward_x_m": (
            None if nearest_x_m is None else float(nearest_x_m)
        ),
        "nearest_point": dict(nearest),
    }


def _sample_forward_corridor_clearance(
    sample_s: float = 1.2,
    poll_s: float = 0.05,
    *,
    half_width_m: float = DEFAULT_STRAIGHT_CORRIDOR_HALF_WIDTH_M,
    acquisition_timeout_s: float = 3.0,
    freshness_s: float = DEFAULT_FULL_SCAN_FRESHNESS_S,
) -> Dict[str, Any]:
    """
    Sample fresh full raw scans and measure only the robot's straight swept path.

    The normal angular safety sectors remain untouched and active. This
    diagnostic is used only by straight-run preflight so lateral walls outside
    the conservative 0.60 m-wide corridor cannot masquerade as a forward wall.
    """
    initial_payload = _read_json(LIDAR_FULL_SCAN_PATH)
    initial_ts = _safe_float(initial_payload.get("ts"), -1.0)
    request_deadline_wall_s = time.time() + max(
        5.0,
        float(acquisition_timeout_s) + float(sample_s) + 2.0,
    )
    _write_json_atomic(
        LIDAR_SCAN_SUBSCRIBER_PATH,
        {
            "full_scan": True,
            "expires_ts": float(request_deadline_wall_s),
            "request_source": "STRAIGHT_CORRIDOR_PREFLIGHT",
        },
    )

    unique_scan_ts = set()
    per_scan_clearance_m: List[float] = []
    per_scan_evidence: List[Dict[str, Any]] = []
    invalid_full_scan_count = 0
    stale_full_scan_count = 0
    first_accepted_mono: Optional[float] = None
    deadline = time.monotonic() + max(
        1.0,
        float(acquisition_timeout_s) + float(sample_s),
    )
    try:
        while time.monotonic() <= deadline:
            payload = _read_json(LIDAR_FULL_SCAN_PATH)
            meta = dict(payload.get("meta") or {})
            scan_ts = _safe_float(payload.get("ts"), -1.0)
            scan = payload.get("scan")
            scan_age_s = time.monotonic() - scan_ts
            full_raw_valid = bool(
                str(meta.get("view", "")) == "full_raw"
                and str(meta.get("mode", "")).startswith("explicit_")
                and isinstance(scan, list)
                and len(scan) >= 10
                and scan_ts > initial_ts + 1e-9
            )
            if not full_raw_valid:
                invalid_full_scan_count += 1
            elif scan_age_s < -0.05 or scan_age_s > float(freshness_s):
                stale_full_scan_count += 1
            elif scan_ts not in unique_scan_ts:
                unique_scan_ts.add(float(scan_ts))
                evidence = _straight_corridor_clearance_from_scan(
                    scan,
                    half_width_m=float(half_width_m),
                )
                clearance_m = evidence.get("min_forward_x_m")
                if clearance_m is not None and _is_finite(clearance_m):
                    per_scan_clearance_m.append(float(clearance_m))
                    per_scan_evidence.append(
                        {
                            "scan_ts": float(scan_ts),
                            "scan_age_s": float(scan_age_s),
                            "total_points": int(len(scan)),
                            **dict(evidence),
                        }
                    )
                    if first_accepted_mono is None:
                        first_accepted_mono = time.monotonic()
            if (
                len(per_scan_clearance_m) >= 2
                and first_accepted_mono is not None
                and (time.monotonic() - first_accepted_mono) >= float(sample_s)
            ):
                break
            time.sleep(max(0.01, float(poll_s)))
    finally:
        _write_json_atomic(
            LIDAR_SCAN_SUBSCRIBER_PATH,
            {
                "full_scan": False,
                "expires_ts": float(time.time()),
                "request_source": "STRAIGHT_CORRIDOR_PREFLIGHT_COMPLETE",
            },
        )

    return {
        "samples": int(len(per_scan_clearance_m)),
        "source": STRAIGHT_CORRIDOR_FULL_RAW_SOURCE,
        "corridor_half_width_m": float(max(0.05, float(half_width_m))),
        "min_dist_min_m": (
            float(min(per_scan_clearance_m))
            if per_scan_clearance_m
            else None
        ),
        "min_dist_median_m": (
            float(statistics.median(per_scan_clearance_m))
            if per_scan_clearance_m
            else None
        ),
        "raw_full_unique_scan_count": int(len(unique_scan_ts)),
        "invalid_full_scan_count": int(invalid_full_scan_count),
        "stale_full_scan_count": int(stale_full_scan_count),
        "per_scan_evidence": list(per_scan_evidence),
    }


def _require_finite_pose(pose: Dict[str, float], label: str) -> None:
    for key in ("x", "y", "theta", "theta_deg", "v"):
        if not _is_finite(pose.get(key)):
            raise RuntimeError(f"{label} pose field is not finite: {key}={pose.get(key)}")


def _precheck(token: str, *, target_distance_m: float, required_clearance_m: float, stop_timeout_s: float) -> Dict[str, Any]:
    st0 = _wait_for_status_progress(min_increments=2, timeout_s=5.0)
    _wait_for_lidar_scan_progress(min_increments=2, timeout_s=3.0)

    if not bool((st0.get("startup") or {}).get("ready", False)):
        raise RuntimeError(f"Runtime startup is not READY: {st0.get('startup')}")

    state0 = str(st0.get("state", "")).strip().upper()
    reset_cmd: Optional[Dict[str, Any]] = None
    if state0 == "FAILSAFE":
        reset_cmd = _send_command_checked("strong_reset", token=token, timeout_s=12.0)
        time.sleep(0.3)
        _wait_for_status_progress(min_increments=2, timeout_s=4.0)

    idle = _ensure_idle_and_stopped(token, timeout_s=stop_timeout_s)
    _send_command_checked("reset_pos", token=token, timeout_s=4.0)
    time.sleep(0.2)
    st = _wait_for_status(timeout_s=2.0)

    odometry_mode = str(st.get("odometry_mode", "")).strip().upper()
    if odometry_mode != "LIDAR_FIRST":
        raise RuntimeError(f"odometry_mode is {odometry_mode}, expected LIDAR_FIRST.")

    pose = dict(st.get("pose") or {})
    encoder_pose_fusion_active = bool(st.get("encoder_pose_fusion_active", False))
    pose_encoder_enabled = bool(pose.get("encoder_enabled", False))
    encoder_trust_mode = str(pose.get("encoder_trust_mode", "")).strip().upper()

    if not bool(st.get("lidar_enabled", False)):
        raise RuntimeError("LIDAR is not enabled.")
    if str(st.get("lidar_health", "")).strip().upper() != "OK":
        raise RuntimeError(f"LIDAR health is not OK: {st.get('lidar_health')}")

    current_pose = _read_json(CURRENT_POSE_PATH)
    current_pose_ts = _safe_float(current_pose.get("ts"), math.nan)
    if not _is_finite(current_pose_ts):
        raise RuntimeError("runtime/current_pose.json is missing or stale.")

    log_switches = load_log_switches()
    if not bool(log_switches.get("telemetry", False)):
        raise RuntimeError("Telemetry logging is disabled in logs/latest/runtime/log_switches.json.")

    clearance = _sample_forward_clearance(sample_s=1.2, poll_s=0.05)
    median_front_m = clearance.get("min_dist_median_m")
    if (
        clearance["invalid_raw_safety_source_count"] > 0
        or clearance["raw_safety_unique_scan_ids"] < 2
    ):
        raise RuntimeError(
            "Forward clearance is not backed by fresh parent raw-scan safety "
            f"summaries: {clearance}"
        )
    if clearance["samples"] <= 0 or median_front_m is None:
        raise RuntimeError("Forward clearance could not be verified from LIDAR.")
    if median_front_m < float(required_clearance_m):
        raise RuntimeError(
            f"Forward clearance too small for ~{target_distance_m:.2f} m run: "
            f"median front min_dist={median_front_m:.3f} m < required {required_clearance_m:.3f} m."
        )
    if clearance["blocked_front_count"] > max(1, int(0.2 * clearance["samples"])):
        raise RuntimeError(
            f"Forward path not consistently clear: blocked_front_count={clearance['blocked_front_count']} "
            f"across {clearance['samples']} samples."
        )

    lidar_scan = _read_json(LIDAR_SCAN_PATH)
    if not _is_finite(_safe_float(lidar_scan.get("ts"), math.nan)):
        raise RuntimeError("runtime/lidar_scan.json missing valid timestamp.")
    if not bool(lidar_scan.get("scan")):
        raise RuntimeError("runtime/lidar_scan.json does not contain scan points.")

    runtime_lidar_stale_stop_s = _resolve_runtime_lidar_stale_stop_s(st)
    freshness_reserve_s = min(
        max(0.05, runtime_lidar_stale_stop_s - 0.05),
        DEFAULT_START_GATE_FRESHNESS_RESERVE_S,
    )
    start_gate = _wait_for_start_gate(
        min_status_increments=DEFAULT_START_GATE_MIN_STATUS_INCREMENTS,
        min_scan_increments=DEFAULT_START_GATE_MIN_SCAN_INCREMENTS,
        min_fresh_scan_streak=DEFAULT_START_GATE_MIN_FRESH_SCAN_STREAK,
        stale_stop_s=runtime_lidar_stale_stop_s,
        freshness_reserve_s=freshness_reserve_s,
        min_confidence=DEFAULT_START_GATE_MIN_CONFIDENCE,
        timeout_s=DEFAULT_START_GATE_TIMEOUT_S,
    )
    gate_status = dict(start_gate.get("status") or {})
    gate_scan = dict(start_gate.get("lidar_scan") or {})
    gate_diag = dict(start_gate.get("diagnostics") or {})

    if str(gate_status.get("odometry_mode", "")).strip().upper() != "LIDAR_FIRST":
        raise RuntimeError(f"odometry_mode changed before run start gate: {gate_status.get('odometry_mode')}")
    if str(gate_status.get("lidar_health", "")).strip().upper() != "OK":
        raise RuntimeError(f"LIDAR health is not OK at run start gate: {gate_status.get('lidar_health')}")
    if not bool((gate_status.get("safety") or {}).get("allow", True)):
        raise RuntimeError(f"Safety is blocking motion before start gate: {gate_status.get('safety')}")
    if not _is_finite(_safe_float(gate_scan.get("ts"), math.nan)):
        raise RuntimeError("Start gate finished without a valid lidar scan timestamp.")

    return {
        "status_version": int(_safe_float(gate_status.get("status_version", st.get("status_version", -1)), -1)),
        "startup": dict(gate_status.get("startup") or st.get("startup") or {}),
        "state_before": state0,
        "state_after": str(gate_status.get("state", st.get("state", ""))),
        "odometry_mode": odometry_mode,
        "reset_cmd": reset_cmd,
        "idle_stop_cmd": idle["stop_cmd"],
        "forward_clearance": clearance,
        "lidar_health": str(gate_status.get("lidar_health", st.get("lidar_health", ""))),
        "lidar_scan_ts": float(_safe_float(gate_scan.get("ts", lidar_scan.get("ts")), 0.0)),
        "current_pose_ts": float(current_pose_ts),
        "encoder_pose_fusion_active": bool(encoder_pose_fusion_active),
        "encoder_pose_enabled": pose_encoder_enabled,
        "encoder_trust_mode": encoder_trust_mode,
        "motion_command_source": str(gate_status.get("motion_command_source", st.get("motion_command_source", ""))),
        "runtime_preset": str(gate_status.get("runtime_preset", st.get("runtime_preset", ""))),
        "control_mode": str(gate_status.get("control_mode", st.get("control_mode", ""))),
        "telemetry_enabled": bool(log_switches.get("telemetry", False)),
        "runtime_lidar_stale_stop_s": float(runtime_lidar_stale_stop_s),
        "start_gate": {
            "status_version": int(_safe_float(gate_status.get("status_version"), -1)),
            "diagnostics": dict(gate_diag),
        },
    }


def _run_trial(
    trial: int,
    *,
    trial_label: str | None = None,
    token: str,
    target_distance_m: float,
    v_mps: float,
    move_timeout_s: float,
    stop_timeout_s: float,
    keepalive_interval_s: float,
    required_clearance_m: float,
    heading_abort_deg: float,
) -> Dict[str, Any]:
    trial_number = int(trial)
    trial_name = str(trial_label or trial_number)
    precheck = _precheck(
        token,
        target_distance_m=target_distance_m,
        required_clearance_m=required_clearance_m,
        stop_timeout_s=stop_timeout_s,
    )

    st_start = _wait_for_status(timeout_s=2.0)
    start_pose = _get_pose(st_start)
    _require_finite_pose(start_pose, "start")
    start_lidar_odom = dict(st_start.get("lidar_odom_status") or {})

    print(
        f"Trial {trial_name}: start pose x={start_pose['x']:.4f}, y={start_pose['y']:.4f}, "
        f"theta={start_pose['theta']:.4f} rad ({start_pose['theta_deg']:.2f} deg)"
    )
    print(
        f"Trial {trial_name}: commanding straight motion via set_twist keepalive "
        f"(v={v_mps:.3f} m/s, omega=0.000 rad/s)"
    )

    motion_started = False
    start_cmd = _send_command_checked(
        "set_twist",
        token=token,
        timeout_s=4.0,
        v=float(v_mps),
        omega=0.0,
        motion_source=DEFAULT_MOTION_SOURCE,
    )
    motion_started = True

    last_status_ver = _status_version(st_start)
    last_status_change = time.monotonic()
    last_scan = _read_json(LIDAR_SCAN_PATH)
    last_scan_ts = _safe_float(last_scan.get("ts"), math.nan)
    if not _is_finite(last_scan_ts):
        last_scan_ts = 0.0
    last_scan_change = time.monotonic()
    last_keepalive = time.monotonic()
    deadline = time.monotonic() + max(0.1, move_timeout_s)
    last_progress_print = time.monotonic()
    blocked_front_consecutive = 0

    status_samples = 0
    applied_samples = 0
    lidar_ekf_applied_samples = 0
    lidar_localization_healthy_samples = 0
    lidar_missing_streak = 0
    lidar_missing_streak_max = 0
    lidar_delivery_counts: Counter[str] = Counter()
    lidar_missing_streak_limit = int(max(12, round(1.20 / max(0.01, DEFAULT_POLL_S))))
    lidar_stale_age_gate_s = float(max(0.25, min(0.60, DEFAULT_LIDAR_STALE_S * 0.35)))
    lidar_confidence_gate = 0.25
    lidar_conf_samples: List[float] = []
    lidar_latest_age_samples: List[float] = []
    lidar_observation_rows: List[Dict[str, Any]] = []
    lidar_health_counts: Counter[str] = Counter()
    state_counts: Counter[str] = Counter()
    primitive_chain_samples = 0
    primitive_chain_mismatch_samples = 0
    primitive_chain_mismatch_consecutive = 0
    primitive_chain_mismatch_max_consecutive = 0
    primitive_chain_req_lim_matches = 0
    primitive_chain_lim_exe_matches = 0
    primitive_chain_req_exe_matches = 0
    primitive_chain_exe_act_matches = 0
    primitive_chain_first_mismatch: Dict[str, Any] = {}
    guidance_heading_hold_active_samples = 0
    guidance_heading_hold_peak_heading_error_deg = 0.0
    guidance_heading_hold_peak_correction_rad_s = 0.0
    guidance_heading_hold_last_diag: Dict[str, Any] = {}
    runtime_notes: List[str] = []
    move_diag: Dict[str, Any] = {}
    end_status: Dict[str, Any] = {}
    move_error: Optional[str] = None
    max_estimated_distance_m = 0.0
    peak_pose = dict(start_pose)

    try:
        while time.monotonic() <= deadline:
            now_mono = time.monotonic()
            if (now_mono - last_keepalive) >= max(0.05, keepalive_interval_s):
                _append_command(
                    "set_twist",
                    token=token,
                    v=float(v_mps),
                    omega=0.0,
                    motion_source=DEFAULT_MOTION_SOURCE,
                )
                last_keepalive = now_mono

            st = _read_json(STATUS_PATH)
            if st:
                end_status = st
                status_samples += 1
                ver = _status_version(st)
                if ver != last_status_ver:
                    last_status_ver = ver
                    last_status_change = now_mono

                state = str(st.get("state", "")).strip().upper()
                state_counts[state] += 1
                if state == "FAILSAFE":
                    raise RuntimeError(f"Trial entered FAILSAFE: {st.get('last_emergency')}")
                if not bool((st.get("safety") or {}).get("allow", True)):
                    raise RuntimeError(f"Safety block active during trial: {st.get('safety')}")
                if str(st.get("odometry_mode", "")).strip().upper() != "LIDAR_FIRST":
                    raise RuntimeError(f"odometry_mode changed during trial: {st.get('odometry_mode')}")

                pose = _get_pose(st)
                _require_finite_pose(pose, "live")
                estimated_distance_m = _pose_distance(start_pose, pose)
                if estimated_distance_m > max_estimated_distance_m:
                    max_estimated_distance_m = float(estimated_distance_m)
                    peak_pose = dict(pose)
                heading_drift_deg = _normalize_angle_deg(pose["theta_deg"] - start_pose["theta_deg"])

                lidar_health = str(st.get("lidar_health", "")).strip().upper() or "UNKNOWN"
                lidar_health_counts[lidar_health] += 1

                lidar = dict(st.get("lidar") or {})
                if bool(lidar.get("blocked_front", False)):
                    blocked_front_consecutive += 1
                else:
                    blocked_front_consecutive = 0
                if blocked_front_consecutive >= 3 and estimated_distance_m < (0.95 * target_distance_m):
                    raise RuntimeError(
                        f"Forward path became blocked during trial: min_dist={_safe_float(lidar.get('min_dist'), math.nan):.3f} m "
                        f"(consecutive_samples={blocked_front_consecutive})"
                    )

                lidar_odom = dict(st.get("lidar_odom_status") or {})
                delivery_status = str(lidar_odom.get("delivery_status", "") or "").strip().lower()
                if delivery_status:
                    lidar_delivery_counts[delivery_status] += 1
                if bool(lidar_odom.get("applied", False)):
                    applied_samples += 1
                    lidar_ekf_applied_samples += 1
                latest_age_s = _safe_float(lidar_odom.get("latest_age_s"), math.nan)
                if _is_finite(latest_age_s):
                    lidar_latest_age_samples.append(float(latest_age_s))
                conf = _safe_float(lidar_odom.get("latest_confidence"), math.nan)
                if _is_finite(conf):
                    lidar_conf_samples.append(float(conf))
                if delivery_status in ("missing", "stale"):
                    lidar_missing_streak += 1
                    lidar_missing_streak_max = max(lidar_missing_streak_max, lidar_missing_streak)
                else:
                    lidar_missing_streak = 0
                localization_status = str(lidar_odom.get("localization_status", "") or "").strip().lower()
                localization_mode_ok = (not localization_status) or (
                    localization_status in ("tracking", "localized", "relocalized")
                )
                age_ok = _is_finite(latest_age_s) and float(latest_age_s) <= float(lidar_stale_age_gate_s)
                conf_ok = _is_finite(conf) and float(conf) >= float(lidar_confidence_gate)
                localization_healthy = bool(localization_mode_ok and age_ok and conf_ok)
                if localization_healthy:
                    lidar_localization_healthy_samples += 1
                truth_surface_sample = _extract_truth_basis(st)
                lidar_observation_rows.append(
                    {
                        "lidar_observation": dict(
                            truth_surface_sample.get("lidar_observation") or {}
                        ),
                        "lidar_odom_applied": bool(lidar_odom.get("applied", False)),
                        "lidar_odom_latest_age_s": (
                            float(latest_age_s) if _is_finite(latest_age_s) else None
                        ),
                        "lidar_odom_latest_confidence": (
                            float(conf) if _is_finite(conf) else None
                        ),
                        "lidar_localization_healthy": bool(localization_healthy),
                    }
                )

                turn_primitives = _canonical_turn_primitives(st)
                primitive_chain_samples += 1
                if turn_primitives.get("requested") == turn_primitives.get("limited"):
                    primitive_chain_req_lim_matches += 1
                if turn_primitives.get("limited") == turn_primitives.get("executed"):
                    primitive_chain_lim_exe_matches += 1
                if turn_primitives.get("requested") == turn_primitives.get("executed"):
                    primitive_chain_req_exe_matches += 1
                if turn_primitives.get("executed") == turn_primitives.get("actual"):
                    primitive_chain_exe_act_matches += 1
                straight_chain_ok = bool(
                    turn_primitives.get("requested") == "STRAIGHT"
                    and turn_primitives.get("limited") == "STRAIGHT"
                    and turn_primitives.get("executed") == "STRAIGHT"
                )
                if not straight_chain_ok:
                    primitive_chain_mismatch_samples += 1
                    primitive_chain_mismatch_consecutive += 1
                    primitive_chain_mismatch_max_consecutive = max(
                        int(primitive_chain_mismatch_max_consecutive),
                        int(primitive_chain_mismatch_consecutive),
                    )
                    if not primitive_chain_first_mismatch:
                        sem_status = dict(st.get("motion_semantics") or {})
                        motion_cmd = dict(st.get("motion_command") or {})
                        primitive_chain_first_mismatch = {
                            "requested": str(turn_primitives.get("requested", "")),
                            "limited": str(turn_primitives.get("limited", "")),
                            "executed": str(turn_primitives.get("executed", "")),
                            "actual": str(turn_primitives.get("actual", "")),
                            "estimated_distance_m": float(estimated_distance_m),
                            "heading_drift_deg": float(heading_drift_deg),
                            "motion_source": str(st.get("motion_command_source", "")),
                            "command_type": str(
                                motion_cmd.get("command_type", st.get("active_motion_type", ""))
                                or ""
                            ),
                            "guidance_heading_owner": str(
                                sem_status.get("heading_hold_owner", "") or ""
                            ),
                            "guidance_heading_hold_active": bool(
                                sem_status.get("heading_hold_applied", False)
                            ),
                        }
                    if primitive_chain_mismatch_consecutive >= 3:
                        raise RuntimeError(
                            "primitive_chain_mismatch: "
                            f"requested={turn_primitives.get('requested')} "
                            f"limited={turn_primitives.get('limited')} "
                            f"executed={turn_primitives.get('executed')} "
                            f"actual={turn_primitives.get('actual')}"
                        )
                else:
                    primitive_chain_mismatch_consecutive = 0

                sem_status = dict(st.get("motion_semantics") or {})
                guidance_heading_hold_live = {
                    "active": bool(sem_status.get("heading_hold_applied", False)),
                    "owner": str(sem_status.get("heading_hold_owner", "") or ""),
                    "mode": str(sem_status.get("heading_hold_mode", "") or ""),
                    "heading_error_deg": sem_status.get("heading_error_deg"),
                    "omega_correction_rad_s": (
                        sem_status.get("omega_target", 0.0)
                        if bool(sem_status.get("heading_hold_applied", False))
                        else 0.0
                    ),
                }
                guidance_heading_hold_last_diag = dict(guidance_heading_hold_live)
                if bool(guidance_heading_hold_live.get("active", False)):
                    guidance_heading_hold_active_samples += 1
                heading_error_live = _safe_float(
                    guidance_heading_hold_live.get("heading_error_deg"), math.nan
                )
                if _is_finite(heading_error_live):
                    guidance_heading_hold_peak_heading_error_deg = max(
                        float(guidance_heading_hold_peak_heading_error_deg),
                        abs(float(heading_error_live)),
                    )
                correction_live = _safe_float(
                    guidance_heading_hold_live.get("omega_correction_rad_s"), math.nan
                )
                if _is_finite(correction_live):
                    guidance_heading_hold_peak_correction_rad_s = max(
                        float(guidance_heading_hold_peak_correction_rad_s),
                        abs(float(correction_live)),
                    )

                if abs(float(heading_drift_deg)) > float(heading_abort_deg):
                    raise RuntimeError(
                        f"Severe heading drift detected: {heading_drift_deg:.2f} deg > {heading_abort_deg:.2f} deg."
                    )

                if estimated_distance_m >= float(target_distance_m):
                    move_diag = {
                        "status_samples": int(status_samples),
                        "applied_samples": int(applied_samples),
                        "state_counts": dict(state_counts),
                    }
                    print(f"Trial {trial_name}: target reached at EKF distance {estimated_distance_m:.4f} m")
                    break

                if (now_mono - last_progress_print) >= 1.0:
                    print(
                        f"  progress: dist={estimated_distance_m:.3f}/{target_distance_m:.3f} m, "
                        f"heading_drift={heading_drift_deg:.2f} deg"
                    )
                    last_progress_print = now_mono

            scan = _read_json(LIDAR_SCAN_PATH)
            scan_ts = _safe_float(scan.get("ts"), math.nan)
            if _is_finite(scan_ts):
                if scan_ts > last_scan_ts:
                    last_scan_ts = scan_ts
                    last_scan_change = now_mono
            if (now_mono - last_status_change) > DEFAULT_STATUS_STALE_S:
                raise RuntimeError("Status stream became stale during motion.")
            if (now_mono - last_scan_change) > DEFAULT_LIDAR_STALE_S:
                raise RuntimeError("Raw LIDAR scan stream became stale during motion.")

            time.sleep(max(0.01, DEFAULT_POLL_S))
        else:
            raise TimeoutError(f"Target EKF distance not reached within {move_timeout_s:.1f}s.")
    except Exception as exc:
        move_error = str(exc)
    finally:
        try:
            stop_cmd = _send_command_checked(
                "set_twist",
                token=token,
                timeout_s=4.0,
                v=0.0,
                omega=0.0,
                motion_source=DEFAULT_MOTION_SOURCE,
            )
        except Exception as exc:
            stop_cmd = {"error": str(exc)}
            _safe_stop_best_effort(token)
        try:
            st_stop = _wait_until_stopped(timeout_s=stop_timeout_s)
        except Exception as exc:
            runtime_notes.append(f"stop_wait_error={exc}")
            st_stop = _read_json(STATUS_PATH)

    end_status = st_stop if st_stop else end_status
    end_pose = _get_pose(end_status)
    _require_finite_pose(end_pose, "end")
    truth_surface = _extract_truth_basis(end_status)
    estimated_distance_m = _pose_distance(start_pose, end_pose)
    heading_drift_deg = _normalize_angle_deg(end_pose["theta_deg"] - start_pose["theta_deg"])
    end_lidar_odom = dict(end_status.get("lidar_odom_status") or {})
    final_state = str(end_status.get("state", ""))
    stop_ok = bool(_is_stopped(end_status) and final_state.strip().upper() == "IDLE")
    if move_error is None and not stop_ok:
        move_error = f"Stop condition failed after motion: state={final_state}"
    if move_error is None and (max_estimated_distance_m - estimated_distance_m) > 0.25 and estimated_distance_m < (0.8 * max_estimated_distance_m):
        move_error = (
            "Pose collapsed after stop: "
            f"peak_distance_m={max_estimated_distance_m:.3f}, end_distance_m={estimated_distance_m:.3f}"
        )

    lidar_observation_summary = _summarize_lidar_observation_rows(lidar_observation_rows)
    applied_samples = int(lidar_observation_summary.get("applied_samples", 0))
    lidar_ekf_applied_samples = int(applied_samples)
    lidar_conf_samples = list(lidar_observation_summary.get("confidence_values") or [])
    lidar_latest_age_samples = list(lidar_observation_summary.get("age_values") or [])
    lidar_localization_healthy_observations = int(
        lidar_observation_summary.get("localization_healthy_samples", 0)
    )
    lidar_conf_summary = {}
    if lidar_conf_samples:
        lidar_conf_summary = {
            "samples": int(len(lidar_conf_samples)),
            "min": float(min(lidar_conf_samples)),
            "mean": float(sum(lidar_conf_samples) / len(lidar_conf_samples)),
            "max": float(max(lidar_conf_samples)),
            "latest": float(lidar_conf_samples[-1]),
        }
    lidar_age_summary = {}
    if lidar_latest_age_samples:
        lidar_age_summary = {
            "samples": int(len(lidar_latest_age_samples)),
            "min": float(min(lidar_latest_age_samples)),
            "median": float(statistics.median(lidar_latest_age_samples)),
            "max": float(max(lidar_latest_age_samples)),
            "latest": float(lidar_latest_age_samples[-1]),
        }
    guidance_heading_hold_summary = {
        "owner": "MOTION_GUIDANCE_L7A",
        "active_samples": int(guidance_heading_hold_active_samples),
        "status_samples": int(status_samples),
        "active_ratio": (
            float(guidance_heading_hold_active_samples / status_samples)
            if status_samples > 0
            else 0.0
        ),
        "peak_heading_error_deg": float(guidance_heading_hold_peak_heading_error_deg),
        "peak_correction_rad_s": float(guidance_heading_hold_peak_correction_rad_s),
        "last": dict(guidance_heading_hold_last_diag),
    }
    primitive_chain_summary = {
        "checked_samples": int(primitive_chain_samples),
        "mismatch_samples": int(primitive_chain_mismatch_samples),
        "mismatch_max_consecutive": int(primitive_chain_mismatch_max_consecutive),
        "requested_vs_limited_match_ratio": (
            float(primitive_chain_req_lim_matches / primitive_chain_samples)
            if primitive_chain_samples > 0
            else None
        ),
        "limited_vs_executed_match_ratio": (
            float(primitive_chain_lim_exe_matches / primitive_chain_samples)
            if primitive_chain_samples > 0
            else None
        ),
        "requested_vs_executed_match_ratio": (
            float(primitive_chain_req_exe_matches / primitive_chain_samples)
            if primitive_chain_samples > 0
            else None
        ),
        "executed_vs_actual_match_ratio": (
            float(primitive_chain_exe_act_matches / primitive_chain_samples)
            if primitive_chain_samples > 0
            else None
        ),
        "first_mismatch": dict(primitive_chain_first_mismatch),
    }
    if not move_diag:
        move_diag = {
            "status_samples": int(status_samples),
            "applied_samples": int(applied_samples),
            "state_counts": dict(state_counts),
        }
    move_diag["applied_samples"] = int(applied_samples)
    move_diag["applied_status_samples"] = int(
        lidar_observation_summary.get("applied_status_samples", 0)
    )
    move_diag["lidar_observation_contract"] = {
        key: lidar_observation_summary.get(key)
        for key in (
            "unique_raw_scan_observations",
            "unique_matcher_result_observations",
            "unique_lidar_odometry_measurements",
            "applied_missing_measurement_id_samples",
            "observation_contract_errors",
        )
    }
    move_diag["primitive_chain"] = dict(primitive_chain_summary)
    move_diag["guidance_heading_hold"] = dict(guidance_heading_hold_summary)

    lidar_updates = {
        "start_total": int(_safe_float(start_lidar_odom.get("total"), 0)),
        "end_total": int(_safe_float(end_lidar_odom.get("total"), 0)),
        "delta_total": int(_safe_float(end_lidar_odom.get("total"), 0) - _safe_float(start_lidar_odom.get("total"), 0)),
        "start_accepted": int(_safe_float(start_lidar_odom.get("accepted"), 0)),
        "end_accepted": int(_safe_float(end_lidar_odom.get("accepted"), 0)),
        "delta_accepted": int(_safe_float(end_lidar_odom.get("accepted"), 0) - _safe_float(start_lidar_odom.get("accepted"), 0)),
        "start_rejected_low_confidence": int(_safe_float(start_lidar_odom.get("rejected_low_confidence"), 0)),
        "end_rejected_low_confidence": int(_safe_float(end_lidar_odom.get("rejected_low_confidence"), 0)),
        "delta_rejected_low_confidence": int(
            _safe_float(end_lidar_odom.get("rejected_low_confidence"), 0) - _safe_float(start_lidar_odom.get("rejected_low_confidence"), 0)
        ),
        "start_rejected_large_jump": int(_safe_float(start_lidar_odom.get("rejected_large_jump"), 0)),
        "end_rejected_large_jump": int(_safe_float(end_lidar_odom.get("rejected_large_jump"), 0)),
        "delta_rejected_large_jump": int(
            _safe_float(end_lidar_odom.get("rejected_large_jump"), 0) - _safe_float(start_lidar_odom.get("rejected_large_jump"), 0)
        ),
        "applied_samples": int(applied_samples),
        "applied_status_samples": int(
            lidar_observation_summary.get("applied_status_samples", 0)
        ),
        "ekf_applied_status_samples": int(
            lidar_observation_summary.get("applied_status_samples", 0)
        ),
        "applied_missing_measurement_id_samples": int(
            lidar_observation_summary.get("applied_missing_measurement_id_samples", 0)
        ),
        "unique_lidar_odometry_measurements": int(
            lidar_observation_summary.get("unique_lidar_odometry_measurements", 0)
        ),
        "unique_matcher_result_observations": int(
            lidar_observation_summary.get("unique_matcher_result_observations", 0)
        ),
        "unique_raw_scan_observations": int(
            lidar_observation_summary.get("unique_raw_scan_observations", 0)
        ),
        "observation_contract_errors": list(
            lidar_observation_summary.get("observation_contract_errors") or []
        ),
        "localization_healthy_status_samples": int(lidar_localization_healthy_samples),
        "localization_healthy_observations": int(lidar_localization_healthy_observations),
        "delivery_status_counts": dict(lidar_delivery_counts),
        "missing_streak_max": int(lidar_missing_streak_max),
        "missing_streak_limit": int(lidar_missing_streak_limit),
        "stale_age_gate_s": float(lidar_stale_age_gate_s),
        "confidence_gate": float(lidar_confidence_gate),
        "end_latest_age_s": float(_safe_float(end_lidar_odom.get("latest_age_s"), math.nan)),
        "end_latest_confidence": float(_safe_float(end_lidar_odom.get("latest_confidence"), math.nan)),
    }
    if int(status_samples) > 0:
        lidar_updates["applied_status_ratio"] = float(
            max(0.0, float(lidar_observation_summary.get("applied_status_samples", 0)))
            / float(status_samples)
        )
        lidar_updates["localization_healthy_status_ratio"] = float(
            max(0.0, float(lidar_localization_healthy_samples)) / float(status_samples)
        )
    if int(lidar_updates["delta_total"]) > 0:
        lidar_updates["update_rate_ratio"] = float(
            max(0.0, float(lidar_updates["delta_accepted"]))
            / float(lidar_updates["delta_total"])
        )
    unique_measurements = int(
        lidar_observation_summary.get("unique_lidar_odometry_measurements", 0)
    )
    if unique_measurements > 0:
        lidar_updates["applied_rate_ratio"] = float(
            max(0.0, float(lidar_ekf_applied_samples)) / float(unique_measurements)
        )
        lidar_updates["localization_healthy_ratio"] = float(
            max(0.0, float(lidar_localization_healthy_observations))
            / float(unique_measurements)
        )

    if move_error is None:
        if lidar_updates["delta_total"] <= 0:
            move_error = "No LiDAR odometry updates observed during trial."
        elif lidar_updates["delta_accepted"] <= 0:
            move_error = "No accepted LiDAR odometry updates observed during trial."
        elif int(lidar_updates["applied_missing_measurement_id_samples"]) > 0:
            move_error = "Applied LiDAR odometry sample is missing its measurement ID."
        elif list(lidar_updates["observation_contract_errors"]):
            move_error = (
                "LiDAR observation contract violation: "
                + ",".join(str(item) for item in lidar_updates["observation_contract_errors"])
            )

    if lidar_health_counts and any(key != "OK" for key in lidar_health_counts):
        runtime_notes.append(f"lidar_health_samples={dict(lidar_health_counts)}")

    truth_basis_payload = dict(truth_surface.get("truth_basis") or {})
    if primitive_chain_samples > 0:
        truth_basis_payload["turn_primitive_requested_vs_limited_match_ratio"] = float(
            primitive_chain_req_lim_matches / primitive_chain_samples
        )
        truth_basis_payload["turn_primitive_limited_vs_executed_match_ratio"] = float(
            primitive_chain_lim_exe_matches / primitive_chain_samples
        )
        truth_basis_payload["turn_primitive_requested_vs_executed_match_ratio"] = float(
            primitive_chain_req_exe_matches / primitive_chain_samples
        )
        truth_basis_payload["turn_primitive_chain_sample_count"] = int(primitive_chain_samples)
    truth_basis_payload["lidar_odom_status_samples"] = int(status_samples)
    truth_basis_payload["lidar_odom_applied_samples"] = int(lidar_ekf_applied_samples)
    truth_basis_payload["lidar_odom_applied_status_samples"] = int(
        lidar_observation_summary.get("applied_status_samples", 0)
    )
    truth_basis_payload["lidar_observation_contract_errors"] = list(
        lidar_observation_summary.get("observation_contract_errors") or []
    )
    truth_basis_payload["lidar_odom_applied_missing_measurement_id_samples"] = int(
        lidar_observation_summary.get("applied_missing_measurement_id_samples", 0)
    )
    truth_basis_payload["lidar_odom_unique_measurement_samples"] = int(unique_measurements)
    truth_basis_payload["lidar_odom_accepted_delta"] = int(lidar_updates.get("delta_accepted", 0))
    truth_basis_payload["lidar_odom_missing_streak_max"] = int(lidar_missing_streak_max)
    truth_basis_payload["lidar_odom_missing_streak_limit"] = int(lidar_missing_streak_limit)
    truth_basis_payload["lidar_odom_localization_healthy_status_samples"] = int(lidar_localization_healthy_samples)
    if "update_rate_ratio" in lidar_updates:
        truth_basis_payload["lidar_odom_update_rate_ratio"] = float(
            lidar_updates["update_rate_ratio"]
        )
    if unique_measurements > 0:
        truth_basis_payload["lidar_odom_applied_rate_ratio"] = float(
            max(0.0, float(lidar_ekf_applied_samples)) / float(unique_measurements)
        )
        truth_basis_payload["lidar_odom_localization_healthy_ratio"] = float(
            max(0.0, float(lidar_localization_healthy_observations))
            / float(unique_measurements)
        )

    result = {
        "success": move_error is None,
        "trial": int(trial_number),
        "trial_label": str(trial_name),
        "physically_executed": bool(motion_started),
        "command_path": f"set_twist via runtime/commands.jsonl ({DEFAULT_MOTION_SOURCE} keepalive, zero-twist stop)",
        "precheck": precheck,
        "command_lifecycle": {
            "start_set_twist": start_cmd,
            "stop_set_twist": stop_cmd,
        },
        "start_pose": start_pose,
        "end_pose": end_pose,
        "estimated_distance_m": float(estimated_distance_m),
        "peak_estimated_distance_m": float(max_estimated_distance_m),
        "start_theta": float(start_pose["theta"]),
        "end_theta": float(end_pose["theta"]),
        "heading_drift_deg": float(heading_drift_deg),
        "lidar_confidence_summary": lidar_conf_summary,
        "lidar_latest_age_summary": lidar_age_summary,
        "lidar_health_summary": dict(lidar_health_counts),
        "lidar_updates": lidar_updates,
        "primitive_chain": primitive_chain_summary,
        "guidance_heading_hold": guidance_heading_hold_summary,
        "runtime_notes": runtime_notes,
        "failure_reason": (str(move_error) if move_error is not None else ""),
        "stop_condition": {
            "state": final_state,
            "is_stopped": bool(_is_stopped(end_status)),
            "pwm_left": float(_safe_float((end_status.get("pwm") or {}).get("left"), 0.0)),
            "pwm_right": float(_safe_float((end_status.get("pwm") or {}).get("right"), 0.0)),
            "v_l_raw": float(_safe_float(end_status.get("v_l_raw"), 0.0)),
            "v_r_raw": float(_safe_float(end_status.get("v_r_raw"), 0.0)),
        },
        "final_state": final_state,
        "last_emergency": dict(end_status.get("last_emergency") or {}),
        "control_mode": str(end_status.get("control_mode", "")),
        "runtime_preset": str(end_status.get("runtime_preset", "")),
        "motion_command_source": str(end_status.get("motion_command_source", "")),
        "move_diagnostics": move_diag,
        "peak_pose": peak_pose,
        "motion_actual_ssot": str(truth_surface.get("motion_actual_ssot", "EKF_POSE_ODOMETRY_SSOT")),
        "truth_basis": dict(truth_basis_payload),
        "lidar_odom_status_truth": dict(truth_surface.get("lidar_odom_status") or {}),
        "lidar_odom_applied": bool(truth_surface.get("lidar_odom_applied", False)),
        "lidar_odom_latest_age_s": truth_surface.get("lidar_odom_latest_age_s"),
        "lidar_odom_latest_confidence": truth_surface.get("lidar_odom_latest_confidence"),
        "encoder_pose_active_samples": int(truth_surface.get("encoder_pose_active_samples", 0)),
        "execution_mode": str(truth_surface.get("execution_mode", "")),
        "turn_primitive_requested": str(truth_surface.get("turn_primitive_requested", "UNKNOWN")),
        "turn_primitive_limited": str(truth_surface.get("turn_primitive_limited", "UNKNOWN")),
        "turn_primitive_executed": str(truth_surface.get("turn_primitive_executed", "UNKNOWN")),
        "turn_primitive_actual": str(truth_surface.get("turn_primitive_actual", "UNKNOWN")),
        "turn_primitives": dict(truth_surface.get("turn_primitives") or {}),
    }
    return result


def main() -> int:
    ap = argparse.ArgumentParser(description="Run one straight-line LIDAR_FIRST validation trial.")
    ap.add_argument("--trial", default="1")
    ap.add_argument("--target-distance-m", type=float, default=DEFAULT_TARGET_DISTANCE_M)
    ap.add_argument("--v-mps", type=float, default=DEFAULT_V_MPS)
    ap.add_argument("--move-timeout-s", type=float, default=DEFAULT_MOVE_TIMEOUT_S)
    ap.add_argument("--stop-timeout-s", type=float, default=DEFAULT_STOP_TIMEOUT_S)
    ap.add_argument("--keepalive-interval-s", type=float, default=DEFAULT_KEEPALIVE_INTERVAL_S)
    ap.add_argument("--required-clearance-m", type=float, default=1.30)
    ap.add_argument("--heading-abort-deg", type=float, default=DEFAULT_HEADING_ABORT_DEG)
    ap.add_argument("--token", default=DEFAULT_TOKEN)
    args = ap.parse_args()

    trial_number, trial_label = _normalize_trial_identity(args.trial)
    print(f"--- LIDAR_FIRST straight-line validation: Trial {trial_label} ---")
    print(f"Status path: {STATUS_PATH}")
    print(f"Command path: {COMMANDS_PATH}")

    try:
        result = _run_trial(
            int(trial_number),
            trial_label=str(trial_label),
            token=str(args.token),
            target_distance_m=float(args.target_distance_m),
            v_mps=float(args.v_mps),
            move_timeout_s=float(args.move_timeout_s),
            stop_timeout_s=float(args.stop_timeout_s),
            keepalive_interval_s=float(args.keepalive_interval_s),
            required_clearance_m=float(args.required_clearance_m),
            heading_abort_deg=float(args.heading_abort_deg),
        )
        _append_jsonl(RAW_LOG_PATH, result)
        print("\n--- Trial Result ---")
        print(
            f"start=({result['start_pose']['x']:.4f}, {result['start_pose']['y']:.4f}, {result['start_pose']['theta']:.4f}) "
            f"end=({result['end_pose']['x']:.4f}, {result['end_pose']['y']:.4f}, {result['end_pose']['theta']:.4f}) "
            f"dist={result['estimated_distance_m']:.4f} m "
            f"heading_drift={result['heading_drift_deg']:.2f} deg"
        )
        print(f"JSON_RESULT: {json.dumps(result, ensure_ascii=False)}")
        return 0 if bool(result.get("success", False)) else 1
    except Exception as exc:
        _safe_stop_best_effort(str(args.token))
        status_now = _read_json(STATUS_PATH)
        truth_surface = _extract_truth_basis(status_now)
        failure = {
            "success": False,
            "trial": int(trial_number),
            "trial_label": str(trial_label),
            "physically_executed": False,
            "command_path": f"set_twist via runtime/commands.jsonl ({DEFAULT_MOTION_SOURCE} keepalive, zero-twist stop)",
            "failure_reason": str(exc),
            "status": status_now,
            "current_pose": _read_json(CURRENT_POSE_PATH),
            "lidar_scan": _read_json(LIDAR_SCAN_PATH),
            "motion_actual_ssot": str(truth_surface.get("motion_actual_ssot", "EKF_POSE_ODOMETRY_SSOT")),
            "truth_basis": dict(truth_surface.get("truth_basis") or {}),
            "lidar_odom_status_truth": dict(truth_surface.get("lidar_odom_status") or {}),
            "lidar_odom_applied": bool(truth_surface.get("lidar_odom_applied", False)),
            "lidar_odom_latest_age_s": truth_surface.get("lidar_odom_latest_age_s"),
            "lidar_odom_latest_confidence": truth_surface.get("lidar_odom_latest_confidence"),
            "encoder_pose_active_samples": int(truth_surface.get("encoder_pose_active_samples", 0)),
            "execution_mode": str(truth_surface.get("execution_mode", "")),
            "turn_primitive_requested": str(truth_surface.get("turn_primitive_requested", "UNKNOWN")),
            "turn_primitive_limited": str(truth_surface.get("turn_primitive_limited", "UNKNOWN")),
            "turn_primitive_executed": str(truth_surface.get("turn_primitive_executed", "UNKNOWN")),
            "turn_primitive_actual": str(truth_surface.get("turn_primitive_actual", "UNKNOWN")),
            "turn_primitives": dict(truth_surface.get("turn_primitives") or {}),
        }
        _append_jsonl(RAW_LOG_PATH, failure)
        print(f"\nFAILED: {exc}")
        print(f"JSON_RESULT: {json.dumps(failure, ensure_ascii=False)}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
