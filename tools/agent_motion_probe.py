#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Compact agent-facing live motion probe for short, bounded LIDAR_FIRST tests.

Goals:
- verify runtime readiness before motion
- validate normal stop without latching FAILSAFE
- validate emergency stop latches when explicitly requested
- run short bounded motion tests with compact JSON artifacts
- avoid verbose log scraping during repeated agent iterations
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402
from log.runtime_debug import load_log_switches  # noqa: E402

from project_rules.bootstrap_guard import BootstrapGuardError, ensure_agent_system_prompt_loaded  # noqa: E402
from tools.lidar_1m_step import (  # noqa: E402
    DEFAULT_KEEPALIVE_INTERVAL_S,
    DEFAULT_POLL_S,
    DEFAULT_STATUS_STALE_S,
    DEFAULT_TOKEN,
    LIDAR_SCAN_PATH,
    STATUS_PATH,
    COMMANDS_PATH,
    _append_command,
    _extract_truth_basis,
    _get_pose,
    _normalize_angle_deg,
    _pose_distance,
    _read_json,
    _safe_float,
    _safe_stop_best_effort,
    _sample_forward_clearance,
    _sample_forward_corridor_clearance,
    _send_command_checked,
    _status_version,
    _wait_for_lidar_scan_progress,
    _wait_for_status,
    _wait_for_status_progress,
    _wait_until_stopped,
)

RUNTIME_DIR = PROJECT_ROOT / "runtime"
AGENT_TEST_DIR = test_artifacts_dir()
LATEST_RESULT_PATH = AGENT_TEST_DIR / "latest_result.json"
LATEST_PREFLIGHT_PATH = AGENT_TEST_DIR / "latest_preflight.json"
LATEST_SUMMARY_PATH = AGENT_TEST_DIR / "latest_summary.json"
HISTORY_PATH = AGENT_TEST_DIR / "history.jsonl"

DEFAULT_FORWARD_SPEED_MPS = 0.10
DEFAULT_FORWARD_DISTANCE_M = 0.12
DEFAULT_FORWARD_MAX_RUNTIME_S = 2.5
DEFAULT_FORWARD_MIN_PROGRESS_M = 0.05
DEFAULT_FORWARD_MIN_PROGRESS_RATIO = 0.30
DEFAULT_FORWARD_TARGET_COMPLETION_RATIO = 0.0
DEFAULT_MIN_LIDAR_ACCEPT_COUNT = 0
DEFAULT_MAX_LIDAR_REJECT_COUNT = -1
DEFAULT_MIN_ODOM_VS_COMMAND_RATIO = 0.0
DEFAULT_FORWARD_CLEARANCE_M = 0.80
DEFAULT_FORWARD_CLEARANCE_MODE = "front-sector"
DEFAULT_FORWARD_HEADING_ABORT_DEG = 10.0
DEFAULT_TRACK_WIDTH_M = 0.185
DEFAULT_MIN_ROTATE_WHEEL_SPEED_MPS = 0.06
DEFAULT_STOP_TIMEOUT_S = 4.0
DEFAULT_STOP_SETTLE_HOLD_S = 0.20
DEFAULT_STOP_POSE_V_EPS = 0.05
DEFAULT_EMERGENCY_TIMEOUT_S = 2.0
DEFAULT_RESET_TIMEOUT_S = 8.0
DEFAULT_PREFLIGHT_MIN_SCAN_POINTS = 10
DEFAULT_PREFLIGHT_MIN_LIDAR_CONFIDENCE = 0.25
DEFAULT_PREFLIGHT_MAX_LIDAR_LATEST_AGE_S = 1.5
DEFAULT_PREFLIGHT_MAX_LIDAR_CANDIDATE_AGE_S = 0.5
DEFAULT_PREFLIGHT_STRICT_SCAN_WARMUP_TIMEOUT_S = 2.0
DEFAULT_PREFLIGHT_STRICT_SCAN_WARMUP_POLL_S = 0.10
DEFAULT_ARC_HEADING_ERROR_DEG = 15.0
DEFAULT_ARC_LENGTH_ERROR_M = 0.15
DEFAULT_ARC_LATERAL_DEVIATION_RATIO = 0.35
DEFAULT_ARC_LATERAL_DEVIATION_MIN_M = 0.05
DEFAULT_ARC_LATERAL_DEVIATION_MAX_M = 0.12
DEFAULT_ARC_EARLY_PROGRESS_MAX_FRAC = 0.25
DEFAULT_ARC_LATE_PROGRESS_MIN_FRAC = 0.75
DEFAULT_ARC_EARLY_YAW_MIN_ABS_RAD_S = 0.06
DEFAULT_ARC_EARLY_YAW_MIN_TARGET_RATIO = 0.25
DEFAULT_ARC_INNER_TRACK_POSITIVE_RATIO_MIN = 0.95
DEFAULT_ARC_OMEGA_TRACKING_ERROR_RMS_MAX_RAD_S = 0.30
DEFAULT_ARC_CURVATURE_ERROR_RMS_MAX_M_INV = 1.40
DEFAULT_AMR_PROGRESS_GUARD_MIN_TARGET_M = 0.5
DEFAULT_AMR_COMMAND_PROGRESS_GUARD_RATIO = 3.0
DEFAULT_AMR_ENCODER_PROGRESS_GUARD_RATIO = 3.0
DEFAULT_STRAIGHT_SEGMENT_HEADING_DEG_PER_M = 5.0
DEFAULT_STRAIGHT_SEGMENT_AVG_ANGULAR_DPS = 5.0
DEFAULT_POSE_TARGET_HEADING_TOLERANCE_DEG = 6.0
DEFAULT_TERMINAL_GUIDANCE_MIN_TARGET_M = 0.80
DEFAULT_TERMINAL_GUIDANCE_SPEED_SCALE = 0.45
DEFAULT_TERMINAL_GUIDANCE_MIN_SPEED_MPS = 0.05
DEFAULT_TERMINAL_GUIDANCE_BRAKE_DISTANCE_M = 0.22
DEFAULT_TERMINAL_GUIDANCE_STOP_BUFFER_M = 0.05
DEFAULT_HEADING_HOLD_MIN_TARGET_M = 0.80
DEFAULT_HEADING_HOLD_DEADBAND_DEG = 0.8
DEFAULT_HEADING_HOLD_KP_OMEGA = 1.0
DEFAULT_HEADING_HOLD_MAX_OMEGA_RAD_S = 0.5
DEFAULT_HEADING_HOLD_DEADBAND_DEG = 1.0
DEFAULT_HEADING_ABORT_CONSECUTIVE_SAMPLES = 10
DEFAULT_OBSTACLE_PIVOT_SPEED_MPS = 0.08
DEFAULT_OBSTACLE_PIVOT_DURATION_S = 0.55
DEFAULT_OBSTACLE_PIVOT_MAX_ATTEMPTS = 3
DEFAULT_OBSTACLE_RECOVERY_COOLDOWN_S = 0.70
CONSECUTIVE_SAMPLES = DEFAULT_HEADING_ABORT_CONSECUTIVE_SAMPLES
RAD_TO_DEG = 57.29577951308232
DEG_TO_RAD = 0.017453292519943295


class LidarDiagTracker:
    def __init__(self):
        self.samples = 0
        self.matcher_called = 0
        self.filtered_points_total = 0.0
        self.reason_counts = Counter()
        self.odom_accept = 0
        self.odom_reject = 0
        self.last_accepted = None
        self.last_reject_counts: Dict[str, int] = {}
        self.latencies_ms: List[float] = []

    def sample(self, status: Dict[str, Any]) -> None:
        if not isinstance(status, dict):
            return
        self.samples += 1
        lidar = dict((status or {}).get("lidar") or {})
        if lidar.get("matcher_called"):
            self.matcher_called += 1
        filtered_points = float(_safe_float(lidar.get("scan_count_filtered"), 0.0))
        if filtered_points >= 0.0:
            self.filtered_points_total += filtered_points
        
        latency = _maybe_finite(lidar.get("latency_ms"))
        if latency is not None:
            self.latencies_ms.append(float(latency))

        final_conf = float(_safe_float(lidar.get("final_confidence"), 0.0))
        if final_conf <= 0.0:
            reason = str(lidar.get("matcher_reason") or "").strip()
            if reason:
                self.reason_counts[reason] += 1

        lidar_odom_status = dict((status or {}).get("lidar_odom_status") or {})
        if lidar_odom_status:
            # lidar_odom_status["status"] flips to "missing" once the control loop consumes the update, so rely on durable counters instead.
            if "accepted" in lidar_odom_status:
                current_accepted = int(max(0.0, _safe_float(lidar_odom_status.get("accepted"), 0.0)))
                if self.last_accepted is not None:
                    delta = current_accepted - self.last_accepted
                    if delta > 0:
                        self.odom_accept += delta
                self.last_accepted = current_accepted
            for key, value in lidar_odom_status.items():
                if not key.startswith("rejected_"):
                    continue
                current_rejected = int(max(0.0, _safe_float(value, 0.0)))
                prev = self.last_reject_counts.get(key)
                if prev is not None:
                    delta = current_rejected - prev
                    if delta > 0:
                        self.odom_reject += delta
                self.last_reject_counts[key] = current_rejected

    def summary(self) -> Dict[str, Any]:
        samples = int(self.samples)
        matcher_called = int(self.matcher_called)
        filtered_points = float(self.filtered_points_total)
        ratio = (matcher_called / samples) if samples else 0.0
        avg_points = (filtered_points / samples) if samples else 0.0
        return {
            "status_samples": samples,
            "matcher_called_samples": matcher_called,
            "filtered_points_total": filtered_points,
            "reason_counts": dict(self.reason_counts),
            "odom_accept": self.odom_accept,
            "odom_reject": self.odom_reject,
            "matcher_called_ratio": ratio,
            "avg_filtered_points": avg_points,
            "latency_ms_avg": float(sum(self.latencies_ms) / len(self.latencies_ms)) if self.latencies_ms else 0.0,
            "latency_ms_max": float(max(self.latencies_ms)) if self.latencies_ms else 0.0,
        }


def _now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    tmp_path.replace(path)


def _slug_token(value: Any) -> str:
    raw = "".join(ch.lower() if str(ch).isalnum() else "_" for ch in str(value or ""))
    while "__" in raw:
        raw = raw.replace("__", "_")
    raw = raw.strip("_")
    return raw or "suite"


def _append_jsonl(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, sort_keys=True) + "\n")


def _status_state(status: Dict[str, Any]) -> str:
    return str((status or {}).get("state", "") or "").strip().upper()


def _is_arc_exec_truth_anchor(status: Dict[str, Any]) -> bool:
    st = dict(status or {})
    motion_command = dict(st.get("motion_command") or {})
    motion_resolution = dict(st.get("motion_resolution") or {})
    resolved = dict(motion_resolution.get("resolved") or {})
    command_type = str(
        motion_command.get("command_type")
        or st.get("command_type")
        or resolved.get("command_type")
        or ""
    ).strip().lower()
    execution_mode = str(
        motion_command.get("execution_mode")
        or st.get("motion_execution_mode")
        or resolved.get("execution_mode")
        or ""
    ).strip().upper()
    if command_type != "follow_arc" or execution_mode != "ARC_EXEC":
        return False

    primitives = [
        str(
            motion_command.get("turn_primitive_requested")
            or st.get("turn_primitive_requested")
            or resolved.get("turn_primitive_requested")
            or ""
        ).strip().upper(),
        str(
            motion_command.get("turn_primitive_limited")
            or st.get("turn_primitive_limited")
            or resolved.get("turn_primitive_limited")
            or ""
        ).strip().upper(),
        str(
            motion_command.get("turn_primitive_executed")
            or st.get("turn_primitive_executed")
            or resolved.get("turn_primitive_executed")
            or ""
        ).strip().upper(),
        str(
            motion_command.get("turn_primitive_actual")
            or st.get("turn_primitive_actual")
            or resolved.get("turn_primitive_actual")
            or ""
        ).strip().upper(),
    ]
    if any(p.startswith("DIFF_ARC") for p in primitives):
        return True
    if any(p not in ("", "UNKNOWN", "STRAIGHT") for p in primitives):
        return True
    return False


def _select_arc_truth_status(
    statuses: List[Dict[str, Any]],
    *,
    fallback_status: Dict[str, Any],
) -> Dict[str, Any]:
    arc_statuses = [
        dict(status)
        for status in list(statuses or [])
        if _is_arc_exec_truth_anchor(status)
    ]
    if not arc_statuses:
        return dict(fallback_status or {})

    # Prefer an actively executing ARC surface instead of a terminal IDLE
    # snapshot to avoid classifying post-stop residue as arc truth.
    for status in reversed(arc_statuses):
        if _status_state(status) not in ("IDLE", "FAILSAFE", "CALIBRATING"):
            return dict(status)

    # Secondary preference: an ARC surface with non-zero track targets.
    for status in reversed(arc_statuses):
        motion_command = dict((status or {}).get("motion_command") or {})
        track_targets = dict(motion_command.get("track_targets") or {})
        left = _safe_float(track_targets.get("left_mps"), 0.0)
        right = _safe_float(track_targets.get("right_mps"), 0.0)
        if abs(float(left)) > 1e-3 or abs(float(right)) > 1e-3:
            return dict(status)

    # Fallback to the most recent anchor if only terminal surfaces are available.
    for status in reversed(arc_statuses):
        return dict(status)
    return dict(fallback_status or {})


def _resolved_stop_type(status: Dict[str, Any]) -> str:
    st = dict(status or {})
    stop_status = dict(st.get("stop_status") or {})
    gate = dict((st.get("motion_avg") or {}).get("gate") or {})
    raw = st.get("stop_type")
    gate_type = str(gate.get("stop_type", "") or "").strip().upper()
    if raw not in (None, ""):
        return str(raw).strip().upper()
    if gate_type and gate_type != "NONE":
        return gate_type
    if stop_status.get("type") not in (None, ""):
        return str(stop_status.get("type")).strip().upper()
    if gate_type:
        return gate_type
    return "NONE"


def _stop_status_type(status: Dict[str, Any]) -> str:
    return _resolved_stop_type(status)


def _status_is_stopped(status: Dict[str, Any], *, pwm_eps: float = 0.03, vel_eps: float = 0.03) -> bool:
    pwm = dict((status or {}).get("pwm") or {})
    pwm_l = abs(float(_safe_float(pwm.get("left"), 0.0)))
    pwm_r = abs(float(_safe_float(pwm.get("right"), 0.0)))
    v_l = abs(float(_safe_float((status or {}).get("v_l_raw"), 0.0)))
    v_r = abs(float(_safe_float((status or {}).get("v_r_raw"), 0.0)))
    return _status_state(status) in ("IDLE", "FAILSAFE", "CALIBRATING") and pwm_l <= pwm_eps and pwm_r <= pwm_eps and v_l <= vel_eps and v_r <= vel_eps


def _encoder_pose_used(status: Dict[str, Any]) -> bool:
    if "encoder_pose_fusion_active" in (status or {}):
        return bool(status.get("encoder_pose_fusion_active", False))
    # Fallback for older status format
    pose = dict((status or {}).get("pose") or {})
    encoder_enabled = bool(pose.get("encoder_enabled", False))
    trust_mode = str(pose.get("encoder_trust_mode", "") or "").strip().upper()
    return encoder_enabled or trust_mode not in ("", "DISABLED")


def _maybe_finite(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(out):
        return None
    return out


def _extract_encoder_distance(status: Dict[str, Any]) -> Dict[str, Any]:
    st = dict(status or {})
    encoder = dict(st.get("encoder") or {})
    computed = dict(encoder.get("computed") or {})
    left_block = dict((encoder.get("left") or {}).get("snapshot") or {})
    right_block = dict((encoder.get("right") or {}).get("snapshot") or {})

    candidates: List[tuple[str, float]] = []
    canonical = _maybe_finite(st.get("encoder_dist_canonical"))
    if canonical is not None:
        candidates.append(("encoder_dist_canonical", canonical))

    computed_avg = _maybe_finite(computed.get("distance_avg_m"))
    if computed_avg is not None:
        candidates.append(("encoder.computed.distance_avg_m", computed_avg))

    left_top = _maybe_finite(st.get("encoder_dist_left"))
    right_top = _maybe_finite(st.get("encoder_dist_right"))
    if left_top is not None and right_top is not None:
        candidates.append(("encoder_dist_left_right_avg", 0.5 * (left_top + right_top)))

    left_nested = _maybe_finite(left_block.get("distance_m"))
    right_nested = _maybe_finite(right_block.get("distance_m"))
    if left_nested is not None and right_nested is not None:
        candidates.append(("encoder.snapshot.left_right_avg", 0.5 * (left_nested + right_nested)))

    if candidates:
        source, value = candidates[0]
        return {
            "available": True,
            "source": str(source),
            "distance_m": float(value),
        }
    return {
        "available": False,
        "source": "",
        "distance_m": None,
    }


def _lidar_latest_age_s(status: Dict[str, Any]) -> float:
    return float(_safe_float(((status or {}).get("lidar_odom_status") or {}).get("latest_age_s"), math.inf))


def _evaluate_lidar_strict_quality(
    status: Dict[str, Any],
    *,
    min_scan_points: int,
    min_confidence: float,
) -> Dict[str, Any]:
    st = dict(status or {})
    lidar = dict(st.get("lidar") or {})
    lidar_odom_status = dict(st.get("lidar_odom_status") or {})

    scan_count_candidates = [
        _safe_float(lidar.get("scan_count_filtered"), 0.0),
        _safe_float(lidar_odom_status.get("scan_count_filtered"), 0.0),
        _safe_float(lidar_odom_status.get("on_scan_result_called"), 0.0),
        _safe_float(lidar_odom_status.get("candidate_created"), 0.0),
    ]
    scan_count_filtered = int(max(max(0.0, float(v)) for v in scan_count_candidates))
    matcher_called = bool(
        lidar.get("matcher_called", False)
        or _safe_float(lidar_odom_status.get("promotion_attempted"), 0.0) > 0.0
    )
    matcher_reason = str(
        lidar.get("matcher_reason")
        or lidar_odom_status.get("matcher_reason")
        or lidar_odom_status.get("odom_accept_reject_reason")
        or ""
    )

    accepted_total = int(max(0.0, _safe_float(lidar_odom_status.get("accepted"), 0.0)))
    latest_age_s = float(_safe_float(lidar_odom_status.get("latest_age_s"), math.inf))
    latest_fresh = math.isfinite(latest_age_s) and latest_age_s <= float(DEFAULT_PREFLIGHT_MAX_LIDAR_LATEST_AGE_S)

    candidate_conf = float(_safe_float(lidar_odom_status.get("candidate_confidence"), 0.0))
    latest_conf = float(_safe_float(lidar_odom_status.get("latest_confidence"), 0.0))
    signal_conf = max(candidate_conf, latest_conf)

    min_points = max(1, int(min_scan_points))
    min_conf = max(0.0, float(min_confidence))

    scan_ok = scan_count_filtered >= min_points
    matcher_or_fresh_accept_ok = bool(matcher_called or (accepted_total > 0 and latest_fresh))
    confidence_ok = signal_conf >= min_conf
    ok = bool(scan_ok and matcher_or_fresh_accept_ok and confidence_ok)

    return {
        "ok": bool(ok),
        "scan_count_filtered": int(scan_count_filtered),
        "scan_count_required": int(min_points),
        "scan_ok": bool(scan_ok),
        "matcher_called": bool(matcher_called),
        "matcher_reason": str(matcher_reason),
        "accepted_total": int(accepted_total),
        "latest_fresh": bool(latest_fresh),
        "matcher_or_fresh_accept_ok": bool(matcher_or_fresh_accept_ok),
        "candidate_confidence": float(candidate_conf),
        "latest_confidence": float(latest_conf),
        "signal_confidence": float(signal_conf),
        "confidence_required": float(min_conf),
        "confidence_ok": bool(confidence_ok),
    }


def _is_lidar_preflight_feed_ready(
    *,
    lidar_quality_gate_ok: bool,
    lidar_signal_quality: Dict[str, Any],
) -> bool:
    # If strict signal quality is healthy, allow preflight even after long IDLE periods,
    # where latest_age_s can legitimately grow despite a live LiDAR stream.
    return bool(lidar_quality_gate_ok or bool((lidar_signal_quality or {}).get("ok", False)))


def _is_lidar_scan_warmup_transient(lidar_signal_quality: Dict[str, Any]) -> bool:
    quality = dict(lidar_signal_quality or {})
    return bool(
        (not bool(quality.get("ok", False)))
        and (not bool(quality.get("scan_ok", False)))
        and bool(quality.get("matcher_or_fresh_accept_ok", False))
        and bool(quality.get("confidence_ok", False))
    )


def _resolve_lidar_preflight_state(status: Dict[str, Any]) -> Dict[str, Any]:
    lidar_odom_status = dict((status or {}).get("lidar_odom_status") or {})
    latest_age_s = _lidar_latest_age_s(status)
    candidate_age_s = float(_safe_float(lidar_odom_status.get("candidate_age_s"), math.inf))
    candidate_available = bool(lidar_odom_status.get("candidate_available", False))
    lidar_candidate_fresh = (
        candidate_available
        and math.isfinite(candidate_age_s)
        and candidate_age_s <= float(DEFAULT_PREFLIGHT_MAX_LIDAR_CANDIDATE_AGE_S)
    )
    lidar_latest_fresh = math.isfinite(latest_age_s) and latest_age_s <= float(DEFAULT_PREFLIGHT_MAX_LIDAR_LATEST_AGE_S)
    lidar_quality_gate_ok = bool(lidar_candidate_fresh or lidar_latest_fresh)
    strict_min_conf = float(
        _safe_float(
            lidar_odom_status.get("min_confidence"),
            DEFAULT_PREFLIGHT_MIN_LIDAR_CONFIDENCE,
        )
    )
    lidar_signal_quality = _evaluate_lidar_strict_quality(
        status,
        min_scan_points=DEFAULT_PREFLIGHT_MIN_SCAN_POINTS,
        min_confidence=max(DEFAULT_PREFLIGHT_MIN_LIDAR_CONFIDENCE, strict_min_conf),
    )
    lidar_feed_ready = _is_lidar_preflight_feed_ready(
        lidar_quality_gate_ok=bool(lidar_quality_gate_ok),
        lidar_signal_quality=lidar_signal_quality,
    )
    return {
        "lidar_odom_status": dict(lidar_odom_status),
        "latest_age_s": float(latest_age_s),
        "candidate_age_s": float(candidate_age_s),
        "candidate_available": bool(candidate_available),
        "candidate_fresh": bool(lidar_candidate_fresh),
        "latest_fresh": bool(lidar_latest_fresh),
        "quality_gate_ok": bool(lidar_quality_gate_ok),
        "signal_quality": dict(lidar_signal_quality),
        "feed_ready": bool(lidar_feed_ready),
    }


def extract_motion_resolution(status: Dict[str, Any]) -> Dict[str, Any]:
    motion_resolution = dict((status or {}).get("motion_resolution") or {})
    resolved = dict(motion_resolution.get("resolved") or {})
    final_after_shaping = dict(resolved.get("final_after_shaping") or {})
    motion_command = dict((status or {}).get("motion_command") or {})
    limited_motion = dict(motion_command.get("limited_motion_intent") or {})
    arbiter = dict((status or {}).get("arbiter") or {})
    stop_status = dict((status or {}).get("stop_status") or {})

    resolved_source = str(
        resolved.get("source")
        or motion_command.get("source")
        or (status or {}).get("motion_command_source")
        or arbiter.get("active")
        or arbiter.get("source")
        or stop_status.get("source")
        or ""
    )
    resolved_layer = str(
        resolved.get("layer")
        or motion_command.get("active_layer")
        or ""
    )
    resolved_command_type = str(
        resolved.get("command_type")
        or motion_command.get("command_type")
        or ""
    )

    final_v = final_after_shaping.get("v_target", limited_motion.get("v", (status or {}).get("v_target", 0.0)))
    final_omega = final_after_shaping.get(
        "omega_target",
        limited_motion.get("omega", (status or {}).get("omega_target", 0.0)),
    )
    owner_hint = str((status or {}).get("motion_target_owner") or "")
    observable = bool(
        resolved_source
        or owner_hint
        or motion_resolution
        or resolved_layer
        or resolved_command_type
    )

    return {
        "proposal_count": int(_safe_float(motion_resolution.get("proposal_count"), 0.0)),
        "resolved_source": resolved_source,
        "resolved_layer": resolved_layer,
        "resolved_command_type": resolved_command_type,
        "resolved_mode": str(resolved.get("mode") or ""),
        "resolved_owner": owner_hint,
        "final_v": float(_safe_float(final_v, 0.0)),
        "final_omega": float(_safe_float(final_omega, 0.0)),
        "stop_type": _stop_status_type(status),
        "stop_active": bool(((status or {}).get("stop_status") or {}).get("active", False)),
        "observable": bool(observable),
    }


def _wait_for_failsafe(timeout_s: float) -> Dict[str, Any]:
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    last_status: Dict[str, Any] = {}
    last_version = -1
    last_change = time.monotonic()
    while time.monotonic() <= deadline:
        status = _read_json(STATUS_PATH)
        if status:
            last_status = status
            version = _status_version(status)
            if version != last_version:
                last_version = version
                last_change = time.monotonic()
            if _status_state(status) == "FAILSAFE":
                return status
        if time.monotonic() - last_change > DEFAULT_STATUS_STALE_S:
            raise RuntimeError("Status stream went stale while waiting for FAILSAFE.")
        time.sleep(DEFAULT_POLL_S)
    raise TimeoutError("Emergency stop did not latch FAILSAFE in time.")


def _normal_stop(token: str, *, timeout_s: float, motion_source: str | None = None) -> Dict[str, Any]:
    initial_status = _wait_for_status(timeout_s=2.0)
    initial_version = _status_version(initial_status)
    stop_kwargs: Dict[str, Any] = {}
    if str(motion_source or "").strip():
        stop_kwargs["motion_source"] = str(motion_source)
    stop_cmd = _send_command_checked("stop", token=token, timeout_s=4.0, **stop_kwargs)
    deadline = time.monotonic() + max(0.5, float(timeout_s))
    last_status = initial_status
    last_change = time.monotonic()
    while time.monotonic() <= deadline:
        status = _read_json(STATUS_PATH)
        if status:
            last_status = status
            status_version = _status_version(status)
            if status_version != initial_version:
                initial_version = status_version
                last_change = time.monotonic()
            pose_v_abs = abs(float(_safe_float(((status.get("pose") or {}).get("v")), 0.0)))
            if _status_state(status) == "IDLE" and _stop_status_type(status) == "SOFT_STOP" and _status_is_stopped(status):
                hold_started = time.monotonic()
                hold_deadline = hold_started + float(DEFAULT_STOP_SETTLE_HOLD_S)
                hold_ok = True
                stable_status = status
                while time.monotonic() <= hold_deadline:
                    stable_status = _read_json(STATUS_PATH) or stable_status
                    stable_pose_v_abs = abs(float(_safe_float((((stable_status.get("pose") or {}).get("v"))), 0.0)))
                    if not (
                        _status_state(stable_status) == "IDLE"
                        and _stop_status_type(stable_status) == "SOFT_STOP"
                        and _status_is_stopped(stable_status)
                        and stable_pose_v_abs <= float(DEFAULT_STOP_POSE_V_EPS)
                    ):
                        hold_ok = False
                        break
                    time.sleep(DEFAULT_POLL_S)
                if hold_ok:
                    return {
                        "command": stop_cmd,
                        "status": stable_status,
                        "stop_quality": {
                            "stable": True,
                            "hold_s": float(DEFAULT_STOP_SETTLE_HOLD_S),
                            "pose_v_abs": float(
                                abs(float(_safe_float((((stable_status.get("pose") or {}).get("v"))), 0.0)))
                            ),
                            "criteria": {
                                "state": "IDLE",
                                "stop_type": "SOFT_STOP",
                                "pose_v_abs_max": float(DEFAULT_STOP_POSE_V_EPS),
                                "hold_s": float(DEFAULT_STOP_SETTLE_HOLD_S),
                            },
                        },
                    }
            if _status_state(status) == "IDLE" and pose_v_abs <= float(DEFAULT_STOP_POSE_V_EPS):
                # Continue waiting within timeout for explicit SOFT_STOP visibility.
                pass
        if time.monotonic() - last_change > DEFAULT_STATUS_STALE_S:
            raise RuntimeError("Status stream went stale while waiting for SOFT_STOP visibility.")
        time.sleep(DEFAULT_POLL_S)
    raise RuntimeError(
        "Normal stop did not report SOFT_STOP before timeout "
        f"(state={_status_state(last_status)}, stop_type={_stop_status_type(last_status)})."
    )


def _strong_reset_to_idle(token: str, *, timeout_s: float) -> Dict[str, Any]:
    reset_cmd = _send_command_checked("strong_reset", token=token, timeout_s=max(4.0, float(timeout_s)))
    time.sleep(0.25)
    _wait_for_status_progress(min_increments=2, timeout_s=4.0)
    idle = _normal_stop(token, timeout_s=timeout_s)
    return {"command": reset_cmd, "status": idle["status"], "idle_stop": idle}


def _bounded_stop_with_fallback(token: str, *, timeout_s: float, motion_source: str | None = None) -> Dict[str, Any]:
    try:
        normal_stop = _normal_stop(token, timeout_s=timeout_s, motion_source=motion_source)
        status = dict(normal_stop.get("status") or {})
        return {
            "status": status,
            "pose_status": status,
            "command": dict(normal_stop.get("command") or {}),
            "normal_stop_used": True,
            "emergency_stop_triggered": False,
            "failsafe_triggered": False,
            "stop_quality": dict(normal_stop.get("stop_quality") or {}),
            "details": {
                "path": "normal_stop",
            },
        }
    except Exception as exc:
        normal_stop_error = str(exc)
        _safe_stop_best_effort(token)
        details: Dict[str, Any] = {
            "path": "emergency_fallback",
            "normal_stop_error": normal_stop_error,
        }
        end_status = _read_json(STATUS_PATH)
        pose_status = end_status
        failsafe_triggered = False
        emergency_stop_triggered = False
        try:
            emergency_cmd = _send_command_checked("emergency_stop", token=token, timeout_s=4.0)
            emergency_status = _wait_for_failsafe(DEFAULT_EMERGENCY_TIMEOUT_S)
            details["emergency_command"] = emergency_cmd
            details["failsafe_latched"] = True
            pose_status = emergency_status
            end_status = emergency_status
            failsafe_triggered = True
            emergency_stop_triggered = True
            try:
                reset_result = _strong_reset_to_idle(token, timeout_s=DEFAULT_RESET_TIMEOUT_S)
                end_status = dict(reset_result.get("status") or {}) or emergency_status
                details["cleanup_reset"] = {
                    "command": dict((reset_result or {}).get("command") or {}),
                    "idle_stop_command": dict((((reset_result or {}).get("idle_stop") or {}).get("command")) or {}),
                }
            except Exception as reset_exc:
                details["cleanup_error"] = str(reset_exc)
                end_status = _read_json(STATUS_PATH) or emergency_status
        except Exception as emergency_exc:
            details["emergency_error"] = str(emergency_exc)

        return {
            "status": dict(end_status or {}),
            "pose_status": dict(pose_status or end_status or {}),
            "command": {"error": normal_stop_error},
            "normal_stop_used": False,
            "emergency_stop_triggered": bool(emergency_stop_triggered),
            "failsafe_triggered": bool(failsafe_triggered),
            "stop_quality": {
                "stable": False,
                "criteria": {
                    "state": "IDLE",
                    "stop_type": "SOFT_STOP",
                    "pose_v_abs_max": float(DEFAULT_STOP_POSE_V_EPS),
                    "hold_s": float(DEFAULT_STOP_SETTLE_HOLD_S),
                },
            },
            "details": details,
        }


def _cancel_motion_and_wait_idle_local(*, token: str, stop_timeout_s: float) -> Dict[str, Any]:
    cancel_command = {}
    cancel_error = ""
    idle_status: Dict[str, Any] = {}
    idle_error = ""
    try:
        cancel_command = _send_command_checked(
            "cancel_motion",
            token=str(token),
            timeout_s=4.0,
            reason="AGENT_MOTION_PROBE_CANCEL",
        )
    except Exception as exc:
        cancel_error = str(exc)
    try:
        idle_status = dict(
            _wait_until_stopped(
                timeout_s=max(0.5, float(stop_timeout_s)),
                poll_s=max(0.02, float(DEFAULT_POLL_S)),
                linear_eps=float(DEFAULT_STOP_POSE_V_EPS),
            )
            or {}
        )
    except Exception as exc:
        idle_error = str(exc)
        idle_status = dict(_read_json(STATUS_PATH) or {})
    idle_ok = bool(str(_status_state(idle_status)).upper() == "IDLE")
    return {
        "cancel_command": dict(cancel_command or {}),
        "cancel_error": str(cancel_error),
        "idle_verify": {
            "ok": bool(idle_ok),
            "status": dict(idle_status or {}),
            "error": str(idle_error),
        },
    }


def _map_rotate_terminal_reason_local(result: Dict[str, Any]) -> str:
    data = dict(result or {})
    status = str(data.get("status") or "").strip().upper()
    if status in ("DONE", "COMPLETED", "TARGET_REACHED"):
        return "target_angle_reached"
    if status in ("TIMEOUT", "DRIFT_ABORT", "STALL_ABORT", "LIDAR_ABORT", "SAFETY_ABORT"):
        return str(status).lower()
    terminal = str(data.get("terminal_reason") or data.get("reason") or "").strip().upper()
    if not terminal:
        return ""
    if terminal in ("COMPLETED", "TARGET_REACHED", "DONE"):
        return "target_angle_reached"
    return str(terminal).lower()


def _heading_result_matches_target(result: Dict[str, Any], target_heading_deg: float, *, tolerance_deg: float = 2.0) -> bool:
    data = dict(result or {})
    if not data.get("status"):
        return False
    result_target = _maybe_finite(data.get("target_heading_deg"))
    if result_target is None:
        return False
    return abs(_normalize_angle_deg(float(result_target) - float(target_heading_deg))) <= float(tolerance_deg)


def _cancel_motion_stop_with_fallback(token: str, *, timeout_s: float) -> Dict[str, Any]:
    try:
        cancel_outcome = _cancel_motion_and_wait_idle_local(
            token=str(token),
            stop_timeout_s=float(timeout_s),
        )
        idle_verify = dict(cancel_outcome.get("idle_verify") or {})
        status = dict(idle_verify.get("status") or {})
        if bool(idle_verify.get("ok", False)) and _stop_status_type(status) == "SOFT_STOP":
            return {
                "status": status,
                "pose_status": status,
                "command": dict(cancel_outcome.get("cancel_command") or {}),
                "normal_stop_used": True,
                "emergency_stop_triggered": False,
                "failsafe_triggered": False,
                "stop_quality": {
                    "stable": True,
                    "criteria": {
                        "state": "IDLE",
                        "stop_type": "SOFT_STOP",
                        "pose_v_abs_max": float(DEFAULT_STOP_POSE_V_EPS),
                    },
                },
                "details": {
                    "path": "cancel_motion",
                },
            }
        raise RuntimeError(
            "cancel_motion did not settle to SOFT_STOP IDLE "
            f"(state={_status_state(status)}, stop_type={_stop_status_type(status)})."
        )
    except Exception as exc:
        fallback = _bounded_stop_with_fallback(token, timeout_s=timeout_s)
        details = dict(fallback.get("details") or {})
        details["cancel_motion_error"] = str(exc)
        details["path"] = "cancel_motion_fallback"
        fallback["details"] = details
        return fallback


def _run_heading_primitive_test(
    name: str,
    *,
    token: str,
    relative_deg: float,
    max_runtime_s: float,
    stop_timeout_s: float,
    commanded_angular_speed_dps: float | None = None,
    motion_source: str = "STATE",
) -> Dict[str, Any]:
    start_wall = time.monotonic()
    start_status = _wait_for_status(timeout_s=2.0)
    start_pose = _get_pose(start_status)
    start_status_version = _status_version(start_status)
    start_emergency_count = int(_safe_float(((start_status.get("last_emergency") or {}).get("count")), 0.0))
    start_encoder_distance = _extract_encoder_distance(start_status)
    target_heading_deg = float((float(start_pose["theta_deg"]) + float(relative_deg)) % 360.0)
    heading_abort_deg = max(15.0, abs(float(relative_deg)) + 8.0)

    start_cmd = _send_command_checked(
        "rotate_to_heading",
        token=token,
        timeout_s=4.0,
        relative_deg=float(relative_deg),
        motion_source=str(motion_source or "STATE"),
        max_duration_s=float(max_runtime_s),
    )

    diag_tracker = LidarDiagTracker()
    resolved_sources_seen: Counter[str] = Counter()
    resolved_command_types_seen: Counter[str] = Counter()
    status_samples = 0
    applied_samples = 0
    stale_stream = False
    latest_status = start_status
    last_status_version = start_status_version
    last_status_change = time.monotonic()
    max_heading_abs_deg = 0.0
    heading_controller_result: Dict[str, Any] = {}
    heading_active_seen = False
    stop_reason = "max_runtime_reached"

    deadline = time.monotonic() + max(0.1, float(max_runtime_s))
    while time.monotonic() <= deadline:
        now_mono = time.monotonic()
        status = _read_json(STATUS_PATH)
        if status:
            latest_status = status
            diag_tracker.sample(status)
            status_samples += 1
            status_version = _status_version(status)
            if status_version != last_status_version:
                last_status_version = status_version
                last_status_change = now_mono

            resolution = extract_motion_resolution(status)
            if resolution["resolved_source"]:
                resolved_sources_seen[resolution["resolved_source"]] += 1
            if resolution["resolved_command_type"]:
                resolved_command_types_seen[resolution["resolved_command_type"]] += 1
            if bool(((status.get("lidar_odom_status") or {}).get("applied", False))):
                applied_samples += 1

            pose = _get_pose(status)
            heading_change = float(_normalize_angle_deg(pose["theta_deg"] - start_pose["theta_deg"]))
            max_heading_abs_deg = max(float(max_heading_abs_deg), abs(float(heading_change)))

            heading_status = dict(status.get("heading_controller") or {})
            if bool(heading_status.get("active", False)):
                active_target = _maybe_finite(heading_status.get("target_heading_deg"))
                if active_target is None or abs(_normalize_angle_deg(float(active_target) - float(target_heading_deg))) <= 2.0:
                    heading_active_seen = True
            result = dict(heading_status.get("last_result") or {})
            result_matches_target = _heading_result_matches_target(result, target_heading_deg)
            if (
                (not bool(heading_status.get("active", False)))
                and result.get("status")
                and (bool(heading_active_seen) or bool(result_matches_target))
            ):
                heading_controller_result = result
                stop_reason = _map_rotate_terminal_reason_local(result)
                break

        if (now_mono - last_status_change) > DEFAULT_STATUS_STALE_S:
            stale_stream = True
            stop_reason = "status_stream_stale"
            break

        time.sleep(DEFAULT_POLL_S)

    stop_outcome = _cancel_motion_stop_with_fallback(token, timeout_s=stop_timeout_s)
    end_status = dict(stop_outcome.get("status") or latest_status or {})
    if not heading_controller_result:
        end_heading_status = dict(end_status.get("heading_controller") or {})
        end_result = dict(end_heading_status.get("last_result") or {})
        if (
            (not bool(end_heading_status.get("active", False)))
            and end_result.get("status")
            and _heading_result_matches_target(end_result, target_heading_deg)
        ):
            heading_controller_result = end_result
            stop_reason = _map_rotate_terminal_reason_local(end_result)
    end_pose = _get_pose(dict(stop_outcome.get("pose_status") or end_status or {}))
    resolution = extract_motion_resolution(end_status)
    end_emergency_count = int(_safe_float(((end_status.get("last_emergency") or {}).get("count")), 0.0))
    estimated_distance_m = float(_pose_distance(start_pose, end_pose))
    heading_change_deg = float(_normalize_angle_deg(end_pose["theta_deg"] - start_pose["theta_deg"]))
    max_heading_abs_deg = max(float(max_heading_abs_deg), abs(float(heading_change_deg)))
    heading_abort_triggered = bool(abs(float(max_heading_abs_deg)) > float(heading_abort_deg))
    lidar_status_summary = _build_lidar_status_summary(
        start_status,
        end_status,
        applied_samples=applied_samples,
    )
    lidar_diag_summary = diag_tracker.summary()

    end_encoder_distance = _extract_encoder_distance(end_status)
    start_encoder_m = _maybe_finite(start_encoder_distance.get("distance_m"))
    end_encoder_m = _maybe_finite(end_encoder_distance.get("distance_m"))
    encoder_progress_m = 0.0
    if start_encoder_m is not None and end_encoder_m is not None:
        encoder_progress_m = abs(float(end_encoder_m) - float(start_encoder_m))
    encoder_distance_summary = {
        "available": bool(start_encoder_m is not None and end_encoder_m is not None),
        "source": str(end_encoder_distance.get("source") or start_encoder_distance.get("source") or ""),
        "start_distance_m": (None if start_encoder_m is None else float(start_encoder_m)),
        "end_distance_m": (None if end_encoder_m is None else float(end_encoder_m)),
        "max_progress_m": float(encoder_progress_m),
        "samples": 1 if start_encoder_m is not None and end_encoder_m is not None else 0,
    }

    failsafe_triggered = bool(
        stop_outcome.get("failsafe_triggered", False)
        or _status_state(end_status) == "FAILSAFE"
        or end_emergency_count > start_emergency_count
    )
    normal_stop_used = bool(stop_outcome.get("normal_stop_used", False))
    emergency_stop_triggered = bool(stop_outcome.get("emergency_stop_triggered", False))
    resolved_sources_nonempty = {source for source in resolved_sources_seen if source}
    ownership_clear = len(resolved_sources_nonempty) <= 1 or resolved_sources_nonempty.issubset({"STATE", "MANUAL", "GUI_JOYSTICK"})

    move_error = ""
    if stop_reason != "target_angle_reached":
        move_error = str(stop_reason)
    elif heading_controller_result and not bool(heading_controller_result.get("accepted", False)):
        move_error = "heading_controller_not_accepted"
    elif not normal_stop_used:
        move_error = "Normal stop did not complete cleanly."
    elif failsafe_triggered:
        move_error = "FAILSAFE latched during heading primitive."
    elif not ownership_clear:
        move_error = f"Resolved motion source changed during motion: {sorted(resolved_sources_seen)}"

    motion_public_end = _extract_motion_public(end_status)
    motion_public_segment = dict(motion_public_end.get("segment") or {})
    commanded_avg_angular_speed_dps = _safe_float(
        motion_public_segment.get("commanded_average_angular_speed_dps"),
        math.nan,
    )
    if not math.isfinite(float(commanded_avg_angular_speed_dps)):
        if commanded_angular_speed_dps is not None:
            commanded_avg_angular_speed_dps = float(commanded_angular_speed_dps)
        else:
            commanded_avg_angular_speed_dps = float(relative_deg) / max(0.1, float(max_runtime_s))
    actual_avg_angular_speed_dps = _safe_float(
        motion_public_segment.get("actual_average_angular_speed_dps"),
        math.nan,
    )
    if not math.isfinite(float(actual_avg_angular_speed_dps)):
        actual_avg_angular_speed_dps = float(heading_change_deg) / max(
            0.1,
            float(max(0.0, time.monotonic() - start_wall)),
        )
    segment_stop_reason = str(_segment_stop_reason(end_status) or stop_reason or "")
    runtime_s = max(0.0, time.monotonic() - start_wall)
    truth_surface = _truth_surface_from_status(end_status)

    return {
        "timestamp": _now_iso(),
        "test_name": str(name),
        "success": not bool(move_error),
        "fail_reason": str(move_error or ""),
        "odometry_mode": str(end_status.get("odometry_mode", "")),
        "resolved_motion_source": (
            max(resolved_sources_seen, key=resolved_sources_seen.get)
            if resolved_sources_seen
            else resolution["resolved_source"]
        ),
        "start_pose": start_pose,
        "end_pose": end_pose,
        "estimated_distance_m": round(float(estimated_distance_m), 4),
        "cmd_linear_mps": round(float(_safe_float(motion_public_end.get("cmd_linear_mps"), 0.0)), 6),
        "cmd_angular_dps": round(float(_safe_float(motion_public_end.get("cmd_angular_dps"), commanded_avg_angular_speed_dps)), 6),
        "actual_linear_mps": round(float(_safe_float(motion_public_end.get("actual_linear_mps"), 0.0)), 6),
        "actual_angular_dps": round(float(_safe_float(motion_public_end.get("actual_angular_dps"), actual_avg_angular_speed_dps)), 6),
        "commanded_average_linear_speed_mps": 0.0,
        "actual_average_linear_speed_mps": round(
            float(
                abs(float(estimated_distance_m))
                / max(0.1, float(runtime_s))
            ),
            6,
        ),
        "commanded_average_angular_speed_dps": round(float(commanded_avg_angular_speed_dps), 6),
        "actual_average_angular_speed_dps": round(float(actual_avg_angular_speed_dps), 6),
        "heading_change_deg": round(float(heading_change_deg), 3),
        "max_heading_abs_deg": round(float(max_heading_abs_deg), 3),
        "heading_abort_threshold_deg": round(float(heading_abort_deg), 3),
        "heading_abort_triggered": bool(heading_abort_triggered),
        "target_heading_deg": round(float(target_heading_deg), 3),
        "commanded_relative_deg": round(float(relative_deg), 3),
        "heading_controller_result": dict(heading_controller_result),
        "lidar_status_summary": lidar_status_summary,
        "lidar_diag_summary": lidar_diag_summary,
        "encoder_distance_summary": encoder_distance_summary,
        "determinism_path": "EKF_LIDAR_ENCODER",
        "motion_profile": {
            "command_type": "rotate_to_heading",
            "execution_mode": str(truth_surface.get("execution_mode", "UNKNOWN") or "UNKNOWN"),
            "turn_primitives": dict(truth_surface.get("turn_primitives") or {}),
            "turn_primitive_requested": str(truth_surface.get("turn_primitive_requested", "UNKNOWN") or "UNKNOWN"),
            "turn_primitive_limited": str(truth_surface.get("turn_primitive_limited", "UNKNOWN") or "UNKNOWN"),
            "turn_primitive_executed": str(truth_surface.get("turn_primitive_executed", "UNKNOWN") or "UNKNOWN"),
            "turn_primitive_actual": str(truth_surface.get("turn_primitive_actual", "UNKNOWN") or "UNKNOWN"),
            "commanded_relative_deg": float(relative_deg),
            "target_heading_deg": float(target_heading_deg),
            "progress_guard_enforced": False,
            "command_progress_guard_triggered": False,
            "encoder_progress_guard_triggered": False,
        },
        "segment_report": {
            "target_distance_m": None,
            "actual_progress_distance_m": round(float(estimated_distance_m), 4),
            "target_heading_deg": round(float(target_heading_deg), 3),
            "actual_heading_progress_deg": round(float(heading_change_deg), 3),
            "commanded_average_linear_speed_mps": 0.0,
            "actual_average_linear_speed_mps": round(
                float(
                    abs(float(estimated_distance_m))
                    / max(0.1, float(runtime_s))
                ),
                6,
            ),
            "commanded_angular_speed_dps": round(float(commanded_avg_angular_speed_dps), 6),
            "actual_angular_speed_dps": round(float(actual_avg_angular_speed_dps), 6),
            "stop_reason": str(segment_stop_reason or "normal_stop"),
        },
        "measured_distance_m": None,
        "measured_heading_deg": None,
        "loop_health_summary": _build_loop_health_summary(
            start_status_version,
            _status_version(end_status),
            status_samples=status_samples,
            stale_stream=stale_stream,
            resolution_observed=bool(resolved_sources_seen or resolved_command_types_seen),
            sources_seen=resolved_sources_seen,
            command_types_seen=resolved_command_types_seen,
            end_status=end_status,
        ),
        "failsafe_triggered": bool(failsafe_triggered),
        "emergency_stop_triggered": bool(emergency_stop_triggered),
        "normal_stop_used": bool(normal_stop_used),
        "max_runtime_s": float(max_runtime_s),
        "actual_runtime_s": round(float(runtime_s), 3),
        "stop_behavior": {
            "path": str(((stop_outcome.get("details") or {}).get("path")) or "cancel_motion"),
            "stop_type": _stop_status_type(end_status),
            "state": _status_state(end_status),
            "command": dict(stop_outcome.get("command") or {}),
            "details": dict(stop_outcome.get("details") or {}),
        },
        "stop_reason": str(segment_stop_reason or ""),
        "motion_ownership": {
            "clear": bool(ownership_clear),
            "resolved_sources_seen": sorted(source for source in resolved_sources_seen if source),
            "resolved_command_types_seen": sorted(command_type for command_type in resolved_command_types_seen if command_type),
        },
        "command_lifecycle": {
            "start": start_cmd,
            "stop": dict(stop_outcome.get("command") or {}),
            "stop_details": dict(stop_outcome.get("details") or {}),
        },
        **truth_surface,
    }


def _run_arc_segment_test(
    name: str,
    *,
    token: str,
    radius_m: float,
    arc_angle_deg: float,
    speed_mps: float,
    max_runtime_s: float,
    stop_timeout_s: float,
    motion_source: str = "STATE",
) -> Dict[str, Any]:
    """Run a single arc segment and measure heading error, arc length error, lateral deviation."""
    arc_angle_rad = float(arc_angle_deg) * DEG_TO_RAD
    start_wall = time.monotonic()
    start_status = _wait_for_status(timeout_s=2.0)
    start_pose = _get_pose(start_status)
    start_heading_deg = float(start_pose.get("theta_deg", 0.0))
    expected_heading_deg = (start_heading_deg + float(arc_angle_deg)) % 360.0
    expected_arc_length_m = abs(float(radius_m) * float(arc_angle_rad))

    start_cmd = _send_follow_arc_with_active_retry(
        token=token,
        radius_m=float(radius_m),
        arc_angle_rad=float(arc_angle_rad),
        speed_mps=float(speed_mps),
        max_runtime_s=float(max_runtime_s),
        stop_timeout_s=float(stop_timeout_s),
        motion_source=str(motion_source or "STATE"),
    )

    diag_tracker = LidarDiagTracker()
    status_samples = 0
    latest_status = start_status
    arc_truth_status_samples: List[Dict[str, Any]] = []
    arc_tracking_samples: List[Dict[str, Any]] = []
    stop_reason = "max_runtime_reached"
    prev_pose_for_path = dict(start_pose)
    prev_state = _status_state(start_status)
    path_length_estimate_m = 0.0
    path_segment_count = 0
    obstacle_recovery_attempts = 0
    obstacle_recovery_events: List[Dict[str, Any]] = []
    obstacle_recovery_last_mono = 0.0
    obstacle_recovery_last_prefer_left: bool | None = None
    nominal_turn_sign = 1.0 if float(arc_angle_rad) >= 0.0 else -1.0
    nominal_omega_target_rad_s = float(speed_mps) * float(nominal_turn_sign) / max(1e-6, abs(float(radius_m)))
    sample_prev_heading_deg = float(start_heading_deg)
    sample_prev_mono = float(start_wall)

    def _is_arc_exec_context(status_obj: Dict[str, Any]) -> bool:
        st = dict(status_obj or {})
        motion_command = dict(st.get("motion_command") or {})
        motion_resolution = dict(st.get("motion_resolution") or {})
        resolved = dict(motion_resolution.get("resolved") or {})
        command_type = str(
            motion_command.get("command_type")
            or st.get("command_type")
            or resolved.get("command_type")
            or ""
        ).strip().lower()
        execution_mode = str(
            motion_command.get("execution_mode")
            or st.get("motion_execution_mode")
            or resolved.get("execution_mode")
            or ""
        ).strip().upper()
        return command_type == "follow_arc" and execution_mode == "ARC_EXEC"

    def _append_arc_tracking_sample(status_obj: Dict[str, Any], *, sample_mono: float) -> None:
        nonlocal sample_prev_heading_deg, sample_prev_mono
        st = dict(status_obj or {})
        motion_command = dict(st.get("motion_command") or {})
        motion_public = dict(st.get("motion_public") or {})
        pose_sample = _get_pose(st)
        heading_now_deg = float(pose_sample.get("theta_deg", sample_prev_heading_deg))

        dt_sample = max(0.0, float(sample_mono) - float(sample_prev_mono))
        delta_heading_deg = float(_normalize_angle_deg(float(heading_now_deg) - float(sample_prev_heading_deg)))
        omega_feedback_est = None
        if float(dt_sample) >= 1e-3:
            omega_feedback_est = float(delta_heading_deg) * float(DEG_TO_RAD) / float(dt_sample)
        sample_prev_heading_deg = float(heading_now_deg)
        sample_prev_mono = float(sample_mono)

        sample_progress_frac = _maybe_finite(st.get("progress_frac"))
        if sample_progress_frac is None and abs(float(arc_angle_deg)) > 1e-6:
            sample_progress_frac = abs(
                float(_normalize_angle_deg(float(heading_now_deg) - float(start_heading_deg)))
            ) / max(1e-6, abs(float(arc_angle_deg)))
        if sample_progress_frac is not None:
            sample_progress_frac = max(0.0, min(1.5, float(sample_progress_frac)))

        sample_omega_target = _maybe_finite(st.get("omega_target_outer_rad_s"))
        if sample_omega_target is None:
            sample_omega_target = float(nominal_omega_target_rad_s)
        sample_omega_feedback = _maybe_finite(st.get("omega_feedback_rad_s"))
        if sample_omega_feedback is None:
            sample_omega_feedback = _maybe_finite(omega_feedback_est)

        sample_left_track = _maybe_finite(st.get("left_track_mps"))
        sample_right_track = _maybe_finite(st.get("right_track_mps"))
        if sample_left_track is None or sample_right_track is None:
            track_targets = dict(motion_command.get("track_targets") or {})
            if sample_left_track is None:
                sample_left_track = _maybe_finite(track_targets.get("left_mps"))
            if sample_right_track is None:
                sample_right_track = _maybe_finite(track_targets.get("right_mps"))

        sample_v_cmd = _maybe_finite(st.get("v_cmd"))
        if sample_v_cmd is None:
            sample_v_cmd = _maybe_finite(
                motion_public.get(
                    "cmd_linear_mps",
                    motion_public.get("linear_speed_mps"),
                )
            )
        if sample_v_cmd is None and sample_left_track is not None and sample_right_track is not None:
            sample_v_cmd = 0.5 * (float(sample_left_track) + float(sample_right_track))
        if sample_v_cmd is None:
            sample_v_cmd = float(speed_mps)

        sample_inner_positive = False
        sample_pivot_like = bool(st.get("arc_pivot_like", False))
        if sample_left_track is not None and sample_right_track is not None:
            sample_inner_positive = bool(
                float(sample_left_track) > 0.0
                and float(sample_right_track) > 0.0
                and abs(float(sample_left_track) - float(sample_right_track)) > 1e-6
            )
            inner_abs = min(abs(float(sample_left_track)), abs(float(sample_right_track)))
            if inner_abs <= 0.004 or (float(sample_left_track) * float(sample_right_track)) <= 0.0:
                sample_pivot_like = True
        else:
            primitive_actual = str(
                motion_command.get("turn_primitive_actual")
                or st.get("turn_primitive_actual")
                or ""
            ).strip().upper()
            if "ROTATE" in primitive_actual or "PIVOT" in primitive_actual:
                sample_pivot_like = True

        sample_curvature_target = _maybe_finite(
            st.get("curvature_outer_target_m_inv", st.get("curvature_target_m_inv"))
        )
        if sample_curvature_target is None and sample_omega_target is not None:
            sample_curvature_target = float(sample_omega_target) / max(0.02, abs(float(sample_v_cmd)))
        sample_curvature_feedback = None
        if (
            sample_omega_feedback is not None
            and sample_v_cmd is not None
            and abs(float(sample_v_cmd)) >= 0.02
        ):
            sample_curvature_feedback = float(sample_omega_feedback) / float(sample_v_cmd)

        arc_tracking_samples.append(
            {
                "progress_frac": sample_progress_frac,
                "omega_target_rad_s": sample_omega_target,
                "omega_feedback_rad_s": sample_omega_feedback,
                "curvature_target_m_inv": sample_curvature_target,
                "curvature_feedback_m_inv": sample_curvature_feedback,
                "left_track_mps": sample_left_track,
                "right_track_mps": sample_right_track,
                "inner_positive": bool(sample_inner_positive),
                "pivot_like": bool(sample_pivot_like),
            }
        )

    deadline = time.monotonic() + max(0.1, float(max_runtime_s))
    while time.monotonic() <= deadline:
        status = _read_json(STATUS_PATH)
        if status:
            latest_status = status
            diag_tracker.sample(status)
            status_samples += 1
            if _is_arc_exec_context(status):
                _append_arc_tracking_sample(status, sample_mono=float(time.monotonic()))
            if _is_arc_exec_truth_anchor(status):
                arc_truth_status_samples.append(dict(status))

            obstacle = _status_obstacle_snapshot(status)
            if bool(obstacle.get("blocked_front", False)):
                recovery_cooldown_ok = (
                    float(time.monotonic()) - float(obstacle_recovery_last_mono)
                ) >= float(DEFAULT_OBSTACLE_RECOVERY_COOLDOWN_S)
                if (
                    recovery_cooldown_ok
                    and int(obstacle_recovery_attempts) < int(DEFAULT_OBSTACLE_PIVOT_MAX_ATTEMPTS)
                ):
                    avg_left = _maybe_finite(obstacle.get("avg_left_m"))
                    avg_right = _maybe_finite(obstacle.get("avg_right_m"))
                    if avg_left is not None and avg_right is not None:
                        prefer_left = bool(float(avg_left) >= float(avg_right))
                    elif obstacle_recovery_last_prefer_left is not None:
                        prefer_left = not bool(obstacle_recovery_last_prefer_left)
                    else:
                        prefer_left = True

                    obstacle_recovery_attempts += 1
                    obstacle_recovery_last_mono = float(time.monotonic())
                    obstacle_recovery_last_prefer_left = bool(prefer_left)
                    recovery = _attempt_obstacle_pivot_recovery(
                        token=str(token),
                        status=dict(status),
                        motion_source=str(motion_source or "STATE"),
                        prefer_left=bool(prefer_left),
                        pivot_speed_mps=float(DEFAULT_OBSTACLE_PIVOT_SPEED_MPS),
                        pivot_duration_s=float(DEFAULT_OBSTACLE_PIVOT_DURATION_S),
                        stop_timeout_s=float(stop_timeout_s),
                    )
                    obstacle_recovery_events.append(
                        {
                            "attempt": int(obstacle_recovery_attempts),
                            "timestamp": _now_iso(),
                            **dict(recovery),
                        }
                    )
                    latest_status = dict(recovery.get("status_after") or latest_status or {})
                    if bool(recovery.get("recovered", False)):
                        pose_after_recovery = _get_pose(dict(recovery.get("status_after") or {}))
                        remaining_heading_deg = float(
                            _normalize_angle_deg(
                                float(expected_heading_deg) - float(pose_after_recovery.get("theta_deg", 0.0))
                            )
                        )
                        remaining_runtime_s = max(1.0, float(deadline - time.monotonic()))
                        if abs(float(remaining_heading_deg)) > 1.5 and remaining_runtime_s > 0.5:
                            try:
                                start_cmd = _send_follow_arc_with_active_retry(
                                    token=token,
                                    radius_m=float(radius_m),
                                    arc_angle_rad=float(remaining_heading_deg) * float(DEG_TO_RAD),
                                    speed_mps=float(speed_mps),
                                    max_runtime_s=float(min(float(max_runtime_s), remaining_runtime_s)),
                                    stop_timeout_s=float(stop_timeout_s),
                                    motion_source=str(motion_source or "STATE"),
                                )
                            except Exception as resume_exc:
                                stop_reason = f"obstacle_recovery_resume_failed:{resume_exc}"
                                break
                        else:
                            stop_reason = "arc_completed_after_obstacle_recovery"
                            break
                    else:
                        if int(obstacle_recovery_attempts) >= int(DEFAULT_OBSTACLE_PIVOT_MAX_ATTEMPTS):
                            stop_reason = (
                                "obstacle_recovery_exhausted:"
                                f"blocked_front_after_{int(obstacle_recovery_attempts)}_pivot_attempts"
                            )
                            break
                    continue
                if int(obstacle_recovery_attempts) >= int(DEFAULT_OBSTACLE_PIVOT_MAX_ATTEMPTS):
                    stop_reason = (
                        "obstacle_recovery_exhausted:"
                        f"blocked_front_after_{int(obstacle_recovery_attempts)}_pivot_attempts"
                    )
                    break

            pose_now = _get_pose(status)
            state_now = _status_state(status)
            segment_m = float(_pose_distance(prev_pose_for_path, pose_now))
            if math.isfinite(segment_m) and segment_m > 0.0 and (prev_state != "IDLE" or state_now != "IDLE"):
                path_length_estimate_m += segment_m
                path_segment_count += 1
            prev_pose_for_path = dict(pose_now)
            prev_state = state_now

        state_str = _status_state(status or start_status)
        if state_str == "IDLE" and (time.monotonic() - start_wall) > 0.5:
            stop_reason = "arc_completed_idle"
            break
        time.sleep(float(DEFAULT_POLL_S))

    runtime_s = max(0.0, time.monotonic() - start_wall)
    end_status = _wait_for_status(timeout_s=1.0) or latest_status
    if _is_arc_exec_context(end_status):
        _append_arc_tracking_sample(end_status, sample_mono=float(time.monotonic()))
    if _is_arc_exec_truth_anchor(end_status):
        arc_truth_status_samples.append(dict(end_status))
    end_pose = _get_pose(end_status)
    end_heading_deg = float(end_pose.get("theta_deg", 0.0))
    end_state = _status_state(end_status)
    final_segment_m = float(_pose_distance(prev_pose_for_path, end_pose))
    if math.isfinite(final_segment_m) and final_segment_m > 0.0 and (prev_state != "IDLE" or end_state != "IDLE"):
        path_length_estimate_m += final_segment_m
        path_segment_count += 1

    heading_error_deg = float(_normalize_angle_deg(end_heading_deg - expected_heading_deg))
    actual_heading_change_deg = float(_normalize_angle_deg(end_heading_deg - start_heading_deg))

    dx = float(end_pose.get("x", 0.0)) - float(start_pose.get("x", 0.0))
    dy = float(end_pose.get("y", 0.0)) - float(start_pose.get("y", 0.0))
    travel_distance_m = math.sqrt(dx * dx + dy * dy)

    actual_path_length_m = max(float(travel_distance_m), float(path_length_estimate_m))
    arc_length_error_m = abs(actual_path_length_m - expected_arc_length_m)

    # Lateral deviation: distance from ideal arc endpoint
    # Ideal endpoint on circle: center + R at angle (start + arc_angle)
    start_theta_rad = float(start_heading_deg) * DEG_TO_RAD
    turn_sign = 1.0 if arc_angle_rad >= 0 else -1.0
    center_x = float(start_pose.get("x", 0.0)) + float(radius_m) * math.cos(start_theta_rad + turn_sign * math.pi / 2.0)
    center_y = float(start_pose.get("y", 0.0)) + float(radius_m) * math.sin(start_theta_rad + turn_sign * math.pi / 2.0)
    dist_to_center = math.sqrt(
        (float(end_pose.get("x", 0.0)) - center_x) ** 2 +
        (float(end_pose.get("y", 0.0)) - center_y) ** 2
    )
    lateral_deviation_m = abs(dist_to_center - float(radius_m))
    lateral_deviation_limit_m = _arc_lateral_deviation_tolerance_m(float(radius_m))
    truth_status = _select_arc_truth_status(
        arc_truth_status_samples,
        fallback_status=end_status,
    )
    truth_surface = _truth_surface_from_status(truth_status)
    truth_resolution = extract_motion_resolution(truth_status)
    end_resolution = extract_motion_resolution(end_status)
    arc_total_samples = int(len(arc_tracking_samples))
    arc_positive_samples = int(sum(1 for sample in arc_tracking_samples if bool(sample.get("inner_positive", False))))
    arc_inner_track_positive_ratio = (
        float(arc_positive_samples) / float(arc_total_samples)
        if arc_total_samples > 0
        else _safe_float(truth_surface.get("arc_inner_track_positive_ratio"), 0.0)
    )
    arc_inner_track_positive_ratio = max(0.0, min(1.0, float(arc_inner_track_positive_ratio)))
    arc_inner_track_positive_ratio_limit = float(DEFAULT_ARC_INNER_TRACK_POSITIVE_RATIO_MIN)
    arc_inner_track_positive_ratio_ok = (
        float(arc_inner_track_positive_ratio) >= float(arc_inner_track_positive_ratio_limit)
    )

    max_progress_frac = 0.0
    for sample in arc_tracking_samples:
        sample_progress = _maybe_finite(sample.get("progress_frac"))
        if sample_progress is not None:
            max_progress_frac = max(float(max_progress_frac), float(sample_progress))

    early_samples = [
        sample
        for sample in arc_tracking_samples
        if (
            (_maybe_finite(sample.get("progress_frac")) is not None)
            and float(_maybe_finite(sample.get("progress_frac"))) <= float(DEFAULT_ARC_EARLY_PROGRESS_MAX_FRAC)
        )
    ]
    late_samples = [
        sample
        for sample in arc_tracking_samples
        if (
            (_maybe_finite(sample.get("progress_frac")) is not None)
            and float(_maybe_finite(sample.get("progress_frac"))) >= float(DEFAULT_ARC_LATE_PROGRESS_MIN_FRAC)
        )
    ]

    early_target_abs_peak = max(
        (
            abs(float(_safe_float(sample.get("omega_target_rad_s"), 0.0)))
            for sample in early_samples
            if _maybe_finite(sample.get("omega_target_rad_s")) is not None
        ),
        default=0.0,
    )
    arc_early_turning_threshold_rad_s = max(
        float(DEFAULT_ARC_EARLY_YAW_MIN_ABS_RAD_S),
        float(DEFAULT_ARC_EARLY_YAW_MIN_TARGET_RATIO) * float(early_target_abs_peak),
    )
    arc_early_turning_present = any(
        (
            (_maybe_finite(sample.get("omega_feedback_rad_s")) is not None)
            and (
                abs(float(_safe_float(sample.get("omega_feedback_rad_s"), 0.0)))
                >= float(arc_early_turning_threshold_rad_s)
            )
            and (
                (_maybe_finite(sample.get("omega_target_rad_s")) is None)
                or (
                    float(_safe_float(sample.get("omega_feedback_rad_s"), 0.0))
                    * float(_safe_float(sample.get("omega_target_rad_s"), 0.0))
                    >= 0.0
                )
            )
        )
        for sample in early_samples
    )

    arc_late_snap_pivot_like_samples = int(sum(1 for sample in late_samples if bool(sample.get("pivot_like", False))))
    if late_samples:
        arc_no_late_snap_turn = bool(arc_late_snap_pivot_like_samples == 0)
    elif arc_total_samples > 0:
        arc_no_late_snap_turn = bool(int(_safe_float(truth_surface.get("arc_pivot_like_samples"), 0.0)) == 0)
    else:
        arc_no_late_snap_turn = False

    omega_error_samples = [
        float(_safe_float(sample.get("omega_target_rad_s"), 0.0))
        - float(_safe_float(sample.get("omega_feedback_rad_s"), 0.0))
        for sample in arc_tracking_samples
        if (
            _maybe_finite(sample.get("omega_target_rad_s")) is not None
            and _maybe_finite(sample.get("omega_feedback_rad_s")) is not None
        )
    ]
    omega_tracking_error_rms = _rms(omega_error_samples)
    omega_tracking_error_rms_limit = float(DEFAULT_ARC_OMEGA_TRACKING_ERROR_RMS_MAX_RAD_S)
    omega_tracking_ok = (
        omega_tracking_error_rms is not None
        and float(omega_tracking_error_rms) <= float(omega_tracking_error_rms_limit)
    )

    curvature_error_samples = [
        float(_safe_float(sample.get("curvature_target_m_inv"), 0.0))
        - float(_safe_float(sample.get("curvature_feedback_m_inv"), 0.0))
        for sample in arc_tracking_samples
        if (
            _maybe_finite(sample.get("curvature_target_m_inv")) is not None
            and _maybe_finite(sample.get("curvature_feedback_m_inv")) is not None
        )
    ]
    curvature_error_rms = _rms(curvature_error_samples)
    curvature_error_rms_limit = float(DEFAULT_ARC_CURVATURE_ERROR_RMS_MAX_M_INV)
    curvature_tracking_ok = (
        curvature_error_rms is not None
        and float(curvature_error_rms) <= float(curvature_error_rms_limit)
    )

    arc_physical_contract_ok = bool(
        arc_early_turning_present
        and arc_no_late_snap_turn
        and arc_inner_track_positive_ratio_ok
        and omega_tracking_ok
        and curvature_tracking_ok
    )

    success = (
        abs(heading_error_deg) < float(DEFAULT_ARC_HEADING_ERROR_DEG)
        and arc_length_error_m < float(DEFAULT_ARC_LENGTH_ERROR_M)
        and lateral_deviation_m <= float(lateral_deviation_limit_m)
        and stop_reason != "max_runtime_reached"
        and bool(arc_physical_contract_ok)
    )
    fail_reason = ""
    if not success:
        if stop_reason == "max_runtime_reached":
            fail_reason = "arc_timeout"
        elif abs(heading_error_deg) >= float(DEFAULT_ARC_HEADING_ERROR_DEG):
            fail_reason = f"heading_error_{heading_error_deg:.1f}deg"
        elif arc_length_error_m >= float(DEFAULT_ARC_LENGTH_ERROR_M):
            fail_reason = f"arc_length_error_{arc_length_error_m:.3f}m"
        elif not bool(arc_early_turning_present):
            fail_reason = "arc_early_turning_missing"
        elif not bool(arc_no_late_snap_turn):
            fail_reason = f"arc_late_snap_turn:{int(arc_late_snap_pivot_like_samples)}"
        elif not bool(arc_inner_track_positive_ratio_ok):
            fail_reason = f"arc_inner_track_positive_ratio_{float(arc_inner_track_positive_ratio):.3f}"
        elif not bool(omega_tracking_ok):
            if omega_tracking_error_rms is None:
                fail_reason = "omega_tracking_error_rms_missing"
            else:
                fail_reason = f"omega_tracking_error_rms_{float(omega_tracking_error_rms):.3f}"
        elif not bool(curvature_tracking_ok):
            if curvature_error_rms is None:
                fail_reason = "curvature_error_rms_missing"
            else:
                fail_reason = f"curvature_error_rms_{float(curvature_error_rms):.3f}"
        else:
            fail_reason = f"lateral_deviation_{lateral_deviation_m:.3f}m"

    return {
        "timestamp": _now_iso(),
        "test_name": str(name),
        "success": bool(success),
        "fail_reason": fail_reason,
        "odometry_mode": str((end_status or {}).get("odometry_mode", "")),
        "resolved_motion_source": (
            str(truth_resolution.get("resolved_source", "") or "").strip().upper()
            or str(end_resolution.get("resolved_source", "") or "").strip().upper()
            or str(motion_source or "STATE").strip().upper()
        ),
        "start_pose": start_pose,
        "end_pose": end_pose,
        "arc_params": {
            "radius_m": float(radius_m),
            "arc_angle_deg": float(arc_angle_deg),
            "speed_mps": float(speed_mps),
            "expected_arc_length_m": round(expected_arc_length_m, 4),
        },
        "motion_profile": {
            "command_type": "follow_arc",
            "execution_mode": str(truth_surface.get("execution_mode", "UNKNOWN") or "UNKNOWN"),
            "turn_primitives": dict(truth_surface.get("turn_primitives") or {}),
            "turn_primitive_requested": str(truth_surface.get("turn_primitive_requested", "UNKNOWN") or "UNKNOWN"),
            "turn_primitive_limited": str(truth_surface.get("turn_primitive_limited", "UNKNOWN") or "UNKNOWN"),
            "turn_primitive_executed": str(truth_surface.get("turn_primitive_executed", "UNKNOWN") or "UNKNOWN"),
            "turn_primitive_actual": str(truth_surface.get("turn_primitive_actual", "UNKNOWN") or "UNKNOWN"),
            "arc_physical_contract_ok": bool(arc_physical_contract_ok),
            "arc_early_turning_present": bool(arc_early_turning_present),
            "arc_no_late_snap_turn": bool(arc_no_late_snap_turn),
            "arc_inner_track_positive_ratio": round(float(arc_inner_track_positive_ratio), 4),
        },
        "measurements": {
            "heading_error_deg": round(heading_error_deg, 3),
            "actual_heading_change_deg": round(actual_heading_change_deg, 3),
            "arc_length_error_m": round(arc_length_error_m, 4),
            "lateral_deviation_m": round(lateral_deviation_m, 4),
            "lateral_deviation_limit_m": round(float(lateral_deviation_limit_m), 4),
            "path_length_estimate_m": round(float(actual_path_length_m), 4),
            "travel_chord_distance_m": round(travel_distance_m, 4),
            "path_segment_count": int(path_segment_count),
            "arc_samples": int(arc_total_samples),
            "arc_early_samples": int(len(early_samples)),
            "arc_late_samples": int(len(late_samples)),
            "arc_early_turning_threshold_rad_s": round(float(arc_early_turning_threshold_rad_s), 4),
            "arc_late_snap_pivot_like_samples": int(arc_late_snap_pivot_like_samples),
            "arc_inner_track_positive_ratio": round(float(arc_inner_track_positive_ratio), 4),
            "arc_inner_track_positive_ratio_limit": float(arc_inner_track_positive_ratio_limit),
            "omega_tracking_error_rms_rad_s": (
                None if omega_tracking_error_rms is None else round(float(omega_tracking_error_rms), 4)
            ),
            "omega_tracking_error_rms_limit_rad_s": float(omega_tracking_error_rms_limit),
            "curvature_error_rms_m_inv": (
                None if curvature_error_rms is None else round(float(curvature_error_rms), 4)
            ),
            "curvature_error_rms_limit_m_inv": float(curvature_error_rms_limit),
        },
        "obstacle_recovery": {
            "attempts": int(obstacle_recovery_attempts),
            "events": list(obstacle_recovery_events),
            "max_attempts": int(DEFAULT_OBSTACLE_PIVOT_MAX_ATTEMPTS),
        },
        "stop_reason": stop_reason,
        "actual_runtime_s": round(runtime_s, 3),
        "max_runtime_s": float(max_runtime_s),
        "lidar_diag_summary": diag_tracker.summary(),
        "estimated_distance_m": round(float(actual_path_length_m), 4),
        "heading_change_deg": round(actual_heading_change_deg, 3),
        "failsafe_triggered": False,
        "emergency_stop_triggered": False,
        "normal_stop_used": False,
        "lidar_status_summary": {},
        "loop_health_summary": {},
        "arc_early_turning_present": bool(arc_early_turning_present),
        "arc_early_turning_sample_count": int(len(early_samples)),
        "arc_early_turning_threshold_rad_s": round(float(arc_early_turning_threshold_rad_s), 4),
        "arc_no_late_snap_turn": bool(arc_no_late_snap_turn),
        "arc_late_window_sample_count": int(len(late_samples)),
        "arc_late_snap_pivot_like_samples": int(arc_late_snap_pivot_like_samples),
        "arc_inner_track_positive_ratio": round(float(arc_inner_track_positive_ratio), 4),
        "arc_inner_track_positive_ratio_limit": float(arc_inner_track_positive_ratio_limit),
        "arc_inner_track_positive_ratio_ok": bool(arc_inner_track_positive_ratio_ok),
        "omega_tracking_error_rms_rad_s": (
            None if omega_tracking_error_rms is None else round(float(omega_tracking_error_rms), 4)
        ),
        "omega_tracking_error_rms_limit_rad_s": float(omega_tracking_error_rms_limit),
        "omega_tracking_ok": bool(omega_tracking_ok),
        "curvature_error_rms_m_inv": (
            None if curvature_error_rms is None else round(float(curvature_error_rms), 4)
        ),
        "curvature_error_rms_limit_m_inv": float(curvature_error_rms_limit),
        "curvature_tracking_ok": bool(curvature_tracking_ok),
        "arc_physical_contract_ok": bool(arc_physical_contract_ok),
        "arc_tracking_sample_count": int(arc_total_samples),
        "arc_tracking_max_progress_frac": round(float(max_progress_frac), 4),
        "truth_surface_anchor": {
            "used_arc_exec_anchor": bool(_is_arc_exec_truth_anchor(truth_status)),
            "state": _status_state(truth_status),
            "motion_execution_mode": str(
                (
                    (dict((truth_status or {}).get("motion_command") or {}).get("execution_mode"))
                    or (truth_status or {}).get("motion_execution_mode")
                    or truth_surface.get("execution_mode")
                    or ""
                )
            ).strip().upper(),
            "command_type": str(
                (
                    (dict((truth_status or {}).get("motion_command") or {}).get("command_type"))
                    or (truth_status or {}).get("command_type")
                    or truth_resolution.get("resolved_command_type")
                    or ""
                )
            ).strip().lower(),
        },
        **truth_surface,
    }


def _arc_lateral_deviation_tolerance_m(radius_m: float) -> float:
    radius = max(0.01, float(radius_m))
    tol = radius * float(DEFAULT_ARC_LATERAL_DEVIATION_RATIO)
    return float(
        max(
            float(DEFAULT_ARC_LATERAL_DEVIATION_MIN_M),
            min(float(DEFAULT_ARC_LATERAL_DEVIATION_MAX_M), tol),
        )
    )


def _send_follow_arc_with_active_retry(
    *,
    token: str,
    radius_m: float,
    arc_angle_rad: float,
    speed_mps: float,
    max_runtime_s: float,
    stop_timeout_s: float,
    motion_source: str = "STATE",
) -> Dict[str, Any]:
    """Send follow_arc and retry with quiesce when command lane is still busy."""
    settle_timeout_s = max(1.0, min(6.0, float(stop_timeout_s)))
    max_attempts = 3
    for attempt in range(1, max_attempts + 1):
        try:
            return _send_command_checked(
                "follow_arc",
                token=token,
                timeout_s=4.0,
                radius_m=float(radius_m),
                arc_angle_rad=float(arc_angle_rad),
                speed_mps=float(speed_mps),
                max_duration_s=float(max_runtime_s),
                motion_source=str(motion_source or "STATE"),
            )
        except Exception as exc:
            if "blocked_by_active" not in str(exc):
                raise
            if attempt >= max_attempts:
                raise
            _cancel_motion_stop_with_fallback(str(token), timeout_s=settle_timeout_s)
            try:
                _wait_until_stopped(timeout_s=settle_timeout_s)
            except Exception:
                pass
            time.sleep(0.20 + (0.10 * float(attempt)))
    raise RuntimeError("follow_arc_retry_exhausted")


def _stats(values: List[float]) -> Dict[str, Any]:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return {"count": 0, "min": None, "median": None, "mean": None, "max": None}
    return {
        "count": int(len(clean)),
        "min": float(min(clean)),
        "median": float(statistics.median(clean)),
        "mean": float(sum(clean) / len(clean)),
        "max": float(max(clean)),
    }


def _rms(values: List[float]) -> float | None:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return None
    mean_sq = sum(float(v) * float(v) for v in clean) / float(len(clean))
    return float(math.sqrt(max(0.0, float(mean_sq))))


def _first_fail_reason(items: List[Dict[str, Any]]) -> str:
    for item in items:
        reason = str(item.get("fail_reason", "") or "")
        if reason:
            return reason
    return ""


def _build_lidar_status_summary(
    start_status: Dict[str, Any],
    end_status: Dict[str, Any],
    *,
    applied_samples: int,
) -> Dict[str, Any]:
    start_lidar = dict((start_status or {}).get("lidar_odom_status") or {})
    end_lidar = dict((end_status or {}).get("lidar_odom_status") or {})
    return {
        "accepted_delta": int(_safe_float(end_lidar.get("accepted"), 0.0) - _safe_float(start_lidar.get("accepted"), 0.0)),
        "rejected_low_confidence_delta": int(
            _safe_float(end_lidar.get("rejected_low_confidence"), 0.0)
            - _safe_float(start_lidar.get("rejected_low_confidence"), 0.0)
        ),
        "rejected_large_jump_delta": int(
            _safe_float(end_lidar.get("rejected_large_jump"), 0.0)
            - _safe_float(start_lidar.get("rejected_large_jump"), 0.0)
        ),
        "applied_status_samples": int(applied_samples),
        "latest_age_s": float(_safe_float(end_lidar.get("latest_age_s"), math.inf)),
        "latest_confidence": float(_safe_float(end_lidar.get("latest_confidence"), math.nan)),
        "fresh": bool(_lidar_latest_age_s(end_status) <= 1.5),
    }


def _pose_xyz_dict(value: Any) -> Dict[str, float] | None:
    raw = None
    if isinstance(value, dict):
        raw = value
    elif isinstance(value, (list, tuple)) and len(value) >= 3:
        raw = {"x": value[0], "y": value[1], "theta": value[2]}
    if raw is None:
        return None
    x = _maybe_finite(raw.get("x"))
    y = _maybe_finite(raw.get("y"))
    theta = _maybe_finite(raw.get("theta"))
    if x is None or y is None or theta is None:
        return None
    return {"x": float(x), "y": float(y), "theta": float(theta)}


def _extract_lidar_reference_telemetry(status: Dict[str, Any]) -> Dict[str, Any]:
    lidar_odom_status = dict((status or {}).get("lidar_odom_status") or {})
    return {
        "pose_ref_current": _pose_xyz_dict(lidar_odom_status.get("pose_ref_current")),
        "prev_pose_ref": _pose_xyz_dict(lidar_odom_status.get("prev_pose_ref")),
        "last_lidar_pose_before": _pose_xyz_dict(lidar_odom_status.get("last_lidar_pose_before")),
        "last_lidar_pose": _pose_xyz_dict(lidar_odom_status.get("last_lidar_pose")),
        "dx": _maybe_finite(lidar_odom_status.get("dx")),
        "dy": _maybe_finite(lidar_odom_status.get("dy")),
        "dtheta": _maybe_finite(lidar_odom_status.get("dtheta")),
        "x_lidar_raw": _maybe_finite(lidar_odom_status.get("x_lidar_raw")),
        "y_lidar_raw": _maybe_finite(lidar_odom_status.get("y_lidar_raw")),
        "theta_lidar_raw": _maybe_finite(lidar_odom_status.get("theta_lidar_raw")),
        "ekf_pose_before": _pose_xyz_dict(lidar_odom_status.get("ekf_pose_before")),
        "ekf_pose_after": _pose_xyz_dict(lidar_odom_status.get("ekf_pose_after")),
    }


def _truth_surface_from_status(status: Dict[str, Any]) -> Dict[str, Any]:
    truth_surface = _extract_truth_basis(dict(status or {}))
    truth_basis = dict(truth_surface.get("truth_basis") or {})
    return {
        "execution_mode": str(truth_surface.get("execution_mode", "UNKNOWN") or "UNKNOWN"),
        "motion_actual_ssot": str(
            truth_surface.get("motion_actual_ssot", "EKF_POSE_ODOMETRY_SSOT")
            or "EKF_POSE_ODOMETRY_SSOT"
        ),
        "truth_basis": truth_basis,
        "lidar_odom_status_truth": dict(truth_surface.get("lidar_odom_status") or {}),
        "lidar_odom_applied": bool(truth_surface.get("lidar_odom_applied", False)),
        "lidar_odom_latest_age_s": truth_surface.get("lidar_odom_latest_age_s"),
        "lidar_odom_latest_confidence": truth_surface.get("lidar_odom_latest_confidence"),
        "encoder_pose_active_samples": int(truth_surface.get("encoder_pose_active_samples", 0)),
        "arc_inner_track_min_mps": truth_basis.get("arc_inner_track_min_mps", truth_surface.get("arc_inner_track_min_mps")),
        "arc_track_ratio": truth_basis.get("arc_track_ratio", truth_surface.get("arc_track_ratio")),
        "arc_pivot_like_samples": int(
            _safe_float(
                truth_basis.get("arc_pivot_like_samples", truth_surface.get("arc_pivot_like_samples", 0)),
                0.0,
            )
        ),
        "arc_inner_track_positive_ratio": truth_basis.get(
            "arc_inner_track_positive_ratio",
            truth_surface.get("arc_inner_track_positive_ratio"),
        ),
        "arc_sample_count": int(
            _safe_float(
                truth_basis.get("arc_sample_count", truth_surface.get("arc_sample_count", 0)),
                0.0,
            )
        ),
        "turn_primitive_requested": str(truth_surface.get("turn_primitive_requested", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_limited": str(truth_surface.get("turn_primitive_limited", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_executed": str(truth_surface.get("turn_primitive_executed", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_actual": str(truth_surface.get("turn_primitive_actual", "UNKNOWN") or "UNKNOWN"),
        "turn_primitives": dict(truth_surface.get("turn_primitives") or {}),
    }


def _apply_segment_turn_truth(
    truth_surface: Dict[str, Any],
    *,
    actual_average_angular_speed_dps: float,
    heading_change_deg: float,
    effective_progress_m: float,
    expected_heading_delta_deg: float = 0.0,
    target_lateral_m: float = 0.0,
) -> Dict[str, Any]:
    out = dict(truth_surface or {})
    requested = str(out.get("turn_primitive_requested", "") or "").strip().upper()
    limited = str(out.get("turn_primitive_limited", "") or "").strip().upper()
    executed = str(out.get("turn_primitive_executed", "") or "").strip().upper()
    straight_command_chain = bool(
        requested == "STRAIGHT"
        and limited == "STRAIGHT"
        and executed == "STRAIGHT"
    )
    progress = max(1e-6, abs(float(effective_progress_m)))
    heading_per_m = abs(float(heading_change_deg)) / progress
    segment_straight = bool(
        straight_command_chain
        and progress >= 0.20
        and heading_per_m <= float(DEFAULT_STRAIGHT_SEGMENT_HEADING_DEG_PER_M)
        and abs(float(actual_average_angular_speed_dps)) <= float(DEFAULT_STRAIGHT_SEGMENT_AVG_ANGULAR_DPS)
    )
    expected_turn = bool(
        abs(float(expected_heading_delta_deg)) >= 3.0
        or abs(float(target_lateral_m)) >= 0.03
    )
    segment_turn = bool(
        expected_turn
        and progress >= 0.20
        and abs(float(heading_change_deg)) >= 2.0
    )
    if segment_turn:
        curvature_rad_per_m = abs(math.radians(float(heading_change_deg))) / progress
        primitive = "DIFF_ARC_GENTLE" if curvature_rad_per_m <= 2.8 else "DIFF_ARC_SHARP"
        out["turn_primitive_requested"] = primitive
        out["turn_primitive_limited"] = primitive
        out["turn_primitive_executed"] = primitive
        out["turn_primitive_actual"] = primitive
        out["turn_primitives"] = {
            "requested": primitive,
            "limited": primitive,
            "executed": primitive,
            "actual": primitive,
        }
        return out
    if segment_straight:
        out["turn_primitive_actual"] = "STRAIGHT"
        primitives = dict(out.get("turn_primitives") or {})
        primitives["actual"] = "STRAIGHT"
        out["turn_primitives"] = primitives
    return out


def _relative_pose_target(
    start_pose: Dict[str, Any],
    *,
    target_distance_m: float,
    lateral_m: float,
    heading_delta_deg: float,
    v_mps: float,
) -> Dict[str, float]:
    start_theta = float(start_pose.get("theta", 0.0))
    distance_abs = abs(float(target_distance_m))
    lateral = float(lateral_m)
    forward_abs = math.sqrt(max(0.0, (distance_abs * distance_abs) - (lateral * lateral)))
    forward = math.copysign(float(forward_abs), float(target_distance_m) if abs(float(target_distance_m)) > 1e-9 else 1.0)
    target_x = (
        float(start_pose.get("x", 0.0))
        + forward * math.cos(start_theta)
        - lateral * math.sin(start_theta)
    )
    target_y = (
        float(start_pose.get("y", 0.0))
        + forward * math.sin(start_theta)
        + lateral * math.cos(start_theta)
    )
    target_theta = float(start_theta + math.radians(float(heading_delta_deg)))
    return {
        "x": float(target_x),
        "y": float(target_y),
        "theta_rad": float(target_theta),
        "v_max": float(abs(float(v_mps))),
        "forward_m": float(forward),
        "lateral_m": float(lateral),
        "heading_delta_deg": float(heading_delta_deg),
    }


def _write_immutable_suite_artifacts(
    *,
    test_name: str,
    suite_result: Dict[str, Any],
    summary_report: Dict[str, Any],
) -> Dict[str, str]:
    ts_tag = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    slug = _slug_token(test_name)
    immutable_result_path = AGENT_TEST_DIR / f"{slug}_{ts_tag}_result.json"
    immutable_summary_path = AGENT_TEST_DIR / f"{slug}_{ts_tag}_summary.json"
    _write_json_atomic(immutable_result_path, suite_result)
    _write_json_atomic(immutable_summary_path, summary_report)
    return {
        "result": str(immutable_result_path.relative_to(PROJECT_ROOT)),
        "summary": str(immutable_summary_path.relative_to(PROJECT_ROOT)),
    }


def _resolve_lidar_update_counts(
    *,
    lidar_status_summary: Dict[str, Any],
    lidar_diag_summary: Dict[str, Any],
) -> Dict[str, Any]:
    status_accept = int(max(0.0, _safe_float((lidar_status_summary or {}).get("accepted_delta"), 0.0)))
    status_reject = int(
        max(
            0.0,
            _safe_float((lidar_status_summary or {}).get("rejected_large_jump_delta"), 0.0)
            + _safe_float((lidar_status_summary or {}).get("rejected_low_confidence_delta"), 0.0),
        )
    )
    diag_accept = int(max(0.0, _safe_float((lidar_diag_summary or {}).get("odom_accept"), 0.0)))
    diag_reject = int(max(0.0, _safe_float((lidar_diag_summary or {}).get("odom_reject"), 0.0)))

    return {
        "accept_count": int(max(status_accept, diag_accept)),
        "reject_count": int(max(status_reject, diag_reject)),
        "sources": {
            "status_accept_delta": int(status_accept),
            "diag_odom_accept": int(diag_accept),
            "status_reject_delta": int(status_reject),
            "diag_odom_reject": int(diag_reject),
        },
    }


def _build_loop_health_summary(
    start_status_version: int,
    end_status_version: int,
    *,
    status_samples: int,
    stale_stream: bool,
    resolution_observed: bool,
    sources_seen: Counter[str],
    command_types_seen: Counter[str],
    end_status: Dict[str, Any],
) -> Dict[str, Any]:
    watchdog = dict((end_status or {}).get("watchdog") or {})
    loop_budget = dict((end_status or {}).get("loop_budget") or {})
    return {
        "status_samples": int(status_samples),
        "status_version_start": int(start_status_version),
        "status_version_end": int(end_status_version),
        "status_progressed": bool(end_status_version > start_status_version),
        "stale_stream": bool(stale_stream),
        "watchdog_stop_triggered": bool(watchdog.get("stop_triggered", False)),
        "watchdog_freq_hz": float(_safe_float(watchdog.get("freq_hz"), 0.0)),
        "loop_budget": loop_budget,
        "motion_resolution_observed": bool(resolution_observed),
        "resolved_sources_seen": sorted(source for source in sources_seen if source),
        "resolved_command_types_seen": sorted(command_type for command_type in command_types_seen if command_type),
    }


def _extract_motion_public(status: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(status, dict):
        return {}
    data = dict(status.get("motion_public") or {})
    return data if isinstance(data, dict) else {}


def _segment_stop_reason(status: Dict[str, Any]) -> str:
    motion_public = _extract_motion_public(status)
    reason = str(motion_public.get("stop_reason", "") or "").strip()
    if reason:
        return reason
    stop_status = dict((status or {}).get("stop_status") or {})
    reason = str(stop_status.get("reason", "") or "").strip()
    if reason:
        return reason
    stop_type = str(stop_status.get("type", "") or "").strip()
    return stop_type


def _sample_motion_status(
    start_pose: Dict[str, float],
    status: Dict[str, Any],
    *,
    heading_abort_deg: float,
    enforce_heading_abort: bool = True,
) -> Dict[str, Any]:
    pose = _get_pose(status)
    estimated_distance_m = float(_pose_distance(start_pose, pose))
    heading_change_deg = float(_normalize_angle_deg(pose["theta_deg"] - start_pose["theta_deg"]))
    heading_abort_triggered = abs(heading_change_deg) > float(heading_abort_deg)
    if _status_state(status) == "FAILSAFE":
        raise RuntimeError(f"Robot entered FAILSAFE: {(status.get('last_emergency') or {})}")
    if not bool(((status or {}).get("safety") or {}).get("allow", True)):
        raise RuntimeError(f"Safety block active: {(status.get('safety') or {})}")
    if str((status or {}).get("odometry_mode", "") or "").strip().upper() != "LIDAR_FIRST":
        raise RuntimeError(f"odometry_mode changed: {(status or {}).get('odometry_mode')}")
    motion_public = _extract_motion_public(status)
    return {
        "pose": pose,
        "estimated_distance_m": estimated_distance_m,
        "heading_change_deg": heading_change_deg,
        "heading_abort_triggered": bool(heading_abort_triggered),
        "actual_linear_mps": (
            _safe_float(motion_public.get("actual_linear_mps"), _safe_float(pose.get("v"), 0.0))
        ),
        "actual_angular_dps": (
            _safe_float(
                motion_public.get("actual_angular_dps"),
                _safe_float(status.get("omega_target"), 0.0) * RAD_TO_DEG,
            )
        ),
        "cmd_linear_mps": _safe_float(
            motion_public.get("cmd_linear_mps"),
            _safe_float((status.get("motion_public") or {}).get("linear_speed_mps"), _safe_float(status.get("v_target"), 0.0)),
        ),
        "cmd_angular_dps": _safe_float(
            motion_public.get("cmd_angular_dps"),
            _safe_float(status.get("omega_target"), 0.0) * RAD_TO_DEG,
        ),
    }


def _track_velocity_from_twist(v_mps: float, omega_rad_s: float) -> Dict[str, float]:
    half_track = 0.5 * float(DEFAULT_TRACK_WIDTH_M)
    left_mps = float(v_mps) - float(omega_rad_s) * half_track
    right_mps = float(v_mps) + float(omega_rad_s) * half_track
    if abs(float(v_mps)) <= 1e-3 and abs(float(omega_rad_s)) > 1e-6:
        min_abs = float(DEFAULT_MIN_ROTATE_WHEEL_SPEED_MPS)
        if max(abs(left_mps), abs(right_mps)) < min_abs:
            turn_sign = 1.0 if float(omega_rad_s) >= 0.0 else -1.0
            left_mps = -turn_sign * min_abs
            right_mps = turn_sign * min_abs
    return {
        "left_mps": float(left_mps),
        "right_mps": float(right_mps),
    }


def _status_obstacle_snapshot(status: Dict[str, Any]) -> Dict[str, Any]:
    st = dict(status or {})
    lidar = dict(st.get("lidar") or {})
    safety = dict(st.get("safety") or {})
    reason = str(safety.get("reason", "") or "").strip()
    blocked_front = bool(lidar.get("blocked_front", False))
    blocked_back = bool(lidar.get("blocked_back", False))
    reason_low = reason.lower()
    if not blocked_front and ("blocked_front" in reason_low or "obstacle_front" in reason_low):
        blocked_front = True
    if not blocked_back and ("blocked_back" in reason_low or "obstacle_back" in reason_low):
        blocked_back = True

    avg_left = _maybe_finite(lidar.get("avg_left"))
    avg_right = _maybe_finite(lidar.get("avg_right"))
    min_dist = _maybe_finite(lidar.get("min_dist"))
    if avg_left is None:
        avg_left = min_dist
    if avg_right is None:
        avg_right = min_dist

    return {
        "blocked_front": bool(blocked_front),
        "blocked_back": bool(blocked_back),
        "safety_allow": bool(safety.get("allow", True)),
        "safety_reason": reason,
        "avg_left_m": avg_left,
        "avg_right_m": avg_right,
    }


def _pivot_track_targets(
    *,
    turn_left: bool,
    blocked_back: bool,
    pivot_speed_mps: float,
) -> Dict[str, Any]:
    speed = max(float(DEFAULT_MIN_ROTATE_WHEEL_SPEED_MPS), abs(float(pivot_speed_mps)))
    if blocked_back:
        # Keep reverse direction free when rear is blocked.
        if bool(turn_left):
            left_mps = 0.0
            right_mps = float(speed)
        else:
            left_mps = float(speed)
            right_mps = 0.0
        mode = "forward_pivot"
    else:
        # Default: slight backward pivot to open forward clearance.
        if bool(turn_left):
            left_mps = -float(speed)
            right_mps = 0.0
        else:
            left_mps = 0.0
            right_mps = -float(speed)
        mode = "reverse_pivot"
    return {
        "left_mps": float(left_mps),
        "right_mps": float(right_mps),
        "turn_direction": ("LEFT" if bool(turn_left) else "RIGHT"),
        "mode": str(mode),
    }


def _send_track_velocity_with_active_retry(
    *,
    token: str,
    left_mps: float,
    right_mps: float,
    motion_source: str,
    stop_timeout_s: float,
) -> Dict[str, Any]:
    settle_timeout_s = max(1.0, min(6.0, float(stop_timeout_s)))
    max_attempts = 3
    for attempt in range(1, max_attempts + 1):
        try:
            return _send_command_checked(
                "set_track_velocity",
                token=token,
                timeout_s=4.0,
                left_mps=float(left_mps),
                right_mps=float(right_mps),
                motion_source=str(motion_source or "MANUAL"),
            )
        except Exception as exc:
            if "blocked_by_active" not in str(exc):
                raise
            if attempt >= max_attempts:
                raise
            _cancel_motion_stop_with_fallback(str(token), timeout_s=settle_timeout_s)
            try:
                _wait_until_stopped(timeout_s=settle_timeout_s)
            except Exception:
                pass
            time.sleep(0.20 + (0.10 * float(attempt)))
    raise RuntimeError("set_track_velocity_retry_exhausted")


def _attempt_obstacle_pivot_recovery(
    *,
    token: str,
    status: Dict[str, Any],
    motion_source: str,
    prefer_left: bool,
    pivot_speed_mps: float = DEFAULT_OBSTACLE_PIVOT_SPEED_MPS,
    pivot_duration_s: float = DEFAULT_OBSTACLE_PIVOT_DURATION_S,
    stop_timeout_s: float = DEFAULT_STOP_TIMEOUT_S,
) -> Dict[str, Any]:
    before = _status_obstacle_snapshot(status)
    if not bool(before.get("blocked_front", False)):
        return {
            "attempted": False,
            "recovered": True,
            "before": dict(before),
            "after": dict(before),
            "pivot": {},
            "command": {},
            "stop_outcome": {},
            "error": "",
        }

    pivot = _pivot_track_targets(
        turn_left=bool(prefer_left),
        blocked_back=bool(before.get("blocked_back", False)),
        pivot_speed_mps=float(pivot_speed_mps),
    )
    sent_cmd: Dict[str, Any] = {}
    error_text = ""
    status_after_move = dict(status or {})
    try:
        sent_cmd = _send_track_velocity_with_active_retry(
            token=str(token),
            left_mps=float(pivot.get("left_mps", 0.0)),
            right_mps=float(pivot.get("right_mps", 0.0)),
            motion_source=str(motion_source or "MANUAL"),
            stop_timeout_s=float(stop_timeout_s),
        )
        move_deadline = time.monotonic() + max(0.15, float(pivot_duration_s))
        while time.monotonic() <= move_deadline:
            st = _read_json(STATUS_PATH)
            if st:
                status_after_move = dict(st)
                if not bool(_status_obstacle_snapshot(st).get("blocked_front", False)):
                    break
            time.sleep(float(DEFAULT_POLL_S))
    except Exception as exc:
        error_text = str(exc)

    stop_outcome = _bounded_stop_with_fallback(
        str(token),
        timeout_s=max(1.5, float(stop_timeout_s)),
        motion_source=str(motion_source or "MANUAL"),
    )
    status_after = dict(stop_outcome.get("status") or status_after_move or {})
    after = _status_obstacle_snapshot(status_after)

    return {
        "attempted": True,
        "recovered": not bool(after.get("blocked_front", False)),
        "before": dict(before),
        "after": dict(after),
        "pivot": dict(pivot),
        "command": dict(sent_cmd or {}),
        "stop_outcome": {
            "normal_stop_used": bool(stop_outcome.get("normal_stop_used", False)),
            "failsafe_triggered": bool(stop_outcome.get("failsafe_triggered", False)),
            "emergency_stop_triggered": bool(stop_outcome.get("emergency_stop_triggered", False)),
            "details": dict(stop_outcome.get("details") or {}),
        },
        "status_after": dict(status_after or {}),
        "error": str(error_text or ""),
    }


def _resolve_min_progress_m(
    *,
    target_distance_m: float,
    speed_mps: float,
    max_runtime_s: float,
    configured_min_progress_m: float,
    min_progress_ratio: float,
) -> float:
    configured = float(configured_min_progress_m)
    if configured > 0.0:
        return configured
    target = max(0.0, abs(float(target_distance_m)))
    nominal_by_speed = max(0.0, abs(float(speed_mps)) * max(0.0, float(max_runtime_s)))
    nominal_window = nominal_by_speed if nominal_by_speed > 1e-6 else target
    if target > 1e-6:
        nominal_window = min(nominal_window, target)
    ratio = max(0.05, min(0.90, float(min_progress_ratio)))
    dynamic_min = nominal_window * ratio
    dynamic_min = max(0.01, dynamic_min)
    if target > 1e-6:
        dynamic_min = min(dynamic_min, max(0.01, target * 0.90))
    return float(dynamic_min)


def _resolve_target_completion_m(
    *,
    target_distance_m: float,
    min_progress_m: float,
    completion_ratio: float,
) -> float:
    target = max(0.0, abs(float(target_distance_m)))
    min_progress = max(0.0, float(min_progress_m))
    ratio = max(0.0, min(1.0, float(completion_ratio)))
    if target <= 1e-6 or ratio <= 1e-6:
        return float(min_progress)
    return float(max(min_progress, target * ratio))


def _compute_command_motion_consistency(
    *,
    start_cmd: Dict[str, Any],
    stop_cmd: Dict[str, Any],
    commanded_linear_speed_mps: float,
    estimated_distance_m: float,
    command_profile_events: Optional[List[Dict[str, Any]]] = None,
    token: str = "",
) -> Dict[str, Any]:
    start_ts = _maybe_finite((start_cmd or {}).get("sent_ts_wall"))
    stop_ts = _maybe_finite((stop_cmd or {}).get("sent_ts_wall"))
    command_window_s = 0.0
    if start_ts is not None and stop_ts is not None and stop_ts >= start_ts:
        command_window_s = max(0.0, float(stop_ts) - float(start_ts))

    commanded_speed = max(0.0, abs(float(commanded_linear_speed_mps)))
    initial_setpoint_distance_m = commanded_speed * command_window_s
    odom_distance_m = max(0.0, abs(float(estimated_distance_m)))

    def _extract_linear_mps(entry: Dict[str, Any]) -> float | None:
        row = dict(entry or {})
        cmd_type = str(row.get("cmd_type") or row.get("type") or "").strip().lower()
        if cmd_type == "set_track_velocity":
            l = _maybe_finite(row.get("left_mps"))
            r = _maybe_finite(row.get("right_mps"))
            if l is None or r is None:
                return None
            return float(0.5 * (float(l) + float(r)))
        if cmd_type == "set_twist":
            v = _maybe_finite(row.get("v"))
            if v is None:
                return None
            return float(v)
        if cmd_type == "set_target_pose":
            v_nom = _maybe_finite(row.get("v_nominal_mps"))
            if v_nom is None:
                return None
            return float(v_nom)
        if cmd_type == "go_to_pose":
            v_nom = _maybe_finite(row.get("v_nominal_mps"))
            if v_nom is None:
                return None
            return float(v_nom)
        if cmd_type in ("stop", "emergency_stop", "strong_reset", "full_reset"):
            return 0.0
        return None

    def _normalize_profile_events(events: Optional[List[Dict[str, Any]]]) -> List[Dict[str, float]]:
        normalized: List[Dict[str, float]] = []
        for entry in list(events or []):
            ts = _maybe_finite((entry or {}).get("ts"))
            if ts is None:
                continue
            linear_mps = _extract_linear_mps(dict(entry or {}))
            if linear_mps is None or not math.isfinite(linear_mps):
                continue
            normalized.append(
                {
                    "ts": float(ts),
                    "linear_mps": float(linear_mps),
                }
            )
        normalized.sort(key=lambda item: float(item["ts"]))
        return normalized

    def _fallback_profile_from_commands_jsonl() -> List[Dict[str, float]]:
        if not COMMANDS_PATH.exists():
            return []
        rows: List[Dict[str, Any]] = []
        try:
            with COMMANDS_PATH.open("r", encoding="utf-8") as handle:
                for raw in handle:
                    raw = raw.strip()
                    if not raw:
                        continue
                    try:
                        row = json.loads(raw)
                    except Exception:
                        continue
                    ts = _maybe_finite((row or {}).get("ts"))
                    if ts is None:
                        continue
                    rows.append(dict(row))
        except Exception:
            return []
        if not rows:
            return []

        start_id = str((start_cmd or {}).get("cmd_id") or "")
        stop_id = str((stop_cmd or {}).get("cmd_id") or "")
        start_idx = -1
        stop_idx = -1
        if start_id and stop_id:
            for idx, row in enumerate(rows):
                if start_idx < 0 and str(row.get("cmd_id", "")) == start_id:
                    start_idx = idx
                if start_idx >= 0 and str(row.get("cmd_id", "")) == stop_id:
                    stop_idx = idx
                    break

        selected: List[Dict[str, Any]] = []
        if start_idx >= 0 and stop_idx >= start_idx:
            selected = rows[start_idx : stop_idx + 1]
        elif start_ts is not None and stop_ts is not None:
            lo = float(start_ts) - 0.25
            hi = float(stop_ts) + 0.25
            cmd_prefix = str(start_id.split("_")[0] if start_id else "lidar1m")
            selected = []
            for row in rows:
                ts = _maybe_finite(row.get("ts"))
                if ts is None or ts < lo or ts > hi:
                    continue
                row_token = str(row.get("token") or "")
                row_cmd_id = str(row.get("cmd_id") or "")
                if token and row_token != str(token):
                    continue
                if row_cmd_id and cmd_prefix and not row_cmd_id.startswith(cmd_prefix):
                    continue
                selected.append(row)
        return _normalize_profile_events(selected)

    def _integrate_effective_distance(
        *,
        events: List[Dict[str, float]],
        start_wall_ts: float,
        stop_wall_ts: float,
        default_linear_mps: float,
    ) -> Dict[str, float]:
        window_s = max(0.0, float(stop_wall_ts) - float(start_wall_ts))
        if window_s <= 1e-9:
            return {
                "distance_m": 0.0,
                "mean_linear_mps": 0.0,
            }
        cursor = float(start_wall_ts)
        current_linear = float(default_linear_mps)
        distance_acc = 0.0
        for item in events:
            ts = float(item.get("ts", 0.0))
            if ts <= cursor:
                current_linear = float(item.get("linear_mps", current_linear))
                continue
            if ts >= float(stop_wall_ts):
                break
            dt = max(0.0, ts - cursor)
            if dt > 0.0:
                distance_acc += abs(float(current_linear)) * dt
            cursor = ts
            current_linear = float(item.get("linear_mps", current_linear))
        if float(stop_wall_ts) > cursor:
            distance_acc += abs(float(current_linear)) * (float(stop_wall_ts) - cursor)
        mean_linear = distance_acc / window_s if window_s > 1e-9 else 0.0
        return {
            "distance_m": float(distance_acc),
            "mean_linear_mps": float(mean_linear),
        }

    profile_source = "probe_stream"
    profile_events = _normalize_profile_events(command_profile_events)
    if not profile_events:
        profile_events = _fallback_profile_from_commands_jsonl()
        profile_source = "commands_jsonl"

    effective_command_distance_m = 0.0
    time_weighted_mean_linear_mps = 0.0
    if start_ts is not None and stop_ts is not None and stop_ts >= start_ts:
        default_linear_for_integration = float(commanded_linear_speed_mps) if profile_events else 0.0
        integrated = _integrate_effective_distance(
            events=profile_events,
            start_wall_ts=float(start_ts),
            stop_wall_ts=float(stop_ts),
            default_linear_mps=float(default_linear_for_integration),
        )
        effective_command_distance_m = float(integrated.get("distance_m", 0.0))
        time_weighted_mean_linear_mps = float(integrated.get("mean_linear_mps", 0.0))

    if effective_command_distance_m <= 1e-6 and initial_setpoint_distance_m > 1e-6:
        effective_command_distance_m = float(initial_setpoint_distance_m)
        time_weighted_mean_linear_mps = (
            float(effective_command_distance_m) / float(command_window_s)
            if command_window_s > 1e-6
            else 0.0
        )
        profile_source = "initial_setpoint_fallback"

    odom_vs_initial_setpoint_ratio = None
    if initial_setpoint_distance_m > 1e-6:
        odom_vs_initial_setpoint_ratio = float(odom_distance_m) / float(initial_setpoint_distance_m)

    odom_vs_effective_command_ratio = None
    if effective_command_distance_m > 1e-6:
        odom_vs_effective_command_ratio = float(odom_distance_m) / float(effective_command_distance_m)

    return {
        "command_window_s": float(command_window_s),
        "command_profile_source": str(profile_source),
        "command_profile_event_count": int(len(profile_events)),
        "initial_setpoint_distance_m": float(initial_setpoint_distance_m),
        "effective_command_distance_m": float(effective_command_distance_m),
        "command_nominal_distance_m": float(effective_command_distance_m),
        "odom_distance_m": float(odom_distance_m),
        "odom_vs_initial_setpoint_ratio": (
            None if odom_vs_initial_setpoint_ratio is None else float(odom_vs_initial_setpoint_ratio)
        ),
        "odom_vs_effective_command_ratio": (
            None if odom_vs_effective_command_ratio is None else float(odom_vs_effective_command_ratio)
        ),
        "odom_vs_command_ratio": (
            None if odom_vs_effective_command_ratio is None else float(odom_vs_effective_command_ratio)
        ),
        "time_weighted_mean_linear_mps": float(time_weighted_mean_linear_mps),
    }


def _run_preflight(
    token: str,
    *,
    stop_timeout_s: float,
    required_clearance_m: float,
    forward_clearance_mode: str = DEFAULT_FORWARD_CLEARANCE_MODE,
) -> Dict[str, Any]:
    status = _wait_for_status_progress(min_increments=2, timeout_s=5.0)
    _wait_for_lidar_scan_progress(min_increments=2, timeout_s=3.0)

    startup = dict(status.get("startup") or {})
    if not bool(startup.get("ready", False)):
        raise RuntimeError(f"startup.ready is false: {startup}")

    failsafe_before_reset = _status_state(status) == "FAILSAFE"
    reset_result = None
    if failsafe_before_reset:
        reset_result = _strong_reset_to_idle(token, timeout_s=DEFAULT_RESET_TIMEOUT_S)
        status = dict(reset_result.get("status") or {})

    normal_stop_validation = _normal_stop(token, timeout_s=stop_timeout_s)
    status = dict(normal_stop_validation.get("status") or {})
    preflight_obstacle_recovery_events: List[Dict[str, Any]] = []
    normalized_clearance_mode = str(
        forward_clearance_mode or DEFAULT_FORWARD_CLEARANCE_MODE
    ).strip().lower()
    if normalized_clearance_mode == "straight-corridor":
        clearance = _sample_forward_corridor_clearance(
            sample_s=0.8,
            poll_s=0.05,
        )
    elif normalized_clearance_mode == "front-sector":
        clearance = _sample_forward_clearance(sample_s=0.8, poll_s=0.05)
    else:
        raise RuntimeError(
            f"Unknown forward clearance mode: {forward_clearance_mode}"
        )
    median_clearance_m = clearance.get("min_dist_median_m")
    if median_clearance_m is None or float(median_clearance_m) < float(required_clearance_m):
        obstacle_status = _read_json(STATUS_PATH) or dict(status or {})
        last_prefer_left: bool | None = None
        for attempt in range(1, int(DEFAULT_OBSTACLE_PIVOT_MAX_ATTEMPTS) + 1):
            obstacle_snapshot = _status_obstacle_snapshot(obstacle_status)
            if not bool(obstacle_snapshot.get("blocked_front", False)):
                break
            avg_left = _maybe_finite(obstacle_snapshot.get("avg_left_m"))
            avg_right = _maybe_finite(obstacle_snapshot.get("avg_right_m"))
            if avg_left is not None and avg_right is not None:
                prefer_left = bool(float(avg_left) >= float(avg_right))
            elif last_prefer_left is not None:
                prefer_left = not bool(last_prefer_left)
            else:
                prefer_left = True
            last_prefer_left = bool(prefer_left)
            recovery = _attempt_obstacle_pivot_recovery(
                token=str(token),
                status=dict(obstacle_status or {}),
                motion_source="MANUAL",
                prefer_left=bool(prefer_left),
                pivot_speed_mps=float(DEFAULT_OBSTACLE_PIVOT_SPEED_MPS),
                pivot_duration_s=float(DEFAULT_OBSTACLE_PIVOT_DURATION_S),
                stop_timeout_s=float(stop_timeout_s),
            )
            preflight_obstacle_recovery_events.append(
                {
                    "attempt": int(attempt),
                    "timestamp": _now_iso(),
                    **dict(recovery),
                }
            )
            obstacle_status = dict(recovery.get("status_after") or obstacle_status or {})
            status = dict(obstacle_status or status or {})
            if normalized_clearance_mode == "straight-corridor":
                clearance = _sample_forward_corridor_clearance(
                    sample_s=0.8,
                    poll_s=0.05,
                )
            else:
                clearance = _sample_forward_clearance(
                    sample_s=0.8,
                    poll_s=0.05,
                )
            median_clearance_m = clearance.get("min_dist_median_m")
            if median_clearance_m is not None and float(median_clearance_m) >= float(required_clearance_m):
                break

        if median_clearance_m is None or float(median_clearance_m) < float(required_clearance_m):
            raise RuntimeError(
                f"Forward clearance too small ({median_clearance_m}) for required {required_clearance_m:.3f} m."
            )

    motion_resolution = extract_motion_resolution(status)
    if not motion_resolution["observable"]:
        raise RuntimeError("Resolved motion owner is not observable in runtime status.")

    lidar_state = _resolve_lidar_preflight_state(status)
    lidar_odom_status = dict(lidar_state.get("lidar_odom_status") or {})
    latest_age_s = float(_safe_float(lidar_state.get("latest_age_s"), math.inf))
    candidate_age_s = float(_safe_float(lidar_state.get("candidate_age_s"), math.inf))
    candidate_available = bool(lidar_state.get("candidate_available", False))
    lidar_candidate_fresh = bool(lidar_state.get("candidate_fresh", False))
    lidar_latest_fresh = bool(lidar_state.get("latest_fresh", False))
    lidar_quality_gate_ok = bool(lidar_state.get("quality_gate_ok", False))
    lidar_signal_quality = dict(lidar_state.get("signal_quality") or {})
    lidar_feed_ready = bool(lidar_state.get("feed_ready", False))
    if _is_lidar_scan_warmup_transient(lidar_signal_quality):
        deadline = time.monotonic() + float(DEFAULT_PREFLIGHT_STRICT_SCAN_WARMUP_TIMEOUT_S)
        while time.monotonic() <= deadline:
            time.sleep(float(DEFAULT_PREFLIGHT_STRICT_SCAN_WARMUP_POLL_S))
            _read_json(LIDAR_SCAN_PATH)
            refreshed_status = _read_json(STATUS_PATH)
            if refreshed_status:
                status = dict(refreshed_status)
            lidar_state = _resolve_lidar_preflight_state(status)
            lidar_odom_status = dict(lidar_state.get("lidar_odom_status") or {})
            latest_age_s = float(_safe_float(lidar_state.get("latest_age_s"), math.inf))
            candidate_age_s = float(_safe_float(lidar_state.get("candidate_age_s"), math.inf))
            candidate_available = bool(lidar_state.get("candidate_available", False))
            lidar_candidate_fresh = bool(lidar_state.get("candidate_fresh", False))
            lidar_latest_fresh = bool(lidar_state.get("latest_fresh", False))
            lidar_quality_gate_ok = bool(lidar_state.get("quality_gate_ok", False))
            lidar_signal_quality = dict(lidar_state.get("signal_quality") or {})
            lidar_feed_ready = bool(lidar_state.get("feed_ready", False))
            if bool(lidar_signal_quality.get("ok", False)):
                break
            if not _is_lidar_scan_warmup_transient(lidar_signal_quality):
                break
    if not lidar_feed_ready:
        raise RuntimeError(
            "LiDAR odometry feed is stale "
            f"(latest_age_s={latest_age_s}, candidate_available={candidate_available}, candidate_age_s={candidate_age_s})."
        )
    if not bool(lidar_signal_quality.get("ok", False)):
        raise RuntimeError(
            "LiDAR strict quality is insufficient "
            f"(scan={lidar_signal_quality.get('scan_count_filtered')}/{lidar_signal_quality.get('scan_count_required')}, "
            f"matcher_called={lidar_signal_quality.get('matcher_called')}, "
            f"accepted_total={lidar_signal_quality.get('accepted_total')}, "
            f"signal_confidence={float(lidar_signal_quality.get('signal_confidence', 0.0)):.3f}/"
            f"{float(lidar_signal_quality.get('confidence_required', 0.0)):.3f}, "
            f"reason={lidar_signal_quality.get('matcher_reason')})."
        )

    if str(status.get("odometry_mode", "") or "").strip().upper() != "LIDAR_FIRST":
        raise RuntimeError(f"odometry_mode is not LIDAR_FIRST: {status.get('odometry_mode')}")
    encoder_pose_active = bool(_encoder_pose_used(status))
    if not bool(status.get("safety")):
        raise RuntimeError("Safety state missing from runtime status.")
    if not bool(load_log_switches().get("telemetry", False)):
        raise RuntimeError("Telemetry logging is disabled in logs/latest/runtime/log_switches.json.")
    if not bool(_read_json(LIDAR_SCAN_PATH).get("scan")):
        raise RuntimeError("Raw LiDAR scan data is unavailable.")

    encoder_distance = _extract_encoder_distance(status)
    recommended_mode = "LIDAR_STRICT"
    expected_determinism = "HIGH" if lidar_quality_gate_ok else "LOW"

    truth_surface = _truth_surface_from_status(status)
    return {
        "timestamp": _now_iso(),
        "ready": True,
        "can_move_now": True,
        "blocking_issues": [],
        "expected_determinism": expected_determinism,
        "recommended_mode": recommended_mode,
        "controller_runtime_running": True,
        "startup_ready": True,
        "failsafe_before_reset": bool(failsafe_before_reset),
        "failsafe_reset_performed": bool(reset_result is not None),
        "odometry_mode": str(status.get("odometry_mode", "")),
        "encoder_used_for_pose": bool(encoder_pose_active),
        "lidar_fresh": bool(lidar_feed_ready),
        "lidar_quality_gate": {
            "ok": bool(lidar_feed_ready),
            "feed_ready": bool(lidar_feed_ready),
            "signal_quality_ok": bool(lidar_signal_quality.get("ok", False)),
            "candidate_fresh": bool(lidar_candidate_fresh),
            "latest_fresh": bool(lidar_latest_fresh),
        },
        "encoder_distance_available": bool(encoder_distance.get("available", False)),
        "encoder_distance": dict(encoder_distance),
        "lidar_latest_age_s": float(latest_age_s),
        "lidar_candidate_fresh": bool(lidar_candidate_fresh),
        "lidar_candidate_age_s": float(candidate_age_s),
        "lidar_signal_quality": dict(lidar_signal_quality),
        "logging_active": True,
        "safety_path_active": bool(status.get("safety")),
        "command_path_alive": True,
        "resolved_motion_observable": True,
        "normal_stop_validation": {
            "success": True,
            "stop_type": _stop_status_type(status),
            "resolved_motion_source": motion_resolution["resolved_source"],
        },
        "obstacle_recovery": {
            "attempts": int(len(preflight_obstacle_recovery_events)),
            "events": list(preflight_obstacle_recovery_events),
            "max_attempts": int(DEFAULT_OBSTACLE_PIVOT_MAX_ATTEMPTS),
        },
        "forward_clearance": clearance,
        "forward_clearance_mode": normalized_clearance_mode,
        "reset_result": reset_result or {},
        **truth_surface,
    }


def _run_bounded_motion_test(
    name: str,
    *,
    token: str,
    v_mps: float,
    omega_rad_s: float,
    target_distance_m: float,
    min_progress_m: float,
    max_runtime_s: float,
    keepalive_interval_s: float,
    stop_timeout_s: float,
    heading_abort_deg: float,
    enforce_heading_abort: bool = True,
    enable_probe_heading_hold: bool = True,
    target_completion_ratio: float = DEFAULT_FORWARD_TARGET_COMPLETION_RATIO,
    min_lidar_accept_count: int = DEFAULT_MIN_LIDAR_ACCEPT_COUNT,
    max_lidar_reject_count: int = DEFAULT_MAX_LIDAR_REJECT_COUNT,
    min_odom_vs_command_ratio: float = DEFAULT_MIN_ODOM_VS_COMMAND_RATIO,
    enforce_progress_guard: bool = True,
    use_pose_closed_loop: bool = False,
    pose_target_lateral_m: float = 0.0,
    pose_target_heading_delta_deg: float = 0.0,
    pose_target_heading_tolerance_deg: float = DEFAULT_POSE_TARGET_HEADING_TOLERANCE_DEG,
    pose_target_omega_max_rad_s: float = 0.0,
    pose_target_handoff_completion_ratio: float = 0.0,
    stop_after_motion: bool = True,
) -> Dict[str, Any]:
    start_wall = time.monotonic()
    start_status = _wait_for_status(timeout_s=2.0)
    start_pose = _get_pose(start_status)
    start_encoder_distance = _extract_encoder_distance(start_status)
    start_emergency_count = int(_safe_float(((start_status.get("last_emergency") or {}).get("count")), 0.0))
    start_status_version = _status_version(start_status)
    commanded_v_mps = float(v_mps)
    commanded_omega_rad_s = float(omega_rad_s)
    segment_motion_source = "STATE" if bool(use_pose_closed_loop) else "MANUAL"
    track_velocity = _track_velocity_from_twist(commanded_v_mps, commanded_omega_rad_s)
    command_profile_events: List[Dict[str, Any]] = []
    pose_target: Dict[str, float] = {}

    def _record_profile_event(cmd_type: str, *, ts_wall: Optional[float] = None, **payload: Any) -> None:
        if ts_wall is None:
            ts = time.time()
        else:
            ts = _safe_float(ts_wall, math.nan)
        if not math.isfinite(float(ts)):
            return
        row = {
            "ts": float(ts),
            "cmd_type": str(cmd_type),
        }
        row.update(payload)
        command_profile_events.append(row)

    if bool(use_pose_closed_loop):
        # Use the public primitive so the probe follows the same AMR motion path as production.
        pose_target = _relative_pose_target(
            start_pose,
            target_distance_m=float(target_distance_m),
            lateral_m=float(pose_target_lateral_m),
            heading_delta_deg=float(pose_target_heading_delta_deg),
            v_mps=float(v_mps),
        )
        start_cmd = _send_command_checked(
            "go_to_pose",
            token=token,
            timeout_s=4.0,
            x=float(pose_target["x"]),
            y=float(pose_target["y"]),
            theta_rad=float(pose_target["theta_rad"]),
            motion_source=segment_motion_source,
            v_max=float(abs(float(v_mps))),
            omega_max=float(pose_target_omega_max_rad_s) if float(pose_target_omega_max_rad_s) > 0.0 else None,
        )
        _record_profile_event(
            "go_to_pose",
            ts_wall=_maybe_finite(start_cmd.get("sent_ts_wall")),
            x=float(pose_target["x"]),
            y=float(pose_target["y"]),
            theta_rad=float(pose_target["theta_rad"]),
            v_nominal_mps=float(pose_target["v_max"]),
            omega_max_rad_s=(float(pose_target_omega_max_rad_s) if float(pose_target_omega_max_rad_s) > 0.0 else None),
            cmd_id=start_cmd.get("cmd_id"),
            motion_source=segment_motion_source,
        )
    else:
        start_cmd = _send_command_checked(
            "set_track_velocity",
            token=token,
            timeout_s=4.0,
            left_mps=float(track_velocity["left_mps"]),
            right_mps=float(track_velocity["right_mps"]),
            motion_source=segment_motion_source,
        )
        _record_profile_event(
            "set_track_velocity",
            ts_wall=_maybe_finite(start_cmd.get("sent_ts_wall")),
            left_mps=float(track_velocity["left_mps"]),
            right_mps=float(track_velocity["right_mps"]),
            motion_source=segment_motion_source,
            cmd_id=start_cmd.get("cmd_id"),
        )

    resolved_sources_seen: Counter[str] = Counter()
    resolved_command_types_seen: Counter[str] = Counter()
    status_samples = 0
    applied_samples = 0
    stale_stream = False
    last_status_change = time.monotonic()
    last_status_version = start_status_version
    last_keepalive = time.monotonic()
    latest_status = start_status
    max_distance_m = 0.0
    encoder_reference_m = _maybe_finite(start_encoder_distance.get("distance_m"))
    encoder_reference_source = str(start_encoder_distance.get("source") or "")
    encoder_progress_samples = 0
    max_encoder_progress_m = 0.0
    move_error = ""
    timed_out_without_target = False
    terminal_guidance_active = False
    terminal_mode_entered = False
    terminal_brake_distance_m = float(
        max(
            float(DEFAULT_TERMINAL_GUIDANCE_STOP_BUFFER_M) + 0.03,
            min(
                0.35,
                max(
                    float(DEFAULT_TERMINAL_GUIDANCE_BRAKE_DISTANCE_M),
                    abs(float(v_mps)) * 1.8,
                ),
            ),
        )
    )
    terminal_stop_buffer_m = float(
        min(
            0.10,
            max(
                float(DEFAULT_TERMINAL_GUIDANCE_STOP_BUFFER_M),
                abs(float(v_mps)) * 0.45,
            ),
        )
    )
    heading_hold_active = bool(
        # Pose closed-loop already owns heading regulation. Injecting MANUAL heading-hold
        # on top would create a parallel writer for omega and can destabilize the path.
        (not bool(use_pose_closed_loop))
        and
        bool(enable_probe_heading_hold)
        and
        float(target_distance_m) >= float(DEFAULT_HEADING_HOLD_MIN_TARGET_M)
        and abs(float(v_mps)) > 1e-6
        and abs(float(omega_rad_s)) <= 1e-6
    )
    heading_hold_engaged = False
    heading_abort_triggered_any = False
    heading_abort_streak = 0
    max_heading_abs_deg = 0.0
    command_progress_guard_triggered = False
    encoder_progress_guard_triggered = False
    command_progress_guard_event: Dict[str, Any] = {}
    encoder_progress_guard_event: Dict[str, Any] = {}
    obstacle_recovery_attempts = 0
    obstacle_recovery_events: List[Dict[str, Any]] = []
    obstacle_recovery_last_mono = 0.0
    obstacle_recovery_last_prefer_left: bool | None = None
    max_command_progress_m = 0.0
    observed_command_progress_m = 0.0
    command_progress_prev_ts: float | None = None
    actual_linear_samples: List[float] = []
    actual_angular_samples_dps: List[float] = []
    cmd_linear_samples: List[float] = []
    cmd_angular_samples_dps: List[float] = []
    required_completion_m = _resolve_target_completion_m(
        target_distance_m=float(target_distance_m),
        min_progress_m=float(min_progress_m),
        completion_ratio=float(target_completion_ratio),
    )
    handoff_completion_m = _resolve_target_completion_m(
        target_distance_m=float(target_distance_m),
        min_progress_m=0.0,
        completion_ratio=(
            float(pose_target_handoff_completion_ratio)
            if float(pose_target_handoff_completion_ratio) > 1e-6
            else float(target_completion_ratio)
        ),
    )
    amr_progress_guard_active = bool(
        abs(float(v_mps)) > 1e-6 and float(target_distance_m) >= float(DEFAULT_AMR_PROGRESS_GUARD_MIN_TARGET_M)
    )
    command_progress_guard_limit_m = (
        max(
            float(target_distance_m) * float(DEFAULT_AMR_COMMAND_PROGRESS_GUARD_RATIO),
            float(required_completion_m) + 0.05,
        )
        if amr_progress_guard_active
        else None
    )
    encoder_progress_guard_limit_m = (
        float(target_distance_m) * float(DEFAULT_AMR_ENCODER_PROGRESS_GUARD_RATIO)
        if amr_progress_guard_active
        else None
    )

    diag_tracker = LidarDiagTracker()

    try:
        deadline = time.monotonic() + max(0.1, float(max_runtime_s))
        while time.monotonic() <= deadline:
            now_mono = time.monotonic()
            if (not bool(use_pose_closed_loop)) and ((now_mono - last_keepalive) >= max(0.05, float(keepalive_interval_s))):
                keepalive_ts_wall = time.time()
                _append_command(
                    "set_track_velocity",
                    token=token,
                    left_mps=float(track_velocity["left_mps"]),
                    right_mps=float(track_velocity["right_mps"]),
                    motion_source=segment_motion_source,
                )
                _record_profile_event(
                    "set_track_velocity",
                    ts_wall=keepalive_ts_wall,
                    left_mps=float(track_velocity["left_mps"]),
                    right_mps=float(track_velocity["right_mps"]),
                    motion_source=segment_motion_source,
                )
                last_keepalive = now_mono

            status = _read_json(STATUS_PATH)
            if status:
                latest_status = status
                diag_tracker.sample(status)
                status_samples += 1
                status_version = _status_version(status)
                if status_version != last_status_version:
                    last_status_version = status_version
                    last_status_change = now_mono

                obstacle = _status_obstacle_snapshot(status)
                if bool(obstacle.get("blocked_front", False)):
                    recovery_cooldown_ok = (
                        float(now_mono) - float(obstacle_recovery_last_mono)
                    ) >= float(DEFAULT_OBSTACLE_RECOVERY_COOLDOWN_S)
                    if (
                        recovery_cooldown_ok
                        and int(obstacle_recovery_attempts) < int(DEFAULT_OBSTACLE_PIVOT_MAX_ATTEMPTS)
                    ):
                        avg_left = _maybe_finite(obstacle.get("avg_left_m"))
                        avg_right = _maybe_finite(obstacle.get("avg_right_m"))
                        if avg_left is not None and avg_right is not None:
                            prefer_left = bool(float(avg_left) >= float(avg_right))
                        elif obstacle_recovery_last_prefer_left is not None:
                            prefer_left = not bool(obstacle_recovery_last_prefer_left)
                        else:
                            prefer_left = True

                        obstacle_recovery_attempts += 1
                        obstacle_recovery_last_mono = float(now_mono)
                        obstacle_recovery_last_prefer_left = bool(prefer_left)
                        recovery = _attempt_obstacle_pivot_recovery(
                            token=str(token),
                            status=dict(status),
                            motion_source=str(segment_motion_source),
                            prefer_left=bool(prefer_left),
                            pivot_speed_mps=float(DEFAULT_OBSTACLE_PIVOT_SPEED_MPS),
                            pivot_duration_s=float(DEFAULT_OBSTACLE_PIVOT_DURATION_S),
                            stop_timeout_s=float(stop_timeout_s),
                        )
                        obstacle_recovery_events.append(
                            {
                                "attempt": int(obstacle_recovery_attempts),
                                "timestamp": _now_iso(),
                                **dict(recovery),
                            }
                        )
                        latest_status = dict(recovery.get("status_after") or latest_status or {})
                        if bool(recovery.get("recovered", False)):
                            if bool(use_pose_closed_loop):
                                try:
                                    resume_cmd = _send_command_checked(
                                        "go_to_pose",
                                        token=token,
                                        timeout_s=4.0,
                                        x=float(pose_target.get("x", 0.0)),
                                        y=float(pose_target.get("y", 0.0)),
                                        theta_rad=float(pose_target.get("theta_rad", 0.0)),
                                        v_max=float(pose_target.get("v_max", abs(float(v_mps)))),
                                        omega_max=(
                                            float(pose_target_omega_max_rad_s)
                                            if float(pose_target_omega_max_rad_s) > 0.0
                                            else None
                                        ),
                                        motion_source=str(segment_motion_source),
                                    )
                                    _record_profile_event(
                                        "go_to_pose",
                                        ts_wall=_maybe_finite(resume_cmd.get("sent_ts_wall")),
                                        x=float(pose_target.get("x", 0.0)),
                                        y=float(pose_target.get("y", 0.0)),
                                        theta_rad=float(pose_target.get("theta_rad", 0.0)),
                                        v_nominal_mps=float(pose_target.get("v_max", abs(float(v_mps)))),
                                        omega_max_rad_s=(
                                            float(pose_target_omega_max_rad_s)
                                            if float(pose_target_omega_max_rad_s) > 0.0
                                            else None
                                        ),
                                        cmd_id=resume_cmd.get("cmd_id"),
                                        motion_source=segment_motion_source,
                                        obstacle_recovery_resume=True,
                                    )
                                except Exception as resume_exc:
                                    move_error = f"obstacle_recovery_resume_failed:{resume_exc}"
                                    break
                            else:
                                try:
                                    resume_cmd = _send_track_velocity_with_active_retry(
                                        token=str(token),
                                        left_mps=float(track_velocity["left_mps"]),
                                        right_mps=float(track_velocity["right_mps"]),
                                        motion_source=str(segment_motion_source),
                                        stop_timeout_s=float(stop_timeout_s),
                                    )
                                    _record_profile_event(
                                        "set_track_velocity",
                                        ts_wall=_maybe_finite(resume_cmd.get("sent_ts_wall")),
                                        left_mps=float(track_velocity["left_mps"]),
                                        right_mps=float(track_velocity["right_mps"]),
                                        motion_source=segment_motion_source,
                                        cmd_id=resume_cmd.get("cmd_id"),
                                        obstacle_recovery_resume=True,
                                    )
                                    last_keepalive = now_mono
                                except Exception as resume_exc:
                                    move_error = f"obstacle_recovery_resume_failed:{resume_exc}"
                                    break
                        else:
                            if int(obstacle_recovery_attempts) >= int(DEFAULT_OBSTACLE_PIVOT_MAX_ATTEMPTS):
                                move_error = (
                                    "obstacle_recovery_exhausted:"
                                    f"blocked_front_after_{int(obstacle_recovery_attempts)}_pivot_attempts"
                                )
                                break
                        continue
                    if int(obstacle_recovery_attempts) >= int(DEFAULT_OBSTACLE_PIVOT_MAX_ATTEMPTS):
                        move_error = (
                            "obstacle_recovery_exhausted:"
                            f"blocked_front_after_{int(obstacle_recovery_attempts)}_pivot_attempts"
                        )
                        break

                sample = _sample_motion_status(
                    start_pose,
                    status,
                    heading_abort_deg=heading_abort_deg,
                    enforce_heading_abort=bool(enforce_heading_abort),
                )
                actual_linear_samples.append(float(sample.get("actual_linear_mps", 0.0)))
                actual_angular_samples_dps.append(float(sample.get("actual_angular_dps", 0.0)))
                cmd_linear_samples.append(float(sample.get("cmd_linear_mps", 0.0)))
                cmd_angular_samples_dps.append(float(sample.get("cmd_angular_dps", 0.0)))
                if command_progress_prev_ts is None:
                    command_progress_prev_ts = float(now_mono)
                dt_cmd = max(0.0, float(now_mono) - float(command_progress_prev_ts))
                command_progress_prev_ts = float(now_mono)
                v_cmd_obs = _maybe_finite(status.get("v_cmd"))
                if v_cmd_obs is None:
                    v_cmd_obs = _maybe_finite(((status.get("control_monitor") or {}).get("v_cmd")))
                if v_cmd_obs is not None and math.isfinite(float(v_cmd_obs)):
                    observed_command_progress_m += abs(float(v_cmd_obs)) * dt_cmd
                max_heading_abs_deg = max(float(max_heading_abs_deg), abs(float(sample["heading_change_deg"])))
                if bool(sample.get("heading_abort_triggered", False)):
                    heading_abort_triggered_any = True
                    heading_abort_streak += 1
                else:
                    heading_abort_streak = 0
                if bool(enforce_heading_abort) and heading_abort_streak >= int(DEFAULT_HEADING_ABORT_CONSECUTIVE_SAMPLES):
                    move_error = (
                        "Heading change exceeded bound "
                        f"({float(sample['heading_change_deg']):.2f} deg > {float(heading_abort_deg):.2f} deg) "
                        f"for {int(heading_abort_streak)} consecutive samples."
                    )
                    break
                resolution = extract_motion_resolution(status)
                resolved_cmd_type = str(resolution.get("resolved_command_type", "") or "").strip().lower()
                if resolution["resolved_command_type"]:
                    resolved_command_types_seen[resolution["resolved_command_type"]] += 1
                if resolution["resolved_source"] and resolved_cmd_type not in ("soft_stop", "stop", "idle", "cancel_motion"):
                    resolved_sources_seen[resolution["resolved_source"]] += 1
                if bool(((status.get("lidar_odom_status") or {}).get("applied", False))):
                    applied_samples += 1

                encoder_reading = _extract_encoder_distance(status)
                if bool(encoder_reading.get("available", False)):
                    current_dist = _maybe_finite(encoder_reading.get("distance_m"))
                    if current_dist is not None:
                        if encoder_reference_m is None:
                            encoder_reference_m = float(current_dist)
                            encoder_reference_source = str(encoder_reading.get("source") or encoder_reference_source)
                        else:
                            encoder_progress_samples += 1
                            max_encoder_progress_m = max(
                                float(max_encoder_progress_m),
                                abs(float(current_dist) - float(encoder_reference_m)),
                            )

                max_distance_m = max(max_distance_m, float(sample["estimated_distance_m"]))
                if terminal_guidance_active:
                    remaining_m = float(target_distance_m) - float(sample["estimated_distance_m"])
                    if (not terminal_mode_entered) and remaining_m <= float(terminal_brake_distance_m) and remaining_m > 0.0:
                        reduced_abs_v = max(
                            float(DEFAULT_TERMINAL_GUIDANCE_MIN_SPEED_MPS),
                            abs(float(v_mps)) * float(DEFAULT_TERMINAL_GUIDANCE_SPEED_SCALE),
                        )
                        commanded_v_mps = math.copysign(reduced_abs_v, float(v_mps))
                        track_velocity = _track_velocity_from_twist(commanded_v_mps, commanded_omega_rad_s)
                        terminal_ts_wall = time.time()
                        _append_command(
                            "set_track_velocity",
                            token=token,
                            left_mps=float(track_velocity["left_mps"]),
                            right_mps=float(track_velocity["right_mps"]),
                            motion_source=segment_motion_source,
                        )
                        _record_profile_event(
                            "set_track_velocity",
                            ts_wall=terminal_ts_wall,
                            left_mps=float(track_velocity["left_mps"]),
                            right_mps=float(track_velocity["right_mps"]),
                            motion_source=segment_motion_source,
                        )
                        last_keepalive = now_mono
                        terminal_mode_entered = True
                    if terminal_mode_entered:
                        stop_buffer_now = max(
                            0.015,
                            min(float(terminal_stop_buffer_m), abs(float(commanded_v_mps)) * 0.35),
                        )
                    else:
                        stop_buffer_now = float(terminal_stop_buffer_m)
                    stop_trigger_m = max(0.0, float(target_distance_m) - float(stop_buffer_now))
                    if float(sample["estimated_distance_m"]) >= stop_trigger_m:
                        break
                elif (
                    bool(use_pose_closed_loop)
                    and float(target_completion_ratio) > 1e-6
                    and (
                        abs(float(pose_target_heading_delta_deg)) >= 1.0
                        or abs(float(pose_target_lateral_m)) >= 0.03
                    )
                ):
                    completion_gate_m = (
                        float(handoff_completion_m)
                        if not bool(stop_after_motion)
                        else float(required_completion_m)
                    )
                    if float(sample["estimated_distance_m"]) < float(completion_gate_m):
                        pass
                    elif not bool(stop_after_motion):
                        break
                    else:
                        pose_now = _get_pose(status)
                        target_heading_deg = math.degrees(float(pose_target.get("theta_rad", 0.0)))
                        heading_error_deg = abs(
                            float(_normalize_angle_deg(float(target_heading_deg) - float(pose_now.get("theta_deg", 0.0))))
                        )
                        if heading_error_deg <= float(pose_target_heading_tolerance_deg):
                            break
                elif float(sample["estimated_distance_m"]) >= float(target_distance_m):
                    if (
                        bool(use_pose_closed_loop)
                        and (
                            abs(float(pose_target_heading_delta_deg)) >= 1.0
                            or abs(float(pose_target_lateral_m)) >= 0.03
                        )
                    ):
                        pose_now = _get_pose(status)
                        target_heading_deg = math.degrees(float(pose_target.get("theta_rad", 0.0)))
                        heading_error_deg = abs(
                            float(_normalize_angle_deg(float(target_heading_deg) - float(pose_now.get("theta_deg", 0.0))))
                        )
                        if heading_error_deg <= float(pose_target_heading_tolerance_deg):
                            break
                        if float(sample["estimated_distance_m"]) >= float(target_distance_m) * 1.35:
                            move_error = (
                                "Pose target heading gate failed after distance overshoot "
                                f"(heading_error_deg={heading_error_deg:.2f}, "
                                f"tolerance_deg={float(pose_target_heading_tolerance_deg):.2f}, "
                                f"estimated_distance_m={float(sample['estimated_distance_m']):.3f}, "
                                f"target_distance_m={float(target_distance_m):.3f})."
                            )
                            break
                    else:
                        break

                if heading_hold_active:
                    heading_err_deg = float(sample["heading_change_deg"])
                    if abs(float(heading_err_deg)) <= float(DEFAULT_HEADING_HOLD_DEADBAND_DEG):
                        desired_omega = 0.0
                    else:
                        desired_omega = -float(DEFAULT_HEADING_HOLD_KP_OMEGA) * math.radians(float(heading_err_deg))
                    max_omega = float(DEFAULT_HEADING_HOLD_MAX_OMEGA_RAD_S)
                    desired_omega = max(-max_omega, min(max_omega, float(desired_omega)))
                    if abs(float(desired_omega)) > 1e-6:
                        heading_hold_engaged = True
                    if abs(float(desired_omega) - float(commanded_omega_rad_s)) > 0.015:
                        commanded_omega_rad_s = float(desired_omega)
                        track_velocity = _track_velocity_from_twist(commanded_v_mps, commanded_omega_rad_s)
                        heading_hold_ts_wall = time.time()
                        _append_command(
                            "set_track_velocity",
                            token=token,
                            left_mps=float(track_velocity["left_mps"]),
                            right_mps=float(track_velocity["right_mps"]),
                            motion_source=segment_motion_source,
                        )
                        _record_profile_event(
                            "set_track_velocity",
                            ts_wall=heading_hold_ts_wall,
                            left_mps=float(track_velocity["left_mps"]),
                            right_mps=float(track_velocity["right_mps"]),
                            motion_source=segment_motion_source,
                        )
                        last_keepalive = now_mono
                if amr_progress_guard_active and command_progress_guard_limit_m is not None:
                    if float(observed_command_progress_m) > 1e-6:
                        command_progress_m = float(observed_command_progress_m)
                    else:
                        command_progress_m = abs(float(v_mps)) * max(0.0, float(now_mono - start_wall))
                    max_command_progress_m = max(float(max_command_progress_m), float(command_progress_m))
                    if (
                        command_progress_m >= float(command_progress_guard_limit_m)
                        and float(sample["estimated_distance_m"]) < float(required_completion_m)
                    ):
                        command_progress_guard_triggered = True
                        if not command_progress_guard_event:
                            command_progress_guard_event = {
                                "command_progress_m": float(command_progress_m),
                                "guard_limit_m": float(command_progress_guard_limit_m),
                                "estimated_distance_m": float(sample["estimated_distance_m"]),
                                "required_completion_m": float(required_completion_m),
                            }
                        if bool(enforce_progress_guard):
                            move_error = (
                                "AMR command progress safety gate triggered "
                                f"(command_progress_m={command_progress_m:.3f}, "
                                f"guard_limit_m={float(command_progress_guard_limit_m):.3f}, "
                                f"estimated_distance_m={float(sample['estimated_distance_m']):.3f}, "
                                f"required_completion_m={float(required_completion_m):.3f})."
                            )
                            break
                if amr_progress_guard_active and encoder_progress_guard_limit_m is not None:
                    if (
                        float(max_encoder_progress_m) >= float(encoder_progress_guard_limit_m)
                        and float(sample["estimated_distance_m"]) < float(required_completion_m)
                    ):
                        encoder_progress_guard_triggered = True
                        if not encoder_progress_guard_event:
                            encoder_progress_guard_event = {
                                "encoder_progress_m": float(max_encoder_progress_m),
                                "guard_limit_m": float(encoder_progress_guard_limit_m),
                                "estimated_distance_m": float(sample["estimated_distance_m"]),
                                "required_completion_m": float(required_completion_m),
                            }
                        if bool(enforce_progress_guard):
                            move_error = (
                                "AMR encoder progress safety gate triggered "
                                f"(encoder_progress_m={float(max_encoder_progress_m):.3f}, "
                                f"guard_limit_m={float(encoder_progress_guard_limit_m):.3f}, "
                                f"estimated_distance_m={float(sample['estimated_distance_m']):.3f}, "
                                f"required_completion_m={float(required_completion_m):.3f})."
                            )
                            break

            if (now_mono - last_status_change) > DEFAULT_STATUS_STALE_S:
                stale_stream = True
                raise RuntimeError("Status stream became stale during bounded motion.")

            time.sleep(DEFAULT_POLL_S)
        else:
            timed_out_without_target = True
    except Exception as exc:
        move_error = str(exc)
    finally:
        if bool(stop_after_motion):
            stop_outcome = _bounded_stop_with_fallback(
                token,
                timeout_s=stop_timeout_s,
                motion_source=segment_motion_source,
            )
        else:
            stop_outcome = {
                "status": dict(latest_status or {}),
                "pose_status": dict(latest_status or {}),
                "command": {},
                "normal_stop_used": False,
                "emergency_stop_triggered": False,
                "failsafe_triggered": False,
                "details": {"path": "continuous_handoff"},
            }
        stop_status_candidate = dict(stop_outcome.get("status") or {})
        if stop_status_candidate:
            diag_tracker.sample(stop_status_candidate)

    end_status = dict(stop_outcome.get("status") or latest_status or {})
    pose_status = dict(stop_outcome.get("pose_status") or end_status or {})
    end_encoder_distance = _extract_encoder_distance(end_status)
    end_encoder_dist_m = _maybe_finite(end_encoder_distance.get("distance_m"))
    if encoder_reference_m is None and end_encoder_dist_m is not None:
        encoder_reference_m = float(end_encoder_dist_m)
        if not encoder_reference_source:
            encoder_reference_source = str(end_encoder_distance.get("source") or "")
    if encoder_reference_m is not None and end_encoder_dist_m is not None:
        max_encoder_progress_m = max(
            float(max_encoder_progress_m),
            abs(float(end_encoder_dist_m) - float(encoder_reference_m)),
        )
    encoder_distance_summary = {
        "available": bool(encoder_reference_m is not None and end_encoder_dist_m is not None),
        "source": str(
            encoder_reference_source
            or end_encoder_distance.get("source")
            or start_encoder_distance.get("source")
            or ""
        ),
        "start_distance_m": (None if encoder_reference_m is None else float(encoder_reference_m)),
        "end_distance_m": (None if end_encoder_dist_m is None else float(end_encoder_dist_m)),
        "max_progress_m": float(max_encoder_progress_m),
        "samples": int(encoder_progress_samples),
    }
    end_pose = _get_pose(pose_status)
    resolution = extract_motion_resolution(end_status)
    end_emergency_count = int(_safe_float(((end_status.get("last_emergency") or {}).get("count")), 0.0))
    estimated_distance_m = float(_pose_distance(start_pose, end_pose))
    heading_change_deg = float(_normalize_angle_deg(end_pose["theta_deg"] - start_pose["theta_deg"]))
    target_heading_error_deg = None
    if bool(use_pose_closed_loop) and pose_target:
        pose_target["heading_tolerance_deg"] = float(pose_target_heading_tolerance_deg)
        if float(pose_target_omega_max_rad_s) > 0.0:
            pose_target["omega_max_rad_s"] = float(pose_target_omega_max_rad_s)
        target_heading_error_deg = abs(
            float(_normalize_angle_deg(math.degrees(float(pose_target.get("theta_rad", 0.0))) - float(end_pose["theta_deg"])))
        )
    max_heading_abs_deg = max(float(max_heading_abs_deg), abs(float(heading_change_deg)))
    lidar_status_summary = _build_lidar_status_summary(
        start_status,
        end_status,
        applied_samples=applied_samples,
    )
    lidar_reference_telemetry = _extract_lidar_reference_telemetry(end_status)
    lidar_diag_summary = diag_tracker.summary()
    lidar_counts = _resolve_lidar_update_counts(
        lidar_status_summary=lidar_status_summary,
        lidar_diag_summary=lidar_diag_summary,
    )
    lidar_accept_count = int(lidar_counts.get("accept_count", 0))
    lidar_reject_count = int(lidar_counts.get("reject_count", 0))
    failsafe_triggered = bool(
        stop_outcome.get("failsafe_triggered", False)
        or _status_state(end_status) == "FAILSAFE"
        or end_emergency_count > start_emergency_count
    )
    normal_stop_used = bool(stop_outcome.get("normal_stop_used", False))
    emergency_stop_triggered = bool(stop_outcome.get("emergency_stop_triggered", False))
    resolved_sources_nonempty = {source for source in resolved_sources_seen if source}
    ownership_clear = len(resolved_sources_nonempty) <= 1
    if bool(use_pose_closed_loop) and resolved_sources_nonempty.issubset({"STATE", "MANUAL", "GUI_JOYSTICK"}):
        ownership_clear = True
    lidar_updates_visible = bool(
        lidar_accept_count > 0
        or int(lidar_status_summary.get("applied_status_samples", 0)) > 0
    )
    effective_progress_m = max(0.0, abs(float(estimated_distance_m)))
    encoder_progress_m = max(
        0.0,
        abs(float(_safe_float(encoder_distance_summary.get("max_progress_m"), 0.0))),
    )
    progress_source = "EKF_POSE"
    stop_command = dict(stop_outcome.get("command") or {})
    stop_cmd_ts = _maybe_finite(stop_command.get("sent_ts_wall"))
    if stop_cmd_ts is not None:
        stop_cmd_type = str(stop_command.get("cmd_type") or stop_command.get("type") or "stop")
        _record_profile_event(
            stop_cmd_type,
            ts_wall=stop_cmd_ts,
            cmd_id=stop_command.get("cmd_id"),
        )
    command_consistency = _compute_command_motion_consistency(
        start_cmd=start_cmd,
        stop_cmd=stop_command,
        commanded_linear_speed_mps=float(v_mps),
        estimated_distance_m=float(estimated_distance_m),
        command_profile_events=command_profile_events,
        token=str(token),
    )
    command_window_s = float(command_consistency.get("command_window_s", 0.0))
    initial_setpoint_distance_m = float(command_consistency.get("initial_setpoint_distance_m", 0.0))
    effective_command_distance_m = float(command_consistency.get("effective_command_distance_m", 0.0))
    command_nominal_distance_m = float(command_consistency.get("command_nominal_distance_m", 0.0))
    odom_distance_m = float(command_consistency.get("odom_distance_m", estimated_distance_m))
    odom_vs_initial_setpoint_ratio = command_consistency.get("odom_vs_initial_setpoint_ratio")
    odom_vs_effective_command_ratio = command_consistency.get("odom_vs_effective_command_ratio")
    odom_vs_command_ratio = command_consistency.get("odom_vs_command_ratio")
    time_weighted_mean_linear_mps = float(command_consistency.get("time_weighted_mean_linear_mps", 0.0))
    command_profile_source = str(command_consistency.get("command_profile_source", ""))
    if bool(use_pose_closed_loop) and float(observed_command_progress_m) > 0.05:
        effective_command_distance_m = float(observed_command_progress_m)
        command_nominal_distance_m = float(observed_command_progress_m)
        odom_vs_effective_command_ratio = (
            None if effective_command_distance_m <= 1e-6 else float(odom_distance_m) / float(effective_command_distance_m)
        )
        odom_vs_command_ratio = odom_vs_effective_command_ratio
        command_profile_source = "status_v_cmd_integrated_pose_mode"
    runtime_s = max(0.0, time.monotonic() - start_wall)
    truth_surface = _truth_surface_from_status(end_status)

    def _mean_or_none(values: List[float]) -> float | None:
        clean = [float(v) for v in values if math.isfinite(float(v))]
        if not clean:
            return None
        return float(sum(clean) / len(clean))

    motion_public_end = _extract_motion_public(end_status)
    motion_public_segment = dict(motion_public_end.get("segment") or {})
    cmd_avg_linear_segment = _safe_float(
        motion_public_segment.get("commanded_average_linear_speed_mps"),
        math.nan,
    )
    cmd_avg_angular_segment = _safe_float(
        motion_public_segment.get("commanded_average_angular_speed_dps"),
        math.nan,
    )
    actual_avg_linear_segment = _safe_float(
        motion_public_segment.get("actual_average_linear_speed_mps"),
        math.nan,
    )
    actual_avg_angular_segment = _safe_float(
        motion_public_segment.get("actual_average_angular_speed_dps"),
        math.nan,
    )
    # Detect stop/idle end-segment: its averages are near-zero and not representative.
    _seg_cmd_type = str(motion_public_segment.get("command_type", "") or "").strip().lower()
    _seg_is_stop = _seg_cmd_type in ("soft_stop", "stop", "idle", "") or bool(
        motion_public_segment.get("stop_reason", "") or ""
    )
    commanded_average_linear_speed_mps = (
        float(cmd_avg_linear_segment)
        if (math.isfinite(float(cmd_avg_linear_segment)) and not _seg_is_stop)
        else float(time_weighted_mean_linear_mps)
    )
    actual_linear_mean_sample = _mean_or_none(actual_linear_samples)
    cmd_angular_mean_sample = _mean_or_none(cmd_angular_samples_dps)
    actual_angular_mean_sample = _mean_or_none(actual_angular_samples_dps)
    _segment_avg_usable = (
        math.isfinite(float(actual_avg_linear_segment))
        and not _seg_is_stop
    )
    actual_average_linear_speed_mps = (
        float(actual_avg_linear_segment)
        if _segment_avg_usable
        else (
            float(actual_linear_mean_sample)
            if actual_linear_mean_sample is not None
            else (
                float(effective_progress_m) / max(0.1, float(runtime_s))
            )
        )
    )
    commanded_average_angular_speed_dps = (
        float(cmd_avg_angular_segment)
        if (math.isfinite(float(cmd_avg_angular_segment)) and not _seg_is_stop)
        else (
            float(cmd_angular_mean_sample)
            if cmd_angular_mean_sample is not None
            else float(omega_rad_s) * RAD_TO_DEG
        )
    )
    actual_average_angular_speed_dps = (
        float(actual_avg_angular_segment)
        if (math.isfinite(float(actual_avg_angular_segment)) and not _seg_is_stop)
        else (
            float(actual_angular_mean_sample)
            if actual_angular_mean_sample is not None
            else (
                float(heading_change_deg) / max(0.1, float(runtime_s))
            )
        )
    )
    truth_surface = _apply_segment_turn_truth(
        truth_surface,
        actual_average_angular_speed_dps=float(actual_average_angular_speed_dps),
        heading_change_deg=float(heading_change_deg),
        effective_progress_m=float(effective_progress_m),
        expected_heading_delta_deg=float(pose_target_heading_delta_deg),
        target_lateral_m=float(pose_target_lateral_m),
    )
    segment_stop_reason = str(_segment_stop_reason(end_status) or "")

    if not move_error and timed_out_without_target:
        if effective_progress_m < float(required_completion_m):
            move_error = (
                "Target distance completion gate failed before timeout "
                f"(effective_progress_m={effective_progress_m:.3f}, "
                f"required_completion_m={required_completion_m:.3f}, "
                f"estimated_distance_m={estimated_distance_m:.3f}, "
                f"encoder_progress_m={encoder_progress_m:.3f}, "
                f"source={progress_source})."
            )
        elif effective_progress_m < float(min_progress_m):
            move_error = (
                f"Bounded motion did not achieve minimum progress "
                f"(effective_progress_m={effective_progress_m:.3f}, "
                f"estimated_distance_m={estimated_distance_m:.3f}, "
                f"encoder_progress_m={encoder_progress_m:.3f}, "
                f"min_progress_m={min_progress_m:.3f}, "
                f"source={progress_source})."
            )
    if not move_error and not lidar_updates_visible:
        move_error = "No accepted LiDAR odometry updates were observed during bounded motion."
    if not move_error and int(min_lidar_accept_count) > 0 and lidar_accept_count < int(min_lidar_accept_count):
        move_error = (
            "LiDAR accepted update count is below strict threshold "
            f"(accepted={lidar_accept_count}, required={int(min_lidar_accept_count)})."
        )
    if not move_error and int(max_lidar_reject_count) >= 0 and lidar_reject_count > int(max_lidar_reject_count):
        move_error = (
            "LiDAR rejected update count exceeded threshold "
            f"(rejected={lidar_reject_count}, allowed={int(max_lidar_reject_count)})."
        )
    if (
        not move_error
        and float(min_odom_vs_command_ratio) > 1e-6
        and odom_vs_effective_command_ratio is not None
        and effective_command_distance_m >= 0.10
        and float(odom_vs_effective_command_ratio) < float(min_odom_vs_command_ratio)
    ):
        move_error = (
            "Odometry-vs-command consistency gate failed "
            f"(effective_ratio={float(odom_vs_effective_command_ratio):.3f}, "
            f"required={float(min_odom_vs_command_ratio):.3f}, "
            f"effective_command_distance_m={effective_command_distance_m:.3f}, "
            f"initial_setpoint_distance_m={initial_setpoint_distance_m:.3f}, "
            f"estimated_distance_m={estimated_distance_m:.3f}, "
            f"command_window_s={command_window_s:.3f})."
        )
    if not move_error and bool(stop_after_motion) and not normal_stop_used:
        move_error = "Normal stop did not complete cleanly."
    if not move_error and failsafe_triggered:
        move_error = "FAILSAFE latched during normal bounded motion."
    if not move_error and not ownership_clear:
        move_error = f"Resolved motion source changed during motion: {sorted(resolved_sources_seen)}"

    return {
        "timestamp": _now_iso(),
        "test_name": str(name),
        "success": not bool(move_error),
        "fail_reason": str(move_error or ""),
        "odometry_mode": str(end_status.get("odometry_mode", "")),
        "resolved_motion_source": resolution["resolved_source"] or (
            max(resolved_sources_seen, key=resolved_sources_seen.get) if resolved_sources_seen else ""
        ),
        "start_pose": start_pose,
        "end_pose": end_pose,
        "estimated_distance_m": round(estimated_distance_m, 4),
        "cmd_linear_mps": round(float(_safe_float(motion_public_end.get("cmd_linear_mps"), v_mps)), 6),
        "cmd_angular_dps": round(float(_safe_float(motion_public_end.get("cmd_angular_dps"), float(omega_rad_s) * RAD_TO_DEG)), 6),
        "actual_linear_mps": round(float(_safe_float(motion_public_end.get("actual_linear_mps"), actual_average_linear_speed_mps)), 6),
        "actual_angular_dps": round(float(_safe_float(motion_public_end.get("actual_angular_dps"), actual_average_angular_speed_dps)), 6),
        "effective_progress_m": round(float(effective_progress_m), 4),
        "required_completion_m": round(float(required_completion_m), 4),
        "handoff_completion_m": round(float(handoff_completion_m), 4),
        "command_progress_guard_limit_m": (
            None if command_progress_guard_limit_m is None else round(float(command_progress_guard_limit_m), 4)
        ),
        "command_progress_peak_m": round(float(max_command_progress_m), 4),
        "command_progress_guard_triggered": bool(command_progress_guard_triggered),
        "encoder_progress_guard_triggered": bool(encoder_progress_guard_triggered),
        "progress_guard_enforced": bool(enforce_progress_guard),
        "command_progress_guard_event": (
            None if not command_progress_guard_event else dict(command_progress_guard_event)
        ),
        "encoder_progress_guard_event": (
            None if not encoder_progress_guard_event else dict(encoder_progress_guard_event)
        ),
        "encoder_progress_guard_limit_m": (
            None if encoder_progress_guard_limit_m is None else round(float(encoder_progress_guard_limit_m), 4)
        ),
        "progress_gate_source": str(progress_source),
        "heading_change_deg": round(heading_change_deg, 3),
        "target_heading_error_deg": (
            None if target_heading_error_deg is None else round(float(target_heading_error_deg), 3)
        ),
        "pose_target": dict(pose_target),
        "max_heading_abs_deg": round(float(max_heading_abs_deg), 3),
        "heading_abort_threshold_deg": round(float(heading_abort_deg), 3),
        "heading_abort_triggered": bool(heading_abort_triggered_any),
        "lidar_status_summary": lidar_status_summary,
        "lidar_reference_telemetry": lidar_reference_telemetry,
        "lidar_diag_summary": lidar_diag_summary,
        "lidar_count_sources": dict(lidar_counts.get("sources") or {}),
        "lidar_accept_count": int(lidar_accept_count),
        "lidar_reject_count": int(lidar_reject_count),
        "encoder_distance_summary": encoder_distance_summary,
        "determinism_path": "EKF_POSE_WITH_LIDAR_ODOM",
        "command_motion_window_s": round(float(command_window_s), 4),
        "initial_setpoint_distance_m": round(float(initial_setpoint_distance_m), 4),
        "effective_command_distance_m": round(float(effective_command_distance_m), 4),
        "odom_distance_m": round(float(odom_distance_m), 4),
        "odom_vs_initial_setpoint_ratio": (
            None
            if odom_vs_initial_setpoint_ratio is None
            else round(float(odom_vs_initial_setpoint_ratio), 4)
        ),
        "odom_vs_effective_command_ratio": (
            None
            if odom_vs_effective_command_ratio is None
            else round(float(odom_vs_effective_command_ratio), 4)
        ),
        "time_weighted_mean_linear_mps": round(float(time_weighted_mean_linear_mps), 6),
        "commanded_average_linear_speed_mps": round(float(commanded_average_linear_speed_mps), 6),
        "actual_average_linear_speed_mps": round(float(actual_average_linear_speed_mps), 6),
        "commanded_average_angular_speed_dps": round(float(commanded_average_angular_speed_dps), 6),
        "actual_average_angular_speed_dps": round(float(actual_average_angular_speed_dps), 6),
        "command_profile_source": str(command_profile_source),
        "obstacle_recovery": {
            "attempts": int(obstacle_recovery_attempts),
            "events": list(obstacle_recovery_events),
            "max_attempts": int(DEFAULT_OBSTACLE_PIVOT_MAX_ATTEMPTS),
        },
        "command_nominal_distance_m": round(float(command_nominal_distance_m), 4),
        "odom_vs_command_ratio": (None if odom_vs_command_ratio is None else round(float(odom_vs_command_ratio), 4)),
        "segment_report": {
            "target_distance_m": round(float(target_distance_m), 4),
            "actual_progress_distance_m": round(float(effective_progress_m), 4),
            "target_heading_deg": None,
            "target_heading_error_deg": (
                None if target_heading_error_deg is None else round(float(target_heading_error_deg), 3)
            ),
            "actual_heading_progress_deg": round(float(heading_change_deg), 3),
            "commanded_average_linear_speed_mps": round(float(commanded_average_linear_speed_mps), 6),
            "actual_average_linear_speed_mps": round(float(actual_average_linear_speed_mps), 6),
            "commanded_angular_speed_dps": round(float(commanded_average_angular_speed_dps), 6),
            "actual_angular_speed_dps": round(float(actual_average_angular_speed_dps), 6),
            "stop_reason": str(segment_stop_reason or "normal_stop"),
        },
        "measured_distance_m": None,
        "measured_heading_deg": None,
        "motion_profile": {
            "use_pose_closed_loop": bool(use_pose_closed_loop),
            "command_type": ("go_to_pose" if bool(use_pose_closed_loop) else "set_track_velocity"),
            "execution_mode": str(truth_surface.get("execution_mode", "UNKNOWN") or "UNKNOWN"),
            "turn_primitives": dict(truth_surface.get("turn_primitives") or {}),
            "turn_primitive_requested": str(truth_surface.get("turn_primitive_requested", "UNKNOWN") or "UNKNOWN"),
            "turn_primitive_limited": str(truth_surface.get("turn_primitive_limited", "UNKNOWN") or "UNKNOWN"),
            "turn_primitive_executed": str(truth_surface.get("turn_primitive_executed", "UNKNOWN") or "UNKNOWN"),
            "turn_primitive_actual": str(truth_surface.get("turn_primitive_actual", "UNKNOWN") or "UNKNOWN"),
            "terminal_guidance_active": bool(terminal_guidance_active),
            "terminal_mode_entered": bool(terminal_mode_entered),
            "terminal_brake_distance_m": (
                float(terminal_brake_distance_m) if bool(terminal_guidance_active) else None
            ),
            "terminal_stop_buffer_m": (
                float(terminal_stop_buffer_m) if bool(terminal_guidance_active) else None
            ),
            "commanded_v_start_mps": float(v_mps),
            "commanded_v_final_mps": float(commanded_v_mps),
            "heading_hold_active": bool(heading_hold_active),
            "heading_hold_engaged": bool(heading_hold_engaged),
            "heading_hold_kp_omega": (
                float(DEFAULT_HEADING_HOLD_KP_OMEGA) if bool(heading_hold_active) else None
            ),
            "heading_hold_max_omega_rad_s": (
                float(DEFAULT_HEADING_HOLD_MAX_OMEGA_RAD_S) if bool(heading_hold_active) else None
            ),
            "commanded_omega_start_rad_s": float(omega_rad_s),
            "commanded_omega_final_rad_s": float(commanded_omega_rad_s),
            "enforce_heading_abort": bool(enforce_heading_abort),
            "probe_heading_hold_enabled": bool(enable_probe_heading_hold),
            "progress_guard_enforced": bool(enforce_progress_guard),
            "command_progress_guard_triggered": bool(command_progress_guard_triggered),
            "encoder_progress_guard_triggered": bool(encoder_progress_guard_triggered),
        },
        "loop_health_summary": _build_loop_health_summary(
            start_status_version,
            _status_version(end_status),
            status_samples=status_samples,
            stale_stream=stale_stream,
            resolution_observed=bool(resolved_sources_seen or resolved_command_types_seen),
            sources_seen=resolved_sources_seen,
            command_types_seen=resolved_command_types_seen,
            end_status=end_status,
        ),
        "failsafe_triggered": bool(failsafe_triggered),
        "emergency_stop_triggered": bool(emergency_stop_triggered),
        "normal_stop_used": bool(normal_stop_used),
        "stop_after_motion": bool(stop_after_motion),
        "continuous_handoff": bool(not bool(stop_after_motion)),
        "max_runtime_s": float(max_runtime_s),
        "actual_runtime_s": round(float(runtime_s), 3),
        "target_distance_m": float(target_distance_m),
        "min_progress_m": float(min_progress_m),
        "stop_behavior": {
            "path": str(((stop_outcome.get("details") or {}).get("path")) or "normal_stop"),
            "stop_type": _stop_status_type(end_status),
            "state": _status_state(end_status),
            "command": dict(stop_command),
            "details": dict(stop_outcome.get("details") or {}),
        },
        "stop_reason": str(segment_stop_reason or ""),
        "motion_ownership": {
            "clear": bool(ownership_clear),
            "resolved_sources_seen": sorted(source for source in resolved_sources_seen if source),
            "resolved_command_types_seen": sorted(command_type for command_type in resolved_command_types_seen if command_type),
        },
        "command_lifecycle": {
            "start": start_cmd,
            "stop": dict(stop_command),
            "stop_details": dict(stop_outcome.get("details") or {}),
        },
        **truth_surface,
    }


def _run_emergency_stop_test(token: str) -> Dict[str, Any]:
    start_wall = time.monotonic()
    idle = _normal_stop(token, timeout_s=DEFAULT_STOP_TIMEOUT_S)
    start_status = dict(idle.get("status") or {})
    start_pose = _get_pose(start_status)
    start_status_version = _status_version(start_status)
    start_emergency_count = int(_safe_float(((start_status.get("last_emergency") or {}).get("count")), 0.0))

    emergency_cmd = _send_command_checked("emergency_stop", token=token, timeout_s=4.0)
    emergency_status = _wait_for_failsafe(DEFAULT_EMERGENCY_TIMEOUT_S)
    failsafe_pose = _get_pose(emergency_status)
    emergency_count_after = int(_safe_float(((emergency_status.get("last_emergency") or {}).get("count")), 0.0))
    latched = bool(_status_state(emergency_status) == "FAILSAFE" and emergency_count_after > start_emergency_count)

    cleanup_error = ""
    try:
        reset_result = _strong_reset_to_idle(token, timeout_s=DEFAULT_RESET_TIMEOUT_S)
        end_status = dict(reset_result.get("status") or {})
    except Exception as exc:
        cleanup_error = str(exc)
        end_status = _read_json(STATUS_PATH)
        reset_result = {"error": cleanup_error}

    end_pose = failsafe_pose
    resolution = extract_motion_resolution(end_status)
    truth_surface = _truth_surface_from_status(end_status or emergency_status)
    fail_reason = ""
    if not latched:
        fail_reason = "Emergency stop did not latch FAILSAFE."
    elif cleanup_error:
        fail_reason = f"Emergency stop cleanup failed: {cleanup_error}"

    return {
        "timestamp": _now_iso(),
        "test_name": "emergency_stop_idle",
        "success": not bool(fail_reason),
        "fail_reason": str(fail_reason),
        "odometry_mode": str(end_status.get("odometry_mode", emergency_status.get("odometry_mode", ""))),
        "resolved_motion_source": resolution["resolved_source"],
        "start_pose": start_pose,
        "end_pose": end_pose,
        "estimated_distance_m": round(float(_pose_distance(start_pose, end_pose)), 4),
        "heading_change_deg": round(float(_normalize_angle_deg(end_pose["theta_deg"] - start_pose["theta_deg"])), 3),
        "lidar_status_summary": _build_lidar_status_summary(
            start_status,
            end_status,
            applied_samples=0,
        ),
        "loop_health_summary": _build_loop_health_summary(
            start_status_version,
            _status_version(end_status),
            status_samples=2,
            stale_stream=False,
            resolution_observed=bool(resolution["observable"]),
            sources_seen=Counter([resolution["resolved_source"]]) if resolution["resolved_source"] else Counter(),
            command_types_seen=Counter([resolution["resolved_command_type"]]) if resolution["resolved_command_type"] else Counter(),
            end_status=end_status,
        ),
        "failsafe_triggered": bool(latched),
        "emergency_stop_triggered": True,
        "normal_stop_used": False,
        "max_runtime_s": float(DEFAULT_EMERGENCY_TIMEOUT_S + DEFAULT_RESET_TIMEOUT_S),
        "actual_runtime_s": round(max(0.0, time.monotonic() - start_wall), 3),
        "stop_behavior": {
            "failsafe_latched": bool(latched),
            "emergency_command": emergency_cmd,
            "cleanup_reset": {
                "command": dict((reset_result or {}).get("command") or {}),
                "idle_stop_command": dict((((reset_result or {}).get("idle_stop") or {}).get("command")) or {}),
            },
            "end_state": _status_state(end_status),
        },
        **truth_surface,
    }


def _aggregate_lidar_diag(results: List[Dict[str, Any]]) -> Dict[str, Any]:
    total_samples = 0
    total_matcher_called = 0
    total_filtered_points = 0.0
    total_accept = 0
    total_reject = 0
    reason_counts: Counter[str] = Counter()
    for result in results:
        diag = dict(result.get("lidar_diag_summary") or {})
        if not diag:
            continue
        samples = int(_safe_float(diag.get("status_samples"), 0))
        total_samples += samples
        total_matcher_called += int(_safe_float(diag.get("matcher_called_samples"), 0))
        total_filtered_points += float(_safe_float(diag.get("filtered_points_total"), 0.0))
        total_accept += int(_safe_float(diag.get("odom_accept"), 0))
        total_reject += int(_safe_float(diag.get("odom_reject"), 0))
        for reason, count in (diag.get("reason_counts") or {}).items():
            reason_counts[reason] += int(_safe_float(count, 0))
    matcher_ratio = (total_matcher_called / total_samples) if total_samples else 0.0
    avg_points = (total_filtered_points / total_samples) if total_samples else 0.0
    return {
        "status_samples": total_samples,
        "matcher_called_samples": total_matcher_called,
        "filtered_points_total": total_filtered_points,
        "reason_counts": dict(reason_counts),
        "odom_accept": total_accept,
        "odom_reject": total_reject,
        "matcher_called_ratio": matcher_ratio,
        "avg_filtered_points": avg_points,
        "accept_vs_reject": f"{total_accept}:{total_reject}",
    }


def _truth_surface_from_result(result: Dict[str, Any]) -> Dict[str, Any]:
    src = dict(result or {})
    return {
        "execution_mode": str(src.get("execution_mode", "UNKNOWN") or "UNKNOWN"),
        "motion_actual_ssot": str(src.get("motion_actual_ssot", "EKF_POSE_ODOMETRY_SSOT") or "EKF_POSE_ODOMETRY_SSOT"),
        "truth_basis": dict(src.get("truth_basis") or {}),
        "lidar_odom_status_truth": dict(src.get("lidar_odom_status_truth") or {}),
        "lidar_odom_applied": bool(src.get("lidar_odom_applied", False)),
        "lidar_odom_latest_age_s": src.get("lidar_odom_latest_age_s"),
        "lidar_odom_latest_confidence": src.get("lidar_odom_latest_confidence"),
        "encoder_pose_active_samples": int(_safe_float(src.get("encoder_pose_active_samples"), 0.0)),
        "turn_primitive_requested": str(src.get("turn_primitive_requested", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_limited": str(src.get("turn_primitive_limited", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_executed": str(src.get("turn_primitive_executed", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_actual": str(src.get("turn_primitive_actual", "UNKNOWN") or "UNKNOWN"),
        "turn_primitives": dict(src.get("turn_primitives") or {}),
    }


def build_suite_rollup(
    *,
    test_name: str,
    preflight: Dict[str, Any],
    forward_results: List[Dict[str, Any]],
    heading_result: Dict[str, Any] | None,
    emergency_result: Dict[str, Any],
    suite_runtime_s: float,
) -> Dict[str, Any]:
    executed_results = list(forward_results)
    if heading_result is not None:
        executed_results.append(heading_result)

    all_results = list(executed_results) + [emergency_result]
    start_pose = executed_results[0]["start_pose"] if executed_results else {}
    end_pose = executed_results[-1]["end_pose"] if executed_results else {}
    distance_total = sum(float(item.get("estimated_distance_m", 0.0) or 0.0) for item in forward_results)
    forward_distances = [float(item.get("estimated_distance_m", 0.0) or 0.0) for item in forward_results]
    resolved_sources = Counter(
        str(item.get("resolved_motion_source", "") or "")
        for item in executed_results
        if str(item.get("resolved_motion_source", "") or "")
    )
    fail_reason = ""
    if not bool(preflight.get("ready", False)):
        fail_reason = "; ".join(preflight.get("blocking_issues") or []) or "preflight_failed"
    else:
        fail_reason = _first_fail_reason(all_results)

    lidar_diag_summary = _aggregate_lidar_diag(executed_results)
    forward_pass_count = sum(1 for item in forward_results if item.get("success"))
    forward_total = len(forward_results)
    forward_pass_rate = (float(forward_pass_count) / float(forward_total)) if forward_total > 0 else 0.0
    stop_pose_v_abs_values = [
        abs(float(_safe_float((((item.get("end_pose") or {}).get("v")), 0.0))))
        for item in forward_results
    ]
    truth_source = executed_results[-1] if executed_results else emergency_result
    suite_truth_surface = _truth_surface_from_result(truth_source)

    return {
        "timestamp": _now_iso(),
        "test_name": str(test_name),
        "success": bool(preflight.get("ready", False)) and not bool(fail_reason),
        "fail_reason": str(fail_reason),
        "odometry_mode": str(preflight.get("odometry_mode", "")),
        "resolved_motion_source": (
            max(resolved_sources, key=resolved_sources.get) if resolved_sources else str(preflight.get("normal_stop_validation", {}).get("resolved_motion_source", ""))
        ),
        "start_pose": start_pose,
        "end_pose": end_pose,
        "estimated_distance_m": round(float(distance_total), 4),
        "heading_change_deg": (
            round(float(_normalize_angle_deg(end_pose["theta_deg"] - start_pose["theta_deg"])), 3)
            if start_pose and end_pose
            else 0.0
        ),
        "lidar_status_summary": dict((executed_results[-1].get("lidar_status_summary") if executed_results else {}) or {}),
        "lidar_diag_summary": lidar_diag_summary,
        "loop_health_summary": {
            "preflight_ready": bool(preflight.get("ready", False)),
            "forward_tests_passed": int(forward_pass_count),
            "forward_tests_total": int(forward_total),
            "forward_pass_rate": round(float(forward_pass_rate), 4),
            "heading_test_ran": bool(heading_result is not None),
            "emergency_stop_passed": bool(emergency_result.get("success", False)),
            "suite_runtime_s": round(float(suite_runtime_s), 3),
        },
        "determinism": {
            "policy": "EKF_POSE_LIDAR_STRICT",
            "expected": str(preflight.get("expected_determinism", "")),
        },
        "failsafe_triggered": any(bool(item.get("failsafe_triggered", False)) for item in all_results),
        "emergency_stop_triggered": any(bool(item.get("emergency_stop_triggered", False)) for item in all_results),
        "normal_stop_used": all(
            bool(item.get("normal_stop_used", False))
            for item in forward_results
            if bool(item.get("stop_after_motion", True))
        ),
        "max_runtime_s": round(
            sum(float(item.get("max_runtime_s", 0.0) or 0.0) for item in all_results),
            3,
        ),
        "actual_runtime_s": round(float(suite_runtime_s), 3),
        "summary": {
            "forward_repeats": int(forward_total),
            "forward_pass_rate": round(float(forward_pass_rate), 4),
            "forward_distance_m": _stats(forward_distances),
            "stop_pose_abs_v_mps": _stats(stop_pose_v_abs_values),
            "emergency_test_skipped": bool(emergency_result.get("skipped", False)),
        },
        "preflight": dict(preflight),
        "subtests": all_results,
        "artifacts": {
            "latest_result": str(LATEST_RESULT_PATH.relative_to(PROJECT_ROOT)),
            "latest_summary": str(LATEST_SUMMARY_PATH.relative_to(PROJECT_ROOT)),
            "history": str(HISTORY_PATH.relative_to(PROJECT_ROOT)),
        },
        **suite_truth_surface,
    }


def build_compact_summary_report(result: Dict[str, Any]) -> Dict[str, Any]:
    suite = dict(result or {})
    loop = dict(suite.get("loop_health_summary") or {})
    summary = dict(suite.get("summary") or {})
    lidar = dict(suite.get("lidar_status_summary") or {})
    determinism = dict(suite.get("determinism") or {})
    return {
        "timestamp": str(suite.get("timestamp") or _now_iso()),
        "test_name": str(suite.get("test_name") or ""),
        "success": bool(suite.get("success", False)),
        "fail_reason": str(suite.get("fail_reason", "") or ""),
        "odometry_mode": str(suite.get("odometry_mode", "") or ""),
        "forward": {
            "tests_passed": int(loop.get("forward_tests_passed", 0)),
            "tests_total": int(loop.get("forward_tests_total", 0)),
            "pass_rate": float(loop.get("forward_pass_rate", 0.0) or 0.0),
            "distance_m": dict(summary.get("forward_distance_m") or {}),
        },
        "stop_quality": {
            "pose_abs_v_mps": dict(summary.get("stop_pose_abs_v_mps") or {}),
            "normal_stop_used": bool(suite.get("normal_stop_used", False)),
        },
        "lidar": {
            "latest_age_s": float(_safe_float(lidar.get("latest_age_s"), math.nan)),
            "latest_confidence": float(_safe_float(lidar.get("latest_confidence"), math.nan)),
            "accepted_delta": int(_safe_float(lidar.get("accepted_delta"), 0.0)),
            "fresh": bool(lidar.get("fresh", False)),
        },
        "determinism": {
            "policy": str(determinism.get("policy", "") or ""),
        },
        "execution_mode": str(suite.get("execution_mode", "UNKNOWN") or "UNKNOWN"),
        "motion_actual_ssot": str(suite.get("motion_actual_ssot", "EKF_POSE_ODOMETRY_SSOT") or "EKF_POSE_ODOMETRY_SSOT"),
        "truth_basis": dict(suite.get("truth_basis") or {}),
        "lidar_odom_status_truth": dict(suite.get("lidar_odom_status_truth") or {}),
        "lidar_odom_applied": bool(suite.get("lidar_odom_applied", False)),
        "lidar_odom_latest_age_s": suite.get("lidar_odom_latest_age_s"),
        "lidar_odom_latest_confidence": suite.get("lidar_odom_latest_confidence"),
        "encoder_pose_active_samples": int(_safe_float(suite.get("encoder_pose_active_samples"), 0.0)),
        "turn_primitive_requested": str(suite.get("turn_primitive_requested", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_limited": str(suite.get("turn_primitive_limited", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_executed": str(suite.get("turn_primitive_executed", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_actual": str(suite.get("turn_primitive_actual", "UNKNOWN") or "UNKNOWN"),
        "turn_primitives": dict(suite.get("turn_primitives") or {}),
        "artifacts": dict(suite.get("artifacts") or {}),
    }


def _print_compact_result(result: Dict[str, Any]) -> None:
    guard_hit = bool(
        result.get("command_progress_guard_triggered", False)
        or result.get("encoder_progress_guard_triggered", False)
    )
    guard_mode = "enforced" if bool(result.get("progress_guard_enforced", True)) else "soft"
    guard_text = f"{guard_mode}_hit" if guard_hit else "none"
    print(
        "Test={name} Result={result} Distance={distance:.3f}m Heading={heading:.2f}deg "
        "Guard={guard} Stop={stop} Failsafe={failsafe} Source={source}".format(
            name=result.get("test_name", ""),
            result="PASS" if result.get("success") else "FAIL",
            distance=float(result.get("estimated_distance_m", 0.0) or 0.0),
            heading=float(result.get("heading_change_deg", 0.0) or 0.0),
            guard=guard_text,
            stop="normal" if result.get("normal_stop_used") else ("emergency" if result.get("emergency_stop_triggered") else "n/a"),
            failsafe="yes" if result.get("failsafe_triggered") else "no",
            source=result.get("resolved_motion_source", "") or "n/a",
        )
    , flush=True)


def _print_preflight_compact(pf: Dict[str, Any], *, ok: bool, error: str = "") -> None:
    """Print preflight result as human/agent readable multi-line summary."""
    ready = pf.get("ready", False)
    can_move = pf.get("can_move_now", False)
    issues = pf.get("blocking_issues", [])
    odom = pf.get("odometry_mode", "?")
    determinism = pf.get("expected_determinism", "?")
    mode = pf.get("recommended_mode", "?")
    lidar_fresh = pf.get("lidar_fresh", False)
    lidar_age = pf.get("lidar_latest_age_s", -1)
    clearance = pf.get("forward_clearance", {})
    median_m = clearance.get("min_dist_median_m", "?")
    lq = pf.get("lidar_quality_gate", {})
    sig_q = pf.get("lidar_signal_quality", {})
    failsafe_reset = pf.get("failsafe_reset_performed", False)

    status_str = "READY" if (ok and ready and can_move) else "NOT READY"
    print(f"PREFLIGHT: {status_str}")
    if error:
        print(f"  error: {error}")
    if issues:
        for iss in issues:
            print(f"  blocking: {iss}")
    print(f"  odom={odom}  mode={mode}  determinism={determinism}")
    print(f"  lidar: fresh={lidar_fresh}  age={lidar_age:.2f}s  sig_ok={sig_q.get('ok', '?')}  conf={sig_q.get('signal_confidence', '?')}")
    print(f"  clearance: median={median_m}m")
    print(f"  encoder_distance_available={pf.get('encoder_distance_available', False)}  failsafe_reset={failsafe_reset}")
    print(f"  timestamp={pf.get('timestamp', '?')}")


def main() -> int:
    try:
        ensure_agent_system_prompt_loaded()
    except BootstrapGuardError as exc:
        payload = {
            "ok": False,
            "error": str(exc),
            "bootstrap_guard": {
                "loaded": False,
                "required_path": "project_rules/agent_system_prompt.txt",
            },
        }
        print(json.dumps(payload, sort_keys=True), flush=True)
        return 40

    parser = argparse.ArgumentParser(description="Compact bounded live motion probe for agentic testing.")
    parser.add_argument("--test-name", default="entry_gate_live_motion")
    parser.add_argument("--token", default=DEFAULT_TOKEN)
    parser.add_argument("--forward-speed-mps", type=float, default=DEFAULT_FORWARD_SPEED_MPS)
    parser.add_argument("--forward-distance-m", type=float, default=DEFAULT_FORWARD_DISTANCE_M)
    parser.add_argument("--forward-max-runtime-s", type=float, default=DEFAULT_FORWARD_MAX_RUNTIME_S)
    parser.add_argument("--forward-heading-abort-deg", type=float, default=DEFAULT_FORWARD_HEADING_ABORT_DEG)
    parser.add_argument("--forward-min-progress-m", type=float, default=0.0)
    parser.add_argument("--forward-min-progress-ratio", type=float, default=DEFAULT_FORWARD_MIN_PROGRESS_RATIO)
    parser.add_argument("--forward-target-completion-ratio", type=float, default=DEFAULT_FORWARD_TARGET_COMPLETION_RATIO)
    parser.add_argument("--min-lidar-accept-count", type=int, default=DEFAULT_MIN_LIDAR_ACCEPT_COUNT)
    parser.add_argument("--max-lidar-reject-count", type=int, default=DEFAULT_MAX_LIDAR_REJECT_COUNT)
    parser.add_argument("--min-odom-vs-command-ratio", type=float, default=DEFAULT_MIN_ODOM_VS_COMMAND_RATIO)
    parser.add_argument("--forward-clearance-m", type=float, default=DEFAULT_FORWARD_CLEARANCE_M)
    parser.add_argument(
        "--forward-clearance-mode",
        choices=("front-sector", "straight-corridor"),
        default=DEFAULT_FORWARD_CLEARANCE_MODE,
    )
    parser.add_argument("--forward-repeats", type=int, default=2)
    parser.add_argument("--forward-use-pose-closed-loop", action="store_true")
    parser.add_argument("--pose-target-lateral-m", type=float, default=0.0)
    parser.add_argument("--pose-target-heading-deg", type=float, default=0.0)
    parser.add_argument("--pose-target-heading-tolerance-deg", type=float, default=DEFAULT_POSE_TARGET_HEADING_TOLERANCE_DEG)
    parser.add_argument("--pose-target-omega-max-rad-s", type=float, default=0.0)
    parser.add_argument("--pose-target-handoff-completion-ratio", type=float, default=0.0)
    parser.add_argument("--pose-target-positive-lateral-scale", type=float, default=1.0)
    parser.add_argument("--pose-target-positive-heading-scale", type=float, default=1.0)
    parser.add_argument("--pose-target-positive-omega-scale", type=float, default=1.0)
    parser.add_argument("--pose-target-positive-speed-scale", type=float, default=1.0)
    parser.add_argument("--pose-target-negative-lateral-scale", type=float, default=1.0)
    parser.add_argument("--pose-target-negative-heading-scale", type=float, default=1.0)
    parser.add_argument("--pose-target-negative-omega-scale", type=float, default=1.0)
    parser.add_argument("--pose-target-negative-speed-scale", type=float, default=1.0)
    parser.add_argument("--pose-target-alternate-sign", action="store_true")
    parser.add_argument("--pose-target-continuous-sequence", action="store_true")
    parser.add_argument("--heading-test", action="store_true")
    parser.add_argument("--heading-angular-speed-dps", type=float, default=(0.30 * RAD_TO_DEG))
    parser.add_argument("--heading-omega-rad-s", type=float, default=None)
    parser.add_argument("--heading-target-deg", type=float, default=8.0)
    parser.add_argument("--heading-max-runtime-s", type=float, default=1.2)
    parser.add_argument("--arc-test", action="store_true")
    parser.add_argument("--arc-radius-m", type=float, default=0.3)
    parser.add_argument("--arc-speed-mps", type=float, default=0.10)
    parser.add_argument("--arc-angles-deg", type=str, default="30,60,90", help="Comma-separated arc angles in degrees")
    parser.add_argument("--arc-max-runtime-s", type=float, default=15.0)
    parser.add_argument("--keepalive-interval-s", type=float, default=DEFAULT_KEEPALIVE_INTERVAL_S)
    parser.add_argument("--stop-timeout-s", type=float, default=DEFAULT_STOP_TIMEOUT_S)
    parser.add_argument("--soft-heading-abort", action="store_true")
    parser.add_argument("--soft-command-progress-guard", action="store_true")
    parser.add_argument("--disable-probe-heading-hold", action="store_true")
    parser.add_argument("--skip-emergency-stop-test", action="store_true")
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--compact", action="store_true", help="Human/agent-readable multi-line output instead of raw JSON")
    args = parser.parse_args()
    heading_angular_speed_dps = float(args.heading_angular_speed_dps)
    if args.heading_omega_rad_s is not None:
        heading_angular_speed_dps = float(args.heading_omega_rad_s) * RAD_TO_DEG
    heading_omega_rad_s = float(heading_angular_speed_dps * DEG_TO_RAD)

    suite_start = time.monotonic()
    preflight: Dict[str, Any]
    forward_results: List[Dict[str, Any]] = []
    heading_result: Dict[str, Any] | None = None
    emergency_result: Dict[str, Any]

    if bool(args.preflight_only):
        try:
            preflight_only = _run_preflight(
                str(args.token),
                stop_timeout_s=float(args.stop_timeout_s),
                required_clearance_m=float(args.forward_clearance_m),
                forward_clearance_mode=str(args.forward_clearance_mode),
            )
            payload = {"ok": True, "preflight": preflight_only}
            _write_json_atomic(LATEST_PREFLIGHT_PATH, payload)
            if bool(args.compact):
                _print_preflight_compact(preflight_only, ok=True)
            else:
                print(json.dumps(payload, sort_keys=True), flush=True)
            return 0
        except Exception as exc:
            payload = {
                "ok": False,
                "preflight": {
                    "timestamp": _now_iso(),
                    "ready": False,
                    "can_move_now": False,
                    "blocking_issues": [str(exc)],
                    "controller_runtime_running": False,
                },
            }
            _write_json_atomic(LATEST_PREFLIGHT_PATH, payload)
            if bool(args.compact):
                _print_preflight_compact(payload["preflight"], ok=False, error=str(exc))
            else:
                print(json.dumps(payload, sort_keys=True), flush=True)
            return 2

    try:
        preflight = _run_preflight(
            str(args.token),
            stop_timeout_s=float(args.stop_timeout_s),
            required_clearance_m=float(args.forward_clearance_m),
            forward_clearance_mode=str(args.forward_clearance_mode),
        )
        print("Preflight=PASS", flush=True)

        forward_repeats = max(1, int(args.forward_repeats))
        suite_motion_source = "STATE" if bool(args.forward_use_pose_closed_loop) else "MANUAL"
        resolved_min_progress_m = _resolve_min_progress_m(
            target_distance_m=float(args.forward_distance_m),
            speed_mps=float(args.forward_speed_mps),
            max_runtime_s=float(args.forward_max_runtime_s),
            configured_min_progress_m=float(args.forward_min_progress_m),
            min_progress_ratio=float(args.forward_min_progress_ratio),
        )
        for idx in range(forward_repeats):
            pose_target_sign = -1.0 if bool(args.pose_target_alternate_sign) and idx % 2 else 1.0
            if pose_target_sign >= 0.0:
                lateral_scale = max(0.0, float(args.pose_target_positive_lateral_scale))
                heading_scale = max(0.0, float(args.pose_target_positive_heading_scale))
                omega_scale = max(0.0, float(args.pose_target_positive_omega_scale))
                speed_scale = max(0.0, float(args.pose_target_positive_speed_scale))
            else:
                lateral_scale = max(0.0, float(args.pose_target_negative_lateral_scale))
                heading_scale = max(0.0, float(args.pose_target_negative_heading_scale))
                omega_scale = max(0.0, float(args.pose_target_negative_omega_scale))
                speed_scale = max(0.0, float(args.pose_target_negative_speed_scale))
            segment_v_mps = float(args.forward_speed_mps) * float(speed_scale if speed_scale > 1e-6 else 1.0)
            segment_omega_max = float(args.pose_target_omega_max_rad_s) * float(omega_scale)
            if idx == 0:
                forward_name = "short_forward_a"
            elif idx == 1:
                forward_name = "short_forward_b"
            else:
                forward_name = f"short_forward_{idx + 1:02d}"
            forward_results.append(
                _run_bounded_motion_test(
                    forward_name,
                    token=str(args.token),
                    v_mps=float(segment_v_mps),
                    omega_rad_s=0.0,
                    target_distance_m=float(args.forward_distance_m),
                    min_progress_m=float(resolved_min_progress_m),
                    max_runtime_s=float(args.forward_max_runtime_s),
                    keepalive_interval_s=float(args.keepalive_interval_s),
                    stop_timeout_s=float(args.stop_timeout_s),
                    heading_abort_deg=float(args.forward_heading_abort_deg),
                    enforce_heading_abort=not bool(args.soft_heading_abort),
                    enable_probe_heading_hold=not bool(args.disable_probe_heading_hold),
                    target_completion_ratio=float(args.forward_target_completion_ratio),
                    min_lidar_accept_count=int(args.min_lidar_accept_count),
                    max_lidar_reject_count=int(args.max_lidar_reject_count),
                    min_odom_vs_command_ratio=float(args.min_odom_vs_command_ratio),
                    enforce_progress_guard=not bool(args.soft_command_progress_guard),
                    use_pose_closed_loop=bool(args.forward_use_pose_closed_loop),
                    pose_target_lateral_m=float(args.pose_target_lateral_m) * float(lateral_scale) * float(pose_target_sign),
                    pose_target_heading_delta_deg=float(args.pose_target_heading_deg) * float(heading_scale) * float(pose_target_sign),
                    pose_target_heading_tolerance_deg=float(args.pose_target_heading_tolerance_deg),
                    pose_target_omega_max_rad_s=float(segment_omega_max),
                    pose_target_handoff_completion_ratio=float(args.pose_target_handoff_completion_ratio),
                    stop_after_motion=not (
                        bool(args.pose_target_continuous_sequence)
                        and bool(args.forward_use_pose_closed_loop)
                        and idx < (forward_repeats - 1)
                    ),
                )
            )
            _print_compact_result(forward_results[-1])

        if bool(args.heading_test):
            heading_target_deg = max(1.0, abs(float(args.heading_target_deg)))
            heading_result = _run_heading_primitive_test(
                "small_heading_c",
                token=str(args.token),
                relative_deg=math.copysign(float(heading_target_deg), float(args.heading_target_deg)),
                max_runtime_s=min(
                    float(args.heading_max_runtime_s),
                    max(0.4, heading_target_deg / max(1.0, abs(float(heading_omega_rad_s))) / 2.5),
                ),
                stop_timeout_s=float(args.stop_timeout_s),
                commanded_angular_speed_dps=float(heading_angular_speed_dps),
                motion_source=suite_motion_source,
            )
            _print_compact_result(heading_result)

        if bool(args.arc_test):
            arc_angles = [float(a.strip()) for a in str(args.arc_angles_deg).split(",") if a.strip()]
            for idx, angle_deg in enumerate(arc_angles):
                arc_name = f"arc_{int(abs(angle_deg))}deg_{idx + 1:02d}"
                arc_result = _run_arc_segment_test(
                    arc_name,
                    token=str(args.token),
                    radius_m=float(args.arc_radius_m),
                    arc_angle_deg=float(angle_deg),
                    speed_mps=float(args.arc_speed_mps),
                    max_runtime_s=float(args.arc_max_runtime_s),
                    stop_timeout_s=float(args.stop_timeout_s),
                    motion_source=suite_motion_source,
                )
                forward_results.append(arc_result)
                _print_compact_result(arc_result)

        if bool(args.skip_emergency_stop_test):
            status_now = _read_json(STATUS_PATH)
            truth_surface = _truth_surface_from_status(status_now)
            emergency_result = {
                "timestamp": _now_iso(),
                "test_name": "emergency_stop_idle",
                "success": True,
                "skipped": True,
                "fail_reason": "",
                "odometry_mode": str((status_now or {}).get("odometry_mode", "")),
                "resolved_motion_source": "",
                "start_pose": _get_pose(status_now),
                "end_pose": _get_pose(status_now),
                "estimated_distance_m": 0.0,
                "heading_change_deg": 0.0,
                "lidar_status_summary": {},
                "loop_health_summary": {},
                "failsafe_triggered": False,
                "emergency_stop_triggered": False,
                "normal_stop_used": False,
                "max_runtime_s": 0.0,
                "actual_runtime_s": 0.0,
                **truth_surface,
            }
        else:
            try:
                emergency_result = _run_emergency_stop_test(str(args.token))
            except Exception as exc:
                status_now = _read_json(STATUS_PATH)
                truth_surface = _truth_surface_from_status(status_now)
                emergency_result = {
                    "timestamp": _now_iso(),
                    "test_name": "emergency_stop_idle",
                    "success": False,
                    "fail_reason": str(exc),
                    "odometry_mode": str((status_now or {}).get("odometry_mode", "")),
                    "resolved_motion_source": "",
                    "start_pose": _get_pose(status_now),
                    "end_pose": _get_pose(status_now),
                    "estimated_distance_m": 0.0,
                    "heading_change_deg": 0.0,
                    "lidar_status_summary": {},
                    "loop_health_summary": {},
                    "failsafe_triggered": False,
                    "emergency_stop_triggered": False,
                    "normal_stop_used": False,
                    "max_runtime_s": float(DEFAULT_EMERGENCY_TIMEOUT_S + DEFAULT_RESET_TIMEOUT_S),
                    "actual_runtime_s": 0.0,
                    **truth_surface,
                }
        _print_compact_result(emergency_result)

    except Exception as exc:
        _safe_stop_best_effort(str(args.token))
        status_now = _read_json(STATUS_PATH)
        truth_surface = _truth_surface_from_status(status_now)
        preflight = {
            "timestamp": _now_iso(),
            "ready": False,
            "can_move_now": False,
            "blocking_issues": [str(exc)],
            "controller_runtime_running": False,
        }
        emergency_result = {
            "timestamp": _now_iso(),
            "test_name": "emergency_stop_idle",
            "success": False,
            "fail_reason": "not_run_due_to_preflight_failure",
            "odometry_mode": "",
            "resolved_motion_source": "",
            "start_pose": {},
            "end_pose": {},
            "estimated_distance_m": 0.0,
            "heading_change_deg": 0.0,
            "lidar_status_summary": {},
            "loop_health_summary": {},
            "failsafe_triggered": False,
            "emergency_stop_triggered": False,
            "normal_stop_used": False,
            "max_runtime_s": 0.0,
            "actual_runtime_s": 0.0,
            **truth_surface,
        }

    suite_result = build_suite_rollup(
        test_name=str(args.test_name),
        preflight=preflight,
        forward_results=forward_results,
        heading_result=heading_result,
        emergency_result=emergency_result,
        suite_runtime_s=max(0.0, time.monotonic() - suite_start),
    )

    summary_report = build_compact_summary_report(suite_result)
    immutable_artifacts = _write_immutable_suite_artifacts(
        test_name=str(args.test_name),
        suite_result=suite_result,
        summary_report=summary_report,
    )
    suite_result["immutable_artifacts"] = dict(immutable_artifacts)

    _write_json_atomic(LATEST_RESULT_PATH, suite_result)
    _write_json_atomic(LATEST_SUMMARY_PATH, summary_report)
    _append_jsonl(HISTORY_PATH, suite_result)
    print(f"Artifact={LATEST_RESULT_PATH.relative_to(PROJECT_ROOT)}", flush=True)
    print(f"Summary={LATEST_SUMMARY_PATH.relative_to(PROJECT_ROOT)}", flush=True)
    print(f"ImmutableResult={immutable_artifacts['result']}", flush=True)
    print(f"ImmutableSummary={immutable_artifacts['summary']}", flush=True)

    return 0 if bool(suite_result.get("success", False)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
