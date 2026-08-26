#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LidarOdometry: Thread-safe scan-to-map LIDAR measurement gate for LIDAR_FIRST.

Consumes the canonical LidarEstimator scan-to-map output and provides gated absolute pose updates
for the EKF.  Runs within the LidarService thread (no new thread created).
The control loop reads the latest result via get_odometry() without blocking.
"""

from __future__ import annotations

import copy
import math
import threading
import time
from typing import Any, Dict, Optional


CONTROL_LOOP_DIAGNOSTIC_KEYS = frozenset(
    {
        "control_loop_lidar_apply_status",
        "control_loop_lidar_flow",
        "scan_seq",
        "raw_scan_rate_hz",
        "raw_scan_latest_age_s",
        "raw_scan_max_gap_s",
        "matcher_latency_ms",
        "matcher_latency_p50_ms",
        "matcher_latency_p95_ms",
        "matcher_latency_max_ms",
        "matcher_queue_delay_ms",
        "matcher_runtime_ms",
        "matcher_cpu_ms",
        "matcher_process_pid",
        "matcher_process_alive",
        "matcher_process_cpu_time_s",
        "matcher_process_rss_kb",
        "matcher_process_peak_rss_kb",
        "matcher_process_input_drops_total",
        "matcher_process_output_drops_total",
        "matcher_contract_id",
        "matcher_confidence_model",
        "matcher_integrity_model",
        "matcher_integrity_model",
        "matcher_process_start_method",
        "matcher_input_queue_capacity",
        "matcher_result_queue_capacity",
        "matcher_max_input_age_s",
        "matcher_max_result_age_s",
        "matcher_queue_depth",
        "matcher_result_queue_depth",
        "matcher_stale_result_drops",
        "matcher_process_errors",
        "matcher_transport",
        "localization_health",
        "initialization_gate",
        "ekf_applied_samples_total",
        "ekf_applied_gap_s",
        "ekf_applied_cadence_hz",
        "ekf_applied_gap_watchdog",
    }
)


def _as_pose_dict(value: Any) -> Optional[Dict[str, float]]:
    pose = None
    if isinstance(value, dict):
        pose = value
    elif isinstance(value, (list, tuple)) and len(value) >= 3:
        pose = {"x": value[0], "y": value[1], "theta": value[2]}
    if pose is None:
        return None
    try:
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        theta = float(pose.get("theta", 0.0))
    except Exception:
        return None
    if not all(math.isfinite(v) for v in (x, y, theta)):
        return None
    return {"x": x, "y": y, "theta": theta}


def _as_finite_or_none(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except Exception:
        return None
    if not math.isfinite(out):
        return None
    return out


def _as_int_or_zero(value: Any) -> int:
    try:
        return int(value)
    except Exception:
        return 0


def _as_positive_int_or_none(value: Any) -> Optional[int]:
    try:
        out = int(value)
    except Exception:
        return None
    return out if out > 0 else None


def _copy_control_diagnostic_value(value: Any, *, depth: int = 0) -> Any:
    if depth >= 4:
        return value
    if isinstance(value, dict):
        return {
            key: _copy_control_diagnostic_value(item, depth=depth + 1)
            for key, item in value.items()
        }
    if isinstance(value, list):
        return [_copy_control_diagnostic_value(item, depth=depth + 1) for item in value]
    if isinstance(value, tuple):
        return tuple(_copy_control_diagnostic_value(item, depth=depth + 1) for item in value)
    return value


class LidarOdometry:
    """
    Consumes scan-matching results from LidarEstimator and produces
    gated absolute-pose measurements for the EKF.

    Thread safety:
        - on_scan_result() is called from the LidarService worker thread.
        - get_odometry()  is called from the 50 Hz control loop thread.
        Both share _lock to exchange the latest accepted result atomically.
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        cfg = dict(config or {})
        self._enabled = bool(cfg.get("enabled", True))
        self._max_scan_age_s = float(cfg.get("max_scan_age_s", 0.15))
        self._min_confidence = float(cfg.get("min_confidence", 0.25))
        self._max_delta_m = float(cfg.get("max_delta_m", 0.5))
        self._max_delta_rad = float(cfg.get("max_delta_rad", 0.5))
        self._bootstrap_anchor_max_delta_m = float(cfg.get("bootstrap_anchor_max_delta_m", self._max_delta_m))
        self._bootstrap_anchor_max_delta_rad = float(cfg.get("bootstrap_anchor_max_delta_rad", self._max_delta_rad))
        self._r_scale_by_confidence = bool(cfg.get("r_scale_by_confidence", True))
        self._jump_gate_max_speed_mps = max(0.0, float(cfg.get("jump_gate_max_speed_mps", 0.80)))
        self._jump_gate_max_yaw_rate_rad_s = max(0.0, float(cfg.get("jump_gate_max_yaw_rate_rad_s", 1.20)))
        self._jump_gate_dt_cap_s = max(0.0, float(cfg.get("jump_gate_dt_cap_s", 1.0)))
        self._relocalization_jump_multiplier = max(1.0, float(cfg.get("relocalization_jump_multiplier", 2.5)))
        self._relocalization_max_delta_m = max(
            self._max_delta_m,
            float(cfg.get("relocalization_max_delta_m", max(1.0, self._max_delta_m * 4.0))),
        )
        self._relocalization_max_delta_rad = max(
            self._max_delta_rad,
            float(cfg.get("relocalization_max_delta_rad", max(0.8, self._max_delta_rad * 4.0))),
        )

        self._lock = threading.Lock()
        self._latest: Optional[Dict[str, Any]] = None
        self._last_candidate: Optional[Dict[str, Any]] = None
        self._bootstrap_anchor_pose: Optional[Dict[str, float]] = None
        self._bootstrap_anchor_active = False
        self._lidar_odometry_measurement_seq = 0
        self._last_matcher_result_id: Optional[int] = None
        self._last_source_raw_scan_id: Optional[int] = None
        self._consumed = True  # True = control loop already read the last result
        self._last_decision = "missing"
        self._last_poll_status = "missing"
        self._stats = {
            "total": 0,
            "accepted": 0,
            "rejected_low_confidence": 0,
            "rejected_large_jump": 0,
            "rejected_bootstrap_jump": 0,
            "rejected_invalid": 0,
            "rejected_disabled": 0,
            "rejected_duplicate_matcher_result": 0,
            "rejected_duplicate_raw_scan": 0,
            "on_scan_result_called": 0,
            "candidate_created": 0,
            "raw_scan_id": None,
            "raw_scan_timestamp": None,
            "matcher_result_id": None,
            "candidate_id": None,
            "matcher_result_timestamp": None,
            "candidate_source_raw_scan_id": None,
            "candidate_source_raw_scan_timestamp": None,
            "lidar_odometry_measurement_id": None,
            "lidar_odometry_measurement_timestamp": None,
            "measurement_source_matcher_result_id": None,
            "measurement_source_raw_scan_id": None,
            "measurement_source_raw_scan_timestamp": None,
            "promotion_attempted": 0,
            "promotion_result": "not_attempted",
            "reject_reason": "",
            "candidate_confidence": None,
            "candidate_measurement_confidence": None,
            "candidate_integrity_score": None,
            "candidate_integrity_state": "INCOMPLETE",
            "candidate_r_scale": None,
            "candidate_age_s": None,
            "latest_present": False,
            "latest_r_scale": None,
            "latest_measurement_confidence": None,
            "latest_integrity_score": None,
            "latest_integrity_state": "INCOMPLETE",
            "bootstrap_anchor_active": False,
            "bootstrap_anchor_pose": None,
            "bootstrap_anchor_delta_m": None,
            "bootstrap_anchor_delta_rad": None,
            "r_scale_by_confidence": bool(self._r_scale_by_confidence),
            "get_odometry_result": "",
            "control_loop_lidar_apply_status": "",
            # LIDAR estimator debug telemetria (scan->odom adatút láthatóság).
            "pose_ref_current": None,
            "prev_pose_ref": None,
            "scan_to_map_seed": None,
            "map_frame_id": "",
            "map_frame_owner": "",
            "yaw_convention": "",
            "last_lidar_pose_before": None,
            "last_lidar_pose": None,
            "dx": None,
            "dy": None,
            "dtheta": None,
            "x_lidar_raw": None,
            "y_lidar_raw": None,
            "theta_lidar_raw": None,
            # Scan-to-map / localization runtime status.
            "matcher_mode": "",
            "localization_status": "",
            "pose_update_event": "",
            "pose_event_step_limited": False,
            "pose_event_raw_delta_m": None,
            "pose_event_raw_delta_rad": None,
            "tracking_reacquire_streak": 0,
            "tracking_reacquire_required": 0,
            "tracking_ready": False,
            "tracking_loss_latched": False,
            "tracking_direction_checked": False,
            "tracking_direction_consistent": True,
            "tracking_direction_rejected": False,
            "tracking_direction_rejected_total": 0,
            "tracking_reference_delta_m": None,
            "tracking_reference_linear_mps": None,
            "tracking_candidate_projection_m": None,
            "tracking_backtrack_debt_m": 0.0,
            "tracking_direction_reference_source": "",
            "tracking_direction_backtrack_tolerance_m": None,
            "tracking_candidate_pose_ref": None,
            "last_lidar_pose_ref": None,
            "relocalized": False,
            "relocalization_attempted": False,
            "relocalization_reason": "",
            "loop_closure_detected": False,
            "loop_closure_applied": False,
            "loop_closure_delta_m": None,
            "loop_closure_delta_rad": None,
            "local_map_points": 0,
            "local_map_keyframes": 0,
            # Driver diagnosztika (lidar driver -> estimator -> odom lánc).
            "driver_connected": None,
            "driver_last_data_age_s": None,
            "driver_rx_buffer_len": None,
            "driver_invalid_packet_count": None,
            "driver_reconnect_count": None,
            # Dynamic jump-gate diagnostics.
            "jump_gate_dynamic_max_m": None,
            "jump_gate_dynamic_max_rad": None,
            "jump_gate_dt_s": None,
            "jump_gate_relocalized": False,
            "control_lock_miss_count": 0,
            "control_lock_busy": False,
        }
        self._control_lock_miss_count = 0
        self._stats_snapshot = dict(self._stats)

    def _record_control_lock_miss(self) -> None:
        self._control_lock_miss_count += 1

    def _publish_stats_snapshot_locked(self, stats: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        snapshot = dict(self._stats if stats is None else stats)
        snapshot["control_lock_miss_count"] = int(self._control_lock_miss_count)
        snapshot["control_lock_busy"] = False
        self._stats["control_lock_miss_count"] = int(self._control_lock_miss_count)
        self._stats["control_lock_busy"] = False
        self._stats_snapshot = dict(snapshot)
        return snapshot

    # ------------------------------------------------------------------
    # Called from LidarService / LidarEstimator thread
    # ------------------------------------------------------------------

    def on_scan_result(self, result: Dict[str, Any]) -> None:
        """
        Accept a scan-matching result dict from LidarEstimator.process_scan().

        Expected keys:
            lidar_pose_x, lidar_pose_y, lidar_pose_theta, lidar_pose_confidence
        """
        with self._lock:
            now = time.monotonic()
            self._stats["on_scan_result_called"] += 1
            self._stats["total"] += 1

            matcher_result_id = _as_positive_int_or_none(result.get("matcher_result_id"))
            candidate_id = _as_positive_int_or_none(result.get("candidate_id"))
            if matcher_result_id is None:
                matcher_result_id = candidate_id
            raw_scan_id = _as_positive_int_or_none(
                result.get("matcher_source_raw_scan_id", result.get("raw_scan_id"))
            )
            raw_scan_timestamp = _as_finite_or_none(
                result.get(
                    "matcher_source_raw_scan_timestamp",
                    result.get("raw_scan_timestamp", result.get("raw_scan_mono_ts")),
                )
            )
            matcher_result_timestamp = _as_finite_or_none(
                result.get("matcher_result_timestamp")
            )
            if matcher_result_timestamp is None:
                matcher_result_timestamp = float(now)

            if (
                matcher_result_id is not None
                and self._last_matcher_result_id is not None
                and int(matcher_result_id) <= int(self._last_matcher_result_id)
            ):
                self._stats["rejected_duplicate_matcher_result"] += 1
                self._last_decision = "rejected_duplicate_matcher_result"
                self._stats["promotion_result"] = "rejected_duplicate_matcher_result"
                self._stats["reject_reason"] = "duplicate_or_out_of_order_matcher_result"
                return
            if (
                raw_scan_id is not None
                and self._last_source_raw_scan_id is not None
                and int(raw_scan_id) <= int(self._last_source_raw_scan_id)
            ):
                self._stats["rejected_duplicate_raw_scan"] += 1
                self._last_decision = "rejected_duplicate_raw_scan"
                self._stats["promotion_result"] = "rejected_duplicate_raw_scan"
                self._stats["reject_reason"] = "duplicate_or_out_of_order_raw_scan"
                return
            if matcher_result_id is not None:
                self._last_matcher_result_id = int(matcher_result_id)
            if raw_scan_id is not None:
                self._last_source_raw_scan_id = int(raw_scan_id)

            if not self._enabled:
                self._stats["rejected_disabled"] += 1
                self._last_candidate = {
                    "timestamp": now,
                    "confidence": None,
                }
                self._last_decision = "rejected_disabled"
                return

            try:
                x = float(result.get("lidar_pose_x", 0.0))
                y = float(result.get("lidar_pose_y", 0.0))
                theta = float(result.get("lidar_pose_theta", 0.0))
                confidence = float(result.get("lidar_pose_confidence", 0.0))
            except (TypeError, ValueError):
                self._stats["rejected_invalid"] += 1
                self._last_candidate = {
                    "timestamp": now,
                    "confidence": None,
                }
                self._last_decision = "rejected_invalid"
                return

            matcher_quality = dict(result.get("matcher_quality") or {})
            measurement_confidence = _as_finite_or_none(
                result.get(
                    "measurement_confidence",
                    matcher_quality.get("measurement_confidence", confidence),
                )
            )
            if measurement_confidence is None:
                measurement_confidence = float(confidence)
            integrity_score = _as_finite_or_none(
                result.get(
                    "localization_integrity_score",
                    matcher_quality.get("localization_integrity_score", confidence),
                )
            )
            if integrity_score is None:
                integrity_score = float(confidence)
            integrity_state = str(
                result.get(
                    "localization_integrity_state",
                    matcher_quality.get("integrity_state", "LEGACY"),
                )
                or "LEGACY"
            ).upper()

            self._stats["candidate_created"] += 1
            candidate_sequence = int(self._stats["candidate_created"])
            if matcher_result_id is None:
                matcher_result_id = int(candidate_sequence)
                self._last_matcher_result_id = int(matcher_result_id)
            if candidate_id is None:
                candidate_id = int(matcher_result_id)

            matcher_mode = str(result.get("matcher_mode", "") or "")
            localization_status = str(result.get("localization_status", "") or "")
            pose_update_event = str(result.get("pose_update_event", "") or "")
            pose_event_step_limited = bool(result.get("pose_event_step_limited", False))
            pose_event_raw_delta_m = _as_finite_or_none(result.get("pose_event_raw_delta_m"))
            pose_event_raw_delta_rad = _as_finite_or_none(result.get("pose_event_raw_delta_rad"))
            tracking_reacquire_streak = max(0, _as_int_or_zero(result.get("tracking_reacquire_streak", 0)))
            tracking_reacquire_required = max(0, _as_int_or_zero(result.get("tracking_reacquire_required", 0)))
            tracking_ready = bool(result.get("tracking_ready", False))
            tracking_loss_latched = bool(result.get("tracking_loss_latched", False))
            tracking_direction_checked = bool(result.get("tracking_direction_checked", False))
            tracking_direction_consistent = bool(result.get("tracking_direction_consistent", True))
            tracking_direction_rejected = bool(result.get("tracking_direction_rejected", False))
            tracking_direction_rejected_total = max(
                0,
                _as_int_or_zero(result.get("tracking_direction_rejected_total", 0)),
            )
            tracking_reference_delta_m = _as_finite_or_none(result.get("tracking_reference_delta_m"))
            tracking_reference_linear_mps = _as_finite_or_none(
                result.get("tracking_reference_linear_mps")
            )
            tracking_candidate_projection_m = _as_finite_or_none(
                result.get("tracking_candidate_projection_m")
            )
            tracking_backtrack_debt_m = _as_finite_or_none(result.get("tracking_backtrack_debt_m"))
            if tracking_backtrack_debt_m is None:
                tracking_backtrack_debt_m = 0.0
            tracking_direction_reference_source = str(
                result.get("tracking_direction_reference_source", "") or ""
            )
            tracking_direction_backtrack_tolerance_m = _as_finite_or_none(
                result.get("tracking_direction_backtrack_tolerance_m")
            )
            relocalized = bool(result.get("relocalized", False))
            relocalization_attempted = bool(result.get("relocalization_attempted", False))
            relocalization_reason = str(result.get("relocalization_reason", "") or "")
            loop_closure_detected = bool(result.get("loop_closure_detected", False))
            loop_closure_applied = bool(result.get("loop_closure_applied", False))
            loop_closure_delta_m = _as_finite_or_none(result.get("loop_closure_delta_m"))
            loop_closure_delta_rad = _as_finite_or_none(result.get("loop_closure_delta_rad"))
            local_map_points = max(0, _as_int_or_zero(result.get("local_map_points", 0)))
            local_map_keyframes = max(0, _as_int_or_zero(result.get("local_map_keyframes", 0)))
            driver_connected = result.get("driver_connected")
            if driver_connected is not None:
                driver_connected = bool(driver_connected)
            driver_last_data_age_s = _as_finite_or_none(result.get("driver_last_data_age_s"))
            driver_rx_buffer_len = _as_finite_or_none(result.get("driver_rx_buffer_len"))
            driver_invalid_packet_count = _as_finite_or_none(result.get("driver_invalid_packet_count"))
            driver_reconnect_count = _as_finite_or_none(result.get("driver_reconnect_count"))

            self._stats["pose_ref_current"] = _as_pose_dict(result.get("pose_ref_current"))
            self._stats["prev_pose_ref"] = _as_pose_dict(result.get("prev_pose_ref"))
            self._stats["scan_to_map_seed"] = _as_pose_dict(result.get("scan_to_map_seed"))
            self._stats["map_frame_id"] = str(result.get("map_frame_id", "") or "")
            self._stats["map_frame_owner"] = str(result.get("map_frame_owner", "") or "")
            self._stats["yaw_convention"] = str(result.get("yaw_convention", "") or "")
            self._stats["last_lidar_pose_before"] = _as_pose_dict(result.get("last_lidar_pose_before"))
            self._stats["last_lidar_pose"] = _as_pose_dict(result.get("last_lidar_pose"))
            self._stats["tracking_candidate_pose_ref"] = _as_pose_dict(
                result.get("tracking_candidate_pose_ref")
            )
            self._stats["last_lidar_pose_ref"] = _as_pose_dict(result.get("last_lidar_pose_ref"))
            self._stats["dx"] = _as_finite_or_none(result.get("dx"))
            self._stats["dy"] = _as_finite_or_none(result.get("dy"))
            self._stats["dtheta"] = _as_finite_or_none(result.get("dtheta"))
            self._stats["x_lidar_raw"] = _as_finite_or_none(result.get("x_lidar_raw"))
            self._stats["y_lidar_raw"] = _as_finite_or_none(result.get("y_lidar_raw"))
            self._stats["theta_lidar_raw"] = _as_finite_or_none(result.get("theta_lidar_raw"))
            self._stats["matcher_mode"] = matcher_mode
            self._stats["matcher_quality"] = copy.deepcopy(matcher_quality)
            self._stats["matcher_degenerate"] = bool(
                result.get("matcher_degenerate", False)
            )
            self._stats["matcher_degeneracy_reasons"] = list(
                result.get("matcher_degeneracy_reasons") or []
            )
            self._stats["localization_status"] = localization_status
            self._stats["pose_update_event"] = pose_update_event
            self._stats["pose_event_step_limited"] = bool(pose_event_step_limited)
            self._stats["pose_event_raw_delta_m"] = pose_event_raw_delta_m
            self._stats["pose_event_raw_delta_rad"] = pose_event_raw_delta_rad
            self._stats["tracking_reacquire_streak"] = int(tracking_reacquire_streak)
            self._stats["tracking_reacquire_required"] = int(tracking_reacquire_required)
            self._stats["tracking_ready"] = bool(tracking_ready)
            self._stats["tracking_loss_latched"] = bool(tracking_loss_latched)
            self._stats["tracking_direction_checked"] = bool(tracking_direction_checked)
            self._stats["tracking_direction_consistent"] = bool(tracking_direction_consistent)
            self._stats["tracking_direction_rejected"] = bool(tracking_direction_rejected)
            self._stats["tracking_direction_rejected_total"] = int(tracking_direction_rejected_total)
            self._stats["tracking_reference_delta_m"] = tracking_reference_delta_m
            self._stats["tracking_reference_linear_mps"] = tracking_reference_linear_mps
            self._stats["tracking_candidate_projection_m"] = tracking_candidate_projection_m
            self._stats["tracking_backtrack_debt_m"] = float(tracking_backtrack_debt_m)
            self._stats["tracking_direction_reference_source"] = tracking_direction_reference_source
            self._stats["tracking_direction_backtrack_tolerance_m"] = (
                tracking_direction_backtrack_tolerance_m
            )
            self._stats["relocalized"] = bool(relocalized)
            self._stats["relocalization_attempted"] = bool(relocalization_attempted)
            self._stats["relocalization_reason"] = relocalization_reason
            self._stats["loop_closure_detected"] = bool(loop_closure_detected)
            self._stats["loop_closure_applied"] = bool(loop_closure_applied)
            self._stats["loop_closure_delta_m"] = loop_closure_delta_m
            self._stats["loop_closure_delta_rad"] = loop_closure_delta_rad
            self._stats["local_map_points"] = int(local_map_points)
            self._stats["local_map_keyframes"] = int(local_map_keyframes)
            self._stats["driver_connected"] = driver_connected
            self._stats["driver_last_data_age_s"] = driver_last_data_age_s
            self._stats["driver_rx_buffer_len"] = int(driver_rx_buffer_len) if driver_rx_buffer_len is not None else None
            self._stats["driver_invalid_packet_count"] = (
                int(driver_invalid_packet_count) if driver_invalid_packet_count is not None else None
            )
            self._stats["driver_reconnect_count"] = int(driver_reconnect_count) if driver_reconnect_count is not None else None
            self._stats["bootstrap_anchor_active"] = bool(self._bootstrap_anchor_active)
            self._stats["bootstrap_anchor_pose"] = _as_pose_dict(self._bootstrap_anchor_pose)
            self._stats["bootstrap_anchor_delta_m"] = None
            self._stats["bootstrap_anchor_delta_rad"] = None
            self._stats["jump_gate_dynamic_max_m"] = None
            self._stats["jump_gate_dynamic_max_rad"] = None
            self._stats["jump_gate_dt_s"] = None
            self._stats["jump_gate_relocalized"] = bool(relocalized)
            self._stats["raw_scan_id"] = raw_scan_id
            self._stats["raw_scan_timestamp"] = raw_scan_timestamp
            self._stats["matcher_result_id"] = int(matcher_result_id)
            self._stats["candidate_id"] = int(candidate_id)
            self._stats["matcher_result_timestamp"] = float(matcher_result_timestamp)
            self._stats["candidate_source_raw_scan_id"] = raw_scan_id
            self._stats["candidate_source_raw_scan_timestamp"] = raw_scan_timestamp
            self._stats["candidate_measurement_confidence"] = float(
                measurement_confidence
            )
            self._stats["candidate_integrity_score"] = float(integrity_score)
            self._stats["candidate_integrity_state"] = str(integrity_state)

            self._last_candidate = {
                "x": x,
                "y": y,
                "theta": theta,
                "confidence": confidence,
                "measurement_confidence": float(measurement_confidence),
                "integrity_score": float(integrity_score),
                "integrity_state": str(integrity_state),
                "r_scale": self._measurement_r_scale(confidence),
                "timestamp": float(matcher_result_timestamp),
                "matcher_result_id": int(matcher_result_id),
                "candidate_id": int(candidate_id),
                "raw_scan_id": raw_scan_id,
                "raw_scan_timestamp": raw_scan_timestamp,
                "matcher_result_timestamp": float(matcher_result_timestamp),
                "scan_to_map_seed": _as_pose_dict(result.get("scan_to_map_seed")),
                "map_frame_id": str(result.get("map_frame_id", "") or ""),
                "map_frame_owner": str(result.get("map_frame_owner", "") or ""),
                "yaw_convention": str(result.get("yaw_convention", "") or ""),
                "matcher_mode": matcher_mode,
                "localization_status": localization_status,
                "pose_update_event": pose_update_event,
                "pose_event_step_limited": bool(pose_event_step_limited),
                "pose_event_raw_delta_m": pose_event_raw_delta_m,
                "pose_event_raw_delta_rad": pose_event_raw_delta_rad,
                "tracking_reacquire_streak": int(tracking_reacquire_streak),
                "tracking_reacquire_required": int(tracking_reacquire_required),
                "tracking_ready": bool(tracking_ready),
                "tracking_loss_latched": bool(tracking_loss_latched),
                "tracking_direction_checked": bool(tracking_direction_checked),
                "tracking_direction_consistent": bool(tracking_direction_consistent),
                "tracking_direction_rejected": bool(tracking_direction_rejected),
                "tracking_direction_rejected_total": int(tracking_direction_rejected_total),
                "tracking_reference_delta_m": tracking_reference_delta_m,
                "tracking_reference_linear_mps": tracking_reference_linear_mps,
                "tracking_candidate_projection_m": tracking_candidate_projection_m,
                "tracking_backtrack_debt_m": float(tracking_backtrack_debt_m),
                "tracking_direction_reference_source": tracking_direction_reference_source,
                "tracking_direction_backtrack_tolerance_m": tracking_direction_backtrack_tolerance_m,
                "relocalized": bool(relocalized),
                "relocalization_attempted": bool(relocalization_attempted),
                "relocalization_reason": relocalization_reason,
                "loop_closure_detected": bool(loop_closure_detected),
                "loop_closure_applied": bool(loop_closure_applied),
                "loop_closure_delta_m": loop_closure_delta_m,
                "loop_closure_delta_rad": loop_closure_delta_rad,
                "local_map_points": int(local_map_points),
                "local_map_keyframes": int(local_map_keyframes),
                "driver_connected": driver_connected,
                "driver_last_data_age_s": driver_last_data_age_s,
                "driver_rx_buffer_len": int(driver_rx_buffer_len) if driver_rx_buffer_len is not None else None,
                "driver_invalid_packet_count": (
                    int(driver_invalid_packet_count) if driver_invalid_packet_count is not None else None
                ),
                "driver_reconnect_count": int(driver_reconnect_count) if driver_reconnect_count is not None else None,
            }
            self._stats["candidate_confidence"] = confidence
            self._stats["candidate_r_scale"] = self._last_candidate["r_scale"]
            self._stats["promotion_attempted"] += 1
            self._stats["promotion_result"] = "pending"
            self._stats["reject_reason"] = ""

            if not all(math.isfinite(v) for v in (x, y, theta, confidence)):
                self._stats["rejected_invalid"] += 1
                self._last_decision = "rejected_invalid"
                self._stats["promotion_result"] = "rejected_invalid"
                self._stats["reject_reason"] = "invalid_finite"
                return

            if confidence < self._min_confidence:
                self._stats["rejected_low_confidence"] += 1
                self._last_decision = "rejected_low_confidence"
                self._stats["promotion_result"] = "rejected_low_confidence"
                self._stats["reject_reason"] = "low_confidence"
                return

            if self._latest is None and self._bootstrap_anchor_active and self._bootstrap_anchor_pose is not None:
                anchor = self._bootstrap_anchor_pose
                dx_anchor = float(x) - float(anchor["x"])
                dy_anchor = float(y) - float(anchor["y"])
                dtheta_anchor = abs(_normalize_angle(float(theta) - float(anchor["theta"])))
                anchor_dist = float(math.hypot(dx_anchor, dy_anchor))
                self._stats["bootstrap_anchor_delta_m"] = anchor_dist
                self._stats["bootstrap_anchor_delta_rad"] = float(dtheta_anchor)
                anchor_max_delta_m = float(self._bootstrap_anchor_max_delta_m)
                anchor_max_delta_rad = float(self._bootstrap_anchor_max_delta_rad)
                if relocalized:
                    anchor_max_delta_m = max(anchor_max_delta_m, float(self._relocalization_max_delta_m))
                    anchor_max_delta_rad = max(anchor_max_delta_rad, float(self._relocalization_max_delta_rad))
                if (
                    anchor_dist > float(anchor_max_delta_m)
                    or float(dtheta_anchor) > float(anchor_max_delta_rad)
                ):
                    self._stats["rejected_large_jump"] += 1
                    self._stats["rejected_bootstrap_jump"] += 1
                    self._last_decision = "rejected_bootstrap_jump"
                    self._stats["promotion_result"] = "rejected_bootstrap_jump"
                    self._stats["reject_reason"] = "bootstrap_jump_too_large"
                    return

            # Jump gating: reject implausible deltas relative to last accepted pose
            if self._latest is not None:
                dx = x - self._latest["x"]
                dy = y - self._latest["y"]
                dth = abs(_normalize_angle(theta - self._latest["theta"]))
                dist = math.hypot(dx, dy)
                dt_since_latest = max(0.0, float(now) - float(self._latest.get("timestamp", now)))
                dt_eff = dt_since_latest
                if self._jump_gate_dt_cap_s > 0.0:
                    dt_eff = min(dt_eff, float(self._jump_gate_dt_cap_s))
                dynamic_max_delta_m = float(self._max_delta_m) + float(self._jump_gate_max_speed_mps) * float(dt_eff)
                dynamic_max_delta_rad = float(self._max_delta_rad) + float(self._jump_gate_max_yaw_rate_rad_s) * float(dt_eff)
                if relocalized:
                    dynamic_max_delta_m = max(
                        dynamic_max_delta_m,
                        float(self._max_delta_m) * float(self._relocalization_jump_multiplier),
                        float(self._relocalization_max_delta_m),
                    )
                    dynamic_max_delta_rad = max(
                        dynamic_max_delta_rad,
                        float(self._max_delta_rad) * float(self._relocalization_jump_multiplier),
                        float(self._relocalization_max_delta_rad),
                    )
                self._stats["jump_gate_dt_s"] = float(dt_eff)
                self._stats["jump_gate_dynamic_max_m"] = float(dynamic_max_delta_m)
                self._stats["jump_gate_dynamic_max_rad"] = float(dynamic_max_delta_rad)
                if dist > float(dynamic_max_delta_m) or dth > float(dynamic_max_delta_rad):
                    self._stats["rejected_large_jump"] += 1
                    self._last_decision = "rejected_jump"
                    self._stats["promotion_result"] = "rejected_jump"
                    self._stats["reject_reason"] = "jump_too_large"
                    return

            self._lidar_odometry_measurement_seq += 1
            measurement_id = int(self._lidar_odometry_measurement_seq)
            measurement_timestamp = float(now)
            self._latest = {
                "x": x,
                "y": y,
                "theta": theta,
                "confidence": confidence,
                "measurement_confidence": float(measurement_confidence),
                "integrity_score": float(integrity_score),
                "integrity_state": str(integrity_state),
                "r_scale": self._last_candidate["r_scale"],
                "timestamp": measurement_timestamp,
                "lidar_odometry_measurement_id": int(measurement_id),
                "lidar_odometry_measurement_timestamp": float(measurement_timestamp),
                "measurement_source_matcher_result_id": int(matcher_result_id),
                "measurement_source_candidate_id": int(candidate_id),
                "measurement_source_raw_scan_id": raw_scan_id,
                "measurement_source_raw_scan_timestamp": raw_scan_timestamp,
                "matcher_result_id": int(matcher_result_id),
                "candidate_id": int(candidate_id),
                "raw_scan_id": raw_scan_id,
                "raw_scan_timestamp": raw_scan_timestamp,
                "matcher_result_timestamp": float(matcher_result_timestamp),
                "scan_to_map_seed": _as_pose_dict(result.get("scan_to_map_seed")),
                "map_frame_id": str(result.get("map_frame_id", "") or ""),
                "map_frame_owner": str(result.get("map_frame_owner", "") or ""),
                "yaw_convention": str(result.get("yaw_convention", "") or ""),
                "matcher_mode": matcher_mode,
                "localization_status": (
                    localization_status
                    or ("relocalized" if relocalized else "tracking")
                ),
                "pose_update_event": pose_update_event,
                "pose_event_step_limited": bool(pose_event_step_limited),
                "pose_event_raw_delta_m": pose_event_raw_delta_m,
                "pose_event_raw_delta_rad": pose_event_raw_delta_rad,
                "tracking_reacquire_streak": int(tracking_reacquire_streak),
                "tracking_reacquire_required": int(tracking_reacquire_required),
                "tracking_ready": bool(tracking_ready),
                "tracking_loss_latched": bool(tracking_loss_latched),
                "tracking_direction_checked": bool(tracking_direction_checked),
                "tracking_direction_consistent": bool(tracking_direction_consistent),
                "tracking_direction_rejected": bool(tracking_direction_rejected),
                "tracking_direction_rejected_total": int(tracking_direction_rejected_total),
                "tracking_reference_delta_m": tracking_reference_delta_m,
                "tracking_reference_linear_mps": tracking_reference_linear_mps,
                "tracking_candidate_projection_m": tracking_candidate_projection_m,
                "tracking_backtrack_debt_m": float(tracking_backtrack_debt_m),
                "tracking_direction_reference_source": tracking_direction_reference_source,
                "tracking_direction_backtrack_tolerance_m": tracking_direction_backtrack_tolerance_m,
                "relocalized": bool(relocalized),
                "relocalization_attempted": bool(relocalization_attempted),
                "relocalization_reason": relocalization_reason,
                "loop_closure_detected": bool(loop_closure_detected),
                "loop_closure_applied": bool(loop_closure_applied),
                "loop_closure_delta_m": loop_closure_delta_m,
                "loop_closure_delta_rad": loop_closure_delta_rad,
                "local_map_points": int(local_map_points),
                "local_map_keyframes": int(local_map_keyframes),
            }
            self._consumed = False
            self._last_decision = "accepted"
            self._last_poll_status = "available"
            self._stats["accepted"] += 1
            self._stats["lidar_odometry_measurement_id"] = int(measurement_id)
            self._stats["lidar_odometry_measurement_timestamp"] = float(measurement_timestamp)
            self._stats["measurement_source_matcher_result_id"] = int(matcher_result_id)
            self._stats["measurement_source_raw_scan_id"] = raw_scan_id
            self._stats["measurement_source_raw_scan_timestamp"] = raw_scan_timestamp
            self._stats["promotion_result"] = "accepted"
            self._stats["reject_reason"] = ""
            self._stats["latest_r_scale"] = self._latest["r_scale"]
            self._stats["latest_measurement_confidence"] = float(
                measurement_confidence
            )
            self._stats["latest_integrity_score"] = float(integrity_score)
            self._stats["latest_integrity_state"] = str(integrity_state)
            self._bootstrap_anchor_pose = None
            self._bootstrap_anchor_active = False
            self._stats["bootstrap_anchor_active"] = False

    # ------------------------------------------------------------------
    # Called from control loop (50 Hz)
    # ------------------------------------------------------------------

    def get_odometry(self) -> Optional[Dict[str, Any]]:
        """
        Return the latest accepted LIDAR odometry measurement, or None.

        Returns None if:
        - No scan result has been accepted yet
        - The latest result was already consumed
        - The latest result is too old (stale)
        """
        if not self._lock.acquire(blocking=False):
            self._record_control_lock_miss()
            return None
        try:
            if self._latest is None:
                self._last_poll_status = "missing"
                self._stats["get_odometry_result"] = "missing"
                self._publish_stats_snapshot_locked()
                return None
            age = time.monotonic() - self._latest["timestamp"]
            if age > self._max_scan_age_s:
                self._last_poll_status = "stale"
                self._stats["get_odometry_result"] = "stale"
                self._publish_stats_snapshot_locked()
                return None
            if self._consumed:
                self._last_poll_status = "missing"
                self._stats["get_odometry_result"] = "consumed"
                self._publish_stats_snapshot_locked()
                return None
            self._last_poll_status = "available"
            self._consumed = True
            self._stats["get_odometry_result"] = "returned"
            self._publish_stats_snapshot_locked()
            return dict(self._latest)
        finally:
            self._lock.release()

    def get_stats(self) -> Dict[str, Any]:
        """Diagnostic counters for telemetry."""
        if not self._lock.acquire(blocking=False):
            self._record_control_lock_miss()
            stats = dict(self._stats_snapshot)
            stats["control_lock_miss_count"] = int(self._control_lock_miss_count)
            stats["control_lock_busy"] = True
            stats["delivery_status"] = "lock_busy"
            stats["get_odometry_result"] = "lock_busy"
            return stats
        try:
            stats = dict(self._stats)
            stats["has_latest"] = self._latest is not None
            stats["rejected_jump"] = int(
                max(
                    0,
                    int(_as_finite_or_none(stats.get("rejected_large_jump")) or 0)
                    + int(_as_finite_or_none(stats.get("rejected_bootstrap_jump")) or 0),
                )
            )
            stats["min_confidence"] = self._min_confidence
            stats["max_scan_age_s"] = self._max_scan_age_s
            stats["bootstrap_anchor_max_delta_m"] = float(self._bootstrap_anchor_max_delta_m)
            stats["bootstrap_anchor_max_delta_rad"] = float(self._bootstrap_anchor_max_delta_rad)
            stats["jump_gate_max_speed_mps"] = float(self._jump_gate_max_speed_mps)
            stats["jump_gate_max_yaw_rate_rad_s"] = float(self._jump_gate_max_yaw_rate_rad_s)
            stats["jump_gate_dt_cap_s"] = float(self._jump_gate_dt_cap_s)
            stats["relocalization_jump_multiplier"] = float(self._relocalization_jump_multiplier)
            stats["relocalization_max_delta_m"] = float(self._relocalization_max_delta_m)
            stats["relocalization_max_delta_rad"] = float(self._relocalization_max_delta_rad)
            stats["last_decision"] = self._last_decision
            stats["delivery_status"] = self._last_poll_status
            stats["latest_present"] = self._latest is not None
            stats["r_scale_by_confidence"] = bool(self._r_scale_by_confidence)
            if self._latest is not None:
                stats["latest_age_s"] = time.monotonic() - self._latest["timestamp"]
                stats["latest_confidence"] = self._latest["confidence"]
                stats["latest_r_scale"] = self._latest.get("r_scale")
                stats["latest_measurement_confidence"] = self._latest.get(
                    "measurement_confidence"
                )
                stats["latest_integrity_score"] = self._latest.get("integrity_score")
                stats["latest_integrity_state"] = self._latest.get(
                    "integrity_state", "LEGACY"
                )
            if self._last_candidate is not None:
                candidate_age_s = time.monotonic() - self._last_candidate["timestamp"]
                stats["candidate_available"] = candidate_age_s <= self._max_scan_age_s
                stats["candidate_age_s"] = candidate_age_s
                stats["candidate_confidence"] = self._last_candidate.get("confidence")
                stats["candidate_measurement_confidence"] = self._last_candidate.get(
                    "measurement_confidence"
                )
                stats["candidate_integrity_score"] = self._last_candidate.get(
                    "integrity_score"
                )
                stats["candidate_integrity_state"] = self._last_candidate.get(
                    "integrity_state", "LEGACY"
                )
                stats["candidate_r_scale"] = self._last_candidate.get("r_scale")
            else:
                stats["candidate_available"] = False
                stats["candidate_confidence"] = None
                stats["candidate_measurement_confidence"] = None
                stats["candidate_integrity_score"] = None
                stats["candidate_integrity_state"] = "INCOMPLETE"
                stats["candidate_r_scale"] = None
            stats["bootstrap_anchor_active"] = bool(self._bootstrap_anchor_active)
            stats["bootstrap_anchor_pose"] = _as_pose_dict(self._bootstrap_anchor_pose)
            return self._publish_stats_snapshot_locked(stats)
        finally:
            self._lock.release()

    def publish_control_loop_diagnostics(self, payload: Dict[str, Any]) -> None:
        """Atomically publish the control-loop-owned diagnostic projection."""
        snapshot = {
            key: _copy_control_diagnostic_value(value)
            for key, value in dict(payload or {}).items()
            if key in CONTROL_LOOP_DIAGNOSTIC_KEYS
        }
        if not self._lock.acquire(blocking=False):
            self._record_control_lock_miss()
            return
        try:
            self._stats.update(snapshot)
            self._publish_stats_snapshot_locked()
        finally:
            self._lock.release()

    def reset(self, pose_hint: Optional[Any] = None) -> None:
        """
        Reset state (e.g. after EKF reset or mode switch).

        pose_hint:
            Optional absolute pose anchor. If provided, the first accepted
            post-reset LIDAR pose must stay within bootstrap jump limits.
        """
        with self._lock:
            self._latest = None
            self._last_candidate = None
            # Default to (0,0,0) if no hint provided, to prevent drifting into 
            # wrong symmetry on the first accepted scan after reset.
            hint = _as_pose_dict(pose_hint)
            if hint is None:
                hint = {"x": 0.0, "y": 0.0, "theta": 0.0}
            self._bootstrap_anchor_pose = hint
            self._bootstrap_anchor_active = True
            self._consumed = True
            self._last_decision = "missing"
            self._last_poll_status = "missing"
            self._stats["bootstrap_anchor_active"] = bool(self._bootstrap_anchor_active)
            self._stats["bootstrap_anchor_pose"] = _as_pose_dict(self._bootstrap_anchor_pose)
            self._stats["bootstrap_anchor_delta_m"] = None
            self._stats["bootstrap_anchor_delta_rad"] = None
            self._stats["raw_scan_id"] = None
            self._stats["raw_scan_timestamp"] = None
            self._stats["matcher_result_id"] = None
            self._stats["candidate_id"] = None
            self._stats["matcher_result_timestamp"] = None
            self._stats["candidate_source_raw_scan_id"] = None
            self._stats["candidate_source_raw_scan_timestamp"] = None
            self._stats["candidate_measurement_confidence"] = None
            self._stats["candidate_integrity_score"] = None
            self._stats["candidate_integrity_state"] = "INCOMPLETE"
            self._stats["lidar_odometry_measurement_id"] = None
            self._stats["lidar_odometry_measurement_timestamp"] = None
            self._stats["measurement_source_matcher_result_id"] = None
            self._stats["measurement_source_raw_scan_id"] = None
            self._stats["measurement_source_raw_scan_timestamp"] = None
            self._stats["latest_measurement_confidence"] = None
            self._stats["latest_integrity_score"] = None
            self._stats["latest_integrity_state"] = "INCOMPLETE"
            self._stats["matcher_mode"] = ""
            self._stats["localization_status"] = ""
            self._stats["pose_update_event"] = ""
            self._stats["pose_event_step_limited"] = False
            self._stats["pose_event_raw_delta_m"] = None
            self._stats["pose_event_raw_delta_rad"] = None
            self._stats["tracking_reacquire_streak"] = 0
            self._stats["tracking_reacquire_required"] = 0
            self._stats["tracking_ready"] = False
            self._stats["tracking_loss_latched"] = False
            self._stats["tracking_direction_checked"] = False
            self._stats["tracking_direction_consistent"] = True
            self._stats["tracking_direction_rejected"] = False
            self._stats["tracking_direction_rejected_total"] = 0
            self._stats["tracking_reference_delta_m"] = None
            self._stats["tracking_reference_linear_mps"] = None
            self._stats["tracking_candidate_projection_m"] = None
            self._stats["tracking_backtrack_debt_m"] = 0.0
            self._stats["tracking_direction_reference_source"] = ""
            self._stats["tracking_direction_backtrack_tolerance_m"] = None
            self._stats["tracking_candidate_pose_ref"] = None
            self._stats["last_lidar_pose_ref"] = None
            self._stats["relocalized"] = False
            self._stats["relocalization_attempted"] = False
            self._stats["relocalization_reason"] = ""
            self._stats["loop_closure_detected"] = False
            self._stats["loop_closure_applied"] = False
            self._stats["loop_closure_delta_m"] = None
            self._stats["loop_closure_delta_rad"] = None
            self._stats["local_map_points"] = 0
            self._stats["local_map_keyframes"] = 0
            self._stats["driver_connected"] = None
            self._stats["driver_last_data_age_s"] = None
            self._stats["driver_rx_buffer_len"] = None
            self._stats["driver_invalid_packet_count"] = None
            self._stats["driver_reconnect_count"] = None
            self._stats["jump_gate_dynamic_max_m"] = None
            self._stats["jump_gate_dynamic_max_rad"] = None
            self._stats["jump_gate_dt_s"] = None
            self._stats["jump_gate_relocalized"] = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def min_confidence(self) -> float:
        return self._min_confidence

    def _measurement_r_scale(self, confidence: float) -> float:
        """
        Confidence-aware LIDAR R scale.
        - 1.0 -> default EKF R_lidar
        - <1.0 -> higher trust in the accepted LIDAR pose
        """
        if not self._r_scale_by_confidence:
            return 1.0
        try:
            conf = float(confidence)
        except (TypeError, ValueError):
            return 1.0
        if not math.isfinite(conf):
            return 1.0
        min_conf = max(1e-6, float(self._min_confidence))
        conf_eff = max(min_conf, conf)
        scale = min_conf / conf_eff
        return float(min(1.0, max(0.35, scale)))


def _normalize_angle(a: float) -> float:
    """Normalize angle to [-pi, pi)."""
    while a >= math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a
