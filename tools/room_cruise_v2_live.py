#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Live gate for layered Room Cruise v2."""

from __future__ import annotations

import argparse
import collections
import json
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import ensure_agent_system_prompt_loaded  # noqa: E402
from tools.lidar_1m_step import _send_command_checked  # noqa: E402
from tools.runtime_status_client import get_runtime_status_client  # noqa: E402

RUNTIME_DIR = PROJECT_ROOT / "runtime"
AGENT_TESTS_DIR = test_artifacts_dir()
STATUS_PATH = RUNTIME_DIR / "status.json"
LATEST_RESULT = AGENT_TESTS_DIR / "latest_room_cruise_v2_result.json"
LATEST_SUMMARY = AGENT_TESTS_DIR / "latest_room_cruise_v2_summary.json"
TOKEN = "GUI_DEFAULT"
POST_STOP_IDLE_SAMPLES = 12
POST_STOP_TIMEOUT_S = 8.0

_STATUS_CLIENT = get_runtime_status_client()


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return float(default)
        out = float(value)
    except Exception:
        return float(default)
    return float(out) if math.isfinite(float(out)) else float(default)


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")
    tmp.replace(path)


def _read_status(*, force: bool = False) -> Dict[str, Any]:
    return _STATUS_CLIENT.read_json(STATUS_PATH, force=force)


def _pose(status: Dict[str, Any]) -> Dict[str, float]:
    src = dict((status or {}).get("pose") or {})
    return {
        "x": _safe_float(src.get("x"), 0.0),
        "y": _safe_float(src.get("y"), 0.0),
        "theta": _safe_float(src.get("theta"), 0.0),
        "theta_deg": _safe_float(src.get("theta_deg"), 0.0),
    }


def _front(status: Dict[str, Any]) -> Optional[float]:
    lidar = dict((status or {}).get("lidar") or {})
    for key in ("front_clearance_m", "min_dist_narrow", "min_dist"):
        value = _safe_float(lidar.get(key), math.nan)
        if math.isfinite(value) and value > 0.0:
            return float(value)
    return None


def _imu_gyro_z_rad_s(imu: Dict[str, Any]) -> float:
    gyro = imu.get("gyro")
    if isinstance(gyro, (list, tuple)) and len(gyro) >= 3:
        return math.radians(_safe_float(gyro[2], math.nan))
    return math.nan


def _signed_angle_delta_deg(current: float, previous: float) -> float:
    return ((float(current) - float(previous) + 180.0) % 360.0) - 180.0


def _relative_heading_change_metrics(samples: list[Dict[str, Any]]) -> Dict[str, Any]:
    """Compare relative yaw changes in the boot robot frame.

    Absolute IMU yaw and boot-map headings do not share an origin, so
    endpoint values are never compared directly. Pose/LIDAR headings are
    unwrapped between samples and IMU yaw is integrated from gyro-z.
    """
    rows: list[Dict[str, Any]] = []
    seen_versions: set[str] = set()
    for sample in samples:
        version = sample.get("status_version")
        if version is not None:
            version_key = str(version)
            if version_key in seen_versions:
                continue
            seen_versions.add(version_key)
        rows.append(sample)

    def measurement_time(row: Dict[str, Any], key: str) -> float:
        return _safe_float(
            row.get(key, row.get("status_time_s", row.get("ts"))),
            math.nan,
        )

    def heading_delta(field_getter, *, time_key: str, sequence_key: str = "") -> float | None:
        total = 0.0
        used = 0
        previous_value = None
        previous_row = None
        previous_sequence = None
        for row in rows:
            value = field_getter(row)
            if not math.isfinite(_safe_float(value, math.nan)):
                continue
            sequence = row.get(sequence_key) if sequence_key else None
            if sequence_key and sequence is not None and sequence == previous_sequence:
                continue
            current = float(value)
            dt_s = (
                measurement_time(row, time_key) - measurement_time(previous_row, time_key)
                if previous_row is not None
                else math.nan
            )
            if previous_value is not None and math.isfinite(dt_s) and 0.0 < dt_s <= 1.0:
                total += _signed_angle_delta_deg(current, previous_value)
                used += 1
            previous_value = current
            previous_row = row
            previous_sequence = sequence
        return float(total) if used else None

    def gyro_delta() -> float | None:
        total_rad = 0.0
        used = 0
        for previous, current in zip(rows, rows[1:]):
            dt_s = measurement_time(current, "status_time_s") - measurement_time(previous, "status_time_s")
            prev_rate = _safe_float(previous.get("imu_gyro_z_rad_s"), math.nan)
            current_rate = _safe_float(current.get("imu_gyro_z_rad_s"), math.nan)
            if not (
                math.isfinite(dt_s)
                and 0.0 < dt_s <= 1.0
                and math.isfinite(prev_rate)
                and math.isfinite(current_rate)
            ):
                continue
            total_rad += 0.5 * (prev_rate + current_rate) * dt_s
            used += 1
        return math.degrees(total_rad) if used else None

    imu_heading_change = heading_delta(
        lambda row: row.get("imu_heading_deg"),
        time_key="imu_measurement_time_s",
        sequence_key="imu_measurement_time_s",
    )
    changes = {
        "ekf_pose": heading_delta(
            lambda row: (row.get("pose") or {}).get("theta_deg"),
            time_key="status_time_s",
        ),
        "imu_gyro": gyro_delta(),
        "lidar": heading_delta(
            lambda row: row.get("lidar_heading_deg"),
            time_key="lidar_pose_time_s",
            sequence_key="lidar_scan_seq",
        ),
    }
    changes = {
        name: float(value)
        for name, value in changes.items()
        if value is not None and math.isfinite(float(value))
    }
    disagreements = [
        abs(left_value - right_value)
        for left_index, left_value in enumerate(changes.values())
        for right_value in list(changes.values())[left_index + 1 :]
    ]
    auxiliary_changes = {}
    if imu_heading_change is not None and math.isfinite(
        float(imu_heading_change)
    ):
        auxiliary_changes["imu_heading"] = float(imu_heading_change)
    return {
        "mode": "relative_boot_frame_heading_change",
        "changes_deg": changes,
        "auxiliary_changes_deg": auxiliary_changes,
        "max_pair_disagreement_deg": max(disagreements) if disagreements else None,
        "deduplicated_sample_count": len(rows),
    }


def _resolved(status: Dict[str, Any]) -> Dict[str, Any]:
    return dict((dict((status or {}).get("motion_resolution") or {})).get("resolved") or {})


def _sample(status: Dict[str, Any]) -> Dict[str, Any]:
    resolved = _resolved(status)
    details = dict(resolved.get("details") or {})
    speed_profile = dict(details.get("speed_profile") or {})
    clearance = dict(details.get("clearance") or {})
    obstacle_avoidance = dict(
        details.get("obstacle_avoidance")
        or clearance.get("obstacle_avoidance")
        or {}
    )
    wall_clearance = dict(obstacle_avoidance.get("wall_clearance") or {})
    room_v2 = dict((status or {}).get("room_cruise_v2") or {})
    m5_full_stack = dict(room_v2.get("m5_full_stack") or {})
    m5_goal = dict(m5_full_stack.get("goal") or {})
    m5_path_primitive = dict(m5_full_stack.get("path_primitive") or {})
    m5_route_selector = dict(m5_full_stack.get("route_selector") or {})
    local_path_segment = dict(details.get("local_path_segment") or {})
    local_nav = dict((status or {}).get("local_navigation") or room_v2.get("local_navigation") or {})
    rolling = dict((status or {}).get("rolling_local_map") or {})
    lidar = dict((status or {}).get("lidar") or {})
    stop_status = dict((status or {}).get("stop_status") or {})
    motion_command = dict((status or {}).get("motion_command") or {})
    motion_semantics = dict((status or {}).get("motion_semantics") or {})
    control_monitor = dict((status or {}).get("control_monitor") or {})
    command_arbitration = dict(motion_command.get("command_arbitration") or {})
    primitive_contract = dict(motion_command.get("primitive_contract") or (status or {}).get("primitive_contract") or {})
    requested = dict(motion_command.get("requested_motion_intent") or {})
    limited = dict(motion_command.get("limited_motion_intent") or {})
    requested_track = dict(motion_command.get("requested_track_reference") or {})
    track_targets = dict(motion_command.get("track_targets") or {})
    motion_public = dict((status or {}).get("motion_public") or {})
    actual_measurement_gate = dict(motion_public.get("actual_measurement_gate") or {})
    encoder = dict((status or {}).get("encoder") or {})
    encoder_service = dict(encoder.get("service") or {})
    encoder_canonical = dict(encoder.get("canonical") or {})
    canonical_velocity = dict(encoder_canonical.get("canonical_velocity") or {})
    canonical_pulses = dict(encoder_canonical.get("pulses_delta") or {})
    encoder_computed = dict(encoder.get("computed") or {})
    pwm = dict((status or {}).get("pwm") or {})
    localization = dict((status or {}).get("localization_gate") or {})
    localization_truth = dict((status or {}).get("localization_truth") or {})
    imu = dict((status or {}).get("imu") or {})
    lidar_odom = dict((status or {}).get("lidar_odom_status") or {})
    lidar_pose = dict(lidar_odom.get("last_lidar_pose") or {})
    watchdog = dict((status or {}).get("watchdog") or {})
    loop_budget = dict((status or {}).get("loop_budget") or {})
    motion_quality = dict((status or {}).get("motion_quality") or {})
    estimator_consistency = dict(motion_quality.get("estimator_consistency") or {})
    safety = dict((status or {}).get("safety") or {})
    last_emergency = dict((status or {}).get("last_emergency") or {})
    final_after_shaping = dict(resolved.get("final_after_shaping") or {})
    peripherals = dict((status or {}).get("peripherals") or {})
    motion_state = dict((status or {}).get("motion_state") or {})
    logger_status = dict((status or {}).get("logger") or {})
    slow_tick = dict((status or {}).get("slow_tick_diagnostics") or {})
    encoder_reliability = dict((status or {}).get("encoder_reliability") or {})
    rolling_has_data = bool(
        rolling.get("has_data", False)
        or lidar.get("rolling_local_map_has_data", False)
    )
    rolling_valid_points = int(
        rolling.get(
            "valid_points",
            lidar.get("rolling_local_map_valid_points", 0),
        )
        or 0
    )
    resolved_v = _safe_float(final_after_shaping.get("v_target", resolved.get("v_target")), 0.0)
    resolved_omega = _safe_float(final_after_shaping.get("omega_target", resolved.get("omega_target")), 0.0)
    status_time_s = _safe_float((status or {}).get("time"), math.nan)
    lidar_latest_age_s = _safe_float(lidar_odom.get("latest_age_s"), math.nan)
    lidar_pose_time_s = (
        float(status_time_s) - float(lidar_latest_age_s)
        if math.isfinite(status_time_s) and math.isfinite(lidar_latest_age_s)
        else math.nan
    )
    return {
        "ts": time.time(),
        "status_time_s": status_time_s,
        "status_version": int((status or {}).get("status_version", 0) or 0),
        "state": str((status or {}).get("state", "") or ""),
        "control_mode": str((status or {}).get("control_mode", "") or ""),
        "motion_state_mode": str(motion_state.get("mode", "") or ""),
        "motion_state_mode_raw": str(motion_state.get("mode_raw", "") or ""),
        "active_v_max_mps": _safe_float(motion_state.get("v_max_active"), math.nan),
        "camera_enabled": bool((status or {}).get("camera_enabled", peripherals.get("camera", False))),
        "peripherals": peripherals,
        "pose": _pose(status),
        "front_m": _front(status),
        "lidar_raw_safety_source": str(
            lidar.get("raw_safety_source", "") or ""
        ),
        "lidar_raw_safety_scan_id": int(
            lidar.get("raw_safety_raw_scan_id", 0) or 0
        ),
        "lidar_raw_safety_scan_timestamp": _safe_float(
            lidar.get("raw_safety_raw_scan_timestamp"),
            math.nan,
        ),
        "lidar_raw_safety_min_dist_point": dict(
            lidar.get("raw_safety_min_dist_point") or {}
        ),
        "lidar_raw_safety_min_dist_narrow_point": dict(
            lidar.get("raw_safety_min_dist_narrow_point") or {}
        ),
        "min_clearance_m": _safe_float(rolling.get("min_dist_m", lidar.get("min_dist")), math.nan),
        "left_clearance_m": _safe_float(rolling.get("left_clearance_m"), math.nan),
        "right_clearance_m": _safe_float(rolling.get("right_clearance_m"), math.nan),
        "rear_clearance_m": _safe_float(rolling.get("rear_clearance_m"), math.nan),
        "blocked_front": bool(rolling.get("blocked_front", False)),
        "blocked_back": bool(rolling.get("blocked_back", False)),
        "stop_type": str(stop_status.get("type", "") or ""),
        "stop_reason": str(stop_status.get("canonical_reason", stop_status.get("reason", "")) or ""),
        "safety_allow": bool((status or {}).get("safety_allow", False)),
        "safety_reason": str(safety.get("reason", "") or ""),
        "safety_action": str(safety.get("action", "") or ""),
        "last_emergency_reason": str(last_emergency.get("reason", "") or ""),
        "lidar_health": str((status or {}).get("lidar_health", "") or ""),
        "localization_mode": str(localization.get("mode", "") or ""),
        "localization_trust": _safe_float(localization.get("trust"), 0.0),
        "localization_allow_motion": bool(localization.get("allow_motion", False)),
        "localization_truth_state": str(localization_truth.get("state", "") or ""),
        "localization_truth_trust": _safe_float(localization_truth.get("trust"), 0.0),
        "localization_truth_allow_motion": bool(localization_truth.get("allow_motion", False)),
        "localization_truth_consistent": bool(localization_truth.get("consistent", True)),
        "active_motion_layer": str(motion_command.get("active_layer", "") or ""),
        "active_motion_type": str(motion_command.get("command_type", "") or ""),
        "service_motion_active": bool((status or {}).get("service_motion_active", False)),
        "room_cruise_v2_active": bool(room_v2.get("active", False)),
        "room_cruise_v2_reason": str(room_v2.get("reason", "") or ""),
        "room_cruise_v2_mode": str(room_v2.get("mode", "") or ""),
        "m5_full_stack_active": bool(m5_full_stack),
        "m5_goal_id": str(m5_goal.get("id", "") or ""),
        "m5_goal_event": str(m5_full_stack.get("goal_event", "") or ""),
        "m5_goal_lifecycle": str(m5_goal.get("lifecycle", "") or ""),
        "m5_completed_waypoints": int(m5_full_stack.get("completed_waypoints", 0) or 0),
        "m5_replan_count": int(m5_full_stack.get("replan_count", 0) or 0),
        "m5_visited_cell_count": int(m5_full_stack.get("visited_cell_count", 0) or 0),
        "m5_route_selector_provider": str(m5_route_selector.get("provider", "") or ""),
        "m5_selected_kappa": _safe_float(m5_route_selector.get("selected_kappa"), math.nan),
        "m5_local_path_segment_active": bool(
            local_path_segment or m5_path_primitive.get("active", False)
        ),
        "m5_local_path_segment_id": str(
            local_path_segment.get("id")
            or m5_path_primitive.get("id")
            or ""
        ),
        "local_navigation_reason": str(local_nav.get("reason", "") or ""),
        "local_navigation_mode": str(local_nav.get("mode", "") or ""),
        "local_navigation_active": bool(
            local_nav.get("active", False)
            or (
                str(resolved.get("command_type", "") or "") == "local_planner_segment"
                and str(resolved.get("layer", "") or "") == "LOCAL_NAVIGATION"
            )
        ),
        "rolling_local_map_has_data": bool(rolling_has_data),
        "rolling_local_map_valid_points": int(rolling_valid_points),
        "local_planner_phase": str(speed_profile.get("phase", "") or ""),
        "local_planner_motion_style": str(details.get("motion_style", "") or ""),
        "obstacle_avoidance_active": bool(
            obstacle_avoidance.get("active", False)
        ),
        "obstacle_avoidance_reason": str(
            obstacle_avoidance.get("reason", "") or ""
        ),
        "obstacle_avoidance_mode": str(
            obstacle_avoidance.get("mode", "") or ""
        ),
        "obstacle_avoidance_side": str(
            obstacle_avoidance.get("side", "") or ""
        ),
        "obstacle_avoidance_side_selection": str(
            obstacle_avoidance.get("side_selection", "") or ""
        ),
        "obstacle_avoidance_turn_sign": _safe_float(
            obstacle_avoidance.get("turn_sign"),
            math.nan,
        ),
        "obstacle_front_clearance_m": _safe_float(
            obstacle_avoidance.get("front_clearance_m"),
            math.nan,
        ),
        "obstacle_left_clearance_m": _safe_float(
            obstacle_avoidance.get("left_clearance_m"),
            math.nan,
        ),
        "obstacle_right_clearance_m": _safe_float(
            obstacle_avoidance.get("right_clearance_m"),
            math.nan,
        ),
        "obstacle_wall_active": bool(wall_clearance.get("active", False)),
        "obstacle_wall_side": str(wall_clearance.get("wall_side", "") or ""),
        "obstacle_wall_escape_side": str(
            wall_clearance.get("escape_side", "") or ""
        ),
        "resolved_name": str(resolved.get("name", "") or ""),
        "resolved_layer": str(resolved.get("layer", "") or ""),
        "resolved_command_type": str(resolved.get("command_type", "") or ""),
        "resolved_source": str(resolved.get("source", "") or ""),
        "resolved_execution_mode": str(resolved.get("execution_mode", "") or ""),
        "motion_execution_mode": str(motion_command.get("execution_mode", "") or ""),
        "resolved_v": float(resolved_v),
        "resolved_omega": float(resolved_omega),
        "requested_v": _safe_float(requested.get("v"), 0.0),
        "requested_omega": _safe_float(requested.get("omega"), 0.0),
        "limited_v": _safe_float(limited.get("v"), 0.0),
        "limited_omega": _safe_float(limited.get("omega"), 0.0),
        "requested_track_reference_source": str(motion_command.get("requested_track_reference_source", "") or ""),
        "requested_track_left_mps": _safe_float(requested_track.get("left_mps"), math.nan),
        "requested_track_right_mps": _safe_float(requested_track.get("right_mps"), math.nan),
        "control_track_reference_mode": str(control_monitor.get("track_reference_mode", "") or ""),
        "control_output_reason": str(control_monitor.get("output_reason", "") or ""),
        "control_wheel_loop_enabled": bool(control_monitor.get("wheel_loop_enabled", False)),
        "control_wheel_loop_feedback_source": str(
            control_monitor.get("wheel_loop_feedback_source", "") or ""
        ),
        "control_wheel_loop_effective_kp": _safe_float(
            control_monitor.get("wheel_loop_effective_kp"),
            math.nan,
        ),
        "guidance_heading_hold_active": bool(motion_semantics.get("heading_hold_applied", False)),
        "local_nav_pivot_track_required": bool(
            control_monitor.get("local_navigation_pivot_track_required", False)
            or control_monitor.get("m3_pivot_track_required", False)
        ),
        "actual_v": _safe_float(motion_public.get("actual_linear_mps"), 0.0),
        "actual_omega": math.radians(_safe_float(motion_public.get("actual_angular_dps"), 0.0)),
        "motion_segment_age_s": _safe_float(
            motion_public.get(
                "segment_age_s",
                (motion_public.get("segment") or {}).get("duration_s"),
            ),
            math.nan,
        ),
        "ekf_v_mps": _safe_float(
            motion_public.get("ekf_linear_mps", motion_public.get("actual_linear_mps")),
            math.nan,
        ),
        "pose_v_mps": _safe_float(motion_public.get("pose_linear_mps"), math.nan),
        "commanded_v_mps": _safe_float(motion_public.get("cmd_linear_mps"), math.nan),
        "commanded_omega_rad_s": math.radians(
            _safe_float(motion_public.get("cmd_angular_dps"), math.nan)
        ),
        "actual_measurement_gate_reasons": list(actual_measurement_gate.get("reasons") or []),
        "actual_primitive_corroboration": dict(
            motion_public.get("actual_primitive_corroboration") or {}
        ),
        "actual_primitive_measurement_available": bool(
            motion_command.get(
                "actual_primitive_measurement_available",
                motion_public.get("actual_measurement_available", False),
            )
        ),
        "actual_primitive_measurement_ready": bool(
            motion_command.get(
                "actual_primitive_measurement_ready",
                motion_public.get("actual_measurement_ready", False),
            )
        ),
        "actual_primitive_measurement_reliable": bool(
            motion_command.get(
                "actual_primitive_measurement_reliable",
                motion_public.get("actual_measurement_reliable", False),
            )
        ),
        "turn_primitive_actual_raw": str(
            motion_command.get(
                "turn_primitive_actual_raw",
                motion_public.get("turn_primitive_actual_raw", ""),
            )
            or ""
        ),
        "motion_actual_ssot": str(motion_public.get("source", "") or ""),
        "target_left_mps": _safe_float(track_targets.get("left_mps"), 0.0),
        "target_right_mps": _safe_float(track_targets.get("right_mps"), 0.0),
        "actual_left_mps": _safe_float(canonical_velocity.get("left_mps"), 0.0),
        "actual_right_mps": _safe_float(canonical_velocity.get("right_mps"), 0.0),
        "encoder_snapshot_time_s": _safe_float(encoder_service.get("snapshot_ts_perf"), math.nan),
        "encoder_snapshot_age_s": _safe_float(encoder_canonical.get("snapshot_age_s"), math.nan),
        "encoder_snapshot_stale": bool(encoder_canonical.get("snapshot_stale", True)),
        "encoder_window_dt_s": _safe_float(
            canonical_pulses.get("dt_aggregation_window_s"), math.nan
        ),
        "encoder_window_start_ts": _safe_float(canonical_pulses.get("window_start_ts"), math.nan),
        "encoder_window_end_ts": _safe_float(canonical_pulses.get("window_end_ts"), math.nan),
        "encoder_left_count_start": canonical_pulses.get("left_count_start"),
        "encoder_left_count_end": canonical_pulses.get("left_count_end"),
        "encoder_right_count_start": canonical_pulses.get("right_count_start"),
        "encoder_right_count_end": canonical_pulses.get("right_count_end"),
        "encoder_left_pulses_delta": canonical_pulses.get("left"),
        "encoder_right_pulses_delta": canonical_pulses.get("right"),
        "encoder_left_step_m": _safe_float(
            encoder_computed.get("step_distance_left_m"), math.nan
        ),
        "encoder_right_step_m": _safe_float(
            encoder_computed.get("step_distance_right_m"), math.nan
        ),
        "pwm_left": _safe_float(pwm.get("left"), 0.0),
        "pwm_right": _safe_float(pwm.get("right"), 0.0),
        "imu_heading_deg": _safe_float(imu.get("heading_deg"), math.nan),
        "imu_gyro_z_rad_s": _imu_gyro_z_rad_s(imu),
        "imu_measurement_time_s": _safe_float(
            imu.get("measurement_timestamp"),
            math.nan,
        ),
        "lidar_heading_deg": math.degrees(_safe_float(lidar_pose.get("theta"), math.nan)),
        "lidar_scan_seq": int(lidar_odom.get("scan_seq", 0) or 0),
        "lidar_pose_time_s": lidar_pose_time_s,
        "watchdog_freq_hz": _safe_float(watchdog.get("freq_hz"), math.nan),
        "watchdog_period_s": _safe_float(watchdog.get("period_sec"), math.nan),
        "watchdog_stop_triggered": bool(watchdog.get("stop_triggered", False)),
        "watchdog_warn_count": int(watchdog.get("warn_count", 0) or 0),
        "loop_budget_total_ema_ms": _safe_float(loop_budget.get("total_ema_ms"), math.nan),
        "loop_budget": loop_budget,
        "slow_tick_count": int(slow_tick.get("slow_tick_count", 0) or 0),
        "slow_observed_tick_count": int(slow_tick.get("observed_tick_count", 0) or 0),
        "slow_lidar_spike_count": int(slow_tick.get("slow_lidar_spike_count", 0) or 0),
        "slow_resolver_spike_count": int(slow_tick.get("slow_resolver_spike_count", 0) or 0),
        "slow_io_event_count": int(slow_tick.get("slow_io_event_count", 0) or 0),
        "slow_gc_count": int(slow_tick.get("slow_gc_count", 0) or 0),
        "slow_scheduler_delay_count": int(slow_tick.get("slow_scheduler_delay_count", 0) or 0),
        "slow_unattributed_spike_count": int(slow_tick.get("slow_unattributed_spike_count", 0) or 0),
        "slow_none_count": int(slow_tick.get("slow_none_count", 0) or 0),
        "slow_multi_label_count": int(slow_tick.get("slow_multi_label_count", 0) or 0),
        "slow_counter_semantics": dict(slow_tick.get("counter_semantics") or {}),
        "slow_coobserved_category_counts": dict(slow_tick.get("coobserved_category_counts") or {}),
        "slow_category_combination_counts": dict(slow_tick.get("category_combination_counts") or {}),
        "slow_primary_timing_class_counts": dict(slow_tick.get("primary_timing_class_counts") or {}),
        "slow_dominant_processing_phase_counts": dict(
            slow_tick.get("dominant_processing_phase_counts") or {}
        ),
        "slow_phase_spike_counts": dict(slow_tick.get("phase_spike_counts") or {}),
        "slow_phase_max_us": dict(slow_tick.get("phase_max_us") or {}),
        "slow_phase_gc_pause_max_us": dict(slow_tick.get("phase_gc_pause_max_us") or {}),
        "slow_last_record": dict(slow_tick.get("last_record") or {}),
        "slow_max_tick_total_us": int(slow_tick.get("max_tick_total_us", 0) or 0),
        "slow_max_gc_pause_us": int(slow_tick.get("max_gc_pause_us", 0) or 0),
        "slow_max_scheduler_delay_us": int(slow_tick.get("max_scheduler_delay_us", 0) or 0),
        "slow_max_unattributed_processing_us": int(
            slow_tick.get("max_unattributed_processing_us", 0) or 0
        ),
        "slow_min_phase_coverage_ratio": _safe_float(
            slow_tick.get("min_phase_coverage_ratio"),
            math.nan,
        ),
        "logger_queue_depth": int(logger_status.get("queue_depth", 0) or 0),
        "logger_dropped_messages": int(logger_status.get("dropped_messages", 0) or 0),
        "logger_write_errors": int(logger_status.get("write_errors", 0) or 0),
        "logger_flush_duration_ms": _safe_float(logger_status.get("last_flush_duration_ms"), math.nan),
        "logger_max_flush_duration_ms": _safe_float(logger_status.get("max_flush_duration_ms"), math.nan),
        "lidar_pose_confidence": _safe_float(lidar.get("lidar_pose_confidence"), math.nan),
        "lidar_candidate_confidence": _safe_float(lidar_odom.get("candidate_confidence"), math.nan),
        "lidar_candidate_measurement_confidence": _safe_float(
            lidar_odom.get("candidate_measurement_confidence"),
            math.nan,
        ),
        "lidar_latest_measurement_confidence": _safe_float(
            lidar_odom.get("latest_measurement_confidence"),
            math.nan,
        ),
        "lidar_matcher_result_id": int(
            lidar_odom.get("matcher_result_id", 0) or 0
        ),
        "lidar_candidate_id": int(lidar_odom.get("candidate_id", 0) or 0),
        "lidar_measurement_id": int(
            lidar_odom.get("lidar_odometry_measurement_id", 0) or 0
        ),
        "lidar_candidate_age_s": _safe_float(
            lidar_odom.get("candidate_age_s"),
            math.nan,
        ),
        "lidar_latest_age_s": _safe_float(
            lidar_odom.get("latest_age_s"),
            math.nan,
        ),
        "lidar_matcher_mode": str(lidar_odom.get("matcher_mode", "") or ""),
        "lidar_localization_status": str(
            lidar_odom.get("localization_status", "") or ""
        ),
        "lidar_matcher_degenerate": bool(
            lidar_odom.get("matcher_degenerate", False)
        ),
        "lidar_matcher_degeneracy_reasons": list(
            lidar_odom.get("matcher_degeneracy_reasons") or []
        ),
        "lidar_matcher_quality": dict(lidar_odom.get("matcher_quality") or {}),
        "lidar_local_map_points": int(lidar_odom.get("local_map_points", 0) or 0),
        "lidar_local_map_keyframes": int(
            lidar_odom.get("local_map_keyframes", 0) or 0
        ),
        "lidar_tracking_ready": bool(lidar_odom.get("tracking_ready", False)),
        "lidar_tracking_loss_latched": bool(
            lidar_odom.get("tracking_loss_latched", False)
        ),
        "lidar_raw_scan_rate_hz": _safe_float(lidar_odom.get("raw_scan_rate_hz"), math.nan),
        "lidar_matcher_latency_ms": _safe_float(lidar_odom.get("matcher_latency_ms"), math.nan),
        "lidar_matcher_queue_delay_ms": _safe_float(
            lidar_odom.get("matcher_queue_delay_ms"),
            math.nan,
        ),
        "lidar_matcher_runtime_ms": _safe_float(
            lidar_odom.get("matcher_runtime_ms"),
            math.nan,
        ),
        "lidar_ekf_applied_gap_s": _safe_float(lidar_odom.get("ekf_applied_gap_s"), math.nan),
        "lidar_ekf_nis": _safe_float(lidar_odom.get("nis"), math.nan),
        "lidar_odom_status": str(lidar_odom.get("status", "") or ""),
        "lidar_odom_delivery_status": str(lidar_odom.get("delivery_status", "") or ""),
        "encoder_reliability_health": str(encoder_reliability.get("snapshot_health", "") or ""),
        "encoder_reliability_trust": _safe_float(encoder_reliability.get("combined_trust"), math.nan),
        "encoder_anomaly_active": bool(encoder_reliability.get("anomaly_active", False)),
        "encoder_direction_switch_recent": bool(encoder_reliability.get("direction_switch_recent", False)),
        "encoder_symmetry_fault_active": bool(encoder_reliability.get("symmetry_fault_active", False)),
        "encoder_symmetry_fault_acc_s": _safe_float(encoder_reliability.get("symmetry_fault_acc_s"), 0.0),
        "imu_health": str(imu.get("health", "") or ""),
        "turn_primitive_requested": str(motion_command.get("turn_primitive_requested", "") or ""),
        "turn_primitive_limited": str(motion_command.get("turn_primitive_limited", "") or ""),
        "turn_primitive_executed": str(motion_command.get("turn_primitive_executed", "") or ""),
        "turn_primitive_actual": str(motion_command.get("turn_primitive_actual", "") or ""),
        "primitive_contract_violation": bool(primitive_contract.get("violation", False)),
        "control_execution_contract_violation": bool(
            ((status or {}).get("control_monitor") or {}).get("execution_mode_contract_violation", False)
        ),
        "command_owner_conflict": bool(command_arbitration.get("conflict", False)),
        "active_route_count": int(command_arbitration.get("active_route_count", 0) or 0),
        "resolved_route": str(command_arbitration.get("resolved_route", "") or ""),
        "motion_quality_state": str(motion_quality.get("quality_state", "") or ""),
        "motion_quality_estimator_confidence": _safe_float(estimator_consistency.get("confidence"), math.nan),
        "motion_quality_innovation_theta_abs": _safe_float(estimator_consistency.get("innovation_theta_abs"), math.nan),
        "motion_quality_innovation_v_abs": _safe_float(estimator_consistency.get("innovation_v_abs"), math.nan),
        "amr_lidar_bad_observation_count": int(
            (safety.get("amr_lidar_guard") or {}).get("bad_observation_count", 0)
            or 0
        ),
        "amr_lidar_last_quality_reason": str(
            (safety.get("amr_lidar_guard") or {}).get("last_quality_reason", "")
            or ""
        ),
        "resolved_has_room_cruise_v2_details": bool(
            details.get("room_cruise_v2")
            or str(resolved.get("name", "") or "") == "room_cruise_v2_local_navigation"
        ),
        "track_width_m": _safe_float(speed_profile.get("track_width_m"), math.nan),
    }


def _motion_audit_trace(samples: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Persist a bounded, time-aligned M5/localization/motion evidence row."""
    rows: List[Dict[str, Any]] = []
    previous: Optional[Dict[str, Any]] = None
    for index, sample in enumerate(samples):
        quality = dict(sample.get("lidar_matcher_quality") or {})
        ts = _safe_float(sample.get("ts"), math.nan)
        previous_ts = _safe_float((previous or {}).get("ts"), math.nan)
        dt_s = (
            max(0.0, float(ts) - float(previous_ts))
            if math.isfinite(ts) and math.isfinite(previous_ts)
            else 0.0
        )
        resolved_v = _safe_float(sample.get("resolved_v"), 0.0)
        resolved_omega = _safe_float(sample.get("resolved_omega"), 0.0)
        previous_v = _safe_float((previous or {}).get("resolved_v"), resolved_v)
        previous_omega = _safe_float(
            (previous or {}).get("resolved_omega"),
            resolved_omega,
        )
        pwm_left = _safe_float(sample.get("pwm_left"), 0.0)
        pwm_right = _safe_float(sample.get("pwm_right"), 0.0)
        previous_pwm_left = _safe_float((previous or {}).get("pwm_left"), pwm_left)
        previous_pwm_right = _safe_float((previous or {}).get("pwm_right"), pwm_right)
        rows.append(
            {
                "index": int(index),
                "ts": None if not math.isfinite(ts) else round(float(ts), 6),
                "dt_s": round(float(dt_s), 6),
                "state": str(sample.get("state", "") or ""),
                "pose": dict(sample.get("pose") or {}),
                "goal_id": str(sample.get("m5_goal_id", "") or ""),
                "goal_event": str(sample.get("m5_goal_event", "") or ""),
                "replan_count": int(sample.get("m5_replan_count", 0) or 0),
                "local_planner_phase": str(
                    sample.get("local_planner_phase", "") or ""
                ),
                "obstacle_mode": str(
                    sample.get("obstacle_avoidance_mode", "") or ""
                ),
                "resolved_execution_mode": str(
                    sample.get("resolved_execution_mode", "") or ""
                ),
                "resolved_v": round(float(resolved_v), 6),
                "resolved_omega": round(float(resolved_omega), 6),
                "requested_v": round(
                    _safe_float(sample.get("requested_v"), 0.0),
                    6,
                ),
                "requested_omega": round(
                    _safe_float(sample.get("requested_omega"), 0.0),
                    6,
                ),
                "limited_v": round(
                    _safe_float(sample.get("limited_v"), 0.0),
                    6,
                ),
                "limited_omega": round(
                    _safe_float(sample.get("limited_omega"), 0.0),
                    6,
                ),
                "v_step": round(float(resolved_v - previous_v), 6),
                "omega_step": round(float(resolved_omega - previous_omega), 6),
                "actual_v": round(_safe_float(sample.get("actual_v"), 0.0), 6),
                "actual_omega": round(
                    _safe_float(sample.get("actual_omega"), 0.0),
                    6,
                ),
                "target_wheels": [
                    round(_safe_float(sample.get("target_left_mps"), 0.0), 6),
                    round(_safe_float(sample.get("target_right_mps"), 0.0), 6),
                ],
                "actual_wheels": [
                    round(_safe_float(sample.get("actual_left_mps"), 0.0), 6),
                    round(_safe_float(sample.get("actual_right_mps"), 0.0), 6),
                ],
                "pwm": [round(float(pwm_left), 6), round(float(pwm_right), 6)],
                "pwm_step": round(
                    max(
                        abs(float(pwm_left - previous_pwm_left)),
                        abs(float(pwm_right - previous_pwm_right)),
                    ),
                    6,
                ),
                "primitive_chain": [
                    str(sample.get("turn_primitive_requested", "") or ""),
                    str(sample.get("turn_primitive_limited", "") or ""),
                    str(sample.get("turn_primitive_executed", "") or ""),
                    str(sample.get("turn_primitive_actual", "") or ""),
                ],
                "headings": {
                    "ekf_deg": _finite_trace_value(
                        (sample.get("pose") or {}).get("theta_deg")
                    ),
                    "lidar_deg": _finite_trace_value(
                        sample.get("lidar_heading_deg")
                    ),
                    "imu_fused_deg": _finite_trace_value(
                        sample.get("imu_heading_deg")
                    ),
                    "imu_gyro_z_rad_s": _finite_trace_value(
                        sample.get("imu_gyro_z_rad_s")
                    ),
                    "imu_measurement_time_s": _finite_trace_value(
                        sample.get("imu_measurement_time_s")
                    ),
                },
                "localization": {
                    "mode": str(sample.get("localization_mode", "") or ""),
                    "trust": round(
                        _safe_float(sample.get("localization_trust"), 0.0),
                        6,
                    ),
                    "pose_confidence": _finite_trace_value(
                        sample.get("lidar_pose_confidence")
                    ),
                    "candidate_confidence": _finite_trace_value(
                        sample.get("lidar_candidate_confidence")
                    ),
                    "candidate_measurement_confidence": _finite_trace_value(
                        sample.get("lidar_candidate_measurement_confidence")
                    ),
                    "latest_measurement_confidence": _finite_trace_value(
                        sample.get("lidar_latest_measurement_confidence")
                    ),
                    "status": str(
                        sample.get("lidar_localization_status", "") or ""
                    ),
                    "tracking_ready": bool(
                        sample.get("lidar_tracking_ready", False)
                    ),
                    "tracking_loss_latched": bool(
                        sample.get("lidar_tracking_loss_latched", False)
                    ),
                    "ekf_gap_s": _finite_trace_value(
                        sample.get("lidar_ekf_applied_gap_s")
                    ),
                    "ekf_nis": _finite_trace_value(sample.get("lidar_ekf_nis")),
                    "innovation_theta_abs": _finite_trace_value(
                        sample.get("motion_quality_innovation_theta_abs")
                    ),
                    "innovation_v_abs": _finite_trace_value(
                        sample.get("motion_quality_innovation_v_abs")
                    ),
                },
                "matcher": {
                    "result_id": int(sample.get("lidar_matcher_result_id", 0) or 0),
                    "measurement_confidence": _finite_trace_value(
                        quality.get("measurement_confidence")
                    ),
                    "integrity_score": _finite_trace_value(
                        quality.get("localization_integrity_score")
                    ),
                    "uniqueness": _finite_trace_value(
                        quality.get("measurement_uniqueness_score")
                    ),
                    "posterior_uniqueness": _finite_trace_value(
                        quality.get("posterior_uniqueness_score")
                    ),
                    "coverage": _finite_trace_value(quality.get("coverage_score")),
                    "sector_coverage": _finite_trace_value(
                        quality.get("sector_coverage")
                    ),
                    "rmse_m": _finite_trace_value(quality.get("robust_rmse_m")),
                    "degenerate": bool(sample.get("lidar_matcher_degenerate", False)),
                    "degeneracy_reasons": list(
                        sample.get("lidar_matcher_degeneracy_reasons") or []
                    ),
                    "timed_out": bool(quality.get("timed_out", False)),
                    "deadline_stage": quality.get("deadline_stage"),
                    "latency_ms": _finite_trace_value(
                        sample.get("lidar_matcher_latency_ms")
                    ),
                    "queue_delay_ms": _finite_trace_value(
                        sample.get("lidar_matcher_queue_delay_ms")
                    ),
                    "runtime_ms": _finite_trace_value(
                        sample.get("lidar_matcher_runtime_ms")
                    ),
                },
                "safety": {
                    "allow": bool(sample.get("safety_allow", False)),
                    "reason": str(sample.get("safety_reason", "") or ""),
                    "stop_type": str(sample.get("stop_type", "") or ""),
                    "bad_lidar_observations": int(
                        sample.get("amr_lidar_bad_observation_count", 0) or 0
                    ),
                    "lidar_quality_reason": str(
                        sample.get("amr_lidar_last_quality_reason", "") or ""
                    ),
                },
            }
        )
        previous = sample
    return rows


def _finite_trace_value(value: Any) -> Optional[float]:
    number = _safe_float(value, math.nan)
    return round(float(number), 6) if math.isfinite(number) else None


def _motion_phase_episode_metrics(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
    def phase_for(sample: Dict[str, Any]) -> str:
        if str(sample.get("stop_type", "") or "").upper() == "EMERGENCY_STOP":
            return "emergency_stop"
        phase = str(sample.get("local_planner_phase", "") or "")
        if phase:
            return phase
        obstacle_mode = str(sample.get("obstacle_avoidance_mode", "") or "")
        if obstacle_mode:
            return obstacle_mode
        moving = bool(
            abs(_safe_float(sample.get("resolved_v"), 0.0)) >= 0.012
            or abs(_safe_float(sample.get("resolved_omega"), 0.0)) >= 0.08
        )
        if bool(sample.get("room_cruise_v2_active", False)) and not moving:
            return "room_cruise_hold"
        return "idle_or_unclassified"

    episodes: List[Dict[str, Any]] = []
    current_phase = ""
    current_start = 0.0
    current_duration = 0.0
    current_samples = 0
    for index, sample in enumerate(samples):
        phase = phase_for(sample)
        ts = _safe_float(sample.get("ts"), float(index))
        next_ts = (
            _safe_float(samples[index + 1].get("ts"), ts)
            if index + 1 < len(samples)
            else ts
        )
        sample_duration = max(0.0, min(0.50, float(next_ts) - float(ts)))
        if phase != current_phase:
            if current_phase:
                episodes.append(
                    {
                        "phase": str(current_phase),
                        "start_ts": round(float(current_start), 6),
                        "duration_s": round(float(current_duration), 4),
                        "sample_count": int(current_samples),
                    }
                )
            current_phase = str(phase)
            current_start = float(ts)
            current_duration = 0.0
            current_samples = 0
        current_duration += float(sample_duration)
        current_samples += 1
    if current_phase:
        episodes.append(
            {
                "phase": str(current_phase),
                "start_ts": round(float(current_start), 6),
                "duration_s": round(float(current_duration), 4),
                "sample_count": int(current_samples),
            }
        )

    phase_counts = collections.Counter(
        phase_for(sample) for sample in samples
    )
    max_duration: Dict[str, float] = {}
    total_duration: Dict[str, float] = {}
    for episode in episodes:
        phase = str(episode["phase"])
        duration = float(episode["duration_s"])
        max_duration[phase] = max(float(max_duration.get(phase, 0.0)), duration)
        total_duration[phase] = float(total_duration.get(phase, 0.0)) + duration
    return {
        "phase_sample_counts": dict(sorted(phase_counts.items())),
        "phase_max_episode_s": {
            key: round(value, 4) for key, value in sorted(max_duration.items())
        },
        "phase_total_s": {
            key: round(value, 4) for key, value in sorted(total_duration.items())
        },
        "episodes": episodes,
    }


def _room_cruise_idle_status(status: Dict[str, Any]) -> bool:
    st = dict(status or {})
    room_v2 = dict(st.get("room_cruise_v2") or {})
    resolved = _resolved(st)
    motion_command = dict(st.get("motion_command") or {})
    pwm = dict(st.get("pwm") or {})
    track_targets = dict(motion_command.get("track_targets") or {})
    final_after_shaping = dict(resolved.get("final_after_shaping") or {})
    resolved_v = _safe_float(final_after_shaping.get("v_target", resolved.get("v_target")), 0.0)
    resolved_omega = _safe_float(final_after_shaping.get("omega_target", resolved.get("omega_target")), 0.0)
    state = str(st.get("state", "") or "").strip().upper()
    execution_state = str(st.get("motion_execution_state", "") or "").strip().lower()
    resolved_name = str(resolved.get("name", "") or "")
    resolved_type = str(resolved.get("command_type", "") or "")
    resolved_layer = str(resolved.get("layer", "") or "")
    room_route_active = bool(
        room_v2.get("active", False)
        or resolved_name == "room_cruise_v2_local_navigation"
        or (
            resolved_type == "local_planner_segment"
            and resolved_layer == "LOCAL_NAVIGATION"
            and bool((dict(resolved.get("details") or {})).get("room_cruise_v2"))
        )
    )
    command_zero = bool(
        abs(float(resolved_v)) <= 0.002
        and abs(float(resolved_omega)) <= 0.010
        and max(
            abs(_safe_float(track_targets.get("left_mps"), 0.0)),
            abs(_safe_float(track_targets.get("right_mps"), 0.0)),
        )
        <= 0.002
    )
    pwm_zero = max(
        abs(_safe_float(pwm.get("left"), 0.0)),
        abs(_safe_float(pwm.get("right"), 0.0)),
    ) <= 0.003
    execution_idle = bool(state == "IDLE" or execution_state in {"succeeded", "cancelled", "idle"})
    return bool((not room_route_active) and command_zero and pwm_zero and execution_idle)


def _stop_room_cruise_and_collect_idle(
    *,
    token: str,
    reason: str,
    poll_s: float,
) -> Dict[str, Any]:
    stop_cmd: Dict[str, Any] = {}
    fallback_cmd: Dict[str, Any] = {}
    stop_error = ""
    idle_samples: List[Dict[str, Any]] = []
    try:
        stop_cmd = _send_command_checked(
            "stop_room_cruise_v2",
            token=str(token),
            timeout_s=5.0,
            motion_source="STATE",
            reason=str(reason),
        )
    except Exception as exc:
        stop_error = str(exc)
        fallback_cmd = _send_command_checked("stop", token=str(token), timeout_s=5.0, motion_source="STATE")

    consecutive_idle = 0
    idle_confirmed = False
    deadline = time.monotonic() + float(POST_STOP_TIMEOUT_S)
    idle_poll_s = max(0.05, min(0.20, float(poll_s)))
    while time.monotonic() <= deadline:
        st = _read_status(force=True)
        if st:
            row = _sample(st)
            row["post_stop_sample"] = True
            row["room_cruise_idle_confirmed_sample"] = bool(_room_cruise_idle_status(st))
            idle_samples.append(row)
            if bool(row["room_cruise_idle_confirmed_sample"]):
                consecutive_idle += 1
            else:
                consecutive_idle = 0
            if consecutive_idle >= int(POST_STOP_IDLE_SAMPLES):
                idle_confirmed = True
                break
        time.sleep(idle_poll_s)

    if not idle_confirmed and not fallback_cmd:
        try:
            fallback_cmd = _send_command_checked("stop", token=str(token), timeout_s=5.0, motion_source="STATE")
        except Exception as exc:
            stop_error = (stop_error + "; " if stop_error else "") + str(exc)

    return {
        "stop_command": stop_cmd,
        "fallback_stop_command": fallback_cmd,
        "stop_error": stop_error,
        "idle_confirmed": bool(idle_confirmed),
        "idle_consecutive_samples": int(consecutive_idle),
        "idle_required_samples": int(POST_STOP_IDLE_SAMPLES),
        "idle_samples": idle_samples,
    }


def _finalize_m3_room_cruise_artifacts(room_result: Dict[str, Any]) -> Dict[str, Any]:
    try:
        from tools import M3_room_cruise_minoseg as m3_room  # type: ignore
    except Exception as exc:
        return {"ok": False, "reason": f"import_failed:{exc}"}

    raw_samples = list((room_result or {}).get("samples") or [])
    if not raw_samples:
        return {"ok": False, "reason": "room_cruise_samples_missing"}

    try:
        base_result = m3_room._base_result_compact(room_result)
        result, samples = m3_room.analyze_samples(raw_samples, base_results=[base_result])
        summary = dict((room_result or {}).get("summary") or {})
        result.update(
            {
                "test_name": "M3_room_cruise_minoseg",
                "duration_s_per_run": _safe_float(summary.get("duration_s"), 0.0),
                "repeat_count": 1,
                "poll_s": None,
                "auto_finalized_by": "tools/room_cruise_v2_live.py",
                "artifact_paths": {
                    "result": str(m3_room.RESULT_PATH.relative_to(PROJECT_ROOT)),
                    "summary": str(m3_room.SUMMARY_PATH.relative_to(PROJECT_ROOT)),
                    "samples": str(m3_room.SAMPLES_PATH.relative_to(PROJECT_ROOT)),
                    "incident": str(m3_room.INCIDENT_PATH.relative_to(PROJECT_ROOT)),
                    "base_room_cruise_summary": str(LATEST_SUMMARY.relative_to(PROJECT_ROOT)),
                    "base_room_cruise_result": str(LATEST_RESULT.relative_to(PROJECT_ROOT)),
                },
                "base_room_cruise_runs": [base_result],
            }
        )
        m3_summary = m3_room.write_artifacts(result, samples)
        return {
            "ok": True,
            "status": str(m3_summary.get("status", "")),
            "summary": str(m3_room.SUMMARY_PATH.relative_to(PROJECT_ROOT)),
            "result": str(m3_room.RESULT_PATH.relative_to(PROJECT_ROOT)),
            "samples": str(m3_room.SAMPLES_PATH.relative_to(PROJECT_ROOT)),
            "incident": str(m3_room.INCIDENT_PATH.relative_to(PROJECT_ROOT)),
        }
    except Exception as exc:
        return {"ok": False, "reason": f"finalize_failed:{exc}"}


def _percentile(values: List[float], fraction: float) -> Optional[float]:
    finite = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not finite:
        return None
    index = min(len(finite) - 1, max(0, int(round((len(finite) - 1) * float(fraction)))))
    return finite[index]


def _angle_delta_deg(a: float, b: float) -> float:
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def _progress(samples: List[Dict[str, Any]]) -> float:
    total = 0.0
    prev: Optional[Dict[str, float]] = None
    for sample in samples:
        pose = dict(sample.get("pose") or {})
        if prev is not None:
            total += math.hypot(float(pose.get("x", 0.0)) - float(prev.get("x", 0.0)), float(pose.get("y", 0.0)) - float(prev.get("y", 0.0)))
        prev = pose
    return float(total)


def _build_ekf_truth_surface(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
    moving = [
        row
        for row in samples
        if not bool(row.get("post_stop_sample", False))
        and (
            abs(_safe_float(row.get("resolved_v"), 0.0)) >= 0.012
            or abs(_safe_float(row.get("resolved_omega"), 0.0)) >= 0.08
        )
    ]
    evidence_rows = moving or [row for row in samples if not bool(row.get("post_stop_sample", False))]

    def _primitive_chain(row: Dict[str, Any]) -> Tuple[str, str, str, str]:
        return tuple(
            str(row.get(key, "") or "").strip()
            for key in (
                "turn_primitive_requested",
                "turn_primitive_limited",
                "turn_primitive_executed",
                "turn_primitive_actual",
            )
        )

    truth_anchor_rows = [
        row
        for row in evidence_rows
        if bool(row.get("actual_primitive_measurement_ready", False))
        and bool(row.get("actual_primitive_measurement_reliable", False))
        and all(_primitive_chain(row))
        and "UNKNOWN" not in {value.upper() for value in _primitive_chain(row)}
        and len(set(_primitive_chain(row))) == 1
    ]
    primitive_rows = truth_anchor_rows or evidence_rows

    def _most_common_text(key: str, rows: Optional[List[Dict[str, Any]]] = None) -> str:
        source_rows = rows if rows is not None else evidence_rows
        values = [str(row.get(key, "") or "").strip() for row in source_rows]
        values = [value for value in values if value]
        if not values:
            return ""
        return max(set(values), key=lambda value: (values.count(value), -values.index(value)))

    def _median_finite(key: str) -> Optional[float]:
        values = [_safe_float(row.get(key), math.nan) for row in evidence_rows]
        finite = [float(value) for value in values if math.isfinite(float(value))]
        return float(statistics.median(finite)) if finite else None

    motion_actual_ssot = _most_common_text("motion_actual_ssot")
    truth_basis = {
        "motion_actual_ssot": motion_actual_ssot,
        "encoder_pose_active_samples": sum(
            1
            for row in evidence_rows
            if bool(row.get("actual_primitive_measurement_available", False))
        ),
        "lidar_odom_latest_age_s": _median_finite("lidar_latest_age_s"),
        "lidar_odom_latest_confidence": _median_finite("lidar_pose_confidence"),
        "lidar_odom_delivery_status": _most_common_text("lidar_odom_delivery_status"),
    }
    return {
        "motion_actual_ssot": motion_actual_ssot,
        "truth_basis": truth_basis,
        "turn_primitive_requested": _most_common_text("turn_primitive_requested", primitive_rows),
        "turn_primitive_limited": _most_common_text("turn_primitive_limited", primitive_rows),
        "turn_primitive_executed": _most_common_text("turn_primitive_executed", primitive_rows),
        "turn_primitive_actual": _most_common_text("turn_primitive_actual", primitive_rows),
        "resolved_command_types_seen": sorted(
            {
                str(row.get("resolved_command_type", "") or "").strip()
                for row in evidence_rows
                if str(row.get("resolved_command_type", "") or "").strip()
            }
        ),
        "sample_count": int(len(evidence_rows)),
        "truth_anchor_sample_count": int(len(truth_anchor_rows)),
        "truth_anchor_policy": "reliable_matching_primitive_chain",
    }


def run(args: argparse.Namespace) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    started = time.time()
    run_tag = time.strftime("room_cruise_v2_live_%Y%m%dT%H%M%SZ", time.gmtime())
    run_dir = AGENT_TESTS_DIR / run_tag
    samples: List[Dict[str, Any]] = []
    start_status = _read_status(force=True)
    start_pose = _pose(start_status)
    stop_result: Dict[str, Any] = {}

    start_cmd = _send_command_checked(
        "start_room_cruise_v2",
        token=str(args.token),
        timeout_s=5.0,
        motion_source="STATE",
        duration_s=float(args.duration_s) + 2.0,
        v_max=float(args.v_max_mps),
        omega_max=float(args.omega_max_rad_s),
    )
    deadline = time.monotonic() + max(1.0, float(args.duration_s))
    try:
        while time.monotonic() <= deadline:
            st = _read_status(force=False)
            if st:
                samples.append(_sample(st))
            time.sleep(max(0.04, float(args.poll_s)))
    finally:
        stop_result = _stop_room_cruise_and_collect_idle(
            token=str(args.token),
            reason="ROOM_CRUISE_V2_LIVE_DONE",
            poll_s=float(args.poll_s),
        )
        samples.extend(list(stop_result.get("idle_samples") or []))

    end_status = _read_status(force=True)
    end_pose = _pose(end_status)
    idle_confirmed = bool(stop_result.get("idle_confirmed", False))
    duration_s = max(0.0, time.time() - started)
    progress_m = _progress(samples)
    front_values = [float(s["front_m"]) for s in samples if s.get("front_m") is not None]
    min_front = min(front_values) if front_values else None
    local_nav_count = sum(1 for s in samples if bool(s.get("local_navigation_active", False)))
    room_v2_count = sum(1 for s in samples if bool(s.get("room_cruise_v2_active", False)))
    rolling_count = sum(1 for s in samples if bool(s.get("rolling_local_map_has_data", False)))
    local_segment_count = sum(
        1
        for s in samples
        if str(s.get("resolved_command_type", "")) == "local_planner_segment"
        and str(s.get("resolved_layer", "")) == "LOCAL_NAVIGATION"
    )
    v2_detail_count = sum(1 for s in samples if bool(s.get("resolved_has_room_cruise_v2_details", False)))
    direct_track_count = sum(1 for s in samples if str(s.get("resolved_command_type", "")) == "set_track_velocity")
    track_exec_count = sum(
        1
        for s in samples
        if str(s.get("motion_execution_mode", s.get("resolved_execution_mode", "")) or "").upper() == "TRACK_EXEC"
        or str(s.get("control_track_reference_mode", "") or "").upper() in {"TRACK_EXEC", "HEADING_PIVOT"}
    )
    pivot_track_required_count = sum(1 for s in samples if bool(s.get("local_nav_pivot_track_required", False)))
    emergency_count = sum(1 for s in samples if str(s.get("stop_type", "")).upper() == "EMERGENCY_STOP")
    moving_count = sum(1 for s in samples if abs(float(s.get("resolved_v", 0.0))) > 0.004 or abs(float(s.get("resolved_omega", 0.0))) > 0.015)
    sample_count = len(samples)
    m5_samples = [s for s in samples if bool(s.get("m5_full_stack_active", False))]
    m5_goal_ids = [str(s.get("m5_goal_id", "") or "") for s in m5_samples if str(s.get("m5_goal_id", "") or "")]
    m5_goal_switches = sum(
        1 for previous_goal, current_goal in zip(m5_goal_ids[:-1], m5_goal_ids[1:])
        if previous_goal != current_goal
    )
    m5_goal_event_counts = dict(
        sorted(
            collections.Counter(
                str(s.get("m5_goal_event", "") or "")
                for s in m5_samples
                if str(s.get("m5_goal_event", "") or "")
            ).items()
        )
    )
    m5_route_selector_count = sum(
        1 for s in m5_samples
        if str(s.get("m5_route_selector_provider", "") or "")
        == "global_motion_policy_navigation_selector"
    )
    m5_local_path_count = sum(1 for s in m5_samples if bool(s.get("m5_local_path_segment_active", False)))
    m5_uncontrolled_reverse_count = sum(
        1
        for s in m5_samples
        if float(s.get("resolved_v", 0.0) or 0.0) < -0.012
        and str(s.get("local_planner_phase", "") or "")
        not in {"room_cruise_reverse_arc", "room_cruise_reverse_straight"}
    )
    m5_completed_waypoints = max(
        (int(s.get("m5_completed_waypoints", 0) or 0) for s in m5_samples),
        default=0,
    )
    m5_replans = max(
        (int(s.get("m5_replan_count", 0) or 0) for s in m5_samples),
        default=0,
    )
    m5_visited_cells = max(
        (int(s.get("m5_visited_cell_count", 0) or 0) for s in m5_samples),
        default=0,
    )
    m5_start_sample = dict(m5_samples[0]) if m5_samples else {}

    def _finite_or_none(value: Any) -> Optional[float]:
        number = _safe_float(value, math.nan)
        return round(float(number), 4) if math.isfinite(number) else None

    m5_start_environment = {
        "pose": dict(m5_start_sample.get("pose") or start_pose),
        "front_m": _finite_or_none(m5_start_sample.get("front_m")),
        "min_clearance_m": _finite_or_none(m5_start_sample.get("min_clearance_m")),
        "left_clearance_m": _finite_or_none(m5_start_sample.get("left_clearance_m")),
        "right_clearance_m": _finite_or_none(m5_start_sample.get("right_clearance_m")),
        "rear_clearance_m": _finite_or_none(m5_start_sample.get("rear_clearance_m")),
    }

    linear_ratios: List[float] = []
    angular_ratios: List[float] = []
    wheel_errors: List[float] = []
    unexplained_zero_count = 0
    v_steps: List[float] = []
    omega_steps: List[float] = []
    pwm_steps: List[float] = []
    pwm_sign_oscillations = 0
    forbidden_path_count = 0
    localization_contradiction_count = 0
    previous: Optional[Dict[str, Any]] = None
    for sample in samples:
        rv = float(sample.get("resolved_v", 0.0))
        rw = float(sample.get("resolved_omega", 0.0))
        av = float(sample.get("actual_v", 0.0))
        aw = float(sample.get("actual_omega", 0.0))
        if abs(rv) >= 0.015 and rv * av > 0.0:
            linear_ratios.append(abs(av / rv))
        if abs(rw) >= 0.12 and rw * aw > 0.0:
            angular_ratios.append(abs(aw / rw))
        for side in ("left", "right"):
            target = float(sample.get(f"target_{side}_mps", 0.0))
            actual = float(sample.get(f"actual_{side}_mps", 0.0))
            if abs(target) >= 0.012:
                wheel_errors.append(abs(target - actual))
        requested_motion = abs(rv) >= 0.012 or abs(rw) >= 0.08
        pwm_zero = max(abs(float(sample.get("pwm_left", 0.0))), abs(float(sample.get("pwm_right", 0.0)))) < 0.015
        if requested_motion and pwm_zero and sample.get("safety_allow", False):
            unexplained_zero_count += 1
        layer = str(sample.get("active_motion_layer", "") or "").upper()
        command_type = str(sample.get("active_motion_type", "") or "").lower()
        if sample.get("service_motion_active", False) or "LEGACY" in layer or command_type in {"set_motor_pwm", "set_tank", "step_tank"}:
            forbidden_path_count += 1
        if str(sample.get("localization_mode", "") or "").upper() == "LOST" and sample.get("localization_allow_motion", False):
            localization_contradiction_count += 1
        if previous is not None:
            v_steps.append(
                abs(
                    float(sample.get("resolved_v", 0.0))
                    - float(previous.get("resolved_v", 0.0))
                )
            )
            omega_steps.append(
                abs(
                    float(sample.get("resolved_omega", 0.0))
                    - float(previous.get("resolved_omega", 0.0))
                )
            )
            pwm_steps.append(
                max(
                    abs(float(sample.get("pwm_left", 0.0)) - float(previous.get("pwm_left", 0.0))),
                    abs(float(sample.get("pwm_right", 0.0)) - float(previous.get("pwm_right", 0.0))),
                )
            )
            for side in ("left", "right"):
                target_now = float(sample.get(f"target_{side}_mps", 0.0))
                target_prev = float(previous.get(f"target_{side}_mps", 0.0))
                pwm_now = float(sample.get(f"pwm_{side}", 0.0))
                pwm_prev = float(previous.get(f"pwm_{side}", 0.0))
                if (
                    abs(target_now) >= 0.012
                    and target_now * target_prev > 0.0
                    and abs(pwm_now) >= 0.03
                    and pwm_now * pwm_prev < 0.0
                ):
                    pwm_sign_oscillations += 1
        previous = sample

    median_linear_ratio = statistics.median(linear_ratios) if linear_ratios else None
    median_angular_ratio = statistics.median(angular_ratios) if angular_ratios else None
    wheel_error_p90 = _percentile(wheel_errors, 0.90)
    v_step_p95 = _percentile(v_steps, 0.95)
    omega_step_p95 = _percentile(omega_steps, 0.95)
    pwm_step_p95 = _percentile(pwm_steps, 0.95)
    endpoint_heading_metrics = _relative_heading_change_metrics(samples)
    endpoint_heading_spread = endpoint_heading_metrics.get("max_pair_disagreement_deg")
    ekf_truth_surface = _build_ekf_truth_surface(samples)
    motion_phase_metrics = _motion_phase_episode_metrics(samples)
    audit_trace = _motion_audit_trace(samples)

    checks = {
        "duration_complete": duration_s >= float(args.duration_s) * 0.95,
        "progress_ok": progress_m >= float(args.min_progress_m),
        "collision_margin_ok": min_front is not None and float(min_front) >= float(args.min_front_m),
        "emergency_free": int(emergency_count) == 0,
        "room_cruise_v2_observed": room_v2_count >= max(3, int(sample_count * 0.20)),
        "local_navigation_observed": local_nav_count >= max(3, int(sample_count * 0.20)),
        "local_segment_resolved": local_segment_count >= max(3, int(sample_count * 0.20)),
        "v2_ownership_details": v2_detail_count >= max(3, int(sample_count * 0.20)),
        "rolling_local_map_observed": rolling_count >= max(1, int(sample_count * 0.10)),
        "no_direct_track_reference": int(direct_track_count) == 0,
        "m3_track_execution_observed": track_exec_count >= max(3, int(local_segment_count * 0.20)),
        "no_blocked_legacy_pivot_path": int(pivot_track_required_count) == 0,
        "motion_observed": moving_count >= max(3, int(sample_count * 0.10)),
        "linear_command_fidelity": median_linear_ratio is not None and 0.55 <= median_linear_ratio <= 1.45,
        "angular_command_fidelity": median_angular_ratio is not None and 0.40 <= median_angular_ratio <= 1.80,
        "wheel_tracking_error": wheel_error_p90 is not None and wheel_error_p90 <= 0.04,
        "no_unexplained_pwm_zero": unexplained_zero_count <= max(2, int(sample_count * 0.04)),
        "pwm_step_smoothness": pwm_step_p95 is not None and pwm_step_p95 <= 0.20,
        "no_pwm_sign_oscillation": int(pwm_sign_oscillations) == 0,
        "no_service_or_legacy_path": int(forbidden_path_count) == 0,
        "localization_truth_consistent": int(localization_contradiction_count) == 0,
        "sensor_endpoint_heading_agreement": endpoint_heading_spread is not None and endpoint_heading_spread <= 15.0,
        "room_cruise_stopped_idle": bool(idle_confirmed),
    }
    ok = all(bool(v) for v in checks.values())
    fail_reasons = [name for name, passed in checks.items() if not bool(passed)]
    summary = {
        "status": "PASS" if ok else "FAIL",
        "duration_s": round(float(duration_s), 3),
        "progress_m": round(float(progress_m), 4),
        "min_front_clearance_m": None if min_front is None else round(float(min_front), 4),
        "sample_count": int(sample_count),
        "room_cruise_v2_samples": int(room_v2_count),
        "local_navigation_samples": int(local_nav_count),
        "local_segment_samples": int(local_segment_count),
        "rolling_local_map_samples": int(rolling_count),
        "direct_track_reference_samples": int(direct_track_count),
        "m3_track_execution_samples": int(track_exec_count),
        "blocked_legacy_pivot_path_samples": int(pivot_track_required_count),
        "emergency_stop_events": int(emergency_count),
        "median_actual_commanded_linear_ratio": None if median_linear_ratio is None else round(median_linear_ratio, 4),
        "median_actual_commanded_angular_ratio": None if median_angular_ratio is None else round(median_angular_ratio, 4),
        "wheel_tracking_error_p90_mps": None if wheel_error_p90 is None else round(wheel_error_p90, 4),
        "v_step_p95_mps": None if v_step_p95 is None else round(v_step_p95, 4),
        "omega_step_p95_rad_s": None if omega_step_p95 is None else round(omega_step_p95, 4),
        "pwm_step_p95": None if pwm_step_p95 is None else round(pwm_step_p95, 4),
        "unexplained_pwm_zero_samples": int(unexplained_zero_count),
        "pwm_sign_oscillations": int(pwm_sign_oscillations),
        "forbidden_path_samples": int(forbidden_path_count),
        "localization_contradiction_samples": int(localization_contradiction_count),
        "endpoint_heading_spread_deg": None if endpoint_heading_spread is None else round(endpoint_heading_spread, 3),
        "endpoint_heading_metrics": endpoint_heading_metrics,
        "room_cruise_idle_confirmed": bool(idle_confirmed),
        "room_cruise_idle_samples": int(stop_result.get("idle_consecutive_samples", 0) or 0),
        "room_cruise_idle_required_samples": int(stop_result.get("idle_required_samples", POST_STOP_IDLE_SAMPLES) or POST_STOP_IDLE_SAMPLES),
        "m5_full_stack_samples": int(len(m5_samples)),
        "m5_unique_goal_count": int(len(set(m5_goal_ids))),
        "m5_goal_switch_count": int(m5_goal_switches),
        "m5_goal_switch_ratio": round(float(m5_goal_switches / max(1, len(m5_goal_ids) - 1)), 5),
        "m5_goal_event_counts": m5_goal_event_counts,
        "m5_completed_waypoints": int(m5_completed_waypoints),
        "m5_replan_count": int(m5_replans),
        "m5_visited_cell_count": int(m5_visited_cells),
        "m5_route_selector_samples": int(m5_route_selector_count),
        "m5_local_path_segment_samples": int(m5_local_path_count),
        "m5_uncontrolled_reverse_samples": int(m5_uncontrolled_reverse_count),
        "m5_start_environment": m5_start_environment,
        "motion_phase_metrics": motion_phase_metrics,
        "fail_reasons": fail_reasons,
    }
    result = {
        "test_name": "room_cruise_v2_live",
        "status": summary["status"],
        "success": bool(ok),
        "checks": checks,
        "summary": summary,
        "start_command": start_cmd,
        "stop_result": {key: value for key, value in stop_result.items() if key != "idle_samples"},
        "start_pose": start_pose,
        "end_pose": end_pose,
        "ekf_truth_surface": ekf_truth_surface,
        "motion_audit_trace": audit_trace,
        "samples": samples if not bool(args.compact) else samples[-12:],
        "artifact_paths": {
            "summary": str(LATEST_SUMMARY.relative_to(PROJECT_ROOT)),
            "result": str(LATEST_RESULT.relative_to(PROJECT_ROOT)),
            "run_dir": str(run_dir.relative_to(PROJECT_ROOT)),
        },
    }
    run_dir.mkdir(parents=True, exist_ok=True)
    _write_json(run_dir / "summary.json", summary)
    _write_json(run_dir / "result.json", result)
    _write_json(LATEST_SUMMARY, summary)
    _write_json(LATEST_RESULT, result)
    m3_finalize = _finalize_m3_room_cruise_artifacts(result)
    if isinstance(m3_finalize, dict):
        result["m3_artifact_finalize"] = dict(m3_finalize)
        summary["m3_artifact_finalize"] = dict(m3_finalize)
        _write_json(run_dir / "summary.json", summary)
        _write_json(run_dir / "result.json", result)
        _write_json(LATEST_SUMMARY, summary)
        _write_json(LATEST_RESULT, result)
    return result


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration-s", type=float, default=45.0)
    ap.add_argument("--min-progress-m", type=float, default=0.35)
    ap.add_argument("--min-front-m", type=float, default=0.27)
    ap.add_argument("--v-max-mps", type=float, default=0.30)
    ap.add_argument("--omega-max-rad-s", type=float, default=0.60)
    ap.add_argument("--poll-s", type=float, default=0.12)
    ap.add_argument("--token", default=TOKEN)
    ap.add_argument("--compact", action="store_true")
    args = ap.parse_args()
    result = run(args)
    summary = dict(result.get("summary") or {})
    print(
        "ROOM_CRUISE_V2|"
        f"status={summary.get('status')}|"
        f"duration_s={summary.get('duration_s')}|"
        f"progress_m={summary.get('progress_m')}|"
        f"min_front={summary.get('min_front_clearance_m')}|"
        f"local_nav={summary.get('local_navigation_samples')}|"
        f"fail={','.join(summary.get('fail_reasons') or [])}"
    )
    print(
        "JSON_RESULT:"
        + json.dumps(
            {
                "schema": "ROOM_CRUISE_V2_HUB_PAYLOAD_V1",
                "status": str(result.get("status", "")),
                "success": bool(result.get("success", False)),
                "ekf_truth_surface": dict(result.get("ekf_truth_surface") or {}),
                "summary": {
                    "progress_m": summary.get("progress_m"),
                    "m5_unique_goal_count": summary.get("m5_unique_goal_count"),
                    "m5_completed_waypoints": summary.get("m5_completed_waypoints"),
                    "m5_replan_count": summary.get("m5_replan_count"),
                    "m5_visited_cell_count": summary.get("m5_visited_cell_count"),
                    "fail_reasons": list(summary.get("fail_reasons") or []),
                },
            },
            sort_keys=True,
            separators=(",", ":"),
        )
    )
    return 0 if str(summary.get("status")) == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
