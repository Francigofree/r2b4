#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Optional

from log.unified_logger import CHANNEL_AUDIT, CHANNEL_SAFETY, CHANNEL_TELEMETRY, get_unified_logger


def _copy_keys(src: dict[str, Any], keys: tuple[str, ...]) -> dict[str, Any]:
    return {key: src.get(key) for key in keys if key in src}


def _compact_status_telemetry(payload: dict[str, Any]) -> dict[str, Any]:
    """Keep runtime telemetry logs useful without persisting the full status tree."""
    src = dict(payload or {})
    out = _copy_keys(
        src,
        (
            "status_version",
            "time",
            "state",
            "pose",
            "v_target",
            "v_cmd",
            "omega_target",
            "motion_command_source",
            "motion_target_owner",
            "control_mode",
            "motion_execution_mode",
            "motion_execution_state",
            "safety_allow",
            "speed_limiting_reason",
            "safety_limiting_reason",
            "camera_enabled",
            "encoder_enabled",
            "odometry_mode",
            "full_log_active",
            "runtime_preset",
            "startup_ready",
        ),
    )
    out["telemetry_schema"] = "STATUS_TELEMETRY_COMPACT_V1"
    out["telemetry_compacted"] = True
    out["pwm"] = _copy_keys(dict(src.get("pwm") or {}), ("left", "right"))
    out["lidar"] = _copy_keys(
        dict(src.get("lidar") or {}),
        (
            "blocked_front",
            "blocked_back",
            "min_dist",
            "min_dist_narrow",
            "min_back",
            "avg_left",
            "avg_right",
            "lidar_pose_confidence",
            "latest_age_s",
            "latest_confidence",
            "odom_status",
        ),
    )
    out["safety"] = _copy_keys(dict(src.get("safety") or {}), ("allow", "reason", "stop_type"))
    out["logger"] = _copy_keys(dict(src.get("logger") or {}), ("queue_depth", "dropped_messages", "write_errors"))
    loop_budget = dict(src.get("loop_budget") or {})
    out["loop_budget"] = _copy_keys(loop_budget, ("total_ema_ms", "updated_ts"))

    adaptive = dict(src.get("adaptive_motion") or {})
    out["adaptive_motion"] = _copy_keys(
        adaptive,
        ("active", "follow_state", "target_distance_m", "target_angle_deg", "target_confidence"),
    )
    for status_key in ("target_camera_status", "target_lidar_status", "target_search_status"):
        status = dict(adaptive.get(status_key) or {})
        if status:
            out["adaptive_motion"][status_key] = _copy_keys(
                status,
                (
                    "state",
                    "source",
                    "target_visible",
                    "target_usable",
                    "frame_ok",
                    "stale",
                    "detector",
                    "detector_confidence",
                    "rotation_deg",
                    "image_width_px",
                    "image_height_px",
                    "target_angle_deg",
                    "target_center_offset_ratio",
                    "bearing_fov_deg",
                    "bearing_fov_source",
                    "distance_source",
                    "distance_m",
                    "angle_deg",
                    "usable_distance",
                    "active",
                    "reason",
                    "rotations_completed",
                    "total_rotated_deg",
                ),
            )

    motion_command = dict(src.get("motion_command") or {})
    out["motion_command"] = _copy_keys(
        motion_command,
        (
            "active_layer",
            "command_type",
            "execution_mode",
            "requested_motion_intent",
            "limited_motion_intent",
            "requested_track_reference",
            "track_targets",
            "turn_primitive_requested",
            "turn_primitive_limited",
            "turn_primitive_executed",
            "turn_primitive_actual",
            "mismatch_reason",
            "speed_limiting_reason",
            "safety_limiting_reason",
            "track_reference_mode",
        ),
    )
    primitive_contract = motion_command.get("primitive_contract")
    if isinstance(primitive_contract, dict):
        out["motion_command"]["primitive_contract"] = _copy_keys(
            primitive_contract,
            ("tracking_contract_active", "strict_expected", "chain_match", "mismatch_excused", "violation"),
        )

    motion_public = dict(src.get("motion_public") or {})
    out["motion_public"] = _copy_keys(
        motion_public,
        (
            "source",
            "linear_speed_mps",
            "angular_speed_dps",
            "actual_linear_mps",
            "actual_angular_dps",
            "cmd_linear_mps",
            "cmd_angular_dps",
            "execution_mode",
        ),
    )

    follow_layer = dict(src.get("follow_layer") or {})
    out["follow_layer"] = _copy_keys(
        follow_layer,
        (
            "active",
            "target_source",
            "reason",
            "distance_to_target_m",
            "desired_distance_m",
            "age_s",
            "confidence",
            "stale",
            "target_id",
        ),
    )

    cruise_layer = dict(src.get("cruise_layer") or {})
    room_cruise = dict(cruise_layer.get("room_cruise") or {})
    out["cruise_layer"] = _copy_keys(
        cruise_layer,
        ("active", "reason", "primitive_type", "motion_style", "target_source", "room_cruise_chain"),
    )
    if room_cruise:
        compact_room = _copy_keys(
            room_cruise,
            ("active", "follow_state", "phase", "reason", "selected_side", "side_selection", "track_reference"),
        )
        compact_room["clearance"] = _copy_keys(
            dict(room_cruise.get("clearance") or {}),
            (
                "front_clearance_m",
                "left_clearance_m",
                "right_clearance_m",
                "rear_clearance_m",
                "blocked_front",
                "blocked_back",
                "lidar_confidence",
            ),
        )
        compact_room["follow_gate"] = _copy_keys(
            dict(room_cruise.get("follow_gate") or {}),
            (
                "target_hold_latched",
                "target_hold_heading_align",
                "camera_front_lidar_hold",
                "camera_front_lidar_hold_human_confirmed",
                "front_hold_align_blocked",
                "front_hold_retreat_active",
                "front_hold_retreat_suppressed",
                "front_hold_retreat_track_mps",
                "target_search_active",
                "target_search_rotate_track_mps",
                "target_search_min_front_clearance_m",
                "camera_simple_follow_active",
                "camera_simple_forward_gate_blocked",
                "camera_simple_retreat_gate_blocked",
                "camera_simple_turn_track_mps",
                "camera_target_turn_side",
                "camera_target_turn_side_clearance_m",
                "camera_target_turn_side_blocked",
                "actual_target_distance_m",
                "actual_target_bearing_error_rad",
            ),
        )
        out["cruise_layer"]["room_cruise"] = compact_room
    return out


class TelemetryLogger:
    """
    Kompatibilitási wrapper a régi audit/telemetria hívásokhoz.

    A tényleges írás a UnifiedLogger session backendjén történik.
    """

    def __init__(self, runtime_dir: str, max_file_mb: int = 20):
        self.runtime = Path(runtime_dir)
        self.runtime.mkdir(parents=True, exist_ok=True)
        self.max_bytes = max_file_mb * 1024 * 1024

    def emit_telemetry(self, payload: dict[str, Any]):
        ul = get_unified_logger()
        if ul is None:
            return
        # The developer full-log switch already increases control diagnostics.
        # Persisting the complete status tree as well created duplicate 30-40 KB
        # records. Full status telemetry therefore requires an explicit opt-in.
        full_payload_requested = str(os.environ.get("R2B4_FULL_TELEMETRY_LOG", "0")).strip().lower() in {
            "1",
            "true",
            "yes",
        }
        if full_payload_requested:
            log_payload = dict(payload or {})
            log_payload["telemetry_schema"] = "STATUS_TELEMETRY_FULL_V1"
            log_payload["telemetry_compacted"] = False
        else:
            log_payload = _compact_status_telemetry(payload)
        ul.emit_telemetry(log_payload, module="controller_status")

    def emit_audit(self, event_type: str, source: str, severity: str = "INFO", details: Optional[dict] = None):
        ul = get_unified_logger()
        if ul is None:
            return
        ul.emit_audit(event_type, source, severity=severity, details=details or {})
        if event_type == "EMERGENCY_STOP":
            ul.log_event(CHANNEL_SAFETY, source.lower(), "emergency_stop", details or {}, level=severity)
