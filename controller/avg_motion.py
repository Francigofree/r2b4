#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AVG (Arbitrate-Validate-Gate) motion snapshot helpers.

The goal is to provide a compact, deterministic, agent-friendly view over the
runtime motion stack without replacing existing rich telemetry.
"""

from __future__ import annotations

import hashlib
import json
import math
from typing import Any, Dict


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return float(default)
        return float(value)
    except Exception:
        return float(default)


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return int(default)
        return int(value)
    except Exception:
        return int(default)


def _is_finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def _first_non_empty(*values: Any, default: str = "") -> str:
    for value in values:
        text = str(value or "").strip()
        if text:
            return text
    return str(default)


def _motion_targets(status: Dict[str, Any]) -> Dict[str, float]:
    motion_resolution = dict((status or {}).get("motion_resolution") or {})
    resolved = dict(motion_resolution.get("resolved") or {})
    final_after_shaping = dict(resolved.get("final_after_shaping") or {})
    motion_command = dict((status or {}).get("motion_command") or {})
    limited = dict(motion_command.get("limited_motion_intent") or {})

    final_v = final_after_shaping.get("v_target", limited.get("v", (status or {}).get("v_target", 0.0)))
    final_omega = final_after_shaping.get("omega_target", limited.get("omega", (status or {}).get("omega_target", 0.0)))
    return {
        "v": _safe_float(final_v, 0.0),
        "omega": _safe_float(final_omega, 0.0),
    }


def build_avg_snapshot(status: Dict[str, Any] | None) -> Dict[str, Any]:
    st = dict(status or {})
    motion_resolution = dict(st.get("motion_resolution") or {})
    resolved = dict(motion_resolution.get("resolved") or {})
    motion_command = dict(st.get("motion_command") or {})
    safety = dict(st.get("safety") or {})
    stop_status = dict(st.get("stop_status") or {})
    lidar_odom_status = dict(st.get("lidar_odom_status") or {})

    targets = _motion_targets(st)
    final_v = float(targets["v"])
    final_omega = float(targets["omega"])
    final_zero = abs(final_v) <= 1e-6 and abs(final_omega) <= 1e-6

    resolved_source = _first_non_empty(
        resolved.get("source"),
        motion_command.get("source"),
        st.get("motion_command_source"),
        default="MANUAL",
    )
    resolved_layer = _first_non_empty(
        resolved.get("layer"),
        motion_command.get("active_layer"),
        default="UNKNOWN",
    )
    resolved_type = _first_non_empty(
        resolved.get("command_type"),
        motion_command.get("command_type"),
        default="idle",
    )

    odometry_mode = str(st.get("odometry_mode", "") or "").strip().upper()
    encoder_pose_fusion_active = bool(st.get("encoder_pose_fusion_active", False))
    lidar_latest_age_s = _safe_float(lidar_odom_status.get("latest_age_s"), math.inf)
    lidar_candidate_age_s = _safe_float(lidar_odom_status.get("candidate_age_s"), math.inf)
    lidar_candidate_fresh = bool(lidar_odom_status.get("candidate_available", False)) and math.isfinite(lidar_candidate_age_s) and lidar_candidate_age_s <= 0.5
    lidar_latest_fresh = math.isfinite(lidar_latest_age_s) and lidar_latest_age_s <= 1.5
    lidar_input_fresh = bool(lidar_candidate_fresh or lidar_latest_fresh)

    finite_targets = _is_finite(final_v) and _is_finite(final_omega)
    validation_ok = bool(finite_targets and (odometry_mode != "LIDAR_FIRST" or lidar_input_fresh))
    lidar_mode_pose_path_ok = True

    safety_allow = bool(safety.get("allow", True))
    stop_active = bool(stop_status.get("active", False))
    gate_allow_motion = bool(safety_allow and not stop_active)
    gate_effective_zero = bool((not gate_allow_motion) or final_zero)

    signature_payload = {
        "source": resolved_source,
        "layer": resolved_layer,
        "command_type": resolved_type,
        "motion_target_owner": str(st.get("motion_target_owner", "") or ""),
        "v": round(final_v, 6),
        "omega": round(final_omega, 6),
        "gate_allow_motion": gate_allow_motion,
        "validation_ok": validation_ok,
        "odometry_mode": odometry_mode,
    }
    signature = hashlib.sha1(
        json.dumps(signature_payload, sort_keys=True, ensure_ascii=True).encode("utf-8")
    ).hexdigest()[:16]

    return {
        "version": 1,
        "principle": "AVG",
        "arbitrate": {
            "source": resolved_source,
            "layer": resolved_layer,
            "command_type": resolved_type,
            "motion_target_owner": str(st.get("motion_target_owner", "") or ""),
            "proposal_count": _safe_int(motion_resolution.get("proposal_count"), 0),
            "resolved_mode": str(resolved.get("mode", "") or ""),
        },
        "validate": {
            "ok": validation_ok,
            "finite_targets": bool(finite_targets),
            "odometry_mode": odometry_mode,
            "encoder_pose_fusion_active": bool(encoder_pose_fusion_active),
            "lidar_mode_pose_path_ok": bool(lidar_mode_pose_path_ok),
            "lidar_input_fresh": bool(lidar_input_fresh),
            "lidar_latest_age_s": (None if not math.isfinite(lidar_latest_age_s) else float(lidar_latest_age_s)),
            "lidar_candidate_age_s": (None if not math.isfinite(lidar_candidate_age_s) else float(lidar_candidate_age_s)),
            "final_targets": {
                "v": round(final_v, 6),
                "omega": round(final_omega, 6),
            },
        },
        "gate": {
            "allow_motion": gate_allow_motion,
            "safety_allow": bool(safety_allow),
            "stop_active": bool(stop_active),
            "stop_type": str(stop_status.get("type", "NONE") or "NONE"),
            "service_motion_active": bool(st.get("service_motion_active", False)),
            "effective_zero": gate_effective_zero,
        },
        "determinism": {
            "status_version": _safe_int(st.get("status_version"), 0),
            "signature": signature,
        },
    }
