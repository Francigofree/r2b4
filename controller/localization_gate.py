#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Localization-to-motion gate.

Uses EKF-applied localization freshness as the control truth surface.
"""

from __future__ import annotations

import math
from typing import Any, Dict

from controller.motion_kinematics import track_velocity_to_twist


IDLE_STATIONARY_RESUME_BRIDGE_S = 3.0


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return float(default)
        return float(value)
    except Exception:
        return float(default)


def _clamp(value: float, low: float, high: float) -> float:
    return max(float(low), min(float(high), float(value)))


def _is_finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def default_gate_config() -> Dict[str, float | bool]:
    return {
        "enabled": True,
        "degraded_grace_s": 1.25,
        "degraded_speed_scale": 0.55,
        "ekf_gap_warn_s": 0.85,
        "ekf_gap_warn_speed_scale": 0.70,
        "ekf_gap_hard_fail_s": 1.20,
        "idle_stationary_resume_speed_scale": 0.40,
        "lost_hard_stop_while_moving": True,
    }


def resolve_gate_config(cfg: Dict[str, Any] | None) -> Dict[str, float | bool]:
    merged = dict(default_gate_config())
    src = dict(cfg or {})
    merged["enabled"] = bool(src.get("enabled", merged["enabled"]))
    merged["degraded_grace_s"] = max(0.1, _safe_float(src.get("degraded_grace_s"), merged["degraded_grace_s"]))
    merged["degraded_speed_scale"] = min(
        1.0,
        max(0.05, _safe_float(src.get("degraded_speed_scale"), merged["degraded_speed_scale"])),
    )
    merged["ekf_gap_warn_s"] = max(0.05, _safe_float(src.get("ekf_gap_warn_s"), merged["ekf_gap_warn_s"]))
    merged["ekf_gap_warn_speed_scale"] = min(
        1.0,
        max(0.05, _safe_float(src.get("ekf_gap_warn_speed_scale"), merged["ekf_gap_warn_speed_scale"])),
    )
    merged["ekf_gap_hard_fail_s"] = max(
        float(merged["ekf_gap_warn_s"]),
        _safe_float(src.get("ekf_gap_hard_fail_s"), merged["ekf_gap_hard_fail_s"]),
    )
    merged["idle_stationary_resume_speed_scale"] = min(
        1.0,
        max(0.05, _safe_float(src.get("idle_stationary_resume_speed_scale"), merged["idle_stationary_resume_speed_scale"])),
    )
    merged["lost_hard_stop_while_moving"] = bool(
        src.get("lost_hard_stop_while_moving", merged["lost_hard_stop_while_moving"])
    )
    return merged


def _resolved_localization_mode(localization_health: str) -> str:
    health = str(localization_health or "").strip().upper()
    if health in ("TRACKING", "LOCALIZED"):
        return "TRACKING"
    if health in ("DEGRADED", "RELOCALIZING", "RELOCALIZED"):
        return "DEGRADED"
    if health == "LOST":
        return "LOST"
    return "DEGRADED"


def _extract_ekf_gap_s(lidar_odom_status: Dict[str, Any]) -> float:
    status = dict(lidar_odom_status or {})
    direct = status.get("ekf_applied_gap_s")
    if _is_finite(direct):
        return max(0.0, _safe_float(direct, 0.0))
    watchdog = dict(status.get("ekf_applied_gap_watchdog") or {})
    watchdog_gap = watchdog.get("current_gap_s")
    if _is_finite(watchdog_gap):
        return max(0.0, _safe_float(watchdog_gap, 0.0))
    return math.inf


def evaluate_localization_gate(
    *,
    lidar_odom_status: Dict[str, Any] | None,
    now_s: float,
    moving_command: bool,
    runtime_state: Dict[str, Any] | None,
    cfg: Dict[str, Any] | None,
) -> Dict[str, Any]:
    config = resolve_gate_config(cfg)
    state = dict(runtime_state or {})
    status = dict(lidar_odom_status or {})
    pose_reset = dict(state.get("pose_reset") or {})

    if bool(pose_reset.get("in_progress", False)) or str(pose_reset.get("state", "")).upper() == "FAILED":
        reset_failed = str(pose_reset.get("state", "")).upper() == "FAILED"
        return {
            "enabled": True,
            "mode": "RESET_FAILED" if reset_failed else "RESETTING",
            "trust": 0.0,
            "allow_motion": False,
            "speed_scale": 0.0,
            "hard_stop": True,
            "reasons": ["pose_reset_failed" if reset_failed else "pose_reset_in_progress"],
            "ekf_applied_gap_s": None,
            "degraded_elapsed_s": 0.0,
            "state_transition": True,
            "raw_localization_health": str(status.get("localization_health", "") or ""),
            "root_cause": "pose_reset_failed" if reset_failed else "pose_reset_in_progress",
            "runtime_state": state,
        }

    if not bool(config.get("enabled", True)):
        return {
            "enabled": False,
            "mode": "DISABLED",
            "trust": 1.0,
            "allow_motion": True,
            "speed_scale": 1.0,
            "hard_stop": False,
            "reasons": [],
            "ekf_applied_gap_s": None,
            "degraded_elapsed_s": 0.0,
            "state_transition": False,
            "raw_localization_health": str(status.get("localization_health", "") or ""),
            "runtime_state": state,
        }

    raw_health = str(status.get("localization_health", "") or "")
    mode = _resolved_localization_mode(raw_health)
    ekf_gap_s = _extract_ekf_gap_s(status)
    raw_reason = str(status.get("localization_health_reason", "") or "").strip().lower()
    delivery_status = str(status.get("delivery_status", "") or "").strip().lower()
    recent_apply_available = bool(status.get("recent_apply_available", False))
    cadence_soft_reapply = bool(status.get("cadence_soft_reapply", False))
    candidate_available = bool(status.get("candidate_available", False))
    delivery_missing_grace_active = bool(
        status.get("delivery_missing_grace_active", False)
        or status.get("delivery_missing_grace_window_active", False)
    )
    candidate_age_s = _safe_float(status.get("candidate_age_s"), math.inf)
    latest_age_s = _safe_float(status.get("latest_age_s"), math.inf)
    recent_apply_grace_s = max(0.15, _safe_float(status.get("recent_apply_grace_s"), 0.70))
    raw_scan_fresh = False
    try:
        raw_scan_age_s = _safe_float(status.get("raw_scan_latest_age_s"), math.inf)
        max_scan_age_s = max(0.12, _safe_float(status.get("max_scan_age_s"), 0.25))
        raw_scan_fresh = bool(_is_finite(raw_scan_age_s) and raw_scan_age_s <= max(0.25, max_scan_age_s))
    except Exception:
        raw_scan_fresh = False
    flow = dict(status.get("control_loop_lidar_flow") or {})
    current_ekf_applied = bool(
        status.get("applied", False)
        or str(status.get("ekf_status", "") or "").strip().lower() == "applied"
        or str(flow.get("ekf_status", "") or "").strip().lower() == "applied"
    )
    idle_guard_status = dict(status.get("idle_stationary_guard") or {})
    idle_stationary_guard_active = bool(
        status.get("idle_stationary_guard_active", False)
        or bool(idle_guard_status.get("active", False))
        or bool(flow.get("idle_stationary_guard_active", False))
        or str(status.get("control_loop_lidar_apply_status", "") or "").strip().lower()
        == "rejected_idle_stationary_guard"
        or str(status.get("ekf_status", "") or "").strip().lower() == "rejected_idle_stationary_guard"
        or raw_reason == "delivery_missing_idle_stationary_guard"
    )
    latest_candidate_recent = bool(
        bool(candidate_available)
        and _is_finite(candidate_age_s)
        and float(candidate_age_s) <= float(max(0.25, recent_apply_grace_s))
    )
    confidence = _safe_float(status.get("confidence"), 0.0)
    min_confidence = max(0.10, _safe_float(status.get("min_confidence"), 0.20))
    idle_resume_reason_ok = bool(
        str(delivery_status) == "available"
        or raw_reason
        in {
            "delivery_missing_idle_stationary_guard",
            "relocalization_in_progress",
            "localization_degraded",
        }
        or str(status.get("localization_status", "") or "").strip().lower()
        in ("tracking", "localized", "relocalized")
    )
    idle_stationary_gap_recoverable = bool(
        bool(idle_stationary_guard_active)
        and bool(raw_scan_fresh)
        and bool(latest_candidate_recent)
        and bool(idle_resume_reason_ok)
        and (float(confidence) <= 0.0 or float(confidence) >= float(min_confidence))
    )
    idle_resume_bridge_until_ts = _safe_float(state.get("idle_stationary_resume_bridge_until_ts"), 0.0)
    if bool(current_ekf_applied):
        idle_resume_bridge_until_ts = 0.0
    if bool(idle_stationary_gap_recoverable):
        idle_resume_bridge_until_ts = max(
            float(idle_resume_bridge_until_ts),
            float(now_s) + float(IDLE_STATIONARY_RESUME_BRIDGE_S),
        )
    idle_resume_bridge_active = bool(
        float(idle_resume_bridge_until_ts) > 0.0
        and float(now_s) <= float(idle_resume_bridge_until_ts)
    )
    recover_gap_s = min(float(config["ekf_gap_warn_s"]) * 0.75, 0.70)
    delivery_missing_soft_recoverable = bool(
        delivery_status in ("missing", "stale")
        and bool(recent_apply_available)
        and bool(cadence_soft_reapply or delivery_missing_grace_active or raw_scan_fresh)
        and _is_finite(ekf_gap_s)
        and float(ekf_gap_s) <= float(recover_gap_s)
    )
    degraded_recoverable = bool(mode == "DEGRADED" and delivery_missing_soft_recoverable)
    lost_stationary_delivery_recoverable = bool(
        mode == "LOST"
        and (not bool(moving_command))
        and raw_reason == "delivery_missing_hard_timeout"
        and delivery_missing_soft_recoverable
    )

    # delivery_missing_hard_timeout softening window:
    # if raw scan is fresh and scan-match is fresh/recent and EKF gap is not dangerous,
    # prefer short DEGRADED window before LOST hard stop while moving.
    hard_timeout_soft_window_s = 0.22
    hard_timeout_soft_window_ticks_max = 3
    hard_timeout_repeat_block_window_s = 1.20
    hard_timeout_repeat_streak_limit = 1
    hard_timeout_soft_started_ts = _safe_float(
        state.get("delivery_missing_hard_timeout_soft_started_ts"), 0.0
    )
    hard_timeout_soft_window_ticks = int(
        max(0.0, _safe_float(state.get("delivery_missing_hard_timeout_soft_window_ticks"), 0.0))
    )
    hard_timeout_soft_consumed = bool(
        state.get("delivery_missing_hard_timeout_soft_consumed", False)
    )
    hard_timeout_repeat_streak = int(
        max(0.0, _safe_float(state.get("delivery_missing_hard_timeout_repeat_streak"), 0.0))
    )
    hard_timeout_last_end_ts = _safe_float(
        state.get("delivery_missing_hard_timeout_last_end_ts"), 0.0
    )
    hard_timeout_mode_active = bool(
        mode == "LOST"
        and raw_reason == "delivery_missing_hard_timeout"
        and delivery_status == "missing"
    )
    scan_match_fresh_or_recent = bool(
        bool(recent_apply_available)
        or bool(cadence_soft_reapply)
        or (
            bool(candidate_available)
            and _is_finite(candidate_age_s)
            and float(candidate_age_s) <= float(max(0.25, recent_apply_grace_s))
        )
    )
    ekf_gap_not_dangerous = bool(
        _is_finite(ekf_gap_s)
        and float(ekf_gap_s) <= float(config["ekf_gap_warn_s"])
    )
    hard_timeout_soft_candidate = bool(
        hard_timeout_mode_active
        and bool(raw_scan_fresh)
        and bool(scan_match_fresh_or_recent)
        and bool(ekf_gap_not_dangerous)
    )
    hard_timeout_soft_window_active = False
    hard_timeout_repeat_blocked = False
    if hard_timeout_mode_active and hard_timeout_soft_candidate:
        elapsed_since_last_end_s = (
            math.inf
            if hard_timeout_last_end_ts <= 0.0
            else max(0.0, float(now_s) - float(hard_timeout_last_end_ts))
        )
        hard_timeout_repeat_blocked = bool(
            int(hard_timeout_repeat_streak) >= int(hard_timeout_repeat_streak_limit)
            and float(elapsed_since_last_end_s) <= float(hard_timeout_repeat_block_window_s)
        )
        soft_window_live = bool(
            hard_timeout_soft_started_ts > 0.0
            and hard_timeout_soft_window_ticks < int(hard_timeout_soft_window_ticks_max)
            and (float(now_s) - float(hard_timeout_soft_started_ts)) <= float(hard_timeout_soft_window_s)
        )
        if soft_window_live:
            hard_timeout_soft_window_ticks += 1
            hard_timeout_soft_window_active = True
            mode = "DEGRADED"
        elif (not bool(hard_timeout_soft_consumed)) and (not bool(hard_timeout_repeat_blocked)):
            hard_timeout_soft_started_ts = float(now_s)
            hard_timeout_soft_window_ticks = 1
            hard_timeout_soft_consumed = True
            hard_timeout_soft_window_active = True
            mode = "DEGRADED"

    if hard_timeout_mode_active and mode == "LOST":
        if hard_timeout_soft_started_ts > 0.0 and not bool(hard_timeout_soft_window_active):
            hard_timeout_soft_started_ts = 0.0
            hard_timeout_soft_window_ticks = 0
            hard_timeout_soft_consumed = True
            hard_timeout_repeat_streak = min(5, int(hard_timeout_repeat_streak) + 1)
            hard_timeout_last_end_ts = float(now_s)
    elif not hard_timeout_mode_active:
        hard_timeout_soft_started_ts = 0.0
        hard_timeout_soft_window_ticks = 0
        hard_timeout_soft_consumed = False
        if mode == "TRACKING" or delivery_status == "available":
            hard_timeout_repeat_streak = 0

    if degraded_recoverable:
        mode = "TRACKING"
    elif lost_stationary_delivery_recoverable:
        mode = "DEGRADED"

    idle_resume_bridge_recoverable = bool(
        not bool(idle_stationary_gap_recoverable)
        and bool(idle_resume_bridge_active)
        and bool(moving_command)
        and mode == "DEGRADED"
        and bool(raw_scan_fresh)
        and bool(latest_candidate_recent)
        and (float(confidence) <= 0.0 or float(confidence) >= float(min_confidence))
    )

    degraded_started_ts = _safe_float(state.get("degraded_started_ts"), 0.0)
    if mode == "DEGRADED":
        if degraded_started_ts <= 0.0:
            degraded_started_ts = float(now_s)
    else:
        degraded_started_ts = 0.0
    degraded_elapsed_s = 0.0 if degraded_started_ts <= 0.0 else max(0.0, float(now_s) - float(degraded_started_ts))

    reasons = []
    speed_scale = 1.0
    allow_motion = True
    hard_stop = False
    if degraded_recoverable:
        reasons.append("degraded_recovered_to_tracking")
    if lost_stationary_delivery_recoverable:
        reasons.append("lost_stationary_delivery_softened_to_degraded")
    if current_ekf_applied:
        reasons.append("ekf_applied_current_recovered")
    if idle_stationary_gap_recoverable:
        reasons.append("idle_stationary_guard_resume")
    if idle_resume_bridge_recoverable:
        reasons.append("idle_stationary_resume_bridge")
    if hard_timeout_soft_window_active:
        reasons.append("lost_delivery_missing_hard_timeout_soft_window")
    if hard_timeout_repeat_blocked:
        reasons.append("lost_delivery_missing_hard_timeout_repeat_blocked")
    if raw_reason:
        reasons.append(f"health_reason:{raw_reason}")

    if mode == "DEGRADED":
        reasons.append("localization_degraded")
        speed_scale = min(speed_scale, float(config["degraded_speed_scale"]))
        if float(degraded_elapsed_s) > float(config["degraded_grace_s"]):
            allow_motion = False
            reasons.append("degraded_timeout")

    if idle_stationary_gap_recoverable or idle_resume_bridge_recoverable:
        allow_motion = True
        hard_stop = False
        speed_scale = min(speed_scale, float(config["idle_stationary_resume_speed_scale"]))

    if mode == "LOST":
        reasons.append("localization_lost")
        allow_motion = False
        hard_stop = bool(config.get("lost_hard_stop_while_moving", True) and bool(moving_command))
        reasons.append("lost_motion_not_allowed")
        if hard_stop:
            reasons.append("lost_while_moving_hard_stop")

    if _is_finite(ekf_gap_s):
        if bool(current_ekf_applied):
            pass
        elif bool(idle_stationary_gap_recoverable or idle_resume_bridge_recoverable):
            speed_scale = min(speed_scale, float(config["idle_stationary_resume_speed_scale"]))
        elif float(ekf_gap_s) > float(config["ekf_gap_hard_fail_s"]):
            allow_motion = False
            hard_stop = True
            reasons.append("ekf_applied_gap_hard_fail")
        elif float(ekf_gap_s) > float(config["ekf_gap_warn_s"]):
            speed_scale = min(speed_scale, float(config["ekf_gap_warn_speed_scale"]))
            reasons.append("ekf_applied_gap_warn")

    if not bool(allow_motion):
        speed_scale = 0.0

    last_mode = str(state.get("last_mode", "") or "")
    state_transition = str(last_mode) != str(mode)
    state["degraded_started_ts"] = float(degraded_started_ts)
    state["last_mode"] = str(mode)
    state["delivery_missing_hard_timeout_soft_started_ts"] = float(hard_timeout_soft_started_ts)
    state["delivery_missing_hard_timeout_soft_window_ticks"] = int(hard_timeout_soft_window_ticks)
    state["delivery_missing_hard_timeout_soft_consumed"] = bool(hard_timeout_soft_consumed)
    state["delivery_missing_hard_timeout_repeat_streak"] = int(hard_timeout_repeat_streak)
    state["delivery_missing_hard_timeout_last_end_ts"] = float(hard_timeout_last_end_ts)
    state["idle_stationary_resume_bridge_until_ts"] = float(idle_resume_bridge_until_ts)

    confidence_for_truth = _safe_float(status.get("confidence"), 0.0)
    if mode == "TRACKING":
        localization_trust = _clamp(confidence_for_truth if confidence_for_truth > 0.0 else 0.75, 0.55, 1.0)
    elif mode == "DEGRADED":
        localization_trust = _clamp(confidence_for_truth if confidence_for_truth > 0.0 else 0.35, 0.0, 0.50)
    else:
        localization_trust = 0.0

    return {
        "enabled": True,
        "mode": str(mode),
        "trust": float(localization_trust),
        "allow_motion": bool(allow_motion),
        "speed_scale": float(speed_scale),
        "hard_stop": bool(hard_stop),
        "reasons": list(dict.fromkeys(str(reason) for reason in reasons if reason)),
        "ekf_applied_gap_s": (None if not _is_finite(ekf_gap_s) else float(ekf_gap_s)),
        "degraded_elapsed_s": float(degraded_elapsed_s),
        "state_transition": bool(state_transition),
        "raw_localization_health": str(raw_health),
        "root_cause": str(raw_reason),
        "delivery_status": str(delivery_status),
        "degraded_recoverable": bool(degraded_recoverable),
        "current_ekf_applied": bool(current_ekf_applied),
        "idle_stationary_guard_active": bool(idle_stationary_guard_active),
        "idle_stationary_gap_recoverable": bool(idle_stationary_gap_recoverable),
        "idle_stationary_resume_bridge_active": bool(idle_resume_bridge_active),
        "idle_stationary_resume_bridge_recoverable": bool(idle_resume_bridge_recoverable),
        "idle_stationary_resume_bridge_remaining_s": round(
            max(0.0, float(idle_resume_bridge_until_ts) - float(now_s)),
            3,
        ),
        "runtime_state": state,
    }


def apply_localization_gate_to_command(
    *,
    v_target: float,
    omega_target: float,
    execution_mode: str,
    requested_track_reference: Dict[str, Any] | None,
    gate_status: Dict[str, Any] | None,
    track_width_m: float,
) -> Dict[str, Any]:
    gate = dict(gate_status or {})
    out = {
        "applied": False,
        "reason": "none",
        "v_target": float(v_target),
        "omega_target": float(omega_target),
        "requested_track_reference": dict(requested_track_reference or {}),
    }
    if not bool(gate.get("enabled", False)):
        return out

    allow_motion = bool(gate.get("allow_motion", True))
    speed_scale = max(0.0, min(1.0, _safe_float(gate.get("speed_scale"), 1.0)))

    track_ref = dict(requested_track_reference or {})
    left = track_ref.get("left_mps")
    right = track_ref.get("right_mps")
    track_ref_valid = False
    if _is_finite(left) and _is_finite(right):
        left = float(left)
        right = float(right)
        track_ref_valid = True

    if not allow_motion:
        out["applied"] = True
        out["reason"] = "localization_gate_stop"
        out["v_target"] = 0.0
        out["omega_target"] = 0.0
        if track_ref_valid:
            out["requested_track_reference"] = {"left_mps": 0.0, "right_mps": 0.0}
        return out

    if speed_scale >= 0.999:
        return out

    out["applied"] = True
    out["reason"] = "localization_gate_speed_limit"
    if str(execution_mode or "").strip().upper() in ("TRACK_EXEC", "HEADING_EXEC") and track_ref_valid:
        left_scaled = float(left) * float(speed_scale)
        right_scaled = float(right) * float(speed_scale)
        v_scaled, omega_scaled = track_velocity_to_twist(
            float(left_scaled),
            float(right_scaled),
            float(track_width_m),
        )
        out["requested_track_reference"] = {
            "left_mps": float(left_scaled),
            "right_mps": float(right_scaled),
        }
        out["v_target"] = float(v_scaled)
        out["omega_target"] = float(omega_scaled)
        return out

    out["v_target"] = float(v_target) * float(speed_scale)
    out["omega_target"] = float(omega_target) * float(speed_scale)
    return out
