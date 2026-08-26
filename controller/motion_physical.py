#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Public motion semantics + physical telemetry SSOT.

This module computes operator-facing motion fields in real physical units:
- linear speed: m/s
- angular speed: deg/s
- distance: m
- heading: deg

The computation path is EKF pose/odometry based so command-vs-actual comparisons
stay truthful and consistent across status, telemetry, and test reporting.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

from controller.motion_schema import (
    MOTION_SCHEMA_VERSION,
    classify_motion_layers,
    normalize_execution_mode,
)

RAD_TO_DEG = 57.29577951308232


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _is_finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def _stable_signature_value(value: Any):
    if isinstance(value, dict):
        return tuple(
            (str(key), _stable_signature_value(item))
            for key, item in sorted(value.items(), key=lambda kv: str(kv[0]))
        )
    if isinstance(value, (list, tuple)):
        return tuple(_stable_signature_value(item) for item in value)
    if isinstance(value, bool):
        return bool(value)
    if isinstance(value, int):
        return int(value)
    if isinstance(value, float):
        return float(value)
    if value is None:
        return None
    return str(value)


def _normalize_angle_deg(angle_deg: float) -> float:
    return ((_safe_float(angle_deg, 0.0) + 180.0) % 360.0) - 180.0


def _round_or_none(value: Any, digits: int = 6):
    if value is None:
        return None
    if not _is_finite(value):
        return None
    return round(float(value), int(digits))


def _actual_pivot_corroboration(
    ctrl,
    *,
    execution_mode: str,
    actual_angular_dps: float,
) -> Dict[str, Any]:
    pipeline = dict(getattr(ctrl, "encoder_pipeline_status", {}) or {})
    canonical = dict(pipeline.get("canonical_velocity") or {})
    left_mps = _safe_float(canonical.get("left_mps"), math.nan)
    right_mps = _safe_float(canonical.get("right_mps"), math.nan)
    track_left = _safe_float(getattr(ctrl, "track_target_left_mps", None), math.nan)
    track_right = _safe_float(getattr(ctrl, "track_target_right_mps", None), math.nan)
    track_width_m = max(
        0.01,
        _safe_float(
            getattr(getattr(ctrl, "motion_executor", None), "track_width", 0.175),
            0.175,
        ),
    )
    wheel_available = bool(_is_finite(left_mps) and _is_finite(right_mps))
    command_track_available = bool(_is_finite(track_left) and _is_finite(track_right))
    wheel_linear_mps = (
        0.5 * (float(left_mps) + float(right_mps)) if wheel_available else math.nan
    )
    wheel_omega_rad_s = (
        (float(right_mps) - float(left_mps)) / track_width_m
        if wheel_available
        else math.nan
    )
    actual_omega_rad_s = math.radians(float(actual_angular_dps))
    actual_opposite = bool(
        wheel_available
        and float(left_mps) * float(right_mps) < 0.0
        and min(abs(float(left_mps)), abs(float(right_mps))) >= 0.006
    )
    commanded_opposite = bool(
        command_track_available
        and float(track_left) * float(track_right) < 0.0
        and min(abs(float(track_left)), abs(float(track_right))) >= 0.005
    )
    yaw_sign_consistent = bool(
        _is_finite(wheel_omega_rad_s)
        and abs(float(wheel_omega_rad_s)) >= 0.04
        and abs(float(actual_omega_rad_s)) >= 0.04
        and float(wheel_omega_rad_s) * float(actual_omega_rad_s) > 0.0
    )
    wheel_linear_bounded = bool(
        _is_finite(wheel_linear_mps) and abs(float(wheel_linear_mps)) <= 0.015
    )
    health = str(pipeline.get("snapshot_health", "") or "").strip().upper()
    trust = _safe_float(pipeline.get("combined_trust"), 0.0)
    encoder_reliable = bool(
        health not in ("STALE", "FAIL", "ERROR")
        and trust >= 0.50
        and not bool(pipeline.get("anomaly_active", False))
    )
    applied = bool(
        str(execution_mode or "").strip().upper() == "TRACK_EXEC"
        and commanded_opposite
        and actual_opposite
        and wheel_linear_bounded
        and yaw_sign_consistent
        and encoder_reliable
    )
    return {
        "applied": applied,
        "reason": (
            "ENCODER_OPPOSITE_TRACKS_WITH_YAW_CONFIRMATION"
            if applied
            else "NO_CORROBORATED_PIVOT"
        ),
        "left_mps": _round_or_none(left_mps, 6),
        "right_mps": _round_or_none(right_mps, 6),
        "wheel_linear_mps": _round_or_none(wheel_linear_mps, 6),
        "wheel_omega_rad_s": _round_or_none(wheel_omega_rad_s, 6),
        "actual_omega_rad_s": _round_or_none(actual_omega_rad_s, 6),
        "commanded_opposite": commanded_opposite,
        "actual_opposite": actual_opposite,
        "wheel_linear_bounded": wheel_linear_bounded,
        "yaw_sign_consistent": yaw_sign_consistent,
        "encoder_reliable": encoder_reliable,
        "encoder_trust": _round_or_none(trust, 4),
        "encoder_health": health or "UNKNOWN",
    }


class MotionPhysicalTelemetry:
    """
    Single authoritative runtime source for physical motion telemetry.
    """

    def __init__(self, cfg: Optional[Dict[str, Any]] = None):
        cfg = dict(cfg or {})
        self._max_dt_s = max(0.05, _safe_float(cfg.get("max_dt_s"), 0.30))
        self._last_pose: Optional[Dict[str, float]] = None
        self._last_ts: Optional[float] = None
        self._segment_id = 0
        self._segment_signature = ""
        self._segment_layer = "IDLE"
        self._segment_command_type = "idle"
        self._segment_source = "MANUAL"
        self._segment_start_pose: Optional[Dict[str, float]] = None
        self._segment_start_ts = 0.0
        self._segment_target_distance_m: Optional[float] = None
        self._segment_target_heading_deg: Optional[float] = None
        self._segment_target_pose: Optional[Dict[str, float]] = None
        self._segment_path_m = 0.0
        self._cmd_linear_integral = 0.0
        self._cmd_angular_integral_dps = 0.0
        self._actual_linear_integral = 0.0
        self._actual_angular_integral_dps = 0.0
        self._segment_time_s = 0.0
        self._last_segment_report: Dict[str, Any] = {}
        self._last_status: Dict[str, Any] = {}
        self._last_stop_active = False

    @staticmethod
    def _read_pose(ekf_state: Dict[str, Any]) -> Optional[Dict[str, float]]:
        if not isinstance(ekf_state, dict):
            return None
        x = _safe_float(ekf_state.get("x"), math.nan)
        y = _safe_float(ekf_state.get("y"), math.nan)
        theta_deg = _safe_float(ekf_state.get("theta_deg"), math.nan)
        if not all(_is_finite(v) for v in (x, y, theta_deg)):
            return None
        return {
            "x": float(x),
            "y": float(y),
            "theta_deg": float(theta_deg),
        }

    @staticmethod
    def _command_from_ctrl(ctrl) -> tuple[float, float]:
        execution_mode = str(getattr(ctrl, "motion_execution_mode", "") or "").strip().upper()
        if execution_mode == "TRACK_EXEC":
            track_ref = dict(getattr(ctrl, "requested_track_reference", {}) or {})
            try:
                left = track_ref.get("left_mps")
                right = track_ref.get("right_mps")
                if left is not None and right is not None:
                    left_v = float(left)
                    right_v = float(right)
                    if math.isfinite(left_v) and math.isfinite(right_v):
                        track_width = max(
                            0.01,
                            _safe_float(
                                getattr(getattr(ctrl, "motion_executor", None), "track_width", 0.175),
                                0.175,
                            ),
                        )
                        cmd_linear = 0.5 * (left_v + right_v)
                        cmd_angular_rad_s = (right_v - left_v) / track_width
                        return float(cmd_linear), float(cmd_angular_rad_s)
            except Exception:
                pass
        limited = dict(getattr(ctrl, "limited_motion_intent", {}) or {})
        cmd_linear = _safe_float(
            limited.get("v"),
            _safe_float(getattr(ctrl, "v_target", 0.0), 0.0),
        )
        cmd_angular_rad_s = _safe_float(
            limited.get("omega"),
            _safe_float(getattr(ctrl, "omega_target", 0.0), 0.0),
        )
        return float(cmd_linear), float(cmd_angular_rad_s)

    @staticmethod
    def _extract_target_pose(ctrl, target_payload: Dict[str, Any]) -> Optional[Dict[str, float]]:
        payload = dict(target_payload or {})
        raw_pose = payload.get("target_pose")
        if isinstance(raw_pose, dict):
            x = _safe_float(raw_pose.get("x"), math.nan)
            y = _safe_float(raw_pose.get("y"), math.nan)
            theta_deg = _safe_float(raw_pose.get("theta_deg"), math.nan)
            if all(_is_finite(v) for v in (x, y, theta_deg)):
                return {"x": float(x), "y": float(y), "theta_deg": float(theta_deg)}

        target_pose = getattr(ctrl, "target_pose", None)
        if isinstance(target_pose, (list, tuple)) and len(target_pose) >= 3:
            x = _safe_float(target_pose[0], math.nan)
            y = _safe_float(target_pose[1], math.nan)
            theta_rad = _safe_float(target_pose[2], math.nan)
            if all(_is_finite(v) for v in (x, y, theta_rad)):
                return {
                    "x": float(x),
                    "y": float(y),
                    "theta_deg": float(theta_rad * RAD_TO_DEG),
                }
        return None

    @staticmethod
    def _targets_from_ctrl(ctrl) -> Dict[str, Any]:
        target_payload = dict(getattr(ctrl, "motion_public_target", {}) or {})
        target_pose = MotionPhysicalTelemetry._extract_target_pose(ctrl, target_payload)
        target_heading_deg = target_payload.get("target_heading_deg")
        if not _is_finite(target_heading_deg):
            heading_status = dict(getattr(ctrl, "heading_controller_status", {}) or {})
            hs_target = heading_status.get("target_heading_deg")
            if _is_finite(hs_target):
                target_heading_deg = float(hs_target)
        if not _is_finite(target_heading_deg):
            behavior = dict(getattr(ctrl, "behavior_motion_status", {}) or {})
            bs_target = behavior.get("target_heading_deg")
            if _is_finite(bs_target):
                target_heading_deg = float(bs_target)
        if not _is_finite(target_heading_deg) and isinstance(target_pose, dict):
            target_heading_deg = float(target_pose["theta_deg"])

        target_distance_m = target_payload.get("target_distance_m")
        if not _is_finite(target_distance_m):
            target_distance_m = None

        return {
            "target_distance_m": (None if target_distance_m is None else float(target_distance_m)),
            "target_heading_deg": (
                None if not _is_finite(target_heading_deg) else float(target_heading_deg)
            ),
            "target_pose": target_pose,
        }

    @staticmethod
    def _stop_reason(ctrl) -> str:
        stop_status = dict(getattr(ctrl, "stop_status", {}) or {})
        if bool(stop_status.get("active", False)):
            canonical_reason = str(stop_status.get("canonical_reason", "") or "").strip()
            if canonical_reason:
                return canonical_reason
            reason = str(stop_status.get("reason", "") or "").strip()
            if reason:
                return reason
            stop_type = str(stop_status.get("type", "") or "").strip()
            if stop_type:
                return stop_type
        state_name = ""
        try:
            state_name = str(ctrl.sm.get_current_state_name())
        except Exception:
            state_name = str(getattr(getattr(ctrl, "sm", None), "current_enum", "") or "")
        if state_name.upper() == "FAILSAFE":
            return "FAILSAFE"
        return ""

    @staticmethod
    def _actual_measurement_gate(
        ctrl,
        *,
        cmd_linear_mps: float,
        cmd_angular_dps: float,
        actual_linear_mps: float,
        actual_angular_dps: float,
        segment_age_s: float,
    ) -> Dict[str, Any]:
        quality = dict(getattr(ctrl, "motion_quality_status", {}) or {})
        estimator = dict(quality.get("estimator_consistency") or {})
        quality_state = str(quality.get("quality_state", "") or "").strip().upper()
        localization_mode = str(estimator.get("localization_mode", "") or "").strip().upper()
        gate_reject = bool(estimator.get("enc_gate_reject", False) or estimator.get("lidar_gate_reject", False))
        command_active = bool(
            abs(float(cmd_linear_mps)) > 0.005
            or abs(float(cmd_angular_dps)) > 1.0
        )
        movement_observed = bool(
            abs(float(actual_linear_mps)) > 0.01
            or abs(float(actual_angular_dps)) > 1.2
        )
        ready = bool(
            (not command_active)
            or (float(segment_age_s) >= 0.10 and movement_observed)
        )
        reliable = bool(
            quality_state != "CRITICAL"
            and localization_mode != "LOST"
            and not gate_reject
        )
        reasons = []
        if command_active and not ready:
            reasons.append("MOTION_ONSET_PENDING")
        if quality_state == "CRITICAL":
            reasons.append("MOTION_QUALITY_CRITICAL")
        if localization_mode == "LOST":
            reasons.append("LOCALIZATION_LOST")
        if gate_reject:
            reasons.append("ESTIMATOR_GATE_REJECT")
        return {
            "available": True,
            "ready": bool(ready),
            "reliable": bool(reliable),
            "segment_age_s": _round_or_none(segment_age_s, 6),
            "command_active": bool(command_active),
            "movement_observed": bool(movement_observed),
            "quality_state": str(quality_state or "UNKNOWN"),
            "estimator_confidence": _round_or_none(estimator.get("confidence"), 4),
            "localization_mode": str(localization_mode or "UNKNOWN"),
            "reasons": reasons,
        }

    @staticmethod
    def _signature(
        *,
        layer: str,
        command_type: str,
        source: str,
        cmd_linear_mps: float,
        cmd_angular_dps: float,
        targets: Dict[str, Any],
    ) -> str:
        payload = (
            ("layer", str(layer or "")),
            ("command_type", str(command_type or "")),
            ("source", str(source or "")),
            ("cmd_linear_mps", round(float(cmd_linear_mps), 4)),
            ("cmd_angular_dps", round(float(cmd_angular_dps), 4)),
            (
                "target_distance_m",
                None
                if targets.get("target_distance_m") is None
                else round(float(targets.get("target_distance_m")), 4),
            ),
            (
                "target_heading_deg",
                None
                if targets.get("target_heading_deg") is None
                else round(float(targets.get("target_heading_deg")), 4),
            ),
            ("target_pose", _stable_signature_value(targets.get("target_pose"))),
        )
        return repr(payload)

    def _reset_segment(self, pose: Dict[str, float], now_s: float, targets: Dict[str, Any]) -> None:
        self._segment_id += 1
        self._segment_start_pose = dict(pose)
        self._segment_start_ts = float(now_s)
        self._segment_path_m = 0.0
        self._cmd_linear_integral = 0.0
        self._cmd_angular_integral_dps = 0.0
        self._actual_linear_integral = 0.0
        self._actual_angular_integral_dps = 0.0
        self._segment_time_s = 0.0

        explicit_target_distance = targets.get("target_distance_m")
        if explicit_target_distance is not None and _is_finite(explicit_target_distance):
            self._segment_target_distance_m = float(explicit_target_distance)
        elif isinstance(targets.get("target_pose"), dict):
            target_pose = dict(targets["target_pose"])
            self._segment_target_distance_m = float(
                math.hypot(
                    _safe_float(target_pose.get("x"), 0.0) - float(pose["x"]),
                    _safe_float(target_pose.get("y"), 0.0) - float(pose["y"]),
                )
            )
        else:
            self._segment_target_distance_m = None

        if targets.get("target_heading_deg") is None:
            self._segment_target_heading_deg = None
        else:
            self._segment_target_heading_deg = float(targets["target_heading_deg"])
        self._segment_target_pose = (
            None
            if not isinstance(targets.get("target_pose"), dict)
            else dict(targets["target_pose"])
        )

    def _segment_payload(
        self,
        *,
        now_s: float,
        cmd_linear_mps: float,
        cmd_angular_dps: float,
        actual_linear_mps: float,
        actual_angular_dps: float,
        progress_distance_m: float,
        progress_heading_deg: float,
        stop_reason: str,
    ) -> Dict[str, Any]:
        duration = max(0.0, float(self._segment_time_s))
        cmd_avg_linear = (
            float(self._cmd_linear_integral) / duration if duration > 1e-6 else float(cmd_linear_mps)
        )
        cmd_avg_angular = (
            float(self._cmd_angular_integral_dps) / duration if duration > 1e-6 else float(cmd_angular_dps)
        )
        actual_avg_linear = (
            float(self._actual_linear_integral) / duration if duration > 1e-6 else float(actual_linear_mps)
        )
        actual_avg_angular = (
            float(self._actual_angular_integral_dps) / duration if duration > 1e-6 else float(actual_angular_dps)
        )
        return {
            "segment_id": int(self._segment_id),
            "command_layer": str(getattr(self, "_segment_layer", "")),
            "command_type": str(getattr(self, "_segment_command_type", "")),
            "source": str(getattr(self, "_segment_source", "")),
            "started_ts": _round_or_none(self._segment_start_ts, 6),
            "updated_ts": _round_or_none(now_s, 6),
            "duration_s": _round_or_none(duration, 6),
            "cmd_linear_mps": _round_or_none(cmd_linear_mps, 6),
            "cmd_angular_dps": _round_or_none(cmd_angular_dps, 6),
            "actual_linear_mps": _round_or_none(actual_linear_mps, 6),
            "actual_angular_dps": _round_or_none(actual_angular_dps, 6),
            "segment_target_distance_m": _round_or_none(self._segment_target_distance_m, 6),
            "segment_progress_m": _round_or_none(progress_distance_m, 6),
            "segment_path_m": _round_or_none(self._segment_path_m, 6),
            "segment_target_heading_deg": _round_or_none(self._segment_target_heading_deg, 6),
            "segment_heading_progress_deg": _round_or_none(progress_heading_deg, 6),
            "target_pose": self._segment_target_pose,
            "commanded_average_linear_speed_mps": _round_or_none(cmd_avg_linear, 6),
            "actual_average_linear_speed_mps": _round_or_none(actual_avg_linear, 6),
            "commanded_average_angular_speed_dps": _round_or_none(cmd_avg_angular, 6),
            "actual_average_angular_speed_dps": _round_or_none(actual_avg_angular, 6),
            "stop_reason": str(stop_reason or ""),
        }

    def _finalize_segment(
        self,
        *,
        now_s: float,
        cmd_linear_mps: float,
        cmd_angular_dps: float,
        actual_linear_mps: float,
        actual_angular_dps: float,
        progress_distance_m: float,
        progress_heading_deg: float,
        stop_reason: str,
    ) -> None:
        self._last_segment_report = self._segment_payload(
            now_s=now_s,
            cmd_linear_mps=cmd_linear_mps,
            cmd_angular_dps=cmd_angular_dps,
            actual_linear_mps=actual_linear_mps,
            actual_angular_dps=actual_angular_dps,
            progress_distance_m=progress_distance_m,
            progress_heading_deg=progress_heading_deg,
            stop_reason=stop_reason,
        )

    def update(
        self,
        *,
        ctrl,
        ekf_state: Dict[str, Any],
        now: float,
    ) -> Dict[str, Any]:
        now_s = float(_safe_float(now, 0.0))
        pose = self._read_pose(ekf_state)
        if pose is None:
            return dict(self._last_status or {})

        cmd_linear_mps, cmd_angular_rad_s = self._command_from_ctrl(ctrl)
        cmd_angular_dps = float(cmd_angular_rad_s * RAD_TO_DEG)
        targets = self._targets_from_ctrl(ctrl)

        # Segment identity follows the persisted user/behavior intent, while
        # physical telemetry continues to report the shaped command above.
        # Otherwise small limiter and straight-hold changes reset the onset
        # timer every few ticks and make a continuous command look unready.
        signature_linear_mps = float(cmd_linear_mps)
        signature_angular_rad_s = float(cmd_angular_rad_s)
        requested_intent = getattr(ctrl, "requested_motion_intent", None)
        if isinstance(requested_intent, dict):
            requested_v = _safe_float(requested_intent.get("v"), math.nan)
            requested_omega = _safe_float(requested_intent.get("omega"), math.nan)
            if _is_finite(requested_v) and _is_finite(requested_omega):
                signature_linear_mps = float(requested_v)
                signature_angular_rad_s = float(requested_omega)

        layer = str(getattr(ctrl, "active_motion_command_layer", "IDLE") or "IDLE")
        command_type = str(getattr(ctrl, "active_motion_command_type", "idle") or "idle")
        source = str(
            getattr(ctrl, "active_motion_command_source", getattr(ctrl, "motion_command_source", "MANUAL"))
            or "MANUAL"
        )
        signature = self._signature(
            layer=layer,
            command_type=command_type,
            source=source,
            cmd_linear_mps=signature_linear_mps,
            cmd_angular_dps=float(signature_angular_rad_s * RAD_TO_DEG),
            targets=targets,
        )

        if self._last_pose is None or self._last_ts is None:
            self._last_pose = dict(pose)
            self._last_ts = float(now_s)
            self._segment_signature = signature
            self._segment_layer = layer
            self._segment_command_type = command_type
            self._segment_source = source
            self._reset_segment(pose, now_s, targets)

        dt = max(0.0, float(now_s - float(self._last_ts or now_s)))
        if dt > self._max_dt_s:
            dt = 0.0
        prev_pose = dict(self._last_pose or pose)
        dx = float(pose["x"]) - float(prev_pose["x"])
        dy = float(pose["y"]) - float(prev_pose["y"])
        dtheta_deg = float(_normalize_angle_deg(float(pose["theta_deg"]) - float(prev_pose["theta_deg"])))
        ds = float(math.hypot(dx, dy))

        pose_linear_mps = 0.0
        pose_angular_dps = 0.0
        if dt > 1e-6:
            heading_rad = math.radians(float(prev_pose["theta_deg"]))
            forward_proj = (dx * math.cos(heading_rad)) + (dy * math.sin(heading_rad))
            if abs(forward_proj) <= 1e-9:
                pose_linear_mps = 0.0
            else:
                pose_linear_mps = math.copysign(ds / dt, forward_proj)
            pose_angular_dps = dtheta_deg / dt

        ekf_linear_mps = _safe_float(ekf_state.get("v"), math.nan)
        if not _is_finite(ekf_linear_mps):
            ekf_linear_mps = pose_linear_mps
        omega_rad_s = _safe_float(ekf_state.get("omega_rad_s"), math.nan)
        if _is_finite(omega_rad_s):
            actual_angular_dps = float(omega_rad_s * RAD_TO_DEG)
        else:
            actual_angular_dps = float(pose_angular_dps)
        actual_linear_mps = float(ekf_linear_mps)

        if signature != self._segment_signature:
            if self._segment_start_pose is not None:
                prev_progress_m = float(
                    math.hypot(
                        float(prev_pose["x"]) - float(self._segment_start_pose["x"]),
                        float(prev_pose["y"]) - float(self._segment_start_pose["y"]),
                    )
                )
                prev_heading_deg = float(
                    _normalize_angle_deg(
                        float(prev_pose["theta_deg"]) - float(self._segment_start_pose["theta_deg"])
                    )
                )
                self._finalize_segment(
                    now_s=now_s,
                    cmd_linear_mps=cmd_linear_mps,
                    cmd_angular_dps=cmd_angular_dps,
                    actual_linear_mps=actual_linear_mps,
                    actual_angular_dps=actual_angular_dps,
                    progress_distance_m=prev_progress_m,
                    progress_heading_deg=prev_heading_deg,
                    stop_reason=self._stop_reason(ctrl) or "SEGMENT_SWITCH",
                )
            self._segment_signature = signature
            self._segment_layer = layer
            self._segment_command_type = command_type
            self._segment_source = source
            self._reset_segment(pose, now_s, targets)

        if dt > 1e-6:
            self._segment_time_s += float(dt)
            self._segment_path_m += float(ds)
            self._cmd_linear_integral += float(cmd_linear_mps) * float(dt)
            self._cmd_angular_integral_dps += float(cmd_angular_dps) * float(dt)
            self._actual_linear_integral += float(actual_linear_mps) * float(dt)
            self._actual_angular_integral_dps += float(actual_angular_dps) * float(dt)

        if self._segment_start_pose is None:
            self._segment_start_pose = dict(pose)

        progress_distance_m = float(
            math.hypot(
                float(pose["x"]) - float(self._segment_start_pose["x"]),
                float(pose["y"]) - float(self._segment_start_pose["y"]),
            )
        )
        progress_heading_deg = float(
            _normalize_angle_deg(
                float(pose["theta_deg"]) - float(self._segment_start_pose["theta_deg"])
            )
        )
        stop_reason = self._stop_reason(ctrl)
        stop_active = bool(stop_reason)
        segment_payload = self._segment_payload(
            now_s=now_s,
            cmd_linear_mps=cmd_linear_mps,
            cmd_angular_dps=cmd_angular_dps,
            actual_linear_mps=actual_linear_mps,
            actual_angular_dps=actual_angular_dps,
            progress_distance_m=progress_distance_m,
            progress_heading_deg=progress_heading_deg,
            stop_reason=(stop_reason if stop_active else ""),
        )
        if stop_active and not self._last_stop_active:
            self._last_segment_report = dict(segment_payload)
        self._last_stop_active = bool(stop_active)

        out = {
            "source": "EKF_POSE_ODOMETRY_SSOT",
            "linear_speed_mps": _round_or_none(cmd_linear_mps, 6),
            "angular_speed_dps": _round_or_none(cmd_angular_dps, 6),
            "target_distance_m": _round_or_none(self._segment_target_distance_m, 6),
            "target_heading_deg": _round_or_none(self._segment_target_heading_deg, 6),
            "target_pose": self._segment_target_pose,
            "actual_linear_mps": _round_or_none(actual_linear_mps, 6),
            "actual_angular_dps": _round_or_none(actual_angular_dps, 6),
            "ekf_linear_mps": _round_or_none(ekf_linear_mps, 6),
            "pose_linear_mps": _round_or_none(pose_linear_mps, 6),
            "pose_angular_dps": _round_or_none(pose_angular_dps, 6),
            "segment_age_s": _round_or_none(self._segment_time_s, 6),
            "progress_distance_m": _round_or_none(progress_distance_m, 6),
            "progress_heading_deg": _round_or_none(progress_heading_deg, 6),
            "cmd_linear_mps": _round_or_none(cmd_linear_mps, 6),
            "cmd_angular_dps": _round_or_none(cmd_angular_dps, 6),
            "segment_target_distance_m": _round_or_none(self._segment_target_distance_m, 6),
            "segment_progress_m": _round_or_none(progress_distance_m, 6),
            "segment_target_heading_deg": _round_or_none(self._segment_target_heading_deg, 6),
            "segment_heading_progress_deg": _round_or_none(progress_heading_deg, 6),
            "commanded_average_linear_speed_mps": segment_payload.get("commanded_average_linear_speed_mps"),
            "actual_average_linear_speed_mps": segment_payload.get("actual_average_linear_speed_mps"),
            "commanded_average_angular_speed_dps": segment_payload.get("commanded_average_angular_speed_dps"),
            "actual_average_angular_speed_dps": segment_payload.get("actual_average_angular_speed_dps"),
            "stop_reason": str(stop_reason or ""),
            "segment": segment_payload,
            "last_segment_report": dict(self._last_segment_report or {}),
        }
        execution_mode = normalize_execution_mode(
            getattr(ctrl, "motion_execution_mode", ""),
            fallback="IDLE_EXEC",
        )
        pivot_corroboration = _actual_pivot_corroboration(
            ctrl,
            execution_mode=execution_mode,
            actual_angular_dps=actual_angular_dps,
        )
        actual_linear_for_primitive_mps = (
            0.0
            if bool(pivot_corroboration.get("applied", False))
            else float(actual_linear_mps)
        )
        actual_measurement_gate = self._actual_measurement_gate(
            ctrl,
            cmd_linear_mps=cmd_linear_mps,
            cmd_angular_dps=cmd_angular_dps,
            actual_linear_mps=actual_linear_for_primitive_mps,
            actual_angular_dps=actual_angular_dps,
            segment_age_s=self._segment_time_s,
        )
        turn_semantics = classify_motion_layers(
            track_width_m=max(0.01, float(getattr(getattr(ctrl, "motion_executor", None), "track_width", 0.175))),
            requested_motion_intent=dict(getattr(ctrl, "requested_motion_intent", {}) or {}),
            limited_motion_intent=dict(getattr(ctrl, "limited_motion_intent", {}) or {}),
            requested_track_reference=dict(getattr(ctrl, "requested_track_reference", {}) or {}),
            executed_track_reference={
                "left_mps": getattr(ctrl, "track_target_left_mps", None),
                "right_mps": getattr(ctrl, "track_target_right_mps", None),
            },
            actual_linear_mps=actual_linear_for_primitive_mps,
            actual_angular_dps=actual_angular_dps,
            execution_mode=execution_mode,
            actual_measurement_ready=bool(actual_measurement_gate.get("ready", False)),
            actual_measurement_reliable=bool(actual_measurement_gate.get("reliable", False)),
        )
        actual_turn = dict(turn_semantics.get("actual") or {})
        out["motion_schema_version"] = MOTION_SCHEMA_VERSION
        out["execution_mode"] = execution_mode
        out["turn_semantics"] = turn_semantics
        out["turn_primitive_actual"] = str(
            (actual_turn.get("turn_primitive") or "UNKNOWN")
        )
        out["turn_primitive_actual_raw"] = str(
            actual_turn.get("raw_turn_primitive", actual_turn.get("turn_primitive", "UNKNOWN")) or "UNKNOWN"
        )
        out["actual_measurement_available"] = bool(actual_turn.get("measurement_available", False))
        out["actual_measurement_ready"] = bool(actual_turn.get("measurement_ready", False))
        out["actual_measurement_reliable"] = bool(actual_turn.get("measurement_reliable", False))
        out["actual_measurement_gate"] = actual_measurement_gate
        out["actual_linear_for_primitive_mps"] = _round_or_none(
            actual_linear_for_primitive_mps,
            6,
        )
        out["actual_primitive_corroboration"] = pivot_corroboration
        self._last_pose = dict(pose)
        self._last_ts = float(now_s)
        self._last_status = out
        return dict(out)
