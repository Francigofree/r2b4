#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""V2.1 L11 motion-quality diagnostics with no control authority."""

from __future__ import annotations

import math
from collections import deque
from typing import Any, Dict, Optional


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _normalize_angle_deg(angle_deg: float) -> float:
    angle = _safe_float(angle_deg, 0.0)
    return ((angle + 180.0) % 360.0) - 180.0


def _normalize_control_mode(mode: Any) -> str:
    return str(mode or "").strip().upper()


class MotionQAMonitor:
    """
    First-class motion quality telemetry model.
    Produces deterministic per-tick quality indicators and degradation causes.
    """

    def __init__(self, cfg: Optional[Dict[str, Any]] = None):
        cfg = dict(cfg or {})
        self.stop_velocity_threshold = _safe_float(cfg.get("stop_velocity_threshold"), 0.03)
        self.degraded_dt_threshold = _safe_float(cfg.get("degraded_dt_threshold"), 0.06)
        self.velocity_stability_window = max(8, int(cfg.get("velocity_stability_window", 40)))
        stop_scales = dict(cfg.get("stop_velocity_threshold_scales") or {})
        self._stop_velocity_scales = {
            "UNIFIED": max(0.5, _safe_float(stop_scales.get("UNIFIED"), 0.9)),
        }
        self._last_pose: Optional[Dict[str, float]] = None
        self._segment_state: Optional[str] = None
        self._segment_start: Optional[Dict[str, float]] = None
        self._segment_path = 0.0
        self._segment_yaw = 0.0
        self._segment_opposite_count = 0
        self._segment_sample_count = 0
        self._segment_symmetry_acc = 0.0
        self._stop_command_ts: Optional[float] = None
        self._stop_settling_time: Optional[float] = None
        self._linear_velocity_window: deque = deque(maxlen=self.velocity_stability_window)
        self._stop_residual_window: deque = deque(maxlen=self.velocity_stability_window)
        self._last_output: Dict[str, Any] = {}

    def _switch_segment(self, state: str, pose: Dict[str, float]) -> None:
        self._segment_state = state
        self._segment_start = dict(pose)
        self._segment_path = 0.0
        self._segment_yaw = 0.0
        self._segment_opposite_count = 0
        self._segment_sample_count = 0
        self._segment_symmetry_acc = 0.0

    def update(
        self,
        *,
        semantic_status: Dict[str, Any],
        ekf_state: Dict[str, Any],
        v_target: float,
        omega_target: float,
        v_cmd: float,
        v_l_raw: float,
        v_r_raw: float,
        pwm_l: float,
        pwm_r: float,
        dt: float,
        now: float,
        encoder_reliability: Optional[Dict[str, Any]] = None,
        safety_state: Optional[Dict[str, Any]] = None,
        motion_source: str = "UNKNOWN",
        command_overlap: Optional[Dict[str, Any]] = None,
        heading_controller_status: Optional[Dict[str, Any]] = None,
        control_mode: str = "UNIFIED",
        localization_gate_status: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        encoder_reliability = dict(encoder_reliability or {})
        safety_state = dict(safety_state or {})
        command_overlap = dict(command_overlap or {})
        heading_controller_status = dict(heading_controller_status or {})
        localization_gate_status = dict(localization_gate_status or {})
        mode_name = _normalize_control_mode(control_mode)

        pose = {
            "x": _safe_float(ekf_state.get("x"), 0.0),
            "y": _safe_float(ekf_state.get("y"), 0.0),
            "theta_deg": _safe_float(ekf_state.get("theta_deg"), 0.0),
        }

        if self._last_pose is None:
            self._last_pose = dict(pose)

        dx = pose["x"] - self._last_pose["x"]
        dy = pose["y"] - self._last_pose["y"]
        ds = math.hypot(dx, dy)
        dyaw = _normalize_angle_deg(pose["theta_deg"] - self._last_pose["theta_deg"])
        self._last_pose = dict(pose)

        semantic_state = str(semantic_status.get("semantic_state", "UNKNOWN") or "UNKNOWN")
        if self._segment_state != semantic_state or self._segment_start is None:
            self._switch_segment(semantic_state, pose)

        self._segment_path += ds
        self._segment_yaw += dyaw

        self._segment_sample_count += 1
        if _safe_float(v_l_raw, 0.0) * _safe_float(v_r_raw, 0.0) < 0.0:
            self._segment_opposite_count += 1
        vel_abs_l = abs(_safe_float(v_l_raw, 0.0))
        vel_abs_r = abs(_safe_float(v_r_raw, 0.0))
        symmetry = 1.0 - (abs(vel_abs_l - vel_abs_r) / max(vel_abs_l, vel_abs_r, 1e-6))
        self._segment_symmetry_acc += _clamp(symmetry, 0.0, 1.0)

        seg_dx = pose["x"] - self._segment_start["x"]
        seg_dy = pose["y"] - self._segment_start["y"]
        net_disp = math.hypot(seg_dx, seg_dy)
        drift_cm = net_disp * 100.0
        opposite_ratio = self._segment_opposite_count / max(1, self._segment_sample_count)
        wheel_symmetry_score = self._segment_symmetry_acc / max(1, self._segment_sample_count)

        heading_error_deg = _safe_float(semantic_status.get("heading_error_deg"), 0.0)
        forward_curvature = 0.0
        if semantic_state in ("FORWARD", "CURVED"):
            forward_curvature = abs(math.radians(self._segment_yaw)) / max(self._segment_path, 1e-3)

        translation_leakage = 0.0
        if semantic_state == "ROTATE":
            translation_leakage = net_disp / max(abs(self._segment_yaw), 1e-3)

        linear_speed_raw = 0.5 * (_safe_float(v_l_raw, 0.0) + _safe_float(v_r_raw, 0.0))
        self._linear_velocity_window.append(float(linear_speed_raw))
        vel_stability_mps = 0.0
        if len(self._linear_velocity_window) >= 5:
            arr = list(self._linear_velocity_window)
            mean_val = sum(arr) / len(arr)
            var = sum((x - mean_val) ** 2 for x in arr) / len(arr)
            vel_stability_mps = math.sqrt(max(0.0, var))

        side_ratio_lr = abs(_safe_float(v_l_raw, 0.0)) / max(abs(_safe_float(v_r_raw, 0.0)), 1e-6)

        # Stop/settle timing
        moving_cmd = abs(_safe_float(v_target, 0.0)) > 0.02 or abs(_safe_float(omega_target, 0.0)) > 0.04
        stop_threshold_mode = float(self.stop_velocity_threshold) * float(self._stop_velocity_scales.get(mode_name, 1.0))
        if moving_cmd:
            self._stop_command_ts = None
            self._stop_settling_time = None
            self._stop_residual_window.clear()
        else:
            if self._stop_command_ts is None:
                self._stop_command_ts = float(now)
            stop_residual = 0.5 * (abs(_safe_float(v_l_raw, 0.0)) + abs(_safe_float(v_r_raw, 0.0)))
            self._stop_residual_window.append(float(stop_residual))
            measured_still = (
                abs(_safe_float(v_l_raw, 0.0)) <= stop_threshold_mode
                and abs(_safe_float(v_r_raw, 0.0)) <= stop_threshold_mode
            )
            if measured_still and self._stop_settling_time is None and self._stop_command_ts is not None:
                self._stop_settling_time = max(0.0, float(now) - float(self._stop_command_ts))
        stop_residual_mps = max(self._stop_residual_window) if self._stop_residual_window else 0.0

        encoder_flags = list(encoder_reliability.get("flags", []) or [])
        enc_gate = bool(ekf_state.get("enc_gate_reject", False))
        lidar_gate = bool(ekf_state.get("lidar_gate_reject", False))
        enc_nis = _safe_float(ekf_state.get("enc_nis"), 0.0)
        innovation_theta = abs(_safe_float(ekf_state.get("innovation_theta"), 0.0))
        innovation_v = abs(_safe_float(ekf_state.get("innovation_v"), 0.0))
        trust = _safe_float(encoder_reliability.get("combined_trust"), 1.0)
        encoder_qa = {
            "left_right_coherence": _safe_float(encoder_reliability.get("left_right_coherence"), 0.0),
            "asymmetry_score": _safe_float(encoder_reliability.get("asymmetry_score"), _safe_float(encoder_reliability.get("side_asymmetry"), 0.0)),
            "forward_reliability": _safe_float(encoder_reliability.get("forward_reliability"), 0.0),
            "rotate_reliability": _safe_float(encoder_reliability.get("rotate_reliability"), 0.0),
            "idle_noise_detection": bool(encoder_reliability.get("idle_noise_detection", encoder_reliability.get("idle_false_pulse", False))),
            "stale_snapshot": bool(encoder_reliability.get("snapshot_stale", False)),
            "ekf_usage_mode": str(encoder_reliability.get("ekf_usage_mode", "NORMAL") or "NORMAL"),
        }

        inconsistency = 0.0
        inconsistency += min(1.0, innovation_theta / 0.15)
        inconsistency += min(1.0, innovation_v / 0.4)
        inconsistency += 1.0 if enc_gate else 0.0
        inconsistency += 0.7 if lidar_gate else 0.0
        inconsistency += min(1.0, max(0.0, (enc_nis - 8.0) / 12.0))
        inconsistency = _clamp(inconsistency / 4.0, 0.0, 1.0)
        confidence = _clamp((1.0 - inconsistency) * _clamp(trust, 0.0, 1.0), 0.0, 1.0)

        degradation_reasons = []
        if _safe_float(dt, 0.0) > self.degraded_dt_threshold:
            degradation_reasons.append("LOOP_TIMING_DEGRADED")
        if bool(command_overlap.get("active", False)):
            degradation_reasons.append("COMMAND_SOURCE_CONFLICT")
        if bool(encoder_reliability.get("anomaly_active", False)):
            degradation_reasons.append("ENCODER_ANOMALY")
        if bool(encoder_reliability.get("snapshot_stale", False)):
            degradation_reasons.append("ENCODER_STALE")
        if str(encoder_reliability.get("ekf_usage_mode", "NORMAL")).upper() in ("REJECT", "DEGRADED"):
            degradation_reasons.append("ENCODER_USAGE_DEGRADED")
        if enc_gate or lidar_gate:
            degradation_reasons.append("ESTIMATOR_GATE_REJECT")
        if not bool(safety_state.get("allow", True)):
            degradation_reasons.append("SAFETY_INTERVENTION")
        localization_mode = str(localization_gate_status.get("mode", "UNKNOWN") or "UNKNOWN").upper()
        localization_trust = _clamp(_safe_float(localization_gate_status.get("trust"), 0.0), 0.0, 1.0)
        if localization_mode == "LOST" or not bool(localization_gate_status.get("allow_motion", True)):
            degradation_reasons.append("LOCALIZATION_NOT_TRUSTED")
            confidence = 0.0
        elif localization_mode == "DEGRADED":
            degradation_reasons.append("LOCALIZATION_DEGRADED")
            confidence = min(float(confidence), float(localization_trust))

        quality_state = "NOMINAL"
        if confidence < 0.35 or len(degradation_reasons) >= 2:
            quality_state = "CRITICAL"
        elif confidence < 0.65 or degradation_reasons:
            quality_state = "DEGRADED"

        output = {
            "ts": round(float(now), 6),
            "control_mode": mode_name,
            "semantic_state": semantic_state,
            "motion_source": str(motion_source or "UNKNOWN"),
            "quality_state": quality_state,
            "degraded": quality_state != "NOMINAL",
            "degradation_reasons": degradation_reasons,
            "achieved_yaw": round(float(self._segment_yaw), 4),
            "heading_error": round(float(heading_error_deg), 4),
            "drift_cm": round(float(drift_cm), 4),
            "translation_leakage": round(float(translation_leakage), 6),
            "path_length": round(float(self._segment_path), 6),
            "net_displacement": round(float(net_disp), 6),
            "wheel_symmetry_score": round(float(wheel_symmetry_score), 4),
            "side_ratio_lr_abs": round(float(side_ratio_lr), 6),
            "opposite_sign_ratio": round(float(opposite_ratio), 4),
            "forward_curvature": round(float(forward_curvature), 6),
            "velocity_stability_mps": round(float(vel_stability_mps), 6),
            "stop_residual_mps": round(float(stop_residual_mps), 6),
            "stop_settling_time": (None if self._stop_settling_time is None else round(float(self._stop_settling_time), 4)),
            "encoder_noise_flags": encoder_flags,
            "encoder_qa": encoder_qa,
            "estimator_consistency": {
                "confidence": round(float(confidence), 4),
                "localization_mode": str(localization_mode),
                "localization_trust": round(float(localization_trust), 4),
                "innovation_theta_abs": round(float(innovation_theta), 6),
                "innovation_v_abs": round(float(innovation_v), 6),
                "enc_nis": round(float(enc_nis), 4),
                "enc_gate_reject": bool(enc_gate),
                "lidar_gate_reject": bool(lidar_gate),
                "trust": round(float(trust), 4),
            },
            "heading_controller": heading_controller_status,
        }
        self._last_output = output
        return output

    @property
    def last_status(self) -> Dict[str, Any]:
        return dict(self._last_output or {})
