#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Motion readiness subsystem:
- Motion semantics enforcement (IDLE/FORWARD/ROTATE meaning hardening)
- Reusable heading/turn execution with acceptance metrics
- Encoder reliability assessment (idle pulses, asymmetry, stale/noise)
- Motion QA telemetry model for GUI + regression testing
- Behavior-ready motion interface for future high-level autonomy
"""

from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

from controller.motion_schema import (
    MOTION_SCHEMA_VERSION,
    TURN_PRIMITIVE_IN_PLACE_ROTATE,
    TURN_PRIMITIVE_ONE_TRACK_PIVOT,
    classify_motion_layers,
    infer_follow_arc_twist_intent,
    normalize_execution_mode,
)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return int(default)


def _safe_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return bool(default)
    if isinstance(value, (int, float)):
        return bool(value)
    s = str(value).strip().lower()
    if s in ("1", "true", "yes", "on", "enable", "enabled"):
        return True
    if s in ("0", "false", "no", "off", "disable", "disabled", ""):
        return False
    return bool(default)


def _twist_is_effectively_zero(intent: Dict[str, Any], eps: float = 1e-3) -> bool:
    src = dict(intent or {})
    v_val = _safe_float(src.get("v"), 0.0)
    omega_val = _safe_float(src.get("omega"), 0.0)
    return abs(float(v_val)) <= float(eps) and abs(float(omega_val)) <= float(eps)


def _twist_needs_arc_semantic_anchor(
    intent: Dict[str, Any],
    *,
    zero_eps: float = 1e-3,
    low_v_eps: float = 0.02,
    min_turn_w: float = 0.05,
) -> bool:
    if _twist_is_effectively_zero(intent, eps=zero_eps):
        return True
    src = dict(intent or {})
    v_val = abs(_safe_float(src.get("v"), 0.0))
    omega_val = abs(_safe_float(src.get("omega"), 0.0))
    return v_val <= float(low_v_eps) and omega_val >= float(min_turn_w)


def _normalize_angle_deg(angle_deg: float) -> float:
    a = _safe_float(angle_deg, 0.0)
    return ((a + 180.0) % 360.0) - 180.0


def _angle_error_deg(target_deg: float, current_deg: float) -> float:
    return _normalize_angle_deg(_safe_float(target_deg, 0.0) - _safe_float(current_deg, 0.0))


def _normalize_control_mode(mode: Any) -> str:
    return str(mode or "").strip().upper()


def _track_width_m(ctrl) -> float:
    motion_executor = getattr(ctrl, "motion_executor", None)
    if motion_executor is not None and getattr(motion_executor, "track_width", None) is not None:
        try:
            return max(0.01, float(motion_executor.track_width))
        except Exception:
            pass
    try:
        return max(0.01, float((getattr(ctrl, "cfg", {}) or {}).get("fizika", {}).get("nyomtav_szelesseg_m", 0.175)))
    except Exception:
        return 0.175


@dataclass
class EncoderReliabilityConfig:
    pwm_idle_threshold: float = 0.02
    cmd_idle_threshold: float = 0.02
    velocity_noise_threshold: float = 0.03
    stale_snapshot_s: float = 0.20
    timing_gap_invalid_s: float = 0.04
    side_asymmetry_warn: float = 0.35
    side_asymmetry_critical: float = 0.65
    velocity_ema_alpha: float = 0.45
    low_speed_threshold: float = 0.06
    rotate_yaw_rate_threshold: float = 0.40
    rotate_linear_limit: float = 0.08
    wheel_base_m: float = 0.3
    asymmetry_floor_mps: float = 0.05
    pulse_aggregation_window_s: float = 0.10
    left_step_distance_m: float = 0.0
    right_step_distance_m: float = 0.0
    default_control_mode: str = "UNIFIED"
    symmetry_pwm_delta_max: float = 0.08
    symmetry_pwm_active_min: float = 0.22
    symmetry_zero_velocity_max_mps: float = 0.015
    symmetry_active_velocity_min_mps: float = 0.06
    symmetry_min_forward_cmd_mps: float = 0.08
    symmetry_fault_confirm_s: float = 0.18
    symmetry_fault_decay_s: float = 0.40


@dataclass
class EncoderFlowProfile:
    idle_noise_scale: float = 1.0
    low_speed_scale: float = 1.0
    rotate_yaw_scale: float = 1.0
    asymmetry_warn_scale: float = 1.0
    asymmetry_critical_scale: float = 1.0
    direction_switch_grace_s: float = 0.14
    low_speed_allow_linear: bool = False
    low_speed_linear_min_trust: float = 0.6


class EncoderReliabilityLayer:
    """
    Canonical encoder processing layer.
    A nyers snapshotból egységes, diagnosztizálható encoder reprezentációt épít,
    amit az EKF és a GUI ugyanazzal a szemantikával használ.
    """

    def __init__(self, cfg: Optional[Dict[str, Any]] = None):
        cfg = dict(cfg or {})
        self.cfg = EncoderReliabilityConfig(
            pwm_idle_threshold=_safe_float(cfg.get("pwm_idle_threshold"), 0.02),
            cmd_idle_threshold=_safe_float(cfg.get("cmd_idle_threshold"), 0.02),
            velocity_noise_threshold=_safe_float(cfg.get("velocity_noise_threshold"), 0.03),
            stale_snapshot_s=_safe_float(cfg.get("stale_snapshot_s"), 0.20),
            timing_gap_invalid_s=max(0.02, _safe_float(cfg.get("timing_gap_invalid_s"), 0.04)),
            side_asymmetry_warn=_safe_float(cfg.get("side_asymmetry_warn"), 0.35),
            side_asymmetry_critical=_safe_float(cfg.get("side_asymmetry_critical"), 0.65),
            velocity_ema_alpha=_safe_float(cfg.get("velocity_ema_alpha"), 0.45),
            low_speed_threshold=_safe_float(cfg.get("low_speed_threshold"), 0.06),
            rotate_yaw_rate_threshold=_safe_float(cfg.get("rotate_yaw_rate_threshold"), 0.40),
            rotate_linear_limit=_safe_float(cfg.get("rotate_linear_limit"), 0.08),
            wheel_base_m=max(0.05, _safe_float(cfg.get("wheel_base_m"), 0.175)),
            asymmetry_floor_mps=max(1e-3, _safe_float(cfg.get("asymmetry_floor_mps"), 0.05)),
            pulse_aggregation_window_s=max(0.02, _safe_float(cfg.get("pulse_aggregation_window_s"), 0.10)),
            left_step_distance_m=max(0.0, _safe_float(cfg.get("left_step_distance_m"), 0.0)),
            right_step_distance_m=max(0.0, _safe_float(cfg.get("right_step_distance_m"), 0.0)),
            default_control_mode=_normalize_control_mode(cfg.get("default_control_mode", "UNIFIED")),
            symmetry_pwm_delta_max=max(0.01, _safe_float(cfg.get("symmetry_pwm_delta_max"), 0.08)),
            symmetry_pwm_active_min=max(0.05, _safe_float(cfg.get("symmetry_pwm_active_min"), 0.22)),
            symmetry_zero_velocity_max_mps=max(1e-4, _safe_float(cfg.get("symmetry_zero_velocity_max_mps"), 0.015)),
            symmetry_active_velocity_min_mps=max(0.005, _safe_float(cfg.get("symmetry_active_velocity_min_mps"), 0.06)),
            symmetry_min_forward_cmd_mps=max(0.01, _safe_float(cfg.get("symmetry_min_forward_cmd_mps"), 0.08)),
            symmetry_fault_confirm_s=max(0.03, _safe_float(cfg.get("symmetry_fault_confirm_s"), 0.18)),
            symmetry_fault_decay_s=max(0.05, _safe_float(cfg.get("symmetry_fault_decay_s"), 0.40)),
        )
        profiles_cfg = dict(cfg.get("control_mode_profiles") or {})
        default_profiles: Dict[str, Dict[str, Any]] = {
            "UNIFIED": {
                "idle_noise_scale": 1.05,
                "low_speed_scale": 0.85,
                "rotate_yaw_scale": 1.05,
                "asymmetry_warn_scale": 1.05,
                "asymmetry_critical_scale": 1.05,
                "direction_switch_grace_s": 0.10,
                "low_speed_allow_linear": True,
                "low_speed_linear_min_trust": 0.50,
            },
        }
        self._flow_profiles: Dict[str, EncoderFlowProfile] = {}
        for mode in ("UNIFIED",):
            mode_cfg = dict(default_profiles.get(mode, {}))
            mode_cfg.update(dict(profiles_cfg.get(mode) or {}))
            self._flow_profiles[mode] = EncoderFlowProfile(
                idle_noise_scale=max(0.4, _safe_float(mode_cfg.get("idle_noise_scale"), 1.0)),
                low_speed_scale=max(0.5, _safe_float(mode_cfg.get("low_speed_scale"), 1.0)),
                rotate_yaw_scale=max(0.5, _safe_float(mode_cfg.get("rotate_yaw_scale"), 1.0)),
                asymmetry_warn_scale=max(0.5, _safe_float(mode_cfg.get("asymmetry_warn_scale"), 1.0)),
                asymmetry_critical_scale=max(0.5, _safe_float(mode_cfg.get("asymmetry_critical_scale"), 1.0)),
                direction_switch_grace_s=max(0.0, _safe_float(mode_cfg.get("direction_switch_grace_s"), 0.14)),
                low_speed_allow_linear=bool(mode_cfg.get("low_speed_allow_linear", False)),
                low_speed_linear_min_trust=_clamp(_safe_float(mode_cfg.get("low_speed_linear_min_trust"), 0.6), 0.1, 1.0),
            )

        self._last_left_pulses: Optional[int] = None
        self._last_right_pulses: Optional[int] = None
        self._last_left_distance: Optional[float] = None
        self._last_right_distance: Optional[float] = None
        self._last_snapshot_ts: Optional[float] = None
        self._canonical_left_distance: float = 0.0
        self._canonical_right_distance: float = 0.0
        self._canonical_distance_total: float = 0.0
        self._side_asymmetry_ema: float = 0.0
        self._last_forward_cmd_sign: int = 0
        self._last_direction_switch_ts: Optional[float] = None
        self._symmetry_fault_acc_s: float = 0.0
        self._symmetry_fault_side: str = "NONE"
        self._timing_gap_count: int = 0
        self._motion_timing_gap_count: int = 0
        self._idle_timing_gap_count: int = 0
        self._last_timing_gap: Dict[str, Any] = {}
        # Exact endpoint samples from the signed KIT0085 counters.  Canonical
        # wheel velocity is derived from these endpoints, never from a second
        # independently accumulated distance-delta stream.
        self._recent_counter_samples: deque = deque()
        self._last_out: Dict[str, Any] = {}

    @staticmethod
    def _sign(value: float, eps: float = 1e-6) -> int:
        if value > eps:
            return 1
        if value < -eps:
            return -1
        return 0

    def _flow_profile(self, control_mode: Optional[str]) -> tuple[str, EncoderFlowProfile]:
        mode = _normalize_control_mode(control_mode or self.cfg.default_control_mode)
        prof = self._flow_profiles.get(mode)
        if prof is None:
            raise ValueError(f"unsupported_control_mode:{mode or 'MISSING'}")
        return mode, prof

    def update(
        self,
        *,
        enc_snapshot,
        pwm_l: float,
        pwm_r: float,
        v_target: float,
        omega_target: float,
        motion_state: str,
        control_mode: Optional[str] = None,
        observation_context: str = "NORMAL",
        now_mono: Optional[float] = None,
    ) -> Dict[str, Any]:
        now_mono = time.perf_counter() if now_mono is None else float(now_mono)
        mode_name, flow_profile = self._flow_profile(control_mode)
        velocity_noise_threshold = max(1e-4, self.cfg.velocity_noise_threshold * flow_profile.idle_noise_scale)
        low_speed_threshold = max(1e-4, self.cfg.low_speed_threshold * flow_profile.low_speed_scale)
        rotate_yaw_rate_threshold = max(1e-4, self.cfg.rotate_yaw_rate_threshold * flow_profile.rotate_yaw_scale)
        asymmetry_warn_threshold = _clamp(
            self.cfg.side_asymmetry_warn * flow_profile.asymmetry_warn_scale,
            0.05,
            0.98,
        )
        asymmetry_critical_threshold = _clamp(
            self.cfg.side_asymmetry_critical * flow_profile.asymmetry_critical_scale,
            asymmetry_warn_threshold + 0.01,
            0.995,
        )
        timing_gap_invalid_threshold = max(0.02, float(self.cfg.timing_gap_invalid_s))

        if enc_snapshot is None:
            out = {
                "available": False,
                "anomaly_active": True,
                "flags": ["ENCODER_SNAPSHOT_MISSING"],
                "left_trust": 0.0,
                "right_trust": 0.0,
                "combined_trust": 0.0,
                "motion_state": str(motion_state or "UNKNOWN"),
                "control_mode": mode_name,
                "canonical_state": "STALE",
                "canonical_available": False,
                "timing_valid": False,
                "timing_error": "SNAPSHOT_MISSING",
                "timing_gap_count": int(self._timing_gap_count),
                "ekf_usage_mode": "REJECT",
                "ekf_usage_reason": "SNAPSHOT_MISSING",
                "pipeline_model": "KIT0085_QUADRATURE",
                "source_truth": "QUADRATURE_SIGNED_PULSE_DELTA",
                "observation_context": str(observation_context or "NORMAL"),
            }
            self._last_out = out
            return out

        left_vel_snap = _safe_float(
            getattr(enc_snapshot, "left_velocity_raw", getattr(enc_snapshot, "left_velocity", 0.0)),
            0.0,
        )
        right_vel_snap = _safe_float(
            getattr(enc_snapshot, "right_velocity_raw", getattr(enc_snapshot, "right_velocity", 0.0)),
            0.0,
        )
        left_vel_unsigned = _safe_float(
            getattr(enc_snapshot, "left_velocity_unsigned", abs(left_vel_snap)),
            abs(left_vel_snap),
        )
        right_vel_unsigned = _safe_float(
            getattr(enc_snapshot, "right_velocity_unsigned", abs(right_vel_snap)),
            abs(right_vel_snap),
        )
        if not math.isfinite(left_vel_snap):
            left_vel_snap = 0.0
        if not math.isfinite(right_vel_snap):
            right_vel_snap = 0.0
        if not math.isfinite(left_vel_unsigned):
            left_vel_unsigned = abs(left_vel_snap)
        if not math.isfinite(right_vel_unsigned):
            right_vel_unsigned = abs(right_vel_snap)

        left_dist = _safe_float(getattr(enc_snapshot, "left_distance", 0.0), 0.0)
        right_dist = _safe_float(getattr(enc_snapshot, "right_distance", 0.0), 0.0)
        left_pulses = int(getattr(enc_snapshot, "left_pulses", 0) or 0)
        right_pulses = int(getattr(enc_snapshot, "right_pulses", 0) or 0)
        snap_ts = _safe_float(getattr(enc_snapshot, "timestamp", 0.0), 0.0)
        snap_health = str(getattr(enc_snapshot, "health", "N/A") or "N/A")
        left_step_distance_m = max(
            0.0,
            _safe_float(
                getattr(enc_snapshot, "left_step_distance_m", None),
                self.cfg.left_step_distance_m,
            ),
        )
        right_step_distance_m = max(
            0.0,
            _safe_float(
                getattr(enc_snapshot, "right_step_distance_m", None),
                self.cfg.right_step_distance_m,
            ),
        )
        new_snapshot_measurement = bool(
            self._last_snapshot_ts is None
            or (snap_ts > 0.0 and snap_ts > float(self._last_snapshot_ts))
        )
        dt_control_window = 0.0
        if self._last_snapshot_ts is not None and snap_ts > float(self._last_snapshot_ts):
            dt_control_window = max(0.0, snap_ts - float(self._last_snapshot_ts))

        dp_l_snap = getattr(enc_snapshot, "left_pulse_delta", None)
        dp_r_snap = getattr(enc_snapshot, "right_pulse_delta", None)
        if dp_l_snap is not None and dp_r_snap is not None:
            dp_l_inst = int(dp_l_snap or 0)
            dp_r_inst = int(dp_r_snap or 0)
        else:
            dp_l_inst = 0
            dp_r_inst = 0

        dp_l_control = None
        dp_r_control = None
        if self._last_left_pulses is not None:
            dp_l_control = left_pulses - int(self._last_left_pulses)
        if self._last_right_pulses is not None:
            dp_r_control = right_pulses - int(self._last_right_pulses)
        dp_l = int(dp_l_control) if dp_l_control is not None else int(dp_l_inst)
        dp_r = int(dp_r_control) if dp_r_control is not None else int(dp_r_inst)

        dt_snap = _safe_float(getattr(enc_snapshot, "sample_dt", 0.0), 0.0)
        if dt_snap <= 0.0 and self._last_snapshot_ts is not None and snap_ts > 0.0:
            dt_snap = max(0.0, snap_ts - float(self._last_snapshot_ts))

        delta_l_dist_snap = getattr(enc_snapshot, "left_distance_delta", None)
        delta_r_dist_snap = getattr(enc_snapshot, "right_distance_delta", None)
        if delta_l_dist_snap is not None and delta_r_dist_snap is not None:
            delta_l_dist_inst = _safe_float(delta_l_dist_snap, 0.0)
            delta_r_dist_inst = _safe_float(delta_r_dist_snap, 0.0)
        else:
            delta_l_dist_inst = 0.0
            delta_r_dist_inst = 0.0

        delta_l_dist_control = None
        delta_r_dist_control = None
        if self._last_left_distance is not None and self._last_right_distance is not None:
            delta_l_dist_control = left_dist - float(self._last_left_distance)
            delta_r_dist_control = right_dist - float(self._last_right_distance)

        delta_l_dist_observed = (
            float(delta_l_dist_control)
            if delta_l_dist_control is not None
            else float(delta_l_dist_inst)
        )
        delta_r_dist_observed = (
            float(delta_r_dist_control)
            if delta_r_dist_control is not None
            else float(delta_r_dist_inst)
        )
        delta_l_dist = (
            float(dp_l) * float(left_step_distance_m)
            if left_step_distance_m > 0.0
            else float(delta_l_dist_observed)
        )
        delta_r_dist = (
            float(dp_r) * float(right_step_distance_m)
            if right_step_distance_m > 0.0
            else float(delta_r_dist_observed)
        )

        v_l_raw = float(left_vel_snap)
        v_r_raw = float(right_vel_snap)
        if dt_control_window > 1e-6:
            v_l_control = delta_l_dist / dt_control_window
            v_r_control = delta_r_dist / dt_control_window
            if abs(delta_l_dist) > 1e-12:
                v_l_raw = v_l_control
            if abs(delta_r_dist) > 1e-12:
                v_r_raw = v_r_control
        if dt_snap > 1e-6:
            if abs(v_l_raw) < 1e-9 and abs(delta_l_dist) > 1e-12:
                v_l_raw = delta_l_dist / dt_snap
            if abs(v_r_raw) < 1e-9 and abs(delta_r_dist) > 1e-12:
                v_r_raw = delta_r_dist / dt_snap

        window_ts = snap_ts if snap_ts > 0.0 else now_mono
        if not self._recent_counter_samples or window_ts > float(
            self._recent_counter_samples[-1].get("ts", 0.0)
        ):
            self._recent_counter_samples.append(
                {
                    "ts": float(window_ts),
                    "left_pulses": int(left_pulses),
                    "right_pulses": int(right_pulses),
                    "left_distance": float(left_dist),
                    "right_distance": float(right_dist),
                }
            )
        elif window_ts < float(self._recent_counter_samples[-1].get("ts", 0.0)):
            # A monotonic measurement clock must not go backwards.  Re-anchor
            # instead of mixing two time domains in one velocity window.
            self._recent_counter_samples.clear()
            self._recent_counter_samples.append(
                {
                    "ts": float(window_ts),
                    "left_pulses": int(left_pulses),
                    "right_pulses": int(right_pulses),
                    "left_distance": float(left_dist),
                    "right_distance": float(right_dist),
                }
            )
        agg_window_s = max(0.02, float(self.cfg.pulse_aggregation_window_s))
        cutoff_ts = float(window_ts) - agg_window_s
        while (
            len(self._recent_counter_samples) >= 2
            and float(self._recent_counter_samples[1].get("ts", 0.0)) <= cutoff_ts
        ):
            self._recent_counter_samples.popleft()

        window_start = dict(self._recent_counter_samples[0])
        window_end = dict(self._recent_counter_samples[-1])
        agg_dt = max(
            0.0,
            float(window_end.get("ts", window_ts))
            - float(window_start.get("ts", window_ts)),
        )
        if len(self._recent_counter_samples) >= 2 and agg_dt > 1e-6:
            agg_dp_l = int(window_end.get("left_pulses", left_pulses)) - int(
                window_start.get("left_pulses", left_pulses)
            )
            agg_dp_r = int(window_end.get("right_pulses", right_pulses)) - int(
                window_start.get("right_pulses", right_pulses)
            )
            agg_delta_l = (
                float(agg_dp_l) * float(left_step_distance_m)
                if left_step_distance_m > 0.0
                else float(window_end.get("left_distance", left_dist))
                - float(window_start.get("left_distance", left_dist))
            )
            agg_delta_r = (
                float(agg_dp_r) * float(right_step_distance_m)
                if right_step_distance_m > 0.0
                else float(window_end.get("right_distance", right_dist))
                - float(window_start.get("right_distance", right_dist))
            )
        else:
            agg_dt = float(dt_snap if dt_snap > 1e-6 else dt_control_window)
            agg_dp_l = int(dp_l)
            agg_dp_r = int(dp_r)
            agg_delta_l = float(delta_l_dist)
            agg_delta_r = float(delta_r_dist)
        has_counter_distance_contract = bool(
            left_step_distance_m > 0.0 and right_step_distance_m > 0.0
        )
        has_measured_endpoint_interval = len(self._recent_counter_samples) >= 2
        if agg_dt > 1e-6 and (
            has_counter_distance_contract or has_measured_endpoint_interval
        ):
            v_l_raw = agg_delta_l / agg_dt if abs(agg_delta_l) > 1e-12 else 0.0
            v_r_raw = agg_delta_r / agg_dt if abs(agg_delta_r) > 1e-12 else 0.0
        dp_l_window = int(agg_dp_l)
        dp_r_window = int(agg_dp_r)

        self._last_left_pulses = left_pulses
        self._last_right_pulses = right_pulses
        self._last_snapshot_ts = snap_ts if snap_ts > 0.0 else self._last_snapshot_ts
        self._last_left_distance = left_dist
        self._last_right_distance = right_dist

        age_s = max(0.0, now_mono - snap_ts) if snap_ts > 0.0 else 999.0
        published_at = _safe_float(getattr(enc_snapshot, "published_at", 0.0), 0.0)
        publication_delay_s = (
            max(0.0, published_at - snap_ts)
            if published_at > 0.0 and snap_ts > 0.0
            else None
        )
        stale_snapshot = age_s > self.cfg.stale_snapshot_s
        timing_gap_s = max(float(dt_snap), float(dt_control_window))
        timing_gap_invalid = bool(
            timing_gap_s > float(timing_gap_invalid_threshold)
        )
        timing_valid = not timing_gap_invalid
        timing_error = "TIMING_GAP" if timing_gap_invalid else ""
        timing_gap_new_event = bool(timing_gap_invalid and new_snapshot_measurement)
        if timing_gap_new_event:
            self._timing_gap_count += 1
            self._last_timing_gap = {
                "measurement_timestamp_s": round(float(snap_ts), 6) if snap_ts > 0.0 else None,
                "dt_snapshot_s": round(float(dt_snap), 6),
                "dt_control_window_s": round(float(dt_control_window), 6),
                "gap_s": round(float(timing_gap_s), 6),
                "threshold_s": round(float(timing_gap_invalid_threshold), 6),
            }
            # Preserve the invalid interval above for telemetry, then make its
            # endpoint the sole anchor.  Otherwise the next healthy 20 ms
            # sample would still divide by a window spanning the lost-callback
            # gap and publish a second false near-zero speed.
            self._recent_counter_samples.clear()
            self._recent_counter_samples.append(
                {
                    "ts": float(window_ts),
                    "left_pulses": int(left_pulses),
                    "right_pulses": int(right_pulses),
                    "left_distance": float(left_dist),
                    "right_distance": float(right_dist),
                }
            )
        timing_ratio = (
            timing_gap_s / timing_gap_invalid_threshold
            if timing_gap_s > 0.0 and timing_gap_invalid_threshold > 1e-9
            else 0.0
        )
        timing_quality = _clamp(1.0 / max(1.0, timing_ratio), 0.0, 1.0)

        motion_state_name = str(motion_state or "").upper()

        motor_off = (
            abs(_safe_float(pwm_l, 0.0)) <= self.cfg.pwm_idle_threshold
            and abs(_safe_float(pwm_r, 0.0)) <= self.cfg.pwm_idle_threshold
        )
        cmd_idle = bool(
            motion_state_name == "IDLE"
            or (
                abs(_safe_float(v_target, 0.0)) <= self.cfg.cmd_idle_threshold
                and abs(_safe_float(omega_target, 0.0)) <= self.cfg.cmd_idle_threshold
            )
        )
        idle_expected = bool(motor_off and cmd_idle)
        if timing_gap_new_event:
            # The encoder sample is evaluated before this tick's motor output is
            # written.  At motion start the command may already be non-zero while
            # the previous applied PWM is still zero; classify that as a
            # non-motion startup-sync gap, not as an active motor-output gap.
            timing_gap_motion_active = not bool(motor_off)
            startup_sync_gap = bool((not cmd_idle) and motor_off)
            if timing_gap_motion_active:
                self._motion_timing_gap_count += 1
            else:
                self._idle_timing_gap_count += 1
            self._last_timing_gap.update(
                {
                    "motion_active": bool(timing_gap_motion_active),
                    "motor_output_active": bool(timing_gap_motion_active),
                    "startup_sync_gap": bool(startup_sync_gap),
                    "classification": (
                        "MOTOR_OUTPUT_ACTIVE"
                        if timing_gap_motion_active
                        else (
                            "STARTUP_SYNC_PWM_ZERO"
                            if startup_sync_gap
                            else "IDLE_OR_PWM_ZERO"
                        )
                    ),
                    "motion_state": str(motion_state_name or "UNKNOWN"),
                    "pwm_zero": bool(motor_off),
                    "command_zero": bool(cmd_idle),
                }
            )

        velocity_noise_raw = (
            abs(v_l_raw) > velocity_noise_threshold
            or abs(v_r_raw) > velocity_noise_threshold
        )
        pulse_noise = abs(dp_l_window) > 0 or abs(dp_r_window) > 0
        idle_false_pulse = bool(idle_expected and (pulse_noise or velocity_noise_raw))

        left_dir = _safe_float(getattr(enc_snapshot, "left_direction", 0.0), 0.0)
        right_dir = _safe_float(getattr(enc_snapshot, "right_direction", 0.0), 0.0)
        left_dir_src = str(getattr(enc_snapshot, "left_direction_source", "N/A") or "N/A")
        right_dir_src = str(getattr(enc_snapshot, "right_direction_source", "N/A") or "N/A")
        left_dir_conf = bool(getattr(enc_snapshot, "left_direction_confident", True))
        right_dir_conf = bool(getattr(enc_snapshot, "right_direction_confident", True))
        unresolved_l = int(getattr(enc_snapshot, "left_unresolved_pulses", 0) or 0)
        unresolved_r = int(getattr(enc_snapshot, "right_unresolved_pulses", 0) or 0)
        direction_uncertain = bool(
            ((not left_dir_conf) and abs(dp_l) > 0)
            or ((not right_dir_conf) and abs(dp_r) > 0)
            or unresolved_l > 0
            or unresolved_r > 0
        )

        # RAW-first: canonical indulásként megegyezik a signed nyers méréssel.
        left_vel = float(v_l_raw)
        right_vel = float(v_r_raw)
        canonical_delta_l = float(delta_l_dist)
        canonical_delta_r = float(delta_r_dist)
        distance_model = "RAW_SIGNED"
        theta_measurement_reliable = True

        if idle_expected and not pulse_noise:
            if abs(left_vel) < velocity_noise_threshold:
                left_vel = 0.0
            if abs(right_vel) < velocity_noise_threshold:
                right_vel = 0.0
        if idle_expected and idle_false_pulse:
            theta_measurement_reliable = False
            left_vel = 0.0
            right_vel = 0.0
            canonical_delta_l = 0.0
            canonical_delta_r = 0.0
            distance_model = "IDLE_CLAMP"

        if stale_snapshot or snap_health != "OK":
            left_vel = 0.0
            right_vel = 0.0
            canonical_delta_l = 0.0
            canonical_delta_r = 0.0
            if distance_model == "RAW_SIGNED":
                distance_model = "STALE_SKIP" if stale_snapshot else "HEALTH_SKIP"
            theta_measurement_reliable = False
        if direction_uncertain and pulse_noise:
            theta_measurement_reliable = False

        vel_abs_l_raw = abs(v_l_raw)
        vel_abs_r_raw = abs(v_r_raw)
        max_vel_abs_raw = max(vel_abs_l_raw, vel_abs_r_raw, 1e-6)
        side_asymmetry_raw = abs(vel_abs_l_raw - vel_abs_r_raw) / max_vel_abs_raw
        asymmetry_den = max(max_vel_abs_raw, self.cfg.asymmetry_floor_mps)
        side_asymmetry_stable = abs(vel_abs_l_raw - vel_abs_r_raw) / asymmetry_den
        command_linear_mps = _safe_float(v_target, 0.0)
        command_omega_rad_s = _safe_float(omega_target, 0.0)
        moving_expected = bool(
            abs(command_linear_mps) > self.cfg.cmd_idle_threshold
            or abs(command_omega_rad_s) > self.cfg.cmd_idle_threshold
        )
        cmd_forward = bool(
            abs(command_linear_mps) > self.cfg.cmd_idle_threshold
            and abs(command_omega_rad_s) <= self.cfg.cmd_idle_threshold
        )
        cmd_rotate = bool(
            abs(command_omega_rad_s) > self.cfg.cmd_idle_threshold
            and abs(command_linear_mps) <= self.cfg.cmd_idle_threshold
        )
        expected_left_mps = command_linear_mps - 0.5 * self.cfg.wheel_base_m * command_omega_rad_s
        expected_right_mps = command_linear_mps + 0.5 * self.cfg.wheel_base_m * command_omega_rad_s
        expected_abs_left = abs(expected_left_mps)
        expected_abs_right = abs(expected_right_mps)
        expected_abs_max = max(expected_abs_left, expected_abs_right, 1e-6)
        expected_side_asymmetry = abs(expected_abs_left - expected_abs_right) / expected_abs_max
        comparison_floor_mps = max(0.01, 0.2 * self.cfg.asymmetry_floor_mps)
        side_asymmetry_comparable = bool(
            moving_expected
            and expected_abs_left >= comparison_floor_mps
            and expected_abs_right >= comparison_floor_mps
        )
        side_asymmetry_command_expected = bool(
            moving_expected and expected_side_asymmetry >= asymmetry_warn_threshold
        )
        normalized_gain_left = 0.0
        normalized_gain_right = 0.0
        side_asymmetry_command_residual = 0.0
        if side_asymmetry_comparable:
            normalized_gain_left = vel_abs_l_raw / max(expected_abs_left, comparison_floor_mps)
            normalized_gain_right = vel_abs_r_raw / max(expected_abs_right, comparison_floor_mps)
            side_asymmetry_command_residual = abs(
                normalized_gain_left - normalized_gain_right
            ) / max(normalized_gain_left, normalized_gain_right, 1e-6)
        alpha_asym = _clamp(self.cfg.velocity_ema_alpha, 0.05, 1.0)
        self._side_asymmetry_ema = (
            (1.0 - alpha_asym) * float(self._side_asymmetry_ema)
            + alpha_asym * float(side_asymmetry_command_residual)
        )
        side_asymmetry = _clamp(
            0.6 * side_asymmetry_command_residual + 0.4 * float(self._side_asymmetry_ema),
            0.0,
            1.0,
        )
        side_asymmetry_excessive = bool(
            side_asymmetry_comparable and side_asymmetry >= asymmetry_warn_threshold
        )
        side_asymmetry_critical = bool(
            side_asymmetry_comparable and side_asymmetry >= asymmetry_critical_threshold
        )
        left_right_coherence = _clamp(1.0 - side_asymmetry, 0.0, 1.0)

        forward_drive_active = bool(
            cmd_forward
            and (
                abs(_safe_float(v_target, 0.0)) >= 0.10
                or abs(_safe_float(pwm_l, 0.0)) >= 0.20
                or abs(_safe_float(pwm_r, 0.0)) >= 0.20
            )
        )

        v_linear = 0.5 * (left_vel + right_vel)
        v_linear_raw = 0.5 * (v_l_raw + v_r_raw)
        yaw_rate = (right_vel - left_vel) / max(self.cfg.wheel_base_m, 1e-3)
        yaw_rate_raw = (v_r_raw - v_l_raw) / max(self.cfg.wheel_base_m, 1e-3)

        forward_direction_match = 1.0
        if cmd_forward:
            forward_direction_match = 1.0 if self._sign(v_linear_raw) == self._sign(v_target) else 0.0
        same_sign_drive = 1.0 if (self._sign(v_l_raw) == self._sign(v_r_raw) and self._sign(v_l_raw) != 0) else 0.0
        forward_reliability = _clamp(
            0.65 * left_right_coherence + 0.25 * forward_direction_match + 0.10 * same_sign_drive,
            0.0,
            1.0,
        )

        rotate_dir_match = 1.0
        if cmd_rotate:
            rotate_dir_match = 1.0 if self._sign(yaw_rate_raw) == self._sign(omega_target) else 0.0
        opposite_sign_drive = 1.0 if (self._sign(left_vel) * self._sign(right_vel) < 0) else 0.0
        rotate_reliability = _clamp(
            0.60 * left_right_coherence + 0.20 * rotate_dir_match + 0.20 * opposite_sign_drive,
            0.0,
            1.0,
        )

        canonical_delta_avg = 0.5 * (canonical_delta_l + canonical_delta_r)

        # A callback/control gap can lose Hall edges while the Python process is
        # stopped.  Keep every raw endpoint in telemetry, but do not integrate
        # the incomplete interval into the canonical pose-distance channel.
        if timing_gap_invalid:
            canonical_delta_l = 0.0
            canonical_delta_r = 0.0
            canonical_delta_avg = 0.0
            theta_measurement_reliable = False

        self._canonical_left_distance += canonical_delta_l
        self._canonical_right_distance += canonical_delta_r
        self._canonical_distance_total += canonical_delta_avg

        idle_noise_score = 0.0
        if idle_expected:
            idle_noise_score += min(1.0, max(vel_abs_l_raw, vel_abs_r_raw) / max(velocity_noise_threshold, 1e-6))
            idle_noise_score += 0.6 if pulse_noise else 0.0
            idle_noise_score = _clamp(idle_noise_score / 1.6, 0.0, 1.0)

        canonical_state = "LOW_SPEED"
        low_speed = max(low_speed_threshold, velocity_noise_threshold)
        if timing_gap_invalid:
            canonical_state = "TIMING_GAP"
        elif stale_snapshot:
            canonical_state = "STALE"
        elif snap_health != "OK":
            canonical_state = "DEGRADED"
        elif idle_expected and abs(v_linear) < low_speed and abs(yaw_rate) < rotate_yaw_rate_threshold * 0.5:
            canonical_state = "IDLE"
        elif abs(yaw_rate) >= rotate_yaw_rate_threshold and abs(v_linear) <= self.cfg.rotate_linear_limit:
            canonical_state = "ROTATE"
        elif abs(v_linear) >= low_speed:
            canonical_state = "FORWARD"
        else:
            canonical_state = "LOW_SPEED"

        # Motor-off IDLE szakaszban az encoder theta abszolút értéke könnyen
        # elcsúszhat hamis pulzusoktól, ezért yaw update-re itt nem tekintjük
        # megbízhatónak. A headinget ilyenkor a gyro + theta_hold stabilizálja.
        if canonical_state == "IDLE":
            theta_measurement_reliable = False

        backward_commanded = bool(cmd_forward and self._sign(v_target) < 0)
        backward_consistent = bool(backward_commanded and self._sign(v_linear_raw) < 0)
        forward_cmd_sign = self._sign(v_target) if cmd_forward else 0
        direction_switch_recent = False
        if forward_cmd_sign != 0:
            if self._last_forward_cmd_sign != 0 and forward_cmd_sign != self._last_forward_cmd_sign:
                self._last_direction_switch_ts = now_mono
            self._last_forward_cmd_sign = forward_cmd_sign
        if self._last_direction_switch_ts is not None:
            direction_switch_recent = (now_mono - float(self._last_direction_switch_ts)) <= float(
                flow_profile.direction_switch_grace_s
            )

        dt_fault = float(dt_snap if dt_snap > 1e-6 else dt_control_window)
        if dt_fault <= 1e-6 and agg_dt > 1e-6 and len(self._recent_counter_samples) > 1:
            dt_fault = float(agg_dt / max(1, len(self._recent_counter_samples) - 1))
        if dt_fault <= 1e-6:
            dt_fault = 0.02

        pwm_abs_l = abs(_safe_float(pwm_l, 0.0))
        pwm_abs_r = abs(_safe_float(pwm_r, 0.0))
        pwm_symmetry_abs_delta = abs(pwm_abs_l - pwm_abs_r)
        pwm_symmetry_expected = bool(
            cmd_forward
            and abs(_safe_float(omega_target, 0.0)) <= self.cfg.cmd_idle_threshold
            and abs(_safe_float(v_target, 0.0)) >= self.cfg.symmetry_min_forward_cmd_mps
            and min(pwm_abs_l, pwm_abs_r) >= self.cfg.symmetry_pwm_active_min
            and pwm_symmetry_abs_delta <= self.cfg.symmetry_pwm_delta_max
        )

        dropout_side_instant = "NONE"
        if (
            vel_abs_l_raw <= self.cfg.symmetry_zero_velocity_max_mps
            and vel_abs_r_raw >= self.cfg.symmetry_active_velocity_min_mps
        ):
            dropout_side_instant = "LEFT"
        elif (
            vel_abs_r_raw <= self.cfg.symmetry_zero_velocity_max_mps
            and vel_abs_l_raw >= self.cfg.symmetry_active_velocity_min_mps
        ):
            dropout_side_instant = "RIGHT"

        symmetry_violation_instant = bool(pwm_symmetry_expected and dropout_side_instant != "NONE")
        symmetry_fault_candidate = bool(
            symmetry_violation_instant
            and not direction_uncertain
            and not direction_switch_recent
            and not stale_snapshot
            and snap_health == "OK"
        )
        if symmetry_fault_candidate:
            if self._symmetry_fault_side not in ("NONE", dropout_side_instant):
                self._symmetry_fault_acc_s = max(0.0, float(self._symmetry_fault_acc_s) * 0.5)
            self._symmetry_fault_side = str(dropout_side_instant)
            self._symmetry_fault_acc_s = min(
                5.0,
                float(self._symmetry_fault_acc_s) + max(0.0, float(dt_fault)),
            )
        else:
            decay_norm = max(1e-6, float(self.cfg.symmetry_fault_decay_s))
            self._symmetry_fault_acc_s = max(
                0.0,
                float(self._symmetry_fault_acc_s) - (max(0.0, float(dt_fault)) / decay_norm),
            )
            if self._symmetry_fault_acc_s <= 1e-6:
                self._symmetry_fault_side = "NONE"
        symmetry_fault_active = bool(self._symmetry_fault_acc_s >= float(self.cfg.symmetry_fault_confirm_s))
        symmetry_fault_side = str(self._symmetry_fault_side if symmetry_fault_active else "NONE")
        if symmetry_violation_instant:
            theta_measurement_reliable = False

        flags = []
        if snap_health != "OK":
            flags.append("ENCODER_HEALTH_NOT_OK")
        if stale_snapshot:
            flags.append("ENCODER_STALE")
        if timing_gap_invalid:
            flags.append("ENCODER_TIMING_GAP")
        if idle_false_pulse:
            flags.append("IDLE_FALSE_PULSE")
        if side_asymmetry_excessive:
            flags.append("SIDE_ASYMMETRY")
        if side_asymmetry_critical:
            flags.append("SIDE_ASYMMETRY_CRITICAL")
        if direction_uncertain and pulse_noise:
            flags.append("DIRECTION_UNRESOLVED")
        if cmd_forward and forward_reliability < 0.45:
            flags.append("FORWARD_COHERENCE_LOW")
        if cmd_forward and forward_direction_match < 0.5 and not direction_switch_recent:
            flags.append("FORWARD_DIRECTION_MISMATCH")
        if backward_commanded and not backward_consistent and not direction_switch_recent:
            flags.append("BACKWARD_DIRECTION_MISMATCH")
        if symmetry_violation_instant:
            flags.append("PWM_ENCODER_SYMMETRY_VIOLATION")
        if symmetry_fault_active:
            flags.append("PWM_ENCODER_SYMMETRY_FAULT")
            if symmetry_fault_side == "LEFT":
                flags.append("LEFT_ENCODER_DROPOUT_SUSPECT")
            elif symmetry_fault_side == "RIGHT":
                flags.append("RIGHT_ENCODER_DROPOUT_SUSPECT")
        if direction_switch_recent:
            flags.append("DIRECTION_SWITCH_GRACE")
        if cmd_rotate and rotate_reliability < 0.45:
            flags.append("ROTATE_COHERENCE_LOW")
        left_trust = 1.0
        right_trust = 1.0

        if snap_health != "OK":
            left_trust *= 0.25
            right_trust *= 0.25
        if stale_snapshot:
            left_trust *= 0.0
            right_trust *= 0.0
        if timing_gap_invalid:
            left_trust = 0.0
            right_trust = 0.0
        if idle_false_pulse:
            left_trust *= 0.2
            right_trust *= 0.2
        if direction_uncertain and pulse_noise:
            left_trust *= 0.5
            right_trust *= 0.5
        if side_asymmetry_excessive:
            slower_is_left = normalized_gain_left < normalized_gain_right
            slower_trust_scale = 0.35 if side_asymmetry_critical else 0.55
            if slower_is_left:
                left_trust *= slower_trust_scale
            else:
                right_trust *= slower_trust_scale
        if symmetry_violation_instant:
            if dropout_side_instant == "LEFT":
                left_trust *= 0.30
                right_trust *= 0.90
            elif dropout_side_instant == "RIGHT":
                right_trust *= 0.30
                left_trust *= 0.90
        if symmetry_fault_active:
            if symmetry_fault_side == "LEFT":
                left_trust *= 0.08
                right_trust *= 0.75
            elif symmetry_fault_side == "RIGHT":
                right_trust *= 0.08
                left_trust *= 0.75
        if cmd_forward:
            left_trust *= forward_reliability
            right_trust *= forward_reliability
        if cmd_rotate:
            left_trust *= rotate_reliability
            right_trust *= rotate_reliability
        if backward_commanded and not backward_consistent:
            mismatch_scale = 0.75 if direction_switch_recent else 0.4
            left_trust *= mismatch_scale
            right_trust *= mismatch_scale

        combined_trust = 0.5 * (left_trust + right_trust)
        anomaly_active = bool(flags)

        if canonical_state not in ("STALE",) and (
            idle_false_pulse
            or timing_gap_invalid
            or side_asymmetry_critical
            or symmetry_fault_active
            or combined_trust < 0.35
        ):
            canonical_state = "TIMING_GAP" if timing_gap_invalid else "DEGRADED"

        ekf_usage_mode = "NORMAL"
        ekf_usage_reason = "NOMINAL"
        if timing_gap_invalid:
            ekf_usage_mode = "REJECT"
            ekf_usage_reason = "TIMING_GAP"
        elif stale_snapshot:
            ekf_usage_mode = "REJECT"
            ekf_usage_reason = "ENCODER_STALE"
        elif snap_health != "OK":
            ekf_usage_mode = "REJECT"
            ekf_usage_reason = "ENCODER_HEALTH"
        elif idle_false_pulse and idle_expected:
            ekf_usage_mode = "REJECT"
            ekf_usage_reason = "IDLE_NOISE"
        elif direction_uncertain and pulse_noise and idle_expected:
            ekf_usage_mode = "REJECT"
            ekf_usage_reason = "DIRECTION_UNRESOLVED"
        elif symmetry_fault_active:
            ekf_usage_mode = "REJECT"
            ekf_usage_reason = "PWM_ENCODER_SYMMETRY_FAULT"
        elif symmetry_violation_instant and moving_expected:
            ekf_usage_mode = "DEGRADED"
            ekf_usage_reason = "PWM_ENCODER_SYMMETRY_WARN"
        elif combined_trust < 0.20:
            ekf_usage_mode = "REJECT"
            ekf_usage_reason = "LOW_TRUST"
        elif canonical_state == "IDLE":
            ekf_usage_mode = "THETA_ONLY"
            ekf_usage_reason = "IDLE_MODE"
        elif canonical_state == "LOW_SPEED":
            low_speed_stable_motion = bool(
                cmd_forward
                and pulse_noise
                and not direction_uncertain
                and not timing_gap_invalid
                and abs(v_linear_raw) >= max(velocity_noise_threshold * 0.6, 0.015)
                and forward_reliability >= 0.55
                and combined_trust >= flow_profile.low_speed_linear_min_trust
            )
            if forward_drive_active or (flow_profile.low_speed_allow_linear and low_speed_stable_motion):
                if combined_trust < 0.55:
                    ekf_usage_mode = "DEGRADED"
                    ekf_usage_reason = "LOW_SPEED_FORWARD_DEGRADED"
                else:
                    if flow_profile.low_speed_allow_linear:
                        ekf_usage_mode = "DEGRADED"
                        ekf_usage_reason = "LOW_SPEED_LINEAR_STABLE"
                    else:
                        ekf_usage_mode = "NORMAL"
                        ekf_usage_reason = "LOW_SPEED_FORWARD"
            else:
                ekf_usage_mode = "THETA_ONLY"
                ekf_usage_reason = "LOW_SPEED_MODE"
        elif canonical_state == "DEGRADED" or combined_trust < 0.55:
            ekf_usage_mode = "DEGRADED"
            ekf_usage_reason = "QUALITY_DEGRADED"

        side_asymmetry_covariance_penalty = max(
            0.0, side_asymmetry - asymmetry_warn_threshold
        ) * 3.0
        ekf_covariance_scale_hint = _clamp(
            1.0
            + (1.0 - _clamp(combined_trust, 0.0, 1.0)) * 4.0
            + side_asymmetry_covariance_penalty
            + max(0.0, timing_ratio - 1.0) * 1.5,
            1.0,
            12.0,
        )
        if ekf_usage_mode == "THETA_ONLY":
            ekf_covariance_scale_hint = max(ekf_covariance_scale_hint, 3.0)
        elif ekf_usage_mode == "DEGRADED":
            ekf_covariance_scale_hint = max(ekf_covariance_scale_hint, 2.0)
        elif ekf_usage_mode == "REJECT":
            ekf_covariance_scale_hint = max(ekf_covariance_scale_hint, 8.0)
        if symmetry_violation_instant:
            ekf_covariance_scale_hint = max(ekf_covariance_scale_hint, 3.5)
        if symmetry_fault_active:
            ekf_covariance_scale_hint = max(ekf_covariance_scale_hint, 10.0)
        ekf_weight_hint = _clamp(1.0 / max(1.0, ekf_covariance_scale_hint), 0.0, 1.0)
        canonical_left_out = None if not timing_valid else round(float(left_vel), 6)
        canonical_right_out = None if not timing_valid else round(float(right_vel), 6)
        canonical_linear_out = None if not timing_valid else round(float(v_linear), 6)
        canonical_yaw_out = None if not timing_valid else round(float(yaw_rate), 6)

        out = {
            "available": True,
            "canonical_available": bool(timing_valid),
            "control_mode": mode_name,
            "observation_context": str(observation_context or "NORMAL"),
            "pipeline_model": "KIT0085_QUADRATURE",
            "source_truth": "QUADRATURE_SIGNED_PULSE_DELTA",
            "motion_state": str(motion_state or "UNKNOWN"),
            "canonical_state": str(canonical_state),
            "snapshot_age_s": round(age_s, 6),
            "measurement_timestamp_s": round(snap_ts, 6) if snap_ts > 0.0 else None,
            "publication_timestamp_s": (
                round(published_at, 6) if published_at > 0.0 else None
            ),
            "publication_delay_s": (
                round(publication_delay_s, 6)
                if publication_delay_s is not None
                else None
            ),
            "snapshot_health": snap_health,
            "snapshot_stale": bool(stale_snapshot),
            "timing_valid": bool(timing_valid),
            "timing_error": str(timing_error),
            "timing_gap_s": round(float(timing_gap_s), 6),
            "timing_gap_threshold_s": round(float(timing_gap_invalid_threshold), 6),
            "timing_gap_count": int(self._timing_gap_count),
            "motion_timing_gap_count": int(self._motion_timing_gap_count),
            "idle_timing_gap_count": int(self._idle_timing_gap_count),
            "last_timing_gap": dict(self._last_timing_gap),
            "motor_off": bool(motor_off),
            "cmd_idle": bool(cmd_idle),
            "idle_expected": bool(idle_expected),
            "idle_false_pulse": bool(idle_false_pulse),
            "idle_noise_detection": bool(idle_false_pulse),
            "idle_noise_score": round(float(idle_noise_score), 6),
            "side_asymmetry": round(float(side_asymmetry), 6),
            "side_asymmetry_raw": round(float(side_asymmetry_raw), 6),
            "side_asymmetry_stable": round(float(side_asymmetry_stable), 6),
            "side_asymmetry_command_residual": round(float(side_asymmetry_command_residual), 6),
            "side_asymmetry_command_expected": bool(side_asymmetry_command_expected),
            "side_asymmetry_comparable": bool(side_asymmetry_comparable),
            "expected_wheel_velocity": {
                "left_mps": round(float(expected_left_mps), 6),
                "right_mps": round(float(expected_right_mps), 6),
            },
            "normalized_wheel_gain": {
                "left": round(float(normalized_gain_left), 6),
                "right": round(float(normalized_gain_right), 6),
            },
            "asymmetry_score": round(float(side_asymmetry), 6),
            "side_asymmetry_excessive": bool(side_asymmetry_excessive),
            "side_asymmetry_critical": bool(side_asymmetry_critical),
            "side_ratio_lr_abs": round(float(vel_abs_l_raw / max(vel_abs_r_raw, 1e-6)), 6),
            "side_ratio_rl_abs": round(float(vel_abs_r_raw / max(vel_abs_l_raw, 1e-6)), 6),
            "left_right_coherence": round(float(left_right_coherence), 6),
            "forward_reliability": round(float(forward_reliability), 6),
            "rotate_reliability": round(float(rotate_reliability), 6),
            "backward_commanded": bool(backward_commanded),
            "backward_consistent": bool(backward_consistent),
            "direction_switch_recent": bool(direction_switch_recent),
            "pwm_symmetry_expected": bool(pwm_symmetry_expected),
            "pwm_symmetry_abs_delta": round(float(pwm_symmetry_abs_delta), 6),
            "symmetry_violation_instant": bool(symmetry_violation_instant),
            "symmetry_dropout_side_instant": str(dropout_side_instant),
            "symmetry_fault_active": bool(symmetry_fault_active),
            "symmetry_fault_side": str(symmetry_fault_side),
            "symmetry_fault_acc_s": round(float(self._symmetry_fault_acc_s), 6),
            "timing_ratio": round(float(timing_ratio), 6),
            "timing_quality": round(float(timing_quality), 6),
            "raw_measurement": {
                "velocity": {
                    "left_mps": round(float(v_l_raw), 6),
                    "right_mps": round(float(v_r_raw), 6),
                    "left_unsigned_mps": round(float(left_vel_unsigned), 6),
                    "right_unsigned_mps": round(float(right_vel_unsigned), 6),
                    "linear_mps": round(float(v_linear_raw), 6),
                    "yaw_rate_rad_s": round(float(yaw_rate_raw), 6),
                },
                "distance_delta": {
                    "left_m": round(float(delta_l_dist), 6),
                    "right_m": round(float(delta_r_dist), 6),
                    "average_m": round(float(0.5 * (delta_l_dist + delta_r_dist)), 6),
                },
                "pulses_delta": {
                    "left": int(dp_l_window),
                    "right": int(dp_r_window),
                    "dt_snapshot_s": round(float(dt_snap), 6),
                    "dt_control_window_s": round(float(dt_control_window), 6),
                    "dt_aggregation_window_s": round(float(agg_dt), 6),
                    "left_instant": int(dp_l_inst),
                    "right_instant": int(dp_r_inst),
                    "left_control_window": int(dp_l),
                    "right_control_window": int(dp_r),
                    "window_start_ts": round(float(window_start.get("ts", window_ts)), 6),
                    "window_end_ts": round(float(window_end.get("ts", window_ts)), 6),
                    "left_count_start": int(window_start.get("left_pulses", left_pulses)),
                    "left_count_end": int(window_end.get("left_pulses", left_pulses)),
                    "right_count_start": int(window_start.get("right_pulses", right_pulses)),
                    "right_count_end": int(window_end.get("right_pulses", right_pulses)),
                },
            },
            "direction_debug": {
                "left_direction": round(float(left_dir), 4),
                "right_direction": round(float(right_dir), 4),
                "left_source": left_dir_src,
                "right_source": right_dir_src,
                "left_confident": bool(left_dir_conf),
                "right_confident": bool(right_dir_conf),
                "left_unresolved_pulses": int(unresolved_l),
                "right_unresolved_pulses": int(unresolved_r),
                "direction_uncertain": bool(direction_uncertain),
            },
            "canonical_velocity": {
                "valid": bool(timing_valid),
                "invalid_reason": str(timing_error),
                "left_mps": canonical_left_out,
                "right_mps": canonical_right_out,
                "linear_mps": canonical_linear_out,
                "yaw_rate_rad_s": canonical_yaw_out,
                "left_raw_mps": round(float(v_l_raw), 6),
                "right_raw_mps": round(float(v_r_raw), 6),
            },
            "canonical_distance": {
                "left_m": round(float(self._canonical_left_distance), 6),
                "right_m": round(float(self._canonical_right_distance), 6),
                "average_m": round(float(self._canonical_distance_total), 6),
                "delta_m": round(float(canonical_delta_avg), 6),
                "left_delta_m": round(float(canonical_delta_l), 6),
                "right_delta_m": round(float(canonical_delta_r), 6),
                "model": str(distance_model),
                "dominant_side": None,
            },
            "distance_canonical_m": round(float(self._canonical_distance_total), 6),
            "distance_delta_canonical_m": round(float(canonical_delta_avg), 6),
            "theta_measurement_reliable": bool(theta_measurement_reliable),
            "v_l_canonical": canonical_left_out,
            "v_r_canonical": canonical_right_out,
            "left_trust": round(float(_clamp(left_trust, 0.0, 1.0)), 4),
            "right_trust": round(float(_clamp(right_trust, 0.0, 1.0)), 4),
            "combined_trust": round(float(_clamp(combined_trust, 0.0, 1.0)), 4),
            "trust_degraded": bool(combined_trust < 0.8),
            "ekf_usage_mode": str(ekf_usage_mode),
            "ekf_usage_reason": str(ekf_usage_reason),
            "ekf_covariance_scale_hint": round(float(ekf_covariance_scale_hint), 4),
            "ekf_weight_hint": round(float(ekf_weight_hint), 6),
            "flags": flags,
            "anomaly_active": bool(anomaly_active),
            "pulses_delta": {
                "left": int(dp_l_window),
                "right": int(dp_r_window),
                "dt_snapshot_s": round(float(dt_snap), 6),
                "dt_control_window_s": round(float(dt_control_window), 6),
                "dt_aggregation_window_s": round(float(agg_dt), 6),
                "left_instant": int(dp_l_inst),
                "right_instant": int(dp_r_inst),
                "left_control_window": int(dp_l),
                "right_control_window": int(dp_r),
                "window_start_ts": round(float(window_start.get("ts", window_ts)), 6),
                "window_end_ts": round(float(window_end.get("ts", window_ts)), 6),
                "left_count_start": int(window_start.get("left_pulses", left_pulses)),
                "left_count_end": int(window_end.get("left_pulses", left_pulses)),
                "right_count_start": int(window_start.get("right_pulses", right_pulses)),
                "right_count_end": int(window_end.get("right_pulses", right_pulses)),
            },
            "ekf_input": {
                "v_l_mps": canonical_left_out,
                "v_r_mps": canonical_right_out,
                "theta_reliable": bool(theta_measurement_reliable),
                "usage_mode": str(ekf_usage_mode),
                "usage_reason": str(ekf_usage_reason),
                "confidence": round(float(_clamp(combined_trust, 0.0, 1.0)), 4),
                "covariance_scale": round(float(ekf_covariance_scale_hint), 4),
                "weight_hint": round(float(ekf_weight_hint), 6),
            },
            "qa": {
                "left_right_coherence": round(float(left_right_coherence), 6),
                "asymmetry_score": round(float(side_asymmetry), 6),
                "side_asymmetry_command_residual": round(float(side_asymmetry_command_residual), 6),
                "side_asymmetry_command_expected": bool(side_asymmetry_command_expected),
                "side_asymmetry_comparable": bool(side_asymmetry_comparable),
                "forward_reliability": round(float(forward_reliability), 6),
                "rotate_reliability": round(float(rotate_reliability), 6),
                "pwm_symmetry_expected": bool(pwm_symmetry_expected),
                "symmetry_violation_instant": bool(symmetry_violation_instant),
                "symmetry_fault_active": bool(symmetry_fault_active),
                "symmetry_fault_side": str(symmetry_fault_side),
                "idle_noise_detection": bool(idle_false_pulse),
                "stale_snapshot": bool(stale_snapshot),
                "ekf_usage_mode": str(ekf_usage_mode),
            },
            "flow_profile": {
                "mode": mode_name,
                "idle_noise_scale": round(float(flow_profile.idle_noise_scale), 4),
                "low_speed_scale": round(float(flow_profile.low_speed_scale), 4),
                "direction_switch_grace_s": round(float(flow_profile.direction_switch_grace_s), 4),
                "low_speed_allow_linear": bool(flow_profile.low_speed_allow_linear),
                "low_speed_linear_min_trust": round(float(flow_profile.low_speed_linear_min_trust), 4),
                "pulse_aggregation_window_s": round(float(self.cfg.pulse_aggregation_window_s), 4),
            },
            "diagnostic_limits": {
                "velocity_noise_threshold": round(float(velocity_noise_threshold), 6),
                "low_speed_threshold": round(float(low_speed), 6),
                "rotate_yaw_rate_threshold": round(float(rotate_yaw_rate_threshold), 6),
                "asymmetry_warn_threshold": round(float(asymmetry_warn_threshold), 6),
                "asymmetry_critical_threshold": round(float(asymmetry_critical_threshold), 6),
                "timing_gap_invalid_s": round(float(timing_gap_invalid_threshold), 6),
                "symmetry_pwm_delta_max": round(float(self.cfg.symmetry_pwm_delta_max), 6),
                "symmetry_pwm_active_min": round(float(self.cfg.symmetry_pwm_active_min), 6),
                "symmetry_zero_velocity_max_mps": round(float(self.cfg.symmetry_zero_velocity_max_mps), 6),
                "symmetry_active_velocity_min_mps": round(float(self.cfg.symmetry_active_velocity_min_mps), 6),
                "symmetry_fault_confirm_s": round(float(self.cfg.symmetry_fault_confirm_s), 6),
            },
        }
        self._last_out = out
        return out

    @property
    def last_status(self) -> Dict[str, Any]:
        return dict(self._last_out or {})


@dataclass
class HeadingTurnConfig:
    kp_heading: float = 0.045
    min_omega_rad_s: float = 0.22
    max_omega_rad_s: float = 1.0
    omega_ramp_rad_s2: float = 3.0
    breakout_min_omega_rad_s: float = 0.28
    breakout_hold_s: float = 0.30
    asymmetry_comp_gain: float = 0.18
    asymmetry_comp_max: float = 0.25
    approach_window_deg: float = 30.0
    settle_tolerance_deg: float = 1.8
    settle_omega_rad_s: float = 0.12
    settle_time_s: float = 0.25
    max_duration_s: float = 8.0
    max_drift_m: float = 0.06
    max_translation_leakage_m_per_deg: float = 0.0020
    yaw_direction_match_accept_min: float = 0.45
    stall_min_error_deg: float = 10.0
    stall_min_command_rad_s: float = 0.28
    stall_min_measured_rad_s: float = 0.08
    stall_timeout_s: float = 0.70
    imu_only_timeout_s: float = 0.80
    lidar_confidence_min: float = 0.20
    default_speed_level: int = 1
    speed_levels: Dict[int, Dict[str, Any]] = field(default_factory=dict)
    runtime_rotate_levels_autoload: bool = True
    runtime_rotate_levels_path: str = "runtime/calibration/rotate_speed_levels_latest.json"
    pivot_primitive_enabled: bool = True
    pivot_in_place: bool = False
    pivot_track_min_mps: float = 0.150
    pivot_track_max_mps: float = 0.150
    stop_prediction_horizon_s: float = 0.07


class HeadingTurnController:
    """
    Reusable heading executor for target yaw alignment.
    Emits a bounded one-track pivot primitive while keeping settle metrics and leakage monitoring.
    """
    TERMINAL_STATUSES = {"DONE", "TIMEOUT", "DRIFT_ABORT", "STALL_ABORT", "LIDAR_ABORT", "SAFETY_ABORT"}

    @staticmethod
    def _default_speed_levels() -> Dict[int, Dict[str, Any]]:
        return {
            0: {
                "label": "fine",
                "target_deg_s": 18.0,
                "range_deg_s": [12.0, 24.0],
            },
            1: {
                "label": "min",
                "target_deg_s": 45.0,
                "range_deg_s": [40.0, 60.0],
            },
            2: {
                "label": "medium",
                "target_deg_s": 90.0,
                "range_deg_s": [80.0, 120.0],
            },
            3: {
                "label": "max",
                "target_deg_s": 180.0,
                "range_deg_s": [160.0, 220.0],
            },
        }

    @staticmethod
    def _deg_to_rad(value_deg_s: float) -> float:
        return float(math.radians(float(value_deg_s)))

    @staticmethod
    def _rad_to_deg(value_rad_s: float) -> float:
        return float(math.degrees(float(value_rad_s)))

    def _normalize_speed_level_record(self, level: int, raw: Dict[str, Any], *, source: str) -> Dict[str, Any]:
        row = dict(raw or {})
        target_deg = _safe_float(row.get("target_deg_s"), math.nan)
        if not math.isfinite(target_deg):
            target_rad = _safe_float(row.get("target_rad_s"), math.nan)
            if math.isfinite(target_rad):
                target_deg = self._rad_to_deg(target_rad)
        if not math.isfinite(target_deg) or target_deg <= 0.0:
            target_deg = 45.0

        range_list = row.get("range_deg_s")
        range_min_deg = math.nan
        range_max_deg = math.nan
        if isinstance(range_list, (list, tuple)) and len(range_list) >= 2:
            range_min_deg = _safe_float(range_list[0], math.nan)
            range_max_deg = _safe_float(range_list[1], math.nan)
        if not math.isfinite(range_min_deg):
            range_min_deg = _safe_float(row.get("range_min_deg_s"), math.nan)
        if not math.isfinite(range_max_deg):
            range_max_deg = _safe_float(row.get("range_max_deg_s"), math.nan)

        if not math.isfinite(range_min_deg) or not math.isfinite(range_max_deg):
            # Conservative fallback band around target.
            range_min_deg = max(5.0, target_deg * 0.85)
            range_max_deg = max(range_min_deg + 1.0, target_deg * 1.15)

        if range_min_deg > range_max_deg:
            range_min_deg, range_max_deg = range_max_deg, range_min_deg

        range_min_deg = max(1.0, float(range_min_deg))
        range_max_deg = max(range_min_deg + 0.5, float(range_max_deg))
        target_deg = _clamp(float(target_deg), range_min_deg, range_max_deg)

        out = {
            "level": int(level),
            "label": str(row.get("label") or f"level_{int(level)}"),
            "target_deg_s": float(target_deg),
            "range_deg_s": [float(range_min_deg), float(range_max_deg)],
            "target_rad_s": float(self._deg_to_rad(target_deg)),
            "range_min_rad_s": float(self._deg_to_rad(range_min_deg)),
            "range_max_rad_s": float(self._deg_to_rad(range_max_deg)),
            "source": str(source or "config"),
        }
        return out

    def _normalize_speed_levels(self, levels_cfg: Dict[str, Any], *, source: str) -> Dict[int, Dict[str, Any]]:
        out: Dict[int, Dict[str, Any]] = {}
        for raw_key, raw_value in dict(levels_cfg or {}).items():
            lvl = _safe_int(raw_key, -1)
            if lvl < 0:
                continue
            if not isinstance(raw_value, dict):
                continue
            out[lvl] = self._normalize_speed_level_record(lvl, raw_value, source=source)
        return out

    def _resolve_speed_level(self, value: Any, *, fallback: int = 1) -> int:
        level_keys = sorted(int(k) for k in self.speed_levels.keys())
        if not level_keys:
            return 1
        lvl = _safe_int(value, int(fallback))
        if lvl in self.speed_levels:
            return int(lvl)
        if lvl <= level_keys[0]:
            return int(level_keys[0])
        if lvl >= level_keys[-1]:
            return int(level_keys[-1])
        return int(min(level_keys, key=lambda x: abs(int(x) - int(lvl))))

    def _load_runtime_rotate_levels(self, rel_or_abs_path: str) -> Dict[int, Dict[str, Any]]:
        raw_path = str(rel_or_abs_path or "").strip()
        if not raw_path:
            return {}
        path = Path(raw_path)
        if not path.is_absolute():
            path = Path(__file__).resolve().parents[1] / path
        try:
            if (not path.exists()) or (not path.is_file()):
                return {}
            raw = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return {}

        if not isinstance(raw, dict):
            return {}
        if not bool(raw.get("success", False)):
            return {}

        rotate_levels = dict(raw.get("rotate_speed_levels") or {})
        levels_cfg = dict(rotate_levels.get("levels") or {})
        normalized = self._normalize_speed_levels(levels_cfg, source=f"runtime:{path}")
        if not normalized:
            return {}

        self.runtime_speed_levels_path = str(path)
        default_level_from_runtime = _safe_int(rotate_levels.get("default_level"), self.default_speed_level)
        if default_level_from_runtime in normalized:
            self.default_speed_level = int(default_level_from_runtime)
        return normalized

    def __init__(self, track_width_m: float, cfg: Optional[Dict[str, Any]] = None):
        cfg = dict(cfg or {})
        default_levels = self._normalize_speed_levels(self._default_speed_levels(), source="default")
        configured_levels = self._normalize_speed_levels(cfg.get("speed_levels") or {}, source="config")
        merged_levels = dict(default_levels)
        merged_levels.update(configured_levels)

        self.cfg = HeadingTurnConfig(
            kp_heading=_safe_float(cfg.get("kp_heading"), 0.045),
            min_omega_rad_s=_safe_float(cfg.get("min_omega_rad_s"), 0.22),
            max_omega_rad_s=_safe_float(cfg.get("max_omega_rad_s"), 1.0),
            omega_ramp_rad_s2=max(0.1, _safe_float(cfg.get("omega_ramp_rad_s2"), 3.0)),
            breakout_min_omega_rad_s=max(0.02, _safe_float(cfg.get("breakout_min_omega_rad_s"), 0.28)),
            breakout_hold_s=max(0.0, _safe_float(cfg.get("breakout_hold_s"), 0.30)),
            asymmetry_comp_gain=max(0.0, _safe_float(cfg.get("asymmetry_comp_gain"), 0.18)),
            asymmetry_comp_max=_clamp(_safe_float(cfg.get("asymmetry_comp_max"), 0.25), 0.0, 0.6),
            approach_window_deg=max(4.0, _safe_float(cfg.get("approach_window_deg"), 30.0)),
            settle_tolerance_deg=_safe_float(cfg.get("settle_tolerance_deg"), 1.8),
            settle_omega_rad_s=_safe_float(cfg.get("settle_omega_rad_s"), 0.12),
            settle_time_s=_safe_float(cfg.get("settle_time_s"), 0.25),
            max_duration_s=_safe_float(cfg.get("max_duration_s"), 8.0),
            max_drift_m=_safe_float(cfg.get("max_drift_m"), 0.06),
            max_translation_leakage_m_per_deg=max(
                1e-6,
                _safe_float(cfg.get("max_translation_leakage_m_per_deg"), 0.0020),
            ),
            yaw_direction_match_accept_min=_clamp(
                _safe_float(cfg.get("yaw_direction_match_accept_min"), 0.45),
                0.0,
                1.0,
            ),
            stall_min_error_deg=max(0.2, _safe_float(cfg.get("stall_min_error_deg"), 10.0)),
            stall_min_command_rad_s=max(0.02, _safe_float(cfg.get("stall_min_command_rad_s"), 0.28)),
            stall_min_measured_rad_s=max(0.0, _safe_float(cfg.get("stall_min_measured_rad_s"), 0.08)),
            stall_timeout_s=max(0.1, _safe_float(cfg.get("stall_timeout_s"), 0.70)),
            imu_only_timeout_s=max(0.0, _safe_float(cfg.get("imu_only_timeout_s"), 0.80)),
            lidar_confidence_min=_clamp(_safe_float(cfg.get("lidar_confidence_min"), 0.20), 0.0, 1.0),
            default_speed_level=_safe_int(cfg.get("default_speed_level"), 1),
            speed_levels=merged_levels,
            runtime_rotate_levels_autoload=_safe_bool(cfg.get("runtime_rotate_levels_autoload"), True),
            runtime_rotate_levels_path=str(
                cfg.get("runtime_rotate_levels_path") or "runtime/calibration/rotate_speed_levels_latest.json"
            ),
            pivot_primitive_enabled=_safe_bool(cfg.get("pivot_primitive_enabled"), True),
            pivot_in_place=_safe_bool(cfg.get("pivot_in_place"), False),
            pivot_track_min_mps=max(0.0, _safe_float(cfg.get("pivot_track_min_mps"), 0.150)),
            pivot_track_max_mps=max(0.0, _safe_float(cfg.get("pivot_track_max_mps"), 0.150)),
            stop_prediction_horizon_s=_clamp(
                _safe_float(cfg.get("stop_prediction_horizon_s"), 0.07),
                0.0,
                0.50,
            ),
        )
        self.track_width_m = max(0.01, float(track_width_m))
        self.speed_levels: Dict[int, Dict[str, Any]] = dict(self.cfg.speed_levels or {})
        self.default_speed_level = self._resolve_speed_level(self.cfg.default_speed_level, fallback=1)
        self.runtime_speed_levels_path = ""
        if bool(self.cfg.runtime_rotate_levels_autoload):
            runtime_levels = self._load_runtime_rotate_levels(self.cfg.runtime_rotate_levels_path)
            if runtime_levels:
                self.speed_levels = dict(runtime_levels)
                if 0 in default_levels and 0 not in self.speed_levels:
                    self.speed_levels[0] = dict(default_levels[0])
                self.default_speed_level = self._resolve_speed_level(self.default_speed_level, fallback=1)

        if self.default_speed_level not in self.speed_levels:
            self.default_speed_level = self._resolve_speed_level(1, fallback=1)

        self.active = False
        self.target_heading_deg = 0.0
        self.start_heading_deg = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.started_at = 0.0
        self.settle_started_at: Optional[float] = None
        self.source = "STATE"
        self.active_speed_level = int(self.default_speed_level)
        self.active_speed_profile: Dict[str, Any] = dict(self.speed_levels.get(self.active_speed_level, {}) or {})
        self._active_cruise_omega_rad_s = _safe_float(
            self.active_speed_profile.get("target_rad_s"),
            self.cfg.min_omega_rad_s,
        )
        self._active_min_omega_rad_s = _safe_float(
            self.active_speed_profile.get("range_min_rad_s"),
            self.cfg.min_omega_rad_s,
        )
        self._active_max_omega_rad_s = _safe_float(
            self.active_speed_profile.get("range_max_rad_s"),
            self.cfg.max_omega_rad_s,
        )

        self._sample_count = 0
        self._opposite_sign_count = 0
        self._symmetry_acc = 0.0
        self._yaw_direction_sample_count = 0
        self._yaw_direction_match_count = 0
        self._stall_elapsed_s = 0.0
        self._last_omega_ref = 0.0
        self._last_pose_omega_ref = 0.0
        self._last_heading_for_rate: Optional[float] = None
        self._last_side_imbalance = 0.0
        self._last_omega_cmd_pre_ramp = 0.0
        self._last_measured_omega_source = "none"
        self._integrated_heading_progress_rad = 0.0
        self._last_control_heading_deg = 0.0
        self._last_control_heading_source = "ekf_pose"
        self._imu_only_started_at: Optional[float] = None
        self._imu_only_last_reason = ""
        self._imu_only_active = False
        self._imu_only_elapsed_s = 0.0
        self._last_cmd = {"v_target": 0.0, "omega_target": 0.0}
        self._last_track_reference = {"left_mps": None, "right_mps": None}
        self._predictive_stop_hold = False
        self._last_predicted_heading_error_deg = 0.0
        self._last_result: Dict[str, Any] = {}

    def _pivot_track_reference_for_omega(self, omega_cmd: float) -> Dict[str, Any]:
        if not bool(self.cfg.pivot_primitive_enabled):
            return {
                "enabled": False,
                "left_mps": None,
                "right_mps": None,
                "v_target": 0.0,
                "omega_target": float(omega_cmd),
                "track_speed_mps": 0.0,
                "turn_primitive": "",
                "reason": "disabled",
            }
        omega = _safe_float(omega_cmd, 0.0)
        if abs(float(omega)) <= 1e-9:
            return {
                "enabled": False,
                "left_mps": None,
                "right_mps": None,
                "v_target": 0.0,
                "omega_target": 0.0,
                "track_speed_mps": 0.0,
                "turn_primitive": "",
                "reason": "zero_omega",
            }

        min_mps = max(0.0, float(self.cfg.pivot_track_min_mps))
        max_mps = max(min_mps, float(self.cfg.pivot_track_max_mps))
        if bool(self.cfg.pivot_in_place):
            speed_mps = _clamp(abs(float(omega)) * float(self.track_width_m) * 0.5, min_mps, max_mps)
            if omega > 0.0:
                left_mps = -float(speed_mps)
                right_mps = float(speed_mps)
            else:
                left_mps = float(speed_mps)
                right_mps = -float(speed_mps)
            primitive = TURN_PRIMITIVE_IN_PLACE_ROTATE
            reason = "heading_decision_in_place_pivot"
        else:
            speed_mps = _clamp(abs(float(omega)) * float(self.track_width_m), min_mps, max_mps)
            if omega > 0.0:
                left_mps = 0.0
                right_mps = float(speed_mps)
            else:
                left_mps = float(speed_mps)
                right_mps = 0.0
            primitive = TURN_PRIMITIVE_ONE_TRACK_PIVOT
            reason = "heading_decision_pivot"
        v_target = 0.5 * (float(left_mps) + float(right_mps))
        omega_target = (float(right_mps) - float(left_mps)) / max(0.01, float(self.track_width_m))
        return {
            "enabled": True,
            "left_mps": float(left_mps),
            "right_mps": float(right_mps),
            "v_target": float(v_target),
            "omega_target": float(omega_target),
            "track_speed_mps": float(speed_mps),
            "turn_primitive": str(primitive),
            "reason": str(reason),
        }

    def start(
        self,
        *,
        target_heading_deg: float,
        current_heading_deg: float,
        pose_x: float,
        pose_y: float,
        source: str = "STATE",
        settle_tolerance_deg: Optional[float] = None,
        settle_time_s: Optional[float] = None,
        max_duration_s: Optional[float] = None,
        speed_level: Optional[int] = None,
    ) -> None:
        self.active = True
        self.target_heading_deg = float(target_heading_deg) % 360.0
        self.start_heading_deg = float(current_heading_deg)
        self.start_x = float(pose_x)
        self.start_y = float(pose_y)
        self.started_at = time.monotonic()
        self.settle_started_at = None
        self.source = str(source or "STATE")
        self.active_speed_level = self._resolve_speed_level(
            self.default_speed_level if speed_level is None else speed_level,
            fallback=self.default_speed_level,
        )
        self.active_speed_profile = dict(self.speed_levels.get(self.active_speed_level, {}) or {})
        self._active_cruise_omega_rad_s = _safe_float(
            self.active_speed_profile.get("target_rad_s"),
            self.cfg.min_omega_rad_s,
        )
        self._active_min_omega_rad_s = _safe_float(
            self.active_speed_profile.get("range_min_rad_s"),
            self.cfg.min_omega_rad_s,
        )
        self._active_max_omega_rad_s = _safe_float(
            self.active_speed_profile.get("range_max_rad_s"),
            max(self.cfg.max_omega_rad_s, self._active_cruise_omega_rad_s),
        )
        if self._active_max_omega_rad_s < self._active_min_omega_rad_s:
            self._active_max_omega_rad_s = self._active_min_omega_rad_s

        if settle_tolerance_deg is not None:
            self.cfg.settle_tolerance_deg = max(0.2, float(settle_tolerance_deg))
        if settle_time_s is not None:
            self.cfg.settle_time_s = max(0.01, float(settle_time_s))
        if max_duration_s is not None:
            self.cfg.max_duration_s = max(0.1, float(max_duration_s))

        self._sample_count = 0
        self._opposite_sign_count = 0
        self._symmetry_acc = 0.0
        self._yaw_direction_sample_count = 0
        self._yaw_direction_match_count = 0
        self._stall_elapsed_s = 0.0
        self._last_omega_ref = 0.0
        self._last_pose_omega_ref = 0.0
        self._last_heading_for_rate = float(current_heading_deg)
        self._last_side_imbalance = 0.0
        self._last_omega_cmd_pre_ramp = 0.0
        self._last_measured_omega_source = "none"
        self._integrated_heading_progress_rad = 0.0
        self._last_control_heading_deg = float(current_heading_deg)
        self._last_control_heading_source = "ekf_pose"
        self._imu_only_started_at = None
        self._imu_only_last_reason = ""
        self._imu_only_active = False
        self._imu_only_elapsed_s = 0.0
        self._last_cmd = {"v_target": 0.0, "omega_target": 0.0}
        self._last_track_reference = {"left_mps": None, "right_mps": None}
        self._predictive_stop_hold = False
        self._last_predicted_heading_error_deg = 0.0

    def _lidar_heading_reference_ok(self, lidar_status: Optional[Dict[str, Any]]) -> tuple[bool, str]:
        if lidar_status is None:
            return True, "status_unavailable"
        status = dict(lidar_status or {})
        if not status:
            return False, "missing_status"

        conf_now = _safe_float(status.get("confidence"), math.nan)
        conf_candidate = _safe_float(status.get("candidate_confidence"), math.nan)
        status_now = str(status.get("status", "") or "").strip().lower()
        delivery_status = str(status.get("delivery_status", "") or "").strip().lower()
        localization_status = str(status.get("localization_status", "") or "").strip().lower()
        ekf_status = str(status.get("ekf_status", "") or "").strip().lower()
        candidate_available = bool(status.get("candidate_available", False))
        applied = bool(status.get("applied", False) or ekf_status == "applied")

        if math.isfinite(conf_now) and conf_now < float(self.cfg.lidar_confidence_min):
            return False, "low_confidence_now"
        if candidate_available and math.isfinite(conf_candidate) and conf_candidate < float(self.cfg.lidar_confidence_min):
            return False, "low_confidence_candidate"
        if "rejected_low_confidence" in (status_now, ekf_status):
            return False, "rejected_low_confidence"
        if status_now in ("missing", "stale", "rejected_invalid", "rejected_nis"):
            if not candidate_available and not applied:
                return False, status_now
        if delivery_status in ("stale",) and not candidate_available and not applied:
            return False, "delivery_stale"
        if localization_status and localization_status not in ("tracking", "localized"):
            if not candidate_available and not applied:
                return False, f"localization_{localization_status}"

        if applied:
            return True, "applied"
        if candidate_available:
            return True, "candidate_available"
        if status_now == "accepted":
            return True, "accepted_pending"
        return False, "no_lidar_update"

    def cancel(self, reason: str = "CANCELLED") -> None:
        if not self.active:
            return
        raw_reason = str(reason or "").strip().upper()
        if raw_reason not in self.TERMINAL_STATUSES:
            raw_reason = "SAFETY_ABORT"
        self._last_result = {
            "status": raw_reason,
            "accepted": False,
            "target_heading_deg": round(self.target_heading_deg, 3),
            "heading_error_deg": None,
            "drift_cm": None,
            "translation_leakage": None,
            "abort_reason": str(reason or ""),
        }
        self.active = False
        self._last_cmd = {"v_target": 0.0, "omega_target": 0.0}
        self._last_track_reference = {"left_mps": None, "right_mps": None}

    def _build_result(
        self,
        *,
        status: str,
        current_heading_deg: float,
        pose_x: float,
        pose_y: float,
        elapsed_s: float,
    ) -> Dict[str, Any]:
        ekf_delta_deg = _normalize_angle_deg(current_heading_deg - self.start_heading_deg)
        ekf_heading_error = _angle_error_deg(self.target_heading_deg, current_heading_deg)
        control_heading_deg = _safe_float(self._last_control_heading_deg, current_heading_deg)
        delta_deg = _normalize_angle_deg(control_heading_deg - self.start_heading_deg)
        heading_error = _angle_error_deg(self.target_heading_deg, control_heading_deg)
        dx = float(pose_x) - self.start_x
        dy = float(pose_y) - self.start_y
        drift_m = math.hypot(dx, dy)
        drift_cm = drift_m * 100.0
        leak = drift_m / max(abs(delta_deg), 1e-3)
        opposite_sign_ratio = self._opposite_sign_count / max(1, self._sample_count)
        wheel_symmetry_score = self._symmetry_acc / max(1, self._sample_count)
        yaw_direction_match_ratio = self._yaw_direction_match_count / max(1, self._yaw_direction_sample_count)
        leakage_limit = max(1e-6, float(self.cfg.max_translation_leakage_m_per_deg))
        drift_ok = drift_m <= float(self.cfg.max_drift_m) or leak <= leakage_limit
        direction_quality_ok = (
            opposite_sign_ratio >= 0.55
            or yaw_direction_match_ratio >= float(self.cfg.yaw_direction_match_accept_min)
        )

        accepted = (
            status == "DONE"
            and abs(heading_error) <= self.cfg.settle_tolerance_deg
            and drift_ok
            and direction_quality_ok
        )

        return {
            "status": status,
            "accepted": bool(accepted),
            "target_heading_deg": round(float(self.target_heading_deg), 3),
            "achieved_yaw": round(float(delta_deg), 3),
            "heading_error": round(float(heading_error), 3),
            "heading_control_source": str(self._last_control_heading_source or "ekf_pose"),
            "ekf_achieved_yaw": round(float(ekf_delta_deg), 3),
            "ekf_heading_error": round(float(ekf_heading_error), 3),
            "drift_cm": round(float(drift_cm), 3),
            "translation_leakage": round(float(leak), 6),
            "translation_leakage_limit": round(float(leakage_limit), 6),
            "opposite_sign_ratio": round(float(opposite_sign_ratio), 4),
            "yaw_direction_match_ratio": round(float(yaw_direction_match_ratio), 4),
            "wheel_symmetry_score": round(float(wheel_symmetry_score), 4),
            "stall_elapsed_s": round(float(self._stall_elapsed_s), 4),
            "measured_omega_rad_s": round(float(self._last_omega_ref), 6),
            "side_imbalance": round(float(self._last_side_imbalance), 6),
            "duration_s": round(float(elapsed_s), 3),
            "speed_level": int(self.active_speed_level),
            "speed_profile": {
                "label": str(self.active_speed_profile.get("label") or f"level_{int(self.active_speed_level)}"),
                "target_deg_s": round(
                    float(_safe_float(self.active_speed_profile.get("target_deg_s"), self._rad_to_deg(self._active_cruise_omega_rad_s))),
                    4,
                ),
                "target_rad_s": round(float(self._active_cruise_omega_rad_s), 6),
                "range_deg_s": [
                    round(
                        float(
                            _safe_float(
                                (self.active_speed_profile.get("range_deg_s") or [0.0, 0.0])[0],
                                self._rad_to_deg(self._active_min_omega_rad_s),
                            )
                        ),
                        4,
                    ),
                    round(
                        float(
                            _safe_float(
                                (self.active_speed_profile.get("range_deg_s") or [0.0, 0.0])[1],
                                self._rad_to_deg(self._active_max_omega_rad_s),
                            )
                        ),
                        4,
                    ),
                ],
            },
        }

    def tick(
        self,
        *,
        current_heading_deg: float,
        pose_x: float,
        pose_y: float,
        v_l_raw: float,
        v_r_raw: float,
        gyro_z_rad_s: Optional[float] = None,
        lidar_status: Optional[Dict[str, Any]] = None,
        odometry_mode: str = "LIDAR_FIRST",
        dt: float,
        now: Optional[float] = None,
    ) -> Optional[Dict[str, Any]]:
        if not self.active:
            return None

        now = time.monotonic() if now is None else float(now)
        dt = max(0.0, float(dt))
        elapsed_s = max(0.0, now - self.started_at)
        pose_omega_ref = 0.0
        if self._last_heading_for_rate is not None and dt > 1e-3:
            heading_step_deg = _normalize_angle_deg(float(current_heading_deg) - float(self._last_heading_for_rate))
            pose_omega_ref = math.radians(float(heading_step_deg)) / max(1e-3, float(dt))
        self._last_heading_for_rate = float(current_heading_deg)
        self._last_pose_omega_ref = float(pose_omega_ref)
        gyro_omega_ref = _safe_float(gyro_z_rad_s, math.nan)
        gyro_valid = math.isfinite(gyro_omega_ref)
        measured_omega_ref = float(gyro_omega_ref if gyro_valid else pose_omega_ref)
        measured_omega_source = "gyro" if gyro_valid else "pose_rate"
        # Safety fallback: if gyro is near-zero but EKF heading clearly changes, use pose rate.
        if abs(float(measured_omega_ref)) < 0.02 and abs(float(pose_omega_ref)) > abs(float(measured_omega_ref)):
            measured_omega_ref = float(pose_omega_ref)
            measured_omega_source = "pose_rate"
        self._last_omega_ref = float(measured_omega_ref)
        self._last_measured_omega_source = str(measured_omega_source)
        self._integrated_heading_progress_rad += float(measured_omega_ref) * float(dt)
        control_heading_deg = (
            float(self.start_heading_deg)
            + math.degrees(float(self._integrated_heading_progress_rad))
        )
        self._last_control_heading_deg = float(control_heading_deg)
        self._last_control_heading_source = (
            "imu_gyro_integrated"
            if measured_omega_source == "gyro"
            else "ekf_pose_rate_integrated"
        )
        heading_error_deg = _angle_error_deg(self.target_heading_deg, control_heading_deg)

        self._sample_count += 1
        if _safe_float(v_l_raw, 0.0) * _safe_float(v_r_raw, 0.0) < 0.0:
            self._opposite_sign_count += 1
        vel_abs_l = abs(_safe_float(v_l_raw, 0.0))
        vel_abs_r = abs(_safe_float(v_r_raw, 0.0))
        symmetry = 1.0 - (abs(vel_abs_l - vel_abs_r) / max(vel_abs_l, vel_abs_r, 1e-6))
        self._symmetry_acc += _clamp(symmetry, 0.0, 1.0)
        side_imbalance = (vel_abs_l - vel_abs_r) / max(vel_abs_l + vel_abs_r, 1e-6)
        self._last_side_imbalance = float(_clamp(side_imbalance, -1.0, 1.0))

        lidar_ok = True
        lidar_reason = "not_required"
        if str(odometry_mode or "").strip().upper() == "LIDAR_FIRST":
            lidar_ok, lidar_reason = self._lidar_heading_reference_ok(lidar_status)
            if lidar_ok:
                self._imu_only_started_at = None
                self._imu_only_active = False
                self._imu_only_elapsed_s = 0.0
                self._imu_only_last_reason = ""
            else:
                if self._imu_only_started_at is None:
                    self._imu_only_started_at = float(now)
                self._imu_only_elapsed_s = max(0.0, float(now) - float(self._imu_only_started_at))
                self._imu_only_active = True
                self._imu_only_last_reason = str(lidar_reason or "lidar_unavailable")
                if self._imu_only_elapsed_s >= float(self.cfg.imu_only_timeout_s):
                    result = self._build_result(
                        status="LIDAR_ABORT",
                        current_heading_deg=current_heading_deg,
                        pose_x=pose_x,
                        pose_y=pose_y,
                        elapsed_s=elapsed_s,
                    )
                    result["imu_only_elapsed_s"] = round(float(self._imu_only_elapsed_s), 4)
                    result["imu_only_timeout_s"] = round(float(self.cfg.imu_only_timeout_s), 4)
                    result["imu_only_reason"] = str(self._imu_only_last_reason or "lidar_unavailable")
                    self._last_result = result
                    self.active = False
                    self._last_cmd = {"v_target": 0.0, "omega_target": 0.0}
                    self._last_track_reference = {"left_mps": None, "right_mps": None}
                    self._stall_elapsed_s = 0.0
                    return {"done": True, "v_target": 0.0, "omega_target": 0.0, "result": result}
        else:
            self._imu_only_started_at = None
            self._imu_only_active = False
            self._imu_only_elapsed_s = 0.0
            self._imu_only_last_reason = ""

        if (
            abs(heading_error_deg) <= self.cfg.settle_tolerance_deg
            and abs(float(measured_omega_ref)) <= self.cfg.settle_omega_rad_s
        ):
            if self.settle_started_at is None:
                self.settle_started_at = now
        else:
            self.settle_started_at = None

        drift_m = math.hypot(float(pose_x) - self.start_x, float(pose_y) - self.start_y)
        yaw_progress_deg = abs(_normalize_angle_deg(float(current_heading_deg) - self.start_heading_deg))
        translation_leakage = drift_m / max(yaw_progress_deg, 1e-3)

        if elapsed_s >= self.cfg.max_duration_s:
            result = self._build_result(
                status="TIMEOUT",
                current_heading_deg=current_heading_deg,
                pose_x=pose_x,
                pose_y=pose_y,
                elapsed_s=elapsed_s,
            )
            self._last_result = result
            self.active = False
            self._last_cmd = {"v_target": 0.0, "omega_target": 0.0}
            self._last_track_reference = {"left_mps": None, "right_mps": None}
            self._stall_elapsed_s = 0.0
            return {"done": True, "v_target": 0.0, "omega_target": 0.0, "result": result}

        leakage_abort_limit = max(1e-6, float(self.cfg.max_translation_leakage_m_per_deg)) * 2.0
        if drift_m > self.cfg.max_drift_m * 2.0 and translation_leakage > leakage_abort_limit:
            result = self._build_result(
                status="DRIFT_ABORT",
                current_heading_deg=current_heading_deg,
                pose_x=pose_x,
                pose_y=pose_y,
                elapsed_s=elapsed_s,
            )
            self._last_result = result
            self.active = False
            self._last_cmd = {"v_target": 0.0, "omega_target": 0.0}
            self._last_track_reference = {"left_mps": None, "right_mps": None}
            self._stall_elapsed_s = 0.0
            return {"done": True, "v_target": 0.0, "omega_target": 0.0, "result": result}

        if self.settle_started_at is not None and (now - self.settle_started_at) >= self.cfg.settle_time_s:
            result = self._build_result(
                status="DONE",
                current_heading_deg=current_heading_deg,
                pose_x=pose_x,
                pose_y=pose_y,
                elapsed_s=elapsed_s,
            )
            self._last_result = result
            self.active = False
            self._last_cmd = {"v_target": 0.0, "omega_target": 0.0}
            self._last_track_reference = {"left_mps": None, "right_mps": None}
            self._stall_elapsed_s = 0.0
            return {"done": True, "v_target": 0.0, "omega_target": 0.0, "result": result}

        err_abs_deg = abs(float(heading_error_deg))
        err_rad = math.radians(float(heading_error_deg))
        omega_cmd = self.cfg.kp_heading * err_rad
        omega_abs = abs(float(omega_cmd))

        # The active KIT0085 pivot contract may expose one fixed, calibrated
        # non-zero track speed. In that case scaling omega near the target has
        # no physical effect: every non-zero request maps back to the same
        # pivot. Predict the remaining yaw at the established stop horizon and
        # hold zero before entering the settle window. Once a predictive hold
        # starts, keep it until angular motion has actually settled; this
        # prevents an overshoot from immediately commanding the opposite fixed
        # pivot while the chassis is still coasting.
        prediction_horizon_s = max(0.0, float(self.cfg.stop_prediction_horizon_s))
        moving_toward_target = (
            abs(float(measured_omega_ref)) > float(self.cfg.settle_omega_rad_s)
            and float(err_rad) * float(measured_omega_ref) > 0.0
        )
        predicted_error_abs_deg = max(
            0.0,
            float(err_abs_deg)
            - math.degrees(abs(float(measured_omega_ref)) * prediction_horizon_s),
        )
        self._last_predicted_heading_error_deg = float(predicted_error_abs_deg)
        if (
            not bool(self._predictive_stop_hold)
            and prediction_horizon_s > 0.0
            and moving_toward_target
            and predicted_error_abs_deg <= float(self.cfg.settle_tolerance_deg)
        ):
            self._predictive_stop_hold = True
        elif (
            bool(self._predictive_stop_hold)
            and abs(float(measured_omega_ref)) <= float(self.cfg.settle_omega_rad_s)
        ):
            self._predictive_stop_hold = False

        if err_abs_deg <= float(self.cfg.settle_tolerance_deg):
            # Final settle region: allow full stop to satisfy settle criteria.
            omega_cmd = 0.0
        elif bool(self._predictive_stop_hold):
            omega_cmd = 0.0
        else:
            hard_max = max(
                float(self._active_max_omega_rad_s),
                0.02,
            )
            approach_window_deg = max(
                float(self.cfg.approach_window_deg),
                float(self.cfg.settle_tolerance_deg) + 0.5,
            )
            approach_ratio = _clamp(err_abs_deg / approach_window_deg, 0.0, 1.0)
            settle_floor = max(0.02, float(self.cfg.settle_omega_rad_s) * 0.5)
            dynamic_floor = settle_floor + (float(self._active_min_omega_rad_s) - settle_floor) * approach_ratio
            breakout_floor = dynamic_floor
            if (
                elapsed_s <= float(self.cfg.breakout_hold_s)
                and err_abs_deg >= float(self.cfg.stall_min_error_deg)
                and abs(float(measured_omega_ref)) <= float(self.cfg.stall_min_measured_rad_s)
            ):
                breakout_floor = max(
                    dynamic_floor,
                    min(
                        hard_max,
                        max(float(self.cfg.breakout_min_omega_rad_s), float(self._active_min_omega_rad_s)),
                    ),
                )
            omega_abs = max(omega_abs, breakout_floor)
            if err_abs_deg >= approach_window_deg:
                omega_abs = max(omega_abs, float(self._active_cruise_omega_rad_s))
            omega_abs = min(hard_max, omega_abs)
            omega_cmd = math.copysign(float(omega_abs), err_rad)
            self._last_omega_cmd_pre_ramp = float(omega_cmd)
            ramp_limit = max(0.1, float(self.cfg.omega_ramp_rad_s2))
            max_step = ramp_limit * max(1e-3, dt)
            prev_omega_cmd = _safe_float(self._last_cmd.get("omega_target", 0.0), 0.0)
            delta = float(omega_cmd) - float(prev_omega_cmd)
            if delta > max_step:
                omega_cmd = float(prev_omega_cmd) + max_step
            elif delta < -max_step:
                omega_cmd = float(prev_omega_cmd) - max_step

        measured_omega_for_stall = float(measured_omega_ref)
        if abs(float(omega_cmd)) >= 0.08 and abs(float(pose_omega_ref)) >= 0.02:
            self._yaw_direction_sample_count += 1
            if math.copysign(1.0, float(omega_cmd)) == math.copysign(1.0, float(pose_omega_ref)):
                self._yaw_direction_match_count += 1

        stall_candidate = (
            err_abs_deg >= float(self.cfg.stall_min_error_deg)
            and abs(float(omega_cmd)) >= float(self.cfg.stall_min_command_rad_s)
            and abs(float(measured_omega_for_stall)) <= float(self.cfg.stall_min_measured_rad_s)
        )
        if stall_candidate:
            self._stall_elapsed_s += dt
        else:
            self._stall_elapsed_s = max(0.0, float(self._stall_elapsed_s) - dt)

        if self._stall_elapsed_s >= float(self.cfg.stall_timeout_s):
            result = self._build_result(
                status="STALL_ABORT",
                current_heading_deg=current_heading_deg,
                pose_x=pose_x,
                pose_y=pose_y,
                elapsed_s=elapsed_s,
            )
            result["translation_leakage_live"] = round(float(translation_leakage), 6)
            self._last_result = result
            self.active = False
            self._last_cmd = {"v_target": 0.0, "omega_target": 0.0}
            self._last_track_reference = {"left_mps": None, "right_mps": None}
            self._stall_elapsed_s = 0.0
            return {"done": True, "v_target": 0.0, "omega_target": 0.0, "result": result}

        pivot_ref = self._pivot_track_reference_for_omega(float(omega_cmd))
        if bool(pivot_ref.get("enabled", False)):
            v_target_cmd = float(pivot_ref.get("v_target", 0.0))
            omega_target_cmd = float(pivot_ref.get("omega_target", 0.0))
            track_reference = {
                "left_mps": float(pivot_ref.get("left_mps", 0.0)),
                "right_mps": float(pivot_ref.get("right_mps", 0.0)),
            }
        else:
            v_target_cmd = 0.0
            omega_target_cmd = float(omega_cmd)
            track_reference = {"left_mps": None, "right_mps": None}

        cmd = {
            "done": False,
            "v_target": float(v_target_cmd),
            "omega_target": float(omega_target_cmd),
            "track_reference": track_reference,
            "turn_primitive": str(pivot_ref.get("turn_primitive") or ""),
            "pivot_primitive": {
                "enabled": bool(pivot_ref.get("enabled", False)),
                "source": "heading_decision",
                "reason": str(pivot_ref.get("reason", "")),
                "left_mps": pivot_ref.get("left_mps"),
                "right_mps": pivot_ref.get("right_mps"),
                "track_speed_mps": float(pivot_ref.get("track_speed_mps", 0.0)),
                "equivalent_omega_rad_s": float(omega_target_cmd),
            },
            "heading_error_deg": float(heading_error_deg),
            "heading_control_deg": float(control_heading_deg),
            "heading_control_source": str(self._last_control_heading_source),
            "ekf_heading_error_deg": float(
                _angle_error_deg(self.target_heading_deg, current_heading_deg)
            ),
            "elapsed_s": float(elapsed_s),
            "speed_level": int(self.active_speed_level),
            "omega_ref_rad_s": float(measured_omega_ref),
            "measured_omega_source": str(measured_omega_source),
            "translation_leakage": float(translation_leakage),
            "stall_elapsed_s": float(self._stall_elapsed_s),
            "side_imbalance": float(self._last_side_imbalance),  # diagnostics only
            "imu_only_active": bool(self._imu_only_active),
            "imu_only_elapsed_s": float(self._imu_only_elapsed_s),
            "imu_only_timeout_s": float(self.cfg.imu_only_timeout_s),
            "lidar_health_ok": bool(lidar_ok),
            "lidar_health_reason": str(lidar_reason),
            "heading_predictive_stop": {
                "active": bool(self._predictive_stop_hold),
                "horizon_s": float(prediction_horizon_s),
                "predicted_error_abs_deg": float(predicted_error_abs_deg),
                "moving_toward_target": bool(moving_toward_target),
            },
        }
        self._last_cmd = {"v_target": cmd["v_target"], "omega_target": cmd["omega_target"]}
        self._last_track_reference = dict(track_reference)
        return cmd

    @property
    def last_result(self) -> Dict[str, Any]:
        return dict(self._last_result or {})

    def status(self) -> Dict[str, Any]:
        available_levels = {
            str(int(level)): {
                "label": str(row.get("label") or f"level_{int(level)}"),
                "target_deg_s": round(float(_safe_float(row.get("target_deg_s"), 0.0)), 4),
                "range_deg_s": [
                    round(float(_safe_float((row.get("range_deg_s") or [0.0, 0.0])[0], 0.0)), 4),
                    round(float(_safe_float((row.get("range_deg_s") or [0.0, 0.0])[1], 0.0)), 4),
                ],
            }
            for level, row in sorted((self.speed_levels or {}).items(), key=lambda kv: int(kv[0]))
        }
        out = {
            "active": bool(self.active),
            "target_heading_deg": round(float(self.target_heading_deg), 3) if self.active else None,
            "source": self.source,
            "v_target_cmd": _safe_float(self._last_cmd.get("v_target", 0.0), 0.0),
            "omega_target_cmd": _safe_float(self._last_cmd.get("omega_target", 0.0), 0.0),
            "track_reference_cmd": dict(getattr(self, "_last_track_reference", {}) or {}),
            "pivot_primitive": {
                "enabled": bool(self.cfg.pivot_primitive_enabled),
                "turn_primitive": TURN_PRIMITIVE_ONE_TRACK_PIVOT,
                "track_min_mps": round(float(self.cfg.pivot_track_min_mps), 6),
                "track_max_mps": round(float(self.cfg.pivot_track_max_mps), 6),
            },
            "heading_predictive_stop": {
                "active": bool(self._predictive_stop_hold),
                "horizon_s": round(float(self.cfg.stop_prediction_horizon_s), 4),
                "predicted_error_abs_deg": round(float(self._last_predicted_heading_error_deg), 4),
            },
            "measured_omega_source": str(self._last_measured_omega_source or "none"),
            "heading_control_deg": round(float(self._last_control_heading_deg), 4),
            "heading_control_source": str(self._last_control_heading_source or "ekf_pose"),
            "imu_only_active": bool(self._imu_only_active),
            "imu_only_elapsed_s": round(float(self._imu_only_elapsed_s), 4),
            "imu_only_timeout_s": round(float(self.cfg.imu_only_timeout_s), 4),
            "imu_only_reason": str(self._imu_only_last_reason or ""),
            "speed_level": int(self.active_speed_level),
            "default_speed_level": int(self.default_speed_level),
            "speed_levels_available": available_levels,
            "runtime_speed_levels_path": str(self.runtime_speed_levels_path or ""),
            "last_result": dict(self._last_result or {}),
            "terminal_statuses": sorted(self.TERMINAL_STATUSES),
        }
        return out


class MotionSemanticsEngine:
    """
    Hardens behavioral meaning of motion states and applies lightweight
    heading-hold correction for low-speed forward quality.
    """

    def __init__(self, cfg: Optional[Dict[str, Any]] = None):
        cfg = dict(cfg or {})
        self.v_eps = _safe_float(cfg.get("v_eps"), 0.015)
        self.w_eps = _safe_float(cfg.get("w_eps"), 0.04)
        self.forward_heading_hold_kp = _safe_float(cfg.get("forward_heading_hold_kp"), 1.6)
        self.forward_heading_hold_max_w = _safe_float(cfg.get("forward_heading_hold_max_w"), 0.30)
        self.forward_heading_hold_enable = bool(cfg.get("forward_heading_hold_enable", True))
        self.forward_curvature_speed_scale_enable = bool(cfg.get("forward_curvature_speed_scale_enable", True))
        self.forward_heading_error_slowdown_deg = max(
            0.5, _safe_float(cfg.get("forward_heading_error_slowdown_deg"), 4.0)
        )
        self.forward_heading_error_full_slowdown_deg = max(
            self.forward_heading_error_slowdown_deg + 0.5,
            _safe_float(cfg.get("forward_heading_error_full_slowdown_deg"), 15.0),
        )
        self.forward_curvature_min_scale = _clamp(
            _safe_float(cfg.get("forward_curvature_min_scale"), 0.50),
            0.0,
            1.0,
        )
        self.forward_min_command_enable = bool(cfg.get("forward_min_command_enable", True))
        self.forward_min_command_mps = max(0.0, _safe_float(cfg.get("forward_min_command_mps"), 0.10))
        self.rotate_enforce_pure = bool(cfg.get("rotate_enforce_pure", True))
        self.idle_enforce_zero = bool(cfg.get("idle_enforce_zero", True))
        self._forward_ref_heading_deg: Optional[float] = None
        self._recovery_forward_ref_heading_deg: Optional[float] = None
        self._last_status: Dict[str, Any] = {}
        self._last_recovery_status: Dict[str, Any] = {}

    def enforce_recovery_heading_hold(
        self,
        ctrl,
        *,
        ekf_state: Optional[Dict[str, Any]] = None,
        now: Optional[float] = None,
    ) -> Dict[str, Any]:
        now = time.monotonic() if now is None else float(now)
        ekf_state = ekf_state or {}
        source = str(getattr(ctrl, "motion_command_source", "UNKNOWN") or "UNKNOWN")
        v_target = _safe_float(getattr(ctrl, "v_target", 0.0), 0.0)
        omega_target = _safe_float(getattr(ctrl, "omega_target", 0.0), 0.0)
        heading_error_deg = 0.0
        heading_hold_applied = False
        actions = []
        semantic_state = "IDLE"

        if abs(v_target) <= self.v_eps and abs(omega_target) <= self.w_eps:
            semantic_state = "IDLE"
        elif abs(v_target) <= self.v_eps and abs(omega_target) > self.w_eps:
            semantic_state = "ROTATE"
        elif abs(v_target) > self.v_eps and abs(omega_target) <= self.w_eps:
            semantic_state = "FORWARD"
        else:
            semantic_state = "CURVED"

        if (
            self.forward_heading_hold_enable
            and semantic_state == "FORWARD"
            and v_target > self.v_eps
            and source in ("GUI_JOYSTICK", "MANUAL", "STATE")
        ):
            heading_deg = _safe_float(ekf_state.get("theta_deg"), 0.0)
            if self._recovery_forward_ref_heading_deg is None:
                self._recovery_forward_ref_heading_deg = heading_deg
            heading_error_deg = _angle_error_deg(self._recovery_forward_ref_heading_deg, heading_deg)
            omega_corr = self.forward_heading_hold_kp * math.radians(heading_error_deg)
            omega_corr = _clamp(omega_corr, -self.forward_heading_hold_max_w, self.forward_heading_hold_max_w)
            if abs(omega_target) <= self.w_eps:
                ctrl.omega_target = float(omega_corr)
                omega_target = float(omega_corr)
                heading_hold_applied = True
                actions.append("RECOVERY_FORWARD_HEADING_HOLD")
        else:
            self._recovery_forward_ref_heading_deg = None

        command_type = str(getattr(ctrl, "active_motion_command_type", "idle") or "idle")
        command_layer = str(getattr(ctrl, "active_motion_command_layer", "IDLE") or "IDLE")
        execution_mode = normalize_execution_mode(
            getattr(ctrl, "motion_execution_mode", ""),
            fallback="IDLE_EXEC",
        )
        motion_public = dict(getattr(ctrl, "motion_public_status", {}) or {})
        requested_intent = dict(getattr(ctrl, "requested_motion_intent", {}) or {})
        limited_intent = {"v": float(v_target), "omega": float(omega_target)}
        arc_twist_hint = infer_follow_arc_twist_intent(
            command_type=command_type,
            execution_mode=execution_mode,
            behavior_status=dict(getattr(ctrl, "behavior_motion_status", {}) or {}),
        )
        if arc_twist_hint is not None:
            if _twist_needs_arc_semantic_anchor(requested_intent):
                requested_intent = dict(arc_twist_hint)
            if _twist_needs_arc_semantic_anchor(limited_intent):
                limited_intent = dict(arc_twist_hint)
        turn_semantics = classify_motion_layers(
            track_width_m=_track_width_m(ctrl),
            requested_motion_intent=requested_intent,
            limited_motion_intent=limited_intent,
            requested_track_reference=dict(getattr(ctrl, "requested_track_reference", {}) or {}),
            executed_track_reference={
                "left_mps": getattr(ctrl, "track_target_left_mps", None),
                "right_mps": getattr(ctrl, "track_target_right_mps", None),
            },
            actual_linear_mps=motion_public.get("actual_linear_mps"),
            actual_angular_dps=motion_public.get("actual_angular_dps"),
            execution_mode=execution_mode,
        )
        status = {
            "ts": round(now, 6),
            "mode": "RECOVERY_FORWARD_HOLD",
            "motion_source": source,
            "semantic_state": semantic_state,
            "v_target": round(float(v_target), 6),
            "omega_target": round(float(omega_target), 6),
            "heading_error_deg": round(float(heading_error_deg), 4),
            "heading_hold_applied": bool(heading_hold_applied),
            "actions": actions,
            "motion_schema_version": MOTION_SCHEMA_VERSION,
            "execution_mode": execution_mode,
            "turn_semantics": turn_semantics,
            "turn_primitive_requested": str((((turn_semantics.get("requested") or {}).get("turn_primitive")) or "UNKNOWN")),
            "turn_primitive_limited": str((((turn_semantics.get("limited") or {}).get("turn_primitive")) or "UNKNOWN")),
            "turn_primitive_executed": str((((turn_semantics.get("executed") or {}).get("turn_primitive")) or "UNKNOWN")),
            "turn_primitive_actual": str((((turn_semantics.get("actual") or {}).get("turn_primitive")) or "UNKNOWN")),
        }
        self._last_recovery_status = status
        return status

    def enforce(self, ctrl, ekf_state: Optional[Dict[str, Any]] = None, now: Optional[float] = None) -> Dict[str, Any]:
        now = time.monotonic() if now is None else float(now)
        ekf_state = ekf_state or {}
        state_name = "NONE"
        try:
            state_name = str(ctrl.sm.get_current_state_name())
        except Exception:
            state_name = str(getattr(getattr(ctrl, "sm", None), "current_enum", "NONE"))

        source = str(getattr(ctrl, "motion_command_source", "UNKNOWN") or "UNKNOWN")
        v_target = _safe_float(getattr(ctrl, "v_target", 0.0), 0.0)
        omega_target = _safe_float(getattr(ctrl, "omega_target", 0.0), 0.0)
        active_command_type = str(getattr(ctrl, "active_motion_command_type", "") or "").strip().lower()
        active_command_layer = str(getattr(ctrl, "active_motion_command_layer", "") or "").strip().upper()
        resolution_status = dict(getattr(ctrl, "motion_resolution_status", {}) or {})
        resolved_motion = dict(resolution_status.get("resolved") or {})
        resolved_command_type = str(resolved_motion.get("command_type", "") or "").strip().lower()
        resolved_layer = str(resolved_motion.get("layer", "") or "").strip().upper()
        local_planner_motion = bool(
            resolved_command_type == "local_planner_segment"
            or active_command_type == "local_planner_segment"
            or resolved_layer in {"LOCAL_PLANNER", "LOCAL_NAVIGATION"}
            or active_command_layer in {"LOCAL_PLANNER", "LOCAL_NAVIGATION"}
        )

        actions = []
        violations = []
        curvature_scale = 1.0
        command_floor_applied = False

        def _semantic_from_targets(v: float, w: float) -> str:
            if abs(v) <= self.v_eps and abs(w) <= self.w_eps:
                return "IDLE"
            if abs(v) <= self.v_eps and abs(w) > self.w_eps:
                return "ROTATE"
            if abs(v) > self.v_eps and abs(w) <= self.w_eps:
                return "FORWARD"
            return "CURVED"

        if self.idle_enforce_zero and state_name in ("IDLE", "FAILSAFE", "CALIBRATING") and not local_planner_motion:
            if abs(v_target) > self.v_eps or abs(omega_target) > self.w_eps:
                ctrl.v_target = 0.0
                ctrl.omega_target = 0.0
                v_target = 0.0
                omega_target = 0.0
                actions.append("IDLE_ZERO_ENFORCED")

        if self.rotate_enforce_pure and state_name == "ROTATE" and not local_planner_motion:
            if abs(v_target) > self.v_eps:
                ctrl.v_target = 0.0
                v_target = 0.0
                actions.append("ROTATE_PURE_ENFORCED")
                violations.append("ROTATE_TRANSLATION_REQUEST")
        elif self.rotate_enforce_pure and state_name == "ROTATE" and local_planner_motion:
            if abs(v_target) > self.v_eps:
                actions.append("ROTATE_STATE_LOCAL_PLANNER_ARC_ALLOWED")

        semantic_initial = _semantic_from_targets(v_target, omega_target)
        semantic = semantic_initial

        # Clearance has exactly one soft-planning owner (local/global policy)
        # and one authoritative hard-output owner (SafetyGate).  This semantic
        # layer must not independently rescale one axis of an already resolved
        # twist: doing so can turn an executable KIT0085 ARC into a twist below
        # the calibrated wheel-speed range.

        heading_error_deg = 0.0
        heading_hold_applied = False
        heading_hold_mode = "DISABLED"
        execution_mode_now = normalize_execution_mode(
            getattr(ctrl, "motion_execution_mode", ""),
            fallback="TWIST_EXEC",
        )
        deterministic_straight_gate_active = bool(
            getattr(ctrl, "deterministic_straight_gate_active", False)
        )
        requested_intent = dict(getattr(ctrl, "requested_motion_intent", {}) or {})
        requested_v = _safe_float(requested_intent.get("v"), v_target)
        requested_omega = _safe_float(requested_intent.get("omega"), omega_target)
        deterministic_straight_twist = bool(
            deterministic_straight_gate_active
            and execution_mode_now == "TWIST_EXEC"
            and (resolved_command_type == "set_twist" or active_command_type == "set_twist")
            and requested_v > self.v_eps
            and abs(requested_omega) <= self.w_eps
        )
        if deterministic_straight_twist and abs(float(omega_target)) > 1e-9:
            # Keep the physical command deterministic before the L8 boundary.
            ctrl.omega_target = 0.0
            omega_target = 0.0
            semantic = _semantic_from_targets(v_target, omega_target)
            actions.append("STRAIGHT_GATE_OMEGA_ZEROED")
        if (
            self.forward_heading_hold_enable
            and semantic == "FORWARD"
            and abs(v_target) > self.v_eps
            and source in ("GUI_JOYSTICK", "MANUAL", "STATE")
            and not local_planner_motion
        ):
            heading_deg = _safe_float(ekf_state.get("theta_deg"), 0.0)
            if self._forward_ref_heading_deg is None:
                self._forward_ref_heading_deg = heading_deg
            heading_error_deg = _angle_error_deg(self._forward_ref_heading_deg, heading_deg)
            omega_corr = self.forward_heading_hold_kp * math.radians(heading_error_deg)
            omega_corr = _clamp(omega_corr, -self.forward_heading_hold_max_w, self.forward_heading_hold_max_w)
            if abs(omega_target) <= self.w_eps:
                ctrl.omega_target = float(omega_corr)
                omega_target = float(omega_corr)
                heading_hold_applied = True
                if v_target < -self.v_eps:
                    heading_hold_mode = "GUIDANCE_APPLIED_REVERSE"
                    actions.append("REVERSE_HEADING_HOLD_GUIDANCE")
                else:
                    heading_hold_mode = "GUIDANCE_APPLIED_FORWARD"
                    actions.append("FORWARD_HEADING_HOLD_GUIDANCE")
            else:
                heading_hold_mode = "GUIDANCE_BYPASSED_EXPLICIT_TURN"
        else:
            self._forward_ref_heading_deg = None
            if local_planner_motion and semantic == "FORWARD":
                heading_hold_mode = "LOCAL_PLANNER_OWNS_HEADING"
                actions.append("LOCAL_PLANNER_HEADING_HOLD_BYPASS")
            else:
                heading_hold_mode = "DISABLED_OR_NOT_FORWARD"

        if (
            self.forward_curvature_speed_scale_enable
            and semantic == "FORWARD"
            and v_target > self.v_eps
        ):
            abs_heading_error = abs(float(heading_error_deg))
            if abs_heading_error > self.forward_heading_error_slowdown_deg:
                if abs_heading_error >= self.forward_heading_error_full_slowdown_deg:
                    curvature_scale = float(self.forward_curvature_min_scale)
                else:
                    denom = max(
                        1e-6,
                        self.forward_heading_error_full_slowdown_deg - self.forward_heading_error_slowdown_deg,
                    )
                    ratio = _clamp(
                        (abs_heading_error - self.forward_heading_error_slowdown_deg) / denom,
                        0.0,
                        1.0,
                    )
                    curvature_scale = float(
                        1.0 - ratio * (1.0 - self.forward_curvature_min_scale)
                    )
                scaled_v = float(v_target) * float(curvature_scale)
                if abs(scaled_v - v_target) > 1e-9:
                    ctrl.v_target = float(scaled_v)
                    v_target = float(scaled_v)
                    actions.append("FORWARD_CURVATURE_SCALED")

        correction_active = bool(
            heading_hold_applied
            or curvature_scale < (1.0 - 1e-6)
        )
        active_command_type = str(getattr(ctrl, "active_motion_command_type", "") or "").strip().lower()
        active_command_layer = str(getattr(ctrl, "active_motion_command_layer", "") or "").strip().upper()
        explicit_v2_twist = bool(
            active_command_layer == "MOTION_TARGET"
            and active_command_type in {"set_twist", "set_motion_target"}
        )
        if (
            self.forward_min_command_enable
            and semantic in ("FORWARD", "CURVED")
            and source in ("GUI_JOYSTICK", "MANUAL", "STATE", "ADAPTIVE")
            and not local_planner_motion
            and v_target > self.v_eps
            and abs(v_target) < self.forward_min_command_mps
            and not correction_active
            and not explicit_v2_twist
        ):
            v_target = math.copysign(self.forward_min_command_mps, v_target)
            ctrl.v_target = float(v_target)
            command_floor_applied = True
            actions.append("FORWARD_MIN_SPEED_ENFORCED")

        semantic = _semantic_from_targets(v_target, omega_target)
        if heading_hold_applied and semantic == "CURVED" and abs(v_target) > self.v_eps:
            semantic = "FORWARD"

        if semantic == "FORWARD" and v_target > self.v_eps and abs(omega_target) > max(self.w_eps, 0.12):
            violations.append("FORWARD_CURVATURE_VISIBLE")

        command_type = str(getattr(ctrl, "active_motion_command_type", "idle") or "idle")
        command_layer = str(getattr(ctrl, "active_motion_command_layer", "IDLE") or "IDLE")
        execution_mode = normalize_execution_mode(
            getattr(ctrl, "motion_execution_mode", ""),
            fallback="IDLE_EXEC",
        )
        motion_public = dict(getattr(ctrl, "motion_public_status", {}) or {})
        requested_intent = dict(getattr(ctrl, "requested_motion_intent", {}) or {})
        limited_intent = {"v": float(v_target), "omega": float(omega_target)}
        arc_twist_hint = infer_follow_arc_twist_intent(
            command_type=command_type,
            execution_mode=execution_mode,
            behavior_status=dict(getattr(ctrl, "behavior_motion_status", {}) or {}),
        )
        if arc_twist_hint is not None:
            if _twist_needs_arc_semantic_anchor(requested_intent):
                requested_intent = dict(arc_twist_hint)
            if _twist_needs_arc_semantic_anchor(limited_intent):
                limited_intent = dict(arc_twist_hint)
        turn_semantics = classify_motion_layers(
            track_width_m=_track_width_m(ctrl),
            requested_motion_intent=requested_intent,
            limited_motion_intent=limited_intent,
            requested_track_reference=dict(getattr(ctrl, "requested_track_reference", {}) or {}),
            executed_track_reference={
                "left_mps": getattr(ctrl, "track_target_left_mps", None),
                "right_mps": getattr(ctrl, "track_target_right_mps", None),
            },
            actual_linear_mps=motion_public.get("actual_linear_mps"),
            actual_angular_dps=motion_public.get("actual_angular_dps"),
            execution_mode=execution_mode,
        )
        status = {
            "ts": round(now, 6),
            "state_name": state_name,
            "motion_source": source,
            "semantic_state_initial": semantic_initial,
            "semantic_state": semantic,
            "v_target": round(float(v_target), 6),
            "omega_target": round(float(omega_target), 6),
            "heading_error_deg": round(float(heading_error_deg), 4),
            "heading_hold_applied": bool(heading_hold_applied),
            "heading_hold_mode": str(heading_hold_mode),
            "heading_hold_owner": "MOTION_GUIDANCE_L7A",
            "deterministic_straight_gate_active": bool(deterministic_straight_gate_active),
            "curvature_scale": round(float(curvature_scale), 4),
            "command_floor_applied": bool(command_floor_applied),
            "actions": actions,
            "violations": violations,
            "motion_schema_version": MOTION_SCHEMA_VERSION,
            "execution_mode": execution_mode,
            "turn_semantics": turn_semantics,
            "turn_primitive_requested": str((((turn_semantics.get("requested") or {}).get("turn_primitive")) or "UNKNOWN")),
            "turn_primitive_limited": str((((turn_semantics.get("limited") or {}).get("turn_primitive")) or "UNKNOWN")),
            "turn_primitive_executed": str((((turn_semantics.get("executed") or {}).get("turn_primitive")) or "UNKNOWN")),
            "turn_primitive_actual": str((((turn_semantics.get("actual") or {}).get("turn_primitive")) or "UNKNOWN")),
        }
        self._last_status = status
        return status

    @property
    def last_status(self) -> Dict[str, Any]:
        return dict(self._last_status or {})

