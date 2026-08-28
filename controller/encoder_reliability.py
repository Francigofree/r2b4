#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""V2.1 L2A encoder measurement trust and lineage assessment."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Any, Dict, Optional


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _normalize_control_mode(mode: Any) -> str:
    return str(mode or "").strip().upper()


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
        if now_mono is None:
            raise ValueError("encoder_reliability_now_mono_required")
        now_mono = float(now_mono)
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
