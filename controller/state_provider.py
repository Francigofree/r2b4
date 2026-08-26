#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
StateProvider: SSOT bridge between sensor snapshots and EKF inputs.

Responsibilities:
- timestamp-aligned sensor fusion inputs (imu + encoder)
- deterministic dt computation with fallback/clamps
- encoder usage blend (for runtime enable/disable without hard discontinuity)
- rolling dt/noise diagnostics for EKF health
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Any, Dict, Optional

import numpy as np

from middleware.robot_frame import POSE_FRAME_ID, POSE_FRAME_OWNER, POSE_FRAME_YAW

from middleware.peripheral_usage import get_cached_peripherals


def _sliding_variance(values: deque, window_size: int) -> Optional[float]:
    if len(values) < window_size:
        return None
    arr = np.array(list(values)[-window_size:], dtype=float)
    if not np.all(np.isfinite(arr)):
        return None
    return float(np.var(arr))


def _inputs_finite(gyro_z: float, accel_x: float, v_l: float, v_r: float, dt: float) -> bool:
    return bool(
        np.isfinite(gyro_z)
        and np.isfinite(accel_x)
        and np.isfinite(v_l)
        and np.isfinite(v_r)
        and np.isfinite(dt)
        and dt > 0.0
    )


def _to_us(timestamp_s: Optional[float]) -> Optional[int]:
    if timestamp_s is None:
        return None
    try:
        ts = float(timestamp_s)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(ts) or ts <= 0.0:
        return None
    return int(round(ts * 1_000_000.0))


class StateProvider:
    """
    Stateful sensor pre-processing for the control loop.
    """

    def __init__(
        self,
        *,
        loop_hz: float = 50.0,
        dt_window_size: int = 100,
        noise_window_size: int = 200,
        dt_std_warning_threshold: float = 0.002,
    ):
        self.loop_hz = max(1.0, float(loop_hz))
        self.dt_target = 1.0 / self.loop_hz
        self.dt_std_warning_threshold = max(0.0, float(dt_std_warning_threshold))

        self._last_imu_ts: Optional[float] = None
        self._last_enc_ts: Optional[float] = None

        self._dt_window: deque = deque(maxlen=max(4, int(dt_window_size)))
        self._gyro_window: deque = deque(maxlen=max(8, int(noise_window_size)))
        self._accel_window: deque = deque(maxlen=max(8, int(noise_window_size)))
        self._encoder_v_window: deque = deque(maxlen=max(8, int(noise_window_size)))

        self._encoder_usage_gain = 1.0
        self._encoder_enabled = True
        self._last_encoder_theta_raw: Optional[float] = None
        self._last_encoder_theta_pose_frame: Optional[float] = None
        self._encoder_theta_frame_offset: Optional[float] = None

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return float((float(angle) + math.pi) % (2.0 * math.pi) - math.pi)

    @staticmethod
    def _live_ekf_theta(ctrl) -> Optional[float]:
        candidates = [
            getattr(ctrl, "ekf", None),
            getattr(getattr(ctrl, "ekf_manager", None), "ekf_live", None),
        ]
        for ekf in candidates:
            if ekf is None or not hasattr(ekf, "get_state"):
                continue
            try:
                theta = float((ekf.get_state() or {}).get("theta"))
            except (AttributeError, TypeError, ValueError):
                continue
            if np.isfinite(theta):
                return StateProvider._wrap_angle(theta)
        return None

    def reset_encoder_yaw_alignment(self) -> None:
        """Forget the raw encoder accumulator anchor after an atomic pose reset."""
        self._last_encoder_theta_raw = None
        self._last_encoder_theta_pose_frame = None
        self._encoder_theta_frame_offset = None

    def _encoder_theta_in_pose_frame(self, ctrl, theta_raw) -> Optional[float]:
        """
        Express pulse-integrated encoder yaw in the current EKF pose frame.

        The encoder estimator owns a relative accumulator whose zero is only
        synchronized during reset.  It is not an independent global heading.
        Preserve its per-tick yaw delta, but anchor that delta to the fused pose
        before the current predict/update cycle.  This keeps accepted LIDAR pose
        corrections from being pulled back toward an obsolete encoder zero.
        """
        try:
            raw = float(theta_raw)
        except (TypeError, ValueError):
            return None
        if not np.isfinite(raw):
            return None

        pose_theta = self._live_ekf_theta(ctrl)
        previous_raw = self._last_encoder_theta_raw
        previous_pose_frame = self._last_encoder_theta_pose_frame

        if previous_raw is None:
            aligned = pose_theta if pose_theta is not None else self._wrap_angle(raw)
        else:
            encoder_delta = self._wrap_angle(raw - float(previous_raw))
            if pose_theta is not None:
                aligned = self._wrap_angle(pose_theta + encoder_delta)
            elif previous_pose_frame is not None:
                aligned = self._wrap_angle(float(previous_pose_frame) + encoder_delta)
            else:
                aligned = self._wrap_angle(raw)

        self._last_encoder_theta_raw = float(raw)
        self._last_encoder_theta_pose_frame = float(aligned)
        self._encoder_theta_frame_offset = self._wrap_angle(float(aligned) - float(raw))
        return float(aligned)

    def compute_dt_ekf(
        self,
        *,
        fallback_dt: float,
        imu_snapshot: Optional[object],
        enc_snapshot: Optional[object],
    ) -> float:
        dt_imu = None
        dt_enc = None

        if imu_snapshot is not None and getattr(imu_snapshot, "health", "") == "OK":
            ts = getattr(imu_snapshot, "timestamp", None)
            if ts is not None and self._last_imu_ts is not None:
                cand = float(ts) - float(self._last_imu_ts)
                if cand > 0.0:
                    dt_imu = cand
            if ts is not None:
                self._last_imu_ts = float(ts)

        if enc_snapshot is not None:
            ts = getattr(enc_snapshot, "timestamp", None)
            if ts is not None and self._last_enc_ts is not None:
                cand = float(ts) - float(self._last_enc_ts)
                if cand > 0.0:
                    dt_enc = cand
            if ts is not None:
                self._last_enc_ts = float(ts)

        dt_out = dt_imu if dt_imu is not None else (dt_enc if dt_enc is not None else fallback_dt)
        if dt_out is None or dt_out <= 0.0:
            dt_out = fallback_dt if fallback_dt and fallback_dt > 0.0 else self.dt_target
        if dt_out > 0.2:
            dt_out = self.dt_target
        if dt_out < 1e-3:
            dt_out = 1e-3
        return float(dt_out)

    def update_encoder_usage_gain(
        self,
        ctrl,
        *,
        dt_ekf: float,
        peripherals: Optional[Dict[str, bool]] = None,
    ) -> tuple[bool, float, float]:
        """
        Encoder usage blend [0..1] to avoid hard estimator/control discontinuity.
        """
        if peripherals is None:
            try:
                peripherals = get_cached_peripherals(status_path=getattr(ctrl, "status_path", None))
            except Exception:
                peripherals = {}
        encoder_enabled = bool((peripherals or {}).get("encoder", True))
        cfg = getattr(ctrl, "cfg", {}) or {}
        vezerles_cfg = cfg.get("vezerles", {}) if isinstance(cfg, dict) else {}
        blend_sec_cfg = float(vezerles_cfg.get("encoder_toggle_blend_sec", 0.55))
        blend_sec = min(2.0, max(0.1, blend_sec_cfg))

        target_gain = 1.0 if encoder_enabled else 0.0
        step = max(0.0, float(dt_ekf)) / max(1e-3, blend_sec)
        if target_gain > self._encoder_usage_gain:
            self._encoder_usage_gain = min(target_gain, self._encoder_usage_gain + step)
        elif target_gain < self._encoder_usage_gain:
            self._encoder_usage_gain = max(target_gain, self._encoder_usage_gain - step)
        if abs(self._encoder_usage_gain - target_gain) < 1e-3:
            self._encoder_usage_gain = target_gain

        self._encoder_enabled = bool(encoder_enabled)
        ctrl.encoder_enabled = bool(encoder_enabled)
        ctrl.encoder_usage_gain = float(self._encoder_usage_gain)
        ctrl.encoder_toggle_blend_sec = float(blend_sec)
        return self._encoder_enabled, float(self._encoder_usage_gain), float(blend_sec)

    def prepare_ekf_inputs(
        self,
        *,
        ctrl,
        dt_loop: float,
        imu_snapshot: Optional[object],
        enc_snapshot: Optional[object],
        v_l_raw: float,
        v_r_raw: float,
        v_cmd_for_ekf: float,
        v_target: float,
        v_l_canonical: Optional[float] = None,
        v_r_canonical: Optional[float] = None,
        encoder_reliability: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        dt_sensor = self.compute_dt_ekf(
            fallback_dt=float(dt_loop),
            imu_snapshot=imu_snapshot,
            enc_snapshot=enc_snapshot,
        )
        try:
            peripherals = get_cached_peripherals(
                status_path=getattr(ctrl, "status_path", None),
            )
        except Exception:
            peripherals = {}
        imu_enabled = bool(peripherals.get("imu", True))
        ctrl.imu_enabled = bool(imu_enabled)
        cfg = getattr(ctrl, "cfg", None) or {}
        use_loop_dt = bool(cfg.get("vezerles", {}).get("ekf_use_loop_dt", False)) if isinstance(cfg, dict) else False
        dt_ekf = float(dt_loop) if use_loop_dt else float(dt_sensor)

        if imu_enabled and imu_snapshot is not None and getattr(imu_snapshot, "health", "") == "OK":
            gx, gy, gz_dps = imu_snapshot.gyro
            gyro_z_rad = math.radians(float(gz_dps))
            ax_g = float(imu_snapshot.accel[0])
            accel_x_mps2 = ax_g * 9.81
            imu_source = str(getattr(imu_snapshot, "source", "bno055") or "bno055")
            imu_heading_deg = getattr(imu_snapshot, "mag", None)
        else:
            gyro_z_rad = 0.0
            accel_x_mps2 = 0.0
            gz_dps = 0.0
            ax_g = 0.0
            imu_source = "DISABLED" if not imu_enabled else "N/A"
            imu_heading_deg = None

        no_throttle_cmd = (
            abs(float(getattr(ctrl, "v_target", 0.0))) < 1e-3
            and abs(float(getattr(ctrl, "omega_target", 0.0))) < 1e-3
            and abs(float(getattr(ctrl, "v_cmd", 0.0))) < 1e-3
        )
        pwm_idle = abs(float(getattr(ctrl, "_prev_pwm_l", 0.0))) < 1e-3 and abs(float(getattr(ctrl, "_prev_pwm_r", 0.0))) < 1e-3
        forced_still = no_throttle_cmd and pwm_idle
        if forced_still:
            accel_x_mps2 = 0.0

        self._dt_window.append(dt_ekf)
        if len(self._dt_window) >= 2:
            arr = np.array(list(self._dt_window), dtype=float)
            mean_dt = float(np.mean(arr))
            std_dt = float(np.std(arr))
            max_dt = float(np.max(arr))
        else:
            mean_dt = dt_ekf
            std_dt = 0.0
            max_dt = dt_ekf
        dt_stats = {
            "mean_dt": mean_dt,
            "std_dt": std_dt,
            "max_dt": max_dt,
            "ekf_timing_warning": std_dt > self.dt_std_warning_threshold,
        }

        sensor_ok = _inputs_finite(gyro_z_rad, accel_x_mps2, float(v_l_raw), float(v_r_raw), dt_ekf)
        if sensor_ok:
            self._gyro_window.append(gyro_z_rad)
            self._accel_window.append(accel_x_mps2)
            self._encoder_v_window.append((float(v_l_raw) + float(v_r_raw)) * 0.5)
        noise_stats = {
            "gyro_var": _sliding_variance(self._gyro_window, self._gyro_window.maxlen),
            "accel_var": _sliding_variance(self._accel_window, self._accel_window.maxlen),
            "encoder_var": _sliding_variance(self._encoder_v_window, self._encoder_v_window.maxlen),
        }

        if getattr(ctrl, "logger", None) is not None and not sensor_ok:
            ctrl.logger.warn(
                "[EKF] input non-finite, skip update: "
                f"gyro_z={gyro_z_rad}, accel_x={accel_x_mps2}, v_l={v_l_raw}, v_r={v_r_raw}, dt={dt_ekf}"
            )

        v_l_ekf = float(v_l_raw)
        v_r_ekf = float(v_r_raw)
        v_l_canonical_raw = None
        v_r_canonical_raw = None
        if v_l_canonical is not None and np.isfinite(float(v_l_canonical)):
            v_l_ekf = float(v_l_canonical)
            v_l_canonical_raw = float(v_l_canonical)
        if v_r_canonical is not None and np.isfinite(float(v_r_canonical)):
            v_r_ekf = float(v_r_canonical)
            v_r_canonical_raw = float(v_r_canonical)

        trust = dict(encoder_reliability or {})
        usage_mode = str(trust.get("ekf_usage_mode", "NORMAL") or "NORMAL").upper()
        usage_reason = str(trust.get("ekf_usage_reason", "") or "")
        cov_hint = float(trust.get("ekf_covariance_scale_hint", 1.0) or 1.0)
        cov_hint = max(1.0, min(30.0, cov_hint))
        weight_hint = float(trust.get("ekf_weight_hint", 1.0) or 1.0)
        weight_hint = max(0.0, min(1.0, weight_hint))
        confidence_hint = float(trust.get("combined_trust", 1.0) or 1.0)
        confidence_hint = max(0.0, min(1.0, confidence_hint))

        # EKF-barát csatorna: THETA_ONLY/REJECT módban ne menjen be lineáris sebesség.
        if usage_mode in ("THETA_ONLY", "REJECT"):
            v_l_ekf = 0.0
            v_r_ekf = 0.0

        encoder_enabled, encoder_usage_gain, encoder_blend_sec = self.update_encoder_usage_gain(
            ctrl,
            dt_ekf=dt_ekf,
            peripherals=peripherals,
        )
        theta_enc_pose_frame = self._encoder_theta_in_pose_frame(
            ctrl,
            getattr(enc_snapshot, "theta_enc", None) if enc_snapshot is not None else None,
        )

        frame_ts_us = int(time.perf_counter_ns() // 1_000)
        timestamps_us = {
            "frame": frame_ts_us,
            "imu": _to_us(getattr(imu_snapshot, "timestamp", None)) if imu_snapshot is not None else None,
            "encoder": _to_us(getattr(enc_snapshot, "timestamp", None)) if enc_snapshot is not None else None,
        }

        ctrl.ekf_dt_stats = dict(dt_stats)
        ctrl.ekf_noise_stats = dict(noise_stats)
        ctrl.ekf_sensor_finite = bool(sensor_ok)
        ctrl.ekf_skip_reason = None if sensor_ok else "sensor_non_finite"
        ctrl.state_timestamps_us = dict(timestamps_us)

        return {
            "dt_ekf": float(dt_ekf),
            "dt_source": "loop" if use_loop_dt else "sensor",
            "imu_data": {
                "accel_x": float(accel_x_mps2),
                "gyro_z": float(gyro_z_rad),
            },
            "encoder_data": {
                "pose_frame_id": POSE_FRAME_ID,
                "pose_frame_owner": POSE_FRAME_OWNER,
                "yaw_convention": POSE_FRAME_YAW,
                "v_l": float(v_l_ekf),
                "v_r": float(v_r_ekf),
                "v_l_raw": float(v_l_raw),
                "v_r_raw": float(v_r_raw),
                "v_l_canonical": float(v_l_ekf),
                "v_r_canonical": float(v_r_ekf),
                "v_l_canonical_raw": (None if v_l_canonical_raw is None else float(v_l_canonical_raw)),
                "v_r_canonical_raw": (None if v_r_canonical_raw is None else float(v_r_canonical_raw)),
                "v_cmd": float(v_cmd_for_ekf),
                "v_target": float(v_target),
                "theta_enc": theta_enc_pose_frame,
                "trust": trust,
                "enabled": bool(encoder_enabled),
                "usage_gain": float(encoder_usage_gain),
                "blend_sec": float(encoder_blend_sec),
                "pipeline_model": str(
                    (encoder_reliability or {}).get("pipeline_model", "KIT0085_QUADRATURE")
                ),
                "source_truth": str((encoder_reliability or {}).get("source_truth", "RAW_SIGNED_PULSE_DELTA")),
                "quality": {
                    "usage_mode": usage_mode,
                    "usage_reason": usage_reason,
                    "confidence": float(confidence_hint),
                    "covariance_scale_hint": float(cov_hint),
                    "weight_hint": float(weight_hint),
                },
                "timestamps_us": {
                    "encoder": timestamps_us.get("encoder"),
                    "frame": frame_ts_us,
                },
            },
            "gyro_z_rad": float(gyro_z_rad),
            "gyro_z_dps": float(gz_dps),
            "accel_x_mps2": float(accel_x_mps2),
            "accel_x_g": float(ax_g),
            "imu_source": str(imu_source),
            "imu_heading_deg": (None if imu_heading_deg is None else float(imu_heading_deg)),
            "sensor_ok": bool(sensor_ok),
            "dt_stats": dict(dt_stats),
            "noise_stats": dict(noise_stats),
            "forced_still": bool(forced_still),
            "encoder_enabled": bool(encoder_enabled),
            "encoder_usage_gain": float(encoder_usage_gain),
            "encoder_blend_sec": float(encoder_blend_sec),
            "imu_enabled": bool(imu_enabled),
            "peripherals": dict(peripherals),
            "timestamps_us": dict(timestamps_us),
        }


def create_state_provider_from_config(vezerles_cfg: Optional[Dict[str, Any]], *, loop_hz: float) -> StateProvider:
    cfg = dict((vezerles_cfg or {}).get("state_provider") or {})
    return StateProvider(
        loop_hz=loop_hz,
        dt_window_size=int(cfg.get("dt_window_size", 100)),
        noise_window_size=int(cfg.get("noise_window_size", 200)),
        dt_std_warning_threshold=float(cfg.get("dt_std_warning_threshold", 0.002)),
    )
