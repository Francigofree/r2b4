#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Encoder calibration observability guard.

Purpose:
- prevent self-confirming calibration loops
- allow calibration ingestion only from physically observable windows
- provide runtime diagnostics for why calibration is gated
"""

from __future__ import annotations

import math
import time
from collections import Counter, deque
from typing import Any, Deque, Dict, List, Optional, Tuple


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


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


def _sign(value: float, eps: float = 1e-9) -> int:
    if value > eps:
        return 1
    if value < -eps:
        return -1
    return 0


def _extract_lidar_confidence(lidar_summary: Dict[str, Any]) -> float:
    s = dict(lidar_summary or {})
    for key in (
        "lidar_pose_confidence",
        "final_confidence_emitted",
        "final_confidence",
        "matcher_confidence",
        "confidence",
    ):
        val = s.get(key)
        if _is_finite(val):
            return _clamp(_safe_float(val, 0.0), 0.0, 1.0)
    return 0.0


def _mean(values: List[float]) -> float:
    vals = [float(v) for v in values if _is_finite(v)]
    if not vals:
        return 0.0
    return float(sum(vals) / max(1, len(vals)))


def _variance(values: List[float]) -> float:
    vals = [float(v) for v in values if _is_finite(v)]
    n = len(vals)
    if n <= 1:
        return 0.0
    mu = _mean(vals)
    return float(sum((v - mu) ** 2 for v in vals) / float(n))


def _std(values: List[float]) -> float:
    return float(math.sqrt(max(0.0, _variance(values))))


def _entropy_normalized(tokens: List[str]) -> float:
    if not tokens:
        return 0.0
    c = Counter(tokens)
    total = float(sum(c.values()))
    if total <= 0.0:
        return 0.0
    probs = [float(v) / total for v in c.values() if v > 0]
    if len(probs) <= 1:
        return 0.0
    h = -sum(p * math.log2(p) for p in probs)
    h_max = math.log2(float(len(probs)))
    if h_max <= 1e-12:
        return 0.0
    return float(_clamp(h / h_max, 0.0, 1.0))


def _linear_slope(values: List[float]) -> float:
    vals = [float(v) for v in values if _is_finite(v)]
    n = len(vals)
    if n <= 1:
        return 0.0
    x_mean = float(n - 1) * 0.5
    y_mean = _mean(vals)
    num = 0.0
    den = 0.0
    for i, y in enumerate(vals):
        dx = float(i) - x_mean
        num += dx * (y - y_mean)
        den += dx * dx
    if den <= 1e-12:
        return 0.0
    return float(num / den)


class EncoderObservabilityGate:
    """
    Rolling-window observability guard for encoder calibration ingestion.

    The gate is intentionally lightweight and runtime-safe:
    - O(1) append + O(window) evaluation per tick (small bounded window)
    - no blocking IO
    """

    def __init__(self, cfg: Optional[Dict[str, Any]] = None):
        cfg = dict(cfg or {})

        self.window_size = max(30, _safe_int(cfg.get("window_size"), 240))
        self.min_window_samples = max(
            20,
            min(self.window_size, _safe_int(cfg.get("min_window_samples"), 60)),
        )

        # Motion observability / segment classification.
        self.motion_v_cmd_min = max(0.0, _safe_float(cfg.get("motion_v_cmd_min"), 0.08))
        self.motion_omega_cmd_min = max(0.01, _safe_float(cfg.get("motion_omega_cmd_min"), 0.45))
        self.straight_pwm_eps = max(0.001, _safe_float(cfg.get("straight_pwm_eps"), 0.08))
        self.straight_omega_cmd_max = max(0.001, _safe_float(cfg.get("straight_omega_cmd_max"), 0.18))
        self.straight_v_cmd_min = max(0.0, _safe_float(cfg.get("straight_v_cmd_min"), 0.08))
        self.rotate_pwm_eps = max(0.001, _safe_float(cfg.get("rotate_pwm_eps"), 0.12))
        self.rotate_pwm_min_abs = max(0.0, _safe_float(cfg.get("rotate_pwm_min_abs"), 0.20))
        self.rotate_omega_cmd_min = max(0.01, _safe_float(cfg.get("rotate_omega_cmd_min"), 0.45))
        self.min_straight_samples = max(3, _safe_int(cfg.get("min_straight_samples"), 12))
        self.min_rotate_samples = max(3, _safe_int(cfg.get("min_rotate_samples"), 12))
        self.max_other_ratio = _clamp(_safe_float(cfg.get("max_other_ratio"), 0.55), 0.05, 0.95)

        # Sensor independence.
        self.lidar_confidence_min = _clamp(_safe_float(cfg.get("lidar_confidence_min"), 0.2), 0.0, 1.0)
        self.lidar_conf_mean_min = _clamp(_safe_float(cfg.get("lidar_conf_mean_min"), 0.28), 0.0, 1.0)
        self.lidar_conf_std_max = max(0.001, _safe_float(cfg.get("lidar_conf_std_max"), 0.12))
        self.lidar_ok_ratio_min = _clamp(_safe_float(cfg.get("lidar_ok_ratio_min"), 0.85), 0.0, 1.0)
        self.max_relocalization_ratio = _clamp(
            _safe_float(cfg.get("max_relocalization_ratio"), 0.0),
            0.0,
            0.5,
        )
        self.max_jump_reject_ratio = _clamp(
            _safe_float(cfg.get("max_jump_reject_ratio"), 0.0),
            0.0,
            0.5,
        )

        # EKF independence.
        self.reject_ratio_max = _clamp(_safe_float(cfg.get("reject_ratio_max"), 0.35), 0.01, 0.95)
        self.innovation_min_samples = max(4, _safe_int(cfg.get("innovation_min_samples"), 16))
        self.innovation_variance_min = max(0.0, _safe_float(cfg.get("innovation_variance_min"), 3e-5))
        self.innovation_moving_ratio_min = _clamp(
            _safe_float(cfg.get("innovation_moving_ratio_min"), 0.35),
            0.05,
            1.0,
        )
        self.innovation_missing_ratio_max = _clamp(
            _safe_float(cfg.get("innovation_missing_ratio_max"), 0.0),
            0.0,
            1.0,
        )
        self.innovation_invalid_ratio_max = _clamp(
            _safe_float(cfg.get("innovation_invalid_ratio_max"), 0.0),
            0.0,
            1.0,
        )
        self.innovation_valid_ratio_min = _clamp(
            _safe_float(cfg.get("innovation_valid_ratio_min"), 1.0),
            0.0,
            1.0,
        )
        self.innovation_temporal_variance_min = max(
            0.0,
            _safe_float(cfg.get("innovation_temporal_variance_min"), 1e-6),
        )
        self.innovation_entropy_min = _clamp(
            _safe_float(cfg.get("innovation_entropy_min"), 0.08),
            0.0,
            1.0,
        )
        self.innovation_bin_size = max(1e-6, _safe_float(cfg.get("innovation_bin_size"), 0.01))
        self.innovation_zero_eps = max(0.0, _safe_float(cfg.get("innovation_zero_eps"), 1e-5))
        self.innovation_zero_ratio_max = _clamp(
            _safe_float(cfg.get("innovation_zero_ratio_max"), 0.98),
            0.0,
            1.0,
        )
        self.innovation_frozen_range_max = max(
            0.0,
            _safe_float(cfg.get("innovation_frozen_range_max"), 1e-4),
        )

        # IMU observability extension.
        self.imu_noise_threshold = max(1e-4, _safe_float(cfg.get("imu_noise_threshold"), 0.22))
        self.imu_drift_threshold = max(1e-4, _safe_float(cfg.get("imu_drift_threshold"), 0.10))
        self.imu_ekf_mismatch_threshold = max(
            1e-4,
            _safe_float(cfg.get("imu_ekf_mismatch_threshold"), 0.18),
        )
        self.imu_ekf_mismatch_ratio_max = _clamp(
            _safe_float(cfg.get("imu_ekf_mismatch_ratio_max"), 0.15),
            0.0,
            1.0,
        )
        self.imu_min_samples = max(4, _safe_int(cfg.get("imu_min_samples"), 16))
        self.imu_zero_v_eps = max(0.0, _safe_float(cfg.get("imu_zero_v_eps"), 0.02))
        self.imu_zero_pwm_eps = max(0.0, _safe_float(cfg.get("imu_zero_pwm_eps"), 0.05))
        self.imu_compare_min_abs_omega = max(
            0.0,
            _safe_float(cfg.get("imu_compare_min_abs_omega"), 0.05),
        )
        self.imu_proxy_ekf_omega_std_max = max(
            1e-4,
            _safe_float(cfg.get("imu_proxy_ekf_omega_std_max"), 0.25),
        )
        self.imu_drift_warning_ratio = _clamp(
            _safe_float(cfg.get("imu_drift_warning_ratio"), 0.02),
            0.0,
            1.0,
        )
        self.imu_drift_block_ratio = _clamp(
            _safe_float(cfg.get("imu_drift_block_ratio"), 0.10),
            0.0,
            1.0,
        )
        self.imu_required = bool(cfg.get("imu_required", False))
        self.imu_missing_innovation_valid_ratio_min = _clamp(
            _safe_float(cfg.get("imu_missing_innovation_valid_ratio_min"), 0.90),
            0.0,
            1.0,
        )
        self.imu_mean_shift_threshold = max(
            1e-5,
            _safe_float(cfg.get("imu_mean_shift_threshold"), 0.06),
        )
        self.imu_bias_trend_slope_threshold = max(
            1e-6,
            _safe_float(cfg.get("imu_bias_trend_slope_threshold"), 0.004),
        )
        self.imu_ekf_direction_mismatch_ratio_max = _clamp(
            _safe_float(cfg.get("imu_ekf_direction_mismatch_ratio_max"), 0.25),
            0.0,
            1.0,
        )
        self.imu_encoder_direction_mismatch_ratio_max = _clamp(
            _safe_float(cfg.get("imu_encoder_direction_mismatch_ratio_max"), 0.35),
            0.0,
            1.0,
        )
        self.imu_confidence_min = _clamp(
            _safe_float(cfg.get("imu_confidence_min"), 0.45),
            0.0,
            1.0,
        )

        # Information richness.
        self.motion_entropy_min = _clamp(_safe_float(cfg.get("motion_entropy_min"), 0.30), 0.0, 1.0)
        self.min_pwm_direction_switches = max(0, _safe_int(cfg.get("min_pwm_direction_switches"), 1))
        self.min_joint_entropy = _clamp(_safe_float(cfg.get("min_joint_entropy"), 0.18), 0.0, 1.0)
        self.min_excitation_diversity = _clamp(
            _safe_float(cfg.get("min_excitation_diversity"), 0.10),
            0.0,
            1.0,
        )
        self.motion_excitation_ratio_min = _clamp(
            _safe_float(cfg.get("motion_excitation_ratio_min"), 0.30),
            0.0,
            1.0,
        )

        # Aggregate score threshold.
        self.allow_score_min = _clamp(_safe_float(cfg.get("allow_score_min"), 0.70), 0.0, 1.0)

        self._window: Deque[Dict[str, Any]] = deque(maxlen=self.window_size)
        self._last_result: Dict[str, Any] = {
            "calibration_allowed": False,
            "reason": "INSUFFICIENT_WINDOW",
            "risk_flags": ["INSUFFICIENT_WINDOW"],
            "observability_score": 0.0,
            "window_stats": {"window_count": 0},
        }

    def _quantize_pwm(self, value: float) -> int:
        v = _safe_float(value, 0.0)
        if v >= 0.66:
            return 2
        if v >= 0.20:
            return 1
        if v <= -0.66:
            return -2
        if v <= -0.20:
            return -1
        return 0

    def _classify_motion(self, *, pwm_l: float, pwm_r: float, v_cmd: float, omega_cmd: float) -> str:
        if (
            abs(pwm_l - pwm_r) <= self.straight_pwm_eps
            and abs(omega_cmd) <= self.straight_omega_cmd_max
            and abs(v_cmd) >= self.straight_v_cmd_min
        ):
            return "STRAIGHT"
        if (
            abs(pwm_l + pwm_r) <= self.rotate_pwm_eps
            and abs(omega_cmd) >= self.rotate_omega_cmd_min
            and min(abs(pwm_l), abs(pwm_r)) >= self.rotate_pwm_min_abs
        ):
            return "ROTATE"
        return "OTHER"

    @staticmethod
    def _extract_innovation_proxy_verbose(ekf_state: Dict[str, Any]) -> Tuple[Optional[float], str]:
        ekf = dict(ekf_state or {})

        any_present = False
        any_invalid = False

        inno_v = ekf.get("innovation_v")
        inno_th = ekf.get("innovation_theta")
        if "innovation_v" in ekf or "innovation_theta" in ekf:
            any_present = True
        if _is_finite(inno_v) and _is_finite(inno_th):
            return float(math.hypot(float(inno_v), float(inno_th))), "valid"
        if _is_finite(inno_v):
            return float(abs(float(inno_v))), "valid"
        if _is_finite(inno_th):
            return float(abs(float(inno_th))), "valid"
        if inno_v is not None and not _is_finite(inno_v):
            any_invalid = True
        if inno_th is not None and not _is_finite(inno_th):
            any_invalid = True

        inno = ekf.get("innovation")
        if "innovation" in ekf:
            any_present = True
        if isinstance(inno, (list, tuple)) and len(inno) >= 2 and _is_finite(inno[0]) and _is_finite(inno[1]):
            return float(math.hypot(float(inno[0]), float(inno[1]))), "valid"
        if isinstance(inno, dict):
            iv = inno.get("v")
            it = inno.get("theta")
            if _is_finite(iv) and _is_finite(it):
                return float(math.hypot(float(iv), float(it))), "valid"
            if iv is not None and not _is_finite(iv):
                any_invalid = True
            if it is not None and not _is_finite(it):
                any_invalid = True
        if _is_finite(inno):
            return float(abs(float(inno))), "valid"
        if inno is not None and not _is_finite(inno) and not isinstance(inno, (list, tuple, dict)):
            any_invalid = True
        if isinstance(inno, (list, tuple)) and len(inno) >= 1:
            # Present but malformed/non-finite list entry.
            any_invalid = True
        if isinstance(inno, dict) and ("v" not in inno and "theta" not in inno):
            any_invalid = True

        if any_invalid:
            return None, "invalid"
        if any_present:
            return None, "missing"
        return None, "missing"

    @staticmethod
    def _extract_imu_omega(
        ekf_state: Dict[str, Any],
        imu_omega_rad_s: Optional[float],
    ) -> Tuple[Optional[float], bool]:
        if imu_omega_rad_s is not None and _is_finite(imu_omega_rad_s):
            return float(imu_omega_rad_s), True
        ekf = dict(ekf_state or {})
        if _is_finite(ekf.get("imu_omega_rad_s")):
            return float(ekf.get("imu_omega_rad_s")), True
        if _is_finite(ekf.get("gyro_z")):
            return float(ekf.get("gyro_z")), True
        return None, False

    @staticmethod
    def _count_pair_switches(pairs: List[Tuple[int, int]]) -> int:
        last: Optional[Tuple[int, int]] = None
        switches = 0
        for pair in pairs:
            if pair == (0, 0):
                continue
            if last is None:
                last = pair
                continue
            if pair != last:
                switches += 1
            last = pair
        return int(switches)

    @staticmethod
    def _append_flag(flags: List[str], flag: str) -> None:
        name = str(flag or "").strip().upper()
        if not name:
            return
        if name not in flags:
            flags.append(name)

    def evaluate(
        self,
        *,
        encoder_reliability: Dict[str, Any],
        ekf_state: Dict[str, Any],
        lidar_summary: Dict[str, Any],
        lidar_health: str,
        v_cmd_mps: float,
        omega_cmd_rad_s: float,
        pwm_l: float,
        pwm_r: float,
        pulse_delta_l: int,
        pulse_delta_r: int,
        imu_omega_rad_s: Optional[float] = None,
        collector_state: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        rel = dict(encoder_reliability or {})
        ekf = dict(ekf_state or {})
        lidar = dict(lidar_summary or {})
        flags = list(rel.get("flags", []) or [])

        usage_mode = str(rel.get("ekf_usage_mode", "REJECT") or "REJECT").upper()
        combined_trust = _clamp(_safe_float(rel.get("combined_trust"), 0.0), 0.0, 1.0)
        symmetry_fault = bool(
            rel.get("symmetry_fault_active", False)
            or ("PWM_ENCODER_SYMMETRY_FAULT" in flags)
        )
        encoder_dropout = bool(
            rel.get("encoder_dropout", False)
            or ("LEFT_ENCODER_DROPOUT_SUSPECT" in flags)
            or ("RIGHT_ENCODER_DROPOUT_SUSPECT" in flags)
            or ("PWM_ENCODER_SYMMETRY_FAULT" in flags)
        )

        v_cmd = _safe_float(v_cmd_mps, 0.0)
        omega_cmd = _safe_float(omega_cmd_rad_s, 0.0)
        pwm_left = _safe_float(pwm_l, 0.0)
        pwm_right = _safe_float(pwm_r, 0.0)
        pulse_left = _safe_int(pulse_delta_l, 0)
        pulse_right = _safe_int(pulse_delta_r, 0)
        kind = self._classify_motion(
            pwm_l=pwm_left,
            pwm_r=pwm_right,
            v_cmd=v_cmd,
            omega_cmd=omega_cmd,
        )

        lidar_conf = _extract_lidar_confidence(lidar)
        lidar_health_ok = str(lidar_health or "").upper() == "OK"
        lidar_ok = bool(lidar_health_ok and lidar_conf >= self.lidar_confidence_min)

        status_hint = str(
            lidar.get("status")
            or lidar.get("odometry_status")
            or lidar.get("odom_status")
            or ""
        ).strip().lower()
        relocalized = bool(
            lidar.get("relocalized", False)
            or lidar.get("relocalization_attempted", False)
            or ("relocal" in status_hint)
        )
        jump_reject = bool(
            str(lidar.get("odom_status", "")).strip().upper() in ("JUMP_REJECT", "BOOTSTRAP_JUMP_REJECT")
            or status_hint in ("rejected_jump", "rejected_bootstrap_jump")
            or ("jump" in status_hint and "reject" in status_hint)
        )

        innovation, innovation_state = self._extract_innovation_proxy_verbose(ekf)
        innovation_available = innovation is not None and innovation_state == "valid"
        imu_omega, imu_available = self._extract_imu_omega(
            ekf_state=ekf,
            imu_omega_rad_s=imu_omega_rad_s,
        )
        ekf_omega = _safe_float(ekf.get("omega_rad_s"), 0.0)

        entry = {
            "ts_s": _safe_float(ekf.get("ts"), time.monotonic()),
            "usage_mode": usage_mode,
            "combined_trust": combined_trust,
            "symmetry_fault": bool(symmetry_fault),
            "encoder_dropout": bool(encoder_dropout),
            "lidar_conf": float(lidar_conf),
            "lidar_ok": bool(lidar_ok),
            "relocalized": bool(relocalized),
            "jump_reject": bool(jump_reject),
            "v_cmd_abs": float(abs(v_cmd)),
            "omega_cmd_abs": float(abs(omega_cmd)),
            "moving_cmd": bool(
                abs(v_cmd) >= self.motion_v_cmd_min
                or abs(omega_cmd) >= self.motion_omega_cmd_min
            ),
            "kind": str(kind),
            "innovation_available": bool(innovation_available),
            "innovation": (None if innovation is None else float(innovation)),
            "innovation_state": str(innovation_state),
            "imu_available": bool(imu_available),
            "imu_omega": (None if imu_omega is None else float(imu_omega)),
            "ekf_omega": float(ekf_omega),
            "imu_ekf_abs_diff": (
                None if (imu_omega is None or not _is_finite(ekf_omega)) else float(abs(float(imu_omega) - float(ekf_omega)))
            ),
            "zero_motion_cmd": bool(abs(v_cmd) < self.imu_zero_v_eps and abs(pwm_left) < self.imu_zero_pwm_eps and abs(pwm_right) < self.imu_zero_pwm_eps),
            "pwm_pair": (_sign(pwm_left), _sign(pwm_right)),
            "pwm_token": (
                f"{self._quantize_pwm(pwm_left)}:{self._quantize_pwm(pwm_right)}:"
                f"{_sign(v_cmd)}:{_sign(omega_cmd)}"
            ),
            "pulse_delta_l": int(pulse_left),
            "pulse_delta_r": int(pulse_right),
            "encoder_omega_proxy_sign": int(_sign(float(pulse_right - pulse_left))),
            "pulse_active": bool(abs(pulse_left) > 0 or abs(pulse_right) > 0),
        }
        self._window.append(entry)

        w = list(self._window)
        n = len(w)
        kind_counts = Counter(str(e.get("kind", "OTHER")) for e in w)
        straight_count = int(kind_counts.get("STRAIGHT", 0))
        rotate_count = int(kind_counts.get("ROTATE", 0))
        other_count = int(kind_counts.get("OTHER", 0))
        other_ratio = float(other_count / max(1, n))

        v_cmd_observed = bool(any(float(e.get("v_cmd_abs", 0.0)) >= self.motion_v_cmd_min for e in w))
        omega_cmd_observed = bool(any(float(e.get("omega_cmd_abs", 0.0)) >= self.motion_omega_cmd_min for e in w))
        moving_ratio = float(sum(1 for e in w if bool(e.get("moving_cmd", False))) / max(1, n))

        reject_count = int(sum(1 for e in w if str(e.get("usage_mode", "")).upper() == "REJECT"))
        degraded_count = int(sum(1 for e in w if str(e.get("usage_mode", "")).upper() == "DEGRADED"))
        reject_ratio = float(reject_count / max(1, n))
        degraded_ratio = float(degraded_count / max(1, n))

        dropout_count = int(sum(1 for e in w if bool(e.get("encoder_dropout", False))))
        symmetry_count = int(sum(1 for e in w if bool(e.get("symmetry_fault", False))))
        dropout_ratio = float(dropout_count / max(1, n))
        symmetry_ratio = float(symmetry_count / max(1, n))

        lidar_confs = [float(e.get("lidar_conf", 0.0)) for e in w]
        lidar_conf_mean = _mean(lidar_confs)
        lidar_conf_std = _std(lidar_confs)
        lidar_ok_ratio = float(sum(1 for e in w if bool(e.get("lidar_ok", False))) / max(1, n))

        relocal_count = int(sum(1 for e in w if bool(e.get("relocalized", False))))
        jump_reject_count = int(sum(1 for e in w if bool(e.get("jump_reject", False))))
        relocal_ratio = float(relocal_count / max(1, n))
        jump_reject_ratio = float(jump_reject_count / max(1, n))

        innovation_values = [float(e.get("innovation", 0.0)) for e in w if bool(e.get("innovation_available", False))]
        innovation_count = int(len(innovation_values))
        innovation_variance = _variance(innovation_values)
        innovation_diffs = [
            float(innovation_values[i] - innovation_values[i - 1])
            for i in range(1, innovation_count)
        ]
        innovation_temporal_variance = _variance(innovation_diffs)
        innovation_missing_count = int(sum(1 for e in w if str(e.get("innovation_state", "")).lower() == "missing"))
        innovation_invalid_count = int(sum(1 for e in w if str(e.get("innovation_state", "")).lower() == "invalid"))
        innovation_valid_ratio = float(innovation_count / max(1, n))
        innovation_missing_ratio = float(innovation_missing_count / max(1, n))
        innovation_invalid_ratio = float(innovation_invalid_count / max(1, n))
        innovation_zero_count = int(sum(1 for v in innovation_values if abs(float(v)) <= self.innovation_zero_eps))
        innovation_zero_ratio = float(innovation_zero_count / max(1, innovation_count))
        innovation_range = (
            float(max(innovation_values) - min(innovation_values))
            if innovation_count > 0
            else 0.0
        )
        innovation_tokens = [
            str(int(round(float(v) / max(1e-9, self.innovation_bin_size))))
            for v in innovation_values
        ]
        innovation_entropy = _entropy_normalized(innovation_tokens)
        innovation_unique_bins = int(len(set(innovation_tokens)))
        innovation_low_variance = bool(
            innovation_count >= self.innovation_min_samples
            and moving_ratio >= self.innovation_moving_ratio_min
            and innovation_variance < self.innovation_variance_min
        )
        innovation_temporal_low_variance = bool(
            innovation_count >= self.innovation_min_samples
            and innovation_temporal_variance < self.innovation_temporal_variance_min
        )
        innovation_entropy_low = bool(
            innovation_count >= self.innovation_min_samples
            and innovation_entropy < self.innovation_entropy_min
        )
        innovation_constant_zero = bool(
            innovation_count >= self.innovation_min_samples
            and innovation_zero_ratio >= self.innovation_zero_ratio_max
        )
        innovation_frozen = bool(
            innovation_count >= self.innovation_min_samples
            and (
                innovation_range <= self.innovation_frozen_range_max
                or innovation_unique_bins <= 1
            )
        )

        motion_observability_class = "FULLY_OBSERVABLE"
        if (
            moving_ratio < self.motion_excitation_ratio_min
            or (straight_count <= 0 and rotate_count <= 0)
        ):
            motion_observability_class = "DEGENERATE"
        elif (
            straight_count < self.min_straight_samples
            or rotate_count < self.min_rotate_samples
            or not v_cmd_observed
            or not omega_cmd_observed
        ):
            motion_observability_class = "PARTIALLY_OBSERVABLE"
        degeneracy_detected = bool(motion_observability_class == "DEGENERATE")

        imu_values = [float(e.get("imu_omega", 0.0)) for e in w if bool(e.get("imu_available", False))]
        imu_count = int(len(imu_values))
        imu_available_ratio = float(imu_count / max(1, n))
        imu_std = _std(imu_values)
        imu_ekf_diff_values = [
            float(e.get("imu_ekf_abs_diff", 0.0))
            for e in w
            if bool(e.get("imu_available", False)) and _is_finite(e.get("imu_ekf_abs_diff"))
        ]
        imu_ekf_diff_std = _std(imu_ekf_diff_values)

        imu_drift_count = int(
            sum(
                1
                for e in w
                if bool(e.get("imu_available", False))
                and bool(e.get("zero_motion_cmd", False))
                and _is_finite(e.get("imu_omega"))
                and abs(float(e.get("imu_omega", 0.0))) > self.imu_drift_threshold
            )
        )
        imu_drift_ratio = float(imu_drift_count / max(1, imu_count))
        imu_drift_warning = bool(imu_drift_ratio > self.imu_drift_warning_ratio)
        imu_drift_detected = bool(imu_drift_ratio > self.imu_drift_block_ratio)

        imu_ekf_mismatch_count = int(
            sum(
                1
                for e in w
                if bool(e.get("imu_available", False))
                and _is_finite(e.get("imu_ekf_abs_diff"))
                and float(e.get("imu_ekf_abs_diff", 0.0)) > self.imu_ekf_mismatch_threshold
            )
        )
        imu_ekf_mismatch_ratio = float(imu_ekf_mismatch_count / max(1, imu_count))
        imu_ekf_mismatch = bool(
            imu_count >= self.imu_min_samples
            and imu_ekf_mismatch_ratio > self.imu_ekf_mismatch_ratio_max
        )

        imu_zero_motion_values = [
            float(e.get("imu_omega", 0.0))
            for e in w
            if bool(e.get("imu_available", False))
            and bool(e.get("zero_motion_cmd", False))
            and _is_finite(e.get("imu_omega"))
        ]
        imu_zero_motion_count = int(len(imu_zero_motion_values))
        imu_zero_motion_slope = _linear_slope(imu_zero_motion_values)
        imu_zero_motion_mean_shift = 0.0
        if imu_zero_motion_count >= 4:
            split = imu_zero_motion_count // 2
            imu_zero_motion_mean_shift = abs(
                _mean(imu_zero_motion_values[:split]) - _mean(imu_zero_motion_values[split:])
            )
        imu_bias_trend = bool(
            imu_zero_motion_count >= self.imu_min_samples
            and abs(imu_zero_motion_slope) > self.imu_bias_trend_slope_threshold
        )
        imu_mean_shift = bool(
            imu_zero_motion_count >= self.imu_min_samples
            and imu_zero_motion_mean_shift > self.imu_mean_shift_threshold
        )

        imu_ekf_dir_total = 0
        imu_ekf_dir_disagree_count = 0
        imu_encoder_dir_total = 0
        imu_encoder_dir_disagree_count = 0
        for e in w:
            if not bool(e.get("imu_available", False)) or not _is_finite(e.get("imu_omega")):
                continue
            imu_s = _sign(float(e.get("imu_omega", 0.0)), eps=self.imu_compare_min_abs_omega)
            if imu_s == 0:
                continue

            ekf_s = _sign(float(e.get("ekf_omega", 0.0)), eps=self.imu_compare_min_abs_omega)
            if ekf_s != 0:
                imu_ekf_dir_total += 1
                if imu_s != ekf_s:
                    imu_ekf_dir_disagree_count += 1

            if bool(e.get("encoder_dropout", False)) or bool(e.get("symmetry_fault", False)):
                continue
            enc_s = _safe_int(e.get("encoder_omega_proxy_sign"), 0)
            if enc_s != 0:
                imu_encoder_dir_total += 1
                if imu_s != enc_s:
                    imu_encoder_dir_disagree_count += 1

        imu_ekf_dir_disagree_ratio = float(imu_ekf_dir_disagree_count / max(1, imu_ekf_dir_total))
        imu_encoder_dir_disagree_ratio = float(
            imu_encoder_dir_disagree_count / max(1, imu_encoder_dir_total)
        )
        imu_ekf_direction_mismatch = bool(
            imu_ekf_dir_total >= self.imu_min_samples
            and imu_ekf_dir_disagree_ratio > self.imu_ekf_direction_mismatch_ratio_max
        )
        imu_encoder_direction_mismatch = bool(
            imu_encoder_dir_total >= max(4, self.imu_min_samples // 2)
            and imu_encoder_dir_disagree_ratio > self.imu_encoder_direction_mismatch_ratio_max
        )

        ekf_omega_values = [float(e.get("ekf_omega", 0.0)) for e in w if _is_finite(e.get("ekf_omega"))]
        ekf_omega_std = _std(ekf_omega_values)
        imu_noise_high = bool(
            imu_count >= self.imu_min_samples
            and (
                (
                    imu_std > self.imu_noise_threshold
                    and ekf_omega_std <= (self.imu_noise_threshold * 0.75)
                )
                or (imu_ekf_diff_std > self.imu_noise_threshold)
            )
        )

        pwm_tokens = [str(e.get("pwm_token", "")) for e in w]
        motion_tokens = [f"{str(e.get('kind', 'OTHER'))}:{str(e.get('pwm_token', ''))}" for e in w]
        joint_motion_innovation_tokens: List[str] = []
        for e in w:
            motion_token = f"{str(e.get('kind', 'OTHER'))}:{str(e.get('pwm_token', ''))}"
            if bool(e.get("innovation_available", False)) and _is_finite(e.get("innovation")):
                ib = int(round(float(e.get("innovation", 0.0)) / max(1e-9, self.innovation_bin_size)))
                innovation_token = str(ib)
            else:
                innovation_token = "NA"
            joint_motion_innovation_tokens.append(f"{motion_token}|{innovation_token}")
        motion_entropy = _entropy_normalized(pwm_tokens)
        joint_motion_innovation_entropy = _entropy_normalized(joint_motion_innovation_tokens)
        excitation_diversity = _clamp(float(len(set(motion_tokens))) / 8.0, 0.0, 1.0)
        pwm_switches = self._count_pair_switches([tuple(e.get("pwm_pair", (0, 0))) for e in w])

        if motion_observability_class == "FULLY_OBSERVABLE":
            motion_excitation_quality = 1.0
        elif motion_observability_class == "PARTIALLY_OBSERVABLE":
            motion_excitation_quality = 0.6
        else:
            motion_excitation_quality = 0.25

        if imu_count > 0:
            imu_confidence = _clamp(
                0.20 * _clamp(imu_available_ratio, 0.0, 1.0)
                + 0.20 * (1.0 - _clamp(float(imu_std) / max(1e-6, float(self.imu_noise_threshold)), 0.0, 1.0))
                + 0.15 * (1.0 - _clamp(float(imu_drift_ratio) / max(1e-6, float(self.imu_drift_block_ratio)), 0.0, 1.0))
                + 0.15 * (1.0 - _clamp(float(imu_ekf_mismatch_ratio) / max(1e-6, float(self.imu_ekf_mismatch_ratio_max)), 0.0, 1.0))
                + 0.15 * (
                    1.0
                    - _clamp(
                        max(
                            float(imu_ekf_dir_disagree_ratio) / max(1e-6, float(self.imu_ekf_direction_mismatch_ratio_max)),
                            float(imu_encoder_dir_disagree_ratio)
                            / max(1e-6, float(self.imu_encoder_direction_mismatch_ratio_max)),
                        ),
                        0.0,
                        1.0,
                    )
                )
                + 0.10
                * (
                    1.0
                    - _clamp(
                        max(
                            abs(float(imu_zero_motion_slope)) / max(1e-6, float(self.imu_bias_trend_slope_threshold)),
                            float(imu_zero_motion_mean_shift) / max(1e-6, float(self.imu_mean_shift_threshold)),
                        ),
                        0.0,
                        1.0,
                    )
                )
                + 0.05 * motion_excitation_quality,
                0.0,
                1.0,
            )
        else:
            proxy = 1.0 - _clamp(
                float(ekf_omega_std) / max(1e-6, float(self.imu_proxy_ekf_omega_std_max)),
                0.0,
                1.0,
            )
            imu_confidence = _clamp(0.35 * proxy + 0.15 * motion_excitation_quality, 0.0, 1.0)

        risk_flags: List[str] = []
        if n < self.min_window_samples:
            self._append_flag(risk_flags, "INSUFFICIENT_WINDOW")
        if not v_cmd_observed:
            self._append_flag(risk_flags, "CMD_LINEAR_UNOBSERVED")
        if not omega_cmd_observed:
            self._append_flag(risk_flags, "CMD_ANGULAR_UNOBSERVED")
        if motion_observability_class == "PARTIALLY_OBSERVABLE":
            self._append_flag(risk_flags, "MOTION_PARTIALLY_OBSERVABLE")
        if degeneracy_detected:
            self._append_flag(risk_flags, "MOTION_DEGENERATE")
        if motion_observability_class != "FULLY_OBSERVABLE":
            self._append_flag(risk_flags, "MOTION_OBSERVABILITY_INSUFFICIENT")
        if straight_count < self.min_straight_samples:
            self._append_flag(risk_flags, "STRAIGHT_SEGMENTS_MISSING")
        if rotate_count < self.min_rotate_samples:
            self._append_flag(risk_flags, "ROTATE_SEGMENTS_MISSING")
        if dropout_count > 0:
            self._append_flag(risk_flags, "ENCODER_DROPOUT_IN_WINDOW")
        if symmetry_count > 0:
            self._append_flag(risk_flags, "SYMMETRY_FAULT_IN_WINDOW")
        if lidar_ok_ratio < self.lidar_ok_ratio_min:
            self._append_flag(risk_flags, "LIDAR_OK_RATIO_LOW")
        if lidar_conf_mean < self.lidar_conf_mean_min:
            self._append_flag(risk_flags, "LIDAR_CONFIDENCE_LOW")
        if lidar_conf_std > self.lidar_conf_std_max:
            self._append_flag(risk_flags, "LIDAR_CONFIDENCE_UNSTABLE")
        if relocal_ratio > self.max_relocalization_ratio:
            self._append_flag(risk_flags, "LIDAR_RELOCALIZATION_ACTIVE")
        if jump_reject_ratio > self.max_jump_reject_ratio:
            self._append_flag(risk_flags, "LIDAR_JUMP_REJECT_ACTIVE")
        if reject_ratio > self.reject_ratio_max:
            self._append_flag(risk_flags, "EKF_REJECT_RATIO_HIGH")
        if innovation_missing_count > 0:
            self._append_flag(risk_flags, "INNOVATION_DATA_MISSING")
        if innovation_invalid_count > 0:
            self._append_flag(risk_flags, "INNOVATION_DATA_INVALID")
        if innovation_low_variance:
            self._append_flag(risk_flags, "EKF_INNOVATION_VARIANCE_LOW")
        if innovation_temporal_low_variance:
            self._append_flag(risk_flags, "INNOVATION_TEMPORAL_VARIANCE_LOW")
        if innovation_entropy_low:
            self._append_flag(risk_flags, "INNOVATION_ENTROPY_LOW")
        if innovation_frozen:
            self._append_flag(risk_flags, "INNOVATION_FROZEN")
        if innovation_constant_zero:
            self._append_flag(risk_flags, "INNOVATION_CONSTANT_ZERO")
        if other_ratio > self.max_other_ratio:
            self._append_flag(risk_flags, "OTHER_BUCKET_TOO_HIGH")
        if motion_entropy < self.motion_entropy_min:
            self._append_flag(risk_flags, "MOTION_ENTROPY_LOW")
        if joint_motion_innovation_entropy < self.min_joint_entropy:
            self._append_flag(risk_flags, "JOINT_INFORMATION_LOW")
        if excitation_diversity < self.min_excitation_diversity:
            self._append_flag(risk_flags, "EXCITATION_DIVERSITY_LOW")
        if pwm_switches < self.min_pwm_direction_switches:
            self._append_flag(risk_flags, "MOTION_DIRECTION_SWITCH_LOW")
        if imu_noise_high:
            self._append_flag(risk_flags, "IMU_NOISE_HIGH")
        if imu_drift_warning:
            self._append_flag(risk_flags, "IMU_DRIFT_WARNING")
        if imu_drift_detected:
            self._append_flag(risk_flags, "IMU_DRIFT_DETECTED")
        if imu_ekf_mismatch:
            self._append_flag(risk_flags, "IMU_EKF_MISMATCH")
        if imu_ekf_direction_mismatch:
            self._append_flag(risk_flags, "IMU_EKF_DIRECTION_MISMATCH")
        if imu_encoder_direction_mismatch:
            self._append_flag(risk_flags, "IMU_ENCODER_DIRECTION_MISMATCH")
        if imu_bias_trend:
            self._append_flag(risk_flags, "IMU_BIAS_TREND")
        if imu_mean_shift:
            self._append_flag(risk_flags, "IMU_MEAN_SHIFT")
        if imu_confidence < self.imu_confidence_min:
            self._append_flag(risk_flags, "IMU_CONFIDENCE_LOW")
        if imu_count <= 0:
            self._append_flag(risk_flags, "IMU_NOT_AVAILABLE")
            if self.imu_required or innovation_valid_ratio < self.imu_missing_innovation_valid_ratio_min:
                self._append_flag(risk_flags, "IMU_REQUIRED_MISSING")

        motion_class_component = 1.0
        if motion_observability_class == "PARTIALLY_OBSERVABLE":
            motion_class_component = 0.55
        elif motion_observability_class == "DEGENERATE":
            motion_class_component = 0.10
        motion_score = (
            0.22 * float(v_cmd_observed)
            + 0.22 * float(omega_cmd_observed)
            + 0.23 * _clamp(float(straight_count) / max(1.0, float(self.min_straight_samples)), 0.0, 1.0)
            + 0.23 * _clamp(float(rotate_count) / max(1.0, float(self.min_rotate_samples)), 0.0, 1.0)
            + 0.10 * motion_class_component
        )
        sensor_score = (
            0.35 * _clamp(float(lidar_ok_ratio) / max(1e-6, float(self.lidar_ok_ratio_min)), 0.0, 1.0)
            + 0.25 * _clamp(float(lidar_conf_mean) / max(1e-6, float(self.lidar_conf_mean_min)), 0.0, 1.0)
            + 0.20 * (1.0 - _clamp(float(lidar_conf_std) / max(1e-6, float(self.lidar_conf_std_max)), 0.0, 1.0))
            + 0.10 * (1.0 - _clamp(dropout_ratio, 0.0, 1.0))
            + 0.10 * (1.0 - _clamp(symmetry_ratio, 0.0, 1.0))
        )
        innovation_quality_shape = _clamp(
            1.0
            - 0.25 * float(innovation_low_variance)
            - 0.20 * float(innovation_temporal_low_variance)
            - 0.15 * float(innovation_entropy_low)
            - 0.20 * float(innovation_frozen)
            - 0.20 * float(innovation_constant_zero),
            0.0,
            1.0,
        )
        innovation_availability = _clamp(float(innovation_valid_ratio), 0.0, 1.0)
        innovation_quality = _clamp(innovation_quality_shape * innovation_availability, 0.0, 1.0)
        independence_score = (
            0.50 * (1.0 - _clamp(float(reject_ratio) / max(1e-6, float(self.reject_ratio_max)), 0.0, 1.0))
            + 0.15 * (1.0 - _clamp(degraded_ratio, 0.0, 1.0))
            + 0.35 * innovation_quality
        )
        other_headroom = 1.0 - _clamp(float(other_ratio) / max(1e-6, float(self.max_other_ratio)), 0.0, 1.0)
        motion_entropy_component = _clamp(float(motion_entropy) / max(1e-6, float(self.motion_entropy_min)), 0.0, 1.0)
        joint_entropy_component = _clamp(
            float(joint_motion_innovation_entropy) / max(1e-6, float(self.min_joint_entropy)),
            0.0,
            1.0,
        )
        excitation_diversity_component = _clamp(
            float(excitation_diversity) / max(1e-6, float(self.min_excitation_diversity)),
            0.0,
            1.0,
        )
        switch_component = _clamp(float(pwm_switches) / max(1.0, float(self.min_pwm_direction_switches)), 0.0, 1.0)
        richness_score = (
            0.30 * motion_entropy_component
            + 0.20 * joint_entropy_component
            + 0.20 * excitation_diversity_component
            + 0.15 * other_headroom
            + 0.15 * switch_component
        )
        imu_score = _clamp(float(imu_confidence), 0.0, 1.0)
        score = _clamp(
            0.24 * motion_score
            + 0.24 * sensor_score
            + 0.22 * independence_score
            + 0.10 * richness_score
            + 0.20 * imu_score,
            0.0,
            1.0,
        )
        if score < self.allow_score_min:
            self._append_flag(risk_flags, "OBSERVABILITY_SCORE_LOW")

        hard_block_flags = {
            "INSUFFICIENT_WINDOW",
            "CMD_LINEAR_UNOBSERVED",
            "CMD_ANGULAR_UNOBSERVED",
            "MOTION_OBSERVABILITY_INSUFFICIENT",
            "STRAIGHT_SEGMENTS_MISSING",
            "ROTATE_SEGMENTS_MISSING",
            "ENCODER_DROPOUT_IN_WINDOW",
            "SYMMETRY_FAULT_IN_WINDOW",
            "LIDAR_OK_RATIO_LOW",
            "LIDAR_CONFIDENCE_LOW",
            "LIDAR_CONFIDENCE_UNSTABLE",
            "LIDAR_RELOCALIZATION_ACTIVE",
            "LIDAR_JUMP_REJECT_ACTIVE",
            "EKF_REJECT_RATIO_HIGH",
            "INNOVATION_DATA_MISSING",
            "INNOVATION_DATA_INVALID",
            "EKF_INNOVATION_VARIANCE_LOW",
            "INNOVATION_TEMPORAL_VARIANCE_LOW",
            "INNOVATION_ENTROPY_LOW",
            "INNOVATION_FROZEN",
            "INNOVATION_CONSTANT_ZERO",
            "OTHER_BUCKET_TOO_HIGH",
            "MOTION_ENTROPY_LOW",
            "JOINT_INFORMATION_LOW",
            "EXCITATION_DIVERSITY_LOW",
            "MOTION_DIRECTION_SWITCH_LOW",
            "OBSERVABILITY_SCORE_LOW",
            "IMU_REQUIRED_MISSING",
        }

        innovation_unavailable = bool(
            innovation_missing_count > 0
            or innovation_invalid_count > 0
            or innovation_valid_ratio < self.innovation_valid_ratio_min
        )
        imu_unstable = bool(
            "IMU_NOISE_HIGH" in risk_flags
            or "IMU_DRIFT_DETECTED" in risk_flags
            or "IMU_EKF_MISMATCH" in risk_flags
            or "IMU_EKF_DIRECTION_MISMATCH" in risk_flags
            or "IMU_ENCODER_DIRECTION_MISMATCH" in risk_flags
            or "IMU_BIAS_TREND" in risk_flags
            or "IMU_MEAN_SHIFT" in risk_flags
        )
        imu_required_missing = bool("IMU_REQUIRED_MISSING" in risk_flags)
        motion_unobservable = bool(motion_observability_class != "FULLY_OBSERVABLE")
        hard_block_active = bool(any(flag in hard_block_flags for flag in risk_flags))

        if innovation_unavailable:
            calibration_allowed = False
            reason = "INNOVATION_UNAVAILABLE"
        elif imu_unstable:
            calibration_allowed = False
            reason = "IMU_UNSTABLE"
        elif imu_required_missing:
            calibration_allowed = False
            reason = "IMU_REQUIRED_MISSING"
        elif motion_unobservable:
            calibration_allowed = False
            reason = "MOTION_OBSERVABILITY_INSUFFICIENT"
        elif hard_block_active or score < self.allow_score_min:
            calibration_allowed = False
            reason = str(
                next((f for f in risk_flags if f in hard_block_flags), "OBSERVABILITY_BLOCKED")
            )
        else:
            calibration_allowed = True
            reason = "ALLOW_CALIBRATION"

        window_stats = {
            "window_count": int(n),
            "min_window_samples": int(self.min_window_samples),
            "motion_observability_class": str(motion_observability_class),
            "degeneracy_detected": bool(degeneracy_detected),
            "innovation_valid_ratio": float(round(innovation_valid_ratio, 6)),
            "innovation_missing_count": int(innovation_missing_count),
            "innovation_missing_ratio": float(round(innovation_missing_ratio, 6)),
            "innovation_invalid_count": int(innovation_invalid_count),
            "innovation_invalid_ratio": float(round(innovation_invalid_ratio, 6)),
            "innovation_entropy": float(round(innovation_entropy, 6)),
            "imu_confidence": float(round(imu_confidence, 6)),
            "kind_counts": {
                "STRAIGHT": int(straight_count),
                "ROTATE": int(rotate_count),
                "OTHER": int(other_count),
            },
            "other_ratio": float(round(other_ratio, 6)),
            "command_observability": {
                "v_cmd_observed": bool(v_cmd_observed),
                "omega_cmd_observed": bool(omega_cmd_observed),
                "moving_ratio": float(round(moving_ratio, 6)),
            },
            "sensor_independence": {
                "lidar_ok_ratio": float(round(lidar_ok_ratio, 6)),
                "lidar_conf_mean": float(round(lidar_conf_mean, 6)),
                "lidar_conf_std": float(round(lidar_conf_std, 6)),
                "dropout_ratio": float(round(dropout_ratio, 6)),
                "symmetry_fault_ratio": float(round(symmetry_ratio, 6)),
                "relocalization_ratio": float(round(relocal_ratio, 6)),
                "jump_reject_ratio": float(round(jump_reject_ratio, 6)),
            },
            "ekf_independence": {
                "reject_ratio": float(round(reject_ratio, 6)),
                "degraded_ratio": float(round(degraded_ratio, 6)),
                "innovation_count": int(innovation_count),
                "innovation_valid_ratio": float(round(innovation_valid_ratio, 6)),
                "innovation_missing_count": int(innovation_missing_count),
                "innovation_missing_ratio": float(round(innovation_missing_ratio, 6)),
                "innovation_invalid_count": int(innovation_invalid_count),
                "innovation_invalid_ratio": float(round(innovation_invalid_ratio, 6)),
                "innovation_variance": float(round(innovation_variance, 8)),
                "innovation_temporal_variance": float(round(innovation_temporal_variance, 8)),
                "innovation_entropy": float(round(innovation_entropy, 6)),
                "innovation_zero_ratio": float(round(innovation_zero_ratio, 6)),
                "innovation_range": float(round(innovation_range, 8)),
                "innovation_unique_bins": int(innovation_unique_bins),
                "innovation_low_variance": bool(innovation_low_variance),
                "innovation_temporal_low_variance": bool(innovation_temporal_low_variance),
                "innovation_entropy_low": bool(innovation_entropy_low),
                "innovation_frozen": bool(innovation_frozen),
                "innovation_constant_zero": bool(innovation_constant_zero),
            },
            "information_richness": {
                "motion_entropy": float(round(motion_entropy, 6)),
                "joint_motion_innovation_entropy": float(round(joint_motion_innovation_entropy, 6)),
                "excitation_diversity": float(round(excitation_diversity, 6)),
                "pwm_direction_switches": int(pwm_switches),
            },
            "imu_observability": {
                "imu_available_ratio": float(round(imu_available_ratio, 6)),
                "imu_sample_count": int(imu_count),
                "imu_confidence": float(round(imu_confidence, 6)),
                "imu_omega_std": float(round(imu_std, 8)),
                "imu_ekf_diff_std": float(round(imu_ekf_diff_std, 8)),
                "imu_noise_high": bool(imu_noise_high),
                "imu_drift_warning": bool(imu_drift_warning),
                "imu_drift_detected": bool(imu_drift_detected),
                "imu_drift_count": int(imu_drift_count),
                "imu_drift_ratio": float(round(imu_drift_ratio, 6)),
                "imu_ekf_mismatch": bool(imu_ekf_mismatch),
                "imu_ekf_mismatch_count": int(imu_ekf_mismatch_count),
                "imu_ekf_mismatch_ratio": float(round(imu_ekf_mismatch_ratio, 6)),
                "imu_ekf_direction_mismatch": bool(imu_ekf_direction_mismatch),
                "imu_ekf_direction_disagree_ratio": float(round(imu_ekf_dir_disagree_ratio, 6)),
                "imu_encoder_direction_mismatch": bool(imu_encoder_direction_mismatch),
                "imu_encoder_direction_disagree_ratio": float(round(imu_encoder_dir_disagree_ratio, 6)),
                "imu_bias_trend": bool(imu_bias_trend),
                "imu_zero_motion_count": int(imu_zero_motion_count),
                "imu_zero_motion_slope": float(round(imu_zero_motion_slope, 8)),
                "imu_mean_shift": bool(imu_mean_shift),
                "imu_zero_motion_mean_shift": float(round(imu_zero_motion_mean_shift, 8)),
                "ekf_omega_std_proxy": float(round(ekf_omega_std, 8)),
            },
            "collector_state": dict(collector_state or {}),
            "score_components": {
                "motion_score": float(round(motion_score, 6)),
                "sensor_score": float(round(sensor_score, 6)),
                "independence_score": float(round(independence_score, 6)),
                "richness_score": float(round(richness_score, 6)),
                "imu_score": float(round(imu_score, 6)),
            },
        }

        out = {
            "calibration_allowed": bool(calibration_allowed),
            "reason": str(reason),
            "risk_flags": list(risk_flags),
            "observability_score": float(round(score, 6)),
            "window_stats": window_stats,
        }
        self._last_result = dict(out)
        return out

    def get_summary(self) -> Dict[str, Any]:
        return dict(self._last_result or {})
