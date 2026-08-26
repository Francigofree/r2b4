#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Automatic encoder calibration utilities.

This module is intentionally runtime-safe:
- O(1) ingest path per tick
- no blocking IO in collector methods
- robust statistics (median + IQR)
"""

from __future__ import annotations

import json
import math
import os
import shutil
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional


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


def _sign(value: float, eps: float = 1e-9) -> float:
    if value > eps:
        return 1.0
    if value < -eps:
        return -1.0
    return 0.0


def _wrap_angle_rad(angle: float) -> float:
    return float((float(angle) + math.pi) % (2.0 * math.pi) - math.pi)


def _median(values: List[float]) -> float:
    vals = sorted(float(v) for v in values if _is_finite(v))
    n = len(vals)
    if n == 0:
        return float("nan")
    mid = n // 2
    if n % 2 == 1:
        return float(vals[mid])
    return float(0.5 * (vals[mid - 1] + vals[mid]))


def _percentile(values: List[float], p: float) -> float:
    vals = sorted(float(v) for v in values if _is_finite(v))
    if not vals:
        return float("nan")
    pp = _clamp(p, 0.0, 1.0)
    idx = (len(vals) - 1) * pp
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return float(vals[lo])
    w = idx - lo
    return float((1.0 - w) * vals[lo] + w * vals[hi])


def _iqr_filter(values: List[float]) -> List[float]:
    vals = [float(v) for v in values if _is_finite(v)]
    if len(vals) < 4:
        return vals
    q1 = _percentile(vals, 0.25)
    q3 = _percentile(vals, 0.75)
    if not _is_finite(q1) or not _is_finite(q3):
        return vals
    iqr = float(q3 - q1)
    if iqr <= 1e-9:
        return vals
    lo = float(q1 - 1.5 * iqr)
    hi = float(q3 + 1.5 * iqr)
    out = [v for v in vals if lo <= v <= hi]
    return out if out else vals


def _stats(values: List[float], *, preview_limit: int = 16) -> Dict[str, Any]:
    vals = [float(v) for v in values if _is_finite(v)]
    if not vals:
        return {
            "count": 0,
            "median": None,
            "min": None,
            "max": None,
            "q1": None,
            "q3": None,
            "values": [],
        }
    return {
        "count": int(len(vals)),
        "median": float(_median(vals)),
        "min": float(min(vals)),
        "max": float(max(vals)),
        "q1": float(_percentile(vals, 0.25)),
        "q3": float(_percentile(vals, 0.75)),
        "values": [round(float(v), 8) for v in vals[: max(0, int(preview_limit))]],
    }


@dataclass
class CalibrationSegment:
    kind: str
    samples: List[Dict[str, Any]]


class EncoderCalibrationCollector:
    """
    Runtime-safe sample collector + robust auto calibration estimator.
    """

    def __init__(
        self,
        *,
        base_step_m: float,
        k_left_old: float,
        k_right_old: float,
        track_width_old_m: float,
        cfg: Optional[Dict[str, Any]] = None,
        max_samples: int = 60000,
    ):
        cfg = dict(cfg or {})
        self.base_step_m = _safe_float(base_step_m, math.nan)
        if not math.isfinite(self.base_step_m) or self.base_step_m <= 0.0:
            raise ValueError("KIT0085 base_step_m must be configured and positive")
        self.k_left_old = max(1e-6, _safe_float(k_left_old, 1.0))
        self.k_right_old = max(1e-6, _safe_float(k_right_old, 1.0))
        self.track_width_old_m = _safe_float(track_width_old_m, math.nan)
        if not math.isfinite(self.track_width_old_m) or self.track_width_old_m <= 0.05:
            raise ValueError("KIT0085 track_width_old_m must be configured")

        self.min_combined_trust = _clamp(_safe_float(cfg.get("min_combined_trust"), 0.6), 0.0, 1.0)
        self.lidar_confidence_min = _clamp(_safe_float(cfg.get("lidar_confidence_min"), 0.2), 0.0, 1.0)

        self.straight_pwm_eps = max(0.001, _safe_float(cfg.get("straight_pwm_eps"), 0.08))
        self.straight_omega_cmd_max = max(0.001, _safe_float(cfg.get("straight_omega_cmd_max"), 0.18))
        self.straight_v_cmd_min = max(0.0, _safe_float(cfg.get("straight_v_cmd_min"), 0.08))

        self.rotate_pwm_eps = max(0.001, _safe_float(cfg.get("rotate_pwm_eps"), 0.12))
        self.rotate_pwm_min_abs = max(0.0, _safe_float(cfg.get("rotate_pwm_min_abs"), 0.20))
        self.rotate_omega_cmd_min = max(0.01, _safe_float(cfg.get("rotate_omega_cmd_min"), 0.45))

        self.min_segment_samples = max(3, _safe_int(cfg.get("min_segment_samples"), 6))
        self.min_straight_ref_distance_m = max(0.01, _safe_float(cfg.get("min_straight_ref_distance_m"), 0.12))
        self.min_straight_raw_distance_m = max(0.005, _safe_float(cfg.get("min_straight_raw_distance_m"), 0.05))
        self.min_rotation_yaw_rad = max(0.03, _safe_float(cfg.get("min_rotation_yaw_rad"), math.radians(10.0)))

        self.validation_improve_rel = _clamp(_safe_float(cfg.get("validation_improve_rel"), 0.03), 0.0, 0.5)
        self.validation_improve_abs_v = max(0.0, _safe_float(cfg.get("validation_improve_abs_v"), 0.005))
        self.validation_improve_abs_omega = max(0.0, _safe_float(cfg.get("validation_improve_abs_omega"), 0.005))

        self.max_samples = max(100, int(max_samples))

        self.sample_count = 0
        self.used_samples = 0
        self.rejected_samples = 0
        self.rejection_reasons: Dict[str, int] = {}

        self._samples: List[Dict[str, Any]] = []
        self._last_result: Dict[str, Any] = {}
        self._last_ts: Optional[float] = None

    def _count_reject(self, reason: str) -> None:
        self.rejected_samples += 1
        self.rejection_reasons[str(reason)] = int(self.rejection_reasons.get(str(reason), 0)) + 1

    def _normalize_sample(self, sample: Dict[str, Any]) -> Dict[str, Any]:
        s = dict(sample or {})
        ts = _safe_float(s.get("ts_s"), time.monotonic())
        dt = _safe_float(s.get("dt_s"), 0.0)
        if dt <= 0.0 and self._last_ts is not None and ts > self._last_ts:
            dt = max(1e-3, ts - self._last_ts)
        if dt <= 0.0:
            dt = 0.02

        out = {
            "ts_s": float(ts),
            "dt_s": float(dt),
            "pwm_l": _safe_float(s.get("pwm_l"), 0.0),
            "pwm_r": _safe_float(s.get("pwm_r"), 0.0),
            "v_cmd_mps": _safe_float(s.get("v_cmd_mps"), 0.0),
            "omega_cmd_rad_s": _safe_float(s.get("omega_cmd_rad_s"), 0.0),
            "pulse_delta_l": _safe_int(s.get("pulse_delta_l"), 0),
            "pulse_delta_r": _safe_int(s.get("pulse_delta_r"), 0),
            "distance_delta_l_scaled_m": _safe_float(s.get("distance_delta_l_scaled_m"), 0.0),
            "distance_delta_r_scaled_m": _safe_float(s.get("distance_delta_r_scaled_m"), 0.0),
            "distance_delta_l_raw_m": _safe_float(s.get("distance_delta_l_raw_m"), 0.0),
            "distance_delta_r_raw_m": _safe_float(s.get("distance_delta_r_raw_m"), 0.0),
            "ekf_v_mps": _safe_float(s.get("ekf_v_mps"), 0.0),
            "ekf_omega_rad_s": _safe_float(s.get("ekf_omega_rad_s"), 0.0),
            "ekf_theta_rad": _safe_float(s.get("ekf_theta_rad"), 0.0),
            "ekf_x_m": _safe_float(s.get("ekf_x_m"), 0.0),
            "ekf_y_m": _safe_float(s.get("ekf_y_m"), 0.0),
            "lidar_confidence": _safe_float(s.get("lidar_confidence"), 0.0),
            "wall_conf": _safe_float(s.get("wall_conf"), 0.0),
            "lidar_ok": bool(s.get("lidar_ok", False)),
            "symmetry_fault_active": bool(s.get("symmetry_fault_active", False)),
            "encoder_dropout": bool(s.get("encoder_dropout", False)),
            "combined_trust": _clamp(_safe_float(s.get("combined_trust"), 0.0), 0.0, 1.0),
            "ekf_usage_mode": str(s.get("ekf_usage_mode", "REJECT") or "REJECT").upper(),
            "flags": list(s.get("flags", []) or []),
        }
        return out

    def ingest(self, sample: Dict[str, Any]) -> bool:
        self.sample_count += 1
        s = self._normalize_sample(sample)
        self._last_ts = float(s["ts_s"])

        if bool(s.get("symmetry_fault_active", False)):
            self._count_reject("symmetry_fault_active")
            return False
        if bool(s.get("encoder_dropout", False)):
            self._count_reject("encoder_dropout")
            return False
        if float(s.get("combined_trust", 0.0)) < self.min_combined_trust:
            self._count_reject("trust_low")
            return False
        if str(s.get("ekf_usage_mode", "REJECT")).upper() == "REJECT":
            self._count_reject("ekf_usage_reject")
            return False
        if not bool(s.get("lidar_ok", False)):
            self._count_reject("lidar_not_ok")
            return False
        if not _is_finite(s.get("dt_s")) or float(s.get("dt_s", 0.0)) <= 0.0:
            self._count_reject("invalid_dt")
            return False

        self.used_samples += 1
        self._samples.append(s)
        if len(self._samples) > self.max_samples:
            drop_n = len(self._samples) - self.max_samples
            if drop_n > 0:
                del self._samples[:drop_n]
        return True

    def _classify(self, s: Dict[str, Any]) -> str:
        pwm_l = float(s.get("pwm_l", 0.0))
        pwm_r = float(s.get("pwm_r", 0.0))
        v_cmd = float(s.get("v_cmd_mps", 0.0))
        omega_cmd = float(s.get("omega_cmd_rad_s", 0.0))

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

    def _split_segments(self) -> List[CalibrationSegment]:
        out: List[CalibrationSegment] = []
        curr_kind: Optional[str] = None
        curr_samples: List[Dict[str, Any]] = []

        for s in self._samples:
            kind = self._classify(s)
            if curr_kind is None:
                curr_kind = kind
                curr_samples = [s]
                continue
            if kind == curr_kind:
                curr_samples.append(s)
                continue
            if curr_kind in ("STRAIGHT", "ROTATE") and len(curr_samples) >= self.min_segment_samples:
                out.append(CalibrationSegment(kind=curr_kind, samples=list(curr_samples)))
            curr_kind = kind
            curr_samples = [s]

        if curr_kind in ("STRAIGHT", "ROTATE") and len(curr_samples) >= self.min_segment_samples:
            out.append(CalibrationSegment(kind=curr_kind, samples=list(curr_samples)))
        return out

    @staticmethod
    def _segment_ref_distance(seg: CalibrationSegment) -> float:
        if not seg.samples:
            return 0.0
        s0 = seg.samples[0]
        s1 = seg.samples[-1]
        x0 = _safe_float(s0.get("ekf_x_m"), math.nan)
        y0 = _safe_float(s0.get("ekf_y_m"), math.nan)
        x1 = _safe_float(s1.get("ekf_x_m"), math.nan)
        y1 = _safe_float(s1.get("ekf_y_m"), math.nan)
        if all(_is_finite(v) for v in (x0, y0, x1, y1)):
            d = float(math.hypot(x1 - x0, y1 - y0))
            if d > 1e-6:
                return d
        dist_int = 0.0
        for s in seg.samples:
            dt = max(1e-6, _safe_float(s.get("dt_s"), 0.0))
            v = abs(_safe_float(s.get("ekf_v_mps"), 0.0))
            dist_int += v * dt
        return float(max(0.0, dist_int))

    @staticmethod
    def _segment_ref_yaw(seg: CalibrationSegment) -> float:
        if not seg.samples:
            return 0.0
        th0 = _safe_float(seg.samples[0].get("ekf_theta_rad"), math.nan)
        th1 = _safe_float(seg.samples[-1].get("ekf_theta_rad"), math.nan)
        if _is_finite(th0) and _is_finite(th1):
            yaw = abs(_wrap_angle_rad(th1 - th0))
            if yaw > 1e-6:
                return float(yaw)
        yaw_int = 0.0
        for s in seg.samples:
            dt = max(1e-6, _safe_float(s.get("dt_s"), 0.0))
            omega = abs(_safe_float(s.get("ekf_omega_rad_s"), 0.0))
            yaw_int += omega * dt
        return float(max(0.0, yaw_int))

    def _validate(
        self,
        straight_segments: List[CalibrationSegment],
        rotate_segments: List[CalibrationSegment],
        *,
        k_left_new: float,
        k_right_new: float,
        track_width_new_m: float,
    ) -> Dict[str, Any]:
        straight_before: List[float] = []
        straight_after: List[float] = []
        rotate_before: List[float] = []
        rotate_after: List[float] = []

        for seg in straight_segments:
            for s in seg.samples:
                dt = max(1e-6, _safe_float(s.get("dt_s"), 0.0))
                raw_l = _safe_float(s.get("distance_delta_l_raw_m"), 0.0)
                raw_r = _safe_float(s.get("distance_delta_r_raw_m"), 0.0)
                v_ekf = _safe_float(s.get("ekf_v_mps"), math.nan)
                if not _is_finite(v_ekf):
                    continue
                v_l_old = (raw_l * self.k_left_old) / dt
                v_r_old = (raw_r * self.k_right_old) / dt
                v_l_new = (raw_l * k_left_new) / dt
                v_r_new = (raw_r * k_right_new) / dt
                v_old = 0.5 * (v_l_old + v_r_old)
                v_new = 0.5 * (v_l_new + v_r_new)
                # Straight sections must improve both canonical linear velocity
                # and left/right consistency against EKF reference speed.
                e_old = (abs(v_old - v_ekf) + abs(v_l_old - v_ekf) + abs(v_r_old - v_ekf)) / 3.0
                e_new = (abs(v_new - v_ekf) + abs(v_l_new - v_ekf) + abs(v_r_new - v_ekf)) / 3.0
                straight_before.append(e_old)
                straight_after.append(e_new)

        for seg in rotate_segments:
            for s in seg.samples:
                dt = max(1e-6, _safe_float(s.get("dt_s"), 0.0))
                raw_l = _safe_float(s.get("distance_delta_l_raw_m"), 0.0)
                raw_r = _safe_float(s.get("distance_delta_r_raw_m"), 0.0)
                om_ekf = _safe_float(s.get("ekf_omega_rad_s"), math.nan)
                if not _is_finite(om_ekf):
                    continue
                v_l_old = (raw_l * self.k_left_old) / dt
                v_r_old = (raw_r * self.k_right_old) / dt
                v_l_new = (raw_l * k_left_new) / dt
                v_r_new = (raw_r * k_right_new) / dt
                om_old = (v_r_old - v_l_old) / max(1e-6, self.track_width_old_m)
                om_new = (v_r_new - v_l_new) / max(1e-6, track_width_new_m)
                rotate_before.append(abs(om_old - om_ekf))
                rotate_after.append(abs(om_new - om_ekf))

        v_before = _median(straight_before)
        v_after = _median(straight_after)
        w_before = _median(rotate_before)
        w_after = _median(rotate_after)

        has_v = len(straight_before) > 0 and _is_finite(v_before) and _is_finite(v_after)
        has_w = len(rotate_before) > 0 and _is_finite(w_before) and _is_finite(w_after)

        v_improved = False
        w_improved = False

        if has_v:
            need_v = max(self.validation_improve_abs_v, abs(v_before) * self.validation_improve_rel)
            v_improved = bool(v_after <= (v_before - need_v))
        if has_w:
            need_w = max(self.validation_improve_abs_omega, abs(w_before) * self.validation_improve_rel)
            w_improved = bool(w_after <= (w_before - need_w))

        valid = bool(v_improved and w_improved)
        reason = "OK" if valid else "VALIDATION_NO_IMPROVEMENT"
        if not has_v:
            valid = False
            reason = "VALIDATION_INSUFFICIENT_STRAIGHT"
        elif not has_w:
            valid = False
            reason = "VALIDATION_INSUFFICIENT_ROTATE"

        return {
            "valid": bool(valid),
            "reason": str(reason),
            "straight_error_before": (None if not _is_finite(v_before) else float(v_before)),
            "straight_error_after": (None if not _is_finite(v_after) else float(v_after)),
            "rotate_error_before": (None if not _is_finite(w_before) else float(w_before)),
            "rotate_error_after": (None if not _is_finite(w_after) else float(w_after)),
            "straight_samples": int(len(straight_before)),
            "rotate_samples": int(len(rotate_before)),
            "straight_improved": bool(v_improved),
            "rotate_improved": bool(w_improved),
        }

    def run_calibration(self) -> Dict[str, Any]:
        segments = self._split_segments()
        straight_segments = [seg for seg in segments if seg.kind == "STRAIGHT"]
        rotate_segments = [seg for seg in segments if seg.kind == "ROTATE"]

        k_left_estimates: List[float] = []
        k_right_estimates: List[float] = []

        for seg in straight_segments:
            d_ref = self._segment_ref_distance(seg)
            if d_ref < self.min_straight_ref_distance_m:
                continue
            d_raw_l = sum(abs(_safe_float(s.get("distance_delta_l_raw_m"), 0.0)) for s in seg.samples)
            d_raw_r = sum(abs(_safe_float(s.get("distance_delta_r_raw_m"), 0.0)) for s in seg.samples)
            if d_raw_l >= self.min_straight_raw_distance_m:
                k_left_estimates.append(d_ref / d_raw_l)
            if d_raw_r >= self.min_straight_raw_distance_m:
                k_right_estimates.append(d_ref / d_raw_r)

        k_left_filtered = _iqr_filter(k_left_estimates)
        k_right_filtered = _iqr_filter(k_right_estimates)

        k_left_new = float(_median(k_left_filtered)) if k_left_filtered else float(self.k_left_old)
        k_right_new = float(_median(k_right_filtered)) if k_right_filtered else float(self.k_right_old)

        b_estimates: List[float] = []
        for seg in rotate_segments:
            yaw_ref = self._segment_ref_yaw(seg)
            if yaw_ref < self.min_rotation_yaw_rad:
                continue
            s_l = sum(_safe_float(s.get("distance_delta_l_raw_m"), 0.0) * k_left_new for s in seg.samples)
            s_r = sum(_safe_float(s.get("distance_delta_r_raw_m"), 0.0) * k_right_new for s in seg.samples)
            num = abs(s_r - s_l)
            if num <= 1e-9:
                continue
            b_est = num / max(1e-9, yaw_ref)
            if _is_finite(b_est) and b_est > 0.05:
                b_estimates.append(b_est)

        b_filtered = _iqr_filter(b_estimates)
        track_width_new = float(_median(b_filtered)) if b_filtered else float(self.track_width_old_m)

        validation = self._validate(
            straight_segments,
            rotate_segments,
            k_left_new=k_left_new,
            k_right_new=k_right_new,
            track_width_new_m=track_width_new,
        )

        valid = bool(validation.get("valid", False))
        reason = str(validation.get("reason", "VALIDATION_FAILED"))
        if not k_left_filtered or not k_right_filtered:
            valid = False
            reason = "INSUFFICIENT_STRAIGHT_SEGMENTS"
        elif not b_filtered:
            valid = False
            reason = "INSUFFICIENT_ROTATION_SEGMENTS"

        result = {
            "valid": bool(valid),
            "reason": str(reason),
            "sample_count": int(self.sample_count),
            "used_samples": int(self.used_samples),
            "rejected_samples": int(self.rejected_samples),
            "rejection_reasons": dict(self.rejection_reasons),
            "segment_count": int(len(segments)),
            "straight_segment_count": int(len(straight_segments)),
            "rotation_segment_count": int(len(rotate_segments)),
            "k_L_estimates": _stats(k_left_filtered),
            "k_R_estimates": _stats(k_right_filtered),
            "B_estimates": _stats(b_filtered),
            "final_values": {
                "k_L_old": float(self.k_left_old),
                "k_R_old": float(self.k_right_old),
                "B_old": float(self.track_width_old_m),
                "k_L_new": float(k_left_new),
                "k_R_new": float(k_right_new),
                "B_new": float(track_width_new),
            },
            "validation_error_before": {
                "straight_v_abs": validation.get("straight_error_before"),
                "rotate_omega_abs": validation.get("rotate_error_before"),
            },
            "validation_error_after": {
                "straight_v_abs": validation.get("straight_error_after"),
                "rotate_omega_abs": validation.get("rotate_error_after"),
            },
            "validation_details": dict(validation),
        }
        self._last_result = dict(result)
        return result

    def get_summary(self) -> Dict[str, Any]:
        if self._last_result:
            return dict(self._last_result)
        return {
            "sample_count": int(self.sample_count),
            "used_samples": int(self.used_samples),
            "rejected_samples": int(self.rejected_samples),
            "rejection_reasons": dict(self.rejection_reasons),
            "k_L_estimates": _stats([]),
            "k_R_estimates": _stats([]),
            "B_estimates": _stats([]),
            "final_values": {
                "k_L_old": float(self.k_left_old),
                "k_R_old": float(self.k_right_old),
                "B_old": float(self.track_width_old_m),
                "k_L_new": None,
                "k_R_new": None,
                "B_new": None,
            },
            "validation_error_before": {"straight_v_abs": None, "rotate_omega_abs": None},
            "validation_error_after": {"straight_v_abs": None, "rotate_omega_abs": None},
            "valid": False,
            "reason": "NOT_RUN",
        }


def _extract_lidar_confidence(lidar_summary: Dict[str, Any]) -> float:
    s = dict(lidar_summary or {})
    for key in (
        "lidar_pose_confidence",
        "final_confidence_emitted",
        "final_confidence",
        "matcher_confidence",
    ):
        val = s.get(key)
        if _is_finite(val):
            return _clamp(_safe_float(val, 0.0), 0.0, 1.0)
    return 0.0


def _derive_wall_conf(lidar_summary: Dict[str, Any]) -> float:
    s = dict(lidar_summary or {})
    for key in ("wall_conf", "wall_confidence"):
        val = s.get(key)
        if _is_finite(val):
            return _clamp(_safe_float(val, 0.0), 0.0, 1.0)

    min_narrow = _safe_float(s.get("min_dist_narrow"), math.nan)
    if _is_finite(min_narrow):
        # heuristic: confidence increases as close wall feature appears in narrow FOV
        return _clamp(1.0 - min(1.0, max(0.0, min_narrow) / 2.0), 0.0, 1.0)
    return 0.0


def _build_sample(
    *,
    ts_s: float,
    dt_s: float,
    pwm_l: float,
    pwm_r: float,
    v_cmd_mps: float,
    omega_cmd_rad_s: float,
    pulse_delta_l: int,
    pulse_delta_r: int,
    direction_l: float,
    direction_r: float,
    direction_conf_l: bool,
    direction_conf_r: bool,
    distance_delta_l_scaled_m: float,
    distance_delta_r_scaled_m: float,
    ekf_v_mps: float,
    ekf_omega_rad_s: float,
    ekf_theta_rad: float,
    ekf_x_m: float,
    ekf_y_m: float,
    lidar_summary: Dict[str, Any],
    lidar_health: str,
    combined_trust: float,
    ekf_usage_mode: str,
    symmetry_fault_active: bool,
    encoder_dropout: bool,
    flags: List[str],
    base_step_m: float,
    k_left_old: float,
    k_right_old: float,
    lidar_confidence_min: float,
) -> Dict[str, Any]:
    base_step = _safe_float(base_step_m, math.nan)
    if not math.isfinite(base_step) or base_step <= 0.0:
        raise ValueError("KIT0085 base_step_m must be configured and positive")
    k_l_old = max(1e-6, _safe_float(k_left_old, 1.0))
    k_r_old = max(1e-6, _safe_float(k_right_old, 1.0))

    raw_l_unsigned = abs(_safe_int(pulse_delta_l, 0)) * base_step
    raw_r_unsigned = abs(_safe_int(pulse_delta_r, 0)) * base_step

    dir_l = _sign(_safe_float(direction_l, 0.0)) if bool(direction_conf_l) else 0.0
    dir_r = _sign(_safe_float(direction_r, 0.0)) if bool(direction_conf_r) else 0.0

    raw_l_signed = dir_l * raw_l_unsigned
    raw_r_signed = dir_r * raw_r_unsigned

    # Fallback from scaled channel if direction is uncertain.
    if abs(raw_l_signed) <= 1e-12 and abs(_safe_float(distance_delta_l_scaled_m, 0.0)) > 0.0:
        raw_l_signed = _safe_float(distance_delta_l_scaled_m, 0.0) / k_l_old
    if abs(raw_r_signed) <= 1e-12 and abs(_safe_float(distance_delta_r_scaled_m, 0.0)) > 0.0:
        raw_r_signed = _safe_float(distance_delta_r_scaled_m, 0.0) / k_r_old

    # At very low pulse deltas (0/1), pulse-only reconstruction is heavily quantized.
    # Prefer scaled-channel reconstruction to avoid systematic bias in calibration.
    if abs(_safe_int(pulse_delta_l, 0)) <= 1 and abs(_safe_float(distance_delta_l_scaled_m, 0.0)) > 0.0:
        raw_l_signed = _safe_float(distance_delta_l_scaled_m, 0.0) / k_l_old
    if abs(_safe_int(pulse_delta_r, 0)) <= 1 and abs(_safe_float(distance_delta_r_scaled_m, 0.0)) > 0.0:
        raw_r_signed = _safe_float(distance_delta_r_scaled_m, 0.0) / k_r_old

    lidar_conf = _extract_lidar_confidence(lidar_summary)
    lidar_ok = bool(str(lidar_health or "").upper() == "OK" and lidar_conf >= float(lidar_confidence_min))

    return {
        "ts_s": float(ts_s),
        "dt_s": float(max(1e-6, _safe_float(dt_s, 0.02))),
        "pwm_l": float(_safe_float(pwm_l, 0.0)),
        "pwm_r": float(_safe_float(pwm_r, 0.0)),
        "v_cmd_mps": float(_safe_float(v_cmd_mps, 0.0)),
        "omega_cmd_rad_s": float(_safe_float(omega_cmd_rad_s, 0.0)),
        "pulse_delta_l": int(_safe_int(pulse_delta_l, 0)),
        "pulse_delta_r": int(_safe_int(pulse_delta_r, 0)),
        "distance_delta_l_scaled_m": float(_safe_float(distance_delta_l_scaled_m, 0.0)),
        "distance_delta_r_scaled_m": float(_safe_float(distance_delta_r_scaled_m, 0.0)),
        "distance_delta_l_raw_m": float(raw_l_signed),
        "distance_delta_r_raw_m": float(raw_r_signed),
        "ekf_v_mps": float(_safe_float(ekf_v_mps, 0.0)),
        "ekf_omega_rad_s": float(_safe_float(ekf_omega_rad_s, 0.0)),
        "ekf_theta_rad": float(_safe_float(ekf_theta_rad, 0.0)),
        "ekf_x_m": float(_safe_float(ekf_x_m, 0.0)),
        "ekf_y_m": float(_safe_float(ekf_y_m, 0.0)),
        "lidar_confidence": float(lidar_conf),
        "wall_conf": float(_derive_wall_conf(lidar_summary)),
        "lidar_ok": bool(lidar_ok),
        "symmetry_fault_active": bool(symmetry_fault_active),
        "encoder_dropout": bool(encoder_dropout),
        "combined_trust": float(_clamp(_safe_float(combined_trust, 0.0), 0.0, 1.0)),
        "ekf_usage_mode": str(ekf_usage_mode or "REJECT").upper(),
        "flags": list(flags or []),
    }


def build_runtime_calibration_sample(
    *,
    now_s: float,
    dt_s: float,
    pwm_l: float,
    pwm_r: float,
    v_cmd_mps: float,
    omega_cmd_rad_s: float,
    enc_snapshot: Any,
    encoder_reliability: Dict[str, Any],
    ekf_state: Dict[str, Any],
    lidar_summary: Dict[str, Any],
    lidar_health: str,
    base_step_m: float,
    k_left_old: float,
    k_right_old: float,
    lidar_confidence_min: float = 0.2,
) -> Dict[str, Any]:
    rel = dict(encoder_reliability or {})
    flags = list(rel.get("flags", []) or [])
    dropout = bool(
        "LEFT_ENCODER_DROPOUT_SUSPECT" in flags
        or "RIGHT_ENCODER_DROPOUT_SUSPECT" in flags
        or "PWM_ENCODER_SYMMETRY_FAULT" in flags
    )

    return _build_sample(
        ts_s=float(_safe_float(now_s, time.monotonic())),
        dt_s=float(_safe_float(dt_s, 0.02)),
        pwm_l=float(_safe_float(pwm_l, 0.0)),
        pwm_r=float(_safe_float(pwm_r, 0.0)),
        v_cmd_mps=float(_safe_float(v_cmd_mps, 0.0)),
        omega_cmd_rad_s=float(_safe_float(omega_cmd_rad_s, 0.0)),
        pulse_delta_l=_safe_int(getattr(enc_snapshot, "left_pulse_delta", 0), 0) if enc_snapshot is not None else 0,
        pulse_delta_r=_safe_int(getattr(enc_snapshot, "right_pulse_delta", 0), 0) if enc_snapshot is not None else 0,
        direction_l=_safe_float(getattr(enc_snapshot, "left_direction", 0.0), 0.0) if enc_snapshot is not None else 0.0,
        direction_r=_safe_float(getattr(enc_snapshot, "right_direction", 0.0), 0.0) if enc_snapshot is not None else 0.0,
        direction_conf_l=bool(getattr(enc_snapshot, "left_direction_confident", False)) if enc_snapshot is not None else False,
        direction_conf_r=bool(getattr(enc_snapshot, "right_direction_confident", False)) if enc_snapshot is not None else False,
        distance_delta_l_scaled_m=_safe_float(getattr(enc_snapshot, "left_distance_delta", 0.0), 0.0) if enc_snapshot is not None else 0.0,
        distance_delta_r_scaled_m=_safe_float(getattr(enc_snapshot, "right_distance_delta", 0.0), 0.0) if enc_snapshot is not None else 0.0,
        ekf_v_mps=_safe_float((ekf_state or {}).get("v"), 0.0),
        ekf_omega_rad_s=_safe_float((ekf_state or {}).get("omega_rad_s"), 0.0),
        ekf_theta_rad=_safe_float((ekf_state or {}).get("theta"), 0.0),
        ekf_x_m=_safe_float((ekf_state or {}).get("x"), 0.0),
        ekf_y_m=_safe_float((ekf_state or {}).get("y"), 0.0),
        lidar_summary=dict(lidar_summary or {}),
        lidar_health=str(lidar_health or "N/A"),
        combined_trust=_safe_float(rel.get("combined_trust"), 0.0),
        ekf_usage_mode=str(rel.get("ekf_usage_mode", "REJECT") or "REJECT"),
        symmetry_fault_active=bool(rel.get("symmetry_fault_active", False)),
        encoder_dropout=bool(dropout),
        flags=flags,
        base_step_m=float(base_step_m),
        k_left_old=float(k_left_old),
        k_right_old=float(k_right_old),
        lidar_confidence_min=float(lidar_confidence_min),
    )


def build_status_calibration_sample(
    status: Dict[str, Any],
    *,
    base_step_m: float,
    k_left_old: float,
    k_right_old: float,
    default_dt_s: float = 0.1,
    lidar_confidence_min: float = 0.2,
) -> Dict[str, Any]:
    st = dict(status or {})
    pwm = dict(st.get("pwm") or {})
    enc = dict(st.get("encoder") or {})
    left = dict((enc.get("left") or {}).get("snapshot") or {})
    right = dict((enc.get("right") or {}).get("snapshot") or {})
    computed = dict((enc.get("computed") or {}))
    rel = dict(st.get("encoder_reliability") or {})
    flags = list(rel.get("flags", []) or [])

    pose = dict(st.get("pose") or {})
    lidar = dict(st.get("lidar") or {})

    dropout = bool(
        "LEFT_ENCODER_DROPOUT_SUSPECT" in flags
        or "RIGHT_ENCODER_DROPOUT_SUSPECT" in flags
        or "PWM_ENCODER_SYMMETRY_FAULT" in flags
    )

    dt = _safe_float(computed.get("sample_dt_s"), 0.0)
    if dt <= 0.0:
        dt = float(max(1e-3, _safe_float(default_dt_s, 0.1)))

    return _build_sample(
        ts_s=_safe_float(st.get("time"), time.monotonic()),
        dt_s=dt,
        pwm_l=_safe_float(pwm.get("left"), 0.0),
        pwm_r=_safe_float(pwm.get("right"), 0.0),
        v_cmd_mps=_safe_float(st.get("v_cmd"), 0.0),
        omega_cmd_rad_s=_safe_float(st.get("omega_target"), 0.0),
        pulse_delta_l=_safe_int(left.get("pulse_delta"), 0),
        pulse_delta_r=_safe_int(right.get("pulse_delta"), 0),
        direction_l=_safe_float(left.get("direction"), 0.0),
        direction_r=_safe_float(right.get("direction"), 0.0),
        direction_conf_l=bool(left.get("direction_confident", False)),
        direction_conf_r=bool(right.get("direction_confident", False)),
        distance_delta_l_scaled_m=_safe_float(left.get("distance_delta_m"), 0.0),
        distance_delta_r_scaled_m=_safe_float(right.get("distance_delta_m"), 0.0),
        ekf_v_mps=_safe_float(pose.get("v"), 0.0),
        ekf_omega_rad_s=_safe_float(pose.get("omega_rad_s"), 0.0),
        ekf_theta_rad=_safe_float(pose.get("theta"), 0.0),
        ekf_x_m=_safe_float(pose.get("x"), 0.0),
        ekf_y_m=_safe_float(pose.get("y"), 0.0),
        lidar_summary=lidar,
        lidar_health=str(st.get("lidar_health", "N/A") or "N/A"),
        combined_trust=_safe_float(rel.get("combined_trust"), 0.0),
        ekf_usage_mode=str(rel.get("ekf_usage_mode", "REJECT") or "REJECT"),
        symmetry_fault_active=bool(rel.get("symmetry_fault_active", False)),
        encoder_dropout=bool(dropout),
        flags=flags,
        base_step_m=float(base_step_m),
        k_left_old=float(k_left_old),
        k_right_old=float(k_right_old),
        lidar_confidence_min=float(lidar_confidence_min),
    )


def _read_json(path: Path) -> Dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    os.replace(tmp, path)


def load_fizika_calibration_params(fizika_path: Path) -> Dict[str, float]:
    cfg = _read_json(Path(fizika_path))
    if "lepes_hossz_m" not in cfg or "nyomtav_szelesseg_m" not in cfg:
        raise ValueError("KIT0085 physics configuration is incomplete")
    return {
        "lepes_hossz_m": max(1e-6, _safe_float(cfg["lepes_hossz_m"], math.nan)),
        "lepes_hossz_bal_szorzo": max(1e-6, _safe_float(cfg.get("lepes_hossz_bal_szorzo", cfg.get("lepes_hossz_bal_scale", 1.0)), 1.0)),
        "lepes_hossz_jobb_szorzo": max(1e-6, _safe_float(cfg.get("lepes_hossz_jobb_szorzo", cfg.get("lepes_hossz_jobb_scale", 1.0)), 1.0)),
        "nyomtav_szelesseg_m": max(0.05, _safe_float(cfg["nyomtav_szelesseg_m"], math.nan)),
    }


def apply_encoder_calibration(
    fizika_path: Path,
    *,
    k_left_new: float,
    k_right_new: float,
    track_width_new_m: float,
    backup_dir: Path,
) -> Dict[str, Any]:
    fizika_path = Path(fizika_path)
    backup_dir = Path(backup_dir)
    backup_dir.mkdir(parents=True, exist_ok=True)

    cfg = _read_json(fizika_path)
    if not cfg:
        raise RuntimeError(f"Cannot read config: {fizika_path}")

    ts = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
    backup_path = backup_dir / f"fizika_pre_encoder_auto_{ts}.json"
    shutil.copy2(fizika_path, backup_path)

    old_values = {
        "lepes_hossz_bal_szorzo": _safe_float(cfg.get("lepes_hossz_bal_szorzo", cfg.get("lepes_hossz_bal_scale", 1.0)), 1.0),
        "lepes_hossz_jobb_szorzo": _safe_float(cfg.get("lepes_hossz_jobb_szorzo", cfg.get("lepes_hossz_jobb_scale", 1.0)), 1.0),
        "nyomtav_szelesseg_m": _safe_float(cfg["nyomtav_szelesseg_m"], math.nan),
    }

    cfg["lepes_hossz_bal_szorzo"] = float(k_left_new)
    cfg["lepes_hossz_jobb_szorzo"] = float(k_right_new)
    cfg["nyomtav_szelesseg_m"] = float(track_width_new_m)
    _write_json_atomic(fizika_path, cfg)

    manifest = {
        "ok": True,
        "type": "encoder_auto_apply",
        "ts_utc": ts,
        "fizika_path": str(fizika_path),
        "backup_path": str(backup_path),
        "old_values": old_values,
        "new_values": {
            "lepes_hossz_bal_szorzo": float(k_left_new),
            "lepes_hossz_jobb_szorzo": float(k_right_new),
            "nyomtav_szelesseg_m": float(track_width_new_m),
        },
    }
    manifest_path = backup_dir / f"encoder_auto_apply_{ts}.json"
    _write_json_atomic(manifest_path, manifest)
    _write_json_atomic(backup_dir / "encoder_auto_latest_backup.json", manifest)
    manifest["manifest_path"] = str(manifest_path)
    return manifest


def rollback_encoder_calibration(
    fizika_path: Path,
    *,
    backup_path: Path,
    backup_dir: Path,
) -> Dict[str, Any]:
    fizika_path = Path(fizika_path)
    backup_path = Path(backup_path)
    backup_dir = Path(backup_dir)
    backup_dir.mkdir(parents=True, exist_ok=True)

    if not backup_path.exists():
        raise RuntimeError(f"Backup not found: {backup_path}")

    curr_cfg = _read_json(fizika_path)
    bak_cfg = _read_json(backup_path)
    if not bak_cfg:
        raise RuntimeError(f"Backup is empty/invalid: {backup_path}")

    ts = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
    rollback_backup = backup_dir / f"fizika_pre_encoder_rollback_{ts}.json"
    if curr_cfg:
        _write_json_atomic(rollback_backup, curr_cfg)

    _write_json_atomic(fizika_path, bak_cfg)

    manifest = {
        "ok": True,
        "type": "encoder_auto_rollback",
        "ts_utc": ts,
        "fizika_path": str(fizika_path),
        "rollback_from": str(backup_path),
        "rollback_backup_current": str(rollback_backup),
    }
    manifest_path = backup_dir / f"encoder_auto_rollback_{ts}.json"
    _write_json_atomic(manifest_path, manifest)
    manifest["manifest_path"] = str(manifest_path)
    return manifest
