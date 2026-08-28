#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""V2.1 L7A closed-loop heading and turn guidance."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

from controller.motion_schema import (
    TURN_PRIMITIVE_IN_PLACE_ROTATE,
    TURN_PRIMITIVE_ONE_TRACK_PIVOT,
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
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on", "enable", "enabled"):
        return True
    if normalized in ("0", "false", "no", "off", "disable", "disabled", ""):
        return False
    return bool(default)


def _normalize_angle_deg(angle_deg: float) -> float:
    angle = _safe_float(angle_deg, 0.0)
    return ((angle + 180.0) % 360.0) - 180.0


def _angle_error_deg(target_deg: float, current_deg: float) -> float:
    return _normalize_angle_deg(
        _safe_float(target_deg, 0.0) - _safe_float(current_deg, 0.0)
    )


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
        now: float,
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
        self.started_at = float(now)
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

        if now is None:
            raise ValueError("heading_guidance_now_required")
        now = float(now)
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
