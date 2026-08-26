#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Központi sebességkorlát-kezelő (Single Source of Truth).

Cél:
- Aktív módban egyetlen lineáris és szögsebesség limit legyen.
- A fokozat csak egy szorzó legyen a v_max-ra.
- Minden réteg ugyanazt a runtime objektumot használja.
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple


CANONICAL_CONTROL_MODE = "UNIFIED"
CANONICAL_PROFILE = "UNIFIED"


@dataclass
class MotionProfile:
    name: str
    v_max: float
    v_min: float
    w_max: float
    w_min: float
    accel_limit: float
    jerk_limit: float
    source: str

    def as_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "v_max": self.v_max,
            "v_min": self.v_min,
            "w_max": self.w_max,
            "w_min": self.w_min,
            "accel_limit": self.accel_limit,
            "jerk_limit": self.jerk_limit,
            "source": self.source,
        }


def _safe_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _required_finite(profile: Dict[str, Any], key: str) -> float:
    if key not in profile:
        raise ValueError(f"motion_profile_missing:{CANONICAL_PROFILE}.{key}")
    try:
        value = float(profile[key])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"motion_profile_invalid:{CANONICAL_PROFILE}.{key}") from exc
    if not math.isfinite(value):
        raise ValueError(f"motion_profile_non_finite:{CANONICAL_PROFILE}.{key}")
    return float(value)


class SpeedLimitsRuntime:
    """
    Futásidejű sebességlimitek kezelése.
    A profile + fokozat alapján állítja elő az effektív parancslimitet.
    """

    def __init__(self, logger=None):
        self.logger = logger
        self.mode = CANONICAL_CONTROL_MODE
        self.profile = MotionProfile(
            name=CANONICAL_PROFILE,
            v_max=0.30,
            v_min=0.15,
            w_max=1.68,
            w_min=0.0,
            accel_limit=0.35,
            jerk_limit=0.70,
            source="safe_default.UNIFIED",
        )
        # Construction is side-effect free for unit-level consumers.  The
        # robot runtime always calls load_from_config(), which replaces this
        # neutral floor with the mandatory active four-curve range.
        self.calibrated_wheel_min_mps = 0.0
        self.calibrated_wheel_max_mps = 0.30
        self.track_width_m = 0.3557
        self.gear_level = 0
        self.gear_ratio = 0.0
        self.max_pwm_cap = 0.90
        self._last_limit_debug = {
            "mode": self.mode,
            "v_commanded": 0.0,
            "v_limited": 0.0,
            "limiter": "",
        }

    def _log(self):
        if not self.logger:
            return
        try:
            self.logger.info(
                "[SpeedLimits] "
                f"mode={self.profile.name} "
                f"v_max={self.profile.v_max:.2f} "
                f"w_max={self.profile.w_max:.2f} "
                f"source={self.profile.source}"
            )
        except Exception:
            pass

    def _profile_from_config(self, vezerles: Dict[str, Any], mode: str) -> MotionProfile:
        motion_profiles = (vezerles or {}).get("motion_profiles") or {}
        profile_name = CANONICAL_PROFILE
        prof = motion_profiles.get(profile_name)
        if not isinstance(prof, dict):
            raise ValueError(f"motion_profile_missing:{profile_name}")

        profile_vmax = _required_finite(prof, "v_max")
        profile_wmax = _required_finite(prof, "w_max")
        profile_vmin = max(0.0, _required_finite(prof, "v_min"))
        profile_wmin = max(0.0, _required_finite(prof, "w_min"))
        profile_accel = max(0.01, _required_finite(prof, "accel"))
        profile_jerk = max(0.01, _required_finite(prof, "jerk"))

        return MotionProfile(
            name=profile_name,
            v_max=max(0.01, profile_vmax),
            v_min=profile_vmin,
            w_max=max(0.01, profile_wmax),
            w_min=profile_wmin,
            accel_limit=profile_accel,
            jerk_limit=profile_jerk,
            source=f"motion_profiles.{profile_name}",
        )

    def set_gear_from_level(self, speed_level: int):
        lvl = max(0, min(9, abs(int(speed_level))))
        self.gear_level = int(lvl)
        # Level 0 = crawl mód (15% v_max), nem blokkolás.
        if lvl == 0:
            self.gear_ratio = 0.15
            return
        self.gear_ratio = max(0.1, min(1.0, lvl / 9.0))

    def set_gear_ratio(self, ratio: float):
        r = max(0.0, min(1.0, _safe_float(ratio, 0.0)))
        self.gear_ratio = r
        self.gear_level = int(round(r * 9.0))

    def _apply_calibrated_caps(self, profile: MotionProfile) -> MotionProfile:
        capped = copy.deepcopy(profile)
        capped.v_max = min(float(capped.v_max), float(self.calibrated_wheel_max_mps))
        capped.v_min = min(
            float(capped.v_max),
            max(float(capped.v_min), float(self.calibrated_wheel_min_mps)),
        )
        physical_w_max = (
            2.0 * float(self.calibrated_wheel_max_mps) / max(0.01, float(self.track_width_m))
        )
        capped.w_max = min(float(capped.w_max), float(physical_w_max))
        capped.w_min = min(float(capped.w_min), float(capped.w_max))
        if (
            abs(float(capped.v_max) - float(profile.v_max)) > 1e-9
            or abs(float(capped.v_min) - float(profile.v_min)) > 1e-9
            or abs(float(capped.w_max) - float(profile.w_max)) > 1e-9
        ):
            capped.source = f"{profile.source}+active_speed_map_cap"
        return capped

    def load_from_config(
        self,
        vezerles: Dict[str, Any],
        control_mode: str,
        speed_level: int,
        max_pwm_cap: float,
        *,
        wheel_speed_range_mps: Tuple[float, float],
        track_width_m: float,
    ):
        self.mode = CANONICAL_CONTROL_MODE
        wheel_min, wheel_max = wheel_speed_range_mps
        self.calibrated_wheel_min_mps = max(0.0, float(wheel_min))
        self.calibrated_wheel_max_mps = max(
            self.calibrated_wheel_min_mps,
            float(wheel_max),
        )
        self.track_width_m = max(0.01, float(track_width_m))
        self.profile = self._apply_calibrated_caps(
            self._profile_from_config(vezerles or {}, self.mode)
        )
        self.max_pwm_cap = max(0.05, min(1.0, _safe_float(max_pwm_cap, 0.90)))
        self.set_gear_from_level(speed_level)
        self._log()

    def update_runtime(self, updates: Dict[str, Any]):
        if not isinstance(updates, dict):
            return
        p = copy.deepcopy(self.profile)
        if "v_max" in updates:
            p.v_max = max(0.01, _safe_float(updates["v_max"], p.v_max))
        if "v_min" in updates:
            p.v_min = max(0.0, _safe_float(updates["v_min"], p.v_min))
        if "w_max" in updates:
            p.w_max = max(0.01, _safe_float(updates["w_max"], p.w_max))
        if "w_min" in updates:
            p.w_min = max(0.0, _safe_float(updates["w_min"], p.w_min))
        if "accel_limit" in updates:
            p.accel_limit = max(0.01, _safe_float(updates["accel_limit"], p.accel_limit))
        if "jerk_limit" in updates:
            p.jerk_limit = max(0.01, _safe_float(updates["jerk_limit"], p.jerk_limit))
        if "gear_level" in updates:
            self.set_gear_from_level(int(updates["gear_level"]))
        if "gear_ratio" in updates:
            self.set_gear_ratio(_safe_float(updates["gear_ratio"], self.gear_ratio))
        if "max_pwm_cap" in updates:
            self.max_pwm_cap = max(0.05, min(1.0, _safe_float(updates["max_pwm_cap"], self.max_pwm_cap)))
        self.profile = self._apply_calibrated_caps(p)
        self._log()

    @property
    def effective_v_max(self) -> float:
        if self.gear_ratio <= 0.0:
            return 0.0
        # A non-zero gear may not expose a command range below the common
        # minimum controllable translational speed.
        return max(0.01, self.profile.v_min, self.profile.v_max * self.gear_ratio)

    @property
    def effective_accel_limit(self) -> float:
        # A fokozat sebesség plafont skáláz, gyorsulás marad profilfüggő.
        return self.profile.accel_limit

    @property
    def effective_w_max(self) -> float:
        # Szögsebesség plafon egységes SSOT eléréshez (jelenleg gear-független).
        return max(0.01, float(self.profile.w_max))

    def clamp_command(
        self,
        v_cmd: float,
        omega_cmd: float,
        *,
        motion_source: Optional[str] = None,
    ) -> Tuple[float, float, Dict[str, Any]]:
        v_in = _safe_float(v_cmd, 0.0)
        w_in = _safe_float(omega_cmd, 0.0)
        v_out = v_in
        w_out = w_in
        limiter = ""

        src = str(motion_source or "")
        src_base = src.split(":", 1)[0]
        src_upper = src.upper()
        explicit_motion_target_pivot_context = bool(
            "MOTION_TARGET" in src_upper
            and ("SET_TWIST" in src_upper or "SET_MOTION_TARGET" in src_upper)
            and v_in >= 0.0
            and abs(v_in) <= min(0.006, max(1e-9, self.profile.v_min * 0.20))
            and abs(w_in) > 1e-6
        )
        explicit_rotate_context = (
            ("ROTATE_TO_HEADING" in src_upper)
            or ("HEADING_PRIMITIVE" in src_upper)
            or explicit_motion_target_pivot_context
        )
        vmax = self.effective_v_max
        # GUI analóg joy esetén a 0 gear (speed_level=0) nem blokkolhatja a mozgást.
        # Ilyenkor a profil alap v_max értékét használjuk (UNIFIED plafon marad).
        if vmax <= 0.0 and src_base == "GUI_JOYSTICK":
            vmax = float(self.profile.v_max)
        if vmax <= 0.0:
            v_out = 0.0
            limiter = f"SpeedLimits.{self.profile.name}.gear_ratio"
        else:
            if v_out > vmax:
                v_out = vmax
                limiter = f"SpeedLimits.{self.profile.name}.v_max"
            elif v_out < -vmax:
                v_out = -vmax
                limiter = f"SpeedLimits.{self.profile.name}.v_max"
            if 0.0 < abs(v_out) < self.profile.v_min:
                # Pure-rotate (kis fordulási sugár) parancsoknál a lineáris minimum
                # visszainjektálása oldalirányú sodródást okoz; maradjon v=0.
                rotate_dominant = False
                if abs(w_out) > 1e-6:
                    turning_radius_m = abs(v_out) / abs(w_out)
                    rotate_dominant = turning_radius_m <= 0.08
                if explicit_rotate_context or rotate_dominant:
                    v_out = 0.0
                    if not limiter:
                        limiter = f"SpeedLimits.{self.profile.name}.rotate_pure_v_zero"
                else:
                    v_out = self.profile.v_min if v_out > 0 else -self.profile.v_min
                    limiter = f"SpeedLimits.{self.profile.name}.v_min"

        if w_out > self.profile.w_max:
            w_out = self.profile.w_max
            if not limiter:
                limiter = f"SpeedLimits.{self.profile.name}.w_max"
        elif w_out < -self.profile.w_max:
            w_out = -self.profile.w_max
            if not limiter:
                limiter = f"SpeedLimits.{self.profile.name}.w_max"
        if 0.0 < abs(w_out) < self.profile.w_min:
            w_out = self.profile.w_min if w_out > 0 else -self.profile.w_min
            if not limiter:
                limiter = f"SpeedLimits.{self.profile.name}.w_min"

        debug = {
            "mode": self.profile.name,
            "v_commanded": v_in,
            "v_limited": v_out,
            "w_commanded": w_in,
            "w_limited": w_out,
            "limiter": limiter,
        }
        self._last_limit_debug = debug
        return v_out, w_out, debug

    def as_runtime_state(self) -> Dict[str, Any]:
        return {
            "mode": self.profile.name,
            "mode_raw": self.mode,
            "profile": self.profile.as_dict(),
            "v_max_active": self.effective_v_max,
            "w_max_active": self.profile.w_max,
            "accel_limit_active": self.profile.accel_limit,
            "jerk_limit_active": self.profile.jerk_limit,
            "gear_level": self.gear_level,
            "gear_ratio": self.gear_ratio,
            "pwm_cap": self.max_pwm_cap,
            "calibrated_wheel_range_mps": {
                "minimum": self.calibrated_wheel_min_mps,
                "maximum": self.calibrated_wheel_max_mps,
                "source": "active_speed_map.common_four_curve_range",
            },
            "track_width_m": self.track_width_m,
            "last_limit_debug": dict(self._last_limit_debug),
        }
