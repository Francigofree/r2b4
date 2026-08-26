#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Hard safety gate: filters motor PWM outputs based on safety state.
This is a non-negotiable safety layer that ONLY filters motor outputs.
It CANNOT change states, enqueue tasks, or call motors directly.
"""

from core.motion.clearance import (
    FRONT_SAFE_FLOOR_M as _FRONT_SAFE_FLOOR_M,
    FRONT_START_EXTRA_M as _FRONT_START_EXTRA_M,
    FRONT_START_GAIN_S as _FRONT_START_GAIN_S,
    FRONT_START_MAX_M as _FRONT_START_MAX_M,
    FRONT_STOP_GAIN_S as _FRONT_STOP_GAIN_S,
    FRONT_STOP_MAX_M as _FRONT_STOP_MAX_M,
    dynamic_front_clearance_thresholds,
)


class SafetyGate:
    """
    Hard safety gate that filters PWM outputs immediately before motor application.
    Hátrameneti proaktív fékezés: a hátsó LIDAR akadály közelségétől függő
    fokozatos PWM csillapítás, mielőtt az emergency stop szükségessé válna.
    """

    # Hátsó akadály-fékezés küszöbértékek (méterben).
    REAR_BRAKE_START_M = 0.30
    REAR_BRAKE_STOP_M = 0.10

    # Előrehaladásnál dinamikus front biztonsági küszöb:
    # a hard stop soha nem mehet FRONT_SAFE_FLOOR_M alá.
    # Korábbi értékek túl agresszívek voltak alacsony sebességnél:
    # v=0.10-nél a fékezési zóna 0.42–0.70 m volt, ami a falkövetésnél
    # (target 0.30 m) a PWM-et a motor minimum alá csökkentette.
    FRONT_SAFE_FLOOR_M = _FRONT_SAFE_FLOOR_M
    FRONT_STOP_GAIN_S = _FRONT_STOP_GAIN_S
    FRONT_STOP_MAX_M = _FRONT_STOP_MAX_M
    FRONT_START_EXTRA_M = _FRONT_START_EXTRA_M
    FRONT_START_GAIN_S = _FRONT_START_GAIN_S
    FRONT_START_MAX_M = _FRONT_START_MAX_M

    def __init__(self):
        self.last_debug = {"path": "init", "clamp_applied": False, "clamp_kind": "none"}

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(float(lo), min(float(hi), float(value)))

    def _dynamic_front_thresholds(self, v_cmd: float) -> tuple[float, float]:
        return dynamic_front_clearance_thresholds(v_cmd)

    def _apply_linear_brake(self, pwm_l: float, pwm_r: float, min_dist: float, start_m: float, stop_m: float) -> tuple[float, float, float]:
        if min_dist >= start_m:
            return float(pwm_l), float(pwm_r), 1.0
        if min_dist <= stop_m:
            return 0.0, 0.0, 0.0
        span = max(1e-6, start_m - stop_m)
        ratio = self._clamp((min_dist - stop_m) / span, 0.0, 1.0)
        return float(pwm_l) * ratio, float(pwm_r) * ratio, float(ratio)

    def _set_debug(
        self,
        *,
        path: str,
        v_cmd: float,
        pwm_l_in: float,
        pwm_r_in: float,
        pwm_l_out: float,
        pwm_r_out: float,
        clamp_kind: str = "none",
        brake_ratio: float = 1.0,
        **extra,
    ) -> None:
        clamp_applied = str(path) != "pass_through"
        self.last_debug = {
            "path": str(path),
            "clamp_applied": bool(clamp_applied),
            "clamp_kind": str(clamp_kind),
            "v_cmd": float(v_cmd),
            "brake_ratio": float(brake_ratio),
            "pwm_in": {"left": float(pwm_l_in), "right": float(pwm_r_in)},
            "pwm_out": {"left": float(pwm_l_out), "right": float(pwm_r_out)},
            "pwm_delta": {
                "left": float(pwm_l_out) - float(pwm_l_in),
                "right": float(pwm_r_out) - float(pwm_r_in),
            },
        }
        self.last_debug.update(dict(extra or {}))

    def filter_pwm(
        self,
        pwm_l: float,
        pwm_r: float,
        safety_state: dict,
        *,
        v_cmd: float = 0.0,
        lidar_summary: dict | None = None,
    ) -> tuple:
        """
        Filter PWM outputs based on safety state.

        Args:
            pwm_l: Left PWM command (-1.0 to 1.0)
            pwm_r: Right PWM command (-1.0 to 1.0)
            safety_state: Dict with keys:
                - allow: bool - whether motion is allowed
                - reason: str - reason for blocking (if not allowed)
            v_cmd: Aktuális v_cmd (m/s) — negatív = hátra.
            lidar_summary: LIDAR összesítés (min_back, blocked_back, stb.)

        Returns:
            Tuple (filtered_pwm_l, filtered_pwm_r)
        """
        pwm_l_in = float(pwm_l)
        pwm_r_in = float(pwm_r)
        self._set_debug(
            path="pass_through",
            v_cmd=float(v_cmd),
            pwm_l_in=pwm_l_in,
            pwm_r_in=pwm_r_in,
            pwm_l_out=pwm_l_in,
            pwm_r_out=pwm_r_in,
        )

        if not safety_state.get("allow", True):
            self._set_debug(
                path="safety_block",
                v_cmd=float(v_cmd),
                pwm_l_in=pwm_l_in,
                pwm_r_in=pwm_r_in,
                pwm_l_out=0.0,
                pwm_r_out=0.0,
                clamp_kind="safety_block",
                brake_ratio=0.0,
                reason=str((safety_state or {}).get("reason", "") or "safety_not_allow"),
            )
            return 0.0, 0.0

        lidar = dict(lidar_summary or {})

        # Előremenetnél dinamikus front-fékezés.
        if v_cmd > 0.01 and lidar:
            min_front = float(lidar.get("min_dist", 999.0) or 999.0)
            blocked_front = bool(lidar.get("blocked_front", False))
            front_start_m, front_stop_m = self._dynamic_front_thresholds(v_cmd)
            if blocked_front:
                self._set_debug(
                    path="front_blocked_hard_stop",
                    v_cmd=float(v_cmd),
                    pwm_l_in=pwm_l_in,
                    pwm_r_in=pwm_r_in,
                    pwm_l_out=0.0,
                    pwm_r_out=0.0,
                    clamp_kind="hard_zero",
                    brake_ratio=0.0,
                    blocked_front=True,
                    min_front_m=float(min_front),
                    front_start_m=float(front_start_m),
                    front_stop_m=float(front_stop_m),
                )
                return 0.0, 0.0
            pwm_l, pwm_r, ratio = self._apply_linear_brake(pwm_l, pwm_r, min_front, front_start_m, front_stop_m)
            if ratio < 1.0:
                self._set_debug(
                    path="front_dynamic_brake",
                    v_cmd=float(v_cmd),
                    pwm_l_in=pwm_l_in,
                    pwm_r_in=pwm_r_in,
                    pwm_l_out=float(pwm_l),
                    pwm_r_out=float(pwm_r),
                    clamp_kind="hard_zero" if float(ratio) <= 0.0 else "soft_brake",
                    brake_ratio=float(ratio),
                    blocked_front=False,
                    min_front_m=float(min_front),
                    front_start_m=float(front_start_m),
                    front_stop_m=float(front_stop_m),
                )

        # Hátramenetnél proaktív fékezés: fokozatos PWM csillapítás
        # a hátsó akadály közelségétől függően.
        if v_cmd < -0.01 and lidar:
            min_back = float(lidar.get("min_back", 999.0) or 999.0)
            if min_back < self.REAR_BRAKE_START_M:
                span = self.REAR_BRAKE_START_M - self.REAR_BRAKE_STOP_M
                if span > 0.001:
                    ratio = max(0.0, (min_back - self.REAR_BRAKE_STOP_M) / span)
                else:
                    ratio = 0.0
                pwm_l *= ratio
                pwm_r *= ratio
                self._set_debug(
                    path="rear_dynamic_brake",
                    v_cmd=float(v_cmd),
                    pwm_l_in=pwm_l_in,
                    pwm_r_in=pwm_r_in,
                    pwm_l_out=float(pwm_l),
                    pwm_r_out=float(pwm_r),
                    clamp_kind="hard_zero" if float(ratio) <= 0.0 else "rear_brake",
                    brake_ratio=float(ratio),
                    min_back_m=float(min_back),
                    rear_start_m=float(self.REAR_BRAKE_START_M),
                    rear_stop_m=float(self.REAR_BRAKE_STOP_M),
                )

        if self.last_debug.get("path") == "pass_through":
            self._set_debug(
                path="pass_through",
                v_cmd=float(v_cmd),
                pwm_l_in=pwm_l_in,
                pwm_r_in=pwm_r_in,
                pwm_l_out=float(pwm_l),
                pwm_r_out=float(pwm_r),
            )

        return pwm_l, pwm_r
