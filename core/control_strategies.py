#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Wheel-only canonical PI primitives owned by MotionExecutor."""

from __future__ import annotations

import json
import math
import os
from typing import Dict, Optional, Tuple

from middleware.ffp import PIDConfig


CANONICAL_CONTROL_MODE = "UNIFIED"
ALLOWED_MODES = (CANONICAL_CONTROL_MODE,)
DEFAULT_MODE = CANONICAL_CONTROL_MODE
WHEEL_PI_ZERO_CROSS_RESET_DEADBAND_MPS = 0.006
WHEEL_PI_FEEDBACK_FILTER_ALPHA = 1.0
WHEEL_STARTUP_RELEASE_SPEED_MPS = 0.020
WHEEL_STARTUP_RELEASE_DWELL_S = 0.10


def wheel_feedback_timing_error(
    *,
    timing_valid: bool,
    stale: bool,
    timing_reason: str,
) -> str:
    """Return the L9 fail-closed reason for a physical wheel sample."""

    encoder_timing_gap = "GAP" in str(timing_reason or "").strip().upper()
    if bool(stale):
        return "WHEEL_FEEDBACK_STALE"
    if not bool(timing_valid):
        return "ENCODER_TIMING_GAP" if encoder_timing_gap else "WHEEL_FEEDBACK_TIMING_INVALID"
    return ""


def normalize_control_mode(mode: Optional[str]) -> str:
    normalized = str(mode or "").strip().upper()
    if normalized != CANONICAL_CONTROL_MODE:
        raise ValueError(f"unsupported_control_mode:{normalized or 'MISSING'}")
    return CANONICAL_CONTROL_MODE


def load_control_mode(path: str) -> str:
    if not os.path.isfile(path):
        raise FileNotFoundError(f"control_mode_config_missing:{path}")
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle) or {}
    return normalize_control_mode(data.get("control_mode"))


def save_control_mode(path: str, mode: str) -> bool:
    normalized = normalize_control_mode(mode)
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump({"control_mode": normalized}, handle, ensure_ascii=False, indent=2)
        return True
    except Exception:
        return False


class SimplePI:
    def __init__(
        self,
        kp: float,
        ki: float,
        integrator_limit: float,
        zero_cross_reset_deadband: float = 0.0,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.integrator_limit = max(0.0, float(integrator_limit))
        self.zero_cross_reset_deadband = max(0.0, float(zero_cross_reset_deadband))
        self.reset()

    def reset(self) -> None:
        self._i = 0.0
        self._p = 0.0

    def update(self, error: float, dt_s: float) -> Tuple[float, float]:
        if dt_s <= 0.0:
            self._p = 0.0
            return 0.0, 0.0
        self._p = self.kp * float(error)
        if (
            abs(float(error)) > max(1e-12, self.zero_cross_reset_deadband)
            and self._i * float(error) < 0.0
        ):
            self._i = 0.0
        self._i += float(error) * float(dt_s)
        if self.integrator_limit > 0.0:
            self._i = max(-self.integrator_limit, min(self.integrator_limit, self._i))
        return float(self._p), float(self.ki * self._i)

    @property
    def integrator_state(self) -> float:
        return float(self._i)

    @property
    def integrator_clamped(self) -> bool:
        return bool(
            self.integrator_limit > 0.0
            and abs(self._i) >= self.integrator_limit - 1e-9
        )


class WheelSpeedPILoop:
    """Exactly one PI update per wheel and control tick."""

    name = "UNIFIED_WHEEL_LOOP"

    def __init__(
        self,
        pid: PIDConfig,
        max_pwm: float,
        dead_zone: float = 0.0,
        overspeed_holdoff_enabled: bool = False,
    ):
        self.max_pwm = max(0.0, min(1.0, float(max_pwm)))
        self.dead_zone = max(0.0, min(1.0, float(dead_zone)))
        self.encoder_feedback_trust_min = max(
            0.0,
            min(1.0, float(pid.wheel_feedback_trust_min)),
        )
        self.overspeed_holdoff_enabled = bool(overspeed_holdoff_enabled)
        self.zero_cross_reset_deadband_mps = WHEEL_PI_ZERO_CROSS_RESET_DEADBAND_MPS
        self.feedback_filter_alpha = WHEEL_PI_FEEDBACK_FILTER_ALPHA
        self.wheel_pi_l = SimplePI(
            pid.kp,
            pid.ki,
            pid.integrator_limit,
            zero_cross_reset_deadband=self.zero_cross_reset_deadband_mps,
        )
        self.wheel_pi_r = SimplePI(
            pid.kp,
            pid.ki,
            pid.integrator_limit,
            zero_cross_reset_deadband=self.zero_cross_reset_deadband_mps,
        )
        self._feedback_filter_mps = {"left": None, "right": None}
        self._feedback_filter_sign = {"left": 0, "right": 0}

    def reset(self) -> None:
        self.wheel_pi_l.reset()
        self.wheel_pi_r.reset()
        self._feedback_filter_mps = {"left": None, "right": None}
        self._feedback_filter_sign = {"left": 0, "right": 0}

    def _reset_side(self, side: str) -> None:
        if side == "left":
            self.wheel_pi_l.reset()
        else:
            self.wheel_pi_r.reset()
        self._feedback_filter_mps[side] = None
        self._feedback_filter_sign[side] = 0

    def _filtered_feedback(self, side: str, reference: float, measured: float) -> float:
        sign = 1 if reference > 0.0 else -1 if reference < 0.0 else 0
        previous = self._feedback_filter_mps[side]
        if sign == 0:
            self._reset_side(side)
            return float(measured)
        if previous is None or self._feedback_filter_sign[side] != sign:
            result = float(measured)
        else:
            alpha = self.feedback_filter_alpha
            result = alpha * float(measured) + (1.0 - alpha) * float(previous)
        self._feedback_filter_mps[side] = float(result)
        self._feedback_filter_sign[side] = sign
        return float(result)

    def _shape(
        self,
        *,
        raw_pwm: float,
        reference: float,
        measured: float,
        pi: SimplePI,
        maintenance_floor_pwm: float,
        residual_pwm: float,
    ) -> tuple[float, str]:
        if abs(reference) <= 1e-9:
            pi.reset()
            return 0.0, "zero_reference"
        opposite = reference * raw_pwm < 0.0
        if opposite:
            pi.reset()
            return 0.0, "direction_clamp"
        if self.overspeed_holdoff_enabled:
            margin = max(0.006, abs(reference) * 0.20)
            overspeed = (
                reference > 0.0 and measured >= reference + margin
            ) or (
                reference < 0.0 and measured <= reference - margin
            )
            if overspeed:
                pi.reset()
                return 0.0, "overspeed_holdoff"
        shaped = float(raw_pwm)
        if abs(shaped) > 1e-9 and self.dead_zone > 0.0:
            shaped = math.copysign(
                self.dead_zone + (1.0 - self.dead_zone) * abs(shaped),
                shaped,
            )
        floor = min(self.max_pwm, max(0.0, abs(float(maintenance_floor_pwm))))
        downward = residual_pwm * reference < -1e-12
        if floor > 0.0 and abs(shaped) < floor and not downward:
            return math.copysign(floor, reference), "maintenance_floor"
        return float(shaped), "pi_downward_below_maintenance" if downward and abs(shaped) < floor else "pi"

    @staticmethod
    def _maintenance_floor_diagnostics(
        *,
        maintenance_floor_pwm_l: float,
        maintenance_floor_pwm_r: float,
        left_reason: str,
        right_reason: str,
    ) -> Dict[str, object]:
        return {
            "maintenance_floor_pwm_l": float(maintenance_floor_pwm_l),
            "maintenance_floor_pwm_r": float(maintenance_floor_pwm_r),
            "wheel_loop_left_maintenance_floor_applied": (
                str(left_reason) == "maintenance_floor"
            ),
            "wheel_loop_right_maintenance_floor_applied": (
                str(right_reason) == "maintenance_floor"
            ),
        }

    def compute(
        self,
        *,
        left_reference_mps: float,
        right_reference_mps: float,
        left_measured_mps: float,
        right_measured_mps: float,
        dt_s: float,
        feedforward_pwm_l: float,
        feedforward_pwm_r: float,
        maintenance_floor_pwm_l: float,
        maintenance_floor_pwm_r: float,
    ) -> tuple[float, float, Dict]:
        left_active = abs(float(left_reference_mps)) > 1e-9
        right_active = abs(float(right_reference_mps)) > 1e-9
        if not left_active:
            self._reset_side("left")
        if not right_active:
            self._reset_side("right")
        left_control = self._filtered_feedback(
            "left",
            float(left_reference_mps),
            float(left_measured_mps),
        ) if left_active else float(left_measured_mps)
        right_control = self._filtered_feedback(
            "right",
            float(right_reference_mps),
            float(right_measured_mps),
        ) if right_active else float(right_measured_mps)
        left_error = float(left_reference_mps) - left_control
        right_error = float(right_reference_mps) - right_control
        left_p, left_i = self.wheel_pi_l.update(left_error, dt_s) if left_active else (0.0, 0.0)
        right_p, right_i = self.wheel_pi_r.update(right_error, dt_s) if right_active else (0.0, 0.0)
        left_raw = max(
            -self.max_pwm,
            min(self.max_pwm, float(feedforward_pwm_l) + left_p + left_i),
        )
        right_raw = max(
            -self.max_pwm,
            min(self.max_pwm, float(feedforward_pwm_r) + right_p + right_i),
        )
        maintenance_floor_l = float(maintenance_floor_pwm_l)
        maintenance_floor_r = float(maintenance_floor_pwm_r)
        left_pwm, left_reason = self._shape(
            raw_pwm=left_raw,
            reference=float(left_reference_mps),
            measured=float(left_measured_mps),
            pi=self.wheel_pi_l,
            maintenance_floor_pwm=maintenance_floor_l,
            residual_pwm=left_p + left_i,
        ) if left_active else (0.0, "inactive")
        right_pwm, right_reason = self._shape(
            raw_pwm=right_raw,
            reference=float(right_reference_mps),
            measured=float(right_measured_mps),
            pi=self.wheel_pi_r,
            maintenance_floor_pwm=maintenance_floor_r,
            residual_pwm=right_p + right_i,
        ) if right_active else (0.0, "inactive")
        floor_diagnostics = self._maintenance_floor_diagnostics(
            maintenance_floor_pwm_l=maintenance_floor_l,
            maintenance_floor_pwm_r=maintenance_floor_r,
            left_reason=left_reason,
            right_reason=right_reason,
        )
        diagnostics = {
            "wheel_pi_enabled": True,
            "wheel_pi_effective_kp": float(self.wheel_pi_l.kp),
            "wheel_pi_effective_ki": float(self.wheel_pi_l.ki),
            "left_reference_mps": float(left_reference_mps),
            "right_reference_mps": float(right_reference_mps),
            "left_measured_mps": float(left_measured_mps),
            "right_measured_mps": float(right_measured_mps),
            "left_control_error_mps": float(left_error),
            "right_control_error_mps": float(right_error),
            "left_p_pwm": float(left_p),
            "left_i_pwm": float(left_i),
            "right_p_pwm": float(right_p),
            "right_i_pwm": float(right_i),
            "left_output_reason": left_reason,
            "right_output_reason": right_reason,
            "integrator_clamped": bool(
                self.wheel_pi_l.integrator_clamped
                or self.wheel_pi_r.integrator_clamped
            ),
            **floor_diagnostics,
        }
        return float(left_pwm), float(right_pwm), diagnostics
