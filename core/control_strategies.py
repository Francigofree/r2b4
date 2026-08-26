#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import time
from typing import Dict, Tuple, Optional

from middleware.ffp import AlbaDriveController, PIDConfig

CANONICAL_CONTROL_MODE = "UNIFIED"
ALLOWED_MODES = (CANONICAL_CONTROL_MODE,)
DEFAULT_MODE = CANONICAL_CONTROL_MODE
WHEEL_PI_ZERO_CROSS_RESET_DEADBAND_MPS = 0.006
WHEEL_PI_FEEDBACK_FILTER_ALPHA = 1.0
WHEEL_STARTUP_RELEASE_SPEED_MPS = 0.020
WHEEL_STARTUP_RELEASE_DWELL_S = 0.10


def normalize_control_mode(mode: Optional[str]) -> str:
    normalized = str(mode or "").strip().upper()
    if normalized != CANONICAL_CONTROL_MODE:
        raise ValueError(f"unsupported_control_mode:{normalized or 'MISSING'}")
    return CANONICAL_CONTROL_MODE


def load_control_mode(path: str) -> str:
    if not os.path.isfile(path):
        raise FileNotFoundError(f"control_mode_config_missing:{path}")
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f) or {}
    return normalize_control_mode(data.get("control_mode"))


def save_control_mode(path: str, mode: str) -> bool:
    normalized = normalize_control_mode(mode)
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"control_mode": normalized}, f, ensure_ascii=False, indent=2)
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
        self.integrator_limit = float(integrator_limit)
        self.zero_cross_reset_deadband = max(0.0, float(zero_cross_reset_deadband))
        self.reset()

    def reset(self):
        self._i = 0.0
        self._p = 0.0

    def update(self, err: float, dt: float) -> Tuple[float, float]:
        if dt <= 0.0:
            self._p = 0.0
            return 0.0, 0.0
        self._p = self.kp * err
        # Simple zero-crossing anti-windup: an onset accumulated integral must
        # not keep accelerating the wheel after the measured error reverses.
        # Zero/deadband samples retain the steady load compensation.
        if abs(err) > max(1e-12, self.zero_cross_reset_deadband) and (self._i * err) < 0.0:
            self._i = 0.0
        self._i += err * dt
        if self.integrator_limit > 0:
            self._i = max(-self.integrator_limit, min(self.integrator_limit, self._i))
        return self._p, self.ki * self._i

    @property
    def integrator_state(self) -> float:
        return float(self._i)

    @property
    def integrator_clamped(self) -> bool:
        if self.integrator_limit <= 0:
            return False
        return abs(self._i) >= (self.integrator_limit - 1e-9)


class ControlStrategy:
    name = "BASE"

    def compute(self, state: Dict, setpoint: Dict) -> Tuple[float, float, Dict]:
        raise NotImplementedError

    def reset(self):
        pass


class WheelSpeedPILoop:
    name = "UNIFIED_WHEEL_LOOP"

    def __init__(
        self,
        pid: PIDConfig,
        max_pwm: float,
        dead_zone: float,
        overspeed_holdoff_enabled: bool = True,
    ):
        self.max_pwm = float(max_pwm)
        self.dead_zone = float(dead_zone)
        self.k_ff = float(getattr(pid, "k_ff", 0.0))
        self.overspeed_holdoff_enabled = bool(overspeed_holdoff_enabled)
        self.encoder_feedback_trust_min = max(
            0.0,
            min(1.0, float(getattr(pid, "wheel_feedback_trust_min", 0.55))),
        )
        self.zero_cross_reset_deadband_mps = WHEEL_PI_ZERO_CROSS_RESET_DEADBAND_MPS
        self.feedback_filter_alpha = max(
            0.0,
            min(1.0, WHEEL_PI_FEEDBACK_FILTER_ALPHA),
        )
        self._feedback_filter_mps = {"left": None, "right": None}
        self._feedback_filter_sign = {"left": 0, "right": 0}
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
        self._last_diag = {}

    def reset(self):
        self.wheel_pi_l.reset()
        self.wheel_pi_r.reset()
        self._reset_feedback_filter("left")
        self._reset_feedback_filter("right")

    def _reset_feedback_filter(self, side: str) -> None:
        self._feedback_filter_mps[side] = None
        self._feedback_filter_sign[side] = 0

    def _control_feedback_mps(self, side: str, ref_mps: float, meas_mps: float) -> float:
        ref = float(ref_mps)
        meas = float(meas_mps)
        sign = 1 if ref > 0.0 else -1 if ref < 0.0 else 0
        if sign == 0:
            self._reset_feedback_filter(side)
            return meas

        previous = self._feedback_filter_mps.get(side)
        if (
            previous is None
            or self._feedback_filter_sign.get(side, 0) != sign
            or not math.isfinite(float(previous))
        ):
            filtered = meas
        else:
            alpha = self.feedback_filter_alpha
            filtered = (alpha * meas) + ((1.0 - alpha) * float(previous))

        self._feedback_filter_mps[side] = float(filtered)
        self._feedback_filter_sign[side] = sign
        return float(filtered)

    def _apply_dead_zone(self, pwm: float) -> float:
        if abs(pwm) < 1e-6:
            return 0.0
        dz = max(0.0, min(1.0, self.dead_zone))
        return math.copysign(dz + (1.0 - dz) * abs(pwm), pwm)

    def _shape_wheel_pwm(
        self,
        *,
        pwm_raw: float,
        ref_mps: float,
        meas_mps: float,
        pi: SimplePI,
        maintenance_floor_pwm: float = 0.0,
        pi_residual_pwm: float = 0.0,
        overspeed_holdoff_enabled: Optional[bool] = None,
    ) -> Tuple[float, str]:
        ref = float(ref_mps)
        meas = float(meas_mps)
        raw = float(pwm_raw)
        holdoff_enabled = (
            bool(self.overspeed_holdoff_enabled)
            if overspeed_holdoff_enabled is None
            else bool(overspeed_holdoff_enabled)
        )
        if abs(ref) <= 1e-6:
            pi.reset()
            return 0.0, "zero_reference"

        overspeed_margin = max(0.006, abs(ref) * 0.20)
        forward_overspeed = ref > 0.0 and meas >= ref + overspeed_margin
        reverse_overspeed = ref < 0.0 and meas <= ref - overspeed_margin
        same_direction_pwm = (ref > 0.0 and raw > 0.0) or (ref < 0.0 and raw < 0.0)
        opposite_direction_pwm = (ref > 0.0 and raw < 0.0) or (ref < 0.0 and raw > 0.0)

        if holdoff_enabled and (forward_overspeed or reverse_overspeed) and same_direction_pwm:
            pi.reset()
            return 0.0, "overspeed_holdoff"
        if opposite_direction_pwm:
            if holdoff_enabled:
                pi.reset()
                return 0.0, "opposite_pwm_holdoff"
            return 0.0, "direction_clamp"

        shaped = self._apply_dead_zone(raw)
        maintenance_floor = max(
            0.0,
            min(float(self.max_pwm), abs(float(maintenance_floor_pwm))),
        )
        downward_pi_correction = bool(float(pi_residual_pwm) * ref < -1e-12)
        if (
            maintenance_floor > 0.0
            and abs(shaped) + 1e-9 < maintenance_floor
            and not downward_pi_correction
        ):
            return math.copysign(maintenance_floor, ref), "maintenance_floor"
        if downward_pi_correction and abs(shaped) + 1e-9 < maintenance_floor:
            return shaped, "pi_downward_below_maintenance"
        return shaped, "deadzone"

    def _select_encoder_track_feedback(
        self,
        *,
        state: Dict,
        v_cmd: float,
    ) -> Tuple[bool, float, float, Dict]:
        diag = {
            "available": False,
            "source": "encoder_unavailable",
            "trust": None,
            "stale": False,
            "timing_valid": True,
            "timing_error": "",
            "timing_gap_s": 0.0,
            "left_mps": 0.0,
            "right_mps": 0.0,
            "distance_window_s": 0.0,
        }
        enc_trust = float(state.get("encoder_combined_trust", 0.0) or 0.0)
        enc_stale = bool(state.get("encoder_snapshot_stale", False))
        enc_timing_valid = bool(state.get("encoder_timing_valid", True))
        enc_timing_error = str(state.get("encoder_timing_error", "") or "")
        try:
            enc_timing_gap_s = float(state.get("encoder_timing_gap_s", 0.0) or 0.0)
            if not math.isfinite(enc_timing_gap_s):
                enc_timing_gap_s = 0.0
        except (TypeError, ValueError):
            enc_timing_gap_s = 0.0
        diag["trust"] = round(enc_trust, 6)
        diag["stale"] = bool(enc_stale)
        diag["timing_valid"] = bool(enc_timing_valid)
        diag["timing_error"] = str(enc_timing_error)
        diag["timing_gap_s"] = float(enc_timing_gap_s)
        if not enc_timing_valid:
            diag["source"] = "encoder_timing_gap"
            return False, 0.0, 0.0, diag
        if enc_stale or enc_trust < self.encoder_feedback_trust_min:
            return False, 0.0, 0.0, diag

        v_l_enc = state.get("v_l_encoder", None)
        v_r_enc = state.get("v_r_encoder", None)
        aggregation_window_s = state.get("encoder_aggregation_window_s", None)

        canonical_available = (
            v_l_enc is not None
            and v_r_enc is not None
            and math.isfinite(float(v_l_enc))
            and math.isfinite(float(v_r_enc))
        )
        try:
            aggregation_window_s_f = float(aggregation_window_s)
            if not math.isfinite(aggregation_window_s_f):
                aggregation_window_s_f = 0.0
        except (TypeError, ValueError):
            aggregation_window_s_f = 0.0

        # The reliability layer is the only owner of wheel-speed aggregation.
        # Raw estimator velocity and distance/dt channels remain telemetry, but
        # must never keep the PI loop alive when canonical feedback is missing.
        if not canonical_available:
            return False, 0.0, 0.0, diag
        v_l_meas = float(v_l_enc)
        v_r_meas = float(v_r_enc)

        diag.update(
            {
                "available": True,
                "source": "encoder_canonical",
                "left_mps": float(v_l_meas),
                "right_mps": float(v_r_meas),
                "distance_window_s": float(aggregation_window_s_f),
            }
        )
        return True, float(v_l_meas), float(v_r_meas), diag

    def compute(
        self,
        *,
        state: Dict,
        v_cmd: float,
        omega_cmd: float,
        omega_cmd_request: float,
        v_l_ref: float,
        v_r_ref: float,
        dt: float,
        feedforward_pwm_l: Optional[float] = None,
        feedforward_pwm_r: Optional[float] = None,
        maintenance_floor_pwm_l: float = 0.0,
        maintenance_floor_pwm_r: float = 0.0,
    ) -> Tuple[bool, float, float, Dict]:
        feedback_ok, v_l_meas, v_r_meas, feedback_diag = self._select_encoder_track_feedback(
            state=state,
            v_cmd=v_cmd,
        )
        if not feedback_ok:
            self.wheel_pi_l.reset()
            self.wheel_pi_r.reset()
            self._reset_feedback_filter("left")
            self._reset_feedback_filter("right")
            return False, 0.0, 0.0, {
                "wheel_loop_enabled": False,
                "wheel_loop_feedback_source": feedback_diag.get("source"),
                "wheel_loop_feedback_trust": feedback_diag.get("trust"),
                "wheel_loop_feedback_timing_valid": feedback_diag.get("timing_valid"),
                "wheel_loop_feedback_timing_error": feedback_diag.get("timing_error"),
                "wheel_loop_feedback_timing_gap_s": feedback_diag.get("timing_gap_s"),
            }

        left_active = abs(float(v_l_ref)) > 1e-6
        right_active = abs(float(v_r_ref)) > 1e-6
        if not left_active:
            self.wheel_pi_l.reset()
            self._reset_feedback_filter("left")
        if not right_active:
            self.wheel_pi_r.reset()
            self._reset_feedback_filter("right")

        v_l_control_meas = (
            self._control_feedback_mps("left", v_l_ref, v_l_meas)
            if left_active
            else float(v_l_meas)
        )
        v_r_control_meas = (
            self._control_feedback_mps("right", v_r_ref, v_r_meas)
            if right_active
            else float(v_r_meas)
        )
        err_l_actual = float(v_l_ref) - float(v_l_meas)
        err_r_actual = float(v_r_ref) - float(v_r_meas)
        err_l_control = float(v_l_ref) - float(v_l_control_meas)
        err_r_control = float(v_r_ref) - float(v_r_control_meas)
        overspeed_holdoff_enabled = bool(
            state.get("wheel_loop_overspeed_holdoff_enabled", self.overspeed_holdoff_enabled)
        )
        track_reference_delta_mps = abs(float(v_r_ref) - float(v_l_ref))
        lp, li = self.wheel_pi_l.update(err_l_control, dt) if left_active else (0.0, 0.0)
        rp, ri = self.wheel_pi_r.update(err_r_control, dt) if right_active else (0.0, 0.0)
        base_l = (
            self.k_ff * float(v_l_ref)
            if feedforward_pwm_l is None
            else float(feedforward_pwm_l)
        )
        base_r = (
            self.k_ff * float(v_r_ref)
            if feedforward_pwm_r is None
            else float(feedforward_pwm_r)
        )
        pwm_raw_l = base_l + lp + li
        pwm_raw_r = base_r + rp + ri

        pwm_before_clamp_l = max(-self.max_pwm, min(self.max_pwm, pwm_raw_l))
        pwm_before_clamp_r = max(-self.max_pwm, min(self.max_pwm, pwm_raw_r))
        if left_active:
            pwm_l, left_output_reason = self._shape_wheel_pwm(
                pwm_raw=pwm_before_clamp_l,
                ref_mps=v_l_ref,
                meas_mps=v_l_meas,
                pi=self.wheel_pi_l,
                maintenance_floor_pwm=maintenance_floor_pwm_l,
                pi_residual_pwm=lp + li,
                overspeed_holdoff_enabled=overspeed_holdoff_enabled,
            )
        else:
            pwm_l = 0.0
            left_output_reason = "inactive"
        if right_active:
            pwm_r, right_output_reason = self._shape_wheel_pwm(
                pwm_raw=pwm_before_clamp_r,
                ref_mps=v_r_ref,
                meas_mps=v_r_meas,
                pi=self.wheel_pi_r,
                maintenance_floor_pwm=maintenance_floor_pwm_r,
                pi_residual_pwm=rp + ri,
                overspeed_holdoff_enabled=overspeed_holdoff_enabled,
            )
        else:
            pwm_r = 0.0
            right_output_reason = "inactive"
        if left_output_reason in {"overspeed_holdoff", "opposite_pwm_holdoff", "direction_clamp"}:
            self._reset_feedback_filter("left")
        if right_output_reason in {"overspeed_holdoff", "opposite_pwm_holdoff", "direction_clamp"}:
            self._reset_feedback_filter("right")

        effective_kp = float(self.wheel_pi_l.kp)

        diag = {
            "control_mode": self.name,
            "active": True,
            "direct_pwm": False,
            "v_cmd": v_cmd,
            "omega_cmd": omega_cmd,
            "omega_cmd_request": omega_cmd_request,
            "v_l_ref": v_l_ref,
            "v_r_ref": v_r_ref,
            "v_l": v_l_meas,
            "v_r": v_r_meas,
            "v_avg": 0.5 * (v_l_meas + v_r_meas),
            "v_err": 0.5 * (err_l_actual + err_r_actual),
            "speed_p": 0.5 * (lp + rp),
            "yaw_p": 0.5 * (rp - lp),
            "yaw_i": 0.5 * (ri - li),
            "yaw_corr": 0.5 * ((rp + ri) - (lp + li)),
            "integrator_state": max(abs(self.wheel_pi_l.integrator_state), abs(self.wheel_pi_r.integrator_state)),
            "integrator_limit": self.wheel_pi_l.integrator_limit,
            "integrator_clamped": self.wheel_pi_l.integrator_clamped or self.wheel_pi_r.integrator_clamped,
            "output_saturated": abs(pwm_raw_l) > self.max_pwm or abs(pwm_raw_r) > self.max_pwm,
            "base_l": base_l,
            "base_r": base_r,
            "pwm_raw_l": pwm_raw_l,
            "pwm_raw_r": pwm_raw_r,
            "pwm_before_clamp_l": pwm_before_clamp_l,
            "pwm_before_clamp_r": pwm_before_clamp_r,
            "pwm_after_clamp_l": pwm_l,
            "pwm_after_clamp_r": pwm_r,
            "pwm_executor_l": pwm_l,
            "pwm_executor_r": pwm_r,
            "wheel_pi_enabled": True,
            "Kp": self.wheel_pi_l.kp,
            "Ki": self.wheel_pi_l.ki,
            "dead_zone": self.dead_zone,
            "wheel_loop_enabled": True,
            "wheel_loop_overspeed_holdoff_enabled": bool(overspeed_holdoff_enabled),
            "wheel_loop_track_reference_delta_mps": float(track_reference_delta_mps),
            "wheel_loop_effective_kp": float(effective_kp),
            "wheel_loop_zero_cross_reset_deadband_mps": float(
                self.zero_cross_reset_deadband_mps
            ),
            "wheel_loop_feedback_filter_alpha": float(self.feedback_filter_alpha),
            "wheel_loop_feedback_source": feedback_diag.get("source"),
            "wheel_loop_feedback_trust": feedback_diag.get("trust"),
            "wheel_loop_v_l_meas": v_l_meas,
            "wheel_loop_v_r_meas": v_r_meas,
            "wheel_loop_v_l_control_meas": v_l_control_meas,
            "wheel_loop_v_r_control_meas": v_r_control_meas,
            "wheel_loop_err_l": err_l_actual,
            "wheel_loop_err_r": err_r_actual,
            "wheel_loop_control_err_l": err_l_control,
            "wheel_loop_control_err_r": err_r_control,
            "wheel_loop_left_p": lp,
            "wheel_loop_left_i": li,
            "wheel_loop_left_pi_residual_pwm": lp + li,
            "wheel_loop_left_feedforward_pwm": base_l,
            "wheel_loop_left_maintenance_floor_pwm": abs(float(maintenance_floor_pwm_l)),
            "wheel_loop_left_maintenance_floor_applied": left_output_reason == "maintenance_floor",
            "wheel_loop_left_pi_downward_below_maintenance": left_output_reason
            == "pi_downward_below_maintenance",
            "wheel_loop_left_output_reason": left_output_reason,
            "wheel_loop_right_p": rp,
            "wheel_loop_right_i": ri,
            "wheel_loop_right_pi_residual_pwm": rp + ri,
            "wheel_loop_right_feedforward_pwm": base_r,
            "wheel_loop_right_maintenance_floor_pwm": abs(float(maintenance_floor_pwm_r)),
            "wheel_loop_right_maintenance_floor_applied": right_output_reason == "maintenance_floor",
            "wheel_loop_right_pi_downward_below_maintenance": right_output_reason
            == "pi_downward_below_maintenance",
            "wheel_loop_right_output_reason": right_output_reason,
            "monitor": {
                "mode": self.name,
                "v_cmd": v_cmd,
                "omega_cmd": omega_cmd,
                "omega_cmd_request": omega_cmd_request,
                "feedforward": 0.5 * (base_l + base_r),
                "v_meas": 0.5 * (v_l_meas + v_r_meas),
                "speed_pi_output": 0.5 * ((lp + li) + (rp + ri)),
                "yaw_pi_output": 0.5 * ((rp + ri) - (lp + li)),
                "yaw_open_loop_pwm": 0.0,
                "yaw_open_loop_raw_pwm": 0.0,
                "wheel_pi_enabled": True,
                "pi_correction_left_pwm": lp + li,
                "pi_correction_right_pwm": rp + ri,
                "wheel_loop_enabled": True,
                "wheel_loop_overspeed_holdoff_enabled": bool(overspeed_holdoff_enabled),
                "wheel_loop_track_reference_delta_mps": float(
                    track_reference_delta_mps
                ),
                "wheel_loop_effective_kp": float(effective_kp),
                "wheel_loop_zero_cross_reset_deadband_mps": float(
                    self.zero_cross_reset_deadband_mps
                ),
                "wheel_loop_feedback_filter_alpha": float(self.feedback_filter_alpha),
                "wheel_loop_feedback_source": feedback_diag.get("source"),
                "wheel_loop_left_ref_mps": v_l_ref,
                "wheel_loop_right_ref_mps": v_r_ref,
                "wheel_loop_left_meas_mps": v_l_meas,
                "wheel_loop_right_meas_mps": v_r_meas,
                "wheel_loop_left_control_meas_mps": v_l_control_meas,
                "wheel_loop_right_control_meas_mps": v_r_control_meas,
                "wheel_loop_left_error_mps": err_l_actual,
                "wheel_loop_right_error_mps": err_r_actual,
                "wheel_loop_left_control_error_mps": err_l_control,
                "wheel_loop_right_control_error_mps": err_r_control,
                "wheel_loop_left_maintenance_floor_pwm": abs(float(maintenance_floor_pwm_l)),
                "wheel_loop_right_maintenance_floor_pwm": abs(float(maintenance_floor_pwm_r)),
                "wheel_loop_left_maintenance_floor_applied": left_output_reason == "maintenance_floor",
                "wheel_loop_right_maintenance_floor_applied": right_output_reason == "maintenance_floor",
                "wheel_loop_left_pi_downward_below_maintenance": left_output_reason
                == "pi_downward_below_maintenance",
                "wheel_loop_right_pi_downward_below_maintenance": right_output_reason
                == "pi_downward_below_maintenance",
                "wheel_loop_left_output_reason": left_output_reason,
                "wheel_loop_right_output_reason": right_output_reason,
                "pwm_raw_l": pwm_raw_l,
                "pwm_raw_r": pwm_raw_r,
                "pwm_executor_l": pwm_l,
                "pwm_executor_r": pwm_r,
                "dead_zone": self.dead_zone,
            },
        }
        return True, pwm_l, pwm_r, diag

class UnifiedMotionStrategy(ControlStrategy):
    name = CANONICAL_CONTROL_MODE
    TRACK_REF_OVERSPEED_HOLDOFF_MAX_MPS = 0.015

    def __init__(
        self,
        pid: PIDConfig,
        max_pwm: float,
        track_width: float,
        turn_intensity: float,
        inplace_turn_omega_deadband: float = 0.06,
    ):
        self.drive_ctrl = AlbaDriveController(pid)
        self.wheel_executor = WheelSpeedPILoop(
            pid,
            max_pwm=max_pwm,
            dead_zone=0.0,
            overspeed_holdoff_enabled=False,
        )
        self.pid_cfg = pid
        self.max_pwm = float(max_pwm)
        self.track_width = float(track_width)
        self.turn_intensity = float(turn_intensity)
        self.inplace_turn_omega_deadband = float(inplace_turn_omega_deadband)
        self._wheel_startup_active = {"left": False, "right": False}
        self._wheel_startup_sign = {"left": 0, "right": 0}
        self._wheel_startup_release_dwell_s = {"left": 0.0, "right": 0.0}
        self._last_diag = {}

    def reset(self):
        self.drive_ctrl.reset()
        self.wheel_executor.reset()
        self._wheel_startup_active = {"left": False, "right": False}
        self._wheel_startup_sign = {"left": 0, "right": 0}
        self._wheel_startup_release_dwell_s = {"left": 0.0, "right": 0.0}

    def _apply_wheel_startup_floor(
        self,
        *,
        side: str,
        reference_mps: float,
        measured_mps: float,
        maintenance_pwm: float,
        feedforward_diag: Dict,
        feedback_reliable: bool,
        dt: float,
    ) -> Tuple[float, Dict]:
        reference = float(reference_mps)
        measured = float(measured_mps)
        maintenance = float(maintenance_pwm)
        if abs(reference) <= 1e-6:
            self._wheel_startup_active[side] = False
            self._wheel_startup_sign[side] = 0
            self._wheel_startup_release_dwell_s[side] = 0.0
            return 0.0, {
                "startup_floor_active": False,
                "startup_floor_applied": False,
                "startup_pwm": abs(float(feedforward_diag.get("startup_pwm", 0.0) or 0.0)),
                "maintenance_feedforward_pwm": 0.0,
                "effective_feedforward_pwm": 0.0,
            }

        sign = 1 if reference > 0.0 else -1
        aligned_speed = sign * measured
        if self._wheel_startup_sign[side] != sign:
            self._wheel_startup_sign[side] = sign
            self._wheel_startup_active[side] = True
            self._wheel_startup_release_dwell_s[side] = 0.0
        elif self._wheel_startup_active[side]:
            if bool(feedback_reliable) and aligned_speed >= WHEEL_STARTUP_RELEASE_SPEED_MPS:
                self._wheel_startup_release_dwell_s[side] += max(0.0, float(dt))
                if (
                    self._wheel_startup_release_dwell_s[side]
                    + 1e-12
                    >= WHEEL_STARTUP_RELEASE_DWELL_S
                ):
                    self._wheel_startup_active[side] = False
            else:
                self._wheel_startup_release_dwell_s[side] = 0.0

        startup_pwm = min(
            self.max_pwm,
            abs(float(feedforward_diag.get("startup_pwm", abs(maintenance)) or abs(maintenance))),
        )
        floor_applied = bool(
            self._wheel_startup_active[side]
            and startup_pwm > abs(maintenance) + 1e-9
        )
        effective = (
            math.copysign(max(abs(maintenance), startup_pwm), reference)
            if self._wheel_startup_active[side]
            else maintenance
        )
        return float(effective), {
            "startup_floor_active": bool(self._wheel_startup_active[side]),
            "startup_floor_applied": floor_applied,
            "startup_pwm": float(startup_pwm),
            "maintenance_feedforward_pwm": float(maintenance),
            "effective_feedforward_pwm": float(effective),
            "startup_release_speed_mps": 0.020,
            "startup_release_dwell_s": float(WHEEL_STARTUP_RELEASE_DWELL_S),
            "startup_release_dwell_observed_s": float(
                self._wheel_startup_release_dwell_s[side]
            ),
            "startup_rearm_policy": "ZERO_OR_DIRECTION_CHANGE_ONLY",
        }

    def compute(self, state: Dict, setpoint: Dict) -> Tuple[float, float, Dict]:
        v_cmd = float(setpoint.get("v_cmd", 0.0))
        omega_cmd = float(setpoint.get("omega_cmd", 0.0))
        omega_cmd_request = float(setpoint.get("omega_cmd_request", omega_cmd))
        v_l = float(state.get("v_l", 0.0))
        v_r = float(state.get("v_r", 0.0))
        dt = float(state.get("dt", 0.0))
        current_yaw = state.get("current_yaw", None)

        v_l_ref = v_cmd - omega_cmd * (self.track_width * 0.5)
        v_r_ref = v_cmd + omega_cmd * (self.track_width * 0.5)

        closed_loop_track_reference = bool(state.get("closed_loop_track_reference_active", False))
        if abs(v_cmd) < 1e-3 and abs(omega_cmd) > 1e-3 and not closed_loop_track_reference:
            if abs(omega_cmd) < self.inplace_turn_omega_deadband:
                diag = {
                    "control_mode": self.name,
                    "active": False,
                    "v_cmd": v_cmd,
                    "omega_cmd": omega_cmd,
                    "v_l_ref": 0.0,
                    "v_r_ref": 0.0,
                    "v_l": v_l,
                    "v_r": v_r,
                    "v_avg": 0.5 * (v_l + v_r),
                    "v_err": 0.0,
                    "speed_p": 0.0,
                    "yaw_p": 0.0,
                    "yaw_i": 0.0,
                    "yaw_corr": 0.0,
                    "integrator_state": 0.0,
                    "integrator_limit": self.pid_cfg.integrator_limit,
                    "integrator_clamped": False,
                    "output_saturated": False,
                    "base_l": 0.0,
                    "base_r": 0.0,
                    "pwm_raw_l": 0.0,
                    "pwm_raw_r": 0.0,
                    "pwm_before_clamp_l": 0.0,
                    "pwm_before_clamp_r": 0.0,
                    "pwm_after_clamp_l": 0.0,
                    "pwm_after_clamp_r": 0.0,
                    "pwm_executor_l": 0.0,
                    "pwm_executor_r": 0.0,
                    "wheel_pi_enabled": True,
                    "Kp": self.pid_cfg.kp,
                    "Ki": self.pid_cfg.ki,
                    "dead_zone": self.drive_ctrl.dead_zone,
                }
                diag["monitor"] = {
                    "mode": self.name,
                    "v_cmd": v_cmd,
                    "omega_cmd": omega_cmd,
                    "feedforward": 0.0,
                    "dead_zone": self.drive_ctrl.dead_zone,
                    "speed_pi_output": 0.0,
                    "yaw_pi_output": 0.0,
                }
                diag["omega_cmd_request"] = omega_cmd_request
                diag["monitor"]["omega_cmd_request"] = omega_cmd_request
                self._last_diag = diag
                return 0.0, 0.0, diag
        executor_straight_hold_active = bool(
            state.get("executor_straight_hold_active", False)
        )
        base_l, ff_diag_l = self.drive_ctrl.get_wheel_feedforward("left", v_l_ref)
        base_r, ff_diag_r = self.drive_ctrl.get_wheel_feedforward("right", v_r_ref)
        if not bool(ff_diag_l.get("valid", False) and ff_diag_r.get("valid", False)):
            diag = {
                "control_mode": self.name,
                "active": False,
                "v_cmd": v_cmd,
                "omega_cmd": omega_cmd,
                "omega_cmd_request": omega_cmd_request,
                "v_l_ref": v_l_ref,
                "v_r_ref": v_r_ref,
                "pwm_executor_l": 0.0,
                "pwm_executor_r": 0.0,
                "output_reason": "WHEEL_SPEED_MAP_UNAVAILABLE",
                "wheel_feedforward": {
                    "left": ff_diag_l,
                    "right": ff_diag_r,
                },
            }
            self._last_diag = diag
            return 0.0, 0.0, diag
        maintenance_l = float(base_l)
        maintenance_r = float(base_r)
        measured_l = float(state.get("v_l_encoder", state.get("v_l", 0.0)) or 0.0)
        measured_r = float(state.get("v_r_encoder", state.get("v_r", 0.0)) or 0.0)
        startup_feedback_reliable = bool(
            not state.get("encoder_snapshot_stale", False)
            and state.get("encoder_timing_valid", True)
            and float(state.get("encoder_combined_trust", 0.0) or 0.0)
            >= self.wheel_executor.encoder_feedback_trust_min
        )
        base_l, startup_diag_l = self._apply_wheel_startup_floor(
            side="left",
            reference_mps=v_l_ref,
            measured_mps=measured_l,
            maintenance_pwm=maintenance_l,
            feedforward_diag=ff_diag_l,
            feedback_reliable=startup_feedback_reliable,
            dt=dt,
        )
        base_r, startup_diag_r = self._apply_wheel_startup_floor(
            side="right",
            reference_mps=v_r_ref,
            measured_mps=measured_r,
            maintenance_pwm=maintenance_r,
            feedforward_diag=ff_diag_r,
            feedback_reliable=startup_feedback_reliable,
            dt=dt,
        )
        ff_diag_l.update(startup_diag_l)
        ff_diag_r.update(startup_diag_r)
        maintenance_floor_l = abs(
            float(ff_diag_l.get("maintenance_pwm", 0.0) or 0.0)
        )
        maintenance_floor_r = abs(
            float(ff_diag_r.get("maintenance_pwm", 0.0) or 0.0)
        )
        track_ref_overspeed_holdoff = bool(
            closed_loop_track_reference
            and float(v_l_ref) * float(v_r_ref) < 0.0
            and max(abs(float(v_l_ref)), abs(float(v_r_ref)))
            <= self.TRACK_REF_OVERSPEED_HOLDOFF_MAX_MPS
        )
        wheel_state = dict(state)
        wheel_state["wheel_loop_overspeed_holdoff_enabled"] = bool(track_ref_overspeed_holdoff)
        wheel_ok, pwm_l, pwm_r, diag = self.wheel_executor.compute(
            state=wheel_state,
            v_cmd=v_cmd,
            omega_cmd=omega_cmd,
            omega_cmd_request=omega_cmd_request,
            v_l_ref=v_l_ref,
            v_r_ref=v_r_ref,
            dt=dt,
            feedforward_pwm_l=base_l,
            feedforward_pwm_r=base_r,
            maintenance_floor_pwm_l=maintenance_floor_l,
            maintenance_floor_pwm_r=maintenance_floor_r,
        )
        if wheel_ok:
            diag["control_mode"] = self.name
            diag["output_reason"] = "WHEEL_SPEED_LOOP"
            diag["four_direction_feedforward"] = True
            diag["wheel_feedforward"] = {
                "left": ff_diag_l,
                "right": ff_diag_r,
            }
            diag["wheel_loop_left_maintenance_feedforward_pwm"] = maintenance_l
            diag["wheel_loop_right_maintenance_feedforward_pwm"] = maintenance_r
            diag["wheel_loop_left_startup_floor_applied"] = bool(
                startup_diag_l["startup_floor_applied"]
            )
            diag["wheel_loop_right_startup_floor_applied"] = bool(
                startup_diag_r["startup_floor_applied"]
            )
            diag["executor_straight_hold_owner"] = bool(executor_straight_hold_active)
            diag["drive_yaw_hold_enabled"] = False
            diag["yaw_hold_enabled"] = False
            diag["track_ref_overspeed_holdoff"] = bool(track_ref_overspeed_holdoff)
            diag["track_ref_overspeed_holdoff_max_mps"] = float(
                self.TRACK_REF_OVERSPEED_HOLDOFF_MAX_MPS
            )
            zero_curvature_execution = bool(
                abs(float(v_cmd)) > 1e-6
                and abs(float(omega_cmd_request)) <= 1e-9
                and not executor_straight_hold_active
            )
            diag["heading_correction_owner"] = (
                "EXECUTOR_STRAIGHT_HOLD"
                if executor_straight_hold_active
                else "ZERO_CURVATURE_WHEEL_EXECUTOR"
                if zero_curvature_execution
                else "EXPLICIT_TRACK_REFERENCE"
            )
            diag["zero_curvature_execution"] = bool(zero_curvature_execution)
            monitor = dict(diag.get("monitor") or {})
            monitor.update(
                {
                    "mode": self.name,
                    "executor_straight_hold_owner": bool(executor_straight_hold_active),
                    "drive_yaw_hold_enabled": False,
                    "yaw_hold_enabled": False,
                    "heading_correction_owner": diag["heading_correction_owner"],
                    "zero_curvature_execution": bool(zero_curvature_execution),
                    "four_direction_feedforward": True,
                    "output_reason": "WHEEL_SPEED_LOOP",
                    "track_ref_overspeed_holdoff": bool(track_ref_overspeed_holdoff),
                }
            )
            diag["monitor"] = monitor
            self._last_diag = diag
            return pwm_l, pwm_r, diag

        feedback_source = str(
            diag.get("wheel_loop_feedback_source", "encoder_unavailable")
            or "encoder_unavailable"
        )
        timing_gap_rejected = feedback_source == "encoder_timing_gap"
        diag = {
            "control_mode": self.name,
            "active": False,
            "v_cmd": v_cmd,
            "omega_cmd": omega_cmd,
            "omega_cmd_request": omega_cmd_request,
            "v_l_ref": v_l_ref,
            "v_r_ref": v_r_ref,
            "pwm_executor_l": 0.0,
            "pwm_executor_r": 0.0,
            "wheel_loop_enabled": False,
            "wheel_loop_feedback_source": str(feedback_source),
            "wheel_loop_feedback_timing_valid": bool(
                diag.get("wheel_loop_feedback_timing_valid", not timing_gap_rejected)
            ),
            "wheel_loop_feedback_timing_error": str(
                diag.get("wheel_loop_feedback_timing_error", "") or ""
            ),
            "wheel_loop_feedback_timing_gap_s": float(
                diag.get("wheel_loop_feedback_timing_gap_s", 0.0) or 0.0
            ),
            "output_reason": (
                "ENCODER_TIMING_GAP" if timing_gap_rejected else "WHEEL_FEEDBACK_UNAVAILABLE"
            ),
            "four_direction_feedforward": True,
            "monitor": {
                "mode": self.name,
                "v_cmd": v_cmd,
                "omega_cmd": omega_cmd,
                "omega_cmd_request": omega_cmd_request,
                "wheel_loop_enabled": False,
                "output_reason": (
                    "ENCODER_TIMING_GAP" if timing_gap_rejected else "WHEEL_FEEDBACK_UNAVAILABLE"
                ),
            },
        }
        self._last_diag = diag
        return 0.0, 0.0, diag

def create_strategy(
    mode: str,
    pid: PIDConfig,
    max_pwm: float,
    track_width: float,
    turn_intensity: float,
    inplace_turn_omega_deadband: float = 0.06,
) -> ControlStrategy:
    normalize_control_mode(mode)
    return UnifiedMotionStrategy(
        pid,
        max_pwm=max_pwm,
        track_width=track_width,
        turn_intensity=turn_intensity,
        inplace_turn_omega_deadband=inplace_turn_omega_deadband,
    )
