#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MotionController: final command shaping layer for smooth teleop and stable estimation.

Responsibilities:
- optional joystick-domain command shaping (deadband + expo)
- velocity/angular slew-rate limiting
- differential-drive inverse kinematics references for diagnostics
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple

from controller.motion_kinematics import (
    enforce_twist_wheel_speed_range,
    twist_to_track_velocity,
)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(v)))


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


class MotionController:
    def __init__(
        self,
        *,
        track_width: float = 0.175,
        enable_input_shaping: bool = True,
        joy_deadband: float = 0.03,
        joy_expo_v: float = 0.35,
        joy_expo_omega: float = 0.45,
        enable_slew: bool = True,
        v_accel_m_s2: float = 0.6,
        v_decel_m_s2: float = 0.8,
        omega_accel_rad_s2: float = 1.8,
        omega_decel_rad_s2: float = 2.4,
    ):
        self.track_width = max(0.01, float(track_width))

        self.enable_input_shaping = bool(enable_input_shaping)
        self.joy_deadband = _clamp(float(joy_deadband), 0.0, 0.5)
        self.joy_expo_v = _clamp(float(joy_expo_v), 0.0, 1.0)
        self.joy_expo_omega = _clamp(float(joy_expo_omega), 0.0, 1.0)

        self.enable_slew = bool(enable_slew)
        self.v_accel_m_s2 = max(0.01, float(v_accel_m_s2))
        self.v_decel_m_s2 = max(self.v_accel_m_s2, float(v_decel_m_s2))
        self.omega_accel_rad_s2 = max(0.05, float(omega_accel_rad_s2))
        self.omega_decel_rad_s2 = max(self.omega_accel_rad_s2, float(omega_decel_rad_s2))

        self._v_prev = 0.0
        self._omega_prev = 0.0

    def reset(self) -> None:
        self._v_prev = 0.0
        self._omega_prev = 0.0

    def inverse_kinematics(self, v_cmd: float, omega_cmd: float) -> Tuple[float, float]:
        return twist_to_track_velocity(
            float(v_cmd),
            float(omega_cmd),
            float(self.track_width),
        )

    def _shape_normalized(self, n: float, deadband: float, expo: float) -> float:
        n = _clamp(n, -1.0, 1.0)
        if abs(n) <= deadband:
            return 0.0
        # Deadband removal with re-normalization keeps full-scale continuity.
        den = max(1e-6, 1.0 - deadband)
        n_db = math.copysign((abs(n) - deadband) / den, n)
        return (1.0 - expo) * n_db + expo * (n_db ** 3)

    def _shape_physical(self, value: float, limit: float, deadband: float, expo: float) -> float:
        lim = max(1e-6, abs(float(limit)))
        n = _clamp(float(value) / lim, -1.0, 1.0)
        return self._shape_normalized(n, deadband, expo) * lim

    def _slew_axis(self, current: float, target: float, dt: float, accel: float, decel: float) -> float:
        if dt <= 0.0:
            return float(current)
        direction_same = (target >= current >= 0.0) or (target <= current <= 0.0)
        slope = accel if direction_same else decel
        step = max(0.0, slope * dt)
        diff = target - current
        if abs(diff) <= step:
            return float(target)
        return float(current + (step if diff > 0.0 else -step))

    def _runtime_limits(self, ctrl) -> Dict[str, float]:
        speed_limits = getattr(ctrl, "speed_limits", None)
        if speed_limits is not None:
            profile = getattr(speed_limits, "profile", None)
            v_max = float(getattr(speed_limits, "effective_v_max", 0.0))
            if v_max <= 0.0:
                v_max = float(getattr(profile, "v_max", 0.6))
            v_min = max(0.0, float(getattr(profile, "v_min", 0.15)))
            w_max = float(getattr(speed_limits, "effective_w_max", 0.0))
            if w_max <= 0.0:
                w_max = float(getattr(profile, "w_max", 1.2))
            v_accel = float(getattr(speed_limits, "effective_accel_limit", self.v_accel_m_s2))
        else:
            v_max = 0.6
            v_min = 0.15
            w_max = 1.2
            v_accel = self.v_accel_m_s2

        cfg = (getattr(ctrl, "cfg", {}) or {}).get("vezerles", {}) if hasattr(ctrl, "cfg") else {}
        mc = (cfg.get("motion_controller") or {}) if isinstance(cfg, dict) else {}
        mozgas = cfg.get("mozgas", {}) if isinstance(cfg, dict) else {}

        out = {
            "v_max": max(0.01, float(v_max)),
            "v_min": min(max(0.0, float(v_min)), max(0.01, float(v_max))),
            "w_max": max(0.01, float(w_max)),
            "v_accel": max(0.01, _safe_float(mc.get("v_accel_m_s2"), v_accel)),
            "v_decel": max(0.01, _safe_float(mc.get("v_decel_m_s2"), max(v_accel, self.v_decel_m_s2))),
            "w_accel": max(0.05, _safe_float(mc.get("omega_accel_rad_s2"), self.omega_accel_rad_s2)),
            "w_decel": max(0.05, _safe_float(mc.get("omega_decel_rad_s2"), self.omega_decel_rad_s2)),
            "joy_deadband": _clamp(_safe_float(mc.get("joy_deadband"), self.joy_deadband), 0.0, 0.5),
            "joy_expo_v": _clamp(_safe_float(mc.get("joy_expo_v"), self.joy_expo_v), 0.0, 1.0),
            "joy_expo_omega": _clamp(_safe_float(mc.get("joy_expo_omega"), self.joy_expo_omega), 0.0, 1.0),
            "enable_input_shaping": bool(mc.get("enable_input_shaping", self.enable_input_shaping)),
            "enable_slew": bool(mc.get("enable_slew", self.enable_slew)),
        }
        out["v_decel"] = max(out["v_accel"], out["v_decel"])
        out["w_decel"] = max(out["w_accel"], out["w_decel"])
        return out

    @staticmethod
    def _apply_track_minimum(
        left_mps: float,
        right_mps: float,
        minimum_mps: float,
    ) -> Tuple[float, float, bool]:
        floor = max(0.0, float(minimum_mps))

        def _floor(value: float) -> float:
            value = float(value)
            if 0.0 < abs(value) < floor:
                return math.copysign(floor, value)
            return value

        left_out = _floor(left_mps)
        right_out = _floor(right_mps)
        applied = bool(
            abs(left_out - float(left_mps)) > 1e-9
            or abs(right_out - float(right_mps)) > 1e-9
        )
        return float(left_out), float(right_out), applied

    @staticmethod
    def _apply_track_maximum(
        left_mps: float,
        right_mps: float,
        maximum_mps: float,
    ) -> Tuple[float, float, bool]:
        ceiling = max(0.0, float(maximum_mps))

        def _cap(value: float) -> float:
            value = float(value)
            if ceiling > 0.0 and abs(value) > ceiling:
                return math.copysign(ceiling, value)
            return value

        left_out = _cap(left_mps)
        right_out = _cap(right_mps)
        applied = bool(
            abs(left_out - float(left_mps)) > 1e-9
            or abs(right_out - float(right_mps)) > 1e-9
        )
        return float(left_out), float(right_out), applied

    def tick_track_reference(
        self,
        *,
        ctrl,
        left_target_mps: float,
        right_target_mps: float,
        dt: float,
        force_zero: bool = False,
    ) -> Tuple[float, float, Dict[str, float]]:
        """Slew-limit an authoritative TRACK_EXEC reference in physical units.

        TRACK references stay on the same MotionController state and diff-drive
        kinematics as twist commands. Safety/localization hard stops use
        ``force_zero`` and therefore remain immediate rather than ramped.
        """
        limits = self._runtime_limits(ctrl)
        left_raw = float(left_target_mps)
        right_raw = float(right_target_mps)
        localization_apply = dict(
            (getattr(ctrl, "localization_gate_status", {}) or {}).get("apply") or {}
        )
        minimum_bypassed_for_localization = bool(
            str(localization_apply.get("reason", "") or "")
            == "localization_gate_speed_limit"
        )
        left_in = float(left_raw)
        right_in = float(right_raw)
        minimum_applied = False
        maximum_applied = False
        if not force_zero and not minimum_bypassed_for_localization:
            left_in, right_in, minimum_applied = self._apply_track_minimum(
                left_in,
                right_in,
                limits["v_min"],
            )
        if not force_zero:
            left_in, right_in, maximum_applied = self._apply_track_maximum(
                left_in,
                right_in,
                limits["v_max"],
            )
        v_in = 0.5 * (left_in + right_in)
        w_in = (right_in - left_in) / self.track_width

        if force_zero:
            self.reset()
            v_out = 0.0
            w_out = 0.0
        elif limits["enable_slew"]:
            v_out = self._slew_axis(
                self._v_prev,
                _clamp(v_in, -limits["v_max"], limits["v_max"]),
                float(dt),
                limits["v_accel"],
                limits["v_decel"],
            )
            w_out = self._slew_axis(
                self._omega_prev,
                _clamp(w_in, -limits["w_max"], limits["w_max"]),
                float(dt),
                limits["w_accel"],
                limits["w_decel"],
            )
        else:
            v_out = _clamp(v_in, -limits["v_max"], limits["v_max"])
            w_out = _clamp(w_in, -limits["w_max"], limits["w_max"])

        self._v_prev = float(v_out)
        self._omega_prev = float(w_out)
        left_out, right_out = self.inverse_kinematics(v_out, w_out)
        clamped = bool(
            abs(float(v_out) - float(v_in)) > 1e-9
            or abs(float(w_out) - float(w_in)) > 1e-9
        )
        ctrl.motion_controller_state = {
            "active": True,
            "mode": "TRACK_REFERENCE_SLEW",
            "source": str(getattr(ctrl, "motion_command_source", "") or ""),
            "force_zero": bool(force_zero),
            "v_in": round(float(v_in), 6),
            "omega_in": round(float(w_in), 6),
            "v_out": round(float(v_out), 6),
            "omega_out": round(float(w_out), 6),
            "v_l_in": round(float(left_in), 6),
            "v_r_in": round(float(right_in), 6),
            "v_l_raw": round(float(left_raw), 6),
            "v_r_raw": round(float(right_raw), 6),
            "track_minimum_mps": round(float(limits["v_min"]), 6),
            "track_minimum_applied": bool(minimum_applied),
            "track_maximum_mps": round(float(limits["v_max"]), 6),
            "track_maximum_applied": bool(maximum_applied),
            "track_minimum_bypassed_for_localization": bool(
                minimum_bypassed_for_localization
            ),
            "v_l_ref": round(float(left_out), 6),
            "v_r_ref": round(float(right_out), 6),
            "clamped": bool(clamped),
            "limiter": "track_reference_slew" if clamped else "",
            "limits": limits,
        }
        ctrl.motion_ref_v_l = float(left_out)
        ctrl.motion_ref_v_r = float(right_out)
        return (
            float(v_out),
            float(w_out),
            {"left_mps": float(left_out), "right_mps": float(right_out)},
        )

    def _apply_forward_dominant_envelope(
        self,
        *,
        v_cmd: float,
        omega_cmd: float,
        v_max: float,
        half_track: float,
        wheel_sign_margin_mps: float,
        turn_enable_eps: float,
    ) -> tuple[float, float, Dict[str, Any]]:
        """
        Curvature-speed envelope for forward-dominant (no reverse track) operation.

        Behaviour goals:
        - keep command physically realizable for forward-only track direction
        - prefer smooth transformation over hard yaw nulling
        - reduce speed on sharp curvature demand, then recompose omega from curvature
        """
        v_abs_in = max(0.0, float(v_cmd))
        w_in = float(omega_cmd)
        half_l = max(1e-6, float(half_track))
        v_eps_curv = max(0.005, min(max(0.0, float(turn_enable_eps)), 0.03))

        # Conservative curvature bounds keep one-track direction with margin.
        kappa_hard_max = max(0.01, (1.0 / half_l) * 0.72)
        kappa_soft_start = max(0.01, kappa_hard_max * 0.45)
        slowdown_min_ratio = 0.30
        slowdown_power = 1.6

        if v_abs_in <= 1e-9 and abs(w_in) <= 1e-9:
            return 0.0, 0.0, {
                "active": False,
                "curvature_in": 0.0,
                "curvature_out": 0.0,
                "kappa_hard_max": float(kappa_hard_max),
                "kappa_soft_start": float(kappa_soft_start),
                "speed_cap_mps": float(v_max),
                "actions": [],
            }

        # Near-zero linear intent still gets a bounded curvature estimate to avoid blow-up.
        if v_abs_in > v_eps_curv:
            kappa_in = w_in / max(v_abs_in, 1e-9)
        else:
            kappa_in = math.copysign(min(abs(w_in) / max(v_eps_curv, 1e-9), kappa_hard_max * 1.5), w_in)

        actions: list[str] = []
        kappa_cmd = float(kappa_in)
        if abs(kappa_cmd) > kappa_hard_max:
            kappa_cmd = math.copysign(kappa_hard_max, kappa_cmd)
            actions.append("limit_curvature_keep_track_direction")

        # Curvature-aware speed cap: higher curvature => lower max speed.
        curvature_ratio = 0.0
        if abs(kappa_cmd) > kappa_soft_start:
            curvature_ratio = _clamp(
                (abs(kappa_cmd) - kappa_soft_start) / max(1e-6, (kappa_hard_max - kappa_soft_start)),
                0.0,
                1.0,
            )
        speed_ratio = 1.0 - ((curvature_ratio ** slowdown_power) * (1.0 - slowdown_min_ratio))
        speed_cap_mps = max(0.0, float(v_max) * float(speed_ratio))
        v_abs_out = min(v_abs_in, speed_cap_mps)
        if v_abs_out + 1e-9 < v_abs_in:
            actions.append("slow_down_for_curvature_envelope")

        w_out = float(kappa_cmd * v_abs_out)

        # Final no-reverse track feasibility with small safety margin.
        max_w_keep_sign = max(0.0, (v_abs_out - max(0.0, float(wheel_sign_margin_mps))) / half_l)
        if abs(w_out) > max_w_keep_sign + 1e-9:
            w_out = math.copysign(max_w_keep_sign, w_out) if max_w_keep_sign > 0.0 else 0.0
            actions.append("limit_curvature_keep_track_direction")

        # Only ultra-low-speed anti-jitter zone keeps yaw at exactly zero.
        if v_abs_out <= min(0.006, v_eps_curv) and abs(w_out) <= 0.03:
            if abs(w_out) > 1e-9:
                actions.append("suppress_low_speed_yaw")
            w_out = 0.0

        actions = list(dict.fromkeys(actions))
        debug = {
            "active": True,
            "curvature_in": float(kappa_in),
            "curvature_out": float(w_out / max(v_abs_out, 1e-9)) if v_abs_out > 1e-9 else 0.0,
            "kappa_hard_max": float(kappa_hard_max),
            "kappa_soft_start": float(kappa_soft_start),
            "curvature_ratio": float(curvature_ratio),
            "speed_ratio": float(speed_ratio),
            "speed_cap_mps": float(speed_cap_mps),
            "actions": actions,
        }
        return float(v_abs_out), float(w_out), debug

    def tick(
        self,
        *,
        ctrl,
        v_target: float,
        omega_target: float,
        dt: float,
        ekf_state: Optional[Dict[str, Any]] = None,
        force_zero: bool = False,
    ) -> Tuple[float, float]:
        limits = self._runtime_limits(ctrl)
        source = str(getattr(ctrl, "motion_command_source", "") or "")
        active_command_type = str(getattr(ctrl, "active_motion_command_type", "") or "").strip()
        active_command_layer = str(getattr(ctrl, "active_motion_command_layer", "") or "").strip()
        limit_source = str(source)
        if active_command_type:
            limit_source = f"{limit_source}:{active_command_type}"
        if active_command_layer:
            limit_source = f"{limit_source}:{active_command_layer}"

        v_in = float(v_target)
        w_in = float(omega_target)

        if force_zero:
            self.reset()
            v_out = 0.0
            w_out = 0.0
            v_pre_limit = 0.0
            w_pre_limit = 0.0
        else:
            # Input shaping is joystick-domain smoothing; keep autonomous/scripted
            # motion intents (including MANUAL API/test commands) unwarped.
            if limits["enable_input_shaping"] and source == "GUI_JOYSTICK":
                v_in = self._shape_physical(
                    v_in,
                    limits["v_max"],
                    deadband=limits["joy_deadband"],
                    expo=limits["joy_expo_v"],
                )
                w_in = self._shape_physical(
                    w_in,
                    limits["w_max"],
                    deadband=limits["joy_deadband"],
                    expo=limits["joy_expo_omega"],
                )

            if limits["enable_slew"]:
                v_out = self._slew_axis(
                    self._v_prev,
                    _clamp(v_in, -limits["v_max"], limits["v_max"]),
                    float(dt),
                    limits["v_accel"],
                    limits["v_decel"],
                )
                w_out = self._slew_axis(
                    self._omega_prev,
                    _clamp(w_in, -limits["w_max"], limits["w_max"]),
                    float(dt),
                    limits["w_accel"],
                    limits["w_decel"],
                )
            else:
                v_out = _clamp(v_in, -limits["v_max"], limits["v_max"])
                w_out = _clamp(w_in, -limits["w_max"], limits["w_max"])
            v_pre_limit = float(v_out)
            w_pre_limit = float(w_out)

        speed_limits = getattr(ctrl, "speed_limits", None)
        limit_debug = None
        if speed_limits is not None and hasattr(speed_limits, "clamp_command"):
            v_out, w_out, limit_debug = speed_limits.clamp_command(
                float(v_out),
                float(w_out),
                motion_source=limit_source,
            )
            ctrl.last_speed_limit_debug = dict(limit_debug or {})
        clamped = bool(
            (limit_debug or {}).get("limiter")
            or abs(float(v_out) - float(v_pre_limit)) > 1e-9
            or abs(float(w_out) - float(w_pre_limit)) > 1e-9
        )

        # Track-direction guard:
        # forward/backward lineáris mozgásnál ne forduljon át egyik lánctalp
        # ellenirányba csak azért, mert az omega túl nagy.
        command_type_l = active_command_type.lower()
        command_layer_u = active_command_layer.upper()
        explicit_motion_target_arc = bool(
            command_layer_u == "MOTION_TARGET"
            and command_type_l in {"set_twist", "set_motion_target"}
            and abs(float(v_target)) > 0.02
            and abs(float(omega_target)) > 0.04
        )
        # Command fidelity: explicit v/omega is authoritative. Curvature is
        # now produced by the closed-loop left/right wheel executor.
        explicit_arc_omega_scale = 1.0
        if explicit_motion_target_arc:
            w_out = float(w_out) * float(explicit_arc_omega_scale)
            clamped = True
        manual_corner_guard = (
            command_type_l in ("set_speed", "turn", "discrete_manual", "recovery_discrete")
            or command_layer_u == "LEGACY_TANK_ADAPTER"
        )
        cfg = (getattr(ctrl, "cfg", {}) or {}).get("vezerles", {}) if hasattr(ctrl, "cfg") else {}
        mc_cfg = (cfg.get("motion_controller") or {}) if isinstance(cfg, dict) else {}
        policy_cfg = (mc_cfg.get("forward_dominant_policy") or {}) if isinstance(mc_cfg, dict) else {}
        policy_enabled = bool(policy_cfg.get("enabled", True))
        policy_allow_heading_rotate = bool(policy_cfg.get("allow_heading_rotate", True))
        policy_allow_explicit_reverse = bool(policy_cfg.get("allow_explicit_reverse_set_twist", True))
        policy_v_no_reverse_eps = max(0.0, _safe_float(policy_cfg.get("v_no_reverse_eps"), 0.02))
        explicit_pivot_omega_eps = 0.03
        policy_v_turn_enable_eps = max(
            policy_v_no_reverse_eps,
            _safe_float(policy_cfg.get("v_turn_enable_eps"), 0.04),
        )
        policy_wheel_sign_margin_mps = max(
            0.0,
            _safe_float(policy_cfg.get("wheel_sign_margin_mps"), 0.010),
        )
        policy_pwm_no_reverse_eps = max(0.0, _safe_float(policy_cfg.get("pwm_no_reverse_eps"), 0.02))
        is_recovery_like = bool(
            command_type_l.startswith("recovery")
            or command_layer_u.startswith("RECOVERY")
        )
        is_heading_rotate = bool(command_type_l in ("rotate_to_heading", "set_target_heading"))
        explicit_motion_target_pivot = bool(
            policy_allow_heading_rotate
            and command_layer_u == "MOTION_TARGET"
            and command_type_l in {"set_twist", "set_motion_target"}
            and max(abs(float(v_target)), abs(float(v_pre_limit)), abs(float(v_out))) <= policy_v_no_reverse_eps
            and max(abs(float(omega_target)), abs(float(w_pre_limit)), abs(float(w_out))) >= explicit_pivot_omega_eps
        )
        explicit_reverse_command = bool(
            policy_allow_explicit_reverse
            and command_type_l in {"set_twist", "set_motion_target", "drive_straight"}
            and (
                float(v_target) < -policy_v_no_reverse_eps
                or float(v_pre_limit) < -policy_v_no_reverse_eps
                or float(v_out) < -policy_v_no_reverse_eps
            )
        )
        resolution_status = dict(getattr(ctrl, "motion_resolution_status", {}) or {})
        resolved_motion = dict(resolution_status.get("resolved") or {})
        resolved_details = dict(resolved_motion.get("details") or {})
        resolved_speed_profile = dict(resolved_details.get("speed_profile") or {})
        resolved_clearance = dict(resolved_details.get("clearance") or {})
        resolved_local_navigation = dict(resolved_details.get("local_navigation") or {})
        resolved_speed_phase = str(resolved_speed_profile.get("phase", "") or "").strip().lower()
        is_local_planner_segment = bool(
            str(resolved_motion.get("layer", "") or "").strip().upper() in {"LOCAL_PLANNER", "LOCAL_NAVIGATION"}
            and str(resolved_motion.get("command_type", "") or "").strip().lower() == "local_planner_segment"
        )
        is_local_planner_heading_pivot = bool(
            is_local_planner_segment
            and resolved_speed_phase in {
                "obstacle_heading_pivot",
                "heading_align",
                "target_heading_align",
                "target_search_in_place",
                "target_reacquire_in_place",
                "camera_target_in_place_align",
                "follow_front_soft_turnout",
                "follow_front_hard_turnout",
            }
        )
        is_v2_follow_close_retreat = bool(
            is_local_planner_segment
            and resolved_speed_phase == "follow_close_retreat"
            and bool(resolved_local_navigation.get("rear_clear_for_retreat", False))
            and bool(resolved_local_navigation.get("global_clear_for_retreat", False))
            and (
                float(v_target) < -policy_v_no_reverse_eps
                or float(v_pre_limit) < -policy_v_no_reverse_eps
                or float(v_out) < -policy_v_no_reverse_eps
            )
        )
        is_local_planner_reverse_segment = bool(
            is_local_planner_segment
            and str(resolved_clearance.get("clearance_direction", "") or "").strip().lower() == "reverse"
            and bool(resolved_clearance.get("feasible", False))
            and not bool(resolved_clearance.get("blocked_back", False))
            and (
                float(v_target) < -policy_v_no_reverse_eps
                or float(v_pre_limit) < -policy_v_no_reverse_eps
                or float(v_out) < -policy_v_no_reverse_eps
            )
        )
        if not policy_enabled:
            policy_mode = "DISABLED"
        elif is_recovery_like:
            policy_mode = "RECOVERY_BYPASS"
        elif policy_allow_heading_rotate and (is_heading_rotate or is_local_planner_heading_pivot):
            policy_mode = "HEADING_ROTATE_BYPASS"
        elif explicit_motion_target_pivot:
            policy_mode = "EXPLICIT_PIVOT_BYPASS"
        elif explicit_reverse_command:
            policy_mode = "EXPLICIT_REVERSE_BYPASS"
        elif is_v2_follow_close_retreat or is_local_planner_reverse_segment:
            policy_mode = "JUSTIFIED_REVERSE_BYPASS"
        else:
            policy_mode = "ACTIVE"
        forward_dominant_no_reverse = bool(policy_mode == "ACTIVE")
        policy_actions: list[str] = []
        policy_applied = False
        reverse_guard_applied = False
        reverse_guard_reason = ""
        reverse_guard_profile = "disabled_for_curved_motion"
        w_before_reverse_guard = float(w_out)
        envelope_debug: Dict[str, Any] = {"active": False, "actions": []}
        half_l = self.track_width * 0.5
        if forward_dominant_no_reverse and half_l > 1e-6:
            reverse_guard_profile = "forward_dominant_envelope"
            if float(v_out) < 0.0:
                v_out = 0.0
                policy_actions.append("block_reverse_linear")
            v_out, w_out, envelope_debug = self._apply_forward_dominant_envelope(
                v_cmd=float(v_out),
                omega_cmd=float(w_out),
                v_max=float(limits["v_max"]),
                half_track=float(half_l),
                wheel_sign_margin_mps=float(policy_wheel_sign_margin_mps),
                turn_enable_eps=float(policy_v_turn_enable_eps),
            )
            policy_actions.extend(list(envelope_debug.get("actions", []) or []))
            policy_actions = list(dict.fromkeys(policy_actions))
            policy_applied = bool(policy_actions)
            reverse_guard_applied = bool(policy_applied)
            reverse_guard_reason = ",".join(policy_actions)
        elif manual_corner_guard and half_l > 1e-6 and abs(float(w_out)) > 1e-6:
            reverse_guard_profile = "manual_corner_guard"
            v_abs = abs(float(v_out))
            if v_abs > 0.03:
                max_w_keep_sign = max(0.0, (v_abs - 0.012) / half_l)
                if max_w_keep_sign > 0.0 and abs(float(w_out)) > max_w_keep_sign:
                    w_out = math.copysign(max_w_keep_sign, float(w_out))
                    reverse_guard_applied = True
                    reverse_guard_reason = "keep_track_direction"

        v_out, w_out, wheel_range_envelope = enforce_twist_wheel_speed_range(
            float(v_out),
            float(w_out),
            float(self.track_width),
            wheel_min_mps=float(limits["v_min"]),
            wheel_max_mps=float(limits["v_max"]),
        )
        if bool(wheel_range_envelope.get("applied", False)):
            clamped = True

        v_slew_state = float(v_out)
        w_slew_state = float(w_out)
        limiter_name = str((limit_debug or {}).get("limiter", "") or "")
        target_sign = math.copysign(1.0, float(v_target)) if abs(float(v_target)) > 1e-9 else 0.0
        output_sign = math.copysign(1.0, float(v_out)) if abs(float(v_out)) > 1e-9 else 0.0
        minimum_stop_transition_active = bool(
            limiter_name.endswith(".v_min")
            and target_sign == 0.0
            and output_sign != 0.0
        )
        if (
            limiter_name.endswith(".v_min")
            and output_sign != 0.0
            and target_sign != output_sign
        ):
            # The calibrated minimum is an actuator-output contract, not a
            # new motion request.  Keep the internal slew state moving toward
            # an exact zero (or through a requested direction change) while
            # the emitted command remains in the executable wheel range.
            # Otherwise v_min is fed back as the next slew state forever and
            # a planner hold can never physically complete.
            v_slew_state = float(v_pre_limit)

        self._v_prev = float(v_slew_state)
        self._omega_prev = float(w_slew_state)
        v_l_ref, v_r_ref = self.inverse_kinematics(v_out, w_out)

        ekf_state = dict(ekf_state or {})
        v_est = _safe_float(ekf_state.get("v"), 0.0)
        w_est = _safe_float(ekf_state.get("omega_rad_s"), 0.0)
        ctrl.motion_controller_state = {
            "active": True,
            "source": source,
            "force_zero": bool(force_zero),
            "v_in": round(float(v_target), 6),
            "omega_in": round(float(omega_target), 6),
            "v_pre_limit": round(float(v_pre_limit), 6),
            "omega_pre_limit": round(float(w_pre_limit), 6),
            "v_out": round(float(v_out), 6),
            "omega_out": round(float(w_out), 6),
            "v_slew_state": round(float(v_slew_state), 6),
            "omega_slew_state": round(float(w_slew_state), 6),
            "minimum_stop_transition_active": bool(minimum_stop_transition_active),
            "v_err_est": round(float(v_out - v_est), 6),
            "omega_err_est": round(float(w_out - w_est), 6),
            "v_l_ref": round(float(v_l_ref), 6),
            "v_r_ref": round(float(v_r_ref), 6),
            "clamped": bool(clamped),
            "limiter": str((limit_debug or {}).get("limiter", "") or ""),
            "omega_before_reverse_guard": round(float(w_before_reverse_guard), 6),
            "reverse_guard_applied": bool(reverse_guard_applied),
            "reverse_guard_reason": str(reverse_guard_reason or ""),
            "reverse_guard_profile": str(reverse_guard_profile),
            "forward_dominant_policy_enabled": bool(policy_enabled),
            "forward_dominant_policy_mode": str(policy_mode),
            "forward_dominant_no_reverse": bool(forward_dominant_no_reverse),
            "forward_dominant_policy_applied": bool(policy_applied),
            "forward_dominant_policy_actions": list(policy_actions),
            "forward_dominant_v_eps": round(float(policy_v_no_reverse_eps), 6),
            "forward_dominant_turn_enable_eps": round(float(policy_v_turn_enable_eps), 6),
            "forward_dominant_pwm_eps": round(float(policy_pwm_no_reverse_eps), 6),
            "forward_dominant_envelope": dict(envelope_debug or {}),
            "wheel_speed_range_envelope": dict(wheel_range_envelope or {}),
            "justified_reverse_segment": bool(is_v2_follow_close_retreat or is_local_planner_reverse_segment),
            "v2_follow_close_retreat": bool(is_v2_follow_close_retreat),
            "local_planner_reverse_segment": bool(is_local_planner_reverse_segment),
            "explicit_motion_target_pivot": bool(explicit_motion_target_pivot),
            "explicit_pivot_omega_eps": round(float(explicit_pivot_omega_eps), 6),
            "explicit_motion_target_arc": bool(explicit_motion_target_arc),
            "explicit_arc_omega_scale": round(float(explicit_arc_omega_scale), 6),
            "limits": limits,
            "speed_limit_debug": dict(limit_debug or {}),
        }
        ctrl.motion_ref_v_l = float(v_l_ref)
        ctrl.motion_ref_v_r = float(v_r_ref)
        return float(v_out), float(w_out)


def create_motion_controller_from_config(vezerles_cfg: Optional[Dict[str, Any]], *, track_width: float) -> MotionController:
    cfg = dict((vezerles_cfg or {}).get("motion_controller") or {})
    return MotionController(
        track_width=track_width,
        enable_input_shaping=bool(cfg.get("enable_input_shaping", True)),
        joy_deadband=float(cfg.get("joy_deadband", 0.03)),
        joy_expo_v=float(cfg.get("joy_expo_v", 0.35)),
        joy_expo_omega=float(cfg.get("joy_expo_omega", 0.45)),
        enable_slew=bool(cfg.get("enable_slew", True)),
        v_accel_m_s2=float(cfg.get("v_accel_m_s2", 0.6)),
        v_decel_m_s2=float(cfg.get("v_decel_m_s2", 0.8)),
        omega_accel_rad_s2=float(cfg.get("omega_accel_rad_s2", 1.8)),
        omega_decel_rad_s2=float(cfg.get("omega_decel_rad_s2", 2.4)),
    )
