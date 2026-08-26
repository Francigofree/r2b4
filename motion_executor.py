#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from middleware.ffp import PIDConfig
from core.control_strategies import create_strategy, normalize_control_mode
from controller.motion_schema import (
    EXEC_MODE_HEADING,
    EXEC_MODE_IDLE,
    EXEC_MODE_TRACK,
    EXEC_MODE_TWIST,
    normalize_execution_mode,
)
from controller.motion_kinematics import (
    track_velocity_to_twist as _track_velocity_to_twist_ssot,
    twist_to_track_velocity as _twist_to_track_velocity_ssot,
)


def _safe_float(value, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _angle_error_deg(target_deg: float, current_deg: float) -> float:
    err = (float(target_deg) - float(current_deg) + 180.0) % 360.0 - 180.0
    return float(err)


def twist_to_track_velocity(v_cmd: float, omega_cmd: float, track_width: float) -> tuple[float, float]:
    return _twist_to_track_velocity_ssot(
        float(v_cmd),
        float(omega_cmd),
        float(track_width),
    )


def track_velocity_to_twist(left_mps: float, right_mps: float, track_width: float) -> tuple[float, float]:
    return _track_velocity_to_twist_ssot(
        float(left_mps),
        float(right_mps),
        float(track_width),
    )


class MotionExecutor:
    """
    Motion executor: converts {v_cmd, omega_cmd} into {pwm_l, pwm_r}.
    Arcade/diff-drive: v_l = v_cmd - omega_cmd*(L/2), v_r = v_cmd + omega_cmd*(L/2), then PWM per wheel.
    Owns PI + feedforward logic, clamping and saturation.
    Stateless except for PID internals.
    NO knowledge of safety, LIDAR, AI, or state machine.
    """
    STRAIGHT_HOLD_HEADING_FILTER_TAU_S = 0.40

    def __init__(self, pid_config: PIDConfig, turn_intensity: float,
                 max_pwm: float, track_width: float = 0.175,
                 control_mode: str = "UNIFIED", direction_switch_hold_s: float = 0.08,
                 direction_switch_debounce_cycles: int = 3,
                 inplace_turn_omega_deadband: float = 0.06):
        """
        Initialize motion executor.

        Args:
            pid_config: wheel PI and executor configuration
            turn_intensity: Turn intensity scaling factor
            max_pwm: Maximum PWM value
            track_width: Distance between wheels (m) for diff-drive omega -> v_l/v_r
        """
        self.drive_pid_cfg = pid_config
        self.turn_intensity = turn_intensity
        self.max_pwm = max_pwm
        self.dz_min = pid_config.dz_min
        self.track_width = max(0.01, float(track_width))
        self.inplace_turn_omega_deadband = max(0.0, float(inplace_turn_omega_deadband))
        self.control_mode = normalize_control_mode(control_mode)
        self.strategy = create_strategy(
            self.control_mode,
            pid_config,
            max_pwm=self.max_pwm,
            track_width=self.track_width,
            turn_intensity=self.turn_intensity,
            inplace_turn_omega_deadband=self.inplace_turn_omega_deadband,
        )
        self.direction_switch_hold_s = max(0.0, float(direction_switch_hold_s))
        self.direction_switch_debounce_cycles = max(1, int(direction_switch_debounce_cycles))
        self._direction_switch_hold_until = 0.0
        self._last_v_cmd_sign = 0
        self._pending_switch_sign = 0
        self._pending_switch_count = 0
        self.straight_hold_enabled = bool(getattr(pid_config, "straight_hold_enabled", True))
        self.straight_hold_kp = max(0.0, float(getattr(pid_config, "straight_hold_kp", 1.15)))
        self.straight_hold_max_w = max(0.0, float(getattr(pid_config, "straight_hold_max_w", 0.14)))
        self.straight_hold_slew_rate = max(0.0, float(getattr(pid_config, "straight_hold_slew_rate", 0.90)))
        self.straight_hold_heading_deadband_deg = max(
            0.0,
            float(getattr(pid_config, "straight_hold_heading_deadband_deg", 0.35)),
        )
        self.straight_hold_v_min_mps = max(
            0.0,
            float(getattr(pid_config, "straight_hold_v_min_mps", 0.03)),
        )
        self.straight_hold_w_request_eps = max(
            0.0,
            float(getattr(pid_config, "straight_hold_w_request_eps", 0.03)),
        )
        self.motor_compensation_enabled = bool(getattr(pid_config, "motor_compensation_enabled", True))
        self.arc_track_positive_enforce = bool(getattr(pid_config, "arc_track_positive_enforce", True))
        self.arc_inner_floor_min_mps = max(
            0.0,
            _safe_float(getattr(pid_config, "arc_inner_floor_min_mps", 0.03), 0.03),
        )
        self.arc_track_diff_min_mps = max(
            0.0,
            _safe_float(getattr(pid_config, "arc_track_diff_min_mps", 0.004), 0.004),
        )
        self._straight_hold_ref_heading_deg = None
        self._straight_hold_filtered_error_deg = 0.0
        self._straight_hold_corr = 0.0
        self._straight_hold_active = False
        self._straight_hold_last_diag = {}

        # Utolsó ciklus wheel-loop diagnosztika.
        self._last_v_cmd = 0.0
        self._last_omega_cmd = 0.0
        self._last_omega_cmd_raw = 0.0
        self._last_v_l = 0.0
        self._last_v_r = 0.0
        self._last_v_l_ref = 0.0
        self._last_v_r_ref = 0.0
        self._last_pwm_l = 0.0
        self._last_pwm_r = 0.0
        self._last_wheel_pi_enabled = False
        self._last_base_l = 0.0
        self._last_base_r = 0.0
        self._last_pid_diag = {}
        # Passive lineage for Replayer captures; it does not participate in control.
        self._replayer_reset_generation = 0

    def _reset_straight_hold(self) -> None:
        self._straight_hold_ref_heading_deg = None
        self._straight_hold_filtered_error_deg = 0.0
        self._straight_hold_corr = 0.0
        self._straight_hold_active = False

    def _compute_straight_hold_omega(
        self,
        *,
        state: dict,
        dt: float,
        v_cmd: float,
        omega_cmd_requested: float,
        resolved_execution_mode: str,
    ) -> tuple[float, dict]:
        diag = {
            "enabled": bool(self.straight_hold_enabled),
            "candidate": False,
            "active": False,
            "reason": "disabled",
            "heading_ref_deg": self._straight_hold_ref_heading_deg,
            "heading_error_deg": 0.0,
            "omega_request_rad_s": float(omega_cmd_requested),
            "omega_correction_target_rad_s": 0.0,
            "omega_correction_rad_s": 0.0,
            "omega_output_rad_s": float(omega_cmd_requested),
            "slew_limited": False,
            "saturated": False,
            "turn_primitive_requested": str(state.get("turn_primitive_requested", "UNKNOWN") or "UNKNOWN"),
            "lidar_latest_age_s": state.get("lidar_latest_age_s"),
            "lidar_latest_confidence": state.get("lidar_latest_confidence"),
            "requested_v": state.get("requested_v"),
            "requested_omega": state.get("requested_omega"),
        }
        if not self.straight_hold_enabled:
            self._reset_straight_hold()
            self._straight_hold_last_diag = dict(diag)
            return float(omega_cmd_requested), diag

        turn_primitive_requested = str(state.get("turn_primitive_requested", "") or "").strip().upper()
        requested_v = _safe_float(state.get("requested_v", v_cmd), v_cmd)
        requested_omega = _safe_float(state.get("requested_omega", omega_cmd_requested), omega_cmd_requested)
        candidate_flag = bool(state.get("straight_hold_executor_candidate", False))
        candidate = bool(
            resolved_execution_mode == EXEC_MODE_TWIST
            and abs(requested_v) > self.straight_hold_v_min_mps
            and abs(float(v_cmd)) > self.straight_hold_v_min_mps
            and abs(requested_omega) <= self.straight_hold_w_request_eps
            and turn_primitive_requested == "STRAIGHT"
            and candidate_flag
        )
        diag["candidate"] = bool(candidate)
        diag["turn_primitive_requested"] = str(turn_primitive_requested or "UNKNOWN")
        diag["requested_v"] = float(requested_v)
        diag["requested_omega"] = float(requested_omega)
        if not candidate:
            if resolved_execution_mode != EXEC_MODE_TWIST:
                diag["reason"] = "execution_mode_not_twist"
            elif turn_primitive_requested != "STRAIGHT":
                diag["reason"] = f"turn_primitive_{turn_primitive_requested or 'unknown'}"
            elif not candidate_flag:
                diag["reason"] = "semantics_not_candidate"
            elif abs(requested_v) <= self.straight_hold_v_min_mps or abs(float(v_cmd)) <= self.straight_hold_v_min_mps:
                diag["reason"] = "below_speed_gate"
            elif abs(requested_omega) > self.straight_hold_w_request_eps:
                diag["reason"] = "requested_turn_nonzero"
            else:
                diag["reason"] = "gate_rejected"
            self._reset_straight_hold()
            self._straight_hold_last_diag = dict(diag)
            return float(omega_cmd_requested), diag

        heading_deg = _safe_float(
            state.get("ekf_theta_deg", state.get("current_yaw", math.nan)),
            math.nan,
        )
        if not math.isfinite(float(heading_deg)):
            diag["reason"] = "missing_heading"
            self._reset_straight_hold()
            self._straight_hold_last_diag = dict(diag)
            return float(omega_cmd_requested), diag

        if self._straight_hold_ref_heading_deg is None:
            self._straight_hold_ref_heading_deg = float(heading_deg)
            self._straight_hold_filtered_error_deg = 0.0
            self._straight_hold_corr = 0.0

        heading_error_raw_deg = _angle_error_deg(self._straight_hold_ref_heading_deg, heading_deg)
        filter_dt = max(1e-3, float(dt))
        filter_tau = float(self.STRAIGHT_HOLD_HEADING_FILTER_TAU_S)
        filter_alpha = filter_dt / (filter_tau + filter_dt)
        self._straight_hold_filtered_error_deg += filter_alpha * (
            float(heading_error_raw_deg) - float(self._straight_hold_filtered_error_deg)
        )
        heading_error_deg = float(self._straight_hold_filtered_error_deg)
        heading_error_rad = math.radians(float(heading_error_deg))
        p_term = self.straight_hold_kp * heading_error_rad
        corr_target = p_term
        # Straight is the zero-curvature ARC case. Hold omega at zero inside
        # the accumulated heading-error deadband; instantaneous yaw-rate noise
        # must not create alternating left/right corrections.
        if abs(float(heading_error_deg)) <= self.straight_hold_heading_deadband_deg:
            corr_target = 0.0
        corr_target = max(-self.straight_hold_max_w, min(self.straight_hold_max_w, float(corr_target)))
        max_delta = max(1e-6, self.straight_hold_slew_rate * max(float(dt), 1e-3))
        delta = float(corr_target) - float(self._straight_hold_corr)
        delta_limited = max(-max_delta, min(max_delta, delta))
        corr_applied = float(self._straight_hold_corr) + float(delta_limited)
        corr_applied = max(-self.straight_hold_max_w, min(self.straight_hold_max_w, corr_applied))

        self._straight_hold_corr = float(corr_applied)
        self._straight_hold_active = True
        omega_out = float(omega_cmd_requested) + float(corr_applied)
        omega_out = max(-self.straight_hold_max_w, min(self.straight_hold_max_w, omega_out))

        diag.update(
            {
                "active": True,
                "reason": "active",
                "heading_ref_deg": float(self._straight_hold_ref_heading_deg),
                "heading_error_deg": float(heading_error_deg),
                "heading_error_raw_deg": float(heading_error_raw_deg),
                "heading_error_filter_tau_s": float(filter_tau),
                "p_term_rad_s": float(p_term),
                "control_law": "ZERO_CURVATURE_HEADING_DRIFT_GUARD",
                "omega_correction_target_rad_s": float(corr_target),
                "omega_correction_rad_s": float(corr_applied),
                "omega_output_rad_s": float(omega_out),
                "slew_limited": abs(delta - delta_limited) > 1e-9,
                "saturated": abs(corr_target) >= (self.straight_hold_max_w - 1e-9),
            }
        )
        self._straight_hold_last_diag = dict(diag)
        return float(omega_out), diag

    def set_control_mode(self, mode: str, reset: bool = True):
        mode = normalize_control_mode(mode)
        self.control_mode = mode
        self.strategy = create_strategy(
            self.control_mode,
            self.drive_pid_cfg,
            max_pwm=self.max_pwm,
            track_width=self.track_width,
            turn_intensity=self.turn_intensity,
            inplace_turn_omega_deadband=self.inplace_turn_omega_deadband,
        )
        if reset and hasattr(self.strategy, "reset"):
            self.strategy.reset()

    def get_control_mode(self) -> str:
        return self.control_mode

    def _apply_motor_model_compensation(
        self,
        *,
        v_cmd: float,
        omega_cmd: float,
        resolved_execution_mode: str,
        sensor_feedback: dict | None = None,
    ) -> tuple[float, float, dict]:
        sensor_feedback = dict(sensor_feedback or {})
        left_raw, right_raw = twist_to_track_velocity(float(v_cmd), float(omega_cmd), self.track_width)
        comp_diag = {
            "enabled": bool(self.motor_compensation_enabled),
            "mode": str(resolved_execution_mode),
            "track_in": {
                "left_mps": float(left_raw),
                "right_mps": float(right_raw),
            },
            "track_out": {
                "left_mps": float(left_raw),
                "right_mps": float(right_raw),
            },
            "twist_out": {
                "v_cmd": float(v_cmd),
                "omega_cmd": float(omega_cmd),
            },
            "left_min_applied": False,
            "right_min_applied": False,
            "fixed_wheel_trim_active": False,
            "arc_contract_active": False,
            "arc_contract_applied": False,
            "arc_contract_reason": "",
            "arc_inner_floor_mps": 0.0,
            "arc_track_diff_min_mps": 0.0,
            "active": False,
            "skipped_reason": "",
        }
        if (not self.motor_compensation_enabled) or resolved_execution_mode == EXEC_MODE_IDLE:
            return float(v_cmd), float(omega_cmd), comp_diag

        left_out = float(left_raw)
        right_out = float(right_raw)

        arc_contract_active = bool(
            self.arc_track_positive_enforce
            and bool(sensor_feedback.get("arc_track_contract_active", False))
        )
        comp_diag["arc_contract_active"] = bool(arc_contract_active)
        if arc_contract_active:
            arc_inner_floor_hint = max(
                0.0,
                _safe_float(sensor_feedback.get("arc_inner_track_min_mps"), 0.0),
            )
            arc_diff_floor_hint = max(
                0.0,
                _safe_float(sensor_feedback.get("arc_track_diff_min_mps"), 0.0),
            )
            turn_left = bool(float(omega_cmd) >= 0.0)
            inner_floor = max(
                float(self.arc_inner_floor_min_mps),
                float(arc_inner_floor_hint),
            )
            diff_floor = max(float(self.arc_track_diff_min_mps), float(arc_diff_floor_hint))
            inner = float(left_out if turn_left else right_out)
            outer = float(right_out if turn_left else left_out)
            arc_contract_reasons = []
            arc_contract_applied = False
            if float(inner) < float(inner_floor):
                inner = float(inner_floor)
                arc_contract_applied = True
                arc_contract_reasons.append("inner_floor")
            if float(outer) <= float(inner + diff_floor):
                outer = float(inner + diff_floor)
                arc_contract_applied = True
                arc_contract_reasons.append("track_diff_floor")
            if turn_left:
                left_out, right_out = float(inner), float(outer)
            else:
                left_out, right_out = float(outer), float(inner)
            comp_diag["arc_contract_applied"] = bool(arc_contract_applied)
            comp_diag["arc_contract_reason"] = ",".join(arc_contract_reasons)
            comp_diag["arc_inner_floor_mps"] = float(inner_floor)
            comp_diag["arc_track_diff_min_mps"] = float(diff_floor)

        v_out, omega_out = track_velocity_to_twist(float(left_out), float(right_out), self.track_width)
        comp_diag["track_out"] = {
            "left_mps": float(left_out),
            "right_mps": float(right_out),
        }
        comp_diag["twist_out"] = {
            "v_cmd": float(v_out),
            "omega_cmd": float(omega_out),
        }
        comp_diag["active"] = bool(
            comp_diag["left_min_applied"]
            or comp_diag["right_min_applied"]
            or comp_diag["arc_contract_applied"]
            or abs(float(left_out) - float(left_raw)) > 1e-9
            or abs(float(right_out) - float(right_raw)) > 1e-9
        )
        return float(v_out), float(omega_out), comp_diag

    def compute_pwm(
        self,
        v_cmd: float,
        omega_cmd: float,
        sensor_feedback: dict,
        dt: float,
        *,
        execution_mode: str = "",
        track_reference: dict | None = None,
    ) -> tuple:
        """
        Compute PWM outputs from velocity and angular velocity commands.
        Arcade/diff-drive: v_l = v_cmd - omega_cmd*(L/2), v_r = v_cmd + omega_cmd*(L/2); then PWM per wheel.

        Args:
            v_cmd: Velocity command (m/s) - already ramped
            omega_cmd: Angular velocity command (rad/s)
            sensor_feedback: Dict with keys:
                - v_l: Left wheel velocity (m/s)
                - v_r: Right wheel velocity (m/s)
                - current_yaw: Current yaw angle (degrees), optional
            dt: Time step (s)

        Returns:
            Tuple (pwm_l, pwm_r) in range [-1.0, 1.0]
        """
        v_l = sensor_feedback.get("v_l", 0.0)
        v_r = sensor_feedback.get("v_r", 0.0)
        v_l_encoder = sensor_feedback.get("v_l_encoder", None)
        v_r_encoder = sensor_feedback.get("v_r_encoder", None)
        v_l_encoder_raw = sensor_feedback.get("v_l_encoder_raw", None)
        v_r_encoder_raw = sensor_feedback.get("v_r_encoder_raw", None)
        encoder_combined_trust = sensor_feedback.get("encoder_combined_trust", 0.0)
        encoder_forward_reliability = sensor_feedback.get("encoder_forward_reliability", 0.0)
        encoder_snapshot_stale = sensor_feedback.get("encoder_snapshot_stale", False)
        encoder_timing_valid = sensor_feedback.get("encoder_timing_valid", True)
        encoder_timing_error = str(sensor_feedback.get("encoder_timing_error", "") or "")
        encoder_timing_gap_s = sensor_feedback.get("encoder_timing_gap_s", None)
        feedback_velocity_source = str(sensor_feedback.get("feedback_velocity_source", "") or "")
        current_yaw = sensor_feedback.get("current_yaw", None)
        motion_source = str(sensor_feedback.get("motion_source", "") or "")
        active_command_type = str(sensor_feedback.get("active_command_type", "") or "").strip().lower()
        active_command_layer = str(sensor_feedback.get("active_command_layer", "") or "").strip().upper()
        active_execution_mode = str(sensor_feedback.get("active_execution_mode", "") or "").strip().upper()
        half_L = self.track_width * 0.5
        explicit_execution_mode = str(execution_mode or "").strip().upper()
        status_execution_mode = str(active_execution_mode or "").strip().upper()
        resolved_execution_mode = normalize_execution_mode(
            explicit_execution_mode or status_execution_mode,
            fallback=EXEC_MODE_TWIST,
        )
        execution_mode_contract_source = (
            "explicit_param"
            if explicit_execution_mode
            else ("status_surface" if status_execution_mode else "fallback_default")
        )
        execution_mode_contract_violation = (execution_mode_contract_source == "fallback_default")

        track_reference = dict(track_reference or {})
        track_left_ref = track_reference.get("left_mps")
        track_right_ref = track_reference.get("right_mps")
        track_ref_valid = False
        try:
            if track_left_ref is not None and track_right_ref is not None:
                track_left_ref = float(track_left_ref)
                track_right_ref = float(track_right_ref)
                if math.isfinite(track_left_ref) and math.isfinite(track_right_ref):
                    track_ref_valid = True
        except (TypeError, ValueError):
            track_ref_valid = False

        heading_track_ref_active = bool(
            resolved_execution_mode == EXEC_MODE_HEADING
            and track_ref_valid
            and active_command_type in ("rotate_to_heading", "set_target_heading")
        )
        authoritative_track_ref_active = bool(
            (resolved_execution_mode == EXEC_MODE_TRACK and track_ref_valid)
            or heading_track_ref_active
        )
        track_reference_mode = (
            "HEADING_PIVOT"
            if heading_track_ref_active
            else ("TRACK_EXEC" if resolved_execution_mode == EXEC_MODE_TRACK and track_ref_valid else "TWIST_DERIVED")
        )

        if authoritative_track_ref_active:
            v_cmd, omega_cmd = track_velocity_to_twist(
                float(track_left_ref),
                float(track_right_ref),
                self.track_width,
            )

        omega_cmd_request = float(omega_cmd)
        omega_cmd, straight_hold_diag = self._compute_straight_hold_omega(
            state=dict(sensor_feedback or {}),
            dt=float(dt),
            v_cmd=float(v_cmd),
            omega_cmd_requested=float(omega_cmd_request),
            resolved_execution_mode=str(resolved_execution_mode),
        )
        if authoritative_track_ref_active:
            _, _, motor_comp_diag = self._apply_motor_model_compensation(
                v_cmd=float(v_cmd),
                omega_cmd=float(omega_cmd),
                resolved_execution_mode=str(EXEC_MODE_IDLE),
                sensor_feedback=dict(sensor_feedback or {}),
            )
            motor_comp_diag["mode"] = str(resolved_execution_mode)
            motor_comp_diag["skipped_reason"] = "authoritative_track_reference"
        else:
            v_cmd, omega_cmd, motor_comp_diag = self._apply_motor_model_compensation(
                v_cmd=float(v_cmd),
                omega_cmd=float(omega_cmd),
                resolved_execution_mode=str(resolved_execution_mode),
                sensor_feedback=dict(sensor_feedback or {}),
            )
        omega_cmd_pre_guard = float(omega_cmd)

        self._last_v_cmd = v_cmd
        self._last_omega_cmd = omega_cmd
        self._last_omega_cmd_raw = omega_cmd_request
        self._last_v_l = v_l
        self._last_v_r = v_r
        if authoritative_track_ref_active:
            self._last_v_l_ref = float(track_left_ref)
            self._last_v_r_ref = float(track_right_ref)
        else:
            self._last_v_l_ref, self._last_v_r_ref = twist_to_track_velocity(
                v_cmd,
                omega_cmd,
                self.track_width,
            )
        self._last_wheel_pi_enabled = False

        # IPARI MEGOLDÁS: ha mindkét parancs 0, biztosan 0 PWM (beragadás elkerülésére)
        if abs(v_cmd) < 0.001 and abs(omega_cmd) < 0.001:
            self._last_v_cmd_sign = 0
            self._pending_switch_sign = 0
            self._pending_switch_count = 0
            self._last_pwm_l, self._last_pwm_r = 0.0, 0.0
            self._last_pid_diag = {
                "control_mode": self.control_mode,
                "active": False,
                "v_cmd": v_cmd,
                "omega_cmd": omega_cmd,
                "omega_cmd_raw": omega_cmd_pre_guard,
                "omega_cmd_request": omega_cmd_request,
                "v_l": v_l,
                "v_r": v_r,
                "feedback_velocity_source": str(feedback_velocity_source or "UNKNOWN"),
                "pwm_executor_l": 0.0,
                "pwm_executor_r": 0.0,
                "output_reason": "ZERO_CMD",
                "track_reverse_guard_applied": False,
                "track_reverse_guard_reason": "",
                "execution_mode": str(resolved_execution_mode),
                "track_reference": {
                    "left_mps": (None if not track_ref_valid else float(track_left_ref)),
                    "right_mps": (None if not track_ref_valid else float(track_right_ref)),
                },
                "execution_mode_contract_source": str(execution_mode_contract_source),
                "execution_mode_contract_violation": bool(execution_mode_contract_violation),
                "track_reference_mode": str(track_reference_mode),
                "straight_hold": dict(straight_hold_diag or {}),
                "motor_compensation": dict(motor_comp_diag or {}),
                "monitor": {
                    "mode": self.control_mode,
                    "v_cmd": v_cmd,
                    "omega_cmd": omega_cmd,
                    "omega_cmd_request": omega_cmd_request,
                    "output_reason": "ZERO_CMD",
                    "execution_mode": str(resolved_execution_mode),
                    "execution_mode_contract_source": str(execution_mode_contract_source),
                    "execution_mode_contract_violation": bool(execution_mode_contract_violation),
                    "track_reference_mode": str(track_reference_mode),
                    "straight_hold_active": bool((straight_hold_diag or {}).get("active", False)),
                    "motor_compensation_active": bool((motor_comp_diag or {}).get("active", False)),
                },
            }
            return 0.0, 0.0

        local_planner_pivot_requires_track = bool(
            not authoritative_track_ref_active
            and active_command_type == "local_planner_segment"
            and active_command_layer in {"LOCAL_NAVIGATION", "LOCAL_PLANNER"}
            and abs(float(v_cmd)) <= 0.006
            and abs(float(omega_cmd)) >= 0.035
        )
        if local_planner_pivot_requires_track:
            self._last_pwm_l, self._last_pwm_r = 0.0, 0.0
            self._last_pid_diag = {
                "control_mode": self.control_mode,
                "active": False,
                "v_cmd": v_cmd,
                "omega_cmd": omega_cmd,
                "omega_cmd_raw": omega_cmd_pre_guard,
                "omega_cmd_request": omega_cmd_request,
                "v_l": v_l,
                "v_r": v_r,
                "v_l_ref": self._last_v_l_ref,
                "v_r_ref": self._last_v_r_ref,
                "feedback_velocity_source": str(feedback_velocity_source or "UNKNOWN"),
                "pwm_executor_l": 0.0,
                "pwm_executor_r": 0.0,
                "output_reason": "LOCAL_NAV_PIVOT_TRACK_REQUIRED",
                "local_navigation_pivot_track_required": True,
                "m3_pivot_track_required": True,
                "track_reverse_guard_applied": False,
                "track_reverse_guard_reason": "",
                "execution_mode": str(resolved_execution_mode),
                "track_reference": {
                    "left_mps": (None if not track_ref_valid else float(track_left_ref)),
                    "right_mps": (None if not track_ref_valid else float(track_right_ref)),
                },
                "execution_mode_contract_source": str(execution_mode_contract_source),
                "execution_mode_contract_violation": bool(execution_mode_contract_violation),
                "track_reference_mode": str(track_reference_mode),
                "straight_hold": dict(straight_hold_diag or {}),
                "motor_compensation": dict(motor_comp_diag or {}),
                "monitor": {
                    "mode": self.control_mode,
                    "v_cmd": v_cmd,
                    "omega_cmd": omega_cmd,
                    "omega_cmd_request": omega_cmd_request,
                    "output_reason": "LOCAL_NAV_PIVOT_TRACK_REQUIRED",
                    "local_navigation_pivot_track_required": True,
                    "m3_pivot_track_required": True,
                    "execution_mode": str(resolved_execution_mode),
                    "execution_mode_contract_source": str(execution_mode_contract_source),
                    "execution_mode_contract_violation": bool(execution_mode_contract_violation),
                    "track_reference_mode": str(track_reference_mode),
                    "straight_hold_active": bool((straight_hold_diag or {}).get("active", False)),
                    "motor_compensation_active": bool((motor_comp_diag or {}).get("active", False)),
                },
            }
            return 0.0, 0.0

        # Szoftveres átmenet: előre <-> hátra váltásnál rövid nullázott holtidő.
        # Ezzel tiltjuk a hirtelen polaritásváltást, ami mechanikailag rántana.
        v_sign = 1 if v_cmd > 0.01 else (-1 if v_cmd < -0.01 else 0)
        now = time.perf_counter()
        if self.direction_switch_hold_s > 0:
            if v_sign != 0 and self._last_v_cmd_sign != 0 and v_sign != self._last_v_cmd_sign:
                if self._pending_switch_sign != v_sign:
                    self._pending_switch_sign = v_sign
                    self._pending_switch_count = 1
                else:
                    self._pending_switch_count += 1
                # Előjelváltást csak stabil (N ciklusos) ellenirány után tekintünk valódinak.
                if self._pending_switch_count >= self.direction_switch_debounce_cycles:
                    self._direction_switch_hold_until = now + self.direction_switch_hold_s
                    self._pending_switch_sign = 0
                    self._pending_switch_count = 0
                    if hasattr(self.strategy, "reset"):
                        self.strategy.reset()
            else:
                self._pending_switch_sign = 0
                self._pending_switch_count = 0
            if now < self._direction_switch_hold_until:
                self._last_pwm_l, self._last_pwm_r = 0.0, 0.0
                self._last_pid_diag = {
                    "control_mode": self.control_mode,
                    "active": False,
                    "v_cmd": v_cmd,
                    "omega_cmd": omega_cmd,
                    "omega_cmd_raw": omega_cmd_pre_guard,
                    "omega_cmd_request": omega_cmd_request,
                "v_l": v_l,
                "v_r": v_r,
                "feedback_velocity_source": str(feedback_velocity_source or "UNKNOWN"),
                "pwm_executor_l": 0.0,
                    "pwm_executor_r": 0.0,
                    "output_reason": "DIRECTION_SWITCH_HOLD",
                    "track_reverse_guard_applied": False,
                    "track_reverse_guard_reason": "",
                    "straight_hold": dict(straight_hold_diag or {}),
                    "motor_compensation": dict(motor_comp_diag or {}),
                    "monitor": {
                        "mode": self.control_mode,
                        "v_cmd": v_cmd,
                        "omega_cmd": omega_cmd,
                        "omega_cmd_request": omega_cmd_request,
                        "output_reason": "DIRECTION_SWITCH_HOLD",
                        "straight_hold_active": bool((straight_hold_diag or {}).get("active", False)),
                        "motor_compensation_active": bool((motor_comp_diag or {}).get("active", False)),
                    },
                }
                return 0.0, 0.0
        if v_sign != 0:
            self._last_v_cmd_sign = v_sign

        # Track-direction guard on actual executed command (post-ramp):
        # avoid one-side reverse kick during forward/backward cornering.
        omega_raw = float(omega_cmd)
        track_reverse_guard_applied = False
        track_reverse_guard_reason = ""
        manual_corner_guard = (
            resolved_execution_mode != EXEC_MODE_TRACK
            and (
                active_command_type in ("set_speed", "turn", "discrete_manual", "recovery_discrete")
                or active_command_layer == "LEGACY_TANK_ADAPTER"
            )
        )
        forward_dominant_no_reverse = bool(sensor_feedback.get("forward_dominant_no_reverse", False))
        try:
            forward_dominant_v_eps = max(0.0, float(sensor_feedback.get("forward_dominant_v_eps", 0.02) or 0.02))
        except (TypeError, ValueError):
            forward_dominant_v_eps = 0.02
        try:
            forward_dominant_pwm_eps = max(0.0, float(sensor_feedback.get("forward_dominant_pwm_eps", 0.02) or 0.02))
        except (TypeError, ValueError):
            forward_dominant_pwm_eps = 0.02
        forward_guard_applied = False
        forward_guard_reason = ""
        if manual_corner_guard and half_L > 1e-6 and abs(omega_raw) > 1e-9:
            v_abs = abs(float(v_cmd))
            rotate_only = (
                v_abs <= 0.01
                and abs(omega_raw) >= float(self.inplace_turn_omega_deadband)
            )
            if (not rotate_only) and v_abs > 0.004:
                max_w_keep_sign = max(0.0, (v_abs - 0.004) / half_L)
                if max_w_keep_sign > 0.0 and abs(omega_raw) > max_w_keep_sign:
                    omega_cmd = math.copysign(max_w_keep_sign, omega_raw)
                    track_reverse_guard_applied = True
                    track_reverse_guard_reason = "post_ramp_keep_track_direction"

        self._last_omega_cmd = float(omega_cmd)
        self._last_omega_cmd_raw = float(omega_raw)
        if authoritative_track_ref_active:
            self._last_v_l_ref = float(track_left_ref)
            self._last_v_r_ref = float(track_right_ref)
        else:
            self._last_v_l_ref, self._last_v_r_ref = twist_to_track_velocity(
                v_cmd,
                omega_cmd,
                self.track_width,
            )

        omega_meas = (v_r - v_l) / max(0.01, self.track_width)
        state = {
            "v_l": v_l,
            "v_r": v_r,
            "v_avg": 0.5 * (v_l + v_r),
            "omega_meas": omega_meas,
            "v_l_encoder": v_l_encoder,
            "v_r_encoder": v_r_encoder,
            "v_l_encoder_raw": v_l_encoder_raw,
            "v_r_encoder_raw": v_r_encoder_raw,
            "encoder_combined_trust": encoder_combined_trust,
            "encoder_forward_reliability": encoder_forward_reliability,
            "encoder_snapshot_stale": bool(encoder_snapshot_stale),
            "encoder_timing_valid": bool(encoder_timing_valid),
            "encoder_timing_error": str(encoder_timing_error),
            "encoder_timing_gap_s": encoder_timing_gap_s,
            "encoder_left_distance_delta_m": sensor_feedback.get("encoder_left_distance_delta_m"),
            "encoder_right_distance_delta_m": sensor_feedback.get("encoder_right_distance_delta_m"),
            "encoder_aggregation_window_s": sensor_feedback.get("encoder_aggregation_window_s"),
            "feedback_velocity_source": str(feedback_velocity_source or "UNKNOWN"),
            "current_yaw": current_yaw,
            "motion_source": motion_source,
            "active_command_type": active_command_type,
            "active_command_layer": active_command_layer,
            "executor_straight_hold_active": bool(
                (straight_hold_diag or {}).get("active", False)
            ),
            "closed_loop_track_reference_active": bool(authoritative_track_ref_active),
            "dt": dt,
        }
        setpoint = {
            "v_cmd": v_cmd,
            "omega_cmd": omega_cmd,
            "omega_cmd_request": float(omega_cmd_request),
        }
        pwm_l, pwm_r, diag = self.strategy.compute(state, setpoint)
        if not isinstance(diag, dict):
            diag = {}
        if resolved_execution_mode == EXEC_MODE_TRACK and track_ref_valid:
            diag["v_l_ref"] = float(track_left_ref)
            diag["v_r_ref"] = float(track_right_ref)
        elif heading_track_ref_active:
            diag["v_l_ref"] = float(track_left_ref)
            diag["v_r_ref"] = float(track_right_ref)
        diag["omega_cmd_raw"] = float(omega_raw)
        diag["omega_cmd_request"] = float(omega_cmd_request)
        diag["omega_cmd_pre_guard"] = float(omega_cmd_pre_guard)
        diag["straight_hold"] = dict(straight_hold_diag or {})
        diag["motor_compensation"] = dict(motor_comp_diag or {})
        diag["track_reverse_guard_applied"] = bool(track_reverse_guard_applied)
        diag["track_reverse_guard_reason"] = str(track_reverse_guard_reason or "")
        diag["track_reverse_guard_profile"] = (
            "manual_corner_guard" if manual_corner_guard else "disabled_for_curved_motion"
        )
        diag["execution_mode"] = str(resolved_execution_mode)
        diag["execution_mode_contract_source"] = str(execution_mode_contract_source)
        diag["execution_mode_contract_violation"] = bool(execution_mode_contract_violation)
        diag["track_reference_mode"] = str(track_reference_mode)
        diag["track_reference"] = {
            "left_mps": (None if not track_ref_valid else float(track_left_ref)),
            "right_mps": (None if not track_ref_valid else float(track_right_ref)),
        }
        diag["feedback_velocity_source"] = str(feedback_velocity_source or "UNKNOWN")
        diag.setdefault("output_reason", "NONE")
        mon = diag.get("monitor")
        if isinstance(mon, dict):
            mon["feedback_velocity_source"] = str(feedback_velocity_source or "UNKNOWN")
            mon["omega_cmd_raw"] = float(omega_raw)
            mon["omega_cmd_request"] = float(omega_cmd_request)
            mon["omega_cmd_pre_guard"] = float(omega_cmd_pre_guard)
            mon["straight_hold_active"] = bool((straight_hold_diag or {}).get("active", False))
            mon["straight_hold_correction"] = _safe_float(
                (straight_hold_diag or {}).get("omega_correction_rad_s"),
                0.0,
            )
            mon["straight_hold_target_rad_s"] = _safe_float(
                (straight_hold_diag or {}).get("omega_correction_target_rad_s"),
                0.0,
            )
            mon["straight_hold_heading_error_deg"] = _safe_float(
                (straight_hold_diag or {}).get("heading_error_deg"),
                0.0,
            )
            mon["straight_hold_slew_limited"] = bool(
                (straight_hold_diag or {}).get("slew_limited", False)
            )
            mon["motor_compensation_active"] = bool((motor_comp_diag or {}).get("active", False))
            mon["track_reverse_guard_applied"] = bool(track_reverse_guard_applied)
            mon["track_reverse_guard_reason"] = str(track_reverse_guard_reason or "")
            mon["track_reverse_guard_profile"] = (
                "manual_corner_guard" if manual_corner_guard else "disabled_for_curved_motion"
            )
            mon["execution_mode"] = str(resolved_execution_mode)
            mon["execution_mode_contract_source"] = str(execution_mode_contract_source)
            mon["execution_mode_contract_violation"] = bool(execution_mode_contract_violation)
            mon["track_reference_mode"] = str(track_reference_mode)
            mon.setdefault("output_reason", "NONE")

        if forward_dominant_no_reverse and resolved_execution_mode != EXEC_MODE_TRACK:
            if abs(float(v_cmd)) <= forward_dominant_v_eps:
                if float(pwm_l) < -forward_dominant_pwm_eps or float(pwm_r) < -forward_dominant_pwm_eps:
                    pwm_l = max(0.0, float(pwm_l))
                    pwm_r = max(0.0, float(pwm_r))
                    if pwm_l <= forward_dominant_pwm_eps and pwm_r <= forward_dominant_pwm_eps:
                        pwm_l, pwm_r = 0.0, 0.0
                    forward_guard_applied = True
                    forward_guard_reason = "low_speed_negative_clip"
            else:
                min_pwm = min(float(pwm_l), float(pwm_r))
                if min_pwm < -forward_dominant_pwm_eps:
                    before_guard_l = float(pwm_l)
                    before_guard_r = float(pwm_r)
                    pwm_l = max(0.0, float(pwm_l))
                    pwm_r = max(0.0, float(pwm_r))
                    left_ref = _safe_float(diag.get("v_l_ref"), self._last_v_l_ref)
                    right_ref = _safe_float(diag.get("v_r_ref"), self._last_v_r_ref)
                    ref_floor = 0.004
                    same_forward_ref = bool(left_ref >= ref_floor and right_ref >= ref_floor)
                    balance_floor = 0.0
                    if same_forward_ref:
                        stronger_pwm = max(float(pwm_l), float(pwm_r), 0.0)
                        low_speed_guard_cap = 0.36
                        pwm_l = min(float(pwm_l), float(low_speed_guard_cap))
                        pwm_r = min(float(pwm_r), float(low_speed_guard_cap))
                        stronger_pwm = min(float(stronger_pwm), float(low_speed_guard_cap))
                        max_ref = max(abs(float(left_ref)), abs(float(right_ref)))
                        min_ref = min(abs(float(left_ref)), abs(float(right_ref)))
                        ref_ratio = 1.0 if max_ref <= 1e-6 else max(0.25, min(1.0, min_ref / max_ref))
                        if stronger_pwm <= forward_dominant_pwm_eps:
                            dead_zone_floor = _safe_float(
                                diag.get("dead_zone", getattr(self, "dz_min", 0.20)),
                                0.20,
                            )
                            stronger_pwm = max(
                                forward_dominant_pwm_eps * 3.0,
                                min(0.24, max(forward_dominant_pwm_eps, dead_zone_floor)),
                            )
                        balance_floor = min(stronger_pwm, stronger_pwm * ref_ratio)
                        if stronger_pwm > forward_dominant_pwm_eps and balance_floor > forward_dominant_pwm_eps:
                            both_negative = bool(
                                before_guard_l < -forward_dominant_pwm_eps
                                and before_guard_r < -forward_dominant_pwm_eps
                            )
                            if both_negative and abs(float(left_ref)) >= abs(float(right_ref)):
                                pwm_l = max(float(pwm_l), float(stronger_pwm))
                                pwm_r = max(float(pwm_r), float(balance_floor))
                            elif both_negative:
                                pwm_l = max(float(pwm_l), float(balance_floor))
                                pwm_r = max(float(pwm_r), float(stronger_pwm))
                            else:
                                if before_guard_l < -forward_dominant_pwm_eps:
                                    pwm_l = max(float(pwm_l), float(balance_floor))
                                if before_guard_r < -forward_dominant_pwm_eps:
                                    pwm_r = max(float(pwm_r), float(balance_floor))
                    diag["forward_dominant_guard_pre_pwm_l"] = float(before_guard_l)
                    diag["forward_dominant_guard_pre_pwm_r"] = float(before_guard_r)
                    diag["forward_dominant_balance_floor_pwm"] = float(balance_floor)
                    diag["forward_dominant_guard_pwm_cap"] = 0.36
                    forward_guard_applied = True
                    forward_guard_reason = "clip_reverse_balance_floor" if balance_floor > 0.0 else "clip_reverse_no_shift"
                elif float(pwm_l) < 0.0 or float(pwm_r) < 0.0:
                    pwm_l = max(0.0, float(pwm_l))
                    pwm_r = max(0.0, float(pwm_r))
                    forward_guard_applied = True
                    forward_guard_reason = "clip_small_negative"

        diag["forward_dominant_guard_active"] = bool(forward_dominant_no_reverse)
        diag["forward_dominant_guard_applied"] = bool(forward_guard_applied)
        diag["forward_dominant_guard_reason"] = str(forward_guard_reason or "")
        diag["forward_dominant_guard_v_eps"] = float(forward_dominant_v_eps)
        diag["forward_dominant_guard_pwm_eps"] = float(forward_dominant_pwm_eps)
        if isinstance(mon, dict):
            mon["forward_dominant_guard_active"] = bool(forward_dominant_no_reverse)
            mon["forward_dominant_guard_applied"] = bool(forward_guard_applied)
            mon["forward_dominant_guard_reason"] = str(forward_guard_reason or "")
        if forward_guard_applied:
            diag["upstream_output_reason"] = str(diag.get("output_reason", "NONE") or "NONE")
            diag["output_reason"] = "FORWARD_DOMINANT_GUARD"
            if isinstance(mon, dict):
                mon["upstream_output_reason"] = str(mon.get("output_reason", "NONE") or "NONE")
                mon["output_reason"] = "FORWARD_DOMINANT_GUARD"

        track_direction_guard_active = False
        track_direction_guard_applied = False
        track_direction_guard_reason = ""
        if resolved_execution_mode != EXEC_MODE_IDLE:
            ref_floor = 0.004
            left_ref = _safe_float(diag.get("v_l_ref"), self._last_v_l_ref)
            right_ref = _safe_float(diag.get("v_r_ref"), self._last_v_r_ref)
            same_forward_ref = bool(left_ref >= ref_floor and right_ref >= ref_floor)
            same_reverse_ref = bool(left_ref <= -ref_floor and right_ref <= -ref_floor)
            if same_forward_ref or same_reverse_ref:
                track_direction_guard_active = True
                before_l, before_r = float(pwm_l), float(pwm_r)
                balance_floor = 0.0
                if same_forward_ref:
                    if float(pwm_l) < 0.0:
                        pwm_l = 0.0
                    if float(pwm_r) < 0.0:
                        pwm_r = 0.0
                    track_direction_guard_reason = "symmetric_forward_ref"
                else:
                    if float(pwm_l) > 0.0:
                        pwm_l = 0.0
                    if float(pwm_r) > 0.0:
                        pwm_r = 0.0
                    max_ref = max(abs(float(left_ref)), abs(float(right_ref)))
                    min_ref = min(abs(float(left_ref)), abs(float(right_ref)))
                    ref_ratio = 0.75 if max_ref <= 1e-6 else max(0.35, min(0.75, min_ref / max_ref))
                    stronger_pwm = max(-float(pwm_l), -float(pwm_r), 0.0)
                    if stronger_pwm <= forward_dominant_pwm_eps:
                        dead_zone_floor = _safe_float(
                            diag.get("dead_zone", getattr(self, "dz_min", 0.20)),
                            0.20,
                        )
                        stronger_pwm = max(
                            forward_dominant_pwm_eps * 3.0,
                            min(0.24, max(forward_dominant_pwm_eps, dead_zone_floor)),
                        )
                    balance_floor = min(stronger_pwm, stronger_pwm * ref_ratio)
                    if before_l > 0.0:
                        pwm_l = -float(balance_floor)
                    if before_r > 0.0:
                        pwm_r = -float(balance_floor)
                    track_direction_guard_reason = "symmetric_reverse_ref"
                track_direction_guard_applied = bool(
                    abs(float(pwm_l) - before_l) > 1e-9
                    or abs(float(pwm_r) - before_r) > 1e-9
                )
                diag["track_direction_balance_floor_pwm"] = float(balance_floor)

        diag["track_direction_guard_active"] = bool(track_direction_guard_active)
        diag["track_direction_guard_applied"] = bool(track_direction_guard_applied)
        diag["track_direction_guard_reason"] = str(track_direction_guard_reason or "")
        if isinstance(mon, dict):
            mon["track_direction_guard_active"] = bool(track_direction_guard_active)
            mon["track_direction_guard_applied"] = bool(track_direction_guard_applied)
            mon["track_direction_guard_reason"] = str(track_direction_guard_reason or "")
        if track_direction_guard_applied and str(diag.get("output_reason", "NONE") or "NONE") == "NONE":
            diag["output_reason"] = "TRACK_DIRECTION_GUARD"
            if isinstance(mon, dict):
                mon["output_reason"] = "TRACK_DIRECTION_GUARD"

        track_one_side_hold_guard_active = False
        track_one_side_hold_guard_applied = False
        track_one_side_hold_guard_reason = ""
        if resolved_execution_mode == EXEC_MODE_TRACK and track_ref_valid:
            ref_eps = 1e-6
            moving_ref_floor = 0.004
            left_ref = float(track_left_ref)
            right_ref = float(track_right_ref)
            left_stationary = bool(abs(left_ref) <= ref_eps and abs(right_ref) >= moving_ref_floor)
            right_stationary = bool(abs(right_ref) <= ref_eps and abs(left_ref) >= moving_ref_floor)
            if left_stationary or right_stationary:
                track_one_side_hold_guard_active = True
                before_l, before_r = float(pwm_l), float(pwm_r)
                if left_stationary:
                    pwm_l = 0.0
                    if right_ref > 0.0 and float(pwm_r) < 0.0:
                        pwm_r = 0.0
                    elif right_ref < 0.0 and float(pwm_r) > 0.0:
                        pwm_r = 0.0
                    track_one_side_hold_guard_reason = "left_track_stationary"
                elif right_stationary:
                    pwm_r = 0.0
                    if left_ref > 0.0 and float(pwm_l) < 0.0:
                        pwm_l = 0.0
                    elif left_ref < 0.0 and float(pwm_l) > 0.0:
                        pwm_l = 0.0
                    track_one_side_hold_guard_reason = "right_track_stationary"
                track_one_side_hold_guard_applied = bool(
                    abs(float(pwm_l) - before_l) > 1e-9
                    or abs(float(pwm_r) - before_r) > 1e-9
                )

        diag["track_one_side_hold_guard_active"] = bool(track_one_side_hold_guard_active)
        diag["track_one_side_hold_guard_applied"] = bool(track_one_side_hold_guard_applied)
        diag["track_one_side_hold_guard_reason"] = str(track_one_side_hold_guard_reason or "")
        if isinstance(mon, dict):
            mon["track_one_side_hold_guard_active"] = bool(track_one_side_hold_guard_active)
            mon["track_one_side_hold_guard_applied"] = bool(track_one_side_hold_guard_applied)
            mon["track_one_side_hold_guard_reason"] = str(track_one_side_hold_guard_reason or "")
        if track_one_side_hold_guard_applied and str(diag.get("output_reason", "NONE") or "NONE") == "NONE":
            diag["output_reason"] = "TRACK_ONE_SIDE_HOLD_GUARD"
            if isinstance(mon, dict):
                mon["output_reason"] = "TRACK_ONE_SIDE_HOLD_GUARD"

        # Final clamping to [-1.0, 1.0]
        pwm_l = max(-1.0, min(1.0, pwm_l))
        pwm_r = max(-1.0, min(1.0, pwm_r))
        diag["pwm_executor_l"] = float(pwm_l)
        diag["pwm_executor_r"] = float(pwm_r)
        if isinstance(mon, dict):
            mon["pwm_executor_l"] = float(pwm_l)
            mon["pwm_executor_r"] = float(pwm_r)
        self._last_pid_diag = diag

        self._last_pwm_l, self._last_pwm_r = pwm_l, pwm_r
        return pwm_l, pwm_r

    def get_last_pid_diagnostics(self) -> dict:
        """Utolsó compute_pwm ciklus diagnosztikája: ref, mérés, P/I, PWM pipeline."""
        out = dict(self._last_pid_diag or {})
        out.setdefault("v_l_ref", self._last_v_l_ref)
        out.setdefault("v_r_ref", self._last_v_r_ref)
        out.setdefault("v_l", self._last_v_l)
        out.setdefault("v_r", self._last_v_r)
        out.setdefault("v_cmd", self._last_v_cmd)
        out.setdefault("omega_cmd", self._last_omega_cmd)
        out.setdefault("omega_cmd_request", self._last_omega_cmd_raw)
        out.setdefault("wheel_pi_enabled", self._last_wheel_pi_enabled)
        out.setdefault("pwm_executor_l", self._last_pwm_l)
        out.setdefault("pwm_executor_r", self._last_pwm_r)
        out.setdefault("base_l", getattr(self, "_last_base_l", 0.0))
        out.setdefault("base_r", getattr(self, "_last_base_r", 0.0))
        out.setdefault("straight_hold", dict(self._straight_hold_last_diag or {}))
        return out

    def compute_calibration_pwm(
        self,
        *,
        left_pwm: float,
        right_pwm: float,
        v_hint: float,
        hard_cap: float = 0.35,
        phase: str = "maintenance",
    ) -> tuple[float, float]:
        """Bounded open-loop calibration output; safety filtering remains downstream."""
        left = _safe_float(left_pwm, 0.0)
        right = _safe_float(right_pwm, 0.0)
        hint = _safe_float(v_hint, 0.0)
        cap = max(0.0, min(0.90, abs(_safe_float(hard_cap, 0.90))))
        valid = bool(
            cap > 0.0
            and left * right > 0.0
            and hint != 0.0
            and math.copysign(1.0, left) == math.copysign(1.0, hint)
            and math.copysign(1.0, right) == math.copysign(1.0, hint)
            and max(abs(left), abs(right)) <= cap + 1e-9
        )
        if not valid:
            left, right = 0.0, 0.0
            reason = "CALIBRATION_PWM_REJECTED"
        else:
            reason = "CALIBRATION_DIRECT_PWM"
        if hasattr(self.strategy, "reset"):
            self.strategy.reset()
        self._last_v_cmd = hint if valid else 0.0
        self._last_omega_cmd = 0.0
        self._last_v_l_ref = 0.0
        self._last_v_r_ref = 0.0
        self._last_pwm_l = float(left)
        self._last_pwm_r = float(right)
        self._last_wheel_pi_enabled = False
        self._last_pid_diag = {
            "control_mode": self.control_mode,
            "active": bool(valid),
            "direct_pwm": True,
            "calibration_pwm": True,
            "v_cmd": hint if valid else 0.0,
            "omega_cmd": 0.0,
            "v_l_ref": 0.0,
            "v_r_ref": 0.0,
            "pwm_executor_l": float(left),
            "pwm_executor_r": float(right),
            "output_reason": reason,
            "calibration_pwm_phase": str(phase or "maintenance"),
            "wheel_pi_enabled": False,
            "pi_correction_left_pwm": 0.0,
            "pi_correction_right_pwm": 0.0,
            "feedforward_map_applied": False,
            "straight_hold_applied": False,
            "planner_correction_applied": False,
            "startup_floor_applied": False,
            "maintenance_floor_applied": False,
            "monitor": {
                "mode": self.control_mode,
                "v_cmd": hint if valid else 0.0,
                "omega_cmd": 0.0,
                "pwm_executor_l": float(left),
                "pwm_executor_r": float(right),
                "direct_pwm": True,
                "calibration_pwm": True,
                "output_reason": reason,
                "calibration_pwm_phase": str(phase or "maintenance"),
                "wheel_pi_enabled": False,
                "pi_correction_left_pwm": 0.0,
                "pi_correction_right_pwm": 0.0,
                "feedforward_map_applied": False,
                "straight_hold_applied": False,
                "planner_correction_applied": False,
                "startup_floor_applied": False,
                "maintenance_floor_applied": False,
            },
        }
        return float(left), float(right)

    def get_last_control_monitor(self) -> dict:
        diag = self.get_last_pid_diagnostics() or {}
        return diag.get("monitor") or {"mode": self.control_mode}

    def reset(self):
        """Reset internal state (ramping, PID integrators)."""
        self._direction_switch_hold_until = 0.0
        self._last_v_cmd_sign = 0
        self._pending_switch_sign = 0
        self._pending_switch_count = 0
        self._reset_straight_hold()
        if hasattr(self.strategy, "reset"):
            self.strategy.reset()
        self._replayer_reset_generation += 1
