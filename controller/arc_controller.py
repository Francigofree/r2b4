#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Arc motion primitive controller.

ARC V2 chain:
- EKF geometric outer loop (radius/heading/chord progress aware)
- IMU yaw-rate inner loop (omega tracking)
- diff-drive physical contract enforcement for normal ARC
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, Optional, Tuple

from controller.motion_schema import YAW_SIGN_CONVENTION


def _wrap_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


class ArcController:
    """
    Follow a circular arc defined by (radius, arc_angle, speed).

    Inputs at start():
        radius_m      – turning radius (always positive)
        arc_angle_rad – total heading change (signed, math convention:
                        positive = CCW / left, negative = CW / right)
        speed_mps     – linear speed (always positive)

    Each tick() returns (v_cmd, omega_cmd, done, status_dict).

    Runtime convention used by this primitive:
        EKF heading increases on left turns (CCW-positive)
        positive omega_target commands left turn
        +Y axis is left in body frame (standard robotics convention)
    """

    # Tuned gains
    K_HEADING = 1.0       # omega correction per heading error
    K_LATERAL = 0.5       # omega correction per lateral error
    MAX_CORRECTION = 0.3  # max outer correction added to omega (rad/s)
    K_OMEGA_INNER = 0.70
    MAX_OMEGA_INNER_CORR = 0.35
    OUTER_LOOP_TIME_CONSTANT_S = 0.22
    TRACK_WIDTH_M = 0.175
    NORMAL_ARC_INNER_MIN_MPS = 0.02
    ARC_TRACK_DIFF_MIN_MPS = 0.003
    ARC_PIVOT_LIKE_TRACK_EPS_MPS = 0.004
    GENTLE_CURVATURE_THRESHOLD_M_INV = 2.8
    GENTLE_CURVATURE_CAP_MARGIN = 0.96
    SHARP_CURVATURE_FLOOR_MARGIN = 1.005
    SHARP_HEADING_GAIN_MULTIPLIER = 1.10
    SHARP_LATERAL_GAIN_MULTIPLIER = 1.35
    LATERAL_SLOWDOWN_START_M = 0.04
    LATERAL_SLOWDOWN_FULL_M = 0.18
    LATERAL_SLOWDOWN_MIN_SCALE = 0.55

    def __init__(
        self,
        *,
        k_heading: float = K_HEADING,
        k_lateral: float = K_LATERAL,
        max_correction: float = MAX_CORRECTION,
        k_omega_inner: float = K_OMEGA_INNER,
        max_omega_inner_correction: float = MAX_OMEGA_INNER_CORR,
        outer_loop_time_constant_s: float = OUTER_LOOP_TIME_CONSTANT_S,
        track_width_m: float = TRACK_WIDTH_M,
        normal_arc_inner_min_mps: float = NORMAL_ARC_INNER_MIN_MPS,
        arc_track_diff_min_mps: float = ARC_TRACK_DIFF_MIN_MPS,
        arc_pivot_like_track_eps_mps: float = ARC_PIVOT_LIKE_TRACK_EPS_MPS,
        gentle_curvature_threshold_m_inv: float = GENTLE_CURVATURE_THRESHOLD_M_INV,
        gentle_curvature_cap_margin: float = GENTLE_CURVATURE_CAP_MARGIN,
        sharp_curvature_floor_margin: float = SHARP_CURVATURE_FLOOR_MARGIN,
        sharp_heading_gain_multiplier: float = SHARP_HEADING_GAIN_MULTIPLIER,
        sharp_lateral_gain_multiplier: float = SHARP_LATERAL_GAIN_MULTIPLIER,
        lateral_slowdown_start_m: float = LATERAL_SLOWDOWN_START_M,
        lateral_slowdown_full_m: float = LATERAL_SLOWDOWN_FULL_M,
        lateral_slowdown_min_scale: float = LATERAL_SLOWDOWN_MIN_SCALE,
    ):
        self.k_heading = max(0.0, float(k_heading))
        self.k_lateral = max(0.0, float(k_lateral))
        self.max_correction = max(0.0, float(max_correction))
        self.k_omega_inner = max(0.0, float(k_omega_inner))
        self.max_omega_inner_correction = max(0.0, float(max_omega_inner_correction))
        self.outer_loop_time_constant_s = max(0.02, float(outer_loop_time_constant_s))
        self.track_width_m = max(0.01, float(track_width_m))
        self.normal_arc_inner_min_mps = max(0.0, float(normal_arc_inner_min_mps))
        self.arc_track_diff_min_mps = max(0.0, float(arc_track_diff_min_mps))
        self.arc_pivot_like_track_eps_mps = max(0.0, float(arc_pivot_like_track_eps_mps))
        self.gentle_curvature_threshold_m_inv = max(0.1, float(gentle_curvature_threshold_m_inv))
        self.gentle_curvature_cap_margin = max(
            0.1,
            min(1.0, float(gentle_curvature_cap_margin)),
        )
        self.sharp_curvature_floor_margin = max(
            1.0,
            float(sharp_curvature_floor_margin),
        )
        self.sharp_heading_gain_multiplier = max(1.0, float(sharp_heading_gain_multiplier))
        self.sharp_lateral_gain_multiplier = max(1.0, float(sharp_lateral_gain_multiplier))
        self.lateral_slowdown_start_m = max(0.0, float(lateral_slowdown_start_m))
        self.lateral_slowdown_full_m = max(
            float(self.lateral_slowdown_start_m) + 1e-6,
            float(lateral_slowdown_full_m),
        )
        self.lateral_slowdown_min_scale = max(
            0.10,
            min(1.0, float(lateral_slowdown_min_scale)),
        )
        self._active = False
        self._reset_state()

    def _reset_state(self) -> None:
        self._radius_m = 0.0
        self._arc_angle_rad = 0.0  # math convention (positive = left)
        self._arc_angle_input_rad = 0.0
        self._speed_mps = 0.0
        self._curvature_base_m_inv = 0.0
        self._curvature_outer_m_inv = 0.0
        self._omega_base = 0.0
        self._start_heading_rad = 0.0
        self._target_heading_rad = 0.0
        self._start_x = 0.0
        self._start_y = 0.0
        self._center_x = 0.0
        self._center_y = 0.0
        self._start_center_angle_rad = 0.0
        self._integrated_angle_rad = 0.0
        self._progress_abs_rad = 0.0
        self._gyro_to_pose_sign: Optional[float] = None
        self._prev_heading_rad: Optional[float] = None
        self._start_mono = 0.0
        self._max_duration_s = 30.0
        self._last_terminal: Dict[str, Any] = {}
        self._arc_sample_count = 0
        self._arc_pivot_like_samples = 0
        self._arc_inner_positive_samples = 0
        self._arc_inner_track_min_mps = math.inf
        self._arc_track_ratio_last = 1.0
        self._arc_track_ratio_peak = 1.0
        self._arc_last_contract: Dict[str, Any] = {}

    @property
    def active(self) -> bool:
        return self._active

    def start(
        self,
        *,
        radius_m: float,
        arc_angle_rad: float,
        speed_mps: float,
        ekf_state: Dict[str, Any],
        max_duration_s: float = 30.0,
    ) -> bool:
        radius_m = abs(float(radius_m))
        if radius_m < 0.01 or abs(float(arc_angle_rad)) < 1e-4 or float(speed_mps) <= 0.0:
            return False

        self._active = True
        self._radius_m = radius_m
        self._speed_mps = float(speed_mps)

        # Math convention is runtime convention too (positive = left).
        self._arc_angle_input_rad = float(arc_angle_rad)
        self._arc_angle_rad = float(arc_angle_rad)

        # Pure diff-drive curvature model: kappa = 1/R, omega = v * kappa.
        # Sign is set by requested arc direction (left/CCW = positive).
        self._curvature_base_m_inv = math.copysign(1.0 / self._radius_m, self._arc_angle_rad)
        self._curvature_outer_m_inv = float(self._curvature_base_m_inv)
        self._omega_base = self._speed_mps * self._curvature_base_m_inv

        # Capture start pose from EKF.
        theta = ekf_state.get("theta")
        if theta is None:
            theta = math.radians(_safe_float(ekf_state.get("theta_deg"), 0.0))
        else:
            theta = float(theta)
        self._start_heading_rad = theta
        self._target_heading_rad = theta + self._arc_angle_rad
        self._start_x = _safe_float(ekf_state.get("x"), 0.0)
        self._start_y = _safe_float(ekf_state.get("y"), 0.0)

        # Arc center (perpendicular to heading, offset by R):
        # left turn -> +pi/2, right turn -> -pi/2.
        turn_sign = 1.0 if self._arc_angle_rad >= 0 else -1.0
        self._center_x = self._start_x + self._radius_m * math.cos(theta + turn_sign * math.pi / 2.0)
        self._center_y = self._start_y + self._radius_m * math.sin(theta + turn_sign * math.pi / 2.0)
        self._start_center_angle_rad = math.atan2(
            self._start_y - self._center_y,
            self._start_x - self._center_x,
        )

        self._integrated_angle_rad = 0.0
        self._progress_abs_rad = 0.0
        self._gyro_to_pose_sign = None
        self._prev_heading_rad = theta
        self._start_mono = time.monotonic()
        self._max_duration_s = max(1.0, float(max_duration_s))
        self._last_terminal = {}
        self._arc_sample_count = 0
        self._arc_pivot_like_samples = 0
        self._arc_inner_positive_samples = 0
        self._arc_inner_track_min_mps = math.inf
        self._arc_track_ratio_last = 1.0
        self._arc_track_ratio_peak = 1.0
        self._arc_last_contract = {}
        return True

    def cancel(self) -> None:
        elapsed = max(0.0, time.monotonic() - float(self._start_mono or time.monotonic()))
        progress_frac = (
            float(self._progress_abs_rad) / max(1e-6, abs(float(self._arc_angle_rad)))
            if abs(float(self._arc_angle_rad)) > 1e-9
            else 0.0
        )
        terminal = {
            "reason": "cancelled",
            "progress_frac": round(float(progress_frac), 4),
            "elapsed_s": round(float(elapsed), 3),
        }
        self._active = False
        self._reset_state()
        self._last_terminal = dict(terminal)

    def tick(
        self,
        ekf_state: Dict[str, Any],
        dt: float,
        *,
        gyro_z_rad_s: Optional[float] = None,
    ) -> Tuple[float, float, bool, Dict[str, Any]]:
        """
        Returns (v_cmd, omega_cmd, done, status).
        """
        if not self._active:
            return 0.0, 0.0, True, {"reason": "not_active", "last_terminal": dict(self._last_terminal or {})}

        # Read current pose from EKF
        theta = ekf_state.get("theta")
        if theta is None:
            theta = math.radians(_safe_float(ekf_state.get("theta_deg"), 0.0))
        else:
            theta = float(theta)
        px = _safe_float(ekf_state.get("x"), 0.0)
        py = _safe_float(ekf_state.get("y"), 0.0)

        # EKF heading integration is kept for diagnostics.
        d_theta_pose = 0.0
        if self._prev_heading_rad is not None and dt > 0:
            d_theta_pose = _wrap_angle(theta - self._prev_heading_rad)
            self._integrated_angle_rad += d_theta_pose
        self._prev_heading_rad = theta

        # Progress is IMU-first: fast omega integration, EKF theta as fallback.
        # Only expected-direction progress is accumulated.
        arc_sign = 1.0 if self._arc_angle_rad >= 0 else -1.0
        omega_measured = _safe_float(gyro_z_rad_s, math.nan)
        if not math.isfinite(omega_measured):
            omega_measured = _safe_float(ekf_state.get("omega_rad_s"), math.nan)

        # Runtime convention calibration:
        # align IMU yaw-rate sign to EKF heading-delta sign once we have
        # a reliable pair in the same tick. This avoids hard-coding platform
        # sign assumptions while still keeping IMU as fast progress source.
        if (
            self._gyro_to_pose_sign is None
            and math.isfinite(omega_measured)
            and abs(float(omega_measured)) >= 0.05
            and abs(float(d_theta_pose)) >= 0.0008
        ):
            self._gyro_to_pose_sign = (
                1.0 if (float(omega_measured) * float(d_theta_pose)) >= 0.0 else -1.0
            )

        omega_for_progress = omega_measured
        if math.isfinite(omega_for_progress) and self._gyro_to_pose_sign is not None:
            omega_for_progress = float(omega_for_progress) * float(self._gyro_to_pose_sign)

        if math.isfinite(omega_measured):
            d_theta_progress = float(omega_for_progress) * max(0.0, float(dt))
        else:
            d_theta_progress = float(d_theta_pose)

        max_step = max(0.02, abs(float(self._omega_base)) * max(0.0, float(dt)) * 4.0 + 0.05)
        d_theta_progress = max(-max_step, min(max_step, float(d_theta_progress)))
        d_expected = arc_sign * float(d_theta_progress)
        if d_expected > 0.0:
            self._progress_abs_rad += float(d_expected)

        # Geometric progress around the arc center suppresses false completion from
        # near in-place yaw motion with little translation.
        center_angle_now = math.atan2(py - self._center_y, px - self._center_x)
        center_delta = _wrap_angle(center_angle_now - self._start_center_angle_rad)
        geom_progress_abs_rad = max(0.0, arc_sign * float(center_delta))
        abs_arc_rad = max(1e-6, abs(float(self._arc_angle_rad)))
        completion_geom_abs_rad = min(float(self._progress_abs_rad), float(geom_progress_abs_rad))

        progress_frac = float(self._progress_abs_rad) / float(abs_arc_rad)
        progress_frac = max(0.0, min(1.5, float(progress_frac)))
        completion_geom_frac = float(completion_geom_abs_rad) / float(abs_arc_rad)
        completion_geom_frac = max(0.0, min(1.5, float(completion_geom_frac)))

        # IMU/chord fallback:
        # keep geometric guard as primary source, but if translation chord progress
        # is substantial, allow bounded IMU completion to prevent late stop on
        # noisy center-angle geometry.
        chord_dx = float(px - self._start_x)
        chord_dy = float(py - self._start_y)
        travel_chord_m = math.sqrt(chord_dx * chord_dx + chord_dy * chord_dy)
        expected_chord_m = max(1e-6, 2.0 * float(self._radius_m) * math.sin(0.5 * float(abs_arc_rad)))
        chord_progress_frac = max(0.0, min(1.5, float(travel_chord_m) / float(expected_chord_m)))
        completion_fallback_frac = 0.0
        completion_progress_source = "imu_geom_min"
        if chord_progress_frac >= 0.45 and progress_frac >= completion_geom_frac:
            completion_fallback_frac = max(0.0, min(1.5, float(progress_frac) * 0.96))

        completion_frac = max(float(completion_geom_frac), float(completion_fallback_frac))
        completion_frac = max(0.0, min(1.5, float(completion_frac)))
        if completion_fallback_frac > completion_geom_frac:
            completion_progress_source = "imu_chord_blend"
        completion_progress_abs_rad = float(completion_frac) * float(abs_arc_rad)
        done = completion_frac >= 1.0

        # Timeout guard
        elapsed = time.monotonic() - self._start_mono
        if elapsed >= self._max_duration_s:
            terminal = {
                "reason": "timeout",
                "progress_frac": round(float(progress_frac), 4),
                "completion_progress_frac": round(float(completion_frac), 4),
                "completion_progress_source": str(completion_progress_source),
                "elapsed_s": round(float(elapsed), 3),
            }
            self._last_terminal = dict(terminal)
            self._active = False
            return 0.0, 0.0, True, dict(terminal)

        if done:
            terminal = {
                "reason": "arc_completed",
                "progress_frac": round(float(progress_frac), 4),
                "completion_progress_frac": round(float(completion_frac), 4),
                "completion_progress_source": str(completion_progress_source),
                "final_heading_error_rad": round(_wrap_angle(self._target_heading_rad - theta), 5),
                "elapsed_s": round(float(elapsed), 3),
            }
            self._last_terminal = dict(terminal)
            self._active = False
            return 0.0, 0.0, True, dict(terminal)

        # --- Outer loop (EKF geometry) ---
        # Expected heading at current progress.
        expected_heading = self._start_heading_rad + self._arc_angle_rad * progress_frac
        heading_error = _wrap_angle(expected_heading - theta)

        # Radius/lateral error from arc center.
        dx_c = px - self._center_x
        dy_c = py - self._center_y
        dist_to_center = math.sqrt(dx_c * dx_c + dy_c * dy_c)
        turn_sign = 1.0 if self._arc_angle_rad >= 0 else -1.0
        radius_error_m = float(dist_to_center - self._radius_m)
        lateral_error = float(radius_error_m * turn_sign)

        correction_heading_gain = float(self.k_heading)
        correction_lateral_gain = float(self.k_lateral)
        if abs(float(self._curvature_base_m_inv)) > float(self.gentle_curvature_threshold_m_inv):
            correction_heading_gain *= float(self.sharp_heading_gain_multiplier)
            correction_lateral_gain *= float(self.sharp_lateral_gain_multiplier)
        outer_omega_correction = correction_heading_gain * heading_error + correction_lateral_gain * lateral_error
        outer_omega_correction = max(-self.max_correction, min(self.max_correction, outer_omega_correction))

        target_curvature_outer = float(self._curvature_base_m_inv)
        if abs(float(self._speed_mps)) > 1e-6:
            target_curvature_outer += float(outer_omega_correction) / max(0.02, abs(float(self._speed_mps)))

        alpha = max(0.0, min(1.0, float(dt) / max(1e-3, float(self.outer_loop_time_constant_s))))
        self._curvature_outer_m_inv = float(self._curvature_outer_m_inv) + alpha * (
            float(target_curvature_outer) - float(self._curvature_outer_m_inv)
        )

        # Deceleration ramp: adaptive start based on arc length.
        # Short arcs (<45°) need earlier decel to avoid overshoot.
        abs_arc_deg = abs(math.degrees(self._arc_angle_rad))
        if abs_arc_deg < 45.0:
            DECEL_START = 0.65  # start decel earlier for short arcs
        else:
            DECEL_START = 0.78  # stronger end-positioning slowdown for live traction
        if progress_frac > DECEL_START:
            ramp = max(0.18, (1.0 - progress_frac) / (1.0 - DECEL_START))
        else:
            ramp = 1.0

        v_cmd = self._speed_mps * ramp
        omega_outer_target = float(v_cmd) * float(self._curvature_outer_m_inv)
        arc_direction_clamp_applied = False

        lateral_speed_scale = 1.0
        lateral_abs = abs(float(lateral_error))
        if lateral_abs > float(self.lateral_slowdown_start_m):
            ratio = (lateral_abs - float(self.lateral_slowdown_start_m)) / max(
                1e-6,
                float(self.lateral_slowdown_full_m - self.lateral_slowdown_start_m),
            )
            ratio = max(0.0, min(1.0, float(ratio)))
            lateral_speed_scale = 1.0 - (ratio * (1.0 - float(self.lateral_slowdown_min_scale)))
            lateral_speed_scale = max(float(self.lateral_slowdown_min_scale), min(1.0, float(lateral_speed_scale)))
            v_cmd *= float(lateral_speed_scale)
            omega_outer_target = float(v_cmd) * float(self._curvature_outer_m_inv)

        # Preserve gentle/sharp semantic boundaries on outer omega target.
        gentle_curvature_cap_applied = False
        sharp_curvature_floor_applied = False
        if abs(float(self._curvature_base_m_inv)) <= float(self.gentle_curvature_threshold_m_inv):
            curvature_cap = float(self.gentle_curvature_threshold_m_inv) * float(self.gentle_curvature_cap_margin)
            omega_cap = abs(float(v_cmd)) * float(curvature_cap)
            if omega_cap > 0.0 and abs(float(omega_outer_target)) > float(omega_cap):
                omega_outer_target = math.copysign(float(omega_cap), float(self._omega_base))
                gentle_curvature_cap_applied = True
        else:
            curvature_floor = float(self.gentle_curvature_threshold_m_inv) * float(self.sharp_curvature_floor_margin)
            omega_floor = abs(float(v_cmd)) * float(curvature_floor)
            if omega_floor > 0.0 and abs(float(omega_outer_target)) < float(omega_floor):
                omega_outer_target = math.copysign(float(omega_floor), float(self._omega_base))
                sharp_curvature_floor_applied = True

        # --- Inner loop (IMU yaw-rate tracking) ---
        omega_feedback_rad_s = float(omega_for_progress) if math.isfinite(float(omega_for_progress)) else (
            float(d_theta_pose) / max(1e-3, float(dt)) if float(dt) > 1e-6 else 0.0
        )
        omega_error = float(omega_outer_target) - float(omega_feedback_rad_s)
        omega_inner_correction = _safe_float(self.k_omega_inner, 0.0) * float(omega_error)
        omega_inner_correction = max(
            -float(self.max_omega_inner_correction),
            min(float(self.max_omega_inner_correction), float(omega_inner_correction)),
        )
        omega_cmd = float(omega_outer_target) + float(omega_inner_correction)

        # Keep commanded turn direction consistent with requested arc direction.
        if abs(float(self._omega_base)) > 1e-9 and (float(omega_cmd) * float(self._omega_base)) < 0.0:
            omega_cmd = math.copysign(abs(float(omega_cmd)), float(self._omega_base))
            arc_direction_clamp_applied = True

        # M6 contract: follow_arc must remain physically executable
        # (both tracks positive and non-equal) unless an explicit pivot mode
        # is commanded. ArcController has no explicit pivot mode, therefore the
        # track-positivity contract is enforced for the whole ARC execution.
        normal_arc_semantic_mode = (
            abs(float(self._curvature_base_m_inv)) <= float(self.gentle_curvature_threshold_m_inv)
        )
        v_cmd, omega_cmd, contract_diag = self._apply_arc_track_contract(
            v_cmd=float(v_cmd),
            omega_cmd=float(omega_cmd),
            normal_arc=True,
        )

        self._arc_sample_count += 1
        if bool(contract_diag.get("pivot_like", False)):
            self._arc_pivot_like_samples += 1
        if bool(contract_diag.get("inner_positive", False)):
            self._arc_inner_positive_samples += 1
        self._arc_inner_track_min_mps = min(
            float(self._arc_inner_track_min_mps),
            float(contract_diag.get("inner_track_abs_mps", 0.0)),
        )
        self._arc_track_ratio_last = float(contract_diag.get("track_ratio", 1.0))
        self._arc_track_ratio_peak = max(float(self._arc_track_ratio_peak), float(self._arc_track_ratio_last))
        self._arc_last_contract = dict(contract_diag)

        endpoint_pose = self._compute_endpoint_pose()
        endpoint_position_error_m = math.sqrt(
            (float(px) - float(endpoint_pose["x"])) ** 2
            + (float(py) - float(endpoint_pose["y"])) ** 2
        )
        endpoint_heading_error_deg = math.degrees(_wrap_angle(float(endpoint_pose["theta"]) - float(theta)))

        status = {
            "reason": "tracking",
            "controller_version": "ARC_CONTROLLER_V2",
            "yaw_sign_convention": YAW_SIGN_CONVENTION,
            "kinematics_model": "DIFF_DRIVE_CURVATURE_SSOT",
            "control_feedback_basis": "EKF_OUTER_PLUS_IMU_INNER",
            "progress_frac": round(progress_frac, 4),
            "completion_progress_frac": round(completion_frac, 4),
            "completion_geom_progress_frac": round(float(completion_geom_frac), 4),
            "completion_fallback_progress_frac": round(float(completion_fallback_frac), 4),
            "completion_progress_source": str(completion_progress_source),
            "progress_abs_deg": round(math.degrees(float(self._progress_abs_rad)), 3),
            "geometric_progress_abs_deg": round(math.degrees(float(geom_progress_abs_rad)), 3),
            "chord_progress_frac": round(float(chord_progress_frac), 4),
            "integrated_angle_deg": round(math.degrees(self._integrated_angle_rad), 3),
            "target_angle_deg": round(math.degrees(self._arc_angle_input_rad), 3),
            "heading_error_deg": round(math.degrees(heading_error), 3),
            "lateral_error_m": round(lateral_error, 4),
            "radius_error_m": round(float(radius_error_m), 4),
            "arc_center_x_m": round(float(self._center_x), 4),
            "arc_center_y_m": round(float(self._center_y), 4),
            "endpoint_position_error_m": round(float(endpoint_position_error_m), 4),
            "endpoint_heading_error_deg": round(float(endpoint_heading_error_deg), 3),
            "correction_heading_gain": round(float(correction_heading_gain), 3),
            "correction_lateral_gain": round(float(correction_lateral_gain), 3),
            "lateral_speed_scale": round(float(lateral_speed_scale), 3),
            "curvature_target_m_inv": round(float(self._curvature_base_m_inv), 4),
            "curvature_outer_target_m_inv": round(float(self._curvature_outer_m_inv), 4),
            "curvature_cap_m_inv": (
                round(
                    float(self.gentle_curvature_threshold_m_inv)
                    * float(self.gentle_curvature_cap_margin),
                    4,
                )
                if abs(float(self._curvature_base_m_inv)) <= float(self.gentle_curvature_threshold_m_inv)
                else None
            ),
            "curvature_floor_m_inv": (
                round(
                    float(self.gentle_curvature_threshold_m_inv)
                    * float(self.sharp_curvature_floor_margin),
                    4,
                )
                if abs(float(self._curvature_base_m_inv)) > float(self.gentle_curvature_threshold_m_inv)
                else None
            ),
            "gentle_curvature_cap_applied": bool(gentle_curvature_cap_applied),
            "sharp_curvature_floor_applied": bool(sharp_curvature_floor_applied),
            "arc_direction_clamp_applied": bool(arc_direction_clamp_applied),
            "omega_base": round(self._omega_base, 4),
            "omega_outer_correction": round(float(outer_omega_correction), 4),
            "omega_target_outer_rad_s": round(float(omega_outer_target), 4),
            "omega_feedback_rad_s": round(float(omega_feedback_rad_s), 4),
            "omega_error_rad_s": round(float(omega_error), 4),
            "omega_inner_correction_rad_s": round(float(omega_inner_correction), 4),
            "omega_cmd": round(omega_cmd, 4),
            "omega_progress_source": ("gyro_z" if math.isfinite(_safe_float(gyro_z_rad_s, math.nan)) else ("ekf_omega" if math.isfinite(_safe_float(ekf_state.get("omega_rad_s"), math.nan)) else "pose_delta")),
            "omega_progress_rad_s": (None if not math.isfinite(omega_measured) else round(float(omega_measured), 4)),
            "omega_progress_aligned_rad_s": (None if not math.isfinite(omega_for_progress) else round(float(omega_for_progress), 4)),
            "gyro_to_pose_sign": (
                None if self._gyro_to_pose_sign is None else int(1 if self._gyro_to_pose_sign >= 0 else -1)
            ),
            "left_track_mps": round(float(contract_diag.get("left_track_mps", 0.0)), 4),
            "right_track_mps": round(float(contract_diag.get("right_track_mps", 0.0)), 4),
            "arc_contract_normal_mode": bool(normal_arc_semantic_mode),
            "arc_contract_applied": bool(contract_diag.get("contract_applied", False)),
            "arc_inner_track_positive": bool(contract_diag.get("inner_positive", False)),
            "arc_inner_track_min_mps": round(float(self._arc_inner_track_min_mps), 4),
            "arc_track_ratio": round(float(self._arc_track_ratio_last), 4),
            "arc_track_ratio_peak": round(float(self._arc_track_ratio_peak), 4),
            "arc_pivot_like": bool(contract_diag.get("pivot_like", False)),
            "arc_pivot_like_samples": int(self._arc_pivot_like_samples),
            "arc_inner_track_positive_ratio": round(
                float(self._arc_inner_positive_samples) / max(1, int(self._arc_sample_count)),
                4,
            ),
            "arc_sample_count": int(self._arc_sample_count),
            "elapsed_s": round(elapsed, 3),
        }
        return v_cmd, omega_cmd, False, status

    def status(self) -> Dict[str, Any]:
        progress = float(self._progress_abs_rad) / max(1e-6, abs(float(self._arc_angle_rad))) if abs(float(self._arc_angle_rad)) > 1e-9 else 0.0
        if (not self._active) and isinstance(self._last_terminal, dict) and self._last_terminal.get("progress_frac") is not None:
            progress = float(_safe_float(self._last_terminal.get("progress_frac"), progress))
        return {
            "active": self._active,
            "controller_version": "ARC_CONTROLLER_V2",
            "yaw_sign_convention": YAW_SIGN_CONVENTION,
            "radius_m": round(self._radius_m, 4),
            "kinematics_model": "DIFF_DRIVE_CURVATURE_SSOT",
            "control_feedback_basis": "EKF_OUTER_PLUS_IMU_INNER",
            "curvature_target_m_inv": round(float(self._curvature_base_m_inv), 4),
            "curvature_outer_target_m_inv": round(float(self._curvature_outer_m_inv), 4),
            "omega_target_rad_s": round(float(self._omega_base), 4),
            "arc_angle_deg": round(math.degrees(self._arc_angle_rad), 3),
            "speed_mps": round(self._speed_mps, 4),
            "integrated_angle_deg": round(math.degrees(self._integrated_angle_rad), 3),
            "progress_abs_deg": round(math.degrees(self._progress_abs_rad), 3),
            "progress_frac": round(float(progress), 4),
            "arc_inner_track_min_mps": round(float(self._arc_inner_track_min_mps), 4),
            "arc_track_ratio": round(float(self._arc_track_ratio_last), 4),
            "arc_track_ratio_peak": round(float(self._arc_track_ratio_peak), 4),
            "arc_pivot_like_samples": int(self._arc_pivot_like_samples),
            "arc_inner_track_positive_ratio": round(
                float(self._arc_inner_positive_samples) / max(1, int(self._arc_sample_count)),
                4,
            ),
            "arc_sample_count": int(self._arc_sample_count),
            "left_track_mps": _safe_float(self._arc_last_contract.get("left_track_mps"), 0.0),
            "right_track_mps": _safe_float(self._arc_last_contract.get("right_track_mps"), 0.0),
            "last_terminal": dict(self._last_terminal or {}),
        }

    def _track_from_twist(self, v_mps: float, omega_rad_s: float) -> Tuple[float, float]:
        half_track = 0.5 * float(self.track_width_m)
        left = float(v_mps) - float(omega_rad_s) * float(half_track)
        right = float(v_mps) + float(omega_rad_s) * float(half_track)
        return float(left), float(right)

    def _twist_from_track(self, left_mps: float, right_mps: float) -> Tuple[float, float]:
        v = 0.5 * (float(left_mps) + float(right_mps))
        omega = (float(right_mps) - float(left_mps)) / max(0.01, float(self.track_width_m))
        return float(v), float(omega)

    def _compute_endpoint_pose(self) -> Dict[str, float]:
        x_end = float(self._center_x) + float(self._radius_m) * math.cos(
            float(self._start_center_angle_rad) + float(self._arc_angle_rad)
        )
        y_end = float(self._center_y) + float(self._radius_m) * math.sin(
            float(self._start_center_angle_rad) + float(self._arc_angle_rad)
        )
        return {
            "x": float(x_end),
            "y": float(y_end),
            "theta": float(self._target_heading_rad),
        }

    def _apply_arc_track_contract(
        self,
        *,
        v_cmd: float,
        omega_cmd: float,
        normal_arc: bool,
    ) -> Tuple[float, float, Dict[str, Any]]:
        left, right = self._track_from_twist(float(v_cmd), float(omega_cmd))
        turn_left = bool((float(self._omega_base) if abs(float(self._omega_base)) > 1e-9 else float(omega_cmd)) >= 0.0)
        inner = float(left if turn_left else right)
        outer = float(right if turn_left else left)

        contract_applied = False
        if bool(normal_arc):
            inner_floor = float(self.normal_arc_inner_min_mps)
            if inner < inner_floor:
                inner = float(inner_floor)
                contract_applied = True
            if outer <= (inner + float(self.arc_track_diff_min_mps)):
                outer = float(inner + float(self.arc_track_diff_min_mps))
                contract_applied = True
            if turn_left:
                left, right = float(inner), float(outer)
            else:
                left, right = float(outer), float(inner)
            v_cmd, omega_cmd = self._twist_from_track(left, right)

        inner_abs = abs(float(left if turn_left else right))
        outer_abs = abs(float(right if turn_left else left))
        ratio = float(outer_abs / max(1e-6, inner_abs))
        pivot_like = bool(
            inner_abs <= float(self.arc_pivot_like_track_eps_mps)
            or (float(left) * float(right)) <= 0.0
        )
        diag = {
            "contract_applied": bool(contract_applied),
            "normal_arc": bool(normal_arc),
            "left_track_mps": float(left),
            "right_track_mps": float(right),
            "inner_track_abs_mps": float(inner_abs),
            "outer_track_abs_mps": float(outer_abs),
            "track_ratio": float(ratio),
            "inner_positive": bool(float(left) > 0.0 and float(right) > 0.0 and abs(float(left) - float(right)) > 1e-9),
            "pivot_like": bool(pivot_like),
        }
        return float(v_cmd), float(omega_cmd), diag
