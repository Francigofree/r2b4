#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Unicycle pose stabilization: target_pose (x, y, theta) → v_cmd, omega_cmd.
EKF állapot alapú zárt hurkú vezérlés; a MotionExecutor továbbra is (v, omega) → kerekek → PWM.
Bővítés: gain scheduling sebesség és állapot szerint; opcionális I tag (anti-windup + integrator_limit).
"""

import math
from typing import Tuple, Optional

from controller.motion_schema import YAW_SIGN_CONVENTION

# Célpozíció típus: (x_m, y_m, theta_rad)
TargetPose = Tuple[float, float, float]
POSE_YAW_SIGN_CONVENTION = YAW_SIGN_CONVENTION


def _wrap_angle(rad: float) -> float:
    """Szög [-pi, pi]-be csomagolása."""
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def _is_finite(value: float) -> bool:
    return math.isfinite(float(value))


def _clamp(value: float, low: float, high: float) -> float:
    if not math.isfinite(float(value)):
        return 0.0
    return max(float(low), min(float(high), float(value)))


class UnicyclePoseController:
    """
    Unicycle pose stabilizáló: robot keretben számolt pozíció- és irányhiba → v, omega.
    Bemenet: target_pose (x, y, theta), EKF állapot (x, y, theta, v).
    Kimenet: v_cmd, omega_cmd; pose_reached flag.
    Gain scheduling: alacsony v-nél k_xy csökkentés; opcionális I tag kis állandó hibákra (anti-windup).
    """

    def __init__(
        self,
        k_p_xy: float = 0.6,
        k_p_theta: float = 0.6,
        k_p_y: float = 0.2,
        v_max: float = 0.15,
        omega_max: float = 0.5,
        tolerance_xy_m: float = 0.02,
        tolerance_theta_rad: float = 0.05,
        # Gain scheduling (opcionális)
        gain_scheduling_enabled: bool = False,
        v_low_threshold: float = 0.05,
        k_xy_scale_at_low_v: float = 0.6,
        dist_rotation_threshold: float = 0.08,
        k_theta_scale_pure_rotation: float = 1.3,
        # Opcionális I tag (kis állandó hibák kompenzálása, anti-windup)
        k_i_xy: float = 0.0,
        k_i_theta: float = 0.0,
        integrator_limit_xy: float = 0.08,
        integrator_limit_theta: float = 0.15,
        rotate_first_threshold_rad: float = 0.80,
    ):
        self.k_p_xy = max(0.01, float(k_p_xy))
        self.k_p_theta = max(0.01, float(k_p_theta))
        self.k_p_y = max(0.0, float(k_p_y))
        self.v_max = max(0.01, float(v_max))
        self.omega_max = max(0.01, float(omega_max))
        self.tolerance_xy = max(0.001, float(tolerance_xy_m))
        self.tolerance_theta = max(0.001, float(tolerance_theta_rad))
        self.gain_scheduling_enabled = bool(gain_scheduling_enabled)
        self.v_low_threshold = float(v_low_threshold)
        self.k_xy_scale_at_low_v = float(k_xy_scale_at_low_v)
        self.dist_rotation_threshold = float(dist_rotation_threshold)
        self.k_theta_scale_pure_rotation = float(k_theta_scale_pure_rotation)
        self.k_i_xy = max(0.0, float(k_i_xy))
        self.k_i_theta = max(0.0, float(k_i_theta))
        self.integrator_limit_xy = max(0.0, float(integrator_limit_xy))
        self.integrator_limit_theta = max(0.0, float(integrator_limit_theta))
        self.rotate_first_threshold = max(0.0, min(math.pi, float(rotate_first_threshold_rad)))
        self._integral_xy = 0.0
        self._integral_theta = 0.0

    def set_limits(self, v_max: float, omega_max: float):
        """Központi SpeedLimits szerinti dinamikus limitfrissítés."""
        self.v_max = max(0.01, float(v_max))
        self.omega_max = max(0.01, float(omega_max))

    def _get_scheduled_gains(self, dist_xy: float, e_theta: float, v_actual: float) -> Tuple[float, float, float]:
        """Sebesség és állapot szerinti gain skálázás."""
        k_xy = self.k_p_xy
        k_theta = self.k_p_theta
        k_y = self.k_p_y
        if not self.gain_scheduling_enabled:
            return k_xy, k_theta, k_y
        if abs(v_actual) < self.v_low_threshold:
            k_xy *= self.k_xy_scale_at_low_v
        if dist_xy < self.dist_rotation_threshold and abs(e_theta) > 0.15:
            k_theta *= self.k_theta_scale_pure_rotation
        return k_xy, k_theta, k_y

    def compute(
        self,
        target_pose: TargetPose,
        ekf_state: dict,
        dt: float = 0.0,
    ) -> Tuple[float, float, bool]:
        """
        Célpozíció és EKF állapot alapján v_cmd, omega_cmd és pose_reached.

        Args:
            target_pose: (x_cél_m, y_cél_m, theta_cél_rad)
            ekf_state: EKF get_state() dict: x, y, theta (rad vagy theta_deg), v
            dt: időlépés (s), I tag integrálásához; 0 esetén I nem frissül

        Returns:
            (v_cmd, omega_cmd, pose_reached)
        """
        try:
            x_t = float(target_pose[0])
            y_t = float(target_pose[1])
            theta_t_rad = float(target_pose[2])
            px = float(ekf_state.get("x", 0.0))
            py = float(ekf_state.get("y", 0.0))
            theta = ekf_state.get("theta")
            if theta is None:
                theta = math.radians(float(ekf_state.get("theta_deg", 0.0)))
            else:
                theta = float(theta)
            v_actual = float(ekf_state.get("v", 0.0))
        except (TypeError, ValueError, IndexError):
            self._integral_xy = 0.0
            self._integral_theta = 0.0
            return 0.0, 0.0, False

        if not all(_is_finite(v) for v in (x_t, y_t, theta_t_rad, px, py, theta, v_actual)):
            self._integral_xy = 0.0
            self._integral_theta = 0.0
            return 0.0, 0.0, False

        dx = x_t - px
        dy = y_t - py
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        e_x = cos_t * dx + sin_t * dy
        e_y = -sin_t * dx + cos_t * dy
        e_theta = _wrap_angle(theta_t_rad - theta)

        dist_xy = math.sqrt(e_x * e_x + e_y * e_y)
        bearing_error = 0.0
        if dist_xy > 1e-9:
            bearing_error = _wrap_angle(math.atan2(dy, dx) - theta)
        if dist_xy < self.tolerance_xy and abs(e_theta) < self.tolerance_theta:
            self._integral_xy = 0.0
            self._integral_theta = 0.0
            return 0.0, 0.0, True

        k_xy, k_theta, k_y = self._get_scheduled_gains(dist_xy, e_theta, v_actual)

        rotate_to_bearing = bool(
            dist_xy > self.tolerance_xy * 2.0
            and abs(bearing_error) > self.rotate_first_threshold
        )
        terminal_heading_only = bool(dist_xy < self.tolerance_xy)
        pure_rotation = bool(rotate_to_bearing or terminal_heading_only)

        if rotate_to_bearing:
            v_cmd = 0.0
            omega_cmd = k_theta * bearing_error
        elif terminal_heading_only:
            v_cmd = 0.0
            omega_cmd = k_theta * e_theta
        else:
            # P + optional I: e_x drives linear motion, e_theta/e_y drives yaw.
            v_cmd = k_xy * e_x
            omega_cmd = k_theta * e_theta + k_y * e_y

        if (not pure_rotation) and self.k_i_xy > 0 and self.integrator_limit_xy > 0 and dt > 1e-6:
            self._integral_xy += e_x * dt
            self._integral_xy = max(-self.integrator_limit_xy, min(self.integrator_limit_xy, self._integral_xy))
            v_cmd += self.k_i_xy * self._integral_xy
        if (not rotate_to_bearing) and self.k_i_theta > 0 and self.integrator_limit_theta > 0 and dt > 1e-6:
            self._integral_theta += e_theta * dt
            self._integral_theta = max(-self.integrator_limit_theta, min(self.integrator_limit_theta, self._integral_theta))
            omega_cmd += self.k_i_theta * self._integral_theta

        v_cmd = _clamp(v_cmd, -self.v_max, self.v_max)
        
        if pure_rotation or abs(v_cmd) <= 1e-6:
            max_omega_stable = self.omega_max
        else:
            # V-proportional omega clamp: avoid one-track reversal during forward drive.
            max_omega_for_speed = max(0.05, abs(v_cmd) * 4.0)
            max_omega_stable = min(self.omega_max, max_omega_for_speed)
        omega_cmd = _clamp(omega_cmd, -max_omega_stable, max_omega_stable)

        # Anti-windup: saturálás esetén ne növeljük az integrátort (már clampleltük fent)
        return v_cmd, omega_cmd, False


def create_from_config(config: dict) -> UnicyclePoseController:
    """Konfig dict alapján példány: vezerles.pose_controller alatt várja a mezőket."""
    pc = config.get("pose_controller") or {}
    gs = pc.get("gain_scheduling") or {}
    return UnicyclePoseController(
        k_p_xy=float(pc.get("k_p_xy", 0.6)),
        k_p_theta=float(pc.get("k_p_theta", 0.6)),
        k_p_y=float(pc.get("k_p_y", 0.2)),
        v_max=0.15,
        omega_max=0.5,
        tolerance_xy_m=float(pc.get("tolerance_xy_m", 0.02)),
        tolerance_theta_rad=float(pc.get("tolerance_theta_rad", 0.05)),
        gain_scheduling_enabled=bool(gs.get("enabled", False)),
        v_low_threshold=float(gs.get("v_low_threshold", 0.05)),
        k_xy_scale_at_low_v=float(gs.get("k_xy_scale_at_low_v", 0.6)),
        dist_rotation_threshold=float(gs.get("dist_rotation_threshold_m", 0.08)),
        k_theta_scale_pure_rotation=float(gs.get("k_theta_scale_pure_rotation", 1.3)),
        k_i_xy=float(pc.get("k_i_xy", 0.0)),
        k_i_theta=float(pc.get("k_i_theta", 0.0)),
        integrator_limit_xy=float(pc.get("integrator_limit_xy", 0.08)),
        integrator_limit_theta=float(pc.get("integrator_limit_theta", 0.15)),
        rotate_first_threshold_rad=float(pc.get("rotate_first_threshold_rad", 0.80)),
    )
