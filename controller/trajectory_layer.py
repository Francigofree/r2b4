#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Trajectory layer: időparaméterezett pálya (waypoint lineáris interpoláció) követése.
Feedforward (v_ref, omega_ref a pályából) + feedback (EKF pose hiba) → v_cmd, omega_cmd.
MotionExecutor és SafetySupervisor/Arbiter változatlanul v_target/omega_target-ot kapnak.
Bővíthető: spline/Bezier sample(t) ugyanebbe a felületbe.
"""

import math
from typing import Tuple, List, Optional

# Waypoint: (t_sec, x_m, y_m, theta_rad) vagy (t_sec, x_m, y_m, theta_rad, v_m_s, omega_rad_s)
Waypoint = Tuple[float, ...]


def _wrap_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


class TimeParameterizedTrajectory:
    """
    Waypoint lista idő szerint; lineáris interpoláció (vagy opcionális spline).
    sample(t) → (x_ref, y_ref, theta_ref, v_ref, omega_ref).
    Ha waypointokon nincs v/omega, véges differenciából becsüljük.
    """

    def __init__(self, waypoints: List[Waypoint]):
        """
        waypoints: [(t0, x0, y0, th0), (t1, x1, y1, th1), ...] vagy
                   [(t0, x0, y0, th0, v0, om0), ...]
        t szerint növekvő sorrend.
        """
        self.waypoints = sorted(waypoints, key=lambda w: w[0])
        self._t_end = self.waypoints[-1][0] if self.waypoints else 0.0

    def duration(self) -> float:
        return self._t_end

    def sample(self, t: float) -> Tuple[float, float, float, float, float]:
        """
        t időpontban: (x_ref, y_ref, theta_ref, v_ref, omega_ref).
        t > T_end: utolsó waypoint, v_ref=0, omega_ref=0.
        """
        if not self.waypoints:
            return 0.0, 0.0, 0.0, 0.0, 0.0
        if t <= self.waypoints[0][0]:
            w = self.waypoints[0]
            x, y, th = w[1], w[2], w[3]
            v, om = (w[4], w[5]) if len(w) >= 6 else (0.0, 0.0)
            return x, y, th, v, om
        if t >= self._t_end:
            w = self.waypoints[-1]
            return w[1], w[2], w[3], 0.0, 0.0

        # Interpolate between waypoints
        for i in range(len(self.waypoints) - 1):
            w0, w1 = self.waypoints[i], self.waypoints[i + 1]
            t0, t1 = w0[0], w1[0]
            if t0 <= t <= t1:
                tau = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                x = w0[1] + tau * (w1[1] - w0[1])
                y = w0[2] + tau * (w1[2] - w0[2])
                th = w0[3] + tau * _wrap_angle(w1[3] - w0[3])
                if len(w0) >= 6 and len(w1) >= 6:
                    v = w0[4] + tau * (w1[4] - w0[4])
                    om = w0[5] + tau * (w1[5] - w0[5])
                else:
                    dt = t1 - t0
                    v = (math.hypot(w1[1] - w0[1], w1[2] - w0[2]) / dt) if dt > 1e-6 else 0.0
                    om = (_wrap_angle(w1[3] - w0[3]) / dt) if dt > 1e-6 else 0.0
                return x, y, th, v, om
        w = self.waypoints[-1]
        return w[1], w[2], w[3], 0.0, 0.0


class TrajectoryFollower:
    """
    Pálya követés: trajectory.sample(t) → ref; feedforward (v_ref, omega_ref) + feedback (pose error).
    """

    def __init__(
        self,
        k_p_x: float = 0.5,
        k_p_y: float = 0.3,
        k_p_theta: float = 1.0,
        ff_scale_v: float = 1.0,
        ff_scale_omega: float = 1.0,
        v_max: float = 10.0,
        omega_max: float = 10.0,
    ):
        self.k_p_x = float(k_p_x)
        self.k_p_y = float(k_p_y)
        self.k_p_theta = float(k_p_theta)
        self.ff_scale_v = float(ff_scale_v)
        self.ff_scale_omega = float(ff_scale_omega)
        self.v_max = max(0.01, float(v_max))
        self.omega_max = max(0.01, float(omega_max))
        self._trajectory: Optional[TimeParameterizedTrajectory] = None
        self._t_start: Optional[float] = None

    def set_limits(self, v_max: float, omega_max: float):
        """Központi SpeedLimits szerinti dinamikus limitfrissítés."""
        self.v_max = max(0.01, float(v_max))
        self.omega_max = max(0.01, float(omega_max))

    def set_trajectory(self, trajectory: TimeParameterizedTrajectory, t_start: float = 0.0):
        self._trajectory = trajectory
        self._t_start = t_start

    def clear_trajectory(self):
        self._trajectory = None
        self._t_start = None

    def has_trajectory(self) -> bool:
        return self._trajectory is not None

    def compute(
        self,
        t_now: float,
        ekf_state: dict,
    ) -> Tuple[float, float, bool]:
        """
        t_now: aktuális idő (pl. monotonic - t_start).
        Vissza: (v_cmd, omega_cmd, trajectory_finished).
        """
        if self._trajectory is None or self._t_start is None:
            return 0.0, 0.0, True
        t = t_now - self._t_start
        if t < 0:
            return 0.0, 0.0, False
        x_ref, y_ref, theta_ref, v_ref, omega_ref = self._trajectory.sample(t)
        if t >= self._trajectory.duration():
            self.clear_trajectory()
            return 0.0, 0.0, True

        px = float(ekf_state.get("x", 0.0))
        py = float(ekf_state.get("y", 0.0))
        theta = ekf_state.get("theta")
        if theta is None:
            theta = math.radians(float(ekf_state.get("theta_deg", 0.0)))
        else:
            theta = float(theta)

        dx = x_ref - px
        dy = y_ref - py
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        e_x = cos_t * dx + sin_t * dy
        e_y = -sin_t * dx + cos_t * dy
        e_theta = _wrap_angle(theta_ref - theta)

        v_cmd = v_ref * self.ff_scale_v + self.k_p_x * e_x
        omega_cmd = omega_ref * self.ff_scale_omega + self.k_p_theta * e_theta + self.k_p_y * e_y
        v_cmd = max(-self.v_max, min(self.v_max, v_cmd))
        omega_cmd = max(-self.omega_max, min(self.omega_max, omega_cmd))
        return v_cmd, omega_cmd, False


def create_trajectory_follower_from_config(config: dict) -> TrajectoryFollower:
    """Konfig: motion_execution.trajectory_follower."""
    c = config.get("trajectory_follower") or {}
    return TrajectoryFollower(
        k_p_x=float(c.get("k_p_x", 0.5)),
        k_p_y=float(c.get("k_p_y", 0.3)),
        k_p_theta=float(c.get("k_p_theta", 1.0)),
        ff_scale_v=float(c.get("ff_scale_v", 1.0)),
        ff_scale_omega=float(c.get("ff_scale_omega", 1.0)),
        v_max=10.0,
        omega_max=10.0,
    )
