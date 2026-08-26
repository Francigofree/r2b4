#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Sebesség és PWM táblázatok kezelése.
Kiszámolja a PWM szinteket a konfig fájlok alapján.
"""

def _build_deadzone_pwm_levels(dz_min: float) -> dict:
    """
    Deadzone-tudatos PWM fokozatok (0-9):
      0 = 0.0 (álló)
      1 = dz_min (deadzone határ — épp elindul)
      2..7 = lineáris lépcsők dz_min → 1.0 (7 egyenlő lépés)
      8, 9 = 1.0 (maximális PWM)
    """
    dz = max(0.0, min(1.0, float(dz_min)))
    levels = {0: 0.0}
    for i in range(1, 8):
        levels[i] = round(dz + (1.0 - dz) * (i - 1) / 7.0, 4)
    levels[8] = 1.0
    levels[9] = 1.0
    return levels


def build_speed_tables(ctrl):
    """Sebesség szintek létrehozása a speed_map.json alapján (0-9)."""
    speed_map = ctrl.cfg.get("speed_map", {}) or {}
    ctrl._speed_map_ref = speed_map

    # Deadzone-tudatos belső szintskála a fizikai v/omega táblákhoz.
    dz_min = getattr(ctrl, "drive_pid_cfg", None)
    dz_val = float(getattr(dz_min, "dz_min", 0.0)) if dz_min else 0.0
    ctrl.speed_pwm_levels = _build_deadzone_pwm_levels(dz_val)

    curves = dict(speed_map.get("curves") or {})
    ctrl.speeds_fwd = map_curves_to_speed_levels(
        ctrl,
        curves.get("left_forward"),
        curves.get("right_forward"),
    )
    ctrl.speeds_rev = map_curves_to_speed_levels(
        ctrl,
        curves.get("left_reverse"),
        curves.get("right_reverse"),
    )

    # Fordulási sebesség (omega) skálázása ugyanarról a belső szintskáláról.
    ctrl.turn_omega_levels = {
        i: (ctrl.speed_pwm_levels[i] / ctrl.turn_intensity) if ctrl.turn_intensity else 0.0
        for i in range(10)
    }

def maybe_refresh_speed_tables(ctrl):
    """Újraépíti a táblákat, ha a speed_map konfig objektum címe megváltozott (reload)."""
    current = ctrl.cfg.get("speed_map", {})
    if current is not ctrl._speed_map_ref:
        build_speed_tables(ctrl)

def map_curves_to_speed_levels(ctrl, left_curve, right_curve):
    """
    Segédfüggvény: PWM értékekhez kikeresi a várható sebességet (m/s)
    a kalibrált speed_map alapján. Interpolációt használ.
    """
    if not isinstance(left_curve, dict) or not isinstance(right_curve, dict):
        raise ValueError(
            "R2B4_WHEEL_SPEED_MAP_V2 curve pair is missing; "
            "the calibrated KIT0085 speed map is mandatory"
        )

    left_points = {}
    right_points = {}
    for row in list(left_curve.get("points") or []):
        if not isinstance(row, dict):
            continue
        try:
            left_points[float(row["speed_mps"])] = abs(float(row["pwm"]))
        except (KeyError, TypeError, ValueError):
            continue
    for row in list(right_curve.get("points") or []):
        if not isinstance(row, dict):
            continue
        try:
            right_points[float(row["speed_mps"])] = abs(float(row["pwm"]))
        except (KeyError, TypeError, ValueError):
            continue

    points = [
        (0.5 * (left_points[speed] + right_points[speed]), speed)
        for speed in sorted(set(left_points) & set(right_points))
    ]

    if not points:
        raise ValueError(
            "R2B4_WHEEL_SPEED_MAP_V2 curve pair has no shared calibrated speeds"
        )

    points.sort(key=lambda x: x[0])

    def interp_speed(target_pwm):
        # Szélső értékek kezelése
        if target_pwm <= points[0][0]:
            return points[0][1]
        if target_pwm >= points[-1][0]:
            return points[-1][1]
        
        # Lineáris interpoláció
        for (p0, s0), (p1, s1) in zip(points, points[1:]):
            if p0 <= target_pwm <= p1:
                if abs(p1 - p0) < 1e-6:
                    return s0
                t = (target_pwm - p0) / (p1 - p0)
                return s0 + t * (s1 - s0)
        return points[-1][1]

    # Szintek feltöltése
    levels = {0: 0.0}
    prev = 0.0
    for i in range(1, 10):
        speed = interp_speed(ctrl.speed_pwm_levels[i])
        # Monotonitás biztosítása (a magasabb fokozat nem lehet lassabb)
        if speed < prev:
            speed = prev
        levels[i] = round(speed, 3)
        prev = speed
    return levels
