#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Canonical differential-drive kinematics helpers.

Sign convention (SSOT):
- +v_mps: forward
- +omega_rad_s: left / counter-clockwise yaw
"""

from __future__ import annotations

import math
from typing import Any, Dict, Tuple

KINEMATICS_SIGN_CONVENTION = "+v=forward,+omega=left(CCW)"


def _safe_float(value, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def normalize_track_width(track_width_m: float) -> float:
    return max(0.01, _safe_float(track_width_m, 0.175))


def twist_to_track_velocity(
    v_mps: float,
    omega_rad_s: float,
    track_width_m: float,
) -> Tuple[float, float]:
    half_track = 0.5 * normalize_track_width(track_width_m)
    v = _safe_float(v_mps, 0.0)
    omega = _safe_float(omega_rad_s, 0.0)
    return (
        float(v - omega * half_track),
        float(v + omega * half_track),
    )


def track_velocity_to_twist(
    left_mps: float,
    right_mps: float,
    track_width_m: float,
) -> Tuple[float, float]:
    width = normalize_track_width(track_width_m)
    left = _safe_float(left_mps, 0.0)
    right = _safe_float(right_mps, 0.0)
    return (
        float(0.5 * (left + right)),
        float((right - left) / width),
    )


def enforce_twist_wheel_speed_range(
    v_mps: float,
    omega_rad_s: float,
    track_width_m: float,
    *,
    wheel_min_mps: float,
    wheel_max_mps: float,
) -> Tuple[float, float, Dict[str, Any]]:
    """Clamp a twist to the calibrated differential-wheel operating range.

    Translation keeps its safety-owned linear speed.  Curvature is reduced
    when either same-direction wheel would fall below ``wheel_min_mps`` or
    exceed ``wheel_max_mps``.  A non-zero pure pivot instead maps to the
    calibrated track-speed range, because scaling omega below that range is
    not physically realizable by the active speed map.
    """

    v_in = _safe_float(v_mps, 0.0)
    omega_in = _safe_float(omega_rad_s, 0.0)
    width = normalize_track_width(track_width_m)
    half_track = 0.5 * width
    minimum = max(0.0, _safe_float(wheel_min_mps, 0.0))
    maximum = max(minimum, _safe_float(wheel_max_mps, minimum))
    v_out = float(v_in)
    omega_out = float(omega_in)
    actions = []

    if abs(v_in) <= 1e-9 and abs(omega_in) > 1e-9:
        track_speed_in = abs(float(omega_in)) * half_track
        track_speed_out = max(minimum, min(maximum, track_speed_in))
        omega_out = math.copysign(track_speed_out / half_track, omega_in)
        if abs(track_speed_out - track_speed_in) > 1e-9:
            actions.append("map_pivot_to_wheel_speed_range")
    elif abs(v_in) > 1e-9 and abs(omega_in) > 1e-9:
        travel_speed = abs(float(v_in))
        if travel_speed < minimum - 1e-9:
            omega_out = 0.0
            actions.append("disable_arc_below_wheel_minimum")
        elif travel_speed > maximum + 1e-9:
            v_out = math.copysign(maximum, v_in)
            travel_speed = maximum
            actions.append("clamp_translation_to_wheel_maximum")

        if abs(omega_out) > 1e-9:
            inner_headroom = max(0.0, travel_speed - minimum)
            outer_headroom = max(0.0, maximum - travel_speed)
            feasible_omega_abs = min(inner_headroom, outer_headroom) / half_track
            if abs(omega_out) > feasible_omega_abs + 1e-9:
                omega_out = (
                    math.copysign(feasible_omega_abs, omega_out)
                    if feasible_omega_abs > 1e-9
                    else 0.0
                )
                actions.append("limit_arc_to_wheel_speed_range")

    left_in, right_in = twist_to_track_velocity(v_in, omega_in, width)
    left_out, right_out = twist_to_track_velocity(v_out, omega_out, width)
    diagnostics: Dict[str, Any] = {
        "active": bool(minimum > 0.0 or maximum > 0.0),
        "applied": bool(actions),
        "actions": list(actions),
        "track_width_m": float(width),
        "wheel_min_mps": float(minimum),
        "wheel_max_mps": float(maximum),
        "v_in_mps": float(v_in),
        "omega_in_rad_s": float(omega_in),
        "left_in_mps": float(left_in),
        "right_in_mps": float(right_in),
        "v_out_mps": float(v_out),
        "omega_out_rad_s": float(omega_out),
        "left_out_mps": float(left_out),
        "right_out_mps": float(right_out),
    }
    return float(v_out), float(omega_out), diagnostics
