#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Tuple

from config_manager import config as global_config

WHEEL_SPEED_MAP_SCHEMA = "R2B4_WHEEL_SPEED_MAP_V2"
WHEEL_CURVE_KEYS = (
    "left_forward",
    "left_reverse",
    "right_forward",
    "right_reverse",
)


def wheel_curve_key(side: str, direction: str) -> str:
    side_name = str(side or "").strip().lower()
    direction_name = str(direction or "").strip().lower()
    key = f"{side_name}_{direction_name}"
    if key not in WHEEL_CURVE_KEYS:
        raise ValueError(f"invalid_wheel_curve:{key}")
    return key


def _curve_points(curve: Dict[str, Any]) -> List[Tuple[float, float]]:
    points: List[Tuple[float, float]] = []
    for row in list(curve.get("points") or []):
        if not isinstance(row, dict):
            continue
        try:
            speed = abs(float(row.get("speed_mps")))
            pwm = abs(float(row.get("pwm")))
        except (TypeError, ValueError):
            continue
        if math.isfinite(speed) and math.isfinite(pwm) and speed > 0.0 and 0.0 < pwm <= 1.0:
            points.append((speed, pwm))
    points.sort(key=lambda item: item[0])
    if len(points) < 2:
        raise ValueError("wheel_curve_requires_at_least_two_points")
    if any(right[0] <= left[0] for left, right in zip(points, points[1:])):
        raise ValueError("wheel_curve_speed_points_not_strictly_increasing")
    if any(right[1] + 1e-9 < left[1] for left, right in zip(points, points[1:])):
        raise ValueError("wheel_curve_pwm_not_monotonic")
    return points


def active_wheel_speed_range(
    speed_map: Dict[str, Any],
    *,
    require_active: bool = True,
) -> Tuple[float, float]:
    """Return the common calibrated speed interval of all four wheel curves.

    The common interval is deliberately derived from the same points used by
    feed-forward.  A profile limit outside this interval would otherwise be
    silently converted by ``lookup_wheel_feedforward`` to a clamped endpoint.
    """

    if str(speed_map.get("schema", "") or "") != WHEEL_SPEED_MAP_SCHEMA:
        raise ValueError("wheel_speed_map_schema_invalid")
    map_state = str(speed_map.get("map_state", "") or "").strip().upper()
    if require_active and map_state != "ACTIVE":
        raise ValueError(f"wheel_speed_map_not_active:{map_state or 'MISSING'}")

    curves = dict(speed_map.get("curves") or {})
    ranges: List[Tuple[float, float]] = []
    for key in WHEEL_CURVE_KEYS:
        curve = dict(curves.get(key) or {})
        if not curve:
            raise ValueError(f"wheel_curve_missing:{key}")
        points = _curve_points(curve)
        ranges.append((float(points[0][0]), float(points[-1][0])))

    common_min = max(item[0] for item in ranges)
    common_max = min(item[1] for item in ranges)
    if common_max + 1e-9 < common_min:
        raise ValueError("wheel_curves_have_no_common_calibrated_speed_range")
    return float(common_min), float(common_max)


def lookup_wheel_feedforward(
    speed_map: Dict[str, Any],
    *,
    side: str,
    target_mps: float,
    require_active: bool = False,
) -> Tuple[float, Dict[str, Any]]:
    if str(speed_map.get("schema", "") or "") != WHEEL_SPEED_MAP_SCHEMA:
        raise ValueError("wheel_speed_map_schema_invalid")
    map_state = str(speed_map.get("map_state", "") or "").strip().upper()
    if require_active and map_state != "ACTIVE":
        raise ValueError(f"wheel_speed_map_not_active:{map_state or 'MISSING'}")

    target = float(target_mps)
    if not math.isfinite(target):
        raise ValueError("wheel_target_not_finite")
    direction = "forward" if target >= 0.0 else "reverse"
    key = wheel_curve_key(side, direction)
    curve = dict((speed_map.get("curves") or {}).get(key) or {})
    if not curve:
        raise ValueError(f"wheel_curve_missing:{key}")
    points = _curve_points(curve)
    maintenance_pwm = abs(
        float(
            curve.get(
                "maintenance_pwm",
                curve.get("dead_zone_pwm", points[0][1]),
            )
            or points[0][1]
        )
    )
    startup_pwm = abs(
        float(curve.get("startup_pwm", maintenance_pwm) or maintenance_pwm)
    )
    if not (
        math.isfinite(maintenance_pwm)
        and math.isfinite(startup_pwm)
        and 0.0 < maintenance_pwm <= 1.0
        and maintenance_pwm <= startup_pwm <= 1.0
    ):
        raise ValueError(f"wheel_curve_thresholds_invalid:{key}")

    if abs(target) <= 1e-9:
        return 0.0, {
            "valid": True,
            "schema": WHEEL_SPEED_MAP_SCHEMA,
            "map_state": map_state,
            "curve": key,
            "target_mps": 0.0,
            "interpolation": "zero",
            "lower_point": None,
            "upper_point": None,
            "ratio": 0.0,
            "feedforward_pwm": 0.0,
            "startup_pwm": float(startup_pwm),
            "maintenance_pwm": float(maintenance_pwm),
            "dead_zone_pwm": float(maintenance_pwm),
        }

    speed = abs(target)
    if speed <= points[0][0]:
        lower = upper = points[0]
        ratio = 0.0
        interpolation = "clamp_low"
        pwm = points[0][1]
    elif speed >= points[-1][0]:
        lower = upper = points[-1]
        ratio = 1.0
        interpolation = "clamp_high"
        pwm = points[-1][1]
    else:
        lower = upper = points[-1]
        ratio = 1.0
        interpolation = "linear"
        pwm = points[-1][1]
        for point_left, point_right in zip(points, points[1:]):
            if point_left[0] <= speed <= point_right[0]:
                lower, upper = point_left, point_right
                ratio = (speed - point_left[0]) / max(1e-9, point_right[0] - point_left[0])
                pwm = point_left[1] + ratio * (point_right[1] - point_left[1])
                break

    signed_pwm = math.copysign(float(pwm), target)
    return signed_pwm, {
        "valid": True,
        "schema": WHEEL_SPEED_MAP_SCHEMA,
        "map_state": map_state,
        "curve": key,
        "target_mps": float(target),
        "interpolation": interpolation,
        "lower_point": {"speed_mps": lower[0], "pwm": lower[1]},
        "upper_point": {"speed_mps": upper[0], "pwm": upper[1]},
        "ratio": float(ratio),
        "feedforward_pwm": float(signed_pwm),
        "startup_pwm": float(startup_pwm),
        "maintenance_pwm": float(maintenance_pwm),
        "dead_zone_pwm": float(maintenance_pwm),
    }


@dataclass
class PIDConfig:
    kp: float = 0.7
    ki: float = 0.0
    integrator_limit: float = 0.3
    k_ff: float = 1.0
    dz_min: float = 0.20  # Alapértelmezett, de felülírható
    wheel_feedback_trust_min: float = 0.55
    # Motor-model compensation (executor-level, non-ARC-specific)
    motor_compensation_enabled: bool = True
    straight_hold_enabled: bool = True
    straight_hold_kp: float = 1.15
    straight_hold_max_w: float = 0.14
    straight_hold_slew_rate: float = 0.90
    straight_hold_heading_deadband_deg: float = 0.35
    straight_hold_v_min_mps: float = 0.03
    straight_hold_w_request_eps: float = 0.03
class AlbaDriveController:
    """The single wheel-level speed-map to PWM feed-forward lookup."""

    def __init__(self, cfg: PIDConfig, map_path=None):
        self.cfg = cfg
        self.dead_zone = cfg.dz_min

        if map_path is None:
            map_path = global_config.path("speed_map.json")
        self.map_path = map_path

        self.speed_map = global_config.get("speed_map", default={}) or {}
        self._diag = {}

    def reset(self):
        self._diag = {}

    def get_wheel_feedforward(self, side: str, target_mps: float) -> Tuple[float, Dict[str, Any]]:
        try:
            pwm, diag = lookup_wheel_feedforward(
                self.speed_map,
                side=side,
                target_mps=target_mps,
                require_active=True,
            )
        except (TypeError, ValueError) as exc:
            pwm = 0.0
            diag = {
                "valid": False,
                "curve": None,
                "target_mps": float(target_mps),
                "feedforward_pwm": 0.0,
                "error": str(exc),
            }
        self._diag[str(side)] = dict(diag)
        return float(pwm), dict(diag)

    def _get_baseline(self, v_target):
        left, _ = self.get_wheel_feedforward("left", v_target)
        right, _ = self.get_wheel_feedforward("right", v_target)
        return left, right

    def get_pid_diagnostics(self) -> dict:
        return dict(self._diag)
