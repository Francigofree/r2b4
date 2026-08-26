#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Cycle-local immutable inputs shared by motion proposal/resolver code."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass(frozen=True, slots=True)
class Pose2D:
    x: float
    y: float
    theta_rad: float


@dataclass(frozen=True, slots=True)
class Velocity:
    v_mps: float
    omega_rad_s: float
    left_mps: float = 0.0
    right_mps: float = 0.0


@dataclass(frozen=True, slots=True)
class MotionTickContext:
    pose: Pose2D
    velocity: Velocity
    front_clearance_m: float
    left_clearance_m: float
    right_clearance_m: float
    emergency: bool
    target_visible: bool
    target_distance_m: float
    target_bearing_rad: float
    lidar_seq: int


def _finite_float(value: Any, default: float = math.nan) -> float:
    try:
        out = float(value)
    except Exception:
        return float(default)
    return float(out) if math.isfinite(out) else float(default)


def _first_positive(data: Dict[str, Any], *keys: str) -> float:
    src = dict(data or {})
    for key in keys:
        value = _finite_float(src.get(key), math.nan)
        if math.isfinite(value) and value > 0.0:
            return float(value)
    return math.nan


def _pose_from_ekf(ekf_state: Dict[str, Any]) -> Pose2D:
    src = dict(ekf_state or {})
    theta = _finite_float(src.get("theta"), math.nan)
    if not math.isfinite(theta):
        theta = math.radians(_finite_float(src.get("theta_deg"), 0.0))
    return Pose2D(
        x=_finite_float(src.get("x"), 0.0),
        y=_finite_float(src.get("y"), 0.0),
        theta_rad=float(theta),
    )


def _velocity_from_inputs(
    ekf_state: Dict[str, Any],
    *,
    v_l_mps: Optional[float] = None,
    v_r_mps: Optional[float] = None,
) -> Velocity:
    src = dict(ekf_state or {})
    v = _finite_float(src.get("v", src.get("v_fused")), math.nan)
    omega = _finite_float(src.get("omega_rad_s", src.get("omega")), math.nan)
    left = _finite_float(v_l_mps, math.nan)
    right = _finite_float(v_r_mps, math.nan)
    if (not math.isfinite(v)) and math.isfinite(left) and math.isfinite(right):
        v = 0.5 * (left + right)
    if not math.isfinite(v):
        v = 0.0
    if not math.isfinite(omega):
        omega = 0.0
    return Velocity(
        v_mps=float(v),
        omega_rad_s=float(omega),
        left_mps=0.0 if not math.isfinite(left) else float(left),
        right_mps=0.0 if not math.isfinite(right) else float(right),
    )


def _target_fields(target_observation: Any) -> tuple[bool, float, float]:
    if target_observation is None:
        return False, math.nan, math.nan
    if hasattr(target_observation, "to_dict"):
        try:
            src = dict(target_observation.to_dict() or {})
        except Exception:
            src = {}
    elif isinstance(target_observation, dict):
        src = dict(target_observation or {})
    else:
        src = {
            "distance_m": getattr(target_observation, "distance_m", None),
            "bearing_rad": getattr(target_observation, "bearing_rad", None),
            "confidence": getattr(target_observation, "confidence", 0.0),
        }
    visible = bool(src.get("target_visible", True)) and not bool(src.get("stale", False))
    confidence = _finite_float(src.get("confidence"), 1.0)
    distance = _finite_float(src.get("distance_m", src.get("distance_to_target_m")), math.nan)
    bearing = _finite_float(src.get("bearing_rad"), math.nan)
    return bool(visible and confidence > 0.0), float(distance), float(bearing)


def build_motion_tick_context(
    *,
    ekf_state: Dict[str, Any],
    lidar_summary: Dict[str, Any],
    emergency: bool = False,
    target_observation: Any = None,
    v_l_mps: Optional[float] = None,
    v_r_mps: Optional[float] = None,
) -> MotionTickContext:
    l_sum = dict(lidar_summary or {})
    target_visible, target_distance_m, target_bearing_rad = _target_fields(target_observation)
    lidar_seq = int(_finite_float(l_sum.get("scan_seq"), 0.0))
    return MotionTickContext(
        pose=_pose_from_ekf(dict(ekf_state or {})),
        velocity=_velocity_from_inputs(dict(ekf_state or {}), v_l_mps=v_l_mps, v_r_mps=v_r_mps),
        front_clearance_m=_first_positive(l_sum, "min_dist_narrow", "front_clearance_m", "front_clearance", "min_dist"),
        left_clearance_m=_first_positive(l_sum, "left_clearance_m", "left_clearance", "min_left_clearance_m", "avg_left"),
        right_clearance_m=_first_positive(l_sum, "right_clearance_m", "right_clearance", "min_right_clearance_m", "avg_right"),
        emergency=bool(emergency),
        target_visible=bool(target_visible),
        target_distance_m=float(target_distance_m),
        target_bearing_rad=float(target_bearing_rad),
        lidar_seq=int(lidar_seq),
    )


def new_motion_tick_cache(context: MotionTickContext) -> Dict[str, Any]:
    return {
        "lidar_seq": int(context.lidar_seq),
        "clearance_cache": {},
        "front_gap_cache": {},
        "proposal_conversion_cache": {},
        "resolver_fast_cache": {},
    }


def motion_tick_context_status(context: MotionTickContext) -> Dict[str, Any]:
    return {
        "pose": {
            "x": round(float(context.pose.x), 5),
            "y": round(float(context.pose.y), 5),
            "theta_rad": round(float(context.pose.theta_rad), 6),
        },
        "velocity": {
            "v_mps": round(float(context.velocity.v_mps), 5),
            "omega_rad_s": round(float(context.velocity.omega_rad_s), 6),
            "left_mps": round(float(context.velocity.left_mps), 5),
            "right_mps": round(float(context.velocity.right_mps), 5),
        },
        "front_clearance_m": None if not math.isfinite(context.front_clearance_m) else round(float(context.front_clearance_m), 4),
        "left_clearance_m": None if not math.isfinite(context.left_clearance_m) else round(float(context.left_clearance_m), 4),
        "right_clearance_m": None if not math.isfinite(context.right_clearance_m) else round(float(context.right_clearance_m), 4),
        "emergency": bool(context.emergency),
        "target_visible": bool(context.target_visible),
        "target_distance_m": None if not math.isfinite(context.target_distance_m) else round(float(context.target_distance_m), 4),
        "target_bearing_rad": None if not math.isfinite(context.target_bearing_rad) else round(float(context.target_bearing_rad), 6),
        "lidar_seq": int(context.lidar_seq),
    }
