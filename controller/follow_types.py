#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Shared target-following data contracts.

These types describe the high-level follow intent only.  They do not write
motor, PWM, or executor state.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple


TARGET_SOURCE_TARGET = "TARGET"
TARGET_SOURCE_SIM_TARGET = "SIM_TARGET"
TARGET_SOURCE_CAMERA_TARGET = "CAMERA_TARGET"
TARGET_SOURCE_CAMERA_SEARCH = "CAMERA_SEARCH"

FRAME_WORLD = "world"
FRAME_ROBOT = "robot"


def safe_float(value: Any, default: Optional[float] = 0.0) -> Optional[float]:
    try:
        if value is None:
            return default
        out = float(value)
        if not math.isfinite(out):
            return default
        return out
    except Exception:
        return default


def wrap_angle_rad(rad: float) -> float:
    out = float(rad)
    while out >= math.pi:
        out -= 2.0 * math.pi
    while out < -math.pi:
        out += 2.0 * math.pi
    return out


def normalize_target_source(source: Any) -> str:
    raw = str(source or TARGET_SOURCE_TARGET).strip().upper()
    if raw in {TARGET_SOURCE_TARGET, TARGET_SOURCE_SIM_TARGET, TARGET_SOURCE_CAMERA_TARGET, TARGET_SOURCE_CAMERA_SEARCH}:
        return raw
    return TARGET_SOURCE_TARGET


def normalize_frame(frame: Any) -> str:
    raw = str(frame or FRAME_WORLD).strip().lower()
    if raw in {FRAME_WORLD, FRAME_ROBOT}:
        return raw
    return FRAME_WORLD


@dataclass(frozen=True)
class TargetObservation:
    """A target measurement from a simulator, behavior, or camera pipeline."""

    source: str = TARGET_SOURCE_TARGET
    frame: str = FRAME_WORLD
    timestamp_s: float = 0.0
    x: Optional[float] = None
    y: Optional[float] = None
    theta: Optional[float] = None
    distance_m: Optional[float] = None
    bearing_rad: Optional[float] = None
    vx: Optional[float] = None
    vy: Optional[float] = None
    confidence: float = 1.0
    desired_distance_m: Optional[float] = None
    v_max_mps: Optional[float] = None
    omega_max_rad_s: Optional[float] = None
    target_id: str = ""
    front_obstacle_distance_m: Optional[float] = None
    target_zone: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "source": normalize_target_source(self.source),
            "frame": normalize_frame(self.frame),
            "timestamp_s": float(self.timestamp_s),
            "x": safe_float(self.x, None),
            "y": safe_float(self.y, None),
            "theta": safe_float(self.theta, None),
            "distance_m": safe_float(self.distance_m, None),
            "bearing_rad": safe_float(self.bearing_rad, None),
            "vx": safe_float(self.vx, None),
            "vy": safe_float(self.vy, None),
            "confidence": max(0.0, min(1.0, float(safe_float(self.confidence, 0.0) or 0.0))),
            "desired_distance_m": safe_float(self.desired_distance_m, None),
            "v_max_mps": safe_float(self.v_max_mps, None),
            "omega_max_rad_s": safe_float(self.omega_max_rad_s, None),
            "target_id": str(self.target_id or ""),
            "front_obstacle_distance_m": safe_float(self.front_obstacle_distance_m, None),
            "target_zone": str(self.target_zone or ""),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any], *, default_ts_s: float = 0.0) -> "TargetObservation":
        src = dict(data or {})
        bearing = src.get("bearing_rad")
        if bearing is None and src.get("bearing_deg") is not None:
            bearing = math.radians(float(src.get("bearing_deg")))
        return cls(
            source=normalize_target_source(src.get("source") or src.get("target_source")),
            frame=normalize_frame(src.get("frame")),
            timestamp_s=float(safe_float(src.get("timestamp_s", src.get("ts")), default_ts_s) or default_ts_s),
            x=safe_float(src.get("x"), None),
            y=safe_float(src.get("y"), None),
            theta=safe_float(src.get("theta", src.get("theta_rad")), None),
            distance_m=safe_float(src.get("distance_m"), None),
            bearing_rad=safe_float(bearing, None),
            vx=safe_float(src.get("vx"), None),
            vy=safe_float(src.get("vy"), None),
            confidence=float(safe_float(src.get("confidence"), 1.0) or 0.0),
            desired_distance_m=safe_float(src.get("desired_distance_m", src.get("follow_distance_m")), None),
            v_max_mps=safe_float(src.get("v_max_mps", src.get("v_max")), None),
            omega_max_rad_s=safe_float(src.get("omega_max_rad_s", src.get("omega_max")), None),
            target_id=str(src.get("target_id", "") or ""),
            front_obstacle_distance_m=safe_float(src.get("front_obstacle_distance_m"), None),
            target_zone=str(src.get("target_zone", "") or ""),
        )


@dataclass(frozen=True)
class FollowRequest:
    """Follow-layer output consumed by the cruise layer."""

    active: bool
    source: str
    target_source: str
    target_x: Optional[float] = None
    target_y: Optional[float] = None
    target_theta: Optional[float] = None
    target_vx: Optional[float] = None
    target_vy: Optional[float] = None
    goal_x: Optional[float] = None
    goal_y: Optional[float] = None
    goal_theta: Optional[float] = None
    distance_to_target_m: Optional[float] = None
    desired_distance_m: float = 0.0
    age_s: Optional[float] = None
    confidence: float = 0.0
    stale: bool = False
    reason: str = ""
    v_max_mps: Optional[float] = None
    omega_max_rad_s: Optional[float] = None
    target_id: str = ""
    front_obstacle_distance_m: Optional[float] = None
    target_zone: str = ""

    def target_pose(self) -> Optional[Tuple[float, float, float]]:
        if not self.active:
            return None
        gx = safe_float(self.goal_x, None)
        gy = safe_float(self.goal_y, None)
        gt = safe_float(self.goal_theta, None)
        if gx is None or gy is None or gt is None:
            return None
        return (float(gx), float(gy), float(gt))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "active": bool(self.active),
            "source": str(self.source or ""),
            "target_source": normalize_target_source(self.target_source),
            "target_x": safe_float(self.target_x, None),
            "target_y": safe_float(self.target_y, None),
            "target_theta": safe_float(self.target_theta, None),
            "target_vx": safe_float(self.target_vx, None),
            "target_vy": safe_float(self.target_vy, None),
            "goal_x": safe_float(self.goal_x, None),
            "goal_y": safe_float(self.goal_y, None),
            "goal_theta": safe_float(self.goal_theta, None),
            "distance_to_target_m": safe_float(self.distance_to_target_m, None),
            "desired_distance_m": float(max(0.0, safe_float(self.desired_distance_m, 0.0) or 0.0)),
            "age_s": safe_float(self.age_s, None),
            "confidence": max(0.0, min(1.0, float(safe_float(self.confidence, 0.0) or 0.0))),
            "stale": bool(self.stale),
            "reason": str(self.reason or ""),
            "v_max_mps": safe_float(self.v_max_mps, None),
            "omega_max_rad_s": safe_float(self.omega_max_rad_s, None),
            "target_id": str(self.target_id or ""),
            "front_obstacle_distance_m": safe_float(self.front_obstacle_distance_m, None),
            "target_zone": str(self.target_zone or ""),
            "target_pose": list(self.target_pose()) if self.target_pose() is not None else None,
        }
