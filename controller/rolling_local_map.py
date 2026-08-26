#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Short-lived local obstacle memory for navigation.

The map is intentionally small and deterministic. It stores recent obstacle
observations in world coordinates, then exposes the current robot-frame view to
the local navigation layer. It never writes motion state or motor outputs.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


ROLLING_LOCAL_MAP_SCHEMA_VERSION = "ROLLING_LOCAL_MAP_V1"

_FRONT_KEYS = ("min_dist_narrow", "front_clearance_m", "front_clearance", "min_dist")
_LEFT_KEYS = ("left_clearance_m", "left_clearance", "min_left_clearance_m", "avg_left")
_RIGHT_KEYS = ("right_clearance_m", "right_clearance", "min_right_clearance_m", "avg_right")
_REAR_KEYS = ("min_back", "back_clearance_m", "back_clearance", "rear_clearance_m", "rear_clearance")

_FRONT_BLOCK_M = 0.30
_REAR_BLOCK_M = 0.25


@dataclass(frozen=True)
class _Observation:
    x: float
    y: float
    ts: float
    source: str


def _safe_float(value: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        if value is None:
            return default
        out = float(value)
    except Exception:
        return default
    if not math.isfinite(out):
        return default
    return float(out)


def _first_finite(data: Dict[str, Any], keys: Iterable[str]) -> Optional[float]:
    src = dict(data or {})
    for key in keys:
        value = _safe_float(src.get(key), None)
        if value is not None and value > 0.0:
            return float(value)
    return None


def _extract_pose(ekf_state: Dict[str, Any]) -> Tuple[float, float, float]:
    src = dict(ekf_state or {})
    x = _safe_float(src.get("x"), 0.0) or 0.0
    y = _safe_float(src.get("y"), 0.0) or 0.0
    theta = _safe_float(src.get("theta"), None)
    if theta is None:
        theta = math.radians(_safe_float(src.get("theta_deg"), 0.0) or 0.0)
    return float(x), float(y), float(theta)


def _robot_to_world(rx: float, ry: float, pose: Tuple[float, float, float]) -> Tuple[float, float]:
    px, py, theta = pose
    c = math.cos(theta)
    s = math.sin(theta)
    return float(px + (c * rx) - (s * ry)), float(py + (s * rx) + (c * ry))


def _world_to_robot(wx: float, wy: float, pose: Tuple[float, float, float]) -> Tuple[float, float]:
    px, py, theta = pose
    dx = float(wx) - float(px)
    dy = float(wy) - float(py)
    c = math.cos(theta)
    s = math.sin(theta)
    return float((c * dx) + (s * dy)), float((-s * dx) + (c * dy))


def _scan_point_robot(point: Any) -> Optional[Tuple[float, float, float]]:
    if not isinstance(point, dict):
        return None
    dist_raw = point.get("dist", point.get("dist_mm", point.get("distance", point.get("distance_mm"))))
    dist_value = _safe_float(dist_raw, None)
    if dist_value is None or dist_value <= 0.0:
        return None
    dist_m = float(dist_value) / 1000.0 if float(dist_value) > 20.0 else float(dist_value)
    if not math.isfinite(dist_m) or dist_m <= 0.01:
        return None

    angle_rad = _safe_float(point.get("angle_rad"), None)
    if angle_rad is None:
        angle_deg = _safe_float(point.get("angle"), None)
        if angle_deg is None:
            return None
        angle_rad = math.radians(float(angle_deg))
    if not math.isfinite(float(angle_rad)):
        return None

    rx = math.cos(float(angle_rad)) * dist_m
    ry = -math.sin(float(angle_rad)) * dist_m
    return float(rx), float(ry), float(dist_m)


def _bearing_to_scan_angle_deg(rx: float, ry: float) -> float:
    return float((-math.degrees(math.atan2(float(ry), float(rx)))) % 360.0)


def _round_or_none(value: Optional[float], places: int = 4) -> Optional[float]:
    if value is None:
        return None
    if not math.isfinite(float(value)):
        return None
    return round(float(value), int(places))


def _angle_delta_rad(a: float, b: float) -> float:
    return float((float(a) - float(b) + math.pi) % (2.0 * math.pi) - math.pi)


class RollingLocalMap:
    """A bounded, time-decayed obstacle memory around the robot."""

    def __init__(
        self,
        *,
        ttl_s: float = 3.0,
        radius_m: float = 2.5,
        max_points: int = 800,
        raw_scan_limit: int = 360,
    ) -> None:
        self.ttl_s = max(0.1, float(ttl_s))
        self.radius_m = max(0.25, float(radius_m))
        self.max_points = max(32, int(max_points))
        self.raw_scan_limit = max(16, int(raw_scan_limit))
        self._observations: List[_Observation] = []
        self._revision: int = 0
        self._snapshot_cache: Dict[str, Any] = {}
        self._snapshot_cache_revision: int = -1
        self._snapshot_cache_pose: Optional[Tuple[float, float, float]] = None
        self._snapshot_cache_ts: float = 0.0
        self._snapshot_cache_has_raw_scan: bool = False

    def reset(self) -> None:
        self._observations = []
        self._revision += 1
        self._snapshot_cache = {}
        self._snapshot_cache_revision = -1
        self._snapshot_cache_pose = None
        self._snapshot_cache_ts = 0.0
        self._snapshot_cache_has_raw_scan = False

    def _purge(self, *, now_s: float, pose: Optional[Tuple[float, float, float]] = None) -> None:
        cutoff = float(now_s) - float(self.ttl_s)
        if pose is None:
            kept = [obs for obs in self._observations if float(obs.ts) >= cutoff]
        else:
            kept = []
            for obs in self._observations:
                if float(obs.ts) < cutoff:
                    continue
                rx, ry = _world_to_robot(obs.x, obs.y, pose)
                if math.hypot(rx, ry) <= self.radius_m:
                    kept.append(obs)
        if len(kept) > self.max_points:
            kept = kept[-self.max_points :]
        self._observations = kept

    def update(
        self,
        *,
        raw_scan: Optional[List[Dict[str, Any]]] = None,
        lidar_summary: Optional[Dict[str, Any]] = None,
        ekf_state: Optional[Dict[str, Any]] = None,
        now_s: Optional[float] = None,
    ) -> Dict[str, Any]:
        now = time.monotonic() if now_s is None else float(now_s)
        pose = _extract_pose(dict(ekf_state or {}))
        self._purge(now_s=now, pose=pose)

        added_scan = 0
        added_summary = 0
        new_obs: List[_Observation] = []
        scan_points = raw_scan if isinstance(raw_scan, list) else list(raw_scan or [])
        for point in scan_points:
            robot_point = _scan_point_robot(point)
            if robot_point is None:
                continue
            rx, ry, dist_m = robot_point
            if dist_m > self.radius_m:
                continue
            wx, wy = _robot_to_world(rx, ry, pose)
            new_obs.append(_Observation(wx, wy, now, "raw_scan"))
            added_scan += 1

        summary = dict(lidar_summary or {})
        summary_points = (
            ("summary_front", _first_finite(summary, _FRONT_KEYS), 0.0),
            ("summary_left", _first_finite(summary, _LEFT_KEYS), math.pi * 0.5),
            ("summary_right", _first_finite(summary, _RIGHT_KEYS), -math.pi * 0.5),
            ("summary_rear", _first_finite(summary, _REAR_KEYS), math.pi),
        )
        for source, dist_m, bearing_rad in summary_points:
            if dist_m is None or float(dist_m) <= 0.0 or float(dist_m) > self.radius_m:
                continue
            rx = math.cos(float(bearing_rad)) * float(dist_m)
            ry = math.sin(float(bearing_rad)) * float(dist_m)
            wx, wy = _robot_to_world(rx, ry, pose)
            new_obs.append(_Observation(wx, wy, now, source))
            added_summary += 1

        if new_obs:
            self._observations.extend(new_obs)
            if len(self._observations) > self.max_points:
                self._observations = self._observations[-self.max_points :]
        self._revision += 1

        snap = self.snapshot(dict(ekf_state or {}), now_s=now, include_raw_scan=False)
        snap["added_scan_points"] = int(added_scan)
        snap["added_summary_points"] = int(added_summary)
        return snap

    def _robot_points(
        self,
        *,
        pose: Tuple[float, float, float],
        now_s: float,
        sort_points: bool = True,
    ) -> List[Tuple[float, float, float, str]]:
        points: List[Tuple[float, float, float, str]] = []
        for obs in self._observations:
            rx, ry = _world_to_robot(obs.x, obs.y, pose)
            dist = math.hypot(rx, ry)
            if dist <= self.radius_m:
                points.append((float(rx), float(ry), float(now_s - obs.ts), str(obs.source)))
        if bool(sort_points):
            points.sort(
                key=lambda item: (
                    _bearing_to_scan_angle_deg(item[0], item[1]),
                    math.hypot(item[0], item[1]),
                    item[3],
                )
            )
        return points

    @staticmethod
    def _sector_mins(points: List[Tuple[float, float, float, str]]) -> Dict[str, Optional[float]]:
        front: List[float] = []
        left: List[float] = []
        right: List[float] = []
        rear: List[float] = []
        all_dist: List[float] = []
        for rx, ry, _age, _source in points:
            dist = math.hypot(rx, ry)
            if not math.isfinite(dist) or dist <= 0.0:
                continue
            all_dist.append(float(dist))
            bearing = math.atan2(ry, rx)
            if rx >= 0.02 and abs(bearing) <= math.radians(32.0):
                front.append(float(dist))
            if ry >= 0.05 and math.radians(35.0) <= bearing <= math.radians(145.0):
                left.append(float(dist))
            if ry <= -0.05 and -math.radians(145.0) <= bearing <= -math.radians(35.0):
                right.append(float(dist))
            if rx <= -0.02 and abs(abs(bearing) - math.pi) <= math.radians(32.0):
                rear.append(float(dist))
        return {
            "min_dist_m": min(all_dist) if all_dist else None,
            "front_clearance_m": min(front) if front else None,
            "left_clearance_m": min(left) if left else None,
            "right_clearance_m": min(right) if right else None,
            "rear_clearance_m": min(rear) if rear else None,
        }

    def snapshot(
        self,
        ekf_state: Optional[Dict[str, Any]] = None,
        *,
        now_s: Optional[float] = None,
        include_raw_scan: bool = True,
    ) -> Dict[str, Any]:
        now = time.monotonic() if now_s is None else float(now_s)
        pose = _extract_pose(dict(ekf_state or {}))
        cached_pose = self._snapshot_cache_pose
        cache_pose_close = bool(
            cached_pose is not None
            and math.hypot(float(pose[0]) - float(cached_pose[0]), float(pose[1]) - float(cached_pose[1])) <= 0.01
            and abs(_angle_delta_rad(float(pose[2]), float(cached_pose[2]))) <= 0.02
        )
        if (
            self._snapshot_cache
            and int(self._snapshot_cache_revision) == int(self._revision)
            and cache_pose_close
            and 0.0 <= float(now) - float(self._snapshot_cache_ts) <= 0.08
            and (not bool(include_raw_scan) or bool(self._snapshot_cache_has_raw_scan))
        ):
            cached = dict(self._snapshot_cache)
            if not include_raw_scan:
                cached["raw_scan"] = []
            return cached
        self._purge(now_s=now, pose=pose)
        points = self._robot_points(pose=pose, now_s=now, sort_points=bool(include_raw_scan))
        mins = self._sector_mins(points)
        ages = [float(age) for _rx, _ry, age, _source in points]

        raw_scan: List[Dict[str, Any]] = []
        if bool(include_raw_scan):
            for rx, ry, _age, _source in points[: self.raw_scan_limit]:
                dist_m = math.hypot(rx, ry)
                if not math.isfinite(dist_m) or dist_m <= 0.0:
                    continue
                raw_scan.append({"angle": _bearing_to_scan_angle_deg(rx, ry), "dist": float(dist_m) * 1000.0})

        front_m = mins["front_clearance_m"]
        rear_m = mins["rear_clearance_m"]
        result = {
            "schema": ROLLING_LOCAL_MAP_SCHEMA_VERSION,
            "enabled": True,
            "has_data": bool(points),
            "ttl_s": round(float(self.ttl_s), 4),
            "radius_m": round(float(self.radius_m), 4),
            "observation_count": int(len(self._observations)),
            "valid_points": int(len(points)),
            "min_dist_m": _round_or_none(mins["min_dist_m"]),
            "front_clearance_m": _round_or_none(front_m),
            "left_clearance_m": _round_or_none(mins["left_clearance_m"]),
            "right_clearance_m": _round_or_none(mins["right_clearance_m"]),
            "rear_clearance_m": _round_or_none(rear_m),
            "blocked_front": bool(front_m is not None and float(front_m) <= _FRONT_BLOCK_M),
            "blocked_back": bool(rear_m is not None and float(rear_m) <= _REAR_BLOCK_M),
            "oldest_age_s": _round_or_none(max(ages) if ages else None),
            "newest_age_s": _round_or_none(min(ages) if ages else None),
            "raw_scan": raw_scan,
        }
        self._snapshot_cache = dict(result)
        self._snapshot_cache_revision = int(self._revision)
        self._snapshot_cache_pose = tuple(pose)
        self._snapshot_cache_ts = float(now)
        self._snapshot_cache_has_raw_scan = bool(include_raw_scan)
        if not include_raw_scan:
            result = dict(result)
            result["raw_scan"] = []
        return result


def _merge_min(summary: Dict[str, Any], keys: Tuple[str, ...], value: Optional[float]) -> None:
    if value is None or not math.isfinite(float(value)) or float(value) <= 0.0:
        return
    for key in keys:
        existing = _safe_float(summary.get(key), None)
        if existing is None or existing <= 0.0 or float(value) < float(existing):
            summary[key] = float(value)


def enhance_lidar_summary(
    lidar_summary: Optional[Dict[str, Any]],
    rolling_snapshot: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    """Merge rolling-map clearances into the live LIDAR summary.

    Existing blocked flags are preserved. Rolling memory may only make a
    clearance more conservative until the TTL expires.
    """

    out = dict(lidar_summary or {})
    snap = dict(rolling_snapshot or {})
    out["rolling_local_map_enabled"] = bool(snap.get("enabled", False))
    out["rolling_local_map_has_data"] = bool(snap.get("has_data", False))
    out["rolling_local_map_observation_count"] = int(snap.get("observation_count", 0) or 0)
    out["rolling_local_map_valid_points"] = int(snap.get("valid_points", 0) or 0)

    if not bool(snap.get("has_data", False)):
        return out

    front_m = _safe_float(snap.get("front_clearance_m"), None)
    left_m = _safe_float(snap.get("left_clearance_m"), None)
    right_m = _safe_float(snap.get("right_clearance_m"), None)
    rear_m = _safe_float(snap.get("rear_clearance_m"), None)
    min_m = _safe_float(snap.get("min_dist_m"), None)

    _merge_min(out, _FRONT_KEYS, front_m)
    _merge_min(out, _LEFT_KEYS, left_m)
    _merge_min(out, _RIGHT_KEYS, right_m)
    _merge_min(out, _REAR_KEYS, rear_m)
    _merge_min(out, ("min_dist",), min_m)

    out["rolling_front_clearance_m"] = front_m
    out["rolling_left_clearance_m"] = left_m
    out["rolling_right_clearance_m"] = right_m
    out["rolling_rear_clearance_m"] = rear_m
    out["rolling_min_dist_m"] = min_m
    out["blocked_front"] = bool(out.get("blocked_front", False) or bool(snap.get("blocked_front", False)))
    out["blocked_back"] = bool(out.get("blocked_back", False) or bool(snap.get("blocked_back", False)))
    out["rolling_local_map_applied"] = True
    return out
