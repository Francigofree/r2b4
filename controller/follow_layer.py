#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Follow layer: target observations -> FollowRequest.

The layer owns target-frame normalization and follow-distance geometry.  It
does not emit motor commands; the cruise layer turns FollowRequest into a safe
planner primitive.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from controller.follow_types import (
    FRAME_ROBOT,
    FRAME_WORLD,
    TARGET_SOURCE_CAMERA_SEARCH,
    TARGET_SOURCE_CAMERA_TARGET,
    TARGET_SOURCE_TARGET,
    FollowRequest,
    TargetObservation,
    normalize_frame,
    normalize_target_source,
    safe_float,
    wrap_angle_rad,
)


FOLLOW_STANDOFF_RECEDE_MIN_SPEED_MPS = 0.025
FOLLOW_STANDOFF_RECEDE_HORIZON_S = 0.80
FOLLOW_STANDOFF_RECEDE_SLACK_M = 0.020
FOLLOW_STANDOFF_RECEDE_INNER_MARGIN_M = 0.12
FOLLOW_STANDOFF_RECEDE_GOAL_MIN_M = 0.045
FOLLOW_STANDOFF_RECEDE_GOAL_MAX_M = 0.090
FOLLOW_SPEED_SCALE_MIN = 0.05
FOLLOW_SPEED_SCALE_MAX = 1.0
CAMERA_DIRECTION_ONLY_DESIRED_DISTANCE_M = 2.0
CAMERA_TARGET_OMEGA_MAX_RAD_S = 0.42
CAMERA_SEARCH_OMEGA_FAST_MAX_RAD_S = 0.80
CAMERA_SEARCH_OMEGA_SLOW_MAX_RAD_S = 0.44
CAMERA_SEARCH_PIVOT_OMEGA_DEFAULT_RAD_S = 0.08
CAMERA_SEARCH_PIVOT_OMEGA_MIN_RAD_S = 0.02
CAMERA_SEARCH_PIVOT_OMEGA_MAX_RAD_S = 0.30
CAMERA_SEARCH_LATENCY_FAST_MS = 120.0
CAMERA_SEARCH_LATENCY_SLOW_MS = 450.0
CAMERA_SEARCH_HEADING_DELTA_RAD = 0.55


@dataclass(frozen=True)
class FollowLayerConfig:
    max_target_age_s: float = 1.0
    min_confidence: float = 0.05
    default_desired_distance_m: float = 0.4
    camera_desired_distance_m: float = 1.2
    standoff_hold_margin_m: float = 0.05


def create_config(raw_cfg: Dict[str, Any]) -> FollowLayerConfig:
    cfg = dict(raw_cfg or {})
    follower = dict(cfg.get("follower") or {})
    follow_layer = dict(cfg.get("follow_layer") or {})
    target_distance = safe_float(follower.get("target_distance_m"), 1.2)
    default_desired = safe_float(follow_layer.get("default_desired_distance_m"), 0.4)
    return FollowLayerConfig(
        max_target_age_s=float(safe_float(follow_layer.get("max_target_age_s"), 1.0) or 1.0),
        min_confidence=float(safe_float(follow_layer.get("min_confidence"), 0.05) or 0.05),
        default_desired_distance_m=float(default_desired if default_desired is not None else 0.4),
        camera_desired_distance_m=float(
            safe_float(follow_layer.get("camera_desired_distance_m"), target_distance) or 1.2
        ),
        standoff_hold_margin_m=float(safe_float(follow_layer.get("standoff_hold_margin_m"), 0.05) or 0.05),
    )


def _follow_speed_scale(ctrl: Any) -> float:
    scale = safe_float(getattr(ctrl, "follow_speed_scale", 1.0), 1.0)
    if scale is None:
        scale = 1.0
    return max(FOLLOW_SPEED_SCALE_MIN, min(FOLLOW_SPEED_SCALE_MAX, float(scale)))


def _scaled_limit(value: Any, scale: float) -> Optional[float]:
    limit = safe_float(value, None)
    if limit is None:
        return None
    return max(0.0, float(limit) * float(scale))


def _search_omega_limit_for_camera(ctrl: Any, base_limit: Optional[float]) -> float:
    camera_status = dict(getattr(ctrl, "_adaptive_target_camera_status", {}) or {})
    latency_ms = safe_float(camera_status.get("detector_latency_ms"), None)
    cap = float(CAMERA_SEARCH_OMEGA_FAST_MAX_RAD_S)
    if latency_ms is not None and float(latency_ms) > CAMERA_SEARCH_LATENCY_FAST_MS:
        ratio = min(
            1.0,
            max(
                0.0,
                (float(latency_ms) - CAMERA_SEARCH_LATENCY_FAST_MS)
                / max(1e-6, CAMERA_SEARCH_LATENCY_SLOW_MS - CAMERA_SEARCH_LATENCY_FAST_MS),
            ),
        )
        cap = CAMERA_SEARCH_OMEGA_FAST_MAX_RAD_S - (
            ratio * (CAMERA_SEARCH_OMEGA_FAST_MAX_RAD_S - CAMERA_SEARCH_OMEGA_SLOW_MAX_RAD_S)
        )
    cfg_limit = safe_float(base_limit, None)
    if cfg_limit is None or float(cfg_limit) <= 0.0:
        cfg_limit = float(cap)
    pivot_limit = safe_float(
        getattr(ctrl, "follow_search_pivot_omega_rad_s", CAMERA_SEARCH_PIVOT_OMEGA_DEFAULT_RAD_S),
        CAMERA_SEARCH_PIVOT_OMEGA_DEFAULT_RAD_S,
    )
    if pivot_limit is None or float(pivot_limit) <= 0.0:
        pivot_limit = CAMERA_SEARCH_PIVOT_OMEGA_DEFAULT_RAD_S
    pivot_limit = max(
        float(CAMERA_SEARCH_PIVOT_OMEGA_MIN_RAD_S),
        min(float(CAMERA_SEARCH_PIVOT_OMEGA_MAX_RAD_S), float(pivot_limit)),
    )
    return float(min(float(cfg_limit), float(cap), float(pivot_limit)))


def _search_side_sign(side: Any) -> float:
    return -1.0 if str(side or "").strip().lower() == "right" else 1.0


def _extract_pose(ekf_state: Dict[str, Any]) -> Tuple[float, float, float]:
    state = dict(ekf_state or {})
    x = float(safe_float(state.get("x"), 0.0) or 0.0)
    y = float(safe_float(state.get("y"), 0.0) or 0.0)
    theta = state.get("theta")
    if theta is None:
        theta = math.radians(float(safe_float(state.get("theta_deg"), 0.0) or 0.0))
    else:
        theta = float(safe_float(theta, 0.0) or 0.0)
    return x, y, theta


def _robot_to_world(px: float, py: float, ptheta: float, rx: float, ry: float) -> Tuple[float, float]:
    cos_t = math.cos(ptheta)
    sin_t = math.sin(ptheta)
    return (
        px + cos_t * float(rx) - sin_t * float(ry),
        py + sin_t * float(rx) + cos_t * float(ry),
    )


def _observation_to_world(
    observation: TargetObservation,
    ekf_state: Dict[str, Any],
) -> Optional[Tuple[float, float, float, Optional[float], Optional[float], float]]:
    px, py, ptheta = _extract_pose(ekf_state)
    frame = normalize_frame(observation.frame)
    if frame == FRAME_WORLD:
        x = safe_float(observation.x, None)
        y = safe_float(observation.y, None)
        if x is None or y is None:
            return None
        theta = safe_float(observation.theta, None)
        if theta is None:
            theta = math.atan2(float(y) - py, float(x) - px)
        return (float(x), float(y), wrap_angle_rad(float(theta)), observation.vx, observation.vy, float(ptheta))

    if frame != FRAME_ROBOT:
        return None

    rx = safe_float(observation.x, None)
    ry = safe_float(observation.y, None)
    if rx is None or ry is None:
        dist = safe_float(observation.distance_m, None)
        bearing = safe_float(observation.bearing_rad, None)
        if dist is None or bearing is None:
            return None
        rx = float(dist) * math.cos(float(bearing))
        ry = float(dist) * math.sin(float(bearing))
    wx, wy = _robot_to_world(px, py, ptheta, float(rx), float(ry))
    theta = wrap_angle_rad(ptheta + float(safe_float(observation.bearing_rad, math.atan2(float(ry), float(rx))) or 0.0))
    vx = observation.vx
    vy = observation.vy
    if vx is not None or vy is not None:
        rvx = float(safe_float(vx, 0.0) or 0.0)
        rvy = float(safe_float(vy, 0.0) or 0.0)
        vx, vy = _robot_to_world(0.0, 0.0, ptheta, rvx, rvy)
    return (float(wx), float(wy), float(theta), vx, vy, float(ptheta))


def observation_from_payload(payload: Dict[str, Any], *, now_s: Optional[float] = None) -> TargetObservation:
    src = dict(payload or {})
    if "source" not in src and "target_source" not in src:
        src["target_source"] = TARGET_SOURCE_TARGET
    if "timestamp_s" not in src and "ts" not in src:
        src["timestamp_s"] = time.time() if now_s is None else float(now_s)
    return TargetObservation.from_dict(src, default_ts_s=(time.time() if now_s is None else float(now_s)))


def camera_observation_from_controller(ctrl: Any, *, now_s: Optional[float] = None) -> Optional[TargetObservation]:
    now = time.time() if now_s is None else float(now_s)
    follower_cfg = dict(getattr(ctrl, "follower_cfg", {}) or {})
    speed_scale = _follow_speed_scale(ctrl)
    v_max_mps = _scaled_limit(follower_cfg.get("max_v_target"), speed_scale)
    omega_max_rad_s = _scaled_limit(follower_cfg.get("max_omega"), speed_scale)
    if bool(getattr(ctrl, "_adaptive_target_search_active", False)):
        search_side = str(getattr(ctrl, "_adaptive_target_search_side", "") or "left").strip().lower()
        if search_side not in {"left", "right"}:
            search_side = "left"
        omega_max_rad_s = _search_omega_limit_for_camera(ctrl, omega_max_rad_s)
        direction_only = bool(
            safe_float(follower_cfg.get("target_distance_m"), 0.0) is not None
            and float(safe_float(follower_cfg.get("target_distance_m"), 0.0) or 0.0)
            >= CAMERA_DIRECTION_ONLY_DESIRED_DISTANCE_M
        )
        bearing_rad = _search_side_sign(search_side) * float(CAMERA_SEARCH_HEADING_DELTA_RAD)
        return TargetObservation(
            source=TARGET_SOURCE_CAMERA_SEARCH,
            frame=FRAME_ROBOT,
            timestamp_s=now,
            distance_m=0.0,
            bearing_rad=float(bearing_rad),
            confidence=1.0,
            desired_distance_m=0.0,
            v_max_mps=v_max_mps,
            omega_max_rad_s=omega_max_rad_s,
            target_id=f"camera_target_search_{search_side}{'_direction_only' if direction_only else ''}",
            target_zone=str(search_side),
        )
    dist = safe_float(getattr(ctrl, "_adaptive_target_dist_m", None), None)
    angle_deg = safe_float(getattr(ctrl, "_adaptive_target_angle_deg", None), None)
    if dist is None or angle_deg is None:
        return None
    last_seen = safe_float(getattr(ctrl, "_adaptive_target_last_seen_ts", None), now)
    confidence = safe_float(getattr(ctrl, "_adaptive_target_confidence", None), 1.0)
    desired = safe_float(getattr(ctrl, "_adaptive_target_desired_distance_m", None), None)
    if desired is None:
        desired = safe_float(follower_cfg.get("target_distance_m"), 1.2)
    if omega_max_rad_s is not None:
        omega_max_rad_s = min(float(omega_max_rad_s), CAMERA_TARGET_OMEGA_MAX_RAD_S)

    # Camera bbox convention in follower.py: negative = image-left, positive = image-right.
    # Robot/navigation convention: y and bearing are left-positive.
    bearing_rad = -math.radians(float(angle_deg))
    vx = safe_float(getattr(ctrl, "_adaptive_target_vx_mps", None), None)
    vy = safe_float(getattr(ctrl, "_adaptive_target_vy_mps", None), None)
    if vy is not None:
        vy = -float(vy)
    adaptive_state = str(getattr(ctrl, "_adaptive_follow_state", "") or "")
    camera_status = dict(getattr(ctrl, "_adaptive_target_camera_status", {}) or {})
    target_zone = str(camera_status.get("target_zone") or "").strip().lower()
    if target_zone not in {"left", "center", "right"}:
        target_zone = ""
    detector = str(camera_status.get("detector") or "")
    detector_confidence = safe_float(camera_status.get("detector_confidence"), 0.0) or 0.0
    front_hold_human_confirmed = bool(
        adaptive_state == "front_lidar_hold"
        and (
            detector in {"mediapipe_pose", "onnx_yolov5_person", "opencv_hog"}
            or (detector == "opencv_template_lock" and float(detector_confidence) >= 0.55)
            or (detector == "opencv_motion_blob" and float(detector_confidence) >= 0.65)
        )
        and bool(camera_status.get("target_usable", False))
        and not bool(camera_status.get("stale", False))
    )
    persisted_target = bool(
        adaptive_state in {"target_persistence_hold", "target_reacquire_hold"}
        or str(camera_status.get("gate") or "") in {"target_persistence_short_hold", "target_persistence_reacquire_hold"}
    )
    reacquire_target = bool(
        adaptive_state == "target_reacquire_hold"
        or str(camera_status.get("gate") or "") == "target_persistence_reacquire_hold"
    )
    front_obstacle_arbitrated = bool(
        str(camera_status.get("gate") or "") == "front_lidar_obstacle_arbitrated_by_camera_distance"
    )
    front_lidar_human_distance_source = bool(
        str(camera_status.get("distance_source") or "")
        in {
            "front_lidar_close_bubble_camera_confirmed",
            "front_lidar_room_bubble_camera_confirmed",
        }
    )
    front_obstacle_distance = safe_float(camera_status.get("front_obstacle_distance_m"), None)
    # Persistence/reacquire is a current arbiter decision over the last camera target,
    # bounded upstream by TargetObstacleArbiter.persistence_timeout_s. Keep the
    # follow/cruise status chain alive so it can hold or scan safely instead of
    # dropping to an empty target source mid-reacquire.
    observation_ts = now if persisted_target else (last_seen if last_seen is not None else now)
    return TargetObservation(
        source=TARGET_SOURCE_CAMERA_TARGET,
        frame=FRAME_ROBOT,
        timestamp_s=float(observation_ts),
        distance_m=float(dist),
        bearing_rad=float(bearing_rad),
        vx=vx,
        vy=vy,
        confidence=float(confidence if confidence is not None else 1.0),
        desired_distance_m=float(desired if desired is not None else 1.2),
        v_max_mps=v_max_mps,
        omega_max_rad_s=omega_max_rad_s,
        target_id=(
            "camera_front_lidar_hold_human_confirmed"
            if front_hold_human_confirmed or front_lidar_human_distance_source
            else (
                "camera_front_lidar_hold"
                if adaptive_state == "front_lidar_hold"
                else (
                    "camera_front_obstacle_arbitrated"
                    if front_obstacle_arbitrated
                    else (
                        "camera_target_reacquire"
                        if reacquire_target
                        else ("camera_target_persisted" if persisted_target else "camera_target")
                    )
                )
            )
        ),
        front_obstacle_distance_m=front_obstacle_distance,
        target_zone=target_zone,
    )


class FollowLayer:
    def __init__(self, config: FollowLayerConfig | None = None):
        self.cfg = config if config is not None else FollowLayerConfig()

    def tick(
        self,
        observation: TargetObservation | Dict[str, Any] | None,
        ekf_state: Dict[str, Any],
        *,
        source: str = "STATE",
        now_s: Optional[float] = None,
    ) -> FollowRequest:
        now = time.time() if now_s is None else float(now_s)
        if observation is None:
            return self._inactive(source=source, reason="no_target_observation")
        if isinstance(observation, dict):
            observation = TargetObservation.from_dict(observation, default_ts_s=now)

        target_source = normalize_target_source(observation.source)
        confidence = max(0.0, min(1.0, float(safe_float(observation.confidence, 0.0) or 0.0)))
        age_s = max(0.0, now - float(safe_float(observation.timestamp_s, now) or now))
        if target_source == TARGET_SOURCE_CAMERA_SEARCH or str(observation.target_id or "") == "camera_target_search":
            robot_x, robot_y, robot_theta = _extract_pose(ekf_state)
            search_side = "right" if "_right" in str(observation.target_id or "").lower() else "left"
            search_heading = wrap_angle_rad(
                float(robot_theta)
                + (_search_side_sign(search_side) * float(CAMERA_SEARCH_HEADING_DELTA_RAD))
            )
            return FollowRequest(
                active=True,
                source=str(source or "STATE"),
                target_source=TARGET_SOURCE_CAMERA_SEARCH,
                target_x=float(robot_x),
                target_y=float(robot_y),
                target_theta=float(search_heading),
                goal_x=float(robot_x),
                goal_y=float(robot_y),
                goal_theta=float(search_heading),
                distance_to_target_m=0.0,
                desired_distance_m=0.0,
                age_s=float(age_s),
                confidence=float(confidence),
                stale=False,
                reason="target_search_scan",
                v_max_mps=safe_float(observation.v_max_mps, None),
                omega_max_rad_s=safe_float(observation.omega_max_rad_s, None),
                target_id=str(observation.target_id or "camera_target_search"),
                target_zone=str(observation.target_zone or search_side),
            )
        if age_s > float(self.cfg.max_target_age_s):
            return self._inactive(
                source=source,
                target_source=target_source,
                reason="target_stale",
                age_s=age_s,
                confidence=confidence,
                stale=True,
            )
        if confidence < float(self.cfg.min_confidence):
            return self._inactive(
                source=source,
                target_source=target_source,
                reason="low_confidence",
                age_s=age_s,
                confidence=confidence,
            )

        world = _observation_to_world(observation, ekf_state)
        if world is None:
            return self._inactive(
                source=source,
                target_source=target_source,
                reason="invalid_target_geometry",
                age_s=age_s,
                confidence=confidence,
            )
        target_x, target_y, target_theta, target_vx, target_vy, robot_theta = world
        robot_x, robot_y, _ = _extract_pose(ekf_state)
        dx = float(target_x) - float(robot_x)
        dy = float(target_y) - float(robot_y)
        dist = math.hypot(dx, dy)
        ux = dx / max(1e-6, dist)
        uy = dy / max(1e-6, dist)

        default_desired = self.cfg.camera_desired_distance_m if target_source == TARGET_SOURCE_CAMERA_TARGET else self.cfg.default_desired_distance_m
        desired = safe_float(observation.desired_distance_m, default_desired)
        desired = max(0.0, float(desired if desired is not None else 0.0))

        target_vx_mps = safe_float(target_vx, None)
        target_vy_mps = safe_float(target_vy, None)
        radial_target_velocity_mps = 0.0
        if dist > 1e-6 and (target_vx_mps is not None or target_vy_mps is not None):
            radial_target_velocity_mps = (
                float(target_vx_mps or 0.0) * ux
                + float(target_vy_mps or 0.0) * uy
            )

        if dist <= desired + float(self.cfg.standoff_hold_margin_m):
            goal_theta = math.atan2(dy, dx) if dist > 1e-6 else robot_theta
            predicted_gap_m = dist + max(0.0, radial_target_velocity_mps) * FOLLOW_STANDOFF_RECEDE_HORIZON_S
            recede_pressure_m = predicted_gap_m - max(0.0, desired - FOLLOW_STANDOFF_RECEDE_SLACK_M)
            max_nudge_m = max(0.0, dist - max(0.0, desired - FOLLOW_STANDOFF_RECEDE_INNER_MARGIN_M))
            if (
                desired > 0.0
                and radial_target_velocity_mps >= FOLLOW_STANDOFF_RECEDE_MIN_SPEED_MPS
                and recede_pressure_m > 0.0
                and max_nudge_m >= FOLLOW_STANDOFF_RECEDE_GOAL_MIN_M
            ):
                nudge_m = min(
                    FOLLOW_STANDOFF_RECEDE_GOAL_MAX_M,
                    max_nudge_m,
                    max(FOLLOW_STANDOFF_RECEDE_GOAL_MIN_M, recede_pressure_m),
                )
                goal_x = float(robot_x) + ux * nudge_m
                goal_y = float(robot_y) + uy * nudge_m
                reason = "inside_follow_standoff_receding"
            else:
                goal_x = robot_x
                goal_y = robot_y
                reason = "inside_follow_standoff"
        else:
            goal_x = float(target_x) - ux * desired
            goal_y = float(target_y) - uy * desired
            goal_theta = math.atan2(float(target_y) - goal_y, float(target_x) - goal_x)
            reason = "follow_goal_ready"

        return FollowRequest(
            active=True,
            source=str(source or "STATE"),
            target_source=target_source,
            target_x=float(target_x),
            target_y=float(target_y),
            target_theta=float(target_theta),
            target_vx=safe_float(target_vx, None),
            target_vy=safe_float(target_vy, None),
            goal_x=float(goal_x),
            goal_y=float(goal_y),
            goal_theta=wrap_angle_rad(float(goal_theta)),
            distance_to_target_m=float(dist),
            desired_distance_m=float(desired),
            age_s=float(age_s),
            confidence=float(confidence),
            stale=False,
            reason=reason,
            v_max_mps=safe_float(observation.v_max_mps, None),
            omega_max_rad_s=safe_float(observation.omega_max_rad_s, None),
            target_id=str(observation.target_id or ""),
            front_obstacle_distance_m=safe_float(observation.front_obstacle_distance_m, None),
            target_zone=str(observation.target_zone or ""),
        )

    def _inactive(
        self,
        *,
        source: str,
        target_source: str = TARGET_SOURCE_TARGET,
        reason: str,
        age_s: Optional[float] = None,
        confidence: float = 0.0,
        stale: bool = False,
    ) -> FollowRequest:
        return FollowRequest(
            active=False,
            source=str(source or "STATE"),
            target_source=normalize_target_source(target_source),
            reason=str(reason or "inactive"),
            age_s=age_s,
            confidence=float(confidence),
            stale=bool(stale),
        )
