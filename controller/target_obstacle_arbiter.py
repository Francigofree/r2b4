#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Target/obstacle arbitration for camera-based person following.

This layer decides whether a front LIDAR return is the human target, a separate
obstacle, or a condition that should switch the follower into search/persistence.
It emits perception decisions only; motor output remains owned by cruise/planner.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class TargetObstacleArbiterConfig:
    front_hold_distance_m: float = 0.76
    camera_obstacle_min_confidence: float = 0.20
    camera_obstacle_min_delta_m: float = 0.35
    camera_obstacle_min_target_distance_m: float = 1.05
    persistence_short_hold_s: float = 0.50
    persistence_timeout_s: float = 1.60
    persistence_confidence_scale: float = 0.50


@dataclass(frozen=True)
class TargetObstacleDecision:
    mode: str
    reason: str
    camera_updates: Dict[str, Any] = field(default_factory=dict)
    lidar_status: Dict[str, Any] = field(default_factory=dict)
    target_distance_m: Optional[float] = None
    target_angle_deg: Optional[float] = None
    target_confidence: float = 0.0
    target_vx_mps: Optional[float] = None
    target_vy_mps: Optional[float] = None
    allow_follow_target: bool = False
    allow_forward: bool = False
    search_required: bool = False
    clear_tracker: bool = False


def _float_or_none(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    if out != out or out in (float("inf"), float("-inf")):
        return None
    return out


def _front_camera_match_delta_m(desired_distance_m: float) -> float:
    desired = max(0.0, _float_or_none(desired_distance_m) or 0.0)
    return max(0.16, min(0.30, desired * 0.45))


class TargetObstacleArbiter:
    def __init__(self, config: TargetObstacleArbiterConfig | None = None):
        self.cfg = config if config is not None else TargetObstacleArbiterConfig()

    def camera_can_arbitrate_front_obstacle(
        self,
        *,
        front_distance_m: Optional[float],
        camera_distance_m: Optional[float],
        camera_distance_confidence: float,
    ) -> bool:
        front = _float_or_none(front_distance_m)
        target = _float_or_none(camera_distance_m)
        conf = float(max(0.0, min(1.0, _float_or_none(camera_distance_confidence) or 0.0)))
        if front is None or target is None:
            return False
        return bool(
            front < float(self.cfg.front_hold_distance_m)
            and conf >= float(self.cfg.camera_obstacle_min_confidence)
            and target >= float(self.cfg.camera_obstacle_min_target_distance_m)
            and (target - front) >= float(self.cfg.camera_obstacle_min_delta_m)
        )

    def camera_front_return_matches_target(
        self,
        *,
        front_distance_m: Optional[float],
        desired_distance_m: float,
        target_confidence: float,
        camera_status: Dict[str, Any],
        camera_distance_m: Optional[float],
        camera_distance_confidence: float,
    ) -> bool:
        front = _float_or_none(front_distance_m)
        target = _float_or_none(camera_distance_m)
        desired = max(0.0, _float_or_none(desired_distance_m) or 0.0)
        target_conf = float(max(0.0, min(1.0, _float_or_none(target_confidence) or 0.0)))
        dist_conf = float(max(0.0, min(1.0, _float_or_none(camera_distance_confidence) or 0.0)))
        status = dict(camera_status or {})
        detector = str(status.get("detector") or "")
        visible = bool(status.get("target_visible", False)) and bool(status.get("target_usable", False))
        if front is None or target is None:
            return False
        return bool(
            desired > 0.0
            and desired < float(self.cfg.front_hold_distance_m)
            and front < float(self.cfg.front_hold_distance_m)
            and visible
            and not bool(status.get("stale", False))
            and detector not in {"", "none", "unknown"}
            and target_conf >= float(self.cfg.camera_obstacle_min_confidence)
            and dist_conf >= float(self.cfg.camera_obstacle_min_confidence)
            and target >= max(0.30, desired - 0.10)
            and abs(target - front) <= _front_camera_match_delta_m(desired)
        )

    def decide_front_conflict(
        self,
        *,
        front_distance_m: float,
        desired_distance_m: float,
        target_angle_deg: float,
        target_confidence: float,
        camera_status: Dict[str, Any],
        camera_distance_m: Optional[float],
        camera_distance_confidence: float,
        lidar_snapshot_age_s: Optional[float],
        lidar_missing: bool,
    ) -> TargetObstacleDecision:
        front = _float_or_none(front_distance_m)
        camera_target = _float_or_none(camera_distance_m)
        desired = max(0.0, _float_or_none(desired_distance_m) or 0.0)
        camera_status_src = dict(camera_status or {})
        detector = str(camera_status_src.get("detector") or "")
        visible = bool(camera_status_src.get("target_visible", False)) and bool(camera_status_src.get("target_usable", False))
        target_conf = float(max(0.0, min(1.0, _float_or_none(target_confidence) or 0.0)))
        dist_conf = float(max(0.0, min(1.0, _float_or_none(camera_distance_confidence) or 0.0)))
        close_bubble_margin_m = 0.30 if desired <= 0.70 else 0.18
        if self.camera_can_arbitrate_front_obstacle(
            front_distance_m=front_distance_m,
            camera_distance_m=camera_distance_m,
            camera_distance_confidence=camera_distance_confidence,
        ):
            target_dist = float(camera_distance_m)  # type: ignore[arg-type]
            return TargetObstacleDecision(
                mode="front_obstacle_arbitrated",
                reason="camera_target_behind_front_lidar_obstacle",
                target_distance_m=target_dist,
                target_angle_deg=float(target_angle_deg),
                target_confidence=float(target_confidence) * max(0.10, float(camera_distance_confidence)),
                allow_follow_target=True,
                allow_forward=True,
                camera_updates={
                    "gate": "front_lidar_obstacle_arbitrated_by_camera_distance",
                    "front_obstacle_distance_m": float(front_distance_m),
                    "distance_used_m": target_dist,
                    "distance_source": "camera_bbox_front_obstacle_arbitrated",
                    "camera_lidar_delta_m": target_dist - float(front_distance_m),
                    "target_obstacle_arbiter_mode": "front_obstacle_arbitrated",
                    "forward_allowed": True,
                },
                lidar_status=self._lidar_status(
                    state="front_obstacle_arbitrated",
                    source="front_lidar_obstacle_camera_target_distance",
                    distance_m=front_distance_m,
                    age_s=lidar_snapshot_age_s,
                    missing=lidar_missing,
                ),
            )
        if (
            front is not None
            and camera_target is not None
            and desired > 0.0
            and desired < float(self.cfg.front_hold_distance_m)
            and front <= desired + close_bubble_margin_m
            and front >= max(0.30, desired - 0.08)
            and visible
            and not bool(camera_status_src.get("stale", False))
            and detector not in {"", "none", "unknown"}
            and target_conf >= float(self.cfg.camera_obstacle_min_confidence)
            and dist_conf >= float(self.cfg.camera_obstacle_min_confidence)
            and camera_target >= max(0.30, desired - 0.10)
            and abs(float(camera_target) - float(front)) <= _front_camera_match_delta_m(desired)
        ):
            allow_forward = bool(
                front >= max(0.72 if desired <= 0.70 else 0.56, desired + 0.24)
                and camera_target > desired + 0.05
            )
            return TargetObstacleDecision(
                mode="front_target_confirmed",
                reason="front_lidar_close_bubble_matches_camera_target",
                target_distance_m=float(front),
                target_angle_deg=float(target_angle_deg),
                target_confidence=float(target_confidence) * max(0.10, float(camera_distance_confidence)),
                allow_follow_target=True,
                allow_forward=allow_forward,
                camera_updates={
                    "gate": "front_lidar_target_confirmed_by_camera",
                    "front_obstacle_distance_m": float(front),
                    "distance_used_m": float(front),
                    "distance_source": "front_lidar_close_bubble_camera_confirmed",
                    "camera_lidar_delta_m": float(camera_target) - float(front),
                    "target_obstacle_arbiter_mode": "front_target_confirmed",
                    "forward_allowed": bool(allow_forward),
                },
                lidar_status=self._lidar_status(
                    state="front_target_confirmed",
                    source="front_lidar_close_bubble_camera_confirmed",
                    distance_m=front,
                    age_s=lidar_snapshot_age_s,
                    missing=lidar_missing,
                ),
            )

        if self.camera_front_return_matches_target(
            front_distance_m=front_distance_m,
            desired_distance_m=desired_distance_m,
            target_confidence=target_confidence,
            camera_status=dict(camera_status or {}),
            camera_distance_m=camera_distance_m,
            camera_distance_confidence=camera_distance_confidence,
        ):
            target_dist = float(camera_distance_m)  # type: ignore[arg-type]
            allow_forward = bool(
                float(front_distance_m) >= max(0.56, float(desired_distance_m) + 0.08)
                and target_dist > float(desired_distance_m) + 0.05
            )
            return TargetObstacleDecision(
                mode="front_target_confirmed",
                reason="front_lidar_return_matches_camera_target",
                target_distance_m=target_dist,
                target_angle_deg=float(target_angle_deg),
                target_confidence=float(target_confidence) * max(0.10, float(camera_distance_confidence)),
                allow_follow_target=True,
                allow_forward=allow_forward,
                camera_updates={
                    "gate": "front_lidar_target_confirmed_by_camera",
                    "front_obstacle_distance_m": float(front_distance_m),
                    "distance_used_m": target_dist,
                    "distance_source": "camera_bbox_front_target_confirmed",
                    "camera_lidar_delta_m": target_dist - float(front_distance_m),
                    "target_obstacle_arbiter_mode": "front_target_confirmed",
                    "forward_allowed": bool(allow_forward),
                },
                lidar_status=self._lidar_status(
                    state="front_target_confirmed",
                    source="front_lidar_camera_target_match",
                    distance_m=front_distance_m,
                    age_s=lidar_snapshot_age_s,
                    missing=lidar_missing,
                ),
            )

        hold_dist = max(0.0, min(float(desired_distance_m), float(front_distance_m)))
        return TargetObstacleDecision(
            mode="front_hold",
            reason="front_lidar_follow_hold",
            target_distance_m=hold_dist,
            target_angle_deg=float(target_angle_deg),
            target_confidence=float(max(0.0, min(1.0, target_confidence))),
            target_vx_mps=0.0,
            target_vy_mps=0.0,
            allow_follow_target=True,
            allow_forward=False,
            camera_updates={
                "raw_state": str((camera_status or {}).get("state") or ""),
                "state": "front_lidar_hold",
                "target_usable": True,
                "gate": "front_lidar_follow_hold",
                "front_hold_distance_m": float(front_distance_m),
                "front_hold_target_angle_deg": float(target_angle_deg),
                "target_obstacle_arbiter_mode": "front_hold",
                "forward_allowed": False,
            },
            lidar_status=self._lidar_status(
                state="front_hold",
                source="front_lidar_follow_hold",
                distance_m=front_distance_m,
                age_s=lidar_snapshot_age_s,
                missing=lidar_missing,
            ),
        )

    def decide_target_loss(
        self,
        *,
        camera_status: Dict[str, Any],
        previous_target: Optional[Dict[str, Any]],
        desired_distance_m: float,
    ) -> TargetObstacleDecision:
        status = dict(camera_status or {})
        age_s = _float_or_none(status.get("age_s"))
        if age_s is None and previous_target is not None:
            age_s = _float_or_none(previous_target.get("age_s"))
        if age_s is None and previous_target is not None:
            last_seen_ts = _float_or_none(previous_target.get("last_seen_ts"))
            if last_seen_ts is not None:
                age_s = max(0.0, time.time() - float(last_seen_ts))
        if previous_target is not None and (age_s is None or age_s <= float(self.cfg.persistence_timeout_s)):
            prev_dist = _float_or_none(previous_target.get("dist_m"))
            prev_angle = _float_or_none(previous_target.get("angle_deg"))
            prev_conf = _float_or_none(previous_target.get("confidence")) or 0.0
            if prev_dist is not None and prev_angle is not None:
                hold_dist = max(0.0, float(desired_distance_m))
                short_hold = bool(age_s is None or float(age_s) <= float(self.cfg.persistence_short_hold_s))
                gate = "target_persistence_short_hold" if short_hold else "target_persistence_reacquire_hold"
                state = "target_persistence_hold" if short_hold else "target_reacquire_hold"
                return TargetObstacleDecision(
                    mode="target_persistence_hold" if short_hold else "target_reacquire_hold",
                    reason="target_short_dropout_persistence",
                    target_distance_m=hold_dist,
                    target_angle_deg=float(prev_angle),
                    target_confidence=max(0.05, min(1.0, prev_conf * float(self.cfg.persistence_confidence_scale))),
                    target_vx_mps=0.0,
                    target_vy_mps=0.0,
                    allow_follow_target=True,
                    allow_forward=False,
                    camera_updates={
                        "gate": str(gate),
                        "state": str(state),
                        "target_visible": False,
                        "target_usable": True,
                        "stale": True,
                        "age_s": None if age_s is None else float(age_s),
                        "distance_used_m": hold_dist,
                        "distance_source": "tracker_persistence_hold",
                        "target_obstacle_arbiter_mode": str(state),
                        "forward_allowed": False,
                    },
                    lidar_status=self._lidar_status(
                        state="held",
                        source="tracker_hold_camera_stale",
                        distance_m=None,
                        age_s=None,
                        missing=False,
                    ),
                )

        return TargetObstacleDecision(
            mode="target_lost_search",
            reason="target_lost_search",
            search_required=True,
            clear_tracker=True,
            allow_follow_target=False,
            allow_forward=False,
            camera_updates={
                "gate": "target_lost_search",
                "target_obstacle_arbiter_mode": "target_lost_search",
                "forward_allowed": False,
            },
        )

    def _lidar_status(
        self,
        *,
        state: str,
        source: str,
        distance_m: Optional[float],
        age_s: Optional[float],
        missing: bool,
    ) -> Dict[str, Any]:
        return {
            "state": str(state),
            "source": str(source),
            "usable_distance": False,
            "stale": False,
            "missing": bool(missing),
            "age_s": None if age_s is None else float(age_s),
            "confidence": 0.0,
            "distance_m": None if distance_m is None else float(distance_m),
            "point_count": 0,
            "cluster_points": 0,
        }
