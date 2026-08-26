#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Navigation intent contracts.

Behavior layers describe *what* the robot should do here.  They do not choose
PWM, tracks, or final v/omega.  Local navigation consumes this contract.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

from controller.follow_types import FollowRequest, safe_float


NAV_INTENT_SCHEMA_VERSION = "NAVIGATION_INTENT_V1"

NAV_MODE_GOAL = "GOAL"
NAV_MODE_FOLLOW = "FOLLOW"
NAV_MODE_ROOM_CRUISE = "ROOM_CRUISE"
NAV_MODE_EXPLORE = "EXPLORE"
NAV_MODE_HOLD = "HOLD"
NAV_MODES = frozenset({NAV_MODE_GOAL, NAV_MODE_FOLLOW, NAV_MODE_ROOM_CRUISE, NAV_MODE_EXPLORE, NAV_MODE_HOLD})


def normalize_nav_mode(value: Any, *, fallback: str = NAV_MODE_GOAL) -> str:
    mode = str(value or "").strip().upper()
    if mode in NAV_MODES:
        return mode
    fb = str(fallback or NAV_MODE_GOAL).strip().upper()
    return fb if fb in NAV_MODES else NAV_MODE_GOAL


def _finite_float(value: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        if value is None:
            return default
        out = float(value)
    except Exception:
        return default
    if not math.isfinite(out):
        return default
    return float(out)


@dataclass(frozen=True)
class NavigationIntent:
    active: bool
    source: str
    behavior: str
    mode: str = NAV_MODE_GOAL
    command_type: str = "navigation_intent"
    goal_x: Optional[float] = None
    goal_y: Optional[float] = None
    goal_theta: Optional[float] = None
    desired_speed_mps: Optional[float] = None
    max_v_mps: Optional[float] = None
    max_omega_rad_s: Optional[float] = None
    standoff_m: Optional[float] = None
    priority: int = 795
    reason: str = ""
    metadata: Dict[str, Any] = field(default_factory=dict)

    def normalized_mode(self) -> str:
        return normalize_nav_mode(self.mode)

    def target_pose(self) -> Optional[Tuple[float, float, float]]:
        if not self.active or self.normalized_mode() == NAV_MODE_HOLD:
            return None
        x = _finite_float(self.goal_x, None)
        y = _finite_float(self.goal_y, None)
        theta = _finite_float(self.goal_theta, None)
        if x is None or y is None or theta is None:
            return None
        return float(x), float(y), float(theta)

    def to_dict(self) -> Dict[str, Any]:
        target = self.target_pose()
        return {
            "schema": NAV_INTENT_SCHEMA_VERSION,
            "active": bool(self.active),
            "source": str(self.source or ""),
            "behavior": str(self.behavior or ""),
            "mode": self.normalized_mode(),
            "command_type": str(self.command_type or "navigation_intent"),
            "goal": (
                None
                if target is None
                else {
                    "x": float(target[0]),
                    "y": float(target[1]),
                    "theta_rad": float(target[2]),
                }
            ),
            "desired_speed_mps": _finite_float(self.desired_speed_mps, None),
            "max_v_mps": _finite_float(self.max_v_mps, None),
            "max_omega_rad_s": _finite_float(self.max_omega_rad_s, None),
            "standoff_m": _finite_float(self.standoff_m, None),
            "priority": int(self.priority),
            "reason": str(self.reason or ""),
            "metadata": dict(self.metadata or {}),
        }

    @classmethod
    def hold(
        cls,
        *,
        source: str = "STATE",
        behavior: str = "HOLD",
        reason: str = "hold",
        priority: int = 795,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> "NavigationIntent":
        return cls(
            active=True,
            source=str(source or "STATE"),
            behavior=str(behavior or "HOLD"),
            mode=NAV_MODE_HOLD,
            command_type="navigation_hold",
            priority=int(priority),
            reason=str(reason or "hold"),
            metadata=dict(metadata or {}),
        )

    @classmethod
    def explore(
        cls,
        *,
        source: str = "STATE",
        desired_speed_mps: float = 0.05,
        max_v_mps: Optional[float] = None,
        max_omega_rad_s: Optional[float] = None,
        priority: int = 795,
        reason: str = "explore",
        metadata: Optional[Dict[str, Any]] = None,
    ) -> "NavigationIntent":
        return cls(
            active=True,
            source=str(source or "STATE"),
            behavior="EXPLORE",
            mode=NAV_MODE_EXPLORE,
            command_type="explore",
            desired_speed_mps=float(desired_speed_mps),
            max_v_mps=max_v_mps,
            max_omega_rad_s=max_omega_rad_s,
            priority=int(priority),
            reason=str(reason or "explore"),
            metadata=dict(metadata or {}),
        )

    @classmethod
    def from_follow_request(
        cls,
        request: FollowRequest,
        *,
        behavior: str = "HUMAN_FOLLOW",
        mode: str = NAV_MODE_ROOM_CRUISE,
        source: Optional[str] = None,
        priority: int = 810,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> "NavigationIntent":
        target_pose = request.target_pose() if isinstance(request, FollowRequest) else None
        if not isinstance(request, FollowRequest) or not request.active or target_pose is None:
            return cls(
                active=False,
                source=str(source or getattr(request, "source", "STATE") or "STATE"),
                behavior=str(behavior or "HUMAN_FOLLOW"),
                mode=normalize_nav_mode(mode, fallback=NAV_MODE_ROOM_CRUISE),
                command_type="follow_navigation_intent",
                priority=int(priority),
                reason=str(getattr(request, "reason", "") or "follow_request_inactive"),
                metadata=dict(metadata or {}),
            )
        gx, gy, gt = target_pose
        max_v = safe_float(getattr(request, "v_max_mps", None), None)
        max_w = safe_float(getattr(request, "omega_max_rad_s", None), None)
        standoff = safe_float(getattr(request, "desired_distance_m", None), None)
        return cls(
            active=True,
            source=str(source or request.source or "STATE"),
            behavior=str(behavior or "HUMAN_FOLLOW"),
            mode=normalize_nav_mode(mode, fallback=NAV_MODE_ROOM_CRUISE),
            command_type="follow_navigation_intent",
            goal_x=float(gx),
            goal_y=float(gy),
            goal_theta=float(gt),
            desired_speed_mps=max_v,
            max_v_mps=max_v,
            max_omega_rad_s=max_w,
            standoff_m=standoff,
            priority=int(priority),
            reason=str(request.reason or "follow_goal_ready"),
            metadata={
                **dict(metadata or {}),
                "target_source": str(request.target_source or ""),
                "target_id": str(request.target_id or ""),
                "target_zone": str(request.target_zone or ""),
                "distance_to_target_m": safe_float(request.distance_to_target_m, None),
            },
        )
