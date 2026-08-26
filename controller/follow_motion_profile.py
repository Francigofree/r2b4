#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Human-follow motion profiling.

This module shapes a camera target into smoother follow-intent constraints.
It does not emit motor, track, PWM, or resolver proposals.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

from controller.follow_types import FollowRequest, safe_float


FOLLOW_PROFILE_PROVIDER = "follow_motion_profile"
FOLLOW_PROFILE_PHASE_ACQUIRE_ALIGN = "acquire_align"
FOLLOW_PROFILE_PHASE_APPROACH_FAR = "approach_far"
FOLLOW_PROFILE_PHASE_APPROACH_SETTLE = "approach_settle"
FOLLOW_PROFILE_PHASE_STANDOFF_ALIGN = "standoff_align"
FOLLOW_PROFILE_PHASE_STANDOFF_HOLD = "standoff_hold"
FOLLOW_PROFILE_PHASE_CLOSE_RETREAT = "close_retreat"
FOLLOW_PROFILE_PHASE_DIRECTION_ONLY = "direction_only"

DISTANCE_TAU_S = 0.45
BEARING_TAU_S = 0.35
MAX_DT_S = 0.25
ALIGN_ENTER_RAD = 0.34
ALIGN_EXIT_RAD = 0.24
STANDOFF_ALIGN_RAD = 0.16
APPROACH_FAR_M = 0.30
APPROACH_SETTLE_M = 0.08
CLOSE_RETREAT_M = -0.12
DIRECTION_ONLY_STANDOFF_M = 2.0


def _wrap_angle(rad: float) -> float:
    return (float(rad) + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(value: float, low: float, high: float) -> float:
    return max(float(low), min(float(high), float(value)))


def _finite(value: Any, default: Optional[float] = None) -> Optional[float]:
    out = safe_float(value, default)
    if out is None:
        return default
    return float(out)


def _extract_pose(ekf_state: Dict[str, Any]) -> Tuple[float, float, float]:
    src = dict(ekf_state or {})
    x = _finite(src.get("x"), 0.0) or 0.0
    y = _finite(src.get("y"), 0.0) or 0.0
    theta = _finite(src.get("theta"), None)
    if theta is None:
        theta = math.radians(_finite(src.get("theta_deg"), 0.0) or 0.0)
    return float(x), float(y), float(theta)


def _alpha(dt_s: float, tau_s: float) -> float:
    dt = _clamp(float(dt_s), 0.0, MAX_DT_S)
    tau = max(1e-6, float(tau_s))
    return _clamp(dt / tau, 0.05, 1.0)


@dataclass
class FollowMotionProfileResult:
    goal_x: Optional[float]
    goal_y: Optional[float]
    goal_theta: Optional[float]
    distance_to_target_m: Optional[float]
    max_v_mps: Optional[float]
    max_omega_rad_s: Optional[float]
    phase: str
    status: Dict[str, Any] = field(default_factory=dict)


class FollowMotionProfile:
    """Stateful profile for live camera human-follow intent constraints."""

    def __init__(self) -> None:
        self._last_ts: Optional[float] = None
        self._target_key: str = ""
        self._distance_m: Optional[float] = None
        self._bearing_rad: Optional[float] = None
        self._phase: str = ""

    def reset(self) -> None:
        self._last_ts = None
        self._target_key = ""
        self._distance_m = None
        self._bearing_rad = None
        self._phase = ""

    def tick(
        self,
        request: FollowRequest,
        ekf_state: Dict[str, Any],
        *,
        now_s: Optional[float] = None,
    ) -> FollowMotionProfileResult:
        now = float(now_s) if now_s is not None else 0.0
        robot_x, robot_y, robot_theta = _extract_pose(ekf_state)
        target_key = "{}:{}".format(
            str(getattr(request, "target_source", "") or ""),
            str(getattr(request, "target_id", "") or ""),
        )
        reset = bool(target_key != self._target_key)
        raw_distance = _finite(getattr(request, "distance_to_target_m", None), None)
        raw_bearing = self._request_bearing(request, robot_x, robot_y, robot_theta)

        if raw_distance is None or raw_bearing is None:
            self.reset()
            return self._passthrough(request, reason="missing_target_geometry")

        if reset or self._last_ts is None or now <= 0.0:
            filtered_distance = float(raw_distance)
            filtered_bearing = float(raw_bearing)
        else:
            dt_s = max(0.0, float(now) - float(self._last_ts))
            dist_alpha = _alpha(dt_s, DISTANCE_TAU_S)
            bearing_alpha = _alpha(dt_s, BEARING_TAU_S)
            prev_distance = float(self._distance_m if self._distance_m is not None else raw_distance)
            prev_bearing = float(self._bearing_rad if self._bearing_rad is not None else raw_bearing)
            filtered_distance = prev_distance + (float(raw_distance) - prev_distance) * dist_alpha
            filtered_bearing = _wrap_angle(prev_bearing + _wrap_angle(float(raw_bearing) - prev_bearing) * bearing_alpha)

        desired = max(0.0, float(_finite(getattr(request, "desired_distance_m", None), 0.0) or 0.0))
        distance_error = float(filtered_distance) - float(desired)
        abs_bearing = abs(float(filtered_bearing))
        phase = self._phase_for(distance_error, abs_bearing, desired)
        max_v, max_w = self._limits_for(
            request=request,
            phase=phase,
            distance_error_m=distance_error,
            abs_bearing_rad=abs_bearing,
        )

        goal_dist = max(0.0, float(filtered_distance) - float(desired))
        world_heading = _wrap_angle(float(robot_theta) + float(filtered_bearing))
        goal_x = float(robot_x) + math.cos(world_heading) * goal_dist
        goal_y = float(robot_y) + math.sin(world_heading) * goal_dist
        goal_theta = world_heading

        self._last_ts = now if now > 0.0 else self._last_ts
        self._target_key = target_key
        self._distance_m = float(filtered_distance)
        self._bearing_rad = float(filtered_bearing)
        self._phase = str(phase)

        status = {
            "active": True,
            "provider": FOLLOW_PROFILE_PROVIDER,
            "phase": str(phase),
            "target_key": str(target_key),
            "raw_distance_m": round(float(raw_distance), 4),
            "filtered_distance_m": round(float(filtered_distance), 4),
            "distance_error_m": round(float(distance_error), 4),
            "raw_bearing_rad": round(float(raw_bearing), 4),
            "filtered_bearing_rad": round(float(filtered_bearing), 4),
            "raw_bearing_abs_deg": round(abs(math.degrees(float(raw_bearing))), 3),
            "filtered_bearing_abs_deg": round(abs(math.degrees(float(filtered_bearing))), 3),
            "desired_distance_m": round(float(desired), 4),
            "max_v_before_mps": _finite(getattr(request, "v_max_mps", None), None),
            "max_v_after_mps": max_v,
            "max_omega_before_rad_s": _finite(getattr(request, "omega_max_rad_s", None), None),
            "max_omega_after_rad_s": max_w,
            "reset": bool(reset),
        }
        return FollowMotionProfileResult(
            goal_x=float(goal_x),
            goal_y=float(goal_y),
            goal_theta=float(goal_theta),
            distance_to_target_m=float(filtered_distance),
            max_v_mps=max_v,
            max_omega_rad_s=max_w,
            phase=str(phase),
            status=status,
        )

    def _request_bearing(
        self,
        request: FollowRequest,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
    ) -> Optional[float]:
        tx = _finite(getattr(request, "target_x", None), None)
        ty = _finite(getattr(request, "target_y", None), None)
        if tx is not None and ty is not None:
            return _wrap_angle(math.atan2(float(ty) - float(robot_y), float(tx) - float(robot_x)) - float(robot_theta))
        goal_theta = _finite(getattr(request, "goal_theta", None), None)
        if goal_theta is None:
            return None
        return _wrap_angle(float(goal_theta) - float(robot_theta))

    def _phase_for(self, distance_error_m: float, abs_bearing_rad: float, desired_m: float) -> str:
        if desired_m >= DIRECTION_ONLY_STANDOFF_M:
            return FOLLOW_PROFILE_PHASE_DIRECTION_ONLY
        if distance_error_m <= CLOSE_RETREAT_M:
            return FOLLOW_PROFILE_PHASE_CLOSE_RETREAT
        if self._phase == FOLLOW_PROFILE_PHASE_ACQUIRE_ALIGN and abs_bearing_rad > ALIGN_EXIT_RAD:
            return FOLLOW_PROFILE_PHASE_ACQUIRE_ALIGN
        if abs_bearing_rad >= ALIGN_ENTER_RAD and distance_error_m > APPROACH_SETTLE_M:
            return FOLLOW_PROFILE_PHASE_ACQUIRE_ALIGN
        if distance_error_m > APPROACH_FAR_M:
            return FOLLOW_PROFILE_PHASE_APPROACH_FAR
        if distance_error_m > APPROACH_SETTLE_M:
            return FOLLOW_PROFILE_PHASE_APPROACH_SETTLE
        if abs_bearing_rad > STANDOFF_ALIGN_RAD:
            return FOLLOW_PROFILE_PHASE_STANDOFF_ALIGN
        return FOLLOW_PROFILE_PHASE_STANDOFF_HOLD

    def _limits_for(
        self,
        *,
        request: FollowRequest,
        phase: str,
        distance_error_m: float,
        abs_bearing_rad: float,
    ) -> Tuple[Optional[float], Optional[float]]:
        base_v = _finite(getattr(request, "v_max_mps", None), None)
        base_w = _finite(getattr(request, "omega_max_rad_s", None), None)
        confidence = _clamp(float(_finite(getattr(request, "confidence", None), 1.0) or 1.0), 0.0, 1.0)
        confidence_scale = _clamp((confidence - 0.20) / 0.80, 0.25, 1.0)

        if base_v is None:
            max_v: Optional[float] = None
        elif phase in {
            FOLLOW_PROFILE_PHASE_ACQUIRE_ALIGN,
            FOLLOW_PROFILE_PHASE_STANDOFF_ALIGN,
            FOLLOW_PROFILE_PHASE_STANDOFF_HOLD,
            FOLLOW_PROFILE_PHASE_DIRECTION_ONLY,
        }:
            max_v = 0.0
        elif phase == FOLLOW_PROFILE_PHASE_CLOSE_RETREAT:
            max_v = min(float(base_v), 0.040) * confidence_scale
        elif phase == FOLLOW_PROFILE_PHASE_APPROACH_SETTLE:
            settle_scale = _clamp(float(distance_error_m) / max(1e-6, APPROACH_FAR_M), 0.22, 0.55)
            bearing_scale = _clamp(math.cos(min(float(abs_bearing_rad), 1.20)), 0.35, 1.0)
            max_v = float(base_v) * float(settle_scale) * float(bearing_scale) * confidence_scale
        else:
            far_scale = _clamp(float(distance_error_m) / 0.85, 0.45, 1.0)
            bearing_scale = _clamp(math.cos(min(float(abs_bearing_rad), 1.20)), 0.40, 1.0)
            max_v = float(base_v) * float(far_scale) * float(bearing_scale) * confidence_scale

        if base_w is None:
            max_w: Optional[float] = None
        elif phase == FOLLOW_PROFILE_PHASE_ACQUIRE_ALIGN:
            max_w = min(float(base_w), 0.28)
        elif phase == FOLLOW_PROFILE_PHASE_APPROACH_FAR:
            max_w = min(float(base_w), 0.24)
        elif phase == FOLLOW_PROFILE_PHASE_APPROACH_SETTLE:
            max_w = min(float(base_w), 0.18)
        elif phase in {FOLLOW_PROFILE_PHASE_STANDOFF_ALIGN, FOLLOW_PROFILE_PHASE_STANDOFF_HOLD}:
            max_w = min(float(base_w), 0.16)
        elif phase == FOLLOW_PROFILE_PHASE_DIRECTION_ONLY:
            max_w = min(float(base_w), 0.22)
        else:
            max_w = min(float(base_w), 0.20)
        return max_v, max_w

    def _passthrough(self, request: FollowRequest, *, reason: str) -> FollowMotionProfileResult:
        return FollowMotionProfileResult(
            goal_x=_finite(getattr(request, "goal_x", None), None),
            goal_y=_finite(getattr(request, "goal_y", None), None),
            goal_theta=_finite(getattr(request, "goal_theta", None), None),
            distance_to_target_m=_finite(getattr(request, "distance_to_target_m", None), None),
            max_v_mps=_finite(getattr(request, "v_max_mps", None), None),
            max_omega_rad_s=_finite(getattr(request, "omega_max_rad_s", None), None),
            phase=str(reason or "passthrough"),
            status={
                "active": False,
                "provider": FOLLOW_PROFILE_PROVIDER,
                "reason": str(reason or "passthrough"),
            },
        )
