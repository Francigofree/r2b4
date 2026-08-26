#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Layered local wall-navigation stack:
1) perception (selected wall model + confidence + corner/opening candidates)
2) motion primitives FSM
3) short-horizon trajectory follower (curvature -> v/omega)
4) lightweight supervisor envelope

This module is intentionally local-frame first. It does not depend on global
odometry to estimate wall geometry; EKF/global pose belongs to higher layers.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


RIGHT_WALL_SLICE = (70.0, 110.0)
LEFT_WALL_SLICE = (250.0, 290.0)
RIGHT_WALL_WIDE = (60.0, 120.0)
LEFT_WALL_WIDE = (240.0, 300.0)
RIGHT_WALL_HEMI = (20.0, 160.0)
LEFT_WALL_HEMI = (200.0, 340.0)
FRONT_SLICE = (340.0, 20.0)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


def _wrap_angle(rad: float) -> float:
    return (float(rad) + math.pi) % (2.0 * math.pi) - math.pi


def _angle_in_slice(angle_deg: float, lo: float, hi: float) -> bool:
    angle_deg = float(angle_deg) % 360.0
    lo = float(lo) % 360.0
    hi = float(hi) % 360.0
    if lo <= hi:
        return lo <= angle_deg <= hi
    return angle_deg >= lo or angle_deg <= hi


def _extract_slice_points(
    scan: list,
    lo_deg: float,
    hi_deg: float,
    max_range_m: float = 3.0,
) -> List[Tuple[float, float, float, float]]:
    """
    Return points inside angular slice:
      (x_body, y_body, angle_deg, distance_m)
    """
    out: List[Tuple[float, float, float, float]] = []
    for pt in scan:
        try:
            angle_deg = float(pt.get("angle"))
            dist_mm = float(pt.get("dist"))
        except Exception:
            continue
        if dist_mm <= 0.0:
            continue
        dist_m = dist_mm / 1000.0
        if dist_m > float(max_range_m):
            continue
        if not _angle_in_slice(angle_deg, lo_deg, hi_deg):
            continue
        rad = math.radians(angle_deg)
        x = dist_m * math.cos(rad)
        y = dist_m * math.sin(rad)
        out.append((float(x), float(y), float(angle_deg), float(dist_m)))
    return out


@dataclass(frozen=True)
class SelectedWall:
    distance: Optional[float]
    heading_error: Optional[float]
    tangent_heading: Optional[float]
    confidence: float
    continuity_id: int
    normal_vector: Tuple[float, float]
    tangent_vector: Tuple[float, float]
    points_used: int


@dataclass(frozen=True)
class WallPerceptionFrame:
    selected_wall: SelectedWall
    front_free_path: Optional[float]
    corner_candidate: bool
    opening_candidate: bool
    lidar_fresh: bool
    source: str


@dataclass
class _WallCandidate:
    source: str
    tangent_heading: float
    heading_error: float
    distance: float
    confidence: float
    points_used: int
    centroid_x: float
    centroid_y: float
    normal_vector: Tuple[float, float]
    tangent_vector: Tuple[float, float]


class WallPerceptionTracker:
    """
    Robust local wall model estimator:
    - multiple fit candidates per frame
    - orientation continuity hysteresis
    - confidence score for control gating
    """

    def __init__(self, *, wall_side: str):
        side = str(wall_side).strip().lower()
        if side not in ("left", "right"):
            side = "right"
        self.wall_side = side
        self._last_tangent_heading: Optional[float] = None
        self._last_distance: Optional[float] = None
        self._last_confidence: float = 0.0
        self._continuity_id: int = 1
        self._opening_hits: int = 0
        self._corner_hits: int = 0

    @property
    def continuity_id(self) -> int:
        return int(self._continuity_id)

    def update(
        self,
        *,
        scan: list,
        target_clearance_m: float,
        lidar_fresh: bool,
    ) -> WallPerceptionFrame:
        front_free = self._estimate_front_free_path(scan) if lidar_fresh else None
        candidates = self._build_candidates(scan) if lidar_fresh else []
        selected = self._select_candidate(candidates)
        opening = self._estimate_opening_candidate(
            selected=selected,
            front_free_path=front_free,
            target_clearance_m=target_clearance_m,
        )
        corner = self._estimate_corner_candidate(
            selected=selected,
            target_clearance_m=target_clearance_m,
        )
        if selected.distance is not None:
            self._last_distance = float(selected.distance)
        self._last_confidence = float(selected.confidence)
        return WallPerceptionFrame(
            selected_wall=selected,
            front_free_path=front_free,
            corner_candidate=bool(corner),
            opening_candidate=bool(opening),
            lidar_fresh=bool(lidar_fresh),
            source=str(selected.points_used if selected.points_used > 0 else "none"),
        )

    def _build_candidates(self, scan: list) -> List[_WallCandidate]:
        if self.wall_side == "right":
            slices = (
                ("narrow", RIGHT_WALL_SLICE),
                ("wide", RIGHT_WALL_WIDE),
                ("hemi", RIGHT_WALL_HEMI),
            )
            side_sign = 1.0
        else:
            slices = (
                ("narrow", LEFT_WALL_SLICE),
                ("wide", LEFT_WALL_WIDE),
                ("hemi", LEFT_WALL_HEMI),
            )
            side_sign = -1.0

        out: List[_WallCandidate] = []
        for name, (lo, hi) in slices:
            pts = _extract_slice_points(scan, lo, hi, max_range_m=2.8)
            cand = self._fit_line_candidate(pts, source=name, side_sign=side_sign)
            if cand is not None:
                out.append(cand)
        return out

    def _fit_line_candidate(
        self,
        pts: List[Tuple[float, float, float, float]],
        *,
        source: str,
        side_sign: float,
    ) -> Optional[_WallCandidate]:
        n = len(pts)
        if n < 4:
            return None

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        mean_x = sum(xs) / n
        mean_y = sum(ys) / n
        # Hard side-consistency gate: reject candidates that are not on the selected wall side.
        # Prevents front-plane lines from being selected as wall track during turns.
        if (side_sign * mean_y) < 0.03:
            return None
        s_xx = sum((x - mean_x) ** 2 for x in xs)
        s_yy = sum((y - mean_y) ** 2 for y in ys)
        s_xy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
        energy = s_xx + s_yy
        if energy < 1e-9:
            return None

        # Principal direction of the wall in robot frame.
        tangent = 0.5 * math.atan2(2.0 * s_xy, s_xx - s_yy)
        if self._last_tangent_heading is not None:
            alt = _wrap_angle(tangent + math.pi)
            d0 = abs(_wrap_angle(tangent - self._last_tangent_heading))
            d1 = abs(_wrap_angle(alt - self._last_tangent_heading))
            tangent = alt if d1 < d0 else tangent

        # Unit normal + signed distance.
        nx = -math.sin(tangent)
        ny = math.cos(tangent)
        signed_dist = nx * mean_x + ny * mean_y
        distance = abs(signed_dist)

        # Flip normal so it points toward the chosen wall side.
        # This keeps normal orientation stable for diagnostics.
        if side_sign > 0.0 and mean_y < 0.0:
            nx, ny = -nx, -ny
        elif side_sign < 0.0 and mean_y > 0.0:
            nx, ny = -nx, -ny

        # Residual from point-to-line distance.
        residuals = [abs(nx * x + ny * y - signed_dist) for x, y in zip(xs, ys)]
        rms = math.sqrt(sum(r * r for r in residuals) / max(1, n))

        # Heading error sign convention: positive means robot points toward wall.
        heading_error = side_sign * tangent
        heading_error = _clamp(heading_error, -math.pi / 2.0, math.pi / 2.0)

        points_score = _clamp((n - 4.0) / 18.0, 0.0, 1.0)
        residual_score = _clamp(1.0 - (rms / 0.09), 0.0, 1.0)
        continuity_score = 0.8
        if self._last_tangent_heading is not None:
            continuity_score = _clamp(
                1.0 - (abs(_wrap_angle(tangent - self._last_tangent_heading)) / math.radians(45.0)),
                0.0,
                1.0,
            )
        distance_score = _clamp(1.0 - max(0.0, distance - 1.2) / 1.2, 0.0, 1.0)
        confidence = (
            0.34 * points_score
            + 0.34 * residual_score
            + 0.22 * continuity_score
            + 0.10 * distance_score
        )
        confidence = float(_clamp(confidence, 0.0, 1.0))
        if source == "narrow":
            confidence = float(_clamp(confidence + 0.06, 0.0, 1.0))
        elif source == "hemi":
            confidence = float(_clamp(confidence - 0.05, 0.0, 1.0))

        return _WallCandidate(
            source=str(source),
            tangent_heading=float(tangent),
            heading_error=float(heading_error),
            distance=float(distance),
            confidence=float(confidence),
            points_used=int(n),
            centroid_x=float(mean_x),
            centroid_y=float(mean_y),
            normal_vector=(float(nx), float(ny)),
            tangent_vector=(float(math.cos(tangent)), float(math.sin(tangent))),
        )

    def _select_candidate(self, candidates: List[_WallCandidate]) -> SelectedWall:
        if not candidates:
            self._continuity_id += 1
            return SelectedWall(
                distance=None,
                heading_error=None,
                tangent_heading=self._last_tangent_heading,
                confidence=0.0,
                continuity_id=int(self._continuity_id),
                normal_vector=(0.0, 0.0),
                tangent_vector=(1.0, 0.0),
                points_used=0,
            )

        best = None
        best_score = -1.0
        for cand in candidates:
            continuity_bonus = 0.0
            if self._last_tangent_heading is not None:
                continuity_bonus = _clamp(
                    1.0 - abs(_wrap_angle(cand.tangent_heading - self._last_tangent_heading)) / math.radians(50.0),
                    0.0,
                    1.0,
                )
            proximity_bonus = 0.0
            if self._last_distance is not None:
                proximity_bonus = _clamp(
                    1.0 - abs(cand.distance - self._last_distance) / 0.55,
                    0.0,
                    1.0,
                )
            score = 0.65 * cand.confidence + 0.25 * continuity_bonus + 0.10 * proximity_bonus
            if score > best_score:
                best = cand
                best_score = score

        if best is None:
            self._continuity_id += 1
            return SelectedWall(
                distance=None,
                heading_error=None,
                tangent_heading=self._last_tangent_heading,
                confidence=0.0,
                continuity_id=int(self._continuity_id),
                normal_vector=(0.0, 0.0),
                tangent_vector=(1.0, 0.0),
                points_used=0,
            )

        if self._last_tangent_heading is not None:
            delta = abs(_wrap_angle(best.tangent_heading - self._last_tangent_heading))
            if delta > math.radians(60.0) and best.confidence < 0.60:
                self._continuity_id += 1
        self._last_tangent_heading = float(best.tangent_heading)

        return SelectedWall(
            distance=float(best.distance),
            heading_error=float(best.heading_error),
            tangent_heading=float(best.tangent_heading),
            confidence=float(best.confidence),
            continuity_id=int(self._continuity_id),
            normal_vector=best.normal_vector,
            tangent_vector=best.tangent_vector,
            points_used=int(best.points_used),
        )

    def _estimate_front_free_path(self, scan: list) -> Optional[float]:
        front_pts = _extract_slice_points(scan, FRONT_SLICE[0], FRONT_SLICE[1], max_range_m=3.5)
        if not front_pts:
            return None
        return float(min(p[3] for p in front_pts))

    def _estimate_corner_candidate(
        self,
        *,
        selected: SelectedWall,
        target_clearance_m: float,
    ) -> bool:
        if selected.distance is None or selected.confidence < 0.30:
            self._corner_hits = max(0, self._corner_hits - 1)
            return False
        threshold_jump = max(0.18, 0.70 * float(target_clearance_m))
        jump = 0.0
        if self._last_distance is not None:
            jump = float(selected.distance) - float(self._last_distance)
        high_angle = abs(float(selected.heading_error or 0.0)) > math.radians(28.0)
        candidate = jump > threshold_jump and high_angle
        self._corner_hits = min(6, self._corner_hits + 1) if candidate else max(0, self._corner_hits - 1)
        return self._corner_hits >= 2

    def _estimate_opening_candidate(
        self,
        *,
        selected: SelectedWall,
        front_free_path: Optional[float],
        target_clearance_m: float,
    ) -> bool:
        if selected.distance is None or selected.confidence < 0.28:
            self._opening_hits = max(0, self._opening_hits - 1)
            return False
        if front_free_path is None:
            self._opening_hits = max(0, self._opening_hits - 1)
            return False
        far_side = selected.distance > max(float(target_clearance_m) * 2.2, float(target_clearance_m) + 0.30)
        free_front = front_free_path > max(0.55, float(target_clearance_m) * 1.9)
        candidate = bool(far_side and free_front)
        self._opening_hits = min(8, self._opening_hits + 1) if candidate else max(0, self._opening_hits - 1)
        return self._opening_hits >= 2


PRIMITIVE_SEEK_WALL = "SEEK_WALL"
PRIMITIVE_CAPTURE_OFFSET = "CAPTURE_OFFSET"
PRIMITIVE_ALIGN_TANGENT = "ALIGN_TANGENT"
PRIMITIVE_TRACK_WALL = "TRACK_WALL"
PRIMITIVE_INSIDE_CORNER_TRANSITION = "INSIDE_CORNER_TRANSITION"
PRIMITIVE_WALL_LOST_RECOVERY = "WALL_LOST_RECOVERY"
PRIMITIVE_SAFE_STOP = "SAFE_STOP"


@dataclass(frozen=True)
class WallPrimitiveDecision:
    primitive: str
    target_offset_m: float
    target_tangent_heading: float
    speed_scale: float
    curvature_bias: float
    confidence_gate: float
    reason: str


class WallFollowPrimitiveFSM:
    """Primitive-level supervisor for wall-follow maneuvers."""

    def __init__(self, *, wall_side: str):
        side = str(wall_side).strip().lower()
        if side not in ("left", "right"):
            side = "right"
        self.wall_side = side
        self.state = PRIMITIVE_SEEK_WALL
        self._state_since = 0.0
        self._low_conf_since: Optional[float] = None
        self._lost_since: Optional[float] = None

    def _toward_wall_turn_sign(self) -> float:
        return 1.0 if self.wall_side == "right" else -1.0

    def _inside_corner_turn_sign(self) -> float:
        return 1.0 if self.wall_side == "left" else -1.0

    def _set_state(self, new_state: str, now_mono: float) -> None:
        if str(new_state) != str(self.state):
            self.state = str(new_state)
            self._state_since = float(now_mono)

    def tick(
        self,
        *,
        perception: WallPerceptionFrame,
        now_mono: float,
        target_clearance_m: float,
        front_stop_m: float,
    ) -> WallPrimitiveDecision:
        wall = perception.selected_wall
        conf = float(wall.confidence or 0.0)
        wall_visible = bool(wall.distance is not None and conf >= 0.28)
        front = perception.front_free_path
        if not perception.lidar_fresh:
            self._set_state(PRIMITIVE_SAFE_STOP, now_mono)
            return WallPrimitiveDecision(
                primitive=self.state,
                target_offset_m=float(target_clearance_m),
                target_tangent_heading=float(wall.tangent_heading or 0.0),
                speed_scale=0.0,
                curvature_bias=0.0,
                confidence_gate=0.0,
                reason="lidar_stale",
            )
        if front is not None and float(front) < float(front_stop_m):
            self._set_state(PRIMITIVE_SAFE_STOP, now_mono)
            return WallPrimitiveDecision(
                primitive=self.state,
                target_offset_m=float(target_clearance_m),
                target_tangent_heading=float(wall.tangent_heading or 0.0),
                speed_scale=0.0,
                curvature_bias=0.0,
                confidence_gate=0.0,
                reason="front_blocked",
            )

        if conf < 0.12:
            if self._low_conf_since is None:
                self._low_conf_since = float(now_mono)
            elif (now_mono - self._low_conf_since) > 1.2:
                self._set_state(PRIMITIVE_SAFE_STOP, now_mono)
        else:
            self._low_conf_since = None

        if not wall_visible:
            if self._lost_since is None:
                self._lost_since = float(now_mono)
            if (now_mono - self._lost_since) > 0.45 and self.state != PRIMITIVE_SAFE_STOP:
                self._set_state(PRIMITIVE_WALL_LOST_RECOVERY, now_mono)
        else:
            self._lost_since = None

        distance = float(wall.distance) if wall.distance is not None else None
        heading_err = float(wall.heading_error or 0.0)
        offset_err = 0.0 if distance is None else (float(target_clearance_m) - distance)
        abs_offset = abs(offset_err)
        abs_heading = abs(heading_err)

        if self.state == PRIMITIVE_SAFE_STOP:
            if wall_visible and (front is None or front > (front_stop_m + 0.10)):
                self._set_state(PRIMITIVE_SEEK_WALL, now_mono)
            else:
                return WallPrimitiveDecision(
                    primitive=self.state,
                    target_offset_m=float(target_clearance_m),
                    target_tangent_heading=float(wall.tangent_heading or 0.0),
                    speed_scale=0.0,
                    curvature_bias=0.0,
                    confidence_gate=0.0,
                    reason="safe_stop_hold",
                )

        if self.state == PRIMITIVE_SEEK_WALL:
            if wall_visible:
                self._set_state(PRIMITIVE_CAPTURE_OFFSET, now_mono)

        elif self.state == PRIMITIVE_CAPTURE_OFFSET:
            if not wall_visible:
                self._set_state(PRIMITIVE_WALL_LOST_RECOVERY, now_mono)
            elif abs_offset <= 0.07:
                self._set_state(PRIMITIVE_ALIGN_TANGENT, now_mono)

        elif self.state == PRIMITIVE_ALIGN_TANGENT:
            if not wall_visible:
                self._set_state(PRIMITIVE_WALL_LOST_RECOVERY, now_mono)
            elif abs_heading <= math.radians(8.0) and abs_offset <= 0.10:
                self._set_state(PRIMITIVE_TRACK_WALL, now_mono)
            elif abs_offset > 0.14:
                self._set_state(PRIMITIVE_CAPTURE_OFFSET, now_mono)

        elif self.state == PRIMITIVE_TRACK_WALL:
            if not wall_visible:
                self._set_state(PRIMITIVE_WALL_LOST_RECOVERY, now_mono)
            elif perception.corner_candidate:
                self._set_state(PRIMITIVE_INSIDE_CORNER_TRANSITION, now_mono)
            elif abs_offset > 0.16:
                self._set_state(PRIMITIVE_CAPTURE_OFFSET, now_mono)

        elif self.state == PRIMITIVE_INSIDE_CORNER_TRANSITION:
            elapsed = now_mono - self._state_since
            if wall_visible and abs_heading <= math.radians(14.0) and not perception.corner_candidate:
                self._set_state(PRIMITIVE_ALIGN_TANGENT, now_mono)
            elif elapsed > 2.2:
                self._set_state(PRIMITIVE_WALL_LOST_RECOVERY, now_mono)

        elif self.state == PRIMITIVE_WALL_LOST_RECOVERY:
            elapsed = now_mono - self._state_since
            if wall_visible:
                self._set_state(PRIMITIVE_CAPTURE_OFFSET, now_mono)
            elif elapsed > 3.2:
                self._set_state(PRIMITIVE_SAFE_STOP, now_mono)

        speed_scale = 0.45
        curvature_bias = 0.0
        reason = self.state.lower()
        if self.state == PRIMITIVE_SEEK_WALL:
            speed_scale = 0.38
            curvature_bias = 0.55 * self._toward_wall_turn_sign()
        elif self.state == PRIMITIVE_CAPTURE_OFFSET:
            speed_scale = 0.52
            curvature_bias = 0.0
        elif self.state == PRIMITIVE_ALIGN_TANGENT:
            speed_scale = 0.56
            curvature_bias = 0.0
        elif self.state == PRIMITIVE_TRACK_WALL:
            speed_scale = 1.0
            curvature_bias = 0.0
        elif self.state == PRIMITIVE_INSIDE_CORNER_TRANSITION:
            speed_scale = 0.50
            curvature_bias = 0.62 * self._inside_corner_turn_sign()
            reason = "inside_corner_transition"
        elif self.state == PRIMITIVE_WALL_LOST_RECOVERY:
            speed_scale = 0.34
            curvature_bias = 0.44 * self._toward_wall_turn_sign()
            reason = "wall_lost_recovery"
        elif self.state == PRIMITIVE_SAFE_STOP:
            speed_scale = 0.0
            curvature_bias = 0.0
            reason = "safe_stop"

        return WallPrimitiveDecision(
            primitive=str(self.state),
            target_offset_m=float(target_clearance_m),
            target_tangent_heading=float(wall.tangent_heading or 0.0),
            speed_scale=float(speed_scale),
            curvature_bias=float(curvature_bias),
            confidence_gate=float(conf),
            reason=str(reason),
        )


@dataclass(frozen=True)
class TrajectoryCommand:
    v_cmd: float
    omega_cmd: float
    lateral_error_m: float
    heading_error_rad: float
    curvature_cmd: float
    confidence_scale: float
    gate_toward_wall: bool


class WallTrajectoryFollower:
    """
    Curvature-first local trajectory tracker.
    Primitive layer outputs intent; this class applies limits and safety gating.
    """

    def __init__(
        self,
        *,
        wall_side: str,
        max_omega: float,
        max_curvature: float = 2.8,
        curvature_slew_per_s: float = 3.5,
        omega_slew_per_s: float = 1.8,
        k_lat_track: float = 2.4,
        k_head_track: float = 1.0,
    ):
        side = str(wall_side).strip().lower()
        if side not in ("left", "right"):
            side = "right"
        self.wall_side = side
        self.max_omega = max(0.01, float(max_omega))
        self.max_curvature = max(0.20, float(max_curvature))
        self.curvature_slew_per_s = max(0.1, float(curvature_slew_per_s))
        self.omega_slew_per_s = max(0.1, float(omega_slew_per_s))
        self.k_lat_track = float(k_lat_track)
        self.k_head_track = float(k_head_track)
        self._last_curvature = 0.0
        self._last_omega = 0.0
        self._last_v = 0.0
        self._last_front: Optional[float] = None
        self._last_state: str = ""
        self._capture_sign_mul: float = 1.0
        self._capture_eval_dt_s: float = 0.0
        self._capture_ref_abs_err: Optional[float] = None
        self._capture_flip_count: int = 0

    def _toward_wall_turn_sign(self) -> float:
        return 1.0 if self.wall_side == "right" else -1.0

    def _tracking_turn_sign(self) -> float:
        """
        Sign for wall-relative tracking terms in physical omega convention.
        Positive omega = physical RIGHT turn on this robot.
        """
        return -1.0 if self.wall_side == "right" else 1.0

    def compute(
        self,
        *,
        decision: WallPrimitiveDecision,
        perception: WallPerceptionFrame,
        linear_speed_mps: float,
        front_stop_m: float,
        dt_s: float,
        speed_policy_fn,
    ) -> TrajectoryCommand:
        dt = max(0.005, min(0.5, float(dt_s)))
        wall = perception.selected_wall
        distance = wall.distance
        heading_err = float(wall.heading_error or 0.0) if wall.heading_error is not None else 0.0
        lat_err = 0.0
        if distance is not None:
            lat_err = float(decision.target_offset_m) - float(distance)

        state = str(decision.primitive)
        if state != self._last_state:
            if state == PRIMITIVE_CAPTURE_OFFSET:
                self._capture_sign_mul = 1.0
                self._capture_eval_dt_s = 0.0
                self._capture_ref_abs_err = None
                self._capture_flip_count = 0
            else:
                self._capture_eval_dt_s = 0.0
                self._capture_ref_abs_err = None
                self._capture_flip_count = 0
                self._capture_sign_mul = 1.0
            self._last_state = state

        if state == PRIMITIVE_CAPTURE_OFFSET:
            k_lat = self.k_lat_track * 1.25
            k_head = self.k_head_track * 0.55
        elif state == PRIMITIVE_ALIGN_TANGENT:
            k_lat = self.k_lat_track * 0.55
            k_head = self.k_head_track * 1.35
        elif state == PRIMITIVE_TRACK_WALL:
            k_lat = self.k_lat_track
            k_head = self.k_head_track
        elif state == PRIMITIVE_INSIDE_CORNER_TRANSITION:
            k_lat = self.k_lat_track * 0.35
            k_head = self.k_head_track * 0.75
        elif state == PRIMITIVE_WALL_LOST_RECOVERY:
            k_lat = 0.0
            k_head = 0.0
        elif state == PRIMITIVE_SEEK_WALL:
            k_lat = 0.0
            k_head = 0.0
        else:
            k_lat = 0.0
            k_head = 0.0

        track_sign = self._tracking_turn_sign()
        if state == PRIMITIVE_CAPTURE_OFFSET:
            abs_err = abs(float(lat_err))
            self._capture_eval_dt_s += dt
            if self._capture_ref_abs_err is None:
                self._capture_ref_abs_err = abs_err
            if self._capture_eval_dt_s >= 1.2:
                improvement = float(self._capture_ref_abs_err) - abs_err
                not_converging = (
                    (improvement < 0.010 and abs_err > 0.20)
                    or (improvement < -0.015)
                )
                if not_converging and self._capture_flip_count < 3:
                    self._capture_sign_mul *= -1.0
                    self._capture_flip_count += 1
                self._capture_ref_abs_err = abs_err
                self._capture_eval_dt_s = 0.0

        effective_sign = track_sign
        if state == PRIMITIVE_CAPTURE_OFFSET:
            effective_sign = track_sign * self._capture_sign_mul

        curv = float(decision.curvature_bias) + effective_sign * (k_lat * lat_err + k_head * heading_err)
        curv = _clamp(curv, -self.max_curvature, self.max_curvature)

        # Curvature slew limit.
        max_dk = self.curvature_slew_per_s * dt
        curv = _clamp(curv, self._last_curvature - max_dk, self._last_curvature + max_dk)
        self._last_curvature = float(curv)

        front = perception.front_free_path
        confidence_scale = _clamp(float(wall.confidence) / 0.70, 0.22, 1.0)
        if state in (PRIMITIVE_SAFE_STOP,):
            confidence_scale = 0.0
        desired_speed = float(linear_speed_mps) * float(decision.speed_scale) * float(confidence_scale)

        omega_for_speed = abs(self._last_omega)
        min_speed = 0.10
        if state in (PRIMITIVE_CAPTURE_OFFSET, PRIMITIVE_ALIGN_TANGENT, PRIMITIVE_WALL_LOST_RECOVERY):
            min_speed = 0.05

        v_cmd = float(
            speed_policy_fn(
                linear_speed=float(max(0.0, linear_speed_mps)),
                abs_error=abs(lat_err),
                abs_omega=omega_for_speed,
                max_omega=float(self.max_omega),
                front_clearance_m=front,
                front_stop_m=float(front_stop_m),
                state_scale=float(decision.speed_scale) * float(confidence_scale),
                min_speed_mps=float(min_speed),
                prev_v_cmd=(self._last_v if self._last_v > 0.0 else None),
            )
        )
        v_cmd = min(v_cmd, max(0.0, desired_speed + 0.05))
        if state == PRIMITIVE_CAPTURE_OFFSET and abs(float(lat_err)) > 0.25:
            v_cmd = min(float(v_cmd), 0.07)
        if state == PRIMITIVE_SAFE_STOP:
            v_cmd = 0.0

        if state == PRIMITIVE_CAPTURE_OFFSET:
            v_for_curv = max(0.18, abs(v_cmd))
        elif state == PRIMITIVE_ALIGN_TANGENT:
            v_for_curv = max(0.14, abs(v_cmd))
        else:
            v_for_curv = max(0.06, abs(v_cmd))
        omega_cmd = curv * v_for_curv
        omega_cmd = _clamp(omega_cmd, -self.max_omega, self.max_omega)

        gate_toward_wall = False
        if distance is not None and front is not None:
            close_wall = float(distance) <= (float(decision.target_offset_m) + 0.05)
            front_shrinking = False
            if self._last_front is not None:
                front_shrinking = (float(front) - float(self._last_front)) < -0.010
            if close_wall and front_shrinking:
                toward_sign = self._toward_wall_turn_sign()
                if omega_cmd * toward_sign > 0.0:
                    gate_toward_wall = True
                    omega_cmd = toward_sign * min(0.06, abs(float(omega_cmd)))

        max_dw = self.omega_slew_per_s * dt
        omega_cmd = _clamp(omega_cmd, self._last_omega - max_dw, self._last_omega + max_dw)

        self._last_omega = float(omega_cmd)
        self._last_v = float(v_cmd)
        self._last_front = front if front is not None else self._last_front

        return TrajectoryCommand(
            v_cmd=float(v_cmd),
            omega_cmd=float(omega_cmd),
            lateral_error_m=float(lat_err),
            heading_error_rad=float(heading_err),
            curvature_cmd=float(curv),
            confidence_scale=float(confidence_scale),
            gate_toward_wall=bool(gate_toward_wall),
        )


@dataclass(frozen=True)
class SupervisorDecision:
    active_skill: str
    mission_state: str
    requested_primitive: str
    event: str


class WallNavigationSupervisor:
    """Minimal deterministic supervisor envelope around wall-follow primitives."""

    def __init__(self):
        self._opening_hits = 0
        self._last_event = ""

    def tick(
        self,
        *,
        perception: WallPerceptionFrame,
        primitive_decision: WallPrimitiveDecision,
    ) -> SupervisorDecision:
        event = ""
        if perception.opening_candidate:
            self._opening_hits = min(8, self._opening_hits + 1)
        else:
            self._opening_hits = max(0, self._opening_hits - 1)
        if self._opening_hits >= 3:
            event = "opening_candidate_detected"
        self._last_event = event
        return SupervisorDecision(
            active_skill="WALL_FOLLOW",
            mission_state="RUNNING",
            requested_primitive=str(primitive_decision.primitive),
            event=str(event),
        )

    @property
    def last_event(self) -> str:
        return str(self._last_event)
