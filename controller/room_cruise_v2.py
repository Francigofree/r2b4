#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Room cruise v2 behavior layer.

This layer owns only the behavior intent.  It does not issue track/PWM commands.
Runtime motion remains:
EKF/rolling map -> existing route selector -> waypoint/local path ->
NavigationIntent -> LocalNavigationLayer -> CruiseLayerV2 -> common M3 execution.
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, replace
from typing import Any, Deque, Dict, List, Optional, Tuple

from controller.navigation_intent import NAV_MODE_ROOM_CRUISE, NavigationIntent
from controller.rolling_local_map import enhance_lidar_summary


@dataclass(frozen=True)
class RoomCruiseV2Config:
    default_duration_s: float = 60.0
    max_v_mps: float = 0.30
    max_omega_rad_s: float = 0.60
    priority: int = 805
    route_horizon_m: float = 0.90
    route_max_kappa_m_inv: float = 1.20
    waypoint_tolerance_m: float = 0.14
    no_progress_timeout_s: float = 3.0
    blocked_replan_s: float = 0.80
    goal_max_age_s: float = 12.0
    visit_cell_size_m: float = 0.35
    visit_history_cells: int = 256
    goal_history_size: int = 24


def _finite_float(value: Any, default: float) -> float:
    try:
        out = float(value)
    except Exception:
        return float(default)
    if not math.isfinite(out):
        return float(default)
    return float(out)


def create_config(raw_cfg: Optional[Dict[str, Any]] = None) -> RoomCruiseV2Config:
    raw = dict(raw_cfg or {})
    return RoomCruiseV2Config(
        default_duration_s=max(1.0, min(600.0, _finite_float(raw.get("default_duration_s"), 60.0))),
        max_v_mps=max(0.15, min(0.30, _finite_float(raw.get("max_v_mps"), 0.30))),
        max_omega_rad_s=max(0.20, min(1.20, _finite_float(raw.get("max_omega_rad_s"), 0.60))),
        priority=int(_finite_float(raw.get("priority"), 805)),
        route_horizon_m=max(0.45, min(1.20, _finite_float(raw.get("route_horizon_m"), 0.90))),
        route_max_kappa_m_inv=max(
            0.20,
            min(2.50, _finite_float(raw.get("route_max_kappa_m_inv"), 1.20)),
        ),
        waypoint_tolerance_m=max(
            0.05,
            min(0.30, _finite_float(raw.get("waypoint_tolerance_m"), 0.14)),
        ),
        no_progress_timeout_s=max(
            0.50,
            min(20.0, _finite_float(raw.get("no_progress_timeout_s"), 3.0)),
        ),
        blocked_replan_s=max(0.20, min(5.0, _finite_float(raw.get("blocked_replan_s"), 0.80))),
        goal_max_age_s=max(2.0, min(60.0, _finite_float(raw.get("goal_max_age_s"), 12.0))),
        visit_cell_size_m=max(0.15, min(1.0, _finite_float(raw.get("visit_cell_size_m"), 0.35))),
        visit_history_cells=max(32, min(2048, int(_finite_float(raw.get("visit_history_cells"), 256)))),
        goal_history_size=max(4, min(128, int(_finite_float(raw.get("goal_history_size"), 24)))),
    )


def _wrap_angle(rad: float) -> float:
    return (float(rad) + math.pi) % (2.0 * math.pi) - math.pi


def _extract_pose(ekf_state: Dict[str, Any]) -> Optional[Tuple[float, float, float]]:
    src = dict(ekf_state or {})
    try:
        x = float(src.get("x"))
        y = float(src.get("y"))
        theta_raw = src.get("theta", src.get("theta_rad"))
        theta = (
            math.radians(float(src.get("theta_deg")))
            if theta_raw is None
            else float(theta_raw)
        )
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (x, y, theta)):
        return None
    return float(x), float(y), float(theta)


def _arc_endpoint(length_m: float, kappa_m_inv: float) -> Tuple[float, float, float]:
    length = max(0.0, float(length_m))
    kappa = float(kappa_m_inv)
    heading_delta = float(kappa * length)
    if abs(kappa) <= 1e-6:
        return float(length), 0.0, 0.0
    return (
        float(math.sin(heading_delta) / kappa),
        float((1.0 - math.cos(heading_delta)) / kappa),
        float(heading_delta),
    )


def _robot_to_world(
    rx: float,
    ry: float,
    pose: Tuple[float, float, float],
) -> Tuple[float, float]:
    x, y, theta = pose
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    return (
        float(x + (cos_t * float(rx)) - (sin_t * float(ry))),
        float(y + (sin_t * float(rx)) + (cos_t * float(ry))),
    )


class RoomCruiseV2Layer:
    def __init__(self, cfg: Optional[RoomCruiseV2Config] = None) -> None:
        self.cfg = cfg or RoomCruiseV2Config()
        self.active = False
        self.started_mono_s = 0.0
        self.duration_s = float(self.cfg.default_duration_s)
        self.source = "STATE"
        self.reason = "idle"
        self.tick_count = 0
        self.local_navigation_count = 0
        self.goal_sequence = 0
        self.completed_waypoints = 0
        self.replan_count = 0
        self._goal: Optional[Dict[str, Any]] = None
        self._goal_history: Deque[Tuple[int, int]] = deque(maxlen=int(self.cfg.goal_history_size))
        self._visited_cells: Deque[Tuple[int, int]] = deque()
        self._visit_counts: Dict[Tuple[int, int], int] = {}
        self._last_visit_cell: Optional[Tuple[int, int]] = None
        self._blocked_since_s = 0.0
        self._last_local_feasible = True
        self._last_motion_phase = ""
        self._last_recovery_lifecycle_active = False
        self._last_recovery_lifecycle_state = ""
        self._last_goal_event = "idle"
        self._last_route_selector: Dict[str, Any] = {}
        self.last_status: Dict[str, Any] = {"active": False, "reason": "idle"}

    def _reset_mission_memory(self) -> None:
        self.goal_sequence = 0
        self.completed_waypoints = 0
        self.replan_count = 0
        self._goal = None
        self._goal_history = deque(maxlen=int(self.cfg.goal_history_size))
        self._visited_cells = deque()
        self._visit_counts = {}
        self._last_visit_cell = None
        self._blocked_since_s = 0.0
        self._last_local_feasible = True
        self._last_motion_phase = ""
        self._last_recovery_lifecycle_active = False
        self._last_recovery_lifecycle_state = ""
        self._last_goal_event = "started"
        self._last_route_selector = {}

    def start(
        self,
        *,
        duration_s: Optional[float] = None,
        max_v_mps: Optional[float] = None,
        max_omega_rad_s: Optional[float] = None,
        source: str = "STATE",
        now_s: Optional[float] = None,
    ) -> Dict[str, Any]:
        self.active = True
        self.started_mono_s = time.monotonic() if now_s is None else float(now_s)
        self.duration_s = max(1.0, min(600.0, _finite_float(duration_s, self.cfg.default_duration_s)))
        self.source = str(source or "STATE")
        self.reason = "started"
        self.tick_count = 0
        self.local_navigation_count = 0
        self._reset_mission_memory()
        if max_v_mps is not None or max_omega_rad_s is not None:
            self.cfg = replace(
                self.cfg,
                max_v_mps=max(0.15, min(0.30, _finite_float(max_v_mps, self.cfg.max_v_mps))),
                max_omega_rad_s=max(
                    0.20,
                    min(1.20, _finite_float(max_omega_rad_s, self.cfg.max_omega_rad_s)),
                ),
            )
        self.last_status = self.status(now_s=self.started_mono_s)
        return dict(self.last_status)

    def stop(self, *, reason: str = "stopped", now_s: Optional[float] = None) -> Dict[str, Any]:
        self.active = False
        self.reason = str(reason or "stopped")
        self.last_status = self.status(now_s=now_s)
        return dict(self.last_status)

    def status(self, *, now_s: Optional[float] = None) -> Dict[str, Any]:
        now = time.monotonic() if now_s is None else float(now_s)
        elapsed = max(0.0, float(now) - float(self.started_mono_s)) if self.started_mono_s > 0.0 else 0.0
        return {
            "active": bool(self.active),
            "provider": "room_cruise_v2",
            "reason": str(self.reason or ""),
            "source": str(self.source or "STATE"),
            "elapsed_s": round(float(elapsed), 3),
            "duration_s": round(float(self.duration_s), 3),
            "tick_count": int(self.tick_count),
            "local_navigation_count": int(self.local_navigation_count),
            "max_v_mps": round(float(self.cfg.max_v_mps), 4),
            "max_omega_rad_s": round(float(self.cfg.max_omega_rad_s), 4),
            "priority": int(self.cfg.priority),
            "m5_full_stack": {
                "goal_sequence": int(self.goal_sequence),
                "completed_waypoints": int(self.completed_waypoints),
                "replan_count": int(self.replan_count),
                "goal_event": str(self._last_goal_event),
                "goal": self._compact_goal(self._goal),
                "path_primitive": self._compact_path_primitive(self._goal),
                "visited_cell_count": int(len(self._visit_counts)),
                "visited_path_entries": int(len(self._visited_cells)),
                "local_feasible": bool(self._last_local_feasible),
                "motion_phase": str(self._last_motion_phase),
                "recovery_lifecycle_active": bool(self._last_recovery_lifecycle_active),
                "recovery_lifecycle_state": str(self._last_recovery_lifecycle_state),
                "route_selector": dict(self._last_route_selector or {}),
                "chain": [
                    "EKF_POSE",
                    "ROLLING_LOCAL_MAP",
                    "GLOBAL_MOTION_POLICY_ROUTE_SELECTOR",
                    "WAYPOINT_LOCAL_PATH",
                    "NAVIGATION_INTENT",
                    "LOCAL_NAVIGATION",
                    "LOCAL_PLANNER",
                    "CRUISE_LAYER_V2",
                    "M3_MOTION_EXECUTION",
                ],
                "localization_safety_owner": "LOCALIZATION_GATE",
                "physical_safety_owner": "SAFETY_SUPERVISOR",
            },
        }

    @staticmethod
    def _compact_goal(goal: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        src = dict(goal or {})
        if not src:
            return {}
        return {
            "id": str(src.get("id") or ""),
            "x": src.get("x"),
            "y": src.get("y"),
            "theta_rad": src.get("theta_rad"),
            "planned_at_s": src.get("planned_at_s"),
            "distance_to_goal_m": src.get("distance_to_goal_m"),
            "path_progress_m": src.get("path_progress_m"),
            "selected_kappa_m_inv": src.get("selected_kappa_m_inv"),
            "continuous_handoff": bool(src.get("continuous_handoff", False)),
            "lifecycle": str(src.get("lifecycle") or ""),
        }

    @staticmethod
    def _compact_path_primitive(goal: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        src = dict(goal or {})
        primitive = dict(src.get("local_path_segment") or {})
        if not primitive:
            return {"active": False}
        return {
            "active": True,
            "id": str(primitive.get("id") or src.get("id") or ""),
            "length_m": primitive.get("length_m"),
            "curvature": primitive.get("curvature"),
            "target_heading_delta_rad": primitive.get("target_heading_delta_rad"),
            "path_progress_m": src.get("path_progress_m"),
        }

    def _cell_for_xy(self, x: float, y: float) -> Tuple[int, int]:
        cell_m = max(0.05, float(self.cfg.visit_cell_size_m))
        return int(math.floor(float(x) / cell_m)), int(math.floor(float(y) / cell_m))

    def _record_pose_visit(self, pose: Tuple[float, float, float]) -> None:
        cell = self._cell_for_xy(pose[0], pose[1])
        if cell == self._last_visit_cell:
            return
        self._last_visit_cell = cell
        self._visited_cells.append(cell)
        self._visit_counts[cell] = int(self._visit_counts.get(cell, 0)) + 1
        history_limit = max(16, int(self.cfg.visit_history_cells))
        while len(self._visited_cells) > history_limit:
            old = self._visited_cells.popleft()
            count = int(self._visit_counts.get(old, 0))
            if count <= 1:
                self._visit_counts.pop(old, None)
            else:
                self._visit_counts[old] = count - 1

    def _visit_pressure(self, wx: float, wy: float) -> float:
        cx, cy = self._cell_for_xy(wx, wy)
        pressure = 0.0
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                weight = 1.0 if dx == 0 and dy == 0 else 0.35
                pressure += weight * float(self._visit_counts.get((cx + dx, cy + dy), 0))
        pressure += 1.5 * float(sum(1 for cell in self._goal_history if cell == (cx, cy)))
        return float(pressure)

    def _coverage_base_kappa(
        self,
        pose: Tuple[float, float, float],
    ) -> Tuple[float, List[Dict[str, Any]]]:
        horizon = max(0.45, float(self.cfg.route_horizon_m))
        candidate_kappas = (0.0, 0.65, -0.65)
        rows: List[Dict[str, Any]] = []
        for order, kappa in enumerate(candidate_kappas):
            rx, ry, _heading_delta = _arc_endpoint(horizon, kappa)
            wx, wy = _robot_to_world(rx, ry, pose)
            pressure = self._visit_pressure(wx, wy)
            rows.append(
                {
                    "kappa": float(kappa),
                    "world_x": round(float(wx), 4),
                    "world_y": round(float(wy), 4),
                    "visit_pressure": round(float(pressure), 4),
                    "tie_order": int(order),
                }
            )
        selected = min(rows, key=lambda row: (float(row["visit_pressure"]), int(row["tie_order"])))
        return float(selected["kappa"]), rows

    @staticmethod
    def _selected_candidate(selector: Dict[str, Any]) -> Dict[str, Any]:
        selected_kappa = _finite_float(selector.get("selected_kappa"), 0.0)
        trajectory = dict(selector.get("trajectory_selection") or {})
        candidates = [dict(row or {}) for row in list(trajectory.get("candidates") or [])]
        if not candidates:
            return {}
        return min(
            candidates,
            key=lambda row: abs(_finite_float(row.get("kappa"), selected_kappa) - selected_kappa),
        )

    @staticmethod
    def _compact_selector(selector: Dict[str, Any]) -> Dict[str, Any]:
        src = dict(selector or {})
        trajectory = dict(src.get("trajectory_selection") or {})
        micro_map = dict(src.get("micro_local_map") or {})
        wall = dict(src.get("wall_follow") or {})
        return {
            "provider": str(src.get("provider") or "fallback_clearance_route"),
            "motion_shaping_applied": bool(src.get("motion_shaping_applied", False)),
            "selected_kappa": src.get("selected_kappa"),
            "chosen_direction": str(src.get("chosen_direction") or ""),
            "direction_confidence": src.get("direction_confidence"),
            "blocked_front": bool(src.get("blocked_front", False)),
            "wall_follow": {
                "active": bool(wall.get("active", False)),
                "side": str(wall.get("side") or ""),
                "reason": str(wall.get("reason") or ""),
            },
            "micro_local_map": {
                "has_data": bool(micro_map.get("has_data", False)),
                "space_classification": str(micro_map.get("space_classification") or ""),
                "corridor_detected": bool(micro_map.get("corridor_detected", False)),
                "open_space_detected": bool(micro_map.get("open_space_detected", False)),
                "constriction_detected": bool(micro_map.get("constriction_detected", False)),
            },
            "trajectory_selection": {
                "reason": str(trajectory.get("reason") or ""),
                "candidate_count": int(trajectory.get("candidate_count", 0) or 0),
                "trajectory_memory_size": int(trajectory.get("trajectory_memory_size", 0) or 0),
                "visited_endpoint_cells": int(trajectory.get("trajectory_visited_endpoint_cells", 0) or 0),
                "visited_heading_bins": int(trajectory.get("trajectory_visited_heading_bins", 0) or 0),
            },
        }

    def _plan_goal(
        self,
        *,
        pose: Tuple[float, float, float],
        local_navigation_layer: Any,
        global_motion_policy: Any,
        lidar_summary: Dict[str, Any],
        raw_scan: List[Dict[str, Any]],
        now_s: float,
        replan_reason: str,
    ) -> Optional[Dict[str, Any]]:
        rolling_map = getattr(local_navigation_layer, "rolling_map", None)
        if rolling_map is not None and hasattr(rolling_map, "snapshot"):
            snapshot = rolling_map.snapshot(
                {"x": pose[0], "y": pose[1], "theta": pose[2]},
                now_s=float(now_s),
                include_raw_scan=True,
            )
        else:
            snapshot = {"enabled": False, "has_data": False, "raw_scan": []}
        enhanced = enhance_lidar_summary(dict(lidar_summary or {}), snapshot)
        remembered_scan = snapshot.get("raw_scan") if isinstance(snapshot, dict) else None
        selector_scan = (
            list(remembered_scan)
            if isinstance(remembered_scan, list) and remembered_scan
            else list(raw_scan or [])
        )
        base_kappa, coverage_candidates = self._coverage_base_kappa(pose)
        selector: Dict[str, Any] = {}
        if global_motion_policy is not None and hasattr(global_motion_policy, "select_navigation_trajectory"):
            selector = dict(
                global_motion_policy.select_navigation_trajectory(
                    raw_scan=selector_scan,
                    lidar_summary=enhanced,
                    base_kappa=float(base_kappa),
                    kappa_hard_max=max(0.10, float(self.cfg.route_max_kappa_m_inv)),
                )
                or {}
            )

        selected_kappa = _finite_float(selector.get("selected_kappa"), base_kappa)
        selected_kappa = max(
            -float(self.cfg.route_max_kappa_m_inv),
            min(float(self.cfg.route_max_kappa_m_inv), float(selected_kappa)),
        )
        selected_candidate = self._selected_candidate(selector)
        horizon = _finite_float(
            (selector.get("trajectory_selection") or {}).get("horizon_m"),
            float(self.cfg.route_horizon_m),
        )
        horizon = max(0.30, min(1.20, float(horizon)))
        safe_progress = _finite_float(selected_candidate.get("safe_progress_m"), horizon)
        if selected_candidate and safe_progress > 0.0:
            segment_length = max(0.24, min(float(horizon), float(safe_progress)))
        else:
            segment_length = float(horizon)

        rx, ry, heading_delta = _arc_endpoint(segment_length, selected_kappa)
        wx, wy = _robot_to_world(rx, ry, pose)
        goal_theta = _wrap_angle(float(pose[2]) + float(heading_delta))
        if not all(math.isfinite(value) for value in (wx, wy, goal_theta, segment_length, selected_kappa)):
            return None

        self.goal_sequence += 1
        goal_id = f"m5_wp_{int(self.goal_sequence)}"
        goal_cell = self._cell_for_xy(wx, wy)
        self._goal_history.append(goal_cell)
        goal = {
            "id": goal_id,
            "x": float(wx),
            "y": float(wy),
            "theta_rad": float(goal_theta),
            "tolerance_m": float(self.cfg.waypoint_tolerance_m),
            "planned_at_s": float(now_s),
            "start_x": float(pose[0]),
            "start_y": float(pose[1]),
            "best_distance_m": float(math.hypot(wx - pose[0], wy - pose[1])),
            "last_progress_s": float(now_s),
            "distance_to_goal_m": float(math.hypot(wx - pose[0], wy - pose[1])),
            "path_progress_m": 0.0,
            "selected_kappa_m_inv": float(selected_kappa),
            "continuous_handoff": True,
            "lifecycle": "ARMED",
            "replan_reason": str(replan_reason),
            "local_path_segment": {
                "id": goal_id,
                "length_m": float(segment_length),
                "curvature": float(selected_kappa),
                "target_heading_delta_rad": float(heading_delta),
                "v_max": float(self.cfg.max_v_mps),
                "omega_max": float(self.cfg.max_omega_rad_s),
            },
            "coverage_candidates": coverage_candidates,
            "rolling_local_map": {
                "has_data": bool(snapshot.get("has_data", False)),
                "observation_count": int(snapshot.get("observation_count", 0) or 0),
                "valid_points": int(snapshot.get("valid_points", 0) or 0),
            },
        }
        self._last_route_selector = self._compact_selector(selector)
        self._blocked_since_s = 0.0
        return goal

    def _goal_replan_reason(
        self,
        *,
        pose: Tuple[float, float, float],
        now_s: float,
    ) -> str:
        goal = self._goal
        if not isinstance(goal, dict):
            return "initial_goal"
        distance = float(math.hypot(float(goal["x"]) - pose[0], float(goal["y"]) - pose[1]))
        path_progress = float(math.hypot(pose[0] - float(goal["start_x"]), pose[1] - float(goal["start_y"])))
        goal["distance_to_goal_m"] = float(distance)
        goal["path_progress_m"] = float(path_progress)
        best_distance = _finite_float(goal.get("best_distance_m"), distance)
        if distance <= max(0.03, float(self.cfg.waypoint_tolerance_m)):
            goal["lifecycle"] = "COMPLETED"
            self.completed_waypoints += 1
            return "waypoint_reached"
        if distance <= best_distance - 0.025:
            goal["best_distance_m"] = float(distance)
            goal["last_progress_s"] = float(now_s)
        goal["lifecycle"] = "RUNNING"

        recovery_phase = str(self._last_motion_phase or "") in {
            "obstacle_tangent_arc",
            "obstacle_heading_pivot",
            "room_cruise_reverse_arc",
            "room_cruise_reverse_straight",
            "room_cruise_stuck_pivot",
            "localization_confidence_pivot",
        }
        recovery_lifecycle = bool(
            recovery_phase or self._last_recovery_lifecycle_active
        )
        forward_projection = (
            math.cos(float(pose[2])) * (float(goal["x"]) - float(pose[0]))
            + math.sin(float(pose[2])) * (float(goal["y"]) - float(pose[1]))
        )
        if not recovery_lifecycle and forward_projection < -0.03:
            return "waypoint_behind_after_recovery"
        if recovery_lifecycle:
            # Recovery motion is purposeful but may initially move away from
            # the old waypoint.  Give the normal route a fresh no-progress
            # window after the physical recovery primitive has completed.
            goal["last_progress_s"] = float(now_s)
            self._blocked_since_s = 0.0
        if (
            self._blocked_since_s > 0.0
            and not recovery_lifecycle
            and float(now_s) - float(self._blocked_since_s) >= float(self.cfg.blocked_replan_s)
        ):
            return "local_path_blocked"
        if (
            not recovery_lifecycle
            and float(now_s) - _finite_float(goal.get("last_progress_s"), now_s)
            >= float(self.cfg.no_progress_timeout_s)
        ):
            return "waypoint_no_progress"
        if (
            not recovery_lifecycle
            and float(now_s) - _finite_float(goal.get("planned_at_s"), now_s)
            >= float(self.cfg.goal_max_age_s)
        ):
            return "waypoint_age_replan"
        return ""

    def tick(
        self,
        *,
        local_navigation_layer: Any,
        lidar_summary: Dict[str, Any],
        ekf_state: Dict[str, Any],
        raw_scan: Optional[List[Dict[str, Any]]] = None,
        source: Optional[str] = None,
        dt: float = 0.02,
        now_s: Optional[float] = None,
        runtime_v_max_mps: Optional[float] = None,
        cruise_layer_v2: Any = None,
        global_motion_policy: Any = None,
        tick_context: Any = None,
        clearance_cache: Optional[Dict[Any, Any]] = None,
    ) -> Dict[str, Any]:
        now = time.monotonic() if now_s is None else float(now_s)
        if not bool(self.active):
            self.last_status = self.status(now_s=now)
            return {"proposal": None, "status": dict(self.last_status)}

        elapsed = max(0.0, float(now) - float(self.started_mono_s))
        if elapsed >= float(self.duration_s):
            status = self.stop(reason="duration_complete", now_s=now)
            return {"proposal": None, "status": status}

        if local_navigation_layer is None or not hasattr(local_navigation_layer, "tick_intent"):
            self.reason = "local_navigation_missing"
            self.last_status = self.status(now_s=now)
            return {"proposal": None, "status": dict(self.last_status)}

        src = str(source or self.source or "STATE")
        self.tick_count += 1
        pose = _extract_pose(dict(ekf_state or {}))
        if pose is None:
            self.reason = "ekf_pose_invalid_hold"
            self._last_goal_event = "ekf_pose_invalid"
            self.last_status = self.status(now_s=now)
            self.last_status["proposal_active"] = False
            return {"proposal": None, "status": dict(self.last_status)}
        self._record_pose_visit(pose)
        replan_reason = self._goal_replan_reason(pose=pose, now_s=now)
        if replan_reason:
            if replan_reason != "initial_goal" and replan_reason != "waypoint_reached":
                self.replan_count += 1
            self._goal = self._plan_goal(
                pose=pose,
                local_navigation_layer=local_navigation_layer,
                global_motion_policy=global_motion_policy,
                lidar_summary=dict(lidar_summary or {}),
                raw_scan=list(raw_scan or []),
                now_s=now,
                replan_reason=replan_reason,
            )
            self._last_goal_event = str(replan_reason)
        goal = dict(self._goal or {})
        if not goal:
            self.reason = "route_selection_failed_hold"
            self.last_status = self.status(now_s=now)
            self.last_status["proposal_active"] = False
            return {"proposal": None, "status": dict(self.last_status)}

        requested_v_max = float(self.cfg.max_v_mps)
        runtime_v_max = _finite_float(runtime_v_max_mps, requested_v_max)
        effective_v_max = min(
            requested_v_max,
            runtime_v_max if runtime_v_max > 0.0 else requested_v_max,
        )
        intent = NavigationIntent(
            active=True,
            source=src,
            behavior="ROOM_CRUISE_V2",
            mode=NAV_MODE_ROOM_CRUISE,
            command_type="room_cruise_v2",
            goal_x=float(goal["x"]),
            goal_y=float(goal["y"]),
            goal_theta=float(goal["theta_rad"]),
            desired_speed_mps=float(effective_v_max),
            max_v_mps=float(effective_v_max),
            max_omega_rad_s=float(self.cfg.max_omega_rad_s),
            priority=int(self.cfg.priority),
            reason="m5_full_stack_waypoint",
            metadata={
                "duration_s": float(self.duration_s),
                "elapsed_s": float(elapsed),
                "layering": "m5_full_stack_room_cruise",
                "requested_v_max_mps": float(requested_v_max),
                "runtime_v_max_mps": float(runtime_v_max),
                "effective_v_max_mps": float(effective_v_max),
                "waypoint": {
                    "id": str(goal["id"]),
                    "x": float(goal["x"]),
                    "y": float(goal["y"]),
                    "theta_rad": float(goal["theta_rad"]),
                    "tolerance_m": float(goal["tolerance_m"]),
                    "continuous_handoff": True,
                    "lifecycle": str(goal.get("lifecycle") or "RUNNING"),
                },
                "local_path_segment": dict(goal.get("local_path_segment") or {}),
                "path_progress_m": float(goal.get("path_progress_m", 0.0) or 0.0),
                "coverage_memory": {
                    "visited_cell_count": int(len(self._visit_counts)),
                    "visited_path_entries": int(len(self._visited_cells)),
                    "goal_history_size": int(len(self._goal_history)),
                },
                "route_selector": dict(self._last_route_selector or {}),
            },
        )
        if cruise_layer_v2 is None or not hasattr(cruise_layer_v2, "tick_intent"):
            from controller.cruise_layer_v2 import CruiseLayerV2

            cruise_layer_v2 = CruiseLayerV2()
        from controller.cruise_layer_v2 import CRUISE_LAYER_V2_ROUTE_ROOM_CRUISE

        result = cruise_layer_v2.tick_intent(
            intent,
            local_navigation_layer=local_navigation_layer,
            lidar_summary=lidar_summary,
            ekf_state=ekf_state,
            raw_scan=raw_scan,
            source=src,
            dt=float(dt),
            now_s=now,
            route=CRUISE_LAYER_V2_ROUTE_ROOM_CRUISE,
            proposal_name="room_cruise_v2_local_navigation",
            update_map=False,
            tick_context=tick_context,
            clearance_cache=clearance_cache,
        )
        proposal = getattr(result, "proposal", None)
        cruise_status = dict(getattr(result, "status", {}) or {})
        diagnostics = dict(cruise_status.get("local_navigation") or {})
        obstacle_diagnostics = dict(diagnostics.get("obstacle_avoidance") or {})
        stuck_evidence = dict(obstacle_diagnostics.get("stuck_evidence") or {})
        attempt_evidence = dict(stuck_evidence.get("attempt") or {})
        recovery_reason = str(obstacle_diagnostics.get("reason") or "")
        recovery_state = str(stuck_evidence.get("state") or "")
        recovery_attempt_mode = str(attempt_evidence.get("attempt_mode") or "")
        self._last_recovery_lifecycle_active = bool(
            str(stuck_evidence.get("policy") or "").startswith("room_cruise_arc_first_stuck")
            and (
                bool(recovery_attempt_mode)
                or bool(stuck_evidence.get("arc_failed", False))
                or bool(stuck_evidence.get("reverse_active", False))
                or bool(stuck_evidence.get("reverse_failed", False))
                or bool(stuck_evidence.get("pivot_active", False))
                or _finite_float(stuck_evidence.get("cooldown_remaining_s"), 0.0) > 0.0
                or recovery_reason.startswith("room_cruise_")
            )
        )
        self._last_recovery_lifecycle_state = str(
            recovery_state or recovery_attempt_mode or recovery_reason
        )
        if proposal is not None:
            self.local_navigation_count += 1
            proposal = dict(proposal)
            proposal["name"] = "room_cruise_v2_local_navigation"
            proposal["source"] = src
            proposal["priority"] = int(self.cfg.priority)
            details = dict(proposal.get("details") or {})
            speed_profile = dict(details.get("speed_profile") or {})
            self._last_motion_phase = str(speed_profile.get("phase") or "")
            details["room_cruise_v2"] = {
                "active": True,
                "source": src,
                "elapsed_s": round(float(elapsed), 3),
                "duration_s": round(float(self.duration_s), 3),
                "route": CRUISE_LAYER_V2_ROUTE_ROOM_CRUISE,
                "runtime_v_max_mps": round(float(runtime_v_max), 4),
                "effective_v_max_mps": round(float(effective_v_max), 4),
                "m5_full_stack": True,
                "goal": self._compact_goal(goal),
                "goal_event": str(self._last_goal_event),
                "visited_cell_count": int(len(self._visit_counts)),
                "replan_count": int(self.replan_count),
                "route_selector": dict(self._last_route_selector or {}),
            }
            proposal["details"] = details

        self._last_local_feasible = bool(diagnostics.get("feasible", proposal is not None))
        if self._last_local_feasible:
            self._blocked_since_s = 0.0
        elif self._blocked_since_s <= 0.0:
            self._blocked_since_s = float(now)

        self.reason = str(cruise_status.get("reason") or diagnostics.get("reason") or "local_navigation_ready")
        self.last_status = self.status(now_s=now)
        self.last_status.update(
            {
                "intent": intent.to_dict(),
                "local_navigation": diagnostics,
                "cruise_layer": cruise_status,
                "route": CRUISE_LAYER_V2_ROUTE_ROOM_CRUISE,
                "proposal_active": bool(proposal is not None),
            }
        )
        return {"proposal": proposal, "status": dict(self.last_status)}
