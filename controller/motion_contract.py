#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Canonical motion contract.

Single, explicit runtime contract for:
- canonical motion modes
- mutual exclusivity
- entry/success conditions
- interrupt reasons
- operator-facing truth fields
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List

from controller.motion_schema import MOTION_SCHEMA_VERSION


CONTRACT_VERSION = "CANONICAL_MOTION_CONTRACT_V2"
EXCLUSIVE_GROUP = "PRIMARY_MOTION_MODE"


@dataclass(frozen=True)
class MotionModeSpec:
    mode: str
    entry_condition: str
    success_condition: str
    interrupt_reasons: tuple[str, ...]
    operator_truth_fields: tuple[str, ...]


def _spec(
    mode: str,
    *,
    entry: str,
    success: str,
    interrupts: Iterable[str],
    truth: Iterable[str],
) -> MotionModeSpec:
    return MotionModeSpec(
        mode=str(mode),
        entry_condition=str(entry),
        success_condition=str(success),
        interrupt_reasons=tuple(str(x) for x in interrupts),
        operator_truth_fields=tuple(str(x) for x in truth),
    )


MODE_SPECS: Dict[str, MotionModeSpec] = {
    "IDLE": _spec(
        "IDLE",
        entry="active_motion_command.layer == IDLE",
        success="|limited_motion_intent.v| <= 0.01 and |limited_motion_intent.omega| <= 0.02",
        interrupts=("new_motion_command", "soft_stop", "emergency_stop"),
        truth=(
            "motion_command.active_layer",
            "motion_command.command_type",
            "motion_execution_state",
            "cmd_linear_mps",
            "cmd_angular_dps",
            "actual_linear_mps",
            "actual_angular_dps",
        ),
    ),
    "TELEOP_VECTOR": _spec(
        "TELEOP_VECTOR",
        entry="command_type in {set_vector,set_speed,step_speed,turn}",
        success="operator command acknowledged and non-zero intent applied",
        interrupts=("soft_stop", "emergency_stop", "higher_priority_source_preempt", "recovery_mode"),
        truth=(
            "motion_command.source",
            "motion_command.command_type",
            "motion_command.requested_motion_intent.v",
            "motion_command.requested_motion_intent.omega",
            "cmd_linear_mps",
            "cmd_angular_dps",
            "actual_linear_mps",
            "actual_angular_dps",
        ),
    ),
    "TWIST": _spec(
        "TWIST",
        entry="command_type in {set_twist,set_motion_target,drive_straight}",
        success="motion_task.execution_state == succeeded",
        interrupts=("soft_stop", "emergency_stop", "env_blocked", "no_progress", "command_preempted"),
        truth=(
            "motion_command.command_type",
            "motion_command.requested_motion_intent.v",
            "motion_command.requested_motion_intent.omega",
            "cmd_linear_mps",
            "cmd_angular_dps",
            "actual_linear_mps",
            "actual_angular_dps",
            "motion_execution_state",
            "segment_stop_reason",
        ),
    ),
    "TRACK_REFERENCE": _spec(
        "TRACK_REFERENCE",
        entry="command_type == set_track_velocity",
        success="motion_task.execution_state == succeeded",
        interrupts=("soft_stop", "emergency_stop", "env_blocked", "no_progress", "command_preempted"),
        truth=(
            "motion_command.requested_track_reference.left_mps",
            "motion_command.requested_track_reference.right_mps",
            "motion_command.track_targets.left_mps",
            "motion_command.track_targets.right_mps",
            "motion_command.turn_semantics.requested.turn_primitive",
            "motion_command.turn_semantics.executed.turn_primitive",
            "motion_execution_mode",
            "actual_linear_mps",
            "actual_angular_dps",
            "motion_execution_state",
            "segment_stop_reason",
        ),
    ),
    "POSE_TARGET": _spec(
        "POSE_TARGET",
        entry="command_type in {set_target_pose,go_to_pose,pose_closed_loop}",
        success="motion_task.execution_state == succeeded and target_pose_public reached",
        interrupts=("soft_stop", "emergency_stop", "env_blocked", "no_progress", "command_preempted"),
        truth=(
            "target_pose_public",
            "target_distance_m",
            "progress_distance_m",
            "target_heading_deg",
            "progress_heading_deg",
            "motion_execution_state",
            "segment_stop_reason",
        ),
    ),
    "FOLLOW_TARGET": _spec(
        "FOLLOW_TARGET",
        entry="command_type == set_follow_target",
        success="follow_layer emits FollowRequest and cruise layer emits room_cruise track primitive",
        interrupts=("soft_stop", "emergency_stop", "target_lost", "env_blocked", "command_preempted"),
        truth=(
            "motion_command.follow_layer",
            "motion_command.cruise_layer",
            "motion_resolution.resolved.command_type",
            "cmd_linear_mps",
            "cmd_angular_dps",
            "actual_linear_mps",
            "actual_angular_dps",
            "motion_execution_state",
            "segment_stop_reason",
        ),
    ),
    "HEADING_TARGET": _spec(
        "HEADING_TARGET",
        entry="command_type in {set_target_heading,rotate_to_heading}",
        success="motion_task.execution_state == succeeded and heading tolerance met",
        interrupts=("soft_stop", "emergency_stop", "command_preempted", "safety_stop"),
        truth=(
            "target_heading_deg",
            "progress_heading_deg",
            "cmd_angular_dps",
            "actual_angular_dps",
            "motion_execution_state",
            "segment_stop_reason",
        ),
    ),
    "WAYPOINT_MISSION": _spec(
        "WAYPOINT_MISSION",
        entry="command_type in {follow_waypoints,follow_arc}",
        success="waypoint_mission.execution_state == succeeded",
        interrupts=("soft_stop", "emergency_stop", "env_blocked", "no_progress", "command_preempted"),
        truth=(
            "waypoint_mission.execution_state",
            "waypoint_mission.active_waypoint_index",
            "segment_target_distance_m",
            "segment_progress_m",
            "actual_linear_mps",
            "actual_angular_dps",
            "segment_stop_reason",
        ),
    ),
    "TRAJECTORY": _spec(
        "TRAJECTORY",
        entry="command_type in {trajectory,local_planner_segment}",
        success="motion_task.execution_state == succeeded",
        interrupts=("soft_stop", "emergency_stop", "env_blocked", "no_progress", "command_preempted"),
        truth=(
            "motion_command.command_type",
            "cmd_linear_mps",
            "cmd_angular_dps",
            "actual_linear_mps",
            "actual_angular_dps",
            "motion_execution_state",
            "segment_stop_reason",
        ),
    ),
    "ADAPTIVE_BEHAVIOR": _spec(
        "ADAPTIVE_BEHAVIOR",
        entry="command_type in {adaptive_direct,search_person,search_person_rotate}",
        success="behavior objective satisfied or explicitly stopped",
        interrupts=("soft_stop", "emergency_stop", "target_lost", "command_preempted"),
        truth=(
            "motion_command.source",
            "motion_command.behavior",
            "cmd_linear_mps",
            "cmd_angular_dps",
            "actual_linear_mps",
            "actual_angular_dps",
            "segment_stop_reason",
        ),
    ),
    "SOFT_STOP": _spec(
        "SOFT_STOP",
        entry="stop_status.type == SOFT_STOP or command_type in {soft_stop,cancel_motion}",
        success="|limited_motion_intent.v| <= 0.01 and |limited_motion_intent.omega| <= 0.02",
        interrupts=("emergency_stop", "new_motion_command"),
        truth=(
            "motion_command.stop.type",
            "motion_command.stop.canonical_reason",
            "segment_stop_reason",
            "cmd_linear_mps",
            "cmd_angular_dps",
            "actual_linear_mps",
            "actual_angular_dps",
        ),
    ),
    "EMERGENCY_STOP": _spec(
        "EMERGENCY_STOP",
        entry="stop_status.type == EMERGENCY_STOP or active command emergency_stop",
        success="motors forced zero and failsafe state active",
        interrupts=("full_reset", "strong_reset"),
        truth=(
            "motion_command.stop.type",
            "motion_command.stop.reason",
            "motion_command.stop.canonical_reason",
            "pwm.left",
            "pwm.right",
            "segment_stop_reason",
        ),
    ),
    "UNCLASSIFIED": _spec(
        "UNCLASSIFIED",
        entry="command_type not mapped to canonical mode",
        success="n/a",
        interrupts=("soft_stop", "emergency_stop", "command_preempted"),
        truth=(
            "motion_command.command_type",
            "motion_command.source",
            "cmd_linear_mps",
            "cmd_angular_dps",
            "actual_linear_mps",
            "actual_angular_dps",
        ),
    ),
}


_TELEOP_TYPES = {"set_vector", "set_speed", "step_speed", "turn"}
_TWIST_TYPES = {"set_twist", "set_motion_target", "drive_straight"}
_TRACK_TYPES = {"set_track_velocity"}
_POSE_TYPES = {"set_target_pose", "go_to_pose", "pose_closed_loop"}
_FOLLOW_TYPES = {"set_follow_target"}
_HEADING_TYPES = {"set_target_heading", "rotate_to_heading"}
_WAYPOINT_TYPES = {"follow_waypoints", "follow_arc", "local_path_segment", "follow_local_path_segments"}
_TRAJECTORY_TYPES = {"trajectory", "local_planner_segment"}
_ADAPTIVE_TYPES = {"adaptive_direct", "search_person", "search_person_rotate"}
_LEGACY_TYPES = set()


def _stop_type(stop_status: dict) -> str:
    return str((stop_status or {}).get("type", "") or "").strip().upper()


def resolve_canonical_motion_mode(layer: str, command_type: str, stop_status: dict | None = None) -> str:
    layer_up = str(layer or "").strip().upper()
    ctype = str(command_type or "").strip().lower()
    s_type = _stop_type(stop_status or {})

    if s_type == "EMERGENCY_STOP" or ctype == "emergency_stop":
        return "EMERGENCY_STOP"
    if s_type == "SOFT_STOP" or ctype in {"soft_stop", "cancel_motion"}:
        return "SOFT_STOP"
    if layer_up == "IDLE" or ctype == "idle":
        return "IDLE"
    if ctype in _TELEOP_TYPES:
        return "TELEOP_VECTOR"
    if ctype in _TWIST_TYPES:
        return "TWIST"
    if ctype in _TRACK_TYPES:
        return "TRACK_REFERENCE"
    if ctype in _POSE_TYPES:
        return "POSE_TARGET"
    if ctype in _FOLLOW_TYPES:
        return "FOLLOW_TARGET"
    if ctype in _HEADING_TYPES:
        return "HEADING_TARGET"
    if ctype in _WAYPOINT_TYPES:
        return "WAYPOINT_MISSION"
    if ctype in _TRAJECTORY_TYPES:
        return "TRAJECTORY"
    if ctype in _ADAPTIVE_TYPES:
        return "ADAPTIVE_BEHAVIOR"
    return "UNCLASSIFIED"


def _entry_condition_met(ctrl, mode: str) -> bool:
    active_layer = str(getattr(ctrl, "active_motion_command_layer", "IDLE") or "IDLE").upper()
    ctype = str(getattr(ctrl, "active_motion_command_type", "idle") or "idle").lower()
    stop_status = dict(getattr(ctrl, "stop_status", {}) or {})
    s_type = _stop_type(stop_status)

    if mode == "IDLE":
        return active_layer == "IDLE"
    if mode == "SOFT_STOP":
        return s_type == "SOFT_STOP" or ctype in {"soft_stop", "cancel_motion"}
    if mode == "EMERGENCY_STOP":
        return s_type == "EMERGENCY_STOP" or ctype == "emergency_stop"
    return resolve_canonical_motion_mode(active_layer, ctype, stop_status) == mode


def _success_condition_met(ctrl, mode: str) -> bool:
    limited = dict(getattr(ctrl, "limited_motion_intent", {}) or {})
    v_abs = abs(float(limited.get("v", 0.0) or 0.0))
    w_abs = abs(float(limited.get("omega", 0.0) or 0.0))
    task = dict(getattr(ctrl, "motion_task_status", {}) or {})
    task_exec = str(task.get("execution_state", "") or "").lower()
    mission = dict(getattr(ctrl, "waypoint_mission_status", {}) or {})
    mission_exec = str(mission.get("execution_state", "") or "").lower()
    stop_status = dict(getattr(ctrl, "stop_status", {}) or {})
    s_type = _stop_type(stop_status)

    if mode in {"IDLE", "SOFT_STOP"}:
        return v_abs <= 0.01 and w_abs <= 0.02
    if mode == "EMERGENCY_STOP":
        return s_type == "EMERGENCY_STOP"
    if mode == "WAYPOINT_MISSION":
        return mission_exec == "succeeded"
    if mode in {"POSE_TARGET", "HEADING_TARGET", "TRACK_REFERENCE", "TWIST", "TRAJECTORY"}:
        return task_exec == "succeeded"
    return False


def _interrupt_reason(ctrl, mode: str, success_met: bool) -> str:
    stop_status = dict(getattr(ctrl, "stop_status", {}) or {})
    if bool(stop_status.get("active", False)):
        canonical = str(stop_status.get("canonical_reason", "") or "").strip()
        if canonical:
            return canonical
        reason = str(stop_status.get("reason", "") or "").strip()
        if reason:
            return reason
        s_type = _stop_type(stop_status)
        if s_type:
            return s_type

    task = dict(getattr(ctrl, "motion_task_status", {}) or {})
    terminal = str(task.get("terminal_reason", "") or "").strip()
    if terminal:
        return terminal

    if success_met:
        return ""
    return "NONE"


def build_contract_catalog() -> List[dict]:
    out: List[dict] = []
    for mode in sorted(MODE_SPECS.keys()):
        spec = MODE_SPECS[mode]
        out.append(
            {
                "mode": spec.mode,
                "entry_condition": spec.entry_condition,
                "success_condition": spec.success_condition,
                "interrupt_reasons": list(spec.interrupt_reasons),
                "operator_truth_fields": list(spec.operator_truth_fields),
            }
        )
    return out


def build_initial_motion_contract_status() -> dict:
    spec = MODE_SPECS["IDLE"]
    return {
        "contract_version": CONTRACT_VERSION,
        "motion_schema_version": MOTION_SCHEMA_VERSION,
        "mutually_exclusive": True,
        "exclusive_group": EXCLUSIVE_GROUP,
        "active_mode": "IDLE",
        "entry_condition": spec.entry_condition,
        "entry_condition_met": True,
        "success_condition": spec.success_condition,
        "success_condition_met": True,
        "interrupt_reason": "",
        "operator_truth_fields": list(spec.operator_truth_fields),
        "active_layer": "IDLE",
        "active_command_type": "idle",
        "active_source": "MANUAL",
        "execution_mode": "IDLE_EXEC",
    }


def update_motion_contract_runtime(ctrl) -> dict:
    stop_status = dict(getattr(ctrl, "stop_status", {}) or {})
    active_layer = str(getattr(ctrl, "active_motion_command_layer", "IDLE") or "IDLE")
    active_type = str(getattr(ctrl, "active_motion_command_type", "idle") or "idle")
    active_source = str(
        getattr(ctrl, "active_motion_command_source", getattr(ctrl, "motion_command_source", "MANUAL"))
        or "MANUAL"
    )
    mode = resolve_canonical_motion_mode(active_layer, active_type, stop_status=stop_status)
    spec = MODE_SPECS.get(mode, MODE_SPECS["UNCLASSIFIED"])
    entry_met = bool(_entry_condition_met(ctrl, mode))
    success_met = bool(_success_condition_met(ctrl, mode))
    status = {
        "contract_version": CONTRACT_VERSION,
        "motion_schema_version": MOTION_SCHEMA_VERSION,
        "mutually_exclusive": True,
        "exclusive_group": EXCLUSIVE_GROUP,
        "active_mode": str(mode),
        "entry_condition": str(spec.entry_condition),
        "entry_condition_met": bool(entry_met),
        "success_condition": str(spec.success_condition),
        "success_condition_met": bool(success_met),
        "interrupt_reason": str(_interrupt_reason(ctrl, mode, success_met)),
        "operator_truth_fields": list(spec.operator_truth_fields),
        "active_layer": str(active_layer),
        "active_command_type": str(active_type),
        "active_source": str(active_source),
        "execution_mode": str(getattr(ctrl, "motion_execution_mode", "IDLE_EXEC") or "IDLE_EXEC"),
    }
    ctrl.motion_contract_status = dict(status)
    return dict(status)
