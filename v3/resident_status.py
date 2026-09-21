"""Compact passive resident status projection and sidecar-owned file output."""

from __future__ import annotations

import json
import os
import stat
import threading
import time
from collections.abc import Mapping
from pathlib import Path

from v3.contracts import AcquisitionFrame, MissionIntent, NavigationPlan, RobotEstimate, WorldSnapshot
from v3.engine import TickResult

RESIDENT_PROCESS_STATUS_SCHEMA = "R2B4_V3_RESIDENT_PROCESS_STATUS_V1"


def _atomic_private_json(path: Path, payload: Mapping[str, object], mode: int) -> None:
    parent = path.parent
    parent.mkdir(parents=True, exist_ok=True)
    if parent.is_symlink() or not parent.is_dir():
        raise ValueError("status parent must be a regular directory")
    rendered = (json.dumps(dict(payload), sort_keys=True) + "\n").encode("utf-8")
    temporary = parent / (
        f".{path.name}.tmp.{os.getpid()}.{threading.get_ident()}.{time.monotonic_ns()}"
    )
    descriptor: int | None = None
    try:
        descriptor = os.open(
            temporary,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0),
            mode,
        )
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise ValueError("temporary status target must be a regular file")
        view = memoryview(rendered)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise OSError("status write made no progress")
            view = view[written:]
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = None
        os.replace(temporary, path)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _layer(result: TickResult, name: str) -> object | None:
    for record in result.trace.layers:
        if record.layer == name:
            return record.output
    return None


def _tick_status(
    result: TickResult,
    ready_for_active: bool = False,
) -> dict[str, object]:
    if type(ready_for_active) is not bool:
        raise TypeError("ready_for_active must be bool")
    acquisition = _layer(result, "L1")
    estimate = _layer(result, "L3")
    world = _layer(result, "L4")
    mission = _layer(result, "L5")
    navigation = _layer(result, "L6")
    health: list[dict[str, object]] = []
    if isinstance(acquisition, AcquisitionFrame):
        health = [
            {
                "device_id": item.device_id,
                "state": item.state.value,
                "reason": item.reason,
            }
            for item in acquisition.io_health
        ]
    estimate_payload: dict[str, object] | None = None
    if isinstance(estimate, RobotEstimate):
        estimate_payload = {
            "frame_id": estimate.frame_id,
            "x_m": estimate.x_m,
            "y_m": estimate.y_m,
            "yaw_rad": estimate.yaw_rad,
            "v_mps": estimate.v_mps,
            "omega_rad_s": estimate.omega_rad_s,
        }

    world_payload: dict[str, object] | None = None
    if isinstance(world, WorldSnapshot):
        person_tracks = [
            {
                "track_id": track.track_id,
                "x_m": track.x_m,
                "y_m": track.y_m,
                "radius_m": track.radius_m,
                "vx_mps": track.vx_mps,
                "vy_mps": track.vy_mps,
                "confidence": track.confidence,
            }
            for track in world.obstacle_tracks
            if track.track_id.startswith("person-")
        ]
        world_payload = {
            "frame_id": world.frame_id,
            "map_revision": world.map_revision,
            "freshness_ns": world.freshness_ns,
            "obstacle_track_count": len(world.obstacle_tracks),
            "person_tracks": person_tracks,
            "local_costmap": (
                None
                if world.local_costmap is None
                else {
                    "revision": world.local_costmap.revision,
                    "occupied_cell_count": len(world.local_costmap.occupied_cells),
                    "freshness_ns": world.local_costmap.freshness_ns,
                    "radius_m": world.local_costmap.radius_m,
                    "resolution_m": world.local_costmap.resolution_m,
                }
            ),
        }

    mission_payload: dict[str, object] | None = None
    if isinstance(mission, MissionIntent):
        mission_payload = {
            "mission_id": mission.mission_id,
            "mode": mission.mode.value,
            "lifecycle": mission.lifecycle.value,
            "stop_reason": mission.stop_reason,
            "constraints": {
                "max_v_mps": mission.constraints.max_v_mps,
                "max_omega_rad_s": mission.constraints.max_omega_rad_s,
            },
            "target_pose": (
                None
                if mission.target_pose is None
                else {
                    "x_m": mission.target_pose.x_m,
                    "y_m": mission.target_pose.y_m,
                    "yaw_rad": mission.target_pose.yaw_rad,
                }
            ),
        }

    navigation_payload: dict[str, object] | None = None
    if isinstance(navigation, NavigationPlan):
        navigation_payload = {
            "mission_id": navigation.mission_id,
            "status": navigation.status.value,
            "reason": navigation.reason,
            "progress": navigation.progress,
            "route_waypoint_count": len(navigation.route),
            "trajectory_candidate_count": len(navigation.trajectory_candidates),
            "local_goal": (
                None
                if navigation.local_goal is None
                else {
                    "x_m": navigation.local_goal.x_m,
                    "y_m": navigation.local_goal.y_m,
                    "yaw_rad": navigation.local_goal.yaw_rad,
                }
            ),
        }

    final = result.final_actuation
    return {
        "schema": RESIDENT_PROCESS_STATUS_SCHEMA,
        "state": "RUNNING",
        "tick_id": result.trace.context.tick_id,
        "monotonic_ns": result.trace.context.monotonic_ns,
        "fault_layer": result.trace.fault_layer,
        "safety_decision": final.safety_decision.value,
        "safety_reason": final.reason,
        "enabled": final.enabled,
        "left_output": final.left_output,
        "right_output": final.right_output,
        "ready_for_active": ready_for_active,
        "source_health": health,
        "estimate": estimate_payload,
        "world": world_payload,
        "mission": mission_payload,
        "navigation": navigation_payload,
    }


