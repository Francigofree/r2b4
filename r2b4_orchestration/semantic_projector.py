"""Project completed, compact V3 evidence into host semantic knowledge.

This adapter has no execution port: it cannot inject a costmap, localization,
safety state or command. Repeated publications preserve the original evidence.
"""
from __future__ import annotations

import hashlib
import math
from collections.abc import Mapping

from .world_model import PublicWorldModel, ValidityScope


class SemanticProjector:
    def __init__(self, world: PublicWorldModel, *, clock_ns, person_identity=None):
        self.world = world
        self.clock_ns = clock_ns
        self.person_identity = person_identity
        self._last_status_identity = None
        self._last_behavior_revision = None
        self._last_geometry_identity = None

    def completed_status(self, status: Mapping[str, object], *, runtime_pid: object = None,
                         vision_status: Mapping[str, object] | None = None) -> None:
        stamp, tick = status.get("monotonic_ns"), status.get("tick_id")
        now = self.clock_ns()
        if self.person_identity is not None:
            self.person_identity.completed_status(status, runtime_pid=runtime_pid, vision_status=vision_status)
        if (type(stamp) is not int or not 0 <= stamp <= now
                or type(tick) is not int or tick < 0):
            return
        identity = (runtime_pid, tick, stamp)
        if identity == self._last_status_identity:
            return
        self._last_status_identity = identity
        source = f"v3:{runtime_pid}"
        lineage = (f"runtime:{runtime_pid}", f"tick:{tick}")
        common = dict(measurement_time_ns=stamp, observation_time_ns=now,
                      confidence=1.0, source=source, lineage=lineage, sequence=tick)
        mission = status.get("mission")
        for attribute, value, domain in (
            ("mission", mission, "robot_state"),
            ("health", status.get("source_health"), "health"),
            ("safety", {key: status.get(key) for key in (
                "state", "safety_decision", "safety_reason", "fault_layer", "enabled")}, "robot_state"),
        ):
            self.world.observe("robot", attribute, value, domain=domain, **common)

        bound_runtime = runtime_pid if type(runtime_pid) is int and runtime_pid > 0 else None
        estimate = status.get("estimate")
        if isinstance(estimate, Mapping):
            quality = estimate.get("localization_quality")
            quality = quality if isinstance(quality, Mapping) else {}
            confidence = {"GOOD": 1.0, "DEGRADED": 0.5, "LOST": 0.0}.get(quality.get("global_position"), 0.0)
            # Estimate time is a reference time, not a renewed sensor measurement.
            value = {key: estimate[key] for key in (
                "frame_id", "x_m", "y_m", "yaw_rad", "v_mps", "omega_rad_s",
                "local_pose", "global_pose", "map_to_odom", "localization_quality",
            ) if key in estimate}
            scope = ValidityScope(frame_id=value.get("frame_id"), runtime_pid=bound_runtime)
            self.world.observe("robot", "pose", value, domain="pose", validity_scope=scope,
                               **{**common, "confidence": confidence if bound_runtime else 0.0,
                                  "lineage": (*lineage, "L3:estimate_reference_time")})

        local = status.get("world")
        if isinstance(local, Mapping) and bound_runtime is not None:
            scope = ValidityScope(frame_id=local.get("frame_id"), runtime_pid=bound_runtime)
            if local.get("map_revision") is not None:
                self.world.observe("robot", "map_reference", {
                    "frame_id": local.get("frame_id"), "runtime_pid": bound_runtime,
                    "map_revision": local["map_revision"],
                }, domain="map", validity_scope=scope, **common)
            self._geometric_area(estimate, local, bound_runtime, stamp, tick, now)
            for track in local.get("person_tracks", ()):
                if not isinstance(track, Mapping):
                    continue
                measured, track_id = track.get("measurement_monotonic_ns"), track.get("track_id")
                if (type(measured) is not int or not 0 <= measured <= stamp
                        or not isinstance(track_id, str) or track.get("estimate_status") != "OBSERVED"):
                    continue
                self.world.observe(f"person:{bound_runtime}:{track_id}", "location", {
                    "x_m": track.get("x_m"), "y_m": track.get("y_m"),
                    "frame_id": local.get("frame_id"), "runtime_pid": bound_runtime,
                    "track_id": track_id,
                }, domain="person_position", validity_scope=scope,
                    **{**common, "measurement_time_ns": measured, "confidence": track.get("confidence", 0.0)})

        if not isinstance(mission, Mapping) or bound_runtime is None:
            return
        mission_id = mission.get("mission_id")
        if not isinstance(mission_id, str) or not mission_id:
            return
        entity = f"mission:{bound_runtime}:{mission_id}"
        if mission.get("lifecycle") in {"COMPLETED", "FAILED", "CANCELLED"}:
            self._outcome(entity, "mission_outcome", {
                key: mission.get(key) for key in ("mission_id", "mode", "lifecycle", "stop_reason")
            }, common)
        navigation = status.get("navigation")
        if (isinstance(navigation, Mapping) and navigation.get("mission_id") == mission_id
                and navigation.get("status") in {"COMPLETE", "NO_PATH", "INVALIDATED"}):
            self._outcome(entity, "navigation_outcome", {
                key: navigation.get(key) for key in ("mission_id", "status", "reason", "progress")
            }, common)

    def _geometric_area(self, estimate, local, runtime_pid, stamp, tick, now):
        """Remember a sampled area, without inferring a room or free space.

        L3 contributes a completed pose reference. L4 contributes the original
        scan time/sequence and compact geometry summary. An unchanged scan is
        never renewed by prediction, status polling or costmap maintenance.
        """
        if not isinstance(estimate, Mapping):
            return
        quality, costmap = estimate.get("localization_quality"), local.get("local_costmap")
        if not isinstance(quality, Mapping) or not isinstance(costmap, Mapping):
            return
        generation, transform = quality.get("generation"), estimate.get("transform_revision")
        frame = local.get("frame_id")
        measured, sequence = costmap.get("measurement_monotonic_ns"), costmap.get("source_sequence")
        map_revision, geometry_revision = local.get("map_revision"), costmap.get("revision")
        if (not isinstance(frame, str) or not 0 < len(frame) <= 256 or costmap.get("frame_id") != frame
                or any(type(value) is not int or value < 0 for value in (
                    generation, transform, measured, sequence, map_revision, geometry_revision))
                or not 0 <= measured <= stamp <= now
                or costmap.get("freshness_ns") != stamp - measured
                or quality.get("local_translation") != "GOOD" or quality.get("heading") != "GOOD"
                or quality.get("local_pose_continuous") is not True
                or quality.get("pose_discontinuity") is not False):
            return
        geometry_policy = self.world.policies["lidar_geometry"]
        if geometry_policy.max_age_ns is not None and now - measured > geometry_policy.max_age_ns:
            return
        pose = next((value for value in (estimate, estimate.get("local_pose"), estimate.get("global_pose"))
                     if isinstance(value, Mapping) and value.get("frame_id") == frame), None)
        local_pose = estimate.get("local_pose")
        local_frame = local_pose.get("frame_id") if isinstance(local_pose, Mapping) else "R2B4_ODOM_LOCAL"
        if pose is None or frame != local_frame and quality.get("global_position") != "GOOD":
            return
        coordinates = tuple(pose.get(key) for key in ("x_m", "y_m", "yaw_rad"))
        radius, resolution, occupied = (costmap.get(key) for key in (
            "radius_m", "resolution_m", "occupied_cell_count"))
        if (any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                for value in (*coordinates, radius, resolution)) or radius <= 0 or resolution <= 0
                or type(occupied) is not int or occupied < 0):
            return
        identity = (runtime_pid, frame, generation, measured, sequence)
        if identity == self._last_geometry_identity:
            return
        previous = self._last_geometry_identity
        if previous is not None and identity[:3] == previous[:3] and (
                measured <= previous[3] or sequence <= previous[4]):
            return
        # The identifier is a bounded frame-qualified one-metre area, not a
        # semantic room name or an assertion that the whole cell is traversable.
        x_m, y_m, yaw = coordinates
        cell = (math.floor(x_m), math.floor(y_m))
        frame_token = hashlib.sha256(frame.encode()).hexdigest()[:16]
        entity = f"geometry:{runtime_pid}:{generation}:{frame_token}:{cell[0]}:{cell[1]}"
        if len(entity) > 256:
            return
        previous = self.world.read(entity, "location").observation
        if (previous is not None and previous.clock_epoch == self.world.clock_epoch
                and measured <= previous.measurement_time_ns):
            return
        self.world.observe(entity, "location", {
            "kind": "visited_area", "x_m": x_m, "y_m": y_m, "yaw_rad": yaw,
            "frame_id": frame, "runtime_pid": runtime_pid,
            "localization_generation": generation, "transform_revision": transform,
            "pose_reference_time_ns": stamp, "pose_reference_tick": tick,
            "area_cell": list(cell), "area_resolution_m": 1.0,
            "geometry": {"measurement_time_ns": measured, "source_sequence": sequence,
                         "revision": geometry_revision, "radius_m": radius,
                         "resolution_m": resolution, "occupied_cell_count": occupied},
            "motion_authority": False,
        }, domain="map", measurement_time_ns=measured, observation_time_ns=now,
            confidence=0.5, source=f"v3:{runtime_pid}:L3/L4",
            sequence=sequence, revision=geometry_revision,
            validity_scope=ValidityScope(frame_id=frame, runtime_pid=runtime_pid, map_revision=map_revision),
            lineage=(f"runtime:{runtime_pid}", f"tick:{tick}", "L3:estimate_reference_time",
                     f"L3:generation:{generation}", f"L3:transform:{transform}",
                     f"L4:map:{map_revision}", f"L4:costmap:{geometry_revision}",
                     f"L4:source_sequence:{sequence}"))
        self._last_geometry_identity = identity

    def _outcome(self, entity: str, domain: str, value: dict, common: dict) -> None:
        previous = self.world.read(entity, domain)
        if previous.observation is not None and previous.value == value:
            return
        self.world.observe(entity, domain, value, domain=domain, **common)

    def completed_behavior(self, snapshot) -> None:
        if snapshot.revision == self._last_behavior_revision:
            return
        self.world.observe("robot", "behavior", snapshot.to_jsonable(), domain="behavior_state",
                           measurement_time_ns=snapshot.measurement_time_ns,
                           observation_time_ns=snapshot.observation_time_ns, confidence=1.0,
                           source="behavior_system", revision=snapshot.revision,
                           lineage=(f"behavior:{snapshot.behavior_id or 'IDLE'}", f"revision:{snapshot.revision}"))
        self._last_behavior_revision = snapshot.revision

    def completed_goal(self, event) -> None:
        """Remember terminal executive truth, never command acceptance as success."""
        goal = event.state
        if event.kind == "GOAL_RESTORED" or goal.lifecycle.value not in {"COMPLETED", "FAILED", "CANCELLED", "INTERRUPTED"}:
            return
        value = {"goal_id": goal.goal_id, "text": goal.text, "source": goal.source,
                 "lifecycle": goal.lifecycle.value, "reason": goal.reason,
                 "created_ns": goal.created_ns, "updated_ns": goal.updated_ns,
                 "step_index": goal.step_index, "step_count": len(goal.steps),
                 "subtask_id": goal.subtask_id, "decision_id": goal.decision_id,
                 "behavior_id": goal.behavior_id, "command_id": goal.command_id,
                 "mission_id": goal.mission_id, "target": dict(goal.target), "result": dict(goal.result)}
        lineage = [f"goal:{goal.goal_id}", f"subtask:{goal.subtask_id}",
                   f"decision:{goal.decision_id}"]
        for name in ("behavior_id", "command_id", "mission_id"):
            identity = getattr(goal, name)
            if identity is not None:
                lineage.append(f"{name}:{identity}")
        if goal.world_target is not None:
            fact = goal.world_target
            value["world_target"] = {"entity_id": fact.entity_id, "world_revision": fact.world_revision,
                "clock_epoch": fact.observation.clock_epoch,
                "measurement_time_ns": fact.observation.measurement_time_ns,
                "validity_scope": fact.observation.validity_scope.to_jsonable()}
        self.world.observe(f"goal:{goal.goal_id}", "outcome", value, domain="task_outcome",
                           measurement_time_ns=event.measurement_time_ns, observation_time_ns=self.clock_ns(),
                           confidence=1.0, source=f"brain:{event.producer_id}", sequence=event.sequence,
                           revision=goal.revision, lineage=lineage)
