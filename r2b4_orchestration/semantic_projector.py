"""Project completed, compact V3 evidence into host semantic knowledge.

This adapter has no execution port: it cannot inject a costmap, localization,
safety state or command. Repeated publications preserve the original evidence.
"""
from __future__ import annotations

from collections.abc import Mapping

from .world_model import PublicWorldModel, ValidityScope


class SemanticProjector:
    def __init__(self, world: PublicWorldModel, *, clock_ns):
        self.world = world
        self.clock_ns = clock_ns
        self._last_status_identity = None
        self._last_behavior_revision = None

    def completed_status(self, status: Mapping[str, object], *, runtime_pid: object = None) -> None:
        stamp, tick = status.get("monotonic_ns"), status.get("tick_id")
        now = self.clock_ns()
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
