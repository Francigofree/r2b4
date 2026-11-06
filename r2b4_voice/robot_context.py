"""Build a compact live robot context from the canonical RobotInterface."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Protocol

from v3.action_catalog import action_descriptor

from .conversation_contracts import RobotContextSnapshot


ROBOT_CONTEXT_SCHEMA = "R2B4_ROBOT_CONTEXT_V6"


class RobotReadInterface(Protocol):
    def capabilities(self) -> Mapping[str, object]: ...
    def read(self, resource: str) -> object: ...


class RobotContextBuilder:
    def __init__(self, interface: RobotReadInterface) -> None:
        if not callable(getattr(interface, "capabilities", None)) or not callable(getattr(interface, "read", None)):
            raise TypeError("interface must provide capabilities() and read()")
        self._interface = interface

    def build(self) -> RobotContextSnapshot:
        raw_caps = self._interface.capabilities()
        caps = raw_caps.get("capabilities") if isinstance(raw_caps, Mapping) else None
        caps = caps if isinstance(caps, Mapping) else {}

        operator_status = self._read_if_available(caps, "operator.status")
        runtime_running: bool | None = None
        if isinstance(operator_status, Mapping) and isinstance(operator_status.get("runtime_running"), bool):
            runtime_running = bool(operator_status.get("runtime_running"))

        status = self._read_if_available(caps, "v3.status")
        # The default LLM turn carries only fresh execution-local state.  The
        # persistent Public World Model is intentionally query-on-demand through
        # world.query / robot.read; injecting its full snapshot made prompt size
        # grow with robot history and duplicated robot.state.
        environment: dict[str, object] = {
            "local_world": None,
            "behavior": self._mapping_or_none(self._read_if_available(caps, "behavior.state")),
            "person": {"available": False, "detected": None, "tracks": []},
            "mission": None,
            "navigation": None,
        }
        if isinstance(status, Mapping):
            world = self._mapping_or_none(status.get("world"))
            mission = self._mapping_or_none(status.get("mission"))
            navigation = self._mapping_or_none(status.get("navigation"))
            environment["local_world"] = world
            environment["mission"] = mission
            environment["navigation"] = navigation
            if world is not None and isinstance(world.get("person_tracks"), list):
                tracks = list(world.get("person_tracks", []))
                environment["person"] = {
                    "available": True,
                    "detected": bool(tracks),
                    "tracks": tracks,
                }

        host = {
            "operator_status_available": isinstance(operator_status, Mapping),
            "runtime_running": runtime_running,
            "runtime_state": (
                "RUNNING" if runtime_running is True
                else "STOPPED" if runtime_running is False
                else "UNKNOWN"
            ),
            "environment": environment,
            # brain.state is small, current task authority data.  Do not add
            # robot.state here: robot.state embeds world + brain + behavior and
            # therefore duplicates the same state in the default LLM payload.
            "brain": self._brain_summary(self._read_if_available(caps, "brain.state")),
        }

        if isinstance(status, Mapping):
            runtime: dict[str, object] = {
                "live_status_available": True,
                "state": status.get("state"),
                "ready_for_active": status.get("ready_for_active"),
                "tick_id": status.get("tick_id"),
                "enabled": status.get("enabled"),
                "fault_layer": status.get("fault_layer"),
                "interpretation": "LIVE_STATUS",
            }
        else:
            state = "STOPPED" if runtime_running is False else "UNAVAILABLE"
            runtime = {
                "live_status_available": False,
                "state": state,
                "ready_for_active": False,
                "tick_id": None,
                "enabled": False,
                "fault_layer": None,
                "interpretation": (
                    "RUNTIME_STOPPED" if state == "STOPPED" else "LIVE_STATUS_UNAVAILABLE"
                ),
            }

        pose = self._mapping_or_none(self._read_if_available(caps, "v3.pose"))
        safety = self._mapping_or_none(self._read_if_available(caps, "v3.safety"))
        raw_health = self._read_if_available(caps, "v3.health")
        health = tuple(raw_health) if isinstance(raw_health, list) else ()

        person = environment.get("person")
        person_known = isinstance(person, Mapping) and person.get("available") is True
        person_detected = isinstance(person, Mapping) and person.get("detected") is True

        actions: list[dict[str, object]] = []
        for name, raw in sorted(caps.items()):
            if not isinstance(raw, Mapping) or raw.get("kind") != "action" or raw.get("supported") is not True:
                continue
            descriptor = action_descriptor(name)
            # Keep the full canonical descriptor internally for validation and
            # structured-output schema generation. RobotContextSnapshot serializes
            # only live status fields, so the descriptor is not duplicated in the
            # textual ROBOT_CONTEXT layer.
            if descriptor is None or descriptor.voice_exposed is not True:
                continue
            ready = raw.get("ready") is True
            reason = raw.get("reason")
            if "person_target" in descriptor.requirements and person_known and not person_detected:
                ready = False
                reason = "PERSON_TARGET_UNAVAILABLE"
            item = descriptor.to_jsonable()
            item.update({
                "available": raw.get("available") is True,
                "ready": ready,
                "reason": reason,
            })
            actions.append(item)

        return RobotContextSnapshot(
            schema=ROBOT_CONTEXT_SCHEMA,
            runtime=runtime,
            pose=pose,
            safety=safety,
            health=health,
            available_actions=tuple(actions),
            host=host,
        )

    def _read_if_available(self, caps: Mapping[str, object], name: str) -> object | None:
        cap = caps.get(name)
        if not isinstance(cap, Mapping) or cap.get("kind") != "read" or cap.get("available") is not True:
            return None
        try:
            return self._interface.read(name)
        except Exception:
            return None

    @staticmethod
    def _mapping_or_none(value: object) -> dict[str, object] | None:
        return dict(value) if isinstance(value, Mapping) else None

    @staticmethod
    def _brain_summary(value: object) -> dict[str, object] | None:
        """Current task metadata; full graphs/results remain on brain.state."""
        if not isinstance(value, Mapping):
            return None
        summary = {name: value[name] for name in ("schema", "identity", "role", "revision", "personality")
                   if name in value}
        def goal(raw):
            if not isinstance(raw, Mapping):
                return None
            # Do not duplicate completed motor/navigation results, graphs or
            # historical user text in every new specialist request. Constraints
            # remain exact; the current user text is separately preserved by
            # PromptAssembler. IDs/times retain correlation with full read tools.
            fields = ("goal_id", "decision_id", "subtask_id", "lifecycle", "priority",
                      "current_subtask", "current_node_id", "step_index", "attempt",
                      "failure_code", "reason", "constraints", "target", "revision", "updated_ns")
            return {name: raw[name] for name in fields if name in raw}
        summary["primary_goal"] = goal(value.get("primary_goal"))
        for name in ("pending_goals", "background_goals"):
            raw = value.get(name)
            if isinstance(raw, (list, tuple)):
                summary[name] = [goal(item) for item in raw[:4]]
                summary[name + "_count"] = len(raw)
                summary[name + "_truncated"] = len(raw) > 4
        summary["details_resource"] = "brain.state"
        return summary


__all__ = ["ROBOT_CONTEXT_SCHEMA", "RobotContextBuilder"]
