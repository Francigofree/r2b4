"""Build a compact live robot context from the canonical RobotInterface."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Protocol

from .conversation_contracts import RobotContextSnapshot


ROBOT_CONTEXT_SCHEMA = "R2B4_ROBOT_CONTEXT_V1"
LLM_ACTION_ALLOWLIST = (
    "v3.command.stop",
    "v3.command.face_person",
    "v3.command.follow_person",
)


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

        runtime: dict[str, object] = {"state": "UNAVAILABLE", "ready_for_active": False}
        status = self._read_if_available(caps, "v3.status")
        if isinstance(status, Mapping):
            runtime = {
                "state": status.get("state"),
                "ready_for_active": status.get("ready_for_active"),
                "tick_id": status.get("tick_id"),
                "enabled": status.get("enabled"),
                "fault_layer": status.get("fault_layer"),
            }

        pose = self._mapping_or_none(self._read_if_available(caps, "v3.pose"))
        safety = self._mapping_or_none(self._read_if_available(caps, "v3.safety"))
        raw_health = self._read_if_available(caps, "v3.health")
        health = tuple(raw_health) if isinstance(raw_health, list) else ()

        actions: list[dict[str, object]] = []
        for name in LLM_ACTION_ALLOWLIST:
            raw = caps.get(name)
            if not isinstance(raw, Mapping):
                continue
            if raw.get("kind") != "action" or raw.get("supported") is not True:
                continue
            actions.append({
                "name": name,
                "available": raw.get("available") is True,
                "ready": raw.get("ready") is True,
                "reason": raw.get("reason"),
            })

        return RobotContextSnapshot(
            schema=ROBOT_CONTEXT_SCHEMA,
            runtime=runtime,
            pose=pose,
            safety=safety,
            health=health,
            available_actions=tuple(actions),
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


__all__ = ["LLM_ACTION_ALLOWLIST", "ROBOT_CONTEXT_SCHEMA", "RobotContextBuilder"]
