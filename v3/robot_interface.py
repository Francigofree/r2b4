"""Single external facade for the R2B4 V3 robot and host-side tools.

The interface is deliberately not a new robot authority.  It delegates every
operation to an already-owned subsystem (V3 command ingress, OperatorController,
camera diagnostics, Test Hub, host status) and never writes motor/GPIO state
itself.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any, Protocol

from v3.action_catalog import ACTION_CATALOG_SCHEMA, action_catalog_jsonable
from v3.interface_adapters import build_adapters
from v3.operator_controller import OperatorController, OperatorEvent


ROBOT_INTERFACE_SCHEMA = "R2B4_ROBOT_INTERFACE_V2"


class RobotInterfaceError(RuntimeError):
    """An external interface request is unsupported, unavailable or invalid."""


class InterfaceAdapter(Protocol):
    """Small host-side adapter contract; not a V3 runtime contract."""

    name: str
    capability_names: frozenset[str]

    def capabilities(self) -> Mapping[str, Mapping[str, object]]: ...

    def read(self, resource: str) -> object: ...

    def execute(self, action: str, **parameters: object) -> object: ...


class RobotInterface:
    """One public facade for external R2B4 clients.

    Live capability state is generated from adapters on every request. Static
    canonical v3.command action contracts come from v3.action_catalog; no duplicated
    live robot state is maintained here.
    """

    def __init__(
        self,
        project_root: Path | str | None = None,
        *,
        event_sink: Callable[[OperatorEvent], None] | None = None,
        controller: OperatorController | None = None,
        adapters: Sequence[InterfaceAdapter] | None = None,
    ) -> None:
        root = Path(project_root) if project_root is not None else Path(__file__).resolve().parents[1]
        self.root = root.resolve()
        self.controller = controller or OperatorController(
            project_root=self.root,
            event_sink=event_sink,
        )
        self._adapters: tuple[InterfaceAdapter, ...] = tuple(
            adapters if adapters is not None else build_adapters(self.controller, self.root)
        )

    def capabilities(self) -> dict[str, object]:
        """Return the current live external surface, with duplicate names rejected."""

        items: dict[str, dict[str, object]] = {}
        for adapter in self._adapters:
            for name, raw in adapter.capabilities().items():
                if name in items:
                    raise RobotInterfaceError(f"duplicate interface capability: {name}")
                item = dict(raw)
                kind = item.get("kind")
                if kind not in {"read", "action"}:
                    raise RobotInterfaceError(
                        f"invalid capability kind from {adapter.name}: {name}={kind!r}"
                    )
                item.setdefault("supported", True)
                item.setdefault("available", True)
                item.setdefault("ready", bool(item["available"]))
                item["adapter"] = adapter.name
                items[name] = item
        return {
            "schema": ROBOT_INTERFACE_SCHEMA,
            "action_catalog_schema": ACTION_CATALOG_SCHEMA,
            "action_catalog": action_catalog_jsonable(),
            "capabilities": dict(sorted(items.items())),
        }

    def read(self, resource: str) -> object:
        adapter, capability = self._resolve(resource, expected_kind="read")
        if capability.get("supported") is not True:
            raise RobotInterfaceError(f"resource is not supported: {resource}")
        if capability.get("available") is not True:
            reason = capability.get("reason") or "UNAVAILABLE"
            raise RobotInterfaceError(f"resource unavailable: {resource}: {reason}")
        return adapter.read(resource)

    def execute(self, action: str, **parameters: object) -> object:
        adapter, capability = self._resolve(action, expected_kind="action")
        if capability.get("supported") is not True:
            raise RobotInterfaceError(f"action is not supported: {action}")
        if capability.get("available") is not True:
            reason = capability.get("reason") or "UNAVAILABLE"
            raise RobotInterfaceError(f"action unavailable: {action}: {reason}")
        # ``ready`` is informative rather than a generic hard gate.  Some
        # actions (notably motion) are allowed to transition the host/runtime
        # into readiness through the canonical OperatorController path.
        return adapter.execute(action, **parameters)

    def stop(self) -> object:
        """First-class fail-safe external STOP request."""

        return self.execute("v3.command.stop")

    def _resolve(
        self,
        name: str,
        *,
        expected_kind: str,
    ) -> tuple[InterfaceAdapter, Mapping[str, object]]:
        owners = [adapter for adapter in self._adapters if name in adapter.capability_names]
        if not owners:
            raise RobotInterfaceError(f"unknown interface {expected_kind}: {name}")
        if len(owners) != 1:
            raise RobotInterfaceError(f"ambiguous interface capability: {name}")
        adapter = owners[0]
        capability = adapter.capabilities().get(name)
        if capability is None:
            raise RobotInterfaceError(
                f"adapter {adapter.name} declared but did not report capability: {name}"
            )
        if capability.get("kind") != expected_kind:
            raise RobotInterfaceError(
                f"{name} is {capability.get('kind')!r}, not {expected_kind!r}"
            )
        return adapter, capability


__all__ = [
    "InterfaceAdapter",
    "ROBOT_INTERFACE_SCHEMA",
    "RobotInterface",
    "RobotInterfaceError",
]
