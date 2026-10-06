"""Single external facade for the R2B4 V3 robot and host-side tools.

The interface is deliberately not a new robot authority.  It delegates every
operation to an already-owned subsystem (V3 command ingress, OperatorController,
camera diagnostics, Test Hub, host status) and never writes motor/GPIO state
itself.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
import time
from typing import TYPE_CHECKING, Any, Protocol
import uuid

if TYPE_CHECKING:
    from r2b4_orchestration.world_model import WorldQuery, WorldQueryResult

from v3.action_catalog import ACTION_CATALOG_SCHEMA, action_catalog_jsonable
from v3.interface_adapters import build_adapters
from v3.operator_controller import OperatorController, OperatorEvent


ROBOT_INTERFACE_SCHEMA = "R2B4_ROBOT_INTERFACE_V2"
_OBSERVATION_TEXT_LIMIT = 256


@dataclass(frozen=True, slots=True)
class RobotInterfaceEvent:
    """Compact action evidence; request time is independent of publication time."""

    kind: str
    request_id: str
    action: str
    resolved_action: str
    measurement_time_ns: int
    observation_time_ns: int
    goal_id: str | None = None
    subtask_id: str | None = None
    decision_id: str | None = None
    behavior_id: str | None = None
    command_id: str | None = None
    mission_id: str | None = None
    runtime_pid: int | None = None
    status: str | None = None
    reason: str | None = None
    error_type: str | None = None

    def to_jsonable(self) -> dict[str, object]:
        return {"schema": "R2B4_ROBOT_INTERFACE_EVENT_V1", **asdict(self)}


def _observation_text(value: object) -> str | None:
    return value[:_OBSERVATION_TEXT_LIMIT] if isinstance(value, str) and value else None


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
        upper_runtime: bool = True,
        observation_sink: Callable[[RobotInterfaceEvent], None] | None = None,
        clock_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        root = Path(project_root) if project_root is not None else Path(__file__).resolve().parents[1]
        self.root = root.resolve()
        self.controller = controller or OperatorController(
            project_root=self.root,
            event_sink=event_sink,
        )
        composed = list(adapters if adapters is not None else build_adapters(self.controller, self.root))
        self._public_robot = None
        if adapters is None and upper_runtime:
            from r2b4_orchestration.robot_runtime import PublicRobotClient, PublicRobotInterfaceAdapter
            composed.append(PublicRobotInterfaceAdapter(PublicRobotClient(self.root), self.controller))
        for adapter in composed:
            if adapter.name == "public_robot":
                self._public_robot = adapter
        self._adapters: tuple[InterfaceAdapter, ...] = tuple(composed)
        self._observation_sink = observation_sink
        self._clock_ns = clock_ns

    def set_observation_sink(
        self,
        sink: Callable[[RobotInterfaceEvent], None] | None,
        *,
        clock_ns: Callable[[], int] | None = None,
    ) -> None:
        """Attach a bounded enqueue callback during host-runtime composition.

        The callback must not perform storage or consumer work. Its failure is
        passive evidence failure and never changes an action or STOP result.
        """
        self._observation_sink = sink
        if clock_ns is not None:
            self._clock_ns = clock_ns

    @property
    def adapters(self) -> tuple[InterfaceAdapter, ...]:
        """Reuse this composition when adding a conversation or UI adapter."""
        return self._adapters

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
                if self._public_robot is not None and name in {"v3.command.explore", "v3.command.follow_person"}:
                    item["execution_owner"] = "behavior_system"
                    item["behavior"] = "room_cruise" if name.endswith("explore") else "follow_person"
                items[name] = item
        for legacy, behavior in (("v3.command.explore", "behavior.room_cruise"),
                                 ("v3.command.follow_person", "behavior.follow_person")):
            if self._public_robot is not None and legacy in items and behavior in items:
                for key in ("available", "ready", "reason"):
                    items[legacy][key] = items[behavior][key]
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

    def query(self, query: WorldQuery) -> WorldQueryResult:
        """Read typed host knowledge without granting V3 state authority."""
        adapter, capability = self._resolve("world.query", expected_kind="read")
        if capability.get("supported") is not True or capability.get("available") is not True:
            raise RobotInterfaceError("public world query unavailable: " + str(capability.get("reason") or "UNAVAILABLE"))
        return adapter.query(query)

    def execute(self, action: str, **parameters: object) -> object:
        return self._execute_observed(action, parameters)

    def _execute_observed(self, action: str, parameters: Mapping[str, object]) -> object:
        resolved = action
        if self._public_robot is not None and action in {"v3.command.explore", "v3.command.follow_person"}:
            resolved = "behavior.room_cruise" if action.endswith("explore") else "behavior.follow_person"
        request = None
        if self._observation_sink is not None:
            try:
                request = (uuid.uuid4().hex, self._clock_ns())
            except Exception:
                pass
        # STOP reaches its physical owner before any observation callback.
        if resolved != "v3.command.stop":
            self._observe_action(request, "ACTION_REQUESTED", action, resolved, parameters)
        try:
            result = self._execute_action(resolved, parameters)
        except Exception as exc:
            self._observe_action(request, "ACTION_ERROR", action, resolved, parameters,
                                 error_type=type(exc).__name__)
            raise
        self._observe_action(request, "ACTION_RESULT", action, resolved, parameters, result=result)
        return result

    def _execute_action(self, action: str, parameters: Mapping[str, object]) -> object:
        if action == "v3.command.stop":
            if parameters:
                raise ValueError("STOP accepts no parameters")
            return self._stop()
        if self._public_robot is not None:
            if (action.startswith("v3.command.") or action in {
                "operator.runtime.start", "operator.runtime.stop", "operator.shutdown", "operator.panic", "operator.proba",
            }):
                self._preempt_upper("PREEMPTED_BY:" + action)
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
        return self._execute_observed("v3.command.stop", {})

    def _stop(self) -> object:

        # Revocation does not read world state/capabilities. Regardless of host
        # service availability, deliver the existing canonical V3 STOP.
        owners = [adapter for adapter in self._adapters if "v3.command.stop" in adapter.capability_names]
        if len(owners) != 1:
            raise RobotInterfaceError("canonical STOP must have exactly one adapter owner")
        if self._public_robot is not None:
            try:
                self._public_robot.client.revoke("STOP")
            except (OSError, RuntimeError, ValueError, TimeoutError):
                pass
        try:
            return owners[0].execute("v3.command.stop")
        finally:
            # The physical STOP above has priority over host-side draining/waiting.
            # Wait for any earlier submission to unwind and stop again there, so a
            # late STARTING action cannot outlive the completed STOP request.
            self._preempt_upper("STOP")

    def _observe_action(
        self,
        request: tuple[str, int] | None,
        kind: str,
        action: str,
        resolved_action: str,
        parameters: Mapping[str, object],
        *,
        result: object = None,
        error_type: str | None = None,
    ) -> None:
        sink = self._observation_sink
        if request is None or sink is None:
            return
        try:
            def field(name: str) -> object:
                value = result.get(name) if isinstance(result, Mapping) else getattr(result, name, None)
                return parameters.get(name) if value is None else value

            identities = {name: _observation_text(field(name)) for name in (
                "goal_id", "subtask_id", "decision_id", "behavior_id", "command_id", "mission_id",
            )}
            runtime_pid = field("runtime_pid")
            status = _observation_text(field("status")) or _observation_text(field("lifecycle"))
            sink(RobotInterfaceEvent(
                kind=kind, request_id=request[0], action=_observation_text(action) or "",
                resolved_action=_observation_text(resolved_action) or "",
                measurement_time_ns=request[1], observation_time_ns=self._clock_ns(),
                **identities,
                runtime_pid=runtime_pid if type(runtime_pid) is int and runtime_pid > 0 else None,
                status=status, reason=_observation_text(field("reason")),
                error_type=_observation_text(error_type),
            ))
        except Exception:
            # No serialization, filesystem I/O or action-result rewriting here.
            pass

    def _preempt_upper(self, reason: str) -> None:
        if self._public_robot is not None:
            try:
                self._public_robot.client.preempt(reason)
            except (OSError, RuntimeError, ValueError, TimeoutError):
                # Public knowledge/behavior availability is never a prerequisite
                # for stopping or admitting an ordinary local V3 operation.
                self.controller.stop()

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
    "RobotInterfaceEvent",
    "RobotInterfaceError",
]
