"""Protocol-neutral, fail-closed external gateway above :mod:`v3.robot_interface`.

This module is host-side only. It owns no V3 runtime state, command mailbox,
motor path, safety state or capability registry. Every live operation is
delegated to the existing RobotInterface facade.
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Protocol


EXTERNAL_GATEWAY_SCHEMA = "R2B4_EXTERNAL_GATEWAY_V1"
_MAX_REQUEST_ID_CHARS = 128
_MAX_NAME_CHARS = 160
_MAX_PARAMETER_COUNT = 32

_SESSION_OWNED_ACTIONS = frozenset(
    {
        "v3.command.forward",
        "v3.command.backward",
        "v3.command.teleop",
        "v3.command.wheels",
        "v3.command.explore",
        "v3.command.face_person",
        "v3.command.follow_person",
    }
)
_RESERVED_SESSION_PARAMETERS = frozenset(
    {"session_owner_pid", "session_watchdog_s"}
)


class ExternalRobotInterface(Protocol):
    def capabilities(self) -> Mapping[str, object]: ...
    def read(self, resource: str) -> object: ...
    def execute(self, action: str, **parameters: object) -> object: ...
    def stop(self) -> object: ...


@dataclass(frozen=True, slots=True)
class GatewayPolicy:
    """Small local policy gate; RobotInterface remains the command facade."""

    allow_execute: bool = False
    session_owner_pid: int = 0
    session_watchdog_s: float = 30.0

    def __post_init__(self) -> None:
        if type(self.allow_execute) is not bool:
            raise TypeError("allow_execute must be bool")
        owner = self.session_owner_pid or os.getpid()
        if not isinstance(owner, int) or isinstance(owner, bool) or owner <= 0:
            raise ValueError("session_owner_pid must be a positive integer")
        watchdog = self.session_watchdog_s
        if (
            not isinstance(watchdog, (int, float))
            or isinstance(watchdog, bool)
            or not 1.0 <= float(watchdog) <= 600.0
        ):
            raise ValueError("session_watchdog_s must be within [1, 600]")
        object.__setattr__(self, "session_owner_pid", owner)
        object.__setattr__(self, "session_watchdog_s", float(watchdog))


@dataclass(frozen=True, slots=True)
class ExternalRequest:
    request_id: str
    operation: str
    name: str | None = None
    parameters: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        request_id = self.request_id.strip() if isinstance(self.request_id, str) else ""
        if not request_id or len(request_id) > _MAX_REQUEST_ID_CHARS:
            raise ValueError("request_id must be 1..128 characters")
        operation = self.operation.strip().lower() if isinstance(self.operation, str) else ""
        if operation not in {"capabilities", "read", "execute", "stop"}:
            raise ValueError("operation must be capabilities, read, execute or stop")
        name = self.name
        if name is not None:
            if not isinstance(name, str) or not name.strip() or len(name.strip()) > _MAX_NAME_CHARS:
                raise ValueError("name must be a non-empty string up to 160 characters")
            name = name.strip()
        if operation in {"read", "execute"} and name is None:
            raise ValueError(f"{operation} requires name")
        if operation in {"capabilities", "stop"} and name is not None:
            raise ValueError(f"{operation} does not accept name")
        if not isinstance(self.parameters, Mapping):
            raise ValueError("parameters must be a mapping")
        params = dict(self.parameters)
        if len(params) > _MAX_PARAMETER_COUNT:
            raise ValueError("too many parameters")
        for key in params:
            if not isinstance(key, str) or not key or len(key) > 96:
                raise ValueError("parameter names must be non-empty strings up to 96 characters")
        if operation in {"capabilities", "read", "stop"} and params:
            raise ValueError(f"{operation} does not accept parameters")
        object.__setattr__(self, "request_id", request_id)
        object.__setattr__(self, "operation", operation)
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "parameters", MappingProxyType(params))

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> "ExternalRequest":
        if not isinstance(value, Mapping):
            raise ValueError("request must be an object")
        allowed = {"request_id", "operation", "name", "parameters"}
        unknown = sorted(set(value) - allowed)
        if unknown:
            raise ValueError(f"unknown request fields: {', '.join(unknown)}")
        return cls(
            request_id=value.get("request_id"),  # type: ignore[arg-type]
            operation=value.get("operation"),  # type: ignore[arg-type]
            name=value.get("name"),  # type: ignore[arg-type]
            parameters=value.get("parameters", {}),  # type: ignore[arg-type]
        )


@dataclass(frozen=True, slots=True)
class ExternalResponse:
    request_id: str | None
    status: str
    result: object | None = None
    error: str | None = None

    def __post_init__(self) -> None:
        if self.status not in {
            "COMPLETED",
            "ACCEPTED",
            "REJECTED",
            "DENIED",
            "UNAVAILABLE",
            "FAULTED",
        }:
            raise ValueError("invalid external response status")

    def to_jsonable(self) -> dict[str, object]:
        return {
            "schema": EXTERNAL_GATEWAY_SCHEMA,
            "request_id": self.request_id,
            "status": self.status,
            "result": self.result,
            "error": self.error,
        }


class ExternalRobotGateway:
    """Validate one external request and delegate to the canonical RobotInterface."""

    def __init__(
        self,
        interface: ExternalRobotInterface,
        *,
        policy: GatewayPolicy | None = None,
    ) -> None:
        for method in ("capabilities", "read", "execute"):
            if not callable(getattr(interface, method, None)):
                raise TypeError(f"interface must provide {method}()")
        if not callable(getattr(interface, "stop", None)):
            raise TypeError("interface must provide stop()")
        resolved_policy = GatewayPolicy() if policy is None else policy
        if not isinstance(resolved_policy, GatewayPolicy):
            raise TypeError("policy must be GatewayPolicy")
        self._interface = interface
        self._policy = resolved_policy

    @property
    def policy(self) -> GatewayPolicy:
        return self._policy

    def handle(self, request: ExternalRequest | Mapping[str, object]) -> ExternalResponse:
        request_id: str | None = None
        try:
            parsed = request if isinstance(request, ExternalRequest) else ExternalRequest.from_mapping(request)
            request_id = parsed.request_id
        except (TypeError, ValueError) as exc:
            return ExternalResponse(request_id, "REJECTED", error=str(exc))

        try:
            if parsed.operation == "capabilities":
                return ExternalResponse(parsed.request_id, "COMPLETED", self._interface.capabilities())
            if parsed.operation == "read":
                assert parsed.name is not None
                return ExternalResponse(parsed.request_id, "COMPLETED", self._interface.read(parsed.name))
            if parsed.operation == "stop":
                return ExternalResponse(parsed.request_id, "COMPLETED", self._interface.stop())
            assert parsed.operation == "execute" and parsed.name is not None

            if parsed.name == "v3.command.stop":
                if parsed.parameters:
                    return ExternalResponse(parsed.request_id, "REJECTED", error="STOP_TAKES_NO_PARAMETERS")
                return ExternalResponse(parsed.request_id, "COMPLETED", self._interface.stop())
            if not self._policy.allow_execute:
                return ExternalResponse(
                    parsed.request_id,
                    "DENIED",
                    error="POSITIVE_ACTION_EXECUTION_DISABLED",
                )

            parameters = dict(parsed.parameters)
            reserved = sorted(set(parameters) & _RESERVED_SESSION_PARAMETERS)
            if reserved:
                return ExternalResponse(
                    parsed.request_id,
                    "REJECTED",
                    error="gateway owns session parameters: " + ", ".join(reserved),
                )
            if parsed.name in _SESSION_OWNED_ACTIONS:
                parameters["session_owner_pid"] = self._policy.session_owner_pid
                parameters["session_watchdog_s"] = self._policy.session_watchdog_s
            result = self._interface.execute(parsed.name, **parameters)
            return ExternalResponse(parsed.request_id, "ACCEPTED", result)
        except (KeyError, TypeError, ValueError) as exc:
            return ExternalResponse(parsed.request_id, "REJECTED", error=f"{type(exc).__name__}: {exc}")
        except RuntimeError as exc:
            message = str(exc)
            status = "UNAVAILABLE" if "unavailable" in message.lower() else "FAULTED"
            return ExternalResponse(parsed.request_id, status, error=f"{type(exc).__name__}: {exc}")
        except Exception as exc:
            return ExternalResponse(parsed.request_id, "FAULTED", error=f"{type(exc).__name__}: {exc}")


__all__ = [
    "EXTERNAL_GATEWAY_SCHEMA",
    "ExternalRequest",
    "ExternalResponse",
    "ExternalRobotGateway",
    "GatewayPolicy",
]
