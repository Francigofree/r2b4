"""Fresh-state host executor for the tiny allowlisted R2B4 voice action surface.

The LLM remains a proposal source only.  This executor performs a second,
fresh RobotInterface/context check immediately before delegating to the canonical
V3 command interface.  It never receives a runtime, motor, GPIO or L12 handle.
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Protocol

from .action_validation import RobotActionValidator
from .conversation_contracts import RobotAction
from .robot_context import RobotContextBuilder

_ALLOWED = frozenset(
    {
        "v3.command.stop",
        "v3.command.face_person",
        "v3.command.follow_person",
    }
)


class VoiceRobotInterface(Protocol):
    def capabilities(self) -> Mapping[str, object]: ...
    def read(self, resource: str) -> object: ...
    def execute(self, action: str, **parameters: object) -> object: ...


@dataclass(frozen=True, slots=True)
class VoiceActionExecution:
    status: str
    action_name: str | None
    executed: bool
    detail: str | None = None


class VoiceActionExecutor:
    """Validate a proposal twice and execute only the explicit high-level allowlist."""

    def __init__(
        self,
        interface: VoiceRobotInterface,
        *,
        mode: str = "shadow",
        session_owner_pid: int | None = None,
        session_watchdog_s: float = 30.0,
        validator: RobotActionValidator | None = None,
    ) -> None:
        normalized = str(mode).strip().lower()
        if normalized not in {"shadow", "execute"}:
            raise ValueError("mode must be shadow or execute")
        if (
            not isinstance(session_watchdog_s, (int, float))
            or isinstance(session_watchdog_s, bool)
            or not 1.0 <= float(session_watchdog_s) <= 600.0
        ):
            raise ValueError("session_watchdog_s must be within [1, 600]")
        owner = os.getpid() if session_owner_pid is None else session_owner_pid
        if not isinstance(owner, int) or isinstance(owner, bool) or owner <= 0:
            raise ValueError("session_owner_pid must be a positive integer")
        for method in ("capabilities", "read", "execute"):
            if not callable(getattr(interface, method, None)):
                raise TypeError(f"interface must provide {method}()")
        self._interface = interface
        self._mode = normalized
        self._owner_pid = owner
        self._watchdog_s = float(session_watchdog_s)
        self._validator = validator or RobotActionValidator()
        self._context = RobotContextBuilder(interface)

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def session_watchdog_s(self) -> float:
        return self._watchdog_s

    def execute_proposal(self, proposal: object) -> VoiceActionExecution:
        action = self._parse(proposal)
        if action is None:
            return VoiceActionExecution("NONE", None, False)
        if action.name not in _ALLOWED:
            return VoiceActionExecution("REJECTED:ACTION_NOT_ALLOWLISTED", action.name, False)

        # P0: never act on the context that was captured before the network/LLM roundtrip.
        fresh = self._context.build()
        validation = self._validator.validate(action, fresh)
        if not validation.accepted:
            return VoiceActionExecution(f"REJECTED:{validation.reason}", action.name, False)

        capability = next(
            (item for item in fresh.available_actions if item.get("name") == action.name),
            None,
        )
        if not isinstance(capability, Mapping) or capability.get("ready") is not True:
            reason = capability.get("reason") if isinstance(capability, Mapping) else None
            return VoiceActionExecution(
                "REJECTED:ACTION_NOT_READY",
                action.name,
                False,
                None if reason is None else str(reason),
            )

        fault_layer = fresh.runtime.get("fault_layer")
        state = str(fresh.runtime.get("state") or "").upper()
        if fault_layer or state in {"FAULT", "ERROR"}:
            return VoiceActionExecution("REJECTED:RUNTIME_FAULT", action.name, False)

        if self._mode == "shadow":
            return VoiceActionExecution("SHADOW_ACCEPTED", action.name, False)

        parameters: dict[str, object] = action.as_dict()
        if action.name != "v3.command.stop":
            # The positive motion session cannot outlive its voice owner indefinitely.
            parameters["session_owner_pid"] = self._owner_pid
            parameters["session_watchdog_s"] = self._watchdog_s
        self._interface.execute(action.name, **parameters)
        return VoiceActionExecution("EXECUTED", action.name, True)

    @staticmethod
    def _parse(proposal: object) -> RobotAction | None:
        if proposal is None:
            return None
        if isinstance(proposal, RobotAction):
            return proposal
        if not isinstance(proposal, Mapping):
            raise ValueError("voice action proposal must be a mapping")
        name = proposal.get("name")
        parameters = proposal.get("parameters", {})
        if not isinstance(name, str) or not isinstance(parameters, Mapping):
            raise ValueError("voice action proposal has invalid shape")
        normalized: list[tuple[str, float]] = []
        for key, value in parameters.items():
            if not isinstance(key, str):
                raise ValueError("voice action parameter name must be a string")
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise ValueError(f"voice action parameter {key} must be numeric")
            normalized.append((key, float(value)))
        return RobotAction(name=name, parameters=tuple(sorted(normalized)))


__all__ = ["VoiceActionExecution", "VoiceActionExecutor"]
