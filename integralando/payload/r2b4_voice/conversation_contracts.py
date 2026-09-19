"""Immutable host-side contracts for R2B4 conversation/LLM integration.

These are deliberately not V3 L0-L12 contracts. They describe the host-side
conversation path above RobotInterface.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping


class ConversationContractError(ValueError):
    pass


def _nonempty(value: str, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ConversationContractError(f"{name} must be a non-empty string")
    return value.strip()


def _optional_text(value: str | None, name: str) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str):
        raise ConversationContractError(f"{name} must be a string or None")
    value = value.strip()
    return value or None


@dataclass(frozen=True, slots=True)
class UserTextTurn:
    turn_id: str
    text: str
    source: str
    monotonic_ns: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "turn_id", _nonempty(self.turn_id, "turn_id"))
        text = _nonempty(self.text, "text")
        if len(text) > 4000:
            raise ConversationContractError("text exceeds 4000 characters")
        object.__setattr__(self, "text", text)
        object.__setattr__(self, "source", _nonempty(self.source, "source"))
        if not isinstance(self.monotonic_ns, int) or isinstance(self.monotonic_ns, bool) or self.monotonic_ns < 0:
            raise ConversationContractError("monotonic_ns must be a non-negative integer")


@dataclass(frozen=True, slots=True)
class RobotAction:
    name: str
    parameters: tuple[tuple[str, float], ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", _nonempty(self.name, "RobotAction.name"))
        keys: list[str] = []
        normalized: list[tuple[str, float]] = []
        for key, value in self.parameters:
            key = _nonempty(key, "RobotAction.parameter key")
            if key in keys:
                raise ConversationContractError(f"duplicate RobotAction parameter: {key}")
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise ConversationContractError(f"RobotAction parameter {key} must be numeric")
            keys.append(key)
            normalized.append((key, float(value)))
        object.__setattr__(self, "parameters", tuple(normalized))

    def as_dict(self) -> dict[str, float]:
        return dict(self.parameters)


@dataclass(frozen=True, slots=True)
class LLMDecision:
    spoken_text: str | None
    robot_action: RobotAction | None
    model: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "spoken_text", _optional_text(self.spoken_text, "spoken_text"))
        if self.robot_action is not None and not isinstance(self.robot_action, RobotAction):
            raise ConversationContractError("robot_action must be RobotAction or None")
        object.__setattr__(self, "model", _nonempty(self.model, "model"))
        if self.spoken_text is None and self.robot_action is None:
            raise ConversationContractError("LLMDecision must contain spoken_text and/or robot_action")


@dataclass(frozen=True, slots=True)
class ConversationMemoryTurn:
    user_text: str
    assistant_text: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "user_text", _nonempty(self.user_text, "user_text"))
        object.__setattr__(self, "assistant_text", _nonempty(self.assistant_text, "assistant_text"))


@dataclass(frozen=True, slots=True)
class RobotContextSnapshot:
    schema: str
    runtime: Mapping[str, object]
    pose: Mapping[str, object] | None
    safety: Mapping[str, object] | None
    health: tuple[object, ...]
    available_actions: tuple[Mapping[str, object], ...]
    host: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "schema", _nonempty(self.schema, "schema"))
        object.__setattr__(self, "runtime", MappingProxyType(dict(self.runtime)))
        if self.pose is not None:
            object.__setattr__(self, "pose", MappingProxyType(dict(self.pose)))
        if self.safety is not None:
            object.__setattr__(self, "safety", MappingProxyType(dict(self.safety)))
        object.__setattr__(self, "health", tuple(self.health))
        object.__setattr__(
            self,
            "available_actions",
            tuple(MappingProxyType(dict(item)) for item in self.available_actions),
        )
        object.__setattr__(self, "host", MappingProxyType(dict(self.host)))

    def to_jsonable(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "host": dict(self.host),
            "runtime": dict(self.runtime),
            "pose": None if self.pose is None else dict(self.pose),
            "safety": None if self.safety is None else dict(self.safety),
            "health": list(self.health),
            "available_actions": [dict(item) for item in self.available_actions],
        }


@dataclass(frozen=True, slots=True)
class ConversationTurnResult:
    turn_id: str
    user_text: str
    spoken_text: str | None
    proposed_action: RobotAction | None
    action_status: str
    error: str | None
    model: str | None

    def __post_init__(self) -> None:
        object.__setattr__(self, "turn_id", _nonempty(self.turn_id, "turn_id"))
        object.__setattr__(self, "user_text", _nonempty(self.user_text, "user_text"))
        object.__setattr__(self, "spoken_text", _optional_text(self.spoken_text, "spoken_text"))
        if self.proposed_action is not None and not isinstance(self.proposed_action, RobotAction):
            raise ConversationContractError("proposed_action must be RobotAction or None")
        object.__setattr__(self, "action_status", _nonempty(self.action_status, "action_status"))
        object.__setattr__(self, "error", _optional_text(self.error, "error"))
        object.__setattr__(self, "model", _optional_text(self.model, "model"))

    def to_jsonable(self) -> dict[str, object]:
        action = None
        if self.proposed_action is not None:
            action = {
                "name": self.proposed_action.name,
                "parameters": self.proposed_action.as_dict(),
            }
        return {
            "turn_id": self.turn_id,
            "user_text": self.user_text,
            "spoken_text": self.spoken_text,
            "proposed_action": action,
            "action_status": self.action_status,
            "error": self.error,
            "model": self.model,
        }


__all__ = [
    "ConversationContractError",
    "ConversationMemoryTurn",
    "ConversationTurnResult",
    "LLMDecision",
    "RobotAction",
    "RobotContextSnapshot",
    "UserTextTurn",
]
