"""Provider-neutral contracts for the host-side R2B4 Agent Core.

These contracts live above RobotInterface and never create robot, motor, safety,
config or evidence authority.  Providers only translate their wire format into
these small immutable objects.
"""
from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from types import MappingProxyType

from r2b4_voice.conversation_contracts import LLMDecision, RobotAction, goal_plan_copy
from r2b4_voice.llm_decision import build_decision_schema, parse_llm_decision
from v3.adapters.vision_media_contracts import VisionJpeg

AGENT_STEP_SCHEMA = "R2B4_AGENT_STEP_V1"
TOOL_RESULT_SCHEMA = "R2B4_AGENT_TOOL_RESULT_V1"


@dataclass(frozen=True, slots=True)
class AgentToolSpec:
    name: str
    description: str
    kind: str = "READ"
    arguments: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        name = self.name.strip() if isinstance(self.name, str) else ""
        if not name or len(name) > 96:
            raise ValueError("tool name must be 1..96 characters")
        if self.kind not in {"READ", "TUNING", "CONFIG_WRITE", "ROBOTICS"}:
            raise ValueError("invalid tool kind")
        description = self.description.strip() if isinstance(self.description, str) else ""
        if not description:
            raise ValueError("tool description must not be empty")
        args = dict(self.arguments)
        if any(not isinstance(k, str) or not k or not isinstance(v, str) for k, v in args.items()):
            raise ValueError("tool arguments must be string:string mappings")
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "description", description)
        object.__setattr__(self, "arguments", MappingProxyType(args))

    def to_jsonable(self) -> dict[str, object]:
        return {
            "name": self.name,
            "description": self.description,
            "kind": self.kind,
            "arguments": dict(self.arguments),
        }


@dataclass(frozen=True, slots=True)
class AgentToolRequest:
    name: str
    arguments: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        name = self.name.strip() if isinstance(self.name, str) else ""
        if not name:
            raise ValueError("tool request name must not be empty")
        if not isinstance(self.arguments, Mapping):
            raise ValueError("tool request arguments must be an object")
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "arguments", MappingProxyType(dict(self.arguments)))


@dataclass(frozen=True, slots=True)
class AgentToolResult:
    name: str
    status: str
    data: object | None = None
    error: str | None = None
    images: tuple[VisionJpeg, ...] = field(default=(), repr=False)

    def __post_init__(self) -> None:
        if not isinstance(self.images, tuple) or len(self.images) > 1 or any(not isinstance(image, VisionJpeg) for image in self.images):
            raise ValueError("agent result supports one canonical calibrated image")

    def to_jsonable(self) -> dict[str, object]:
        return {
            "schema": TOOL_RESULT_SCHEMA,
            "name": self.name,
            "status": self.status,
            "data": self.data,
            "error": self.error,
        }


@dataclass(frozen=True, slots=True)
class AgentModelReply:
    model: str
    spoken_text: str | None = None
    tool_request: AgentToolRequest | None = None
    robot_action: RobotAction | None = None
    goal_plan: Mapping[str, object] | None = None

    def __post_init__(self) -> None:
        selected = sum(item is not None for item in (self.spoken_text, self.tool_request, self.robot_action, self.goal_plan))
        if selected != 1:
            raise ValueError("agent reply must contain exactly one final, tool request or robot action")
        if self.goal_plan is not None:
            object.__setattr__(self, "goal_plan", goal_plan_copy(self.goal_plan))

    def to_decision(self) -> LLMDecision:
        if self.tool_request is not None:
            raise ValueError("tool request is not a final LLM decision")
        return LLMDecision(self.spoken_text, self.robot_action, self.model, self.goal_plan)


def build_agent_step_schema(
    tool_catalog: Sequence[Mapping[str, object]],
    action_catalog: Sequence[Mapping[str, object]],
) -> dict[str, object]:
    tool_names: list[str] = []
    for item in tool_catalog:
        name = item.get("name") if isinstance(item, Mapping) else None
        if isinstance(name, str) and name and name not in tool_names:
            tool_names.append(name)
    decision = build_decision_schema(action_catalog)
    properties = decision["properties"]
    return {
        "type": "object",
        "properties": {
            "kind": {"type": "string", "enum": ["final", "tool", "action", "plan"]},
            "spoken_text": {"type": ["string", "null"]},
            "tool_name": {"type": ["string", "null"], "enum": [None, *tool_names]},
            # Keeping arguments as a JSON string preserves a strict closed outer
            # schema while allowing organically growing R2B4 tool contracts.
            "tool_arguments_json": {"type": ["string", "null"]},
            "action_name": properties["action_name"],
            "action_parameters": properties["action_parameters"],
            "plan_json": {"type": ["string", "null"]},
        },
        "required": [
            "kind", "spoken_text", "tool_name", "tool_arguments_json",
            "action_name", "action_parameters", "plan_json",
        ],
        "additionalProperties": False,
    }


def parse_agent_model_reply(
    raw: object,
    *,
    model: str,
    tool_catalog: Sequence[Mapping[str, object]],
    action_catalog: Sequence[Mapping[str, object]],
) -> AgentModelReply:
    if not isinstance(raw, Mapping):
        raise ValueError("agent reply is not an object")
    expected = {
        "kind", "spoken_text", "tool_name", "tool_arguments_json",
        "action_name", "action_parameters",
    }
    if set(raw) not in (expected, expected | {"plan_json"}):
        raise ValueError("agent reply has unexpected fields")
    kind = raw.get("kind")
    if kind not in {"final", "tool", "action", "plan"}:
        raise ValueError("invalid agent reply kind")

    tool_names = {
        str(item["name"])
        for item in tool_catalog
        if isinstance(item, Mapping) and isinstance(item.get("name"), str)
    }
    spoken = raw.get("spoken_text")
    tool_name = raw.get("tool_name")
    tool_args_raw = raw.get("tool_arguments_json")
    plan_raw = raw.get("plan_json")
    if kind != "plan" and plan_raw is not None:
        raise ValueError("non-plan reply contains plan_json")

    if kind == "plan":
        if spoken is not None or tool_name is not None or tool_args_raw is not None or raw.get("action_name") is not None:
            raise ValueError("plan reply contains text, tool or action fields")
        params = raw.get("action_parameters")
        if not isinstance(params, Mapping) or any(value is not None for value in params.values()):
            raise ValueError("plan reply must have null action parameters")
        if not isinstance(plan_raw, str) or len(plan_raw.encode()) > 32_768:
            raise ValueError("plan_json must be a bounded JSON string")
        try:
            plan = json.loads(plan_raw)
        except json.JSONDecodeError as exc:
            raise ValueError("plan_json is not valid JSON") from exc
        return AgentModelReply(model=model, goal_plan=goal_plan_copy(plan))

    if kind == "final":
        if not isinstance(spoken, str) or not spoken.strip():
            raise ValueError("final agent reply requires spoken_text")
        if tool_name is not None or tool_args_raw is not None or raw.get("action_name") is not None:
            raise ValueError("final agent reply contains non-final fields")
        params = raw.get("action_parameters")
        if not isinstance(params, Mapping) or any(value is not None for value in params.values()):
            raise ValueError("final agent reply must have null action parameters")
        return AgentModelReply(model=model, spoken_text=spoken.strip())

    if kind == "tool":
        if spoken is not None or raw.get("action_name") is not None:
            raise ValueError("tool reply cannot contain text or robot action")
        params = raw.get("action_parameters")
        if not isinstance(params, Mapping) or any(value is not None for value in params.values()):
            raise ValueError("tool reply must have null action parameters")
        if not isinstance(tool_name, str) or tool_name not in tool_names:
            raise ValueError("tool_name is not present in the current tool catalog")
        if not isinstance(tool_args_raw, str):
            raise ValueError("tool request requires tool_arguments_json")
        try:
            arguments = json.loads(tool_args_raw)
        except json.JSONDecodeError as exc:
            raise ValueError("tool_arguments_json is not valid JSON") from exc
        if not isinstance(arguments, Mapping):
            raise ValueError("tool arguments must decode to an object")
        return AgentModelReply(model=model, tool_request=AgentToolRequest(tool_name, arguments))

    if spoken is not None or tool_name is not None or tool_args_raw is not None:
        raise ValueError("action reply contains text or tool fields")
    decision = parse_llm_decision(
        {
            "spoken_text": None,
            "action_name": raw.get("action_name"),
            "action_parameters": raw.get("action_parameters"),
        },
        model=model,
        action_catalog=action_catalog,
    )
    if decision.robot_action is None:
        raise ValueError("action reply requires a robot action")
    return AgentModelReply(model=model, robot_action=decision.robot_action)


__all__ = [
    "AGENT_STEP_SCHEMA",
    "TOOL_RESULT_SCHEMA",
    "AgentModelReply",
    "AgentToolRequest",
    "AgentToolResult",
    "AgentToolSpec",
    "build_agent_step_schema",
    "parse_agent_model_reply",
]
