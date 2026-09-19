"""Provider-neutral structured LLM decision schema and parser."""

from __future__ import annotations

from collections.abc import Mapping

from .conversation_contracts import LLMDecision, RobotAction


class DecisionParseError(ValueError):
    pass


DECISION_SCHEMA: dict[str, object] = {
    "type": "object",
    "properties": {
        "spoken_text": {"type": ["string", "null"]},
        "action_name": {
            "type": ["string", "null"],
            "enum": [
                None,
                "v3.command.stop",
                "v3.command.face_person",
                "v3.command.follow_person",
            ],
        },
        "action_parameters": {
            "type": "object",
            "properties": {
                "max_v_mps": {"type": ["number", "null"]},
                "max_omega_rad_s": {"type": ["number", "null"]},
            },
            "required": ["max_v_mps", "max_omega_rad_s"],
            "additionalProperties": False,
        },
    },
    "required": ["spoken_text", "action_name", "action_parameters"],
    "additionalProperties": False,
}


def parse_llm_decision(raw: object, *, model: str) -> LLMDecision:
    if not isinstance(raw, Mapping):
        raise DecisionParseError("LLM decision is not an object")
    allowed_keys = {"spoken_text", "action_name", "action_parameters"}
    if set(raw) != allowed_keys:
        raise DecisionParseError("LLM decision has unexpected fields")

    spoken_text = raw.get("spoken_text")
    if spoken_text is not None and not isinstance(spoken_text, str):
        raise DecisionParseError("spoken_text must be string or null")

    action_name = raw.get("action_name")
    if action_name is not None and not isinstance(action_name, str):
        raise DecisionParseError("action_name must be string or null")
    if action_name not in {None, "v3.command.stop", "v3.command.face_person", "v3.command.follow_person"}:
        raise DecisionParseError("action_name is not allowlisted")

    params_raw = raw.get("action_parameters")
    if not isinstance(params_raw, Mapping):
        raise DecisionParseError("action_parameters must be an object")
    if set(params_raw) != {"max_v_mps", "max_omega_rad_s"}:
        raise DecisionParseError("action_parameters has unexpected fields")

    action: RobotAction | None = None
    if action_name is not None:
        params: list[tuple[str, float]] = []
        for key in ("max_v_mps", "max_omega_rad_s"):
            value = params_raw.get(key)
            if value is None:
                continue
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise DecisionParseError(f"{key} must be numeric or null")
            params.append((key, float(value)))
        action = RobotAction(action_name, tuple(params))
    elif any(params_raw.get(key) is not None for key in ("max_v_mps", "max_omega_rad_s")):
        raise DecisionParseError("action_parameters must be null-valued when action_name is null")

    try:
        return LLMDecision(spoken_text, action, model)
    except ValueError as exc:
        raise DecisionParseError(str(exc)) from exc


__all__ = ["DECISION_SCHEMA", "DecisionParseError", "parse_llm_decision"]
