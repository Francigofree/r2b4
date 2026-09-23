"""Provider-neutral structured LLM decision schema generated from the action catalog."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

from v3.action_catalog import voice_action_descriptors

from .conversation_contracts import LLMDecision, RobotAction


class DecisionParseError(ValueError):
    pass


def _default_catalog() -> tuple[Mapping[str, object], ...]:
    return tuple(item.to_jsonable() for item in voice_action_descriptors())


def _catalog_items(action_catalog: Sequence[Mapping[str, object]] | None) -> tuple[Mapping[str, object], ...]:
    source = _default_catalog() if action_catalog is None else tuple(action_catalog)
    items: list[Mapping[str, object]] = []
    seen: set[str] = set()
    for item in source:
        if not isinstance(item, Mapping):
            continue
        name = item.get("name")
        if not isinstance(name, str) or not name.startswith("v3.command."):
            continue
        if item.get("voice_exposed") is False:
            continue
        if name in seen:
            raise DecisionParseError(f"duplicate action in dynamic catalog: {name}")
        seen.add(name)
        items.append(item)
    return tuple(items)


def build_decision_schema(action_catalog: Sequence[Mapping[str, object]] | None = None) -> dict[str, object]:
    items = _catalog_items(action_catalog)
    names = [item["name"] for item in items]
    merged: dict[str, dict[str, object]] = {}
    for item in items:
        parameters = item.get("parameters")
        if not isinstance(parameters, Mapping):
            continue
        for name, raw in parameters.items():
            if not isinstance(name, str) or not isinstance(raw, Mapping):
                continue
            slot = merged.setdefault(name, {"type": ["number", "null"]})
            minimum = raw.get("minimum")
            maximum = raw.get("maximum")
            if isinstance(minimum, (int, float)) and not isinstance(minimum, bool):
                old = slot.get("minimum")
                slot["minimum"] = float(minimum) if old is None else min(float(old), float(minimum))
            if isinstance(maximum, (int, float)) and not isinstance(maximum, bool):
                old = slot.get("maximum")
                slot["maximum"] = float(maximum) if old is None else max(float(old), float(maximum))

    return {
        "type": "object",
        "properties": {
            "spoken_text": {"type": ["string", "null"]},
            "action_name": {"type": ["string", "null"], "enum": [None, *names]},
            "action_parameters": {
                "type": "object",
                "properties": dict(sorted(merged.items())),
                # Strict providers require a closed object shape. Every catalog
                # parameter is present but may be null when another action uses it.
                "required": sorted(merged),
                "additionalProperties": False,
            },
        },
        "required": ["spoken_text", "action_name", "action_parameters"],
        "additionalProperties": False,
    }


DECISION_SCHEMA: dict[str, object] = build_decision_schema()


def parse_llm_decision(
    raw: object,
    *,
    model: str,
    action_catalog: Sequence[Mapping[str, object]] | None = None,
) -> LLMDecision:
    if not isinstance(raw, Mapping):
        raise DecisionParseError("LLM decision is not an object")
    allowed_keys = {"spoken_text", "action_name", "action_parameters"}
    if set(raw) != allowed_keys:
        raise DecisionParseError("LLM decision has unexpected fields")

    spoken_text = raw.get("spoken_text")
    if spoken_text is not None and not isinstance(spoken_text, str):
        raise DecisionParseError("spoken_text must be string or null")

    items = _catalog_items(action_catalog)
    by_name = {str(item["name"]): item for item in items}
    union_params: set[str] = set()
    for item in items:
        params = item.get("parameters")
        if isinstance(params, Mapping):
            union_params.update(str(key) for key in params)

    action_name = raw.get("action_name")
    if action_name is not None and not isinstance(action_name, str):
        raise DecisionParseError("action_name must be string or null")
    if action_name is not None and action_name not in by_name:
        raise DecisionParseError("action_name is not present in the current action catalog")

    params_raw = raw.get("action_parameters")
    if not isinstance(params_raw, Mapping):
        raise DecisionParseError("action_parameters must be an object")
    unknown_schema_keys = set(params_raw) - union_params
    if unknown_schema_keys:
        raise DecisionParseError("action_parameters has unexpected fields")

    if action_name is None:
        if any(value is not None for value in params_raw.values()):
            raise DecisionParseError("action_parameters must be null-valued when action_name is null")
        action: RobotAction | None = None
    else:
        selected = by_name[action_name]
        selected_params = selected.get("parameters")
        selected_params = selected_params if isinstance(selected_params, Mapping) else {}
        params: list[tuple[str, float]] = []
        for key, value in params_raw.items():
            if value is None:
                continue
            if key not in selected_params:
                raise DecisionParseError(f"parameter is not valid for {action_name}: {key}")
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
                raise DecisionParseError(f"{key} must be a finite number or null")
            spec = selected_params[key]
            if isinstance(spec, Mapping):
                minimum = spec.get("minimum")
                maximum = spec.get("maximum")
                if isinstance(minimum, (int, float)) and value < minimum:
                    raise DecisionParseError(f"{key} is below minimum")
                if isinstance(maximum, (int, float)) and value > maximum:
                    raise DecisionParseError(f"{key} exceeds maximum")
            params.append((str(key), float(value)))
        for key, spec in selected_params.items():
            if isinstance(spec, Mapping) and spec.get("required") is True and params_raw.get(key) is None:
                raise DecisionParseError(f"missing required action parameter: {key}")
        action = RobotAction(action_name, tuple(sorted(params)))

    try:
        return LLMDecision(spoken_text, action, model)
    except ValueError as exc:
        raise DecisionParseError(str(exc)) from exc


__all__ = ["DECISION_SCHEMA", "DecisionParseError", "build_decision_schema", "parse_llm_decision"]
