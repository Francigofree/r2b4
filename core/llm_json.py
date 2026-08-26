#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LLM communication layer: STRICTLY JSON-ONLY.
Fixed schema validation and TaskFactory. No optional fields or dynamic schema.
"""

from __future__ import annotations

from .task_model import RobotTask, TaskType, TaskPriority

# FIXED schema: intent, params (distance_m, angle_deg, target_id | steps for SEQUENCE), confidence
VALID_INTENTS = ("MOVE", "ROTATE", "FOLLOW", "STOP", "QUERY", "SEQUENCE")
# Steps inside SEQUENCE may only be motion intents (no STOP/QUERY in chain)
STEP_INTENTS = ("MOVE", "ROTATE", "FOLLOW")


def _validate_params_common(params: dict) -> tuple[bool, str]:
    """Shared param checks: distance_m, angle_deg, target_id types."""
    if not isinstance(params, dict):
        return False, "params must be object"
    for key in ("distance_m", "angle_deg", "target_id"):
        if key not in params:
            return False, f"params must contain: {key}"
    if params.get("distance_m") is not None and not isinstance(params["distance_m"], (int, float)):
        return False, "params.distance_m must be number or null"
    if params.get("angle_deg") is not None and not isinstance(params["angle_deg"], (int, float)):
        return False, "params.angle_deg must be number or null"
    if params.get("target_id") is not None and not isinstance(params["target_id"], str):
        return False, "params.target_id must be string or null"
    return True, ""


def _validate_step(step: dict) -> tuple[bool, str]:
    """Validate one step in a SEQUENCE: intent in STEP_INTENTS, params object with optional keys."""
    if not isinstance(step, dict):
        return False, "each step must be object"
    if "intent" not in step:
        return False, "step must have intent"
    if step["intent"] not in STEP_INTENTS:
        return False, f"step intent must be one of {STEP_INTENTS}"
    if "params" not in step:
        return False, "step must have params"
    p = step["params"]
    if not isinstance(p, dict):
        return False, "step params must be object"
    for key in ("distance_m", "angle_deg", "target_id"):
        if key not in p:
            return False, f"step params must contain: {key}"
    if p.get("distance_m") is not None and not isinstance(p["distance_m"], (int, float)):
        return False, "step params.distance_m must be number or null"
    if p.get("angle_deg") is not None and not isinstance(p["angle_deg"], (int, float)):
        return False, "step params.angle_deg must be number or null"
    if p.get("target_id") is not None and not isinstance(p["target_id"], str):
        return False, "step params.target_id must be string or null"
    return True, ""


def validate_llm_json(data: dict) -> tuple[bool, str]:
    """
    Validates parsed JSON against the FIXED schema.
    Returns (ok: bool, reason: str). reason is empty when ok is True.
    """
    if not isinstance(data, dict):
        return False, "root must be object"

    if "intent" not in data:
        return False, "missing field: intent"
    intent = data["intent"]
    if not isinstance(intent, str):
        return False, "intent must be string"
    if intent not in VALID_INTENTS:
        return False, f"intent must be one of {VALID_INTENTS}"

    if "params" not in data:
        return False, "missing field: params"
    params = data["params"]
    if not isinstance(params, dict):
        return False, "params must be object"

    if intent == "SEQUENCE":
        if "steps" not in params:
            return False, "SEQUENCE params must contain: steps"
        steps = params["steps"]
        if not isinstance(steps, list):
            return False, "params.steps must be array"
        if not steps:
            return False, "params.steps must not be empty"
        for i, step in enumerate(steps):
            ok, reason = _validate_step(step)
            if not ok:
                return False, f"steps[{i}]: {reason}"
    else:
        ok, reason = _validate_params_common(params)
        if not ok:
            return False, reason

    if "confidence" not in data:
        return False, "missing field: confidence"
    if not isinstance(data["confidence"], (int, float)):
        return False, "confidence must be number"

    return True, ""


def _single_intent_to_tasks(intent: str, params: dict) -> list[RobotTask]:
    """
    Converts one motion intent + params to a list of RobotTasks (0 or 1).
    Used for single-intent JSON and for each step in SEQUENCE.
    """
    tasks: list[RobotTask] = []

    if intent == "MOVE":
        distance_m = params.get("distance_m")
        if distance_m is None:
            distance_m = 0.5
        distance_m = float(distance_m)
        if distance_m < 0:
            distance_m = abs(distance_m)
            direction = -1
        else:
            direction = 1
        tasks.append(RobotTask(
            type=TaskType.MOVE,
            params={
                "distance": distance_m,
                "direction": direction,
                "speed_level": 3,
            },
            priority=TaskPriority.HIGH,
        ))
        return tasks

    if intent == "ROTATE":
        angle_deg = params.get("angle_deg")
        if angle_deg is None:
            angle_deg = 90.0
        angle_deg = float(angle_deg)
        tasks.append(RobotTask(
            type=TaskType.TURN,
            params={"angle": angle_deg},
            priority=TaskPriority.HIGH,
        ))
        return tasks

    if intent == "FOLLOW":
        tasks.append(RobotTask(
            type=TaskType.APPROACH,
            params={"target_id": params.get("target_id")},
            priority=TaskPriority.HIGH,
        ))
        return tasks

    return tasks


def task_factory(data: dict) -> list[RobotTask]:
    """
    Converts VALID (already validated) JSON into existing TaskQueue-compatible RobotTasks.
    SEQUENCE expands into multiple tasks in order; single intents into 0 or 1 task.
    """
    intent = data["intent"]
    params = data["params"]
    tasks: list[RobotTask] = []

    if intent == "STOP":
        tasks.append(RobotTask(type=TaskType.STOP, priority=TaskPriority.CRITICAL))
        return tasks

    if intent == "QUERY":
        return tasks

    if intent == "SEQUENCE":
        for step in params["steps"]:
            step_intent = step["intent"]
            step_params = step["params"]
            tasks.extend(_single_intent_to_tasks(step_intent, step_params))
        return tasks

    tasks.extend(_single_intent_to_tasks(intent, params))
    return tasks
