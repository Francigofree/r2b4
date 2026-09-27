"""Host-side execution-mode selection for R2B4 user requests.

This selector owns no robot state or authority. It classifies one already-typed
LLM decision against the fresh public capability snapshot.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .conversation_contracts import LLMDecision, RobotContextSnapshot


class ExecutionMode(str, Enum):
    CONVERSATION = "CONVERSATION"
    OBSERVATION = "OBSERVATION"
    ROBOT_ACTION = "ROBOT_ACTION"
    ER2_STREAM = "ER2_STREAM"
    ER2_PREVIEW = "ER2_PREVIEW"


@dataclass(frozen=True, slots=True)
class ExecutionPlan:
    mode: ExecutionMode
    observation_name: str | None = None


class ExecutionModeSelector:
    """Select a host execution path without acquiring any robot authority."""

    def select(
        self,
        decision: LLMDecision,
        context: RobotContextSnapshot,
    ) -> ExecutionPlan:
        if not isinstance(decision, LLMDecision):
            raise TypeError("decision must be LLMDecision")
        if not isinstance(context, RobotContextSnapshot):
            raise TypeError("context must be RobotContextSnapshot")
        if decision.robot_action is not None and decision.observation_name is not None:
            raise ValueError("one turn cannot request robot action and observation together")
        if decision.robot_action is not None:
            return ExecutionPlan(ExecutionMode.ROBOT_ACTION)
        if decision.observation_name is not None:
            capability = next(
                (
                    item
                    for item in context.available_observations
                    if item.get("name") == decision.observation_name
                ),
                None,
            )
            if capability is None:
                raise ValueError("observation is not advertised")
            if capability.get("available") is not True or capability.get("ready") is not True:
                raise ValueError("observation is not ready")
            return ExecutionPlan(
                ExecutionMode.OBSERVATION,
                observation_name=decision.observation_name,
            )
        return ExecutionPlan(ExecutionMode.CONVERSATION)


__all__ = ["ExecutionMode", "ExecutionModeSelector", "ExecutionPlan"]
