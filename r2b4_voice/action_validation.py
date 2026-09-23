"""Side-effect-free validation of LLM-proposed canonical robot actions."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

from v3.action_catalog import action_descriptor

from .conversation_contracts import RobotAction, RobotContextSnapshot


PROPOSAL_ACCEPTED = "PROPOSAL_ACCEPTED"


@dataclass(frozen=True, slots=True)
class ActionValidation:
    accepted: bool
    reason: str


class RobotActionValidator:
    """Validate proposals against the canonical V3 ActionDescriptor catalog."""

    def validate(self, action: RobotAction, context: RobotContextSnapshot) -> ActionValidation:
        descriptor = action_descriptor(action.name)
        if descriptor is None:
            return ActionValidation(False, "ACTION_NOT_CATALOGED")
        if descriptor.voice_exposed is not True:
            return ActionValidation(False, "ACTION_NOT_EXPOSED")

        capability = self._find_capability(action.name, context)
        if capability is None:
            return ActionValidation(False, "ACTION_NOT_ADVERTISED")
        if capability.get("available") is not True:
            return ActionValidation(False, "ACTION_UNAVAILABLE")

        params = action.as_dict()
        specs = {item.name: item for item in descriptor.parameters}
        unknown = set(params) - set(specs)
        if unknown:
            return ActionValidation(False, "UNKNOWN_PARAMETER")

        for name, spec in specs.items():
            if spec.required and name not in params:
                return ActionValidation(False, f"MISSING_REQUIRED_PARAMETER:{name}")
        for name, value in params.items():
            if not math.isfinite(float(value)):
                return ActionValidation(False, f"PARAMETER_NOT_FINITE:{name}")
            spec = specs[name]
            if spec.minimum is not None and value < spec.minimum:
                return ActionValidation(False, f"PARAMETER_OUT_OF_RANGE:{name}")
            if spec.maximum is not None and value > spec.maximum:
                return ActionValidation(False, f"PARAMETER_OUT_OF_RANGE:{name}")
        return ActionValidation(True, PROPOSAL_ACCEPTED)

    @staticmethod
    def _find_capability(name: str, context: RobotContextSnapshot) -> Mapping[str, object] | None:
        for item in context.available_actions:
            if item.get("name") == name:
                return item
        return None


__all__ = ["PROPOSAL_ACCEPTED", "ActionValidation", "RobotActionValidator"]
