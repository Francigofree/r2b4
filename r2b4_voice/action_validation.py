"""Side-effect-free validation of LLM-proposed high-level robot actions."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from .conversation_contracts import RobotAction, RobotContextSnapshot


@dataclass(frozen=True, slots=True)
class ActionValidation:
    accepted: bool
    reason: str


class RobotActionValidator:
    """Validate only high-level allowlisted intents.

    Validation itself is side-effect-free.  Physical execution, when explicitly
    enabled, belongs to ``VoiceActionExecutor`` and performs a separate fresh-state
    gate immediately before delegating to the canonical ``RobotInterface``.
    """

    _LIMITS = {
        "v3.command.stop": {},
        "v3.command.face_person": {"max_omega_rad_s": (0.01, 0.50)},
        "v3.command.follow_person": {
            "max_v_mps": (0.01, 0.15),
            "max_omega_rad_s": (0.01, 0.30),
        },
    }

    def validate(self, action: RobotAction, context: RobotContextSnapshot) -> ActionValidation:
        if action.name not in self._LIMITS:
            return ActionValidation(False, "ACTION_NOT_ALLOWLISTED")
        capability = self._find_capability(action.name, context)
        if capability is None:
            return ActionValidation(False, "ACTION_NOT_ADVERTISED")
        if capability.get("available") is not True:
            return ActionValidation(False, "ACTION_UNAVAILABLE")

        params = action.as_dict()
        limits = self._LIMITS[action.name]
        unknown = set(params) - set(limits)
        if unknown:
            return ActionValidation(False, "UNKNOWN_PARAMETER")
        if action.name == "v3.command.stop" and params:
            return ActionValidation(False, "STOP_TAKES_NO_PARAMETERS")
        for key, value in params.items():
            low, high = limits[key]
            if not low <= value <= high:
                return ActionValidation(False, f"PARAMETER_OUT_OF_RANGE:{key}")
        return ActionValidation(True, "SHADOW_ACCEPTED")

    @staticmethod
    def _find_capability(name: str, context: RobotContextSnapshot) -> Mapping[str, object] | None:
        for item in context.available_actions:
            if item.get("name") == name:
                return item
        return None


__all__ = ["ActionValidation", "RobotActionValidator"]
