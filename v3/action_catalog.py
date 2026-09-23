"""Canonical machine-readable R2B4 robot action catalog.

This module is the single source of truth for the static contract of canonical
``v3.command.*`` actions exposed through :class:`v3.robot_interface.RobotInterface`.
It owns no runtime state and no actuation authority. Live ``available``, ``ready``
and ``reason`` values remain owned by the interface adapters.
"""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping


ACTION_CATALOG_SCHEMA = "R2B4_ACTION_CATALOG_V1"
ACTION_DESCRIPTOR_SCHEMA = "R2B4_ACTION_DESCRIPTOR_V1"

# Canonical default FOLLOW envelope. All operator/agent ingress surfaces import
# these values so a stale UI/adapter default cannot silently reintroduce a
# lower nominal speed. Dynamic safety/geometry layers may still tighten it.
FOLLOW_PERSON_DEFAULT_MAX_V_MPS = 0.25
FOLLOW_PERSON_DEFAULT_MAX_OMEGA_RAD_S = 0.30


@dataclass(frozen=True, slots=True)
class ActionParameterDescriptor:
    name: str
    description: str
    required: bool = False
    minimum: float | None = None
    maximum: float | None = None
    default: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("parameter name must be non-empty")
        if not isinstance(self.description, str) or not self.description.strip():
            raise ValueError(f"parameter description must be non-empty: {self.name}")
        if type(self.required) is not bool:
            raise TypeError(f"parameter required must be bool: {self.name}")
        for value, label in ((self.minimum, "minimum"), (self.maximum, "maximum"), (self.default, "default")):
            if value is not None and (not isinstance(value, (int, float)) or isinstance(value, bool)):
                raise TypeError(f"{self.name}.{label} must be numeric or None")
        if self.minimum is not None and self.maximum is not None and self.minimum > self.maximum:
            raise ValueError(f"{self.name}: minimum exceeds maximum")
        if self.default is not None:
            if self.minimum is not None and self.default < self.minimum:
                raise ValueError(f"{self.name}: default below minimum")
            if self.maximum is not None and self.default > self.maximum:
                raise ValueError(f"{self.name}: default above maximum")
        if self.required and self.default is not None:
            raise ValueError(f"{self.name}: required parameter must not define a default")

    def to_jsonable(self) -> dict[str, object]:
        return {
            "type": "number",
            "description": self.description,
            "required": self.required,
            "minimum": self.minimum,
            "maximum": self.maximum,
            "default": self.default,
        }


@dataclass(frozen=True, slots=True)
class ActionDescriptor:
    name: str
    description: str
    parameters: tuple[ActionParameterDescriptor, ...] = ()
    voice_exposed: bool = False
    session_watchdog: bool = False
    requirements: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.startswith("v3.command."):
            raise ValueError("action name must be canonical v3.command.*")
        if not isinstance(self.description, str) or not self.description.strip():
            raise ValueError(f"action description must be non-empty: {self.name}")
        if type(self.voice_exposed) is not bool:
            raise TypeError(f"voice_exposed must be bool: {self.name}")
        if type(self.session_watchdog) is not bool:
            raise TypeError(f"session_watchdog must be bool: {self.name}")
        names = [item.name for item in self.parameters]
        if len(names) != len(set(names)):
            raise ValueError(f"duplicate action parameter: {self.name}")
        if len(self.requirements) != len(set(self.requirements)):
            raise ValueError(f"duplicate action requirement: {self.name}")
        if any(not isinstance(item, str) or not item for item in self.requirements):
            raise ValueError(f"action requirements must be non-empty strings: {self.name}")

    def parameter(self, name: str) -> ActionParameterDescriptor | None:
        return next((item for item in self.parameters if item.name == name), None)

    def to_jsonable(self) -> dict[str, object]:
        return {
            "schema": ACTION_DESCRIPTOR_SCHEMA,
            "name": self.name,
            "description": self.description,
            "voice_exposed": self.voice_exposed,
            "session_watchdog": self.session_watchdog,
            "requirements": list(self.requirements),
            "parameters": {item.name: item.to_jsonable() for item in self.parameters},
        }


def _p(name: str, description: str, *, required: bool = False,
       minimum: float | None = None, maximum: float | None = None,
       default: float | None = None) -> ActionParameterDescriptor:
    return ActionParameterDescriptor(name, description, required, minimum, maximum, default)


# All current canonical robot actions are deliberately voice_exposed=True for this
# development phase. The flag remains fail-closed by default so later selection is
# a one-field catalog decision, not a voice-system refactor.
_DESCRIPTORS = (
    ActionDescriptor(
        "v3.command.stop",
        "Stop current robot motion through the canonical fail-safe command path.",
        voice_exposed=True,
    ),
    ActionDescriptor(
        "v3.command.forward",
        "Drive forward at a bounded linear speed.",
        (_p("speed_mps", "Forward speed in metres per second.", minimum=0.01, maximum=0.50, default=0.15),),
        voice_exposed=True,
        session_watchdog=True,
    ),
    ActionDescriptor(
        "v3.command.backward",
        "Drive backward at a bounded linear speed.",
        (_p("speed_mps", "Backward speed magnitude in metres per second.", minimum=0.01, maximum=0.50, default=0.15),),
        voice_exposed=True,
        session_watchdog=True,
    ),
    ActionDescriptor(
        "v3.command.teleop",
        "Command bounded linear and angular robot velocity.",
        (
            _p("v_mps", "Requested linear velocity in metres per second.", required=True, minimum=-0.50, maximum=0.50),
            _p("omega_rad_s", "Requested angular velocity in radians per second.", required=True, minimum=-1.20, maximum=1.20),
            _p("max_v_mps", "Absolute linear speed limit.", minimum=0.01, maximum=0.50, default=0.50),
            _p("max_omega_rad_s", "Absolute angular speed limit.", minimum=0.01, maximum=1.20, default=1.20),
        ),
        voice_exposed=True,
        session_watchdog=True,
    ),
    ActionDescriptor(
        "v3.command.wheels",
        "Command bounded left and right wheel target speeds.",
        (
            _p("left_mps", "Left wheel target speed in metres per second.", required=True, minimum=-0.50, maximum=0.50),
            _p("right_mps", "Right wheel target speed in metres per second.", required=True, minimum=-0.50, maximum=0.50),
        ),
        voice_exposed=True,
        session_watchdog=True,
    ),
    ActionDescriptor(
        "v3.command.explore",
        "Start autonomous room exploration using the canonical navigation stack.",
        voice_exposed=True,
        session_watchdog=True,
    ),
    ActionDescriptor(
        "v3.command.face_person",
        "Rotate in place toward the currently tracked person.",
        (_p("max_omega_rad_s", "Maximum turning speed.", minimum=0.01, maximum=1.20, default=0.50),),
        voice_exposed=True,
        session_watchdog=True,
        requirements=("person_target",),
    ),
    ActionDescriptor(
        "v3.command.follow_person",
        "Follow the currently tracked person using the canonical navigation stack.",
        (
            _p("max_v_mps", "Maximum following linear speed.", minimum=0.01, maximum=0.50, default=FOLLOW_PERSON_DEFAULT_MAX_V_MPS),
            _p("max_omega_rad_s", "Maximum following angular speed.", minimum=0.01, maximum=1.20, default=FOLLOW_PERSON_DEFAULT_MAX_OMEGA_RAD_S),
        ),
        voice_exposed=True,
        session_watchdog=True,
        requirements=("person_target",),
    ),
)

if len({item.name for item in _DESCRIPTORS}) != len(_DESCRIPTORS):
    raise RuntimeError("duplicate canonical action name")

ACTION_CATALOG: Mapping[str, ActionDescriptor] = MappingProxyType(
    {item.name: item for item in _DESCRIPTORS}
)


def action_descriptor(name: str) -> ActionDescriptor | None:
    return ACTION_CATALOG.get(name)


def action_catalog_jsonable() -> dict[str, object]:
    return {
        "schema": ACTION_CATALOG_SCHEMA,
        "actions": {name: descriptor.to_jsonable() for name, descriptor in ACTION_CATALOG.items()},
    }


def voice_action_descriptors() -> tuple[ActionDescriptor, ...]:
    return tuple(item for item in ACTION_CATALOG.values() if item.voice_exposed is True)


__all__ = [
    "ACTION_CATALOG",
    "ACTION_CATALOG_SCHEMA",
    "ACTION_DESCRIPTOR_SCHEMA",
    "FOLLOW_PERSON_DEFAULT_MAX_OMEGA_RAD_S",
    "FOLLOW_PERSON_DEFAULT_MAX_V_MPS",
    "ActionDescriptor",
    "ActionParameterDescriptor",
    "action_catalog_jsonable",
    "action_descriptor",
    "voice_action_descriptors",
]
