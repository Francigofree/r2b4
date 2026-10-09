"""The installed person methods of the host executive.

These descriptors connect existing behavior factories, public capabilities and
local proposals. They own no execution state or V3 primitive definitions.
"""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from v3.action_catalog import action_descriptor
from .world_model import _text


@dataclass(frozen=True, slots=True)
class HostPersonSkill:
    action: str
    method_id: str
    version: str
    behavior_name: str | None = None
    requires_motion: bool = True
    target_binding: bool = False

    def validate_parameters(self, parameters: Mapping[str, object]) -> None:
        if self.action != "person.teach":
            raise ValueError("person skill parameters use the canonical action catalog")
        if not isinstance(parameters, Mapping) or set(parameters) - {
                "name", "entity_id", "target_track_id", "aliases", "request_id", "source"}:
            raise ValueError("unknown person teaching parameters")
        _text(parameters.get("name"), "name", 96)
        for key in ("entity_id", "target_track_id", "request_id"):
            if key in parameters:
                _text(parameters[key], key)
        if parameters.get("source", "HUMAN") != "HUMAN":
            raise ValueError("person teaching requires HUMAN source")
        aliases = parameters.get("aliases", ())
        if not isinstance(aliases, (list, tuple)) or len(aliases) > 8:
            raise ValueError("person aliases must be a bounded list")
        for alias in aliases:
            _text(alias, "alias", 96)

    def to_jsonable(self) -> dict[str, object]:
        descriptor = action_descriptor(self.action)
        result = descriptor.to_jsonable() if descriptor is not None else {
            "name": self.action,
            "description": "Teach a name for an explicitly selected fresh observed person track.",
            "parameters": {
                "name": {"type": "string", "required": True, "maxLength": 96},
                "entity_id": {"type": "string", "required": False},
                "target_track_id": {"type": "string", "required": False},
                "aliases": {"type": "array", "required": False, "maxItems": 8},
                "request_id": {"type": "string", "required": True},
            },
        }
        result.update(method_id=self.method_id, method_version=self.version,
                      requires_motion=self.requires_motion, target_binding=self.target_binding,
                      max_steps=32, max_retries=0)
        return result


PERSON_SKILLS = (
    HostPersonSkill("behavior.search_any_person", "person.acquire", "1", "search_any_person"),
    HostPersonSkill("behavior.search_person", "person.named_search", "1", "search_person"),
    HostPersonSkill("behavior.follow_person", "person.bound_follow", "1", "follow_person", target_binding=True),
    HostPersonSkill("person.teach", "person.teach", "1", requires_motion=False, target_binding=True),
)


def skill_descriptor(action: str) -> HostPersonSkill | None:
    return next((skill for skill in PERSON_SKILLS if skill.action == action), None)


def validate_skill_parameters(action: str, parameters: Mapping[str, object]) -> bool:
    skill = skill_descriptor(action)
    if skill is None or skill.requires_motion:
        return False
    skill.validate_parameters(parameters)
    return True


def person_behavior_factories():
    from .search_person import SearchAnyPerson, SearchPerson
    factories = {"search_any_person": SearchAnyPerson, "search_person": SearchPerson}
    return tuple((skill.behavior_name, factories[skill.behavior_name])
                 for skill in PERSON_SKILLS if skill.behavior_name in factories)
