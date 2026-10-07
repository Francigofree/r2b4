"""Small deterministic task methods above RobotInterface.

The planner owns neither goals nor spatial memory. It proposes canonical actions,
using public facts only to resolve names; Brain qualifies execution evidence.
Unknown clauses remain specialist work and are never silently discarded.
"""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import re
import unicodedata

from v3.action_catalog import ACTION_CATALOG
from .execution_mode import is_stop_intent
from .world_model import WorldQuery


def _fold(text: str) -> str:
    return " ".join("".join(c for c in unicodedata.normalize("NFKD", text.casefold())
                             if not unicodedata.combining(c)).split()).strip(" .!?;")


_NUMBERS = {"fel": .5, "half": .5, "egy": 1, "one": 1, "ket": 2, "ketto": 2,
            "two": 2, "harom": 3, "three": 3, "negy": 4, "four": 4,
            "ot": 5, "five": 5, "tiz": 10, "ten": 10, "hatvan": 60,
            "kilencven": 90, "ninety": 90}
_NUMBER = r"(?:\d+(?:[.,]\d+)?|" + "|".join(_NUMBERS) + r")"
_DISTANCE = r"(?:metert?|meter(?:s)?|metres?|m|centimetert?|centimeters?|cm)"
_MOVE_VERB = r"(?:menj|haladj|indulj|move|go|drive)"
_DIRECTION = r"(?:elore|hatra|forward|forwards|backward|backwards)"
_METRIC_SUFFIX = r"(?:-?(?:t|rol|re|tol|nyi))?"
_DURATION = re.compile(r"\s+(?:for\s+)?(" + _NUMBER + r")\s*(masodperc(?:ig)?|perc(?:ig)?|seconds?|minutes?|s)\s*$")
_CLAUSES = re.compile(r"(?<!\d),(?!\d)|;|(?<!\d)\.(?!\d)|\b(?:azt[aá]n|majd|then|and then)\b|\s+(?:[eé]s|and)\s+(?=(?:menj|haladj|fordulj|n[eé]zz|keress|keresd|k[oö]vesd|k[oö]vess|move|go|turn|look|search|find|follow)\b)", re.IGNORECASE)


def _number(raw: str) -> float:
    return _NUMBERS[raw] if raw in _NUMBERS else float(raw.replace(",", "."))


def _motion_instruction(text: str) -> tuple[str, float] | None:
    """Concrete motion grammar shared by local parsing and request admission."""
    metric = r"(" + _NUMBER + r")\s*(" + _DISTANCE + r")" + _METRIC_SUFFIX
    move = re.fullmatch(r"(?:" + _MOVE_VERB + r"\s+)?(?:(?:meg|tovabb|another)\s+)?(" + _DIRECTION + r")?\s*" + metric + r"(?:\s+(" + _DIRECTION + r"))?", text)
    if move and (move[1] or move[4] or re.match(_MOVE_VERB + r"\b", text)):
        direction = move[1] or move[4]
        if move[1] and move[4] and move[1] != move[4]:
            return None
        distance = _number(move[2]) / (100 if move[3].startswith(("centi", "cm")) else 1)
        return "move", -distance if direction in {"hatra", "backward", "backwards"} else distance
    direction = re.fullmatch(r"(?:" + _MOVE_VERB + r"\s+)?(" + _DIRECTION + r")", text)
    if direction:
        return "direction", -1.0 if direction[1] in {"hatra", "backward", "backwards"} else 1.0
    turn = re.fullmatch(r"(?:fordulj|turn)(?:\s+(?:to\s+the|the))?\s+(balra|jobbra|left|right)(?:\s+(" + _NUMBER + r")\s*(?:fok(?:ot)?|degrees?|deg)?)?", text)
    reverse_turn = re.fullmatch(r"(?:fordulj|turn)\s+(" + _NUMBER + r")\s*(?:fok(?:ot)?|degrees?|deg)?\s+(balra|jobbra|left|right)", text)
    if turn or reverse_turn:
        direction, number = (turn[1], turn[2]) if turn else (reverse_turn[2], reverse_turn[1])
        angle = _number(number) if number else 90.0
        return "turn", -angle if direction in {"jobbra", "right"} else angle
    return None


def explicit_metric_constraints(text: str) -> dict[str, object]:
    """Preserve user metrics independently of any proposed plan or provider.

    This pure extraction also covers familiar spelling such as ``1 m-t`` that
    need not be part of an executable local method. Distances attach to their
    preceding move/follow verb so a translation before following cannot become
    an invented following distance.
    """
    result: dict[str, object] = {}
    folded = _fold(text)
    durations = re.findall(r"\b(" + _NUMBER + r")\s*(masodperc(?:ig)?|perc(?:ig)?|seconds?|minutes?|s)\b", folded)
    if durations:
        values = tuple(_number(number) * (60 if unit.startswith(("perc", "minute")) else 1)
                       for number, unit in durations)
        result["duration_s" if len(values) == 1 else "durations_s"] = values[0] if len(values) == 1 else values
    moves, follows, motion = [], [], []
    previous_owner = None
    for clause in _CLAUSES.split(folded):
        instruction = _motion_instruction(clause.strip(" ,"))
        if instruction is None and previous_owner in {"menj", "haladj", "indulj", "move", "go", "drive"} and re.match(r"\s*(?:meg|tovabb|another)\b", clause):
            instruction = _motion_instruction("menj " + clause.strip())
        if instruction is not None:
            motion.append(instruction)
        elif clause.strip(" ,") in {"nezz korul", "nezz korbe", "look around", "look round"}:
            motion.extend([("turn", 90.0)] * 4)
        else:
            # Extra semantic words cannot erase a clearly stated motion. This
            # scan extracts only the same small grammar, never provider text.
            metric = _NUMBER + r"\s*" + _DISTANCE + _METRIC_SUFFIX
            pattern = (r"\b" + _MOVE_VERB + r"\s+(?:(?:meg|tovabb|another)\s+)?(?:" + _DIRECTION + r"\s+)?" + metric + r"(?:\s+" + _DIRECTION + r")?"
                       r"|\b(?:fordulj|turn)\s+(?:balra|jobbra|left|right)(?:\s+" + _NUMBER + r"\s*(?:fok(?:ot)?|degrees?|deg)?)?")
            for match in re.finditer(pattern, clause):
                item = _motion_instruction(match[0])
                if item is not None:
                    motion.append(item)
        verbs = list(re.finditer(r"\b(menj|haladj|indulj|move|go|drive|koves\w*|follow)\b", clause))
        for metric in re.finditer(r"\b(" + _NUMBER + r")\s*(" + _DISTANCE + r")(?:-?(?:t|rol|re|tol|nyi))?\b", clause):
            owner = next((verb[1] for verb in reversed(verbs) if verb.start() < metric.start()), None)
            if owner is None and not verbs and re.match(r"\s*(?:meg|tovabb|another|" + _NUMBER + r")\b", clause):
                owner = previous_owner
            if owner is None:
                if instruction is not None and instruction[0] == "move":
                    owner = "move"
                else:
                    continue
            distance = _number(metric[1]) / (100 if metric[2].startswith(("centi", "cm")) else 1)
            (follows if owner.startswith(("koves", "follow")) else moves).append(distance)
        if verbs:
            previous_owner = verbs[-1][1]
    if moves:
        result["distance_m" if len(moves) == 1 else "distances_m"] = moves[0] if len(moves) == 1 else tuple(moves)
    if follows:
        result["follow_distance_m" if len(follows) == 1 else "follow_distances_m"] = follows[0] if len(follows) == 1 else tuple(follows)
    if motion:
        result["motion_sequence"] = tuple(motion)
    return result


def _step(action: str, **parameters: object) -> dict[str, object]:
    if action != "vision.observe" and action not in ACTION_CATALOG:
        raise ValueError("local method requires a canonical capability")
    return {"action": action, "parameters": parameters}


@dataclass(frozen=True, slots=True)
class LocalResolution:
    plan: Mapping[str, object] | None = None
    spoken_text: str | None = None
    unresolved_text: str | None = None
    prefix: tuple[Mapping[str, object], ...] = ()
    suffix: tuple[Mapping[str, object], ...] = ()
    unfulfilled: bool = False

    @property
    def resolved(self) -> bool:
        return self.plan is not None or self.spoken_text is not None

    def merge_specialist_plan(self, plan: Mapping[str, object]) -> dict[str, object]:
        """Keep locally resolved actions fixed around one unresolved clause."""
        if not self.prefix and not self.suffix:
            return dict(plan)
        rows = plan.get("steps")
        if not isinstance(rows, (list, tuple)) or not rows:
            raise ValueError("SCOPED_SPECIALIST_REQUIRES_LINEAR_PLAN")
        steps = [*self.prefix, *rows, *self.suffix]
        if len(steps) > 16:
            raise ValueError("LOCAL_PLAN_EXCEEDED_BOUND")
        return {"steps": steps, "constraints": dict(plan.get("constraints", {}))}


class LocalTaskPlanner:
    """Stateless bounded parsing and hierarchical composition, no execution."""

    __slots__ = ()

    def resolve(self, text: str, interface: object, *, goal_id: str | None = None) -> LocalResolution:
        if not isinstance(text, str) or not text.strip() or len(text) > 4000:
            raise ValueError("local task text must contain 1..4000 characters")
        if is_stop_intent(text):
            return LocalResolution(plan={"steps": [_step("v3.command.stop")]})
        folded = _fold(text)
        answer = self._answer(folded, interface, goal_id)
        if answer is not None:
            return LocalResolution(spoken_text=answer, unfulfilled=folded in {"menj oda", "go there", "navigate there"})
        house_search = self._house_search(folded, interface)
        if house_search is not None:
            return house_search
        long_task = self._search_follow_return(folded, interface)
        if long_task is not None:
            return long_task
        recovery = self._navigation_recovery(folded, interface)
        if recovery is not None:
            return recovery
        if folded.startswith(("keresd meg es kovesd", "find and follow someone")):
            rows = self._method(folded, interface)
            if rows:
                return LocalResolution(plan={"steps": rows})
        clauses = [clause.strip(" ,") for clause in _CLAUSES.split(text.strip(" .!?;")) if clause.strip(" ,")]
        if not 1 <= len(clauses) <= 16:
            return LocalResolution(unresolved_text=text)
        segments: list[list[dict[str, object]] | None] = []
        unresolved: list[int] = []
        for index, clause in enumerate(clauses):
            normalized = _fold(clause)
            instruction = _motion_instruction(normalized)
            if instruction is not None and instruction[0] == "turn" and not 0 < abs(instruction[1]) <= 360:
                return LocalResolution(spoken_text="Egy kérésben 0 és 360 fok közötti fordulást tudok végrehajtani.", unfulfilled=True)
            if re.fullmatch(r"(?:(?:fordulj|turn)\s+)?" + _NUMBER + r"\s*(?:fok(?:ot)?|degrees?|deg)", normalized):
                return LocalResolution(spoken_text="Melyik irányba forduljak: balra vagy jobbra?", unfulfilled=True)
            rows = self._method(normalized, interface)
            segments.append(rows)
            if rows is None:
                navigation = re.fullmatch(r"(?:menj|navigalj|go to|navigate to)\s+(?:(?:a|az|the)\s+)?(.+)", normalized)
                if navigation and not re.search(r"\b" + _NUMBER + r"\b|elore|hatra|forward|backward", navigation[1]):
                    # An unknown or ambiguous semantic destination needs a
                    # referent, not coordinates invented by a specialist.
                    return LocalResolution(spoken_text="Nem ismerek egyértelmű célpontot ezzel a névvel. Pontosítsd az ismert helyet vagy tárgyat.", unfulfilled=True)
                unresolved.append(index)
        # Search -> follow always binds the exact person and runtime identity.
        for index in range(1, len(segments)):
            before, after = segments[index - 1], segments[index]
            if (before and after and before[-1]["action"] in {"behavior.search_any_person", "behavior.search_person"}
                    and after[0]["action"] == "behavior.follow_person"):
                before[-1]["bind_target"] = True
                after[0]["use_bound_target"] = True
        if not unresolved:
            rows = [row for segment in segments for row in segment]
            if len(rows) <= 16:
                return LocalResolution(plan={"steps": rows})
            if len(rows) <= 32:
                nodes = []
                for index, row in enumerate(rows):
                    duration = row["parameters"].get("max_duration_s", 0)
                    node = {**row, "node_id": f"step-{index}", "kind": "action",
                            "timeout_s": min(3600, max(120, duration + 5))}
                    if index + 1 < len(rows):
                        node["on_success"] = f"step-{index + 1}"
                    nodes.append(node)
                return LocalResolution(plan={"entry": "step-0", "nodes": nodes})
            return LocalResolution(spoken_text="A feladat túl sok lépésből áll; bontsd több rövidebb kérésre.", unfulfilled=True)
        if len(unresolved) == 1:
            index = unresolved[0]
            return LocalResolution(unresolved_text=clauses[index],
                prefix=tuple(row for segment in segments[:index] for row in segment),
                suffix=tuple(row for segment in segments[index + 1:] for row in segment))
        return LocalResolution(unresolved_text=text)

    def _search_follow_return(self, text: str, interface: object) -> LocalResolution | None:
        marker = re.search(r"\b(?:ha talalsz valakit|if you find (?:someone|a person))\b[, ]*", text)
        if marker is None or not text.endswith(("gyere vissza", "terj vissza", "come back", "return to origin")):
            return None
        normalized = text[:marker.start()] + text[marker.end():]
        clauses = [part.strip(" ,") for part in _CLAUSES.split(normalized) if part.strip(" ,")]
        rows = []
        for clause in clauses:
            segment = self._method(_fold(clause), interface)
            if segment is None:
                return None
            rows.extend(segment)
        searches = [index for index, row in enumerate(rows) if row["action"] in {"behavior.search_any_person", "behavior.search_person"}]
        follows = [index for index, row in enumerate(rows) if row["action"] == "behavior.follow_person"]
        if (len(searches) != 1 or len(follows) != 1 or searches[0] >= follows[0]
                or not rows[-1].get("return_to_origin") or len(rows) > 30):
            return None
        rows[searches[0]]["bind_target"] = True
        rows[follows[0]]["use_bound_target"] = True
        nodes = []
        for index, row in enumerate(rows):
            duration = row["parameters"].get("max_duration_s", 0)
            node = {**row, "node_id": f"step-{index}", "kind": "action",
                    "timeout_s": min(3600, max(120, duration + 5))}
            if index + 1 < len(rows):
                node["on_success"] = f"step-{index + 1}"
            if index == searches[0]:
                node.update(on_failure="not-found", failure_on=["TARGET_LOST"])
            nodes.append(node)
        nodes.append({"node_id": "not-found", "kind": "report", "failure_code": "TARGET_LOST",
                      "message": "Nem találtam frissen azonosított személyt, ezért a követés nem indult el."})
        return LocalResolution(plan={"entry": "step-0", "nodes": nodes})

    def _house_search(self, text: str, interface: object) -> LocalResolution | None:
        match = re.fullmatch(r"(?:keress (?:valakit|egy embert) a (?:lakasban|hazban)|(?:find|search for) (?:someone|a person) in the (?:house|apartment))(?:\s+(?:es|and)\s+(kovesd(?: ot)?|follow(?: them)?)(.*))?", text)
        if match is None:
            return None
        follow_rows = self._method(match[1] + match[2], interface) if match[1] else None
        if match[1] and not follow_rows:
            return None
        rooms = sorted(entity for entity, facts in self._entities(interface, "place_or_object").items()
                       if any(fact.get("domain") == "room_topology" for fact in facts))
        if not rooms:
            return LocalResolution(spoken_text="A kereséshez még nincsenek megismert szobák vagy helyek.", unfulfilled=True)
        if len(rooms) > 6:
            # No silent omission of known search places when the graph is full.
            return LocalResolution(spoken_text="Ennyi ismert helyet egy feladatban nem tudok végigkeresni; szűkítsd a keresési helyeket.", unfulfilled=True)
        nodes = []
        for index, room in enumerate(rooms):
            next_place = f"place-{index + 1}" if index + 1 < len(rooms) else "exhausted"
            nodes.append({**_step("v3.command.navigate"), "node_id": f"place-{index}",
                "kind": "action", "target_entity_id": room, "on_success": f"search-{index}",
                "on_failure": next_place, "failure_on": ["NO_PATH"]})
            search = {**_step("behavior.search_any_person", max_duration_s=60),
                "node_id": f"search-{index}", "kind": "action", "on_failure": next_place,
                "failure_on": ["TARGET_LOST"], "bind_target": bool(follow_rows)}
            if follow_rows:
                search["on_success"] = "follow"
            nodes.append(search)
        if follow_rows:
            duration = follow_rows[0]["parameters"].get("max_duration_s", 120)
            nodes.append({**follow_rows[0], "node_id": "follow", "kind": "action",
                          "timeout_s": min(3600, max(120, duration + 5)), "use_bound_target": True})
        nodes.append({"node_id": "exhausted", "kind": "report", "failure_code": "TARGET_LOST",
                      "message": "A megismert keresési helyeken nem lett friss személytalálat."})
        return LocalResolution(plan={"entry": "place-0", "nodes": nodes})

    def _method(self, text: str, interface: object) -> list[dict[str, object]] | None:
        duration = None
        match = _DURATION.search(text)
        if match:
            duration = _number(match[1]) * (60 if match[2].startswith(("perc", "minute")) else 1)
            text = text[:match.start()].strip()
            if not .1 <= duration <= 3600:
                return None
        if text in {"nezz korul", "nezz korbe", "look around", "look round"} and duration is None:
            rows = [_step("vision.observe")]
            for _ in range(4):
                rows += [_step("v3.command.turn_by", angle_deg=90), _step("vision.observe")]
            return rows
        if text in {"keszits kepet", "keszits egy kepet", "csinalj egy kepet", "csinalj kepet", "nezd meg", "take a picture", "observe", "look"} and duration is None:
            return [_step("vision.observe")]
        if text in {"gyere vissza", "terj vissza", "come back", "return", "return to origin"} and duration is None:
            return [{**_step("v3.command.navigate"), "return_to_origin": True}]
        if text in {"menj korbe", "jarj korbe", "jarjad be a szobat", "jarj be a szobat", "szoba felfedezes", "barangolj", "room cruise", "cruise the room", "explore the room"}:
            return [_step("behavior.room_cruise", **({"max_duration_s": duration} if duration else {}))]
        motion = _motion_instruction(text)
        if motion is not None and duration is None:
            kind, value = motion
            if kind == "move" and 0 < abs(value) <= 100:
                return [_step("v3.command.move_relative", forward_m=value)]
            if kind == "direction":
                return [_step("v3.command.backward" if value < 0 else "v3.command.forward")]
            if kind == "turn" and 0 < abs(value) <= 360:
                # Each physical primitive obeys the existing V3 <=180 bound.
                first = min(abs(value), 180) * (1 if value > 0 else -1)
                return [_step("v3.command.turn_by", angle_deg=first)] + ([_step("v3.command.turn_by", angle_deg=value - first)] if value != first else [])
            return None
        if text in {"keress valakit", "keress egy embert", "keress embert", "keress egy szemelyt", "keresd meg es kovesd", "find someone", "find a person", "search for someone", "search for a person", "find and follow someone"}:
            # The existing anonymous acquisition behavior performs a bounded scan.
            follow = text in {"keresd meg es kovesd", "find and follow someone"}
            search = _step("behavior.search_any_person", **({"max_duration_s": duration} if duration and not follow else {}))
            if follow:
                search["bind_target"] = True
                return [search, {**_step("behavior.follow_person", **({"max_duration_s": duration} if duration else {})), "use_bound_target": True}]
            return [search]
        if text in {"kovess", "kovesd", "kovesd ot", "kovesd az embert", "follow", "follow them", "follow him", "follow her", "follow the person"}:
            return [_step("behavior.follow_person", **({"max_duration_s": duration} if duration else {}))]
        search = re.fullmatch(r"(?:keresd meg|keress|find|search for)\s+(.+)", text)
        if search:
            person = self._resolve_entity(interface, search[1], kind="person")
            if person:
                return [_step("behavior.search_person", entity_id=person,
                              **({"max_duration_s": duration} if duration else {}))]
        named_follow = re.fullmatch(r"(?:kovesd|follow)\s+(.+)", text)
        if named_follow:
            person = self._resolve_entity(interface, named_follow[1], kind="person")
            if person:
                return [{**_step("behavior.search_person", entity_id=person), "bind_target": True},
                        {**_step("behavior.follow_person", **({"max_duration_s": duration} if duration else {})), "use_bound_target": True}]
        navigate = re.fullmatch(r"(?:menj|navigalj|go to|navigate to)\s+(?:(?:a|az|the)\s+)?(.+)", text)
        if navigate and duration is None:
            entity = self._resolve_entity(interface, navigate[1], kind="place_or_object")
            if entity:
                return [{**_step("v3.command.navigate"), "target_entity_id": entity}]
        return None

    def _navigation_recovery(self, text: str, interface: object) -> LocalResolution | None:
        # One concrete hierarchical recovery method; V3 retains local replanning.
        parts = re.split(r"[.,;]?\s+(?:ha|if)\s+", text, maxsplit=1)
        if len(parts) != 2 or not re.search(r"(?:masik ut|ujra|retry|another (?:way|route))", parts[1]):
            return None
        # Preserve all conditions: this method accepts only an unreachable/path
        # condition with one retry and an optional report request.
        if not re.fullmatch(r"(?:nem lehet odajutni|nem tudsz odajutni|nem sikerul|you cannot get there|unreachable|blocked)[ ,]*(?:probald ujra|probalj masik utat|try another (?:way|route)|retry)(?:[ .;,]*(?:ha az sem sikerul,? szolj|if that fails,? (?:tell me|report)))?", parts[1]):
            return None
        rows = self._method(parts[0].strip(" .,"), interface)
        if not rows or len(rows) != 1 or rows[0]["action"] != "v3.command.navigate":
            return None
        target = rows[0]["target_entity_id"]
        return LocalResolution(plan={"entry": "navigate", "nodes": [
            {**rows[0], "node_id": "navigate", "kind": "action", "on_failure": "refresh", "failure_on": ["NO_PATH"]},
            {"node_id": "refresh", "kind": "world_wait", "timeout_s": 5,
             "condition": {"entity_id": target, "attribute": "location", "predicate": "exists", "require_current": True,
                           "newer_than_node_entry": True},
             "on_success": "retry", "on_failure": "report", "failure_on": ["TIMEOUT"]},
            {**rows[0], "node_id": "retry", "kind": "action", "on_failure": "report", "failure_on": ["NO_PATH"]},
            {"node_id": "report", "kind": "report", "message": "A célhoz az újrapróbálás után sem találtam járható utat.", "failure_code": "NO_PATH"},
        ]})

    @staticmethod
    def _facts(interface: object, *, domain: str | None = None, entity_id: str | None = None) -> tuple[Mapping[str, object], ...]:
        query = getattr(interface, "query", None)
        if not callable(query):
            return ()
        result = query(WorldQuery(domain=domain, entity_id=entity_id, limit=64))
        facts = getattr(result, "facts", ())
        return tuple(fact.to_jsonable() for fact in facts)

    def _entities(self, interface: object, kind: str | None) -> dict[str, list[Mapping[str, object]]]:
        entities: dict[str, list[Mapping[str, object]]] = {}
        spatial_query = getattr(interface, "spatial_query", None)
        if callable(spatial_query):
            from .spatial_service import SpatialQuery
            result = spatial_query(SpatialQuery(kind="entities", limit=64))
            for entity in result.entities:
                if kind == "person" and entity.kind != "person":
                    continue
                if kind == "place_or_object" and entity.kind not in {"place", "object", "spatial_entity"}:
                    continue
                entities[entity.entity_id] = [fact.to_jsonable() for fact in entity.facts]
            return entities
        domains = ("person_identity", "person_position") if kind == "person" else (
            ("room_topology", "object_identity", "object_position") if kind == "place_or_object" else
            ("room_topology", "object_identity", "object_position", "person_identity", "person_position"))
        for domain in domains:
            for fact in self._facts(interface, domain=domain):
                entities.setdefault(fact["entity_id"], []).append(fact)
        return entities

    @staticmethod
    def _names(entity_id: str, facts: list[Mapping[str, object]]) -> set[str]:
        names = {_fold(entity_id), _fold(entity_id.rsplit(":", 1)[-1])}
        for fact in facts:
            value = fact.get("value")
            if isinstance(value, str) and fact.get("attribute") in {"name", "label", "identity"}:
                names.add(_fold(value))
            elif isinstance(value, Mapping):
                for key in ("name", "label", "display_name"):
                    if isinstance(value.get(key), str):
                        names.add(_fold(value[key]))
                aliases = value.get("aliases", ())
                if isinstance(aliases, (list, tuple)):
                    names.update(_fold(alias) for alias in aliases if isinstance(alias, str))
        return names

    def _resolve_entity(self, interface: object, reference: str, *, kind: str | None = None) -> str | None:
        matches = []
        for entity_id, facts in self._entities(interface, kind).items():
            names = self._names(entity_id, facts)
            # Hungarian object/place inflection is accepted only against an
            # actually published name; no arbitrary substring/entity invention.
            forms = {name + ending for name in names for ending in ("", "t", "et", "ot", "hoz", "hez", "ba", "be")}
            if reference in forms:
                matches.append(entity_id)
        return matches[0] if len(matches) == 1 else None

    def _answer(self, text: str, interface: object, goal_id: str | None) -> str | None:
        if text in {"menj oda", "go there", "navigate there"}:
            return "Melyik ismert helyre vagy tárgyhoz menjek?"
        if text in {"mi a feladat allapota", "feladat statusz", "task status", "what is the task status", "mit csinalsz", "what are you doing"}:
            state = interface.read("brain.state")
            active = state.get("primary_goal") if isinstance(state, Mapping) else None
            if isinstance(active, Mapping) and active.get("goal_id") != goal_id:
                running = active.get("lifecycle") in {"PENDING", "STARTING", "ACTIVE"}
                label = "A jelenlegi feladat" if running else "Az utolsó feladat"
                subtask = f" Részfeladat: {active.get('current_subtask')}." if running and active.get("current_subtask") else ""
                return f"{label}: {str(active.get('text', active.get('goal_id')))[:256]}. Állapot: {active.get('lifecycle')}; ok: {str(active.get('reason') or 'nincs')[:1024]}.{subtask}"
            return "Jelenleg nincs aktív feladat."
        if text.startswith(("miert ", "why ")) and re.search(r"feladat|task|elozo|previous|nem talalt|didn.t find|stopp|allt", text):
            history = interface.read("brain.history")
            reference = re.search(r"(?:nem talaltad meg|didn.t (?:you )?find)\s+(.+)", text)
            if isinstance(history, (tuple, list)):
                for event in reversed(history):
                    if not isinstance(event, Mapping):
                        continue
                    outcome = event.get("goal", event.get("snapshot", event))
                    if (isinstance(outcome, Mapping) and outcome.get("goal_id") != goal_id
                            and outcome.get("lifecycle") in {"FAILED", "CANCELLED", "INTERRUPTED", "COMPLETED"}):
                        if reference and reference[1] not in _fold(str(outcome.get("text", ""))):
                            continue
                        reason = str(outcome.get("reason") or "nem ismert")
                        from .task_graph import classify_failure
                        failure_code = classify_failure(reason).value
                        explanations = {
                            "SEARCH_PLACES_EXHAUSTED": "A megismert keresési helyeket kimerítettem, de nem érkezett friss azonosítás a kért személyről.",
                            "SEARCH_VIEWS_EXHAUSTED": "A keresési nézeteket végigellenőriztem, de nem volt friss személytalálat.",
                            "NO_PATH": "Nem találtam járható útvonalat a célhoz.",
                            "TARGET_LOST": "A korábban kiválasztott személyhez nem volt használható friss megfigyelés.",
                        }
                        explanation = (next((value for code, value in explanations.items() if code in reason), "")
                                       if failure_code in {"NO_PATH", "TARGET_LOST"} else "")
                        summary = self._history_summary(history, outcome.get("goal_id"))
                        return f"A korábbi feladat: {str(outcome.get('text', outcome.get('goal_id')))[:256]}. Eredmény: {outcome.get('lifecycle')}; igazolt ok: {reason[:1024]}. {explanation} {summary}".strip()
            return "Nincs elérhető korábbi feladateredmény, amelyből az okot igazolhatnám."
        if text in {"milyen helyeket ismersz", "milyen szobakat ismersz", "list known places", "what places do you know"}:
            entities = self._entities(interface, "place_or_object")
            places = sorted(entity for entity, facts in entities.items()
                            if any(fact.get("domain") == "room_topology" for fact in facts))
            return "Ismert helyek: " + ", ".join(places) + "." if places else "Nincsenek megismert helyek."
        where = re.fullmatch(r"(?:hol van|where is)\s+(?:(?:a|az|the)\s+)?(.+)", text)
        if where:
            entity = self._resolve_entity(interface, where[1])
            if entity is None:
                return None
            locations = [fact for fact in self._facts(interface, entity_id=entity)
                         if fact.get("attribute") == "location" and isinstance(fact.get("value"), Mapping)]
            if not locations:
                return f"{entity}: nincs ismert helyadat."
            fact = max(locations, key=lambda item: item.get("measurement_time_ns", 0))
            value = fact["value"]
            place = value.get("place_id")
            description = str(place) if place else f"x={value.get('x_m')} m, y={value.get('y_m')} m; frame={value.get('frame_id')}"
            qualifier = "Friss helyadat" if fact.get("freshness") == "FRESH" and fact.get("state") in {"KNOWN", "LIKELY"} else "Utolsó ismert helyadat; jelenlegi helyzete nem igazolt"
            return f"{entity}: {description}. {qualifier}."
        return None

    @staticmethod
    def _history_summary(history, goal_id) -> str:
        """Only completed results carrying this goal/subtask identity count."""
        rows = {}
        for event in history:
            if not isinstance(event, Mapping) or event.get("kind") not in {"ACTION_RESULT", "BEHAVIOR_RESULT", "OBSERVATION_RESULT"}:
                continue
            outcome = event.get("goal", event.get("snapshot", event))
            if not isinstance(outcome, Mapping) or outcome.get("goal_id") != goal_id:
                continue
            result = outcome.get("result")
            subtask = outcome.get("subtask_id")
            if not isinstance(result, Mapping) or not isinstance(subtask, str):
                continue
            label = str(outcome.get("current_subtask") or subtask)[:96]
            target = outcome.get("world_target")
            if isinstance(target, Mapping) and target.get("entity_id"):
                label += " → " + str(target["entity_id"])[:96]
            status = result.get("status") or result.get("reason") or outcome.get("reason") or "rögzített eredmény"
            rows[subtask] = f"{label}: {str(status)[:128]}"
        if not rows:
            return "Nincs elérhető korrelált részfeladat-eredmény."
        details = list(rows.values())[-6:]
        prefix = f"Utolsó 6 részfeladat ({len(rows)} rögzített): " if len(rows) > 6 else "Rögzített részfeladatok: "
        return prefix + "; ".join(details) + "."


__all__ = ["LocalResolution", "LocalTaskPlanner", "explicit_metric_constraints"]
