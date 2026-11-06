"""HRI adoption and passive feedback for Brain-owned user goals."""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import threading
import time
import json
import re

from .local_task_planner import LocalResolution, LocalTaskPlanner


def resolve_brain_request(interface: object, text: str, *, goal_id: str | None = None) -> LocalResolution:
    """Resolve a pending request locally; the result is still only a proposal."""
    # Explicit library invocation is deterministic and keeps both parameters
    # and the complete human request. Free language outside these known names
    # remains specialist interpretation, rather than guessed skill selection.
    match = re.fullmatch(r"(?:run|use|futtasd|indítsd|inditsd)(?:\s+(?:skill|a skillt))?\s+([A-Za-z_][A-Za-z0-9_]*)(?:\s+(\{.*\}))?", text.strip(), re.IGNORECASE)
    if match is None and re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", text.strip()) is None:
        return LocalTaskPlanner().resolve(text, interface, goal_id=goal_id)
    candidate = match[1] if match else text.strip()
    parameters = json.loads(match[2]) if match and match[2] else {}
    if not isinstance(parameters, dict):
        raise ValueError("skill invocation parameters must be an object")
    try:
        catalogue = interface.read("skill.list")
    except Exception:
        catalogue = ()
    items = catalogue.get("skills", ()) if isinstance(catalogue, Mapping) else catalogue
    if isinstance(items, (list, tuple)):
        matches = [item for item in items if isinstance(item, Mapping) and (
            str(item.get("name", "")).casefold() == candidate.casefold()
            or candidate.casefold() in tuple(str(alias).casefold() for alias in item.get("aliases", ())))]
        if len(matches) == 1:
            return LocalResolution(plan={"skill": matches[0]["name"], "parameters": parameters})
    return LocalTaskPlanner().resolve(text, interface, goal_id=goal_id)


def skill_cooperation_context(interface: object, text: str, goal_id: str | None) -> dict[str, object]:
    """Small goal/source/outcome slice for one planning request."""
    result: dict[str, object] = {"request_id": goal_id, "goal": text,
        "question": "Choose an existing skill or create/repair a Python skill for this complete goal.",
        "sdk": "async capabilities(), read(resource), query(query), spatial_query(query), call(name, **parameters), events.wait(...), result(handle), cancel(handle), stop(). Robot operations use the canonical shared interface."}
    try:
        state = interface.read("brain.state")
        if isinstance(state, Mapping):
            result["generation"] = state.get("generation")
        primary = state.get("primary_goal") if isinstance(state, Mapping) else None
        if isinstance(primary, Mapping):
            selected = primary.get("skill")
            result["previous_execution"] = {key: primary.get(key) for key in (
                "goal_id", "text", "lifecycle", "reason", "result", "skill", "skill_run_id", "skill_source_hash")}
            if isinstance(selected, str):
                source = interface.execute("skill.source", name=selected)
                if isinstance(source, Mapping) and isinstance(source.get("source"), str) and len(source["source"]) <= 12_000:
                    result["selected_source"] = dict(source)
    except Exception:
        pass
    try:
        catalogue = interface.read("skill.list")
        items = catalogue.get("skills", ()) if isinstance(catalogue, Mapping) else catalogue
        if isinstance(items, (list, tuple)):
            relevant = [item for item in items if isinstance(item, Mapping) and any(
                word.casefold() in text.casefold() for word in (str(item.get("name", "")),)
                if word)]
            result["library"] = [{key: item.get(key) for key in (
                "name", "description", "parameters", "result", "source_hash")}
                for item in (relevant or [item for item in items if isinstance(item, Mapping)])[:8]]
    except Exception:
        pass
    # Large program/results stay query-on-demand; never flood the prompt with
    # maps, images, capture or raw sidecar payloads.
    if len(json.dumps(result, ensure_ascii=False, default=str)) > 16_000:
        result.pop("selected_source", None)
    if len(json.dumps(result, ensure_ascii=False, default=str)) > 16_000:
        result.pop("previous_execution", None)
    return result


@dataclass(frozen=True, slots=True)
class BrainAdoption:
    status: str
    text: str
    goal_id: str | None = None
    command_id: str | None = None
    mission_id: str | None = None


def _person_feedback(snapshot: Mapping[str, object]) -> str | None:
    """Explain only the correlated teaching result, never a proposed name."""
    lifecycle, reason = snapshot.get("lifecycle"), str(snapshot.get("reason") or "")
    teaching = snapshot.get("current_subtask") == "person.teach" or "PERSON_TEACHING_BUSY" in reason
    if not teaching:
        return None
    result = snapshot.get("result")
    if (lifecycle == "COMPLETED" and isinstance(result, Mapping) and result.get("status") == "TAUGHT"
            and result.get("identity_taught") is True and isinstance(result.get("name"), str)):
        text = "A látható személyt " + result["name"] + " néven jegyeztem meg."
        if result.get("durability") == "MEMORY_ONLY":
            text += " A tartós mentés nem sikerült."
        return text
    if lifecycle == "FAILED":
        if "PERSON_SELECTION_REQUIRES_CLARIFICATION" in reason:
            return "Több személyt látok. Add meg, melyik látható személyt nevezzem el."
        if "PERSON_SELECTION_REQUIRES_FRESH_TRACK" in reason or "PERSON_SELECTED_TRACK_STALE_OR_UNAVAILABLE" in reason:
            return "Nem látok friss, igazolható személyt a név megtanításához."
        if "PERSON_TEACHING_BUSY" in reason:
            return "Egy másik személy nevének megtanítása még folyamatban van."
        if "PERSON_VISION" in reason or "PERSON_RUNTIME" in reason or "PERSON_FRAME" in reason:
            return "A név megtanításához most nem tudom igazolni a kamera és a személyészlelés friss állapotát."
    return None


def _skill_feedback(snapshot: Mapping[str, object]) -> str | None:
    if not snapshot.get("skill"):
        return None
    reason = str(snapshot.get("reason") or "")
    if reason == "SKILL_RETURNED":
        return "A Python-skill lefutott és visszatért. A teljes cél teljesülése még nincs igazolva."
    for prefix, message in (
        ("SKILL_REPORTED_FAILURE:", "A Python-skill hibát jelzett: "),
        ("SKILL_REPORTED_PARTIAL:", "A Python-skill részleges eredményt jelzett: "),
        ("SKILL_CLEANUP_FAILED:", "A Python-skill lezárása hibás: "),
    ):
        if reason.startswith(prefix):
            return message + reason[len(prefix):] + "."
    return None


def _goal_snapshot(interface: object, goal_id: str) -> Mapping[str, object] | None:
    """Read this goal even after another goal becomes primary."""
    state = interface.read("brain.state")
    if isinstance(state, Mapping):
        candidates = [state, *(value for value in state.values() if isinstance(value, Mapping))]
        snapshot = next((value for value in candidates if value.get("goal_id") == goal_id), None)
        if snapshot is not None:
            return snapshot
    history = interface.read("brain.history")
    if isinstance(history, (tuple, list)):
        for event in reversed(history):
            if isinstance(event, Mapping):
                value = event.get("goal", event.get("snapshot", event))
                if isinstance(value, Mapping) and value.get("goal_id") == goal_id:
                    return value
    return None


def wait_for_brain_goal(interface: object, adoption: BrainAdoption, *, timeout_s: float) -> BrainAdoption:
    """Bounded, read-only CLI feedback; goal execution remains asynchronous."""
    deadline = time.monotonic() + max(0.0, timeout_s)
    while time.monotonic() < deadline:
        try:
            snapshot = _goal_snapshot(interface, adoption.goal_id)
            if snapshot is not None:
                lifecycle = snapshot.get("lifecycle")
                reason = str(snapshot.get("reason") or lifecycle)
                if lifecycle in {"COMPLETED", "FAILED", "CANCELLED", "INTERRUPTED"}:
                    status = "COMPLETED" if lifecycle == "COMPLETED" else str(lifecycle) + ":" + reason
                    text = _person_feedback(snapshot) or _skill_feedback(snapshot) or ("A feladat befejeződött." if lifecycle == "COMPLETED"
                            else "A feladat megszakadt: " + reason + "." if lifecycle in {"CANCELLED", "INTERRUPTED"}
                            else "A feladat nem teljesült: " + reason + ".")
                    return BrainAdoption(status, text, adoption.goal_id,
                                         snapshot.get("command_id"), snapshot.get("mission_id"))
        except Exception:
            # Missing observation cannot become success or revoke execution.
            pass
        time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
    return BrainAdoption("ACTIVE:GOAL_RESULT_UNCONFIRMED",
                         "A feladat végeredményét a várakozási időn belül nem tudtam igazolni.",
                         adoption.goal_id, adoption.command_id, adoption.mission_id)


def adopt_brain_result(interface: object, result: Mapping[str, object], *,
                       execute: bool = True) -> BrainAdoption:
    """A proposal can become physical intent only after Brain admission."""
    action = result.get("proposed_action")
    plan = result.get("proposed_plan")
    goal_id = result.get("goal_id")
    if isinstance(action, Mapping) and action.get("name") == "v3.command.stop":
        if action.get("parameters", {}):
            raise ValueError("STOP accepts no parameters")
        if not execute:
            return BrainAdoption("SHADOW_ACCEPTED", "Értettem, de a végrehajtás teszt módban van.")
        interface.execute("v3.command.stop")
        return BrainAdoption("EXECUTED", "Megálltam.", goal_id=goal_id)
    if not isinstance(goal_id, str) or not goal_id:
        raise ValueError("robot proposal requires a Brain-owned pending goal")
    if plan is None and isinstance(action, Mapping):
        plan = {"steps": [{"action": action.get("name"), "parameters": action.get("parameters", {})}]}
    if not isinstance(plan, Mapping):
        raise ValueError("Brain adoption requires a plan proposal")
    if not execute:
        interface.execute("brain.fail", goal_id=goal_id, reason="SHADOW_MODE", pending_only=True)
        return BrainAdoption("SHADOW_ACCEPTED", "Értettem, de a végrehajtás teszt módban van.", goal_id)
    snapshot = interface.execute("brain.adopt", goal_id=goal_id, plan=dict(plan))
    if not isinstance(snapshot, Mapping):
        raise RuntimeError("Brain adoption returned no goal snapshot")
    lifecycle = str(snapshot.get("lifecycle", "UNKNOWN"))
    reason = str(snapshot.get("reason") or lifecycle)
    if lifecycle == "COMPLETED":
        status, text = "COMPLETED", _person_feedback(snapshot) or _skill_feedback(snapshot) or "A feladat befejeződött."
    elif lifecycle in {"PENDING", "STARTING", "ACTIVE"}:
        status, text = "ACTIVE", "A feladatot elfogadtam."
    else:
        status = "REJECTED:" + reason
        person_text = _person_feedback(snapshot)
        if person_text is not None:
            text = person_text
        elif "follow_distance_m" in reason:
            text = "A kért követési távolságot a jelenlegi képesség nem támogatja."
        elif "duration" in reason.casefold():
            text = "A kért időtartamot a jelenlegi képesség nem tudja teljesíteni."
        elif "CONSTRAINT" in reason:
            text = "A kérés egyik feltételét a jelenlegi képesség nem támogatja."
        elif "TARGET_BINDING" in reason:
            text = "Nem tudom igazolni, hogy ugyanazt a személyt követném."
        else:
            text = "A feladatot most nem tudom végrehajtani."
    return BrainAdoption(status, text, goal_id, snapshot.get("command_id"), snapshot.get("mission_id"))


class BrainGoalObserver:
    """Watch the primary goal and one teaching interjection without commands."""

    def __init__(self, interface: object, *, feedback_sink=None, event_sink=None):
        self._interface, self._feedback_sink, self._event_sink = interface, feedback_sink, event_sink
        self._closed = threading.Event()
        self._lock = threading.Lock()
        self._generation = 0
        self._watches: dict[str, int] = {}

    def observe(self, goal_id: str, **lineage: object) -> None:
        physical_id = None
        try:
            state = self._interface.read("brain.state")
            primary = state.get("primary_goal") if isinstance(state, Mapping) else None
            snapshot = _goal_snapshot(self._interface, goal_id)
            if (isinstance(primary, Mapping) and primary.get("goal_id") != goal_id
                    and primary.get("lifecycle") in {"STARTING", "ACTIVE"}
                    and isinstance(snapshot, Mapping) and snapshot.get("current_subtask") == "person.teach"):
                physical_id = primary.get("goal_id")
        except Exception:
            pass  # Unavailable status does not create a second physical owner.
        with self._lock:
            if goal_id in self._watches or self._closed.is_set():
                return
            self._generation += 1
            generation = self._generation
            # A teaching interjection preserves only the authoritative primary
            # watch. A replacement physical goal retains the previous routing.
            retained = {physical_id: self._watches[physical_id]} if physical_id in self._watches else {}
            self._watches = {**retained, goal_id: generation}
        threading.Thread(target=self._watch, args=(goal_id, generation, lineage),
                         name="r2b4-brain-goal-feedback", daemon=True).start()

    def close(self) -> None:
        self._closed.set()
        with self._lock:
            self._watches.clear()

    def _watch(self, goal_id: str, generation: int, lineage: Mapping[str, object]) -> None:
        # Goal identity can move from primary to history; an unrelated mission
        # completion never proves this goal complete.
        while True:
            if self._closed.wait(0.1):
                return
            with self._lock:
                if self._watches.get(goal_id) != generation:
                    return
            try:
                snapshot = _goal_snapshot(self._interface, goal_id)
                if snapshot is None:
                    continue
                lifecycle = snapshot.get("lifecycle")
                if lifecycle not in {"COMPLETED", "FAILED", "CANCELLED", "INTERRUPTED"}:
                    continue
                with self._lock:
                    if self._watches.get(goal_id) != generation:
                        return
                    del self._watches[goal_id]
                fields = {**lineage, "goal_id": goal_id, "lifecycle": lifecycle,
                          "reason": snapshot.get("reason")}
                text = _person_feedback(snapshot) or _skill_feedback(snapshot) or ("A feladat befejeződött." if lifecycle == "COMPLETED"
                        else "A feladat megszakadt." if lifecycle in {"CANCELLED", "INTERRUPTED"}
                        else "A feladat nem teljesült.")
                if self._event_sink is not None:
                    try:
                        self._event_sink("BRAIN_GOAL_RESULT", **fields)
                    except Exception:
                        pass
                if self._feedback_sink is not None:
                    try:
                        self._feedback_sink(text, fields)
                    except Exception:
                        pass
                return
            except Exception:
                # Public observation can disappear without creating success or
                # affecting Brain lifecycle or canonical physical execution.
                continue
__all__ = ["BrainAdoption", "BrainGoalObserver", "adopt_brain_result", "resolve_brain_request", "wait_for_brain_goal"]
