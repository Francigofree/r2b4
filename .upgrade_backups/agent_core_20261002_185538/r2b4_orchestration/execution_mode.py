"""System-level execution mode selection for natural-language R2B4 requests.

This module owns routing semantics only. It owns no motor, runtime, camera, LLM,
or safety authority. Executors must still use the canonical RobotInterface / ER2
boundaries.
"""
from __future__ import annotations

import json
import os
import re
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Mapping


class ExecutionMode(str, Enum):
    GEMINI_CHAT = "GEMINI_CHAT"
    HOST_READ = "HOST_READ"
    OBSERVATION = "OBSERVATION"
    DIRECT_V3 = "DIRECT_V3"
    ER2_PREVIEW = "ER2_PREVIEW"
    ER2_STREAM = "ER2_STREAM"


@dataclass(frozen=True, slots=True)
class ExecutionPlan:
    route_id: str
    text: str
    source: str
    mode: ExecutionMode
    reason: str
    requires_v3: bool
    capability: str | None = None
    action_name: str | None = None
    action_parameters: Mapping[str, object] = field(default_factory=dict)
    camera: bool = False
    tools: bool = False

    def to_jsonable(self) -> dict[str, object]:
        return {
            "schema": "R2B4_EXECUTION_PLAN_V1",
            "route_id": self.route_id,
            "source": self.source,
            "mode": self.mode.value,
            "reason": self.reason,
            "requires_v3": self.requires_v3,
            "capability": self.capability,
            "action_name": self.action_name,
            "action_parameters": dict(self.action_parameters),
            "camera": self.camera,
            "tools": self.tools,
            "text": self.text,
        }


_SPACE_RE = re.compile(r"\s+")
_METRIC_RE = re.compile(
    r"(?<!\w)(?:\d+(?:[.,]\d+)?|egy|kettő|ketto|két|ket|fél|fel)\s*"
    r"(?:m|méter|meter|cm|centi(?:méter|meter)?|mm|milliméter|millimeter)(?!\w)",
    re.IGNORECASE,
)
_ANGLE_RE = re.compile(
    r"(?<!\w)(?:\d+(?:[.,]\d+)?)\s*(?:°|fok(?:kal)?|degree|degrees)(?!\w)",
    re.IGNORECASE,
)

# Commands are deliberately narrow. Ambiguous prose is not turned into actuation.
_STOP_RE = re.compile(
    r"^(?:állj(?:\s+meg)?|allj(?:\s+meg)?|stop|állítsd\s+meg|allitsd\s+meg|megállás|megallas)[.!?]*$",
    re.IGNORECASE,
)
_FORWARD_RE = re.compile(
    r"^(?:menj|indulj|haladj|gurulj)\s+(?:előre|elore)(?:\s+(?:lassan|óvatosan|ovatosan))?[.!?]*$",
    re.IGNORECASE,
)
_BACKWARD_RE = re.compile(
    r"^(?:menj|indulj|haladj|gurulj)\s+(?:hátra|hatra)(?:\s+(?:lassan|óvatosan|ovatosan))?[.!?]*$",
    re.IGNORECASE,
)
_EXPLORE_RE = re.compile(
    r"^(?:járd\s+be|jard\s+be|fedezd\s+fel|nézz\s+körül|nezz\s+korul)(?:\s+(?:a\s+)?(?:szobát|szobat|helyiséget|helyiseget))?[.!?]*$",
    re.IGNORECASE,
)
_FOLLOW_RE = re.compile(
    r"^(?:kövess|kovess|kövess\s+engem|kovess\s+engem|kövesd\s+(?:az\s+)?embert|kovesd\s+(?:az\s+)?embert)[.!?]*$",
    re.IGNORECASE,
)
_FACE_RE = re.compile(
    r"^(?:fordulj\s+(?:az\s+)?ember\s+felé|fordulj\s+(?:az\s+)?ember\s+fele|nézz\s+rám|nezz\s+ram)[.!?]*$",
    re.IGNORECASE,
)

_OBSERVATION_PATTERNS = (
    "mit látsz",
    "mit latsz",
    "mi van előtted",
    "mi van elotted",
    "nézz körül",
    "nezz korul",
    "nézz körbe",
    "nezz korbe",
    "kamera kép",
    "kamerakép",
    "what do you see",
    "look around",
)
_STATUS_PATTERNS = (
    "robot állapot",
    "robot allapot",
    "robot státusz",
    "robot status",
    "v3 állapot",
    "v3 allapot",
    "v3 status",
    "fut a v3",
    "runtime állapot",
    "runtime allapot",
)

# These verbs indicate a likely physical command. If it is not simple enough for
# DIRECT_V3, route it to ER2 rather than silently treating it as ordinary chat.
_ACTUATION_PATTERNS = (
    "menj ", "indulj ", "haladj ", "gurulj ", "fordulj ", "kerüld ", "keruld ",
    "navigálj ", "navigalj ", "kövess", "kovess", "kövesd", "kovesd", "állj meg", "allj meg",
)
_EXPLANATION_PREFIXES = (
    "hogyan ", "miért ", "miert ", "mit jelent", "magyarázd", "magyarazd", "elmagyaráznád", "elmagyaraznad",
    "mi történne", "mi tortenne", "szerinted", "lehetséges", "lehetseges",
)


def _normalized(text: str) -> str:
    return _SPACE_RE.sub(" ", text.strip()).lower()


def _route_id() -> str:
    return f"route-{int(time.time() * 1000)}-{uuid.uuid4().hex[:8]}"


class ExecutionModeSelector:
    """Deterministic, fail-closed first-pass selector.

    The selector intentionally resolves only intents that are safe to classify
    locally. Generic language stays on the default Gemini conversation path.
    Likely physical commands that are not simple canonical actions are routed to
    ER2, never to plain chat.
    """

    def select(self, text: str, *, source: str = "human") -> ExecutionPlan:
        if not isinstance(text, str) or not text.strip():
            raise ValueError("execution request text must be non-empty")
        original = text.strip()
        normalized = _normalized(original)
        route_id = _route_id()

        if _STOP_RE.fullmatch(normalized):
            return ExecutionPlan(
                route_id, original, source, ExecutionMode.DIRECT_V3,
                "EXACT_STOP_INTENT", False,
                capability="v3.command.stop", action_name="v3.command.stop",
            )

        if any(normalized.startswith(prefix) for prefix in _EXPLANATION_PREFIXES):
            return ExecutionPlan(
                route_id, original, source, ExecutionMode.GEMINI_CHAT,
                "EXPLANATORY_OR_INFORMATIONAL_REQUEST", False,
                capability="conversation.text",
            )

        if any(pattern in normalized for pattern in _STATUS_PATTERNS):
            return ExecutionPlan(
                route_id, original, source, ExecutionMode.HOST_READ,
                "ROBOT_STATUS_READ", False,
                capability="operator.status",
            )

        if any(pattern in normalized for pattern in _OBSERVATION_PATTERNS):
            return ExecutionPlan(
                route_id, original, source, ExecutionMode.OBSERVATION,
                "VISUAL_OBSERVATION_REQUEST", False,
                capability="camera.latest+er2.preview", camera=True, tools=False,
            )

        physical_language = any(normalized.startswith(pattern) for pattern in _ACTUATION_PATTERNS)
        metric_or_angle = bool(_METRIC_RE.search(normalized) or _ANGLE_RE.search(normalized))
        if physical_language and metric_or_angle:
            return ExecutionPlan(
                route_id, original, source, ExecutionMode.ER2_STREAM,
                "METRIC_OR_ANGULAR_ROBOT_TASK", True,
                capability="er2.robotics", camera=True, tools=True,
            )

        if _FORWARD_RE.fullmatch(normalized):
            return ExecutionPlan(
                route_id, original, source, ExecutionMode.DIRECT_V3,
                "SIMPLE_CANONICAL_FORWARD", True,
                capability="v3.command.forward", action_name="v3.command.forward",
            )

        if _BACKWARD_RE.fullmatch(normalized):
            return ExecutionPlan(
                route_id, original, source, ExecutionMode.DIRECT_V3,
                "SIMPLE_CANONICAL_BACKWARD", True,
                capability="v3.command.backward", action_name="v3.command.backward",
            )

        if _EXPLORE_RE.fullmatch(normalized):
            return ExecutionPlan(
                route_id, original, source, ExecutionMode.DIRECT_V3,
                "SIMPLE_CANONICAL_EXPLORE", True,
                capability="v3.command.explore", action_name="v3.command.explore",
            )

        if _FOLLOW_RE.fullmatch(normalized):
            return ExecutionPlan(
                route_id, original, source, ExecutionMode.DIRECT_V3,
                "SIMPLE_CANONICAL_FOLLOW_PERSON", True,
                capability="v3.command.follow_person", action_name="v3.command.follow_person",
            )

        if _FACE_RE.fullmatch(normalized):
            return ExecutionPlan(
                route_id, original, source, ExecutionMode.DIRECT_V3,
                "SIMPLE_CANONICAL_FACE_PERSON", True,
                capability="v3.command.face_person", action_name="v3.command.face_person",
            )

        if physical_language:
            return ExecutionPlan(
                route_id, original, source, ExecutionMode.ER2_STREAM,
                "ROBOT_TASK_REQUIRES_HIGHER_LEVEL_REASONING", True,
                capability="er2.robotics", camera=True, tools=True,
            )

        return ExecutionPlan(
            route_id, original, source, ExecutionMode.GEMINI_CHAT,
            "DEFAULT_CONVERSATION", False,
            capability="conversation.text",
        )


class RouteEvidenceJournal:
    """Append-only route evidence with no execution authority."""

    def __init__(self, project_root: Path | str) -> None:
        root = Path(project_root).expanduser().resolve()
        self.path = root / "runtime" / "execution_routes.ndjson"

    def emit(self, event: str, plan: ExecutionPlan, **extra: object) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema": "R2B4_EXECUTION_ROUTE_EVENT_V1",
            "event": event,
            "wall_time_ns": time.time_ns(),
            "monotonic_ns": time.monotonic_ns(),
            "pid": os.getpid(),
            "plan": plan.to_jsonable(),
            **extra,
        }
        line = json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"
        fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
        try:
            os.write(fd, line.encode("utf-8"))
        finally:
            os.close(fd)


__all__ = [
    "ExecutionMode",
    "ExecutionModeSelector",
    "ExecutionPlan",
    "RouteEvidenceJournal",
]
