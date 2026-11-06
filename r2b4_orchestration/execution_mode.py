"""Minimal system entry routing for natural-language R2B4 requests.

Exact STOP remains an immediate deterministic fast-path. Other requests enter
the Brain-owned conversation service, whose stateless Local Task Planner first
resolves known capabilities. Unresolved semantics reach the provider-neutral
Agent Core as bounded proposals or answers.

Legacy mode enum members remain for compatibility with explicit/internal plans.
"""
from __future__ import annotations

import json
import os
import re
import sys
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Mapping


class ExecutionMode(str, Enum):
    AGENT = "AGENT"
    # Compatibility alias for callers/tests that still refer to the historical
    # name. New route evidence serializes this member as AGENT.
    GEMINI_CHAT = "AGENT"
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
            "schema": "R2B4_EXECUTION_PLAN_V2",
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
_STOP_RE = re.compile(
    r"^(?:állj(?:\s+meg)?|allj(?:\s+meg)?|stop|állítsd\s+meg|allitsd\s+meg|megállás|megallas)[.!?]*$",
    re.IGNORECASE,
)


def _normalized(text: str) -> str:
    return _SPACE_RE.sub(" ", text.strip()).lower()


def _route_id() -> str:
    return f"route-{int(time.time() * 1000)}-{uuid.uuid4().hex[:8]}"


def is_stop_intent(text: str) -> bool:
    return isinstance(text, str) and bool(_STOP_RE.fullmatch(_normalized(text)))


class ExecutionModeSelector:
    """Deterministic entry gate: exact STOP or Brain conversation ingress.

    This component intentionally does not try to understand open-ended human
    language. Local task resolution and specialist escalation happen inside the
    conversation service; actual admission remains Brain-owned.
    """

    def select(self, text: str, *, source: str = "human") -> ExecutionPlan:
        if not isinstance(text, str) or not text.strip():
            raise ValueError("execution request text must be non-empty")
        original = text.strip()
        route_id = _route_id()
        if is_stop_intent(original):
            return ExecutionPlan(
                route_id,
                original,
                source,
                ExecutionMode.DIRECT_V3,
                "EXACT_STOP_FAST_PATH",
                False,
                capability="v3.command.stop",
                action_name="v3.command.stop",
            )
        return ExecutionPlan(
            route_id,
            original,
            source,
            ExecutionMode.AGENT,
            "DEFAULT_AGENT_CORE",
            False,
            capability="conversation.text",
        )


class RouteEvidenceJournal:
    """Append-only entry-route evidence with no execution authority."""

    def __init__(self, project_root: Path | str) -> None:
        root = Path(project_root).expanduser().resolve()
        self.path = root / "runtime" / "execution_routes.ndjson"
        self.evidence_dropped = 0
        self.last_error: str | None = None

    def emit(self, event: str, plan: ExecutionPlan, **extra: object) -> None:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            payload = {
                "schema": "R2B4_EXECUTION_ROUTE_EVENT_V2",
                "event": event,
                "wall_time_ns": time.time_ns(),
                "monotonic_ns": time.monotonic_ns(),
                "pid": os.getpid(),
                "plan": plan.to_jsonable(),
                "evidence_dropped": self.evidence_dropped,
                **extra,
            }
            line = (json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")
            fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
            try:
                if os.write(fd, line) != len(line):
                    raise OSError("incomplete route evidence write")
            finally:
                os.close(fd)
        except Exception as exc:
            # Route evidence has no execution authority. Keep loss explicit,
            # including on recovery, without vetoing STOP or a Brain request.
            self.evidence_dropped += 1
            self.last_error = f"ROUTE_EVIDENCE_WRITE_FAILED:{type(exc).__name__}:{exc}"
            if self.evidence_dropped == 1:
                try:
                    print(f"route evidence: {self.last_error}", file=sys.stderr, flush=True)
                except OSError:
                    pass


__all__ = [
    "ExecutionMode",
    "ExecutionModeSelector",
    "ExecutionPlan",
    "RouteEvidenceJournal",
    "is_stop_intent",
]
