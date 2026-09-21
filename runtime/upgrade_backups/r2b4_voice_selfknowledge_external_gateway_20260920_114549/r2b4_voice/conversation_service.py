"""Bounded asynchronous conversation orchestrator for R2B4 voice/LLM turns."""

from __future__ import annotations

import queue
import threading
import time
import uuid
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Protocol

from .action_validation import RobotActionValidator
from .conversation_contracts import (
    ConversationMemoryTurn,
    ConversationTurnResult,
    LLMDecision,
    UserTextTurn,
)
from .conversation_journal import ConversationJournal
from .prompting import PROMPT_VERSION, PromptAssembler
from .robot_context import RobotContextBuilder


class ConversationBusyError(RuntimeError):
    pass


class LLMPort(Protocol):
    @property
    def model(self) -> str: ...
    def complete(self, messages: Sequence[Mapping[str, str]]) -> LLMDecision: ...


@dataclass(frozen=True, slots=True)
class ConversationServiceConfig:
    queue_size: int = 8
    max_history_turns: int = 8

    def __post_init__(self) -> None:
        if not isinstance(self.queue_size, int) or isinstance(self.queue_size, bool) or self.queue_size <= 0:
            raise ValueError("queue_size must be a positive integer")
        if not isinstance(self.max_history_turns, int) or isinstance(self.max_history_turns, bool) or self.max_history_turns < 0:
            raise ValueError("max_history_turns must be non-negative")


class ConversationService:
    def __init__(
        self,
        *,
        llm: LLMPort,
        robot_context: RobotContextBuilder,
        prompt_assembler: PromptAssembler,
        journal: ConversationJournal,
        validator: RobotActionValidator | None = None,
        config: ConversationServiceConfig = ConversationServiceConfig(),
        monotonic_ns=time.monotonic_ns,
    ) -> None:
        if not callable(getattr(llm, "complete", None)):
            raise TypeError("llm must provide complete()")
        self._llm = llm
        self._context = robot_context
        self._prompt = prompt_assembler
        self._journal = journal
        self._validator = validator or RobotActionValidator()
        self._config = config
        self._monotonic_ns = monotonic_ns
        self._queue: queue.Queue[UserTextTurn | None] = queue.Queue(maxsize=config.queue_size)
        self._history: list[ConversationMemoryTurn] = []
        self._lock = threading.Lock()
        self._last_turn: ConversationTurnResult | None = None
        self._last_error: str | None = None
        self._closed = False
        self._worker = threading.Thread(target=self._run, name="r2b4-conversation", daemon=True)
        self._worker.start()
        self._journal.append(
            "session_start",
            {"prompt_version": PROMPT_VERSION, "model": self._llm.model, "action_mode": "SHADOW"},
        )

    @property
    def session_id(self) -> str:
        return self._journal.session_id

    def submit_text(self, text: str, *, source: str = "stt") -> str:
        with self._lock:
            if self._closed:
                raise RuntimeError("conversation service is closed")
        turn = UserTextTurn(
            turn_id=uuid.uuid4().hex,
            text=text,
            source=source,
            monotonic_ns=self._monotonic_ns(),
        )
        # Journal the human input before making it visible to the worker so
        # the append-only record preserves causal order even on a very fast LLM.
        self._journal.append(
            "user",
            {"turn_id": turn.turn_id, "source": turn.source, "text": turn.text},
            monotonic_ns=turn.monotonic_ns,
        )
        try:
            self._queue.put_nowait(turn)
        except queue.Full as exc:
            self._journal.append(
                "user_rejected",
                {"turn_id": turn.turn_id, "source": turn.source, "text": turn.text, "reason": "QUEUE_FULL"},
                monotonic_ns=turn.monotonic_ns,
            )
            raise ConversationBusyError("conversation queue is full") from exc
        return turn.turn_id

    def status(self) -> dict[str, object]:
        with self._lock:
            last_turn_id = self._last_turn.turn_id if self._last_turn is not None else None
            return {
                "state": "CLOSED" if self._closed else "RUNNING",
                "session_id": self._journal.session_id,
                "journal": str(self._journal.path),
                "queue_depth": self._queue.qsize(),
                "queue_capacity": self._config.queue_size,
                "worker_alive": self._worker.is_alive(),
                "last_turn_id": last_turn_id,
                "last_error": self._last_error,
                "action_mode": "SHADOW",
                "model": self._llm.model,
            }

    def last_turn(self) -> dict[str, object] | None:
        with self._lock:
            return None if self._last_turn is None else self._last_turn.to_jsonable()

    def wait_for_turn(self, turn_id: str, timeout_s: float = 20.0) -> dict[str, object] | None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            item = self.last_turn()
            if isinstance(item, dict) and item.get("turn_id") == turn_id:
                return item
            time.sleep(0.02)
        return None

    def close(self, timeout_s: float = 2.0) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        try:
            self._queue.put(None, timeout=max(0.01, timeout_s))
        except queue.Full:
            pass
        self._worker.join(timeout=timeout_s)
        self._journal.append("session_stop", {"worker_alive": self._worker.is_alive()})

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is None:
                    return
                self._process(item)
            finally:
                self._queue.task_done()

    def _process(self, turn: UserTextTurn) -> None:
        try:
            context = self._context.build()
            self._journal.append(
                "llm_request_meta",
                {
                    "turn_id": turn.turn_id,
                    "prompt_version": PROMPT_VERSION,
                    "model": self._llm.model,
                    "robot_context": context.to_jsonable(),
                },
            )
            with self._lock:
                history = tuple(self._history[-self._config.max_history_turns :])
            messages = self._prompt.build_messages(turn, context, history)
            decision = self._llm.complete(messages)

            action_status = "NONE"
            if decision.robot_action is not None:
                validation = self._validator.validate(decision.robot_action, context)
                action_status = validation.reason if validation.accepted else f"REJECTED:{validation.reason}"

            result = ConversationTurnResult(
                turn_id=turn.turn_id,
                user_text=turn.text,
                spoken_text=decision.spoken_text,
                proposed_action=decision.robot_action,
                action_status=action_status,
                error=None,
                model=decision.model,
            )
            self._journal.append("assistant", result.to_jsonable())
            with self._lock:
                self._last_turn = result
                self._last_error = None
                if decision.spoken_text:
                    self._history.append(ConversationMemoryTurn(turn.text, decision.spoken_text))
                    if len(self._history) > self._config.max_history_turns:
                        del self._history[: len(self._history) - self._config.max_history_turns]
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            result = ConversationTurnResult(
                turn_id=turn.turn_id,
                user_text=turn.text,
                spoken_text=None,
                proposed_action=None,
                action_status="ERROR",
                error=error,
                model=getattr(self._llm, "model", None),
            )
            self._journal.append("error", result.to_jsonable())
            with self._lock:
                self._last_turn = result
                self._last_error = error


__all__ = [
    "ConversationBusyError",
    "ConversationService",
    "ConversationServiceConfig",
    "LLMPort",
]
