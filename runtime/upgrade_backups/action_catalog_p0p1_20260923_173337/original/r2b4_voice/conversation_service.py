"""Bounded asynchronous conversation orchestrator for R2B4 voice/LLM turns."""

from __future__ import annotations

import queue
import threading
import time
import uuid
from collections import OrderedDict
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


class SelfKnowledgePort(Protocol):
    def build(self, query: str) -> Mapping[str, object]: ...


@dataclass(frozen=True, slots=True)
class ConversationServiceConfig:
    queue_size: int = 8
    max_history_turns: int = 8
    completion_cache_size: int = 32

    def __post_init__(self) -> None:
        if not isinstance(self.queue_size, int) or isinstance(self.queue_size, bool) or self.queue_size <= 0:
            raise ValueError("queue_size must be a positive integer")
        if not isinstance(self.max_history_turns, int) or isinstance(self.max_history_turns, bool) or self.max_history_turns < 0:
            raise ValueError("max_history_turns must be non-negative")
        if not isinstance(self.completion_cache_size, int) or isinstance(self.completion_cache_size, bool) or self.completion_cache_size <= 0:
            raise ValueError("completion_cache_size must be a positive integer")


class ConversationService:
    def __init__(
        self,
        *,
        llm: LLMPort,
        robot_context: RobotContextBuilder,
        prompt_assembler: PromptAssembler,
        journal: ConversationJournal,
        validator: RobotActionValidator | None = None,
        self_knowledge: SelfKnowledgePort | None = None,
        config: ConversationServiceConfig = ConversationServiceConfig(),
        monotonic_ns=time.monotonic_ns,
    ) -> None:
        if not callable(getattr(llm, "complete", None)):
            raise TypeError("llm must provide complete()")
        if self_knowledge is not None and not callable(getattr(self_knowledge, "build", None)):
            raise TypeError("self_knowledge must provide build()")
        self._llm = llm
        self._context = robot_context
        self._prompt = prompt_assembler
        self._journal = journal
        self._validator = validator or RobotActionValidator()
        self._self_knowledge = self_knowledge
        self._config = config
        self._monotonic_ns = monotonic_ns
        self._queue: queue.Queue[UserTextTurn | None] = queue.Queue(maxsize=config.queue_size)
        self._history: list[ConversationMemoryTurn] = []
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._completed: OrderedDict[str, ConversationTurnResult] = OrderedDict()
        self._last_turn: ConversationTurnResult | None = None
        self._last_error: str | None = None
        self._closed = False
        self._worker = threading.Thread(target=self._run, name="r2b4-conversation", daemon=True)
        self._worker.start()
        self._journal.append(
            "session_start",
            {"prompt_version": PROMPT_VERSION, "model": self._llm.model, "action_mode": "PROPOSAL_ONLY"},
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
                "completed_turns_cached": len(self._completed),
                "completion_cache_capacity": self._config.completion_cache_size,
                "last_error": self._last_error,
                "action_mode": "PROPOSAL_ONLY",
                "model": self._llm.model,
            }

    def last_turn(self) -> dict[str, object] | None:
        with self._lock:
            return None if self._last_turn is None else self._last_turn.to_jsonable()

    def wait_for_turn(self, turn_id: str, timeout_s: float = 20.0) -> dict[str, object] | None:
        if not isinstance(turn_id, str) or not turn_id:
            raise ValueError("turn_id must be non-empty")
        if not isinstance(timeout_s, (int, float)) or isinstance(timeout_s, bool) or timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        deadline = time.monotonic() + float(timeout_s)
        with self._condition:
            while True:
                result = self._completed.get(turn_id)
                if result is not None:
                    return result.to_jsonable()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(timeout=remaining)

    def close(self, timeout_s: float = 2.0) -> None:
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._condition.notify_all()
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

    def _store_result(self, result: ConversationTurnResult, *, error: str | None) -> None:
        with self._condition:
            self._last_turn = result
            self._last_error = error
            self._completed[result.turn_id] = result
            self._completed.move_to_end(result.turn_id)
            while len(self._completed) > self._config.completion_cache_size:
                self._completed.popitem(last=False)
            self._condition.notify_all()

    def _process(self, turn: UserTextTurn) -> None:
        try:
            context = self._context.build()
            knowledge: Mapping[str, object] | None = None
            if self._self_knowledge is not None:
                try:
                    knowledge = self._self_knowledge.build(turn.text)
                except Exception as exc:
                    # Self-knowledge is informative only; failure must not break normal dialogue.
                    self._journal.append(
                        "self_knowledge_error",
                        {"turn_id": turn.turn_id, "error": f"{type(exc).__name__}: {exc}"},
                    )
            self._journal.append(
                "llm_request_meta",
                {
                    "turn_id": turn.turn_id,
                    "prompt_version": PROMPT_VERSION,
                    "model": self._llm.model,
                    "robot_context": context.to_jsonable(),
                    "self_knowledge_categories": (
                        list(knowledge.get("matched_categories", []))
                        if isinstance(knowledge, Mapping)
                        else []
                    ),
                },
            )
            with self._lock:
                history = tuple(self._history[-self._config.max_history_turns :])
            if knowledge:
                messages = self._prompt.build_messages(
                    turn,
                    context,
                    history,
                    self_knowledge=knowledge,
                )
            else:
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
                if decision.spoken_text:
                    self._history.append(ConversationMemoryTurn(turn.text, decision.spoken_text))
                    if len(self._history) > self._config.max_history_turns:
                        del self._history[: len(self._history) - self._config.max_history_turns]
            self._store_result(result, error=None)
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
            self._store_result(result, error=error)


__all__ = [
    "ConversationBusyError",
    "ConversationService",
    "ConversationServiceConfig",
    "LLMPort",
    "SelfKnowledgePort",
]
