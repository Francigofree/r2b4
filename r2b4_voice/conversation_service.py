"""Bounded asynchronous conversation orchestrator for R2B4 voice/LLM turns."""

from __future__ import annotations

import queue
import math
import threading
import time
import uuid
from collections import OrderedDict
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, replace
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


class AgentPort(Protocol):
    @property
    def model(self) -> str: ...

    def run(
        self,
        messages: Sequence[Mapping[str, str]],
        action_catalog: Sequence[Mapping[str, object]],
        *,
        event_sink: Callable[[str, Mapping[str, object]], None] | None = None,
        cancel_event: threading.Event | None = None,
        deadline: float | None = None,
    ) -> LLMDecision: ...


class SelfKnowledgePort(Protocol):
    def build(self, query: str) -> Mapping[str, object]: ...


@dataclass(frozen=True, slots=True)
class ConversationServiceConfig:
    queue_size: int = 8
    max_history_turns: int = 8
    completion_cache_size: int = 32
    turn_timeout_s: float = 90.0

    def __post_init__(self) -> None:
        if (isinstance(self.turn_timeout_s, bool) or not isinstance(self.turn_timeout_s, (int, float))
                or not math.isfinite(self.turn_timeout_s) or self.turn_timeout_s <= 0):
            raise ValueError("turn_timeout_s must be finite and positive")
        if not isinstance(self.queue_size, int) or isinstance(self.queue_size, bool) or self.queue_size <= 0:
            raise ValueError("queue_size must be a positive integer")
        if not isinstance(self.max_history_turns, int) or isinstance(self.max_history_turns, bool) or self.max_history_turns < 0:
            raise ValueError("max_history_turns must be non-negative")
        if not isinstance(self.completion_cache_size, int) or isinstance(self.completion_cache_size, bool) or self.completion_cache_size <= 0:
            raise ValueError("completion_cache_size must be a positive integer")


@dataclass(frozen=True, slots=True)
class _PendingTurn:
    turn: UserTextTurn
    cancelled: threading.Event
    deadline: float
    goal_id: str | None = None


class ConversationService:
    def __init__(
        self,
        *,
        llm: LLMPort,
        robot_context: RobotContextBuilder,
        prompt_assembler: PromptAssembler,
        journal: ConversationJournal,
        agent: AgentPort | None = None,
        validator: RobotActionValidator | None = None,
        self_knowledge: SelfKnowledgePort | None = None,
        config: ConversationServiceConfig = ConversationServiceConfig(),
        monotonic_ns=time.monotonic_ns,
        brain_interface: object | None = None,
        observation_sink: Callable[[str, Mapping[str, object]], None] | None = None,
    ) -> None:
        if not callable(getattr(llm, "complete", None)):
            raise TypeError("llm must provide complete()")
        if agent is not None and not callable(getattr(agent, "run", None)):
            raise TypeError("agent must provide run()")
        if self_knowledge is not None and not callable(getattr(self_knowledge, "build", None)):
            raise TypeError("self_knowledge must provide build()")
        self._llm = llm
        self._agent = agent
        self._context = robot_context
        self._prompt = prompt_assembler
        self._journal = journal
        self._validator = validator or RobotActionValidator()
        self._self_knowledge = self_knowledge
        self._config = config
        self._monotonic_ns = monotonic_ns
        self._brain_interface = brain_interface
        self._observation_sink = observation_sink
        self._observation_producer_id = uuid.uuid4().hex
        self._observation_sequence = 0
        self._observation_dropped = 0
        self._queue: queue.Queue[_PendingTurn | None] = queue.Queue(maxsize=config.queue_size)
        self._pending: dict[str, _PendingTurn] = {}
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
            {
                "prompt_version": PROMPT_VERSION,
                "model": self.model,
                "action_mode": "PROPOSAL_ONLY",
                "agent_core": self._agent is not None,
            },
        )

    @property
    def model(self) -> str:
        return self._agent.model if self._agent is not None else self._llm.model

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
        goal_id = None
        if self._brain_interface is not None:
            accepted = self._brain_interface.execute(
                "brain.submit", text=turn.text,
                source="AUTONOMOUS" if source.upper() == "AUTONOMOUS" else "HUMAN",
                request_id=turn.turn_id,
            )
            goal_id = accepted.get("goal_id") if isinstance(accepted, Mapping) else None
            if not isinstance(goal_id, str) or not goal_id:
                raise RuntimeError("Brain admission returned no goal_id")
        try:
            with self._lock:
                if self._closed:
                    raise RuntimeError("conversation service is closed")
                pending = _PendingTurn(turn, threading.Event(), time.monotonic() + self._config.turn_timeout_s, goal_id)
                self._queue.put_nowait(pending)
                self._pending[turn.turn_id] = pending
        except queue.Full as exc:
            self._finish_brain(goal_id, "CONVERSATION_QUEUE_FULL")
            self._journal.append(
                "user_rejected",
                {"turn_id": turn.turn_id, "source": turn.source, "text": turn.text, "reason": "QUEUE_FULL"},
                monotonic_ns=turn.monotonic_ns,
            )
            raise ConversationBusyError("conversation queue is full") from exc
        except RuntimeError:
            self._cancel_brain_goals((goal_id,) if goal_id else (), "CONVERSATION_CLOSED")
            raise
        return turn.turn_id

    def _finish_brain(self, goal_id: str | None, reason: str) -> None:
        if goal_id is not None and self._brain_interface is not None:
            self._brain_interface.execute("brain.fail", goal_id=goal_id, reason=reason, pending_only=True)

    def _cancel_brain_goals(self, goal_ids: tuple[str, ...], reason: str) -> None:
        if not goal_ids or self._brain_interface is None:
            return
        def finish() -> None:
            for goal_id in goal_ids:
                try:
                    self._finish_brain(goal_id, reason)
                except Exception:
                    pass
        # STOP only sets local cancellation flags; socket availability cannot
        # delay delivery of its canonical physical command.
        threading.Thread(target=finish, name="r2b4-pending-goal-cancel", daemon=True).start()

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
                "observation_dropped": self._observation_dropped,
                "action_mode": "PROPOSAL_ONLY",
                "model": self.model,
                "agent_core": self._agent is not None,
            }

    def last_turn(self) -> dict[str, object] | None:
        with self._lock:
            return None if self._last_turn is None else self._last_turn.to_jsonable()

    def wait_for_turn(self, turn_id: str, timeout_s: float = 20.0) -> dict[str, object] | None:
        if not isinstance(turn_id, str) or not turn_id:
            raise ValueError("turn_id must be non-empty")
        if not isinstance(timeout_s, (int, float)) or isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        deadline = time.monotonic() + float(timeout_s)
        with self._condition:
            while True:
                if self._closed:
                    return None
                result = self._completed.get(turn_id)
                if result is not None:
                    return result.to_jsonable()
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self._closed:
                    pending = self._pending.get(turn_id)
                    if pending is not None:
                        pending.cancelled.set()
                    goal_id = pending.goal_id if pending is not None else None
                    break
                self._condition.wait(timeout=remaining)
        self._cancel_brain_goals((goal_id,) if goal_id else (), "CONVERSATION_TIMEOUT")
        return None

    def cancel_pending_turns(self) -> None:
        """Revoke queued/running host intents, including in-flight delegates."""
        with self._condition:
            goal_ids = tuple(pending.goal_id for pending in self._pending.values()
                             if pending.goal_id is not None and not pending.cancelled.is_set())
            for pending in self._pending.values():
                pending.cancelled.set()
            self._condition.notify_all()
        self._cancel_brain_goals(goal_ids, "CONVERSATION_CANCELLED")

    def close(self, timeout_s: float = 2.0) -> None:
        with self._condition:
            if self._closed:
                return
            self._closed = True
            goal_ids = tuple(pending.goal_id for pending in self._pending.values()
                             if pending.goal_id is not None and not pending.cancelled.is_set())
            for pending in self._pending.values():
                pending.cancelled.set()
            self._condition.notify_all()
        self._cancel_brain_goals(goal_ids, "CONVERSATION_CLOSED")
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        self._worker.join(timeout=timeout_s)
        self._journal.append("session_stop", {"worker_alive": self._worker.is_alive()})

    def _run(self) -> None:
        while True:
            with self._lock:
                if self._closed:
                    return
            item = self._queue.get()
            try:
                if item is None:
                    return
                self._process(item)
            finally:
                if item is not None:
                    with self._lock:
                        self._pending.pop(item.turn.turn_id, None)
                self._queue.task_done()

    def _store_result(self, result: ConversationTurnResult, *, error: str | None) -> ConversationTurnResult:
        with self._condition:
            pending = self._pending.get(result.turn_id)
            if result.error is None and (self._closed or (pending is not None and (
                pending.cancelled.is_set() or time.monotonic() >= pending.deadline
            ))):
                error = "CONVERSATION_TURN_CANCELLED_OR_EXPIRED"
                result = replace(result, spoken_text=None, proposed_action=None, proposed_plan=None,
                                 action_status="ERROR", error=error)
            if result.error is None and result.spoken_text:
                self._history.append(ConversationMemoryTurn(result.user_text, result.spoken_text))
                if len(self._history) > self._config.max_history_turns:
                    del self._history[: len(self._history) - self._config.max_history_turns]
            self._last_turn = result
            self._last_error = error
            self._completed[result.turn_id] = result
            self._completed.move_to_end(result.turn_id)
            while len(self._completed) > self._config.completion_cache_size:
                self._completed.popitem(last=False)
            self._condition.notify_all()
            return result

    def _process(self, pending: _PendingTurn) -> None:
        turn = pending.turn
        def check_active() -> None:
            if pending.cancelled.is_set() or time.monotonic() >= pending.deadline:
                raise TimeoutError("CONVERSATION_TURN_CANCELLED_OR_EXPIRED")
        try:
            check_active()
            self._observe_agent("AGENT_TURN_STARTED", pending, {"source": turn.source})
            context = self._context.build()
            knowledge: Mapping[str, object] | None = None
            if self._self_knowledge is not None:
                try:
                    knowledge = self._self_knowledge.build(turn.text)
                except Exception as exc:
                    self._journal.append(
                        "self_knowledge_error",
                        {"turn_id": turn.turn_id, "error": f"{type(exc).__name__}: {exc}"},
                    )
            self._journal.append(
                "llm_request_meta",
                {
                    "turn_id": turn.turn_id,
                    "prompt_version": PROMPT_VERSION,
                    "model": self.model,
                    "robot_context": context.to_jsonable(),
                    "agent_core": self._agent is not None,
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
                messages = self._prompt.build_messages(turn, context, history, self_knowledge=knowledge)
            else:
                messages = self._prompt.build_messages(turn, context, history)
            if pending.goal_id is not None:
                messages = list(messages)
                user_index = next((index for index in range(len(messages) - 1, -1, -1)
                                   if messages[index].get("role") == "user"), len(messages))
                messages.insert(user_index, {"role": "system", "content":
                    "PROMPT_LAYER_KIND=UNTRUSTED_RUNTIME_DATA\nBRAIN_PENDING_GOAL_ID=" + pending.goal_id})

            if self._agent is not None:
                def emit(event: str, payload: Mapping[str, object]) -> None:
                    self._journal.append(event, {"turn_id": turn.turn_id, **dict(payload)})
                    self._observe_agent(event.upper(), pending, payload)

                decision = self._agent.run(messages, context.available_actions, event_sink=emit,
                                           cancel_event=pending.cancelled, deadline=pending.deadline)
            else:
                complete_with_actions = getattr(self._llm, "complete_with_actions", None)
                if callable(complete_with_actions):
                    decision = complete_with_actions(messages, context.available_actions)
                else:
                    decision = self._llm.complete(messages)

            check_active()
            action_status = "NONE"
            if decision.robot_action is not None:
                validation = self._validator.validate(decision.robot_action, context)
                action_status = validation.reason if validation.accepted else f"REJECTED:{validation.reason}"
            elif decision.goal_plan is not None:
                action_status = "PLAN_PROPOSED"
            else:
                self._finish_brain(pending.goal_id, "ANSWERED")

            result = ConversationTurnResult(
                turn_id=turn.turn_id,
                user_text=turn.text,
                spoken_text=decision.spoken_text,
                proposed_action=decision.robot_action,
                action_status=action_status,
                error=None,
                model=decision.model,
                goal_id=pending.goal_id,
                proposed_plan=decision.goal_plan,
            )
            self._journal.append("assistant", result.to_jsonable())
            result = self._store_result(result, error=None)
            self._observe_agent("AGENT_TURN_FAILED" if result.error else "AGENT_TURN_COMPLETED", pending, {
                "status": result.action_status,
                "error": result.error,
                "action_name": result.proposed_action.name if result.proposed_action is not None else None,
            })
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            try:
                self._finish_brain(pending.goal_id, "INTERPRETATION_FAILED:" + type(exc).__name__)
            except Exception:
                pass
            result = ConversationTurnResult(
                turn_id=turn.turn_id,
                user_text=turn.text,
                spoken_text=None,
                proposed_action=None,
                action_status="ERROR",
                error=error,
                model=self.model,
                goal_id=pending.goal_id,
            )
            self._journal.append("error", result.to_jsonable())
            result = self._store_result(result, error=error)
            self._observe_agent("AGENT_TURN_FAILED", pending, {
                "status": "ERROR", "error": type(exc).__name__,
            })

    def _observe_agent(self, event: str, pending: _PendingTurn, fields: Mapping[str, object]) -> None:
        """Compact host evidence; storage failure never changes a turn or proposal.

        The sink is a bounded enqueue callback; the composition's collector
        owns journal I/O. Tool arguments, results, images and provider messages
        never enter the flight recorder.
        """
        if self._observation_sink is None:
            return
        self._observation_sequence += 1
        row: dict[str, object] = {
            "observation_topic": "r2b4.agent",
            "monotonic_ns": self._monotonic_ns(),
            "request_time_ns": pending.turn.monotonic_ns,
            "session_id": self.session_id,
            "turn_id": pending.turn.turn_id,
            "goal_id": pending.goal_id,
            "producer_id": self._observation_producer_id,
            "event_sequence": self._observation_sequence,
            "evidence_dropped": self._observation_dropped,
        }
        for key in ("source", "round", "tool", "status", "error", "action_name"):
            value = fields.get(key)
            if value is None or type(value) in (bool, int):
                row[key] = value
            elif isinstance(value, str):
                row[key] = value[:256]
        try:
            self._observation_sink(event, row)
        except Exception:
            self._observation_dropped += 1


__all__ = [
    "AgentPort",
    "ConversationBusyError",
    "ConversationService",
    "ConversationServiceConfig",
    "LLMPort",
    "SelfKnowledgePort",
]
