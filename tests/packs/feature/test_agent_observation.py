"""Agent reasoning evidence joins the passive recorder without raw media."""

from types import SimpleNamespace
import queue
import threading
import time

import pytest

from r2b4_voice.conversation_contracts import LLMDecision, RobotContextSnapshot
from r2b4_voice.conversation_journal import ConversationJournal
from r2b4_voice.conversation_interface import _AgentObservationJournal
from r2b4_voice.conversation_service import ConversationService
from v3.hri_evidence import HRI_EVENT_TOPIC, HriEventFollower, default_hri_journal


def service(root, sink, *, fail=False, llm_metadata=None):
    class Model:
        model = "offline-agent"
        def complete(self, _messages):
            raise AssertionError("agent supplies the decision")

    class Agent:
        model = "offline-agent"
        def run(self, _messages, _actions, *, event_sink, **_options):
            event_sink("agent_tool_requested", {
                "round": 1, "tool": "vision.observe", "arguments": {"raw_image": "never record"},
            })
            event_sink("agent_tool_completed", {
                "round": 1, "tool": "vision.observe", "status": "COMPLETED",
                "images": ["large image"], "data": {"raw": "payload"},
            })
            if llm_metadata is not None:
                event_sink("agent_llm_completed", {
                    **llm_metadata, "messages": ["never record provider content"],
                })
            if fail:
                raise RuntimeError("model failed")
            return LLMDecision("Kész.", None, self.model)

    calls = []
    class Brain:
        def execute(self, action, **parameters):
            calls.append((action, parameters))
            return {"goal_id": "goal-recorder"}

    context = RobotContextSnapshot("offline", {}, None, None, (), ())
    return ConversationService(
        llm=Model(), agent=Agent(), brain_interface=Brain(),
        robot_context=SimpleNamespace(build=lambda: context),
        prompt_assembler=SimpleNamespace(build_messages=lambda *_args, **_kwargs: []),
        journal=ConversationJournal(root / "conversations"), observation_sink=sink,
    ), calls


@pytest.mark.parametrize("fail", [False, True])
def test_agent_turn_tool_and_goal_lineage_reaches_live_hri_edge(tmp_path, fail):
    journal = default_hri_journal(tmp_path)
    follower = HriEventFollower(tmp_path)
    follower.start()
    conversation, calls = service(tmp_path, lambda event, fields: journal.append(event, **fields), fail=fail)
    try:
        turn_id = conversation.submit_text("Körülnézés", source="launcher")
        result = conversation.wait_for_turn(turn_id, timeout_s=2)
        assert result["goal_id"] == "goal-recorder"
        # The result wakes its client before passive terminal publication.
        rows = []
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rows.extend(payload for topic, payload in follower.drain() if topic == HRI_EVENT_TOPIC)
            if any(row["event_type"] in {"AGENT_TURN_FAILED", "AGENT_TURN_COMPLETED"} for row in rows):
                break
            time.sleep(.001)
        assert [row["event_type"] for row in rows] == [
            "AGENT_TURN_STARTED", "PROMPT_SIZE", "AGENT_TOOL_REQUESTED", "AGENT_TOOL_COMPLETED",
            "AGENT_TURN_FAILED" if fail else "AGENT_TURN_COMPLETED",
        ]
        assert all(row["turn_id"] == turn_id and row["goal_id"] == result["goal_id"] for row in rows)
        assert {row["session_id"] for row in rows} == {conversation.session_id}
        assert [row["event_sequence"] for row in rows] == list(range(1, len(rows) + 1))
        assert all(row["request_time_ns"] <= row["monotonic_ns"] for row in rows)
        assert all(row["observation_topic"] == "r2b4.agent" for row in rows)
        assert not any(key in row for row in rows for key in ("arguments", "data", "images", "messages"))
        assert all(value is None or type(value) in (str, int, bool) for row in rows for value in row.values())
        assert [action for action, _ in calls] == ["brain.submit", "brain.fail"]
        assert bool(result["error"]) is fail
    finally:
        conversation.close()
        follower.close()


def test_broken_evidence_sink_does_not_change_agent_answer_or_brain_result(tmp_path):
    rows = []
    def sink(event, fields):
        if not rows:
            rows.append((event, None))
            raise OSError("journal unavailable")
        rows.append((event, fields))
    conversation, calls = service(tmp_path, sink)
    try:
        turn_id = conversation.submit_text("Teszt", source="launcher")
        result = conversation.wait_for_turn(turn_id, timeout_s=2)
        assert result["spoken_text"] == "Kész." and result["error"] is None
        assert conversation.status()["observation_dropped"] == 1
        assert all(row["evidence_dropped"] == 1 for _, row in rows[1:])
        assert rows[1][1]["event_sequence"] == 2
        assert [action for action, _ in calls] == ["brain.submit", "brain.fail"]
    finally:
        conversation.close()


def test_groq_admission_estimate_and_reported_usage_reach_passive_hri_without_prompt(tmp_path):
    journal = default_hri_journal(tmp_path)
    follower = HriEventFollower(tmp_path)
    follower.start()
    size = {
        "provider": "groq", "attempt_count": 3,
        "provider_request_utf8_bytes": 19000,
        "provider_request_token_estimate": 7486,
        "provider_request_token_budget": 8000,
        "max_completion_tokens": 1024,
        "token_estimate_method": "utf8_bytes/3+128+max_completion_tokens",
        "input_tokens": 3100, "output_tokens": 120,
    }
    conversation, _calls = service(tmp_path, lambda event, fields: journal.append(event, **fields),
                                  llm_metadata=size)
    try:
        turn_id = conversation.submit_text("Megfigyelés", source="launcher")
        assert conversation.wait_for_turn(turn_id, timeout_s=2)["error"] is None
    finally:
        conversation.close()
    try:
        rows = [row for batch in follower.finish() for topic, row in batch if topic == HRI_EVENT_TOPIC]
        completed = next(row for row in rows if row["event_type"] == "AGENT_LLM_COMPLETED")
        assert {key: completed[key] for key in size} == size
        assert completed["turn_id"] == turn_id
        assert not any(key in completed for key in ("messages", "images", "data", "arguments"))
    finally:
        follower.close()


def test_slow_journal_never_delays_agent_and_close_drains_accepted_evidence(tmp_path):
    journal = default_hri_journal(tmp_path)
    entered, release = threading.Event(), threading.Event()
    class SlowJournal:
        def append(self, event, **fields):
            entered.set()
            assert release.wait(2)
            return journal.append(event, **fields)
    observation = _AgentObservationJournal(SlowJournal())
    conversation, _ = service(tmp_path, observation.offer)
    follower = HriEventFollower(tmp_path)
    follower.start()
    try:
        turn_id = conversation.submit_text("Megfigyelés", source="launcher")
        assert entered.wait(1)
        result = conversation.wait_for_turn(turn_id, timeout_s=1)
        assert result["spoken_text"] == "Kész." and result["error"] is None
    finally:
        release.set()
        conversation.close()
        observation.close()
    try:
        rows = [row for batch in follower.finish() for topic, row in batch if topic == HRI_EVENT_TOPIC]
        assert [row["event_sequence"] for row in rows] == list(range(1, len(rows) + 1))
        assert rows[-1]["event_type"] == "AGENT_TURN_COMPLETED"
    finally:
        follower.close()


def test_agent_queue_overflow_is_explicit_even_when_last_event_is_lost(tmp_path):
    journal = default_hri_journal(tmp_path)
    entered, release = threading.Event(), threading.Event()
    class SlowJournal:
        def append(self, event, **fields):
            entered.set()
            assert release.wait(2)
            return journal.append(event, **fields)
    observation = _AgentObservationJournal(SlowJournal())
    # A smaller mailbox makes the same bounded-overflow case deterministic.
    observation._queue = queue.Queue(maxsize=2)
    follower = HriEventFollower(tmp_path)
    follower.start()
    def fields(sequence):
        return {"monotonic_ns": time.monotonic_ns(), "producer_id": "agent-overflow",
                "event_sequence": sequence, "evidence_dropped": 0}
    try:
        observation.offer("AGENT_TURN_STARTED", fields(1))
        assert entered.wait(1)
        observation.offer("AGENT_TOOL_REQUESTED", fields(2))
        observation.offer("AGENT_TOOL_COMPLETED", fields(3))
        with pytest.raises(queue.Full):
            observation.offer("AGENT_TURN_COMPLETED", fields(4))
    finally:
        release.set()
        observation.close()
    try:
        frames = [frame for batch in follower.finish() for frame in batch]
        rows = [row for topic, row in frames if topic == HRI_EVENT_TOPIC]
        assert [row["event_sequence"] for row in rows] == [1, 2, 3, 5]
        assert rows[-1]["event_type"] == "AGENT_EVIDENCE_LOSS"
        assert rows[-1]["evidence_dropped"] == 1
        assert "HRI_PRODUCER_LOSS" in {row.get("integrity_reason") for _, row in frames}
    finally:
        follower.close()
