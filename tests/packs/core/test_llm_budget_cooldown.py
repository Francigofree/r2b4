"""Host-only provider admission/backoff evidence; no robot or network calls."""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from r2b4_orchestration.agent_contracts import AgentModelReply
from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker
from r2b4_orchestration.agent_tools import build_default_agent_tools
from r2b4_voice.conversation_contracts import UserTextTurn
from r2b4_voice.groq_llm import GroqStructuredChatClient
from r2b4_voice.llm_failover import FailoverLLMClient, LLMProviderCandidate
from r2b4_voice.prompting import PromptAssembler, PromptBudgetError
from r2b4_voice.robot_context import RobotContextBuilder
from v3.action_catalog import voice_action_descriptors


class _Unavailable:
    model = "offline"

    def __init__(self):
        self.calls = 0

    def complete_agent_step(self, *_args, **_kwargs):
        self.calls += 1
        raise RuntimeError("HTTP 429 quota exceeded")


def _groq_http(calls):
    def request(wire, **_kwargs):
        calls.append(json.loads(wire.data))
        reply = {
            "kind": "final", "spoken_text": "Rendben.", "tool_name": None,
            "tool_arguments_json": None, "action_name": None,
            "action_parameters": {}, "plan_json": None,
        }
        class Response:
            def read(self):
                return json.dumps({"choices": [{"message": {"content": json.dumps(reply)}}],
                                   "usage": {"prompt_tokens": 3000, "completion_tokens": 20}}).encode()
        return Response()
    return request


class _CurrentRobot:
    """Representative stopped launcher state, with large prior task result."""

    def __init__(self):
        self.descriptors = voice_action_descriptors()
        self.constraints = {"distance_m": 0.3, "translation_allowed": True}

    def capabilities(self):
        caps = {item.name: {"kind": "action", "supported": True,
                            "available": True, "ready": True} for item in self.descriptors}
        caps.update({name: {"kind": "read", "supported": True, "available": True}
                     for name in ("operator.status", "behavior.state", "brain.state")})
        return {"schema": "test", "capabilities": caps}

    def read(self, resource):
        if resource == "operator.status":
            return {"runtime_running": False}
        if resource == "behavior.state":
            return {"name": "room_cruise", "lifecycle": "CANCELLED", "revision": 428,
                    "reason": "PREEMPTED_BY:operator.runtime.stop", "measurement_time_ns": 100,
                    "observation_time_ns": 100, "lineage": {"goal_id": "previous", "revision": 428}}
        if resource == "brain.state":
            return {"schema": "R2B4_BRAIN_STATE_V1", "revision": 208,
                    "primary_goal": {"goal_id": "previous", "lifecycle": "CANCELLED",
                                     "constraints": self.constraints, "updated_ns": 100,
                                     "text": "previous task", "steps": [{"action": "behavior.room_cruise"}],
                                     "result": {"diagnostic": "past run" * 10000}},
                    "pending_goals": [{"goal_id": "current", "lifecycle": "PENDING",
                                       "constraints": {}, "updated_ns": 200}],
                    "background_goals": []}
        raise AssertionError(resource)


def test_compact_full_policy_current_request_and_all_actions_fit_groq_fallback(tmp_path):
    robot = _CurrentRobot()
    context = RobotContextBuilder(robot).build()
    assert context.host["brain"]["primary_goal"]["constraints"] == robot.constraints
    assert "result" not in context.host["brain"]["primary_goal"]
    assert context.host["brain"]["primary_goal"]["updated_ns"] == 100
    assert {item["name"] for item in context.available_actions} == {item.name for item in robot.descriptors}
    root = Path(__file__).resolve().parents[3]
    text = "Menj előre 1 métert, majd fordulj balra 185 fokot, és nézz körül."
    messages = PromptAssembler(root / "conf/r2b4_agent_system.md").build_messages(
        UserTextTurn("current", text, "test", 200), context, (),
    )
    primary, secondary = _Unavailable(), _Unavailable()
    calls, events = [], []
    groq = GroqStructuredChatClient(api_key="test", urlopen=_groq_http(calls))
    failover = FailoverLLMClient([
        LLMProviderCandidate("openai_oauth", primary), LLMProviderCandidate("gemini", secondary),
        LLMProviderCandidate("groq", groq),
    ], cooldown_path=tmp_path / "cooldowns.json")
    agent = AgentCore(failover, AgentToolBroker(build_default_agent_tools(tmp_path, interface=robot)))
    assert agent.run(messages, context.available_actions,
                     event_sink=lambda kind, row: events.append((kind, row))).spoken_text == "Rendben."
    assert primary.calls == secondary.calls == len(calls) == 1
    assert calls[0]["messages"][-1] == {"role": "user", "content": text}
    assert calls[0]["max_completion_tokens"] > 0
    names = calls[0]["response_format"]["json_schema"]["schema"]["properties"]["action_name"]["enum"]
    assert set(names) == {None, *(item.name for item in robot.descriptors)}
    completed = next(row for kind, row in events if kind == "agent_llm_completed")
    assert completed["provider"] == "groq" and completed["attempt_count"] == 3
    assert completed["provider_request_token_estimate"] <= completed["provider_request_token_budget"]
    assert completed["max_completion_tokens"] == calls[0]["max_completion_tokens"]
    assert completed["token_estimate_method"].startswith("utf8_bytes/")
    assert completed["input_tokens"] == 3000  # Provider-reported usage, distinct from estimate.


def test_oversize_groq_is_not_sent_retried_or_quarantined(tmp_path):
    calls = []
    groq = GroqStructuredChatClient(api_key="test", urlopen=_groq_http(calls))
    with pytest.raises(PromptBudgetError, match="GROQ_REQUEST_TOKEN_BUDGET_EXCEEDED"):
        groq.complete_agent_step([{"role": "user", "content": "ő" * 20000}], (), ())
    assert calls == []
    class Fallback:
        model = "fallback"
        calls = 0
        def complete_agent_step(self, *_args):
            self.calls += 1
            return AgentModelReply(self.model, spoken_text="fallback")
    fallback = Fallback()
    chain = FailoverLLMClient([LLMProviderCandidate("groq", groq), LLMProviderCandidate("fallback", fallback)],
                              cooldown_path=tmp_path / "cooldowns.json")
    assert chain.complete_agent_step([{"role": "user", "content": "ő" * 20000}], (), ()).spoken_text == "fallback"
    assert fallback.calls == 1 and calls == []
    assert not (tmp_path / "cooldowns.json").exists()
    # A smaller next turn can use Groq immediately; no quota has been consumed.
    assert chain.complete_agent_step([{"role": "user", "content": "Hello"}], (), ()).spoken_text == "Rendben."
    assert len(calls) == 1 and fallback.calls == 1


def test_provider_quota_backoff_survives_new_client_process_and_expires(tmp_path):
    path = tmp_path / "cooldowns.json"
    class Client:
        model = "same-model"
        def __init__(self, result):
            self.result, self.calls = result, 0
        def complete_text(self, _text):
            self.calls += 1
            if isinstance(self.result, Exception):
                raise self.result
            return self.result
    primary, fallback = Client(RuntimeError("HTTP 429 quota")), Client("ok")
    first = FailoverLLMClient([LLMProviderCandidate("primary", primary), LLMProviderCandidate("fallback", fallback)],
                              monotonic=lambda: 100.0, cooldown_path=path)
    assert first.complete_text("one") == "ok" and primary.calls == 1
    # The launcher process dies after every request; prove an independent process
    # observes the same monotonic, boot-scoped backoff file without replaying work.
    script = '''
import json, sys
from pathlib import Path
from r2b4_voice.llm_failover import FailoverLLMClient, LLMProviderCandidate
class Client:
    model = "same-model"
    def __init__(self, text): self.text, self.calls = text, 0
    def complete_text(self, prompt): self.calls += 1; return self.text
p, f = Client("primary"), Client("fallback")
c = FailoverLLMClient([LLMProviderCandidate("primary", p), LLMProviderCandidate("fallback", f)],
                      monotonic=lambda: 100.0, cooldown_path=Path(sys.argv[1]))
print(json.dumps({"result": c.complete_text("two"), "primary_calls": p.calls, "fallback_calls": f.calls}))
'''
    result = subprocess.run([sys.executable, "-c", script, str(path)], check=True, capture_output=True,
                            text=True, timeout=10, cwd=Path(__file__).resolve().parents[3])
    assert json.loads(result.stdout) == {"result": "fallback", "primary_calls": 0, "fallback_calls": 1}
    assert "quota" not in path.read_text()  # Only names/model and deadlines, no error or prompt data.
    ready = Client("restored")
    third = FailoverLLMClient([LLMProviderCandidate("primary", ready)], monotonic=lambda: 401.0,
                              cooldown_path=path)
    assert third.complete_text("three") == "restored" and ready.calls == 1
