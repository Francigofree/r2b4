from __future__ import annotations

import json

import pytest

from r2b4_voice.openai_llm import (
    OpenAIChatConfig,
    OpenAIRequestError,
    OpenAIResponsesChatClient,
)


class FakeResponse:
    def __init__(self, payload: object) -> None:
        self._payload = json.dumps(payload).encode("utf-8")

    def read(self) -> bytes:
        return self._payload


def test_openai_agent_step_uses_responses_structured_output() -> None:
    seen: dict[str, object] = {}

    raw = {
        "kind": "final",
        "spoken_text": "rendben",
        "tool_name": None,
        "tool_arguments_json": None,
        "action_name": None,
        "action_parameters": {},
    }
    envelope = {
        "status": "completed",
        "output": [
            {
                "type": "message",
                "content": [{"type": "output_text", "text": json.dumps(raw)}],
            }
        ],
    }

    def fake_urlopen(request, timeout):
        seen["url"] = request.full_url
        seen["headers"] = dict(request.header_items())
        seen["body"] = json.loads(request.data.decode("utf-8"))
        seen["timeout"] = timeout
        return FakeResponse(envelope)

    client = OpenAIResponsesChatClient(
        api_key="test-key",
        config=OpenAIChatConfig(model="gpt-5.6", reasoning_effort="low"),
        urlopen=fake_urlopen,
    )
    reply = client.complete_agent_step(
        [{"role": "system", "content": "system"}, {"role": "user", "content": "teszt"}],
        (),
        (),
    )

    assert reply.spoken_text == "rendben"
    assert seen["url"] == "https://api.openai.com/v1/responses"
    body = seen["body"]
    assert body["model"] == "gpt-5.6"
    assert body["store"] is False
    assert body["reasoning"] == {"effort": "low"}
    assert body["text"]["format"]["type"] == "json_schema"
    assert body["text"]["format"]["strict"] is True
    assert body["text"]["format"]["schema"]["additionalProperties"] is False
    assert body["input"][0]["role"] == "system"
    auth = {str(k).lower(): v for k, v in seen["headers"].items()}
    assert auth["authorization"] == "Bearer test-key"


def test_openai_refusal_is_fail_closed() -> None:
    envelope = {
        "status": "completed",
        "output": [
            {
                "type": "message",
                "content": [{"type": "refusal", "refusal": "not available"}],
            }
        ],
    }

    def fake_urlopen(_request, timeout):
        assert timeout > 0
        return FakeResponse(envelope)

    client = OpenAIResponsesChatClient(api_key="test-key", urlopen=fake_urlopen)
    with pytest.raises(OpenAIRequestError, match="refused"):
        client.complete_agent_step([{"role": "user", "content": "teszt"}], (), ())
