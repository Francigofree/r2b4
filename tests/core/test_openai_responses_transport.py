from __future__ import annotations

import json

from r2b4_voice.openai_llm import OpenAIChatConfig, OpenAIResponsesChatClient


class SSE:
    def __init__(self, events):
        self._lines = []
        for event in events:
            self._lines.extend([f"data: {json.dumps(event)}\n".encode(), b"\n"])

    def __iter__(self):
        return iter(self._lines)


def test_oauth_compatible_streaming_structured_request() -> None:
    seen = {}
    raw = {
        "kind": "final",
        "spoken_text": "rendben",
        "tool_name": None,
        "tool_arguments_json": None,
        "action_name": None,
        "action_parameters": {},
    }

    def fake_urlopen(request, timeout):
        seen["body"] = json.loads(request.data.decode())
        seen["headers"] = {str(k).lower(): v for k, v in request.header_items()}
        seen["timeout"] = timeout
        text = json.dumps(raw)
        return SSE(
            [
                {"type": "response.output_text.delta", "delta": text[:10]},
                {"type": "response.output_text.delta", "delta": text[10:]},
                {"type": "response.completed", "response": {"status": "completed", "output": []}},
            ]
        )

    client = OpenAIResponsesChatClient(
        api_key="test-key",
        config=OpenAIChatConfig(model="gpt-5.6"),
        urlopen=fake_urlopen,
    )
    reply = client.complete_agent_step(
        [{"role": "system", "content": "system rules"}, {"role": "user", "content": "teszt"}],
        (),
        (),
    )
    assert reply.spoken_text == "rendben"
    body = seen["body"]
    assert body["store"] is False
    assert body["stream"] is True
    assert body["instructions"] == "system rules"
    assert body["input"] == [{"role": "user", "content": "teszt"}]
    assert body["text"]["format"]["strict"] is True
    assert seen["headers"]["authorization"] == "Bearer test-key"
