from __future__ import annotations

import json

import pytest

from r2b4_voice.gemini_llm import GeminiChatConfig, GeminiRequestError, GeminiStructuredChatClient


class _Response:
    def __init__(self, payload: object) -> None:
        self._payload = json.dumps(payload).encode("utf-8")

    def read(self) -> bytes:
        return self._payload


def test_gemini_25_agent_structured_output_uses_generate_content_json_schema() -> None:
    seen: dict[str, object] = {}

    def fake_urlopen(request, timeout):
        seen["url"] = request.full_url
        seen["timeout"] = timeout
        seen["headers"] = dict(request.header_items())
        seen["body"] = json.loads(request.data.decode("utf-8"))
        return _Response({
            "candidates": [{
                "content": {
                    "role": "model",
                    "parts": [
                        {"thought": True, "text": "hidden summary that must not be parsed"},
                        {"text": json.dumps({
                            "kind": "final",
                            "spoken_text": "kész",
                            "tool_name": None,
                            "tool_arguments_json": None,
                            "action_name": None,
                            "action_parameters": {},
                        })},
                    ],
                },
                "finishReason": "STOP",
            }]
        })

    client = GeminiStructuredChatClient(
        api_key="test-key",
        config=GeminiChatConfig(model="gemini-2.5-flash", thinking_level="low"),
        urlopen=fake_urlopen,
    )
    reply = client.complete_agent_step(
        [
            {"role": "system", "content": "system contract"},
            {"role": "user", "content": "teszt"},
        ],
        (),
        (),
    )

    assert reply.spoken_text == "kész"
    assert seen["url"] == (
        "https://generativelanguage.googleapis.com/v1beta/models/"
        "gemini-2.5-flash:generateContent"
    )
    body = seen["body"]
    assert isinstance(body, dict)
    assert "model" not in body
    assert body["systemInstruction"] == {"parts": [{"text": "system contract"}]}
    assert body["contents"] == [{"role": "user", "parts": [{"text": "USER:\nteszt"}]}]
    generation = body["generationConfig"]
    assert generation["responseMimeType"] == "application/json"
    assert generation["responseJsonSchema"]["type"] == "object"
    assert generation["thinkingConfig"] == {"thinkingBudget": 0}


def test_gemini_structured_non_json_output_has_diagnostic_prefix() -> None:
    def fake_urlopen(_request, *, timeout):
        assert timeout == 20.0
        return _Response({
            "candidates": [{
                "content": {"parts": [{"text": "Ez nem JSON."}]},
                "finishReason": "STOP",
            }]
        })

    client = GeminiStructuredChatClient(api_key="test-key", urlopen=fake_urlopen)
    with pytest.raises(GeminiRequestError, match="structured output was not JSON") as caught:
        client.complete_agent_step([{"role": "user", "content": "teszt"}], (), ())
    assert "Ez nem JSON." in str(caught.value)


def test_gemini_3_uses_thinking_level_on_generate_content() -> None:
    client = GeminiStructuredChatClient(
        api_key="test-key",
        config=GeminiChatConfig(model="gemini-3.8-flash", thinking_level="medium"),
        urlopen=lambda *_args, **_kwargs: None,
    )
    assert client._thinking_config() == {"thinkingLevel": "medium"}
