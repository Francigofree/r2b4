import json

import pytest

from r2b4_voice.gemini_llm import GeminiChatConfig, GeminiRequestError, GeminiStructuredChatClient


class Response:
    def __init__(self, payload):
        self._payload = payload
    def read(self):
        return self._payload


def _response(decision):
    payload = json.dumps({
        "id": "int_test",
        "status": "completed",
        "steps": [{
            "type": "model_output",
            "content": [{"type": "text", "text": json.dumps(decision)}],
        }],
    }).encode()
    return Response(payload)


def test_gemini_uses_stateless_interactions_structured_output():
    captured = {}
    def urlopen(request, timeout):
        captured["url"] = request.full_url
        captured["headers"] = dict(request.header_items())
        captured["body"] = json.loads(request.data.decode())
        captured["timeout"] = timeout
        return _response({
            "spoken_text": "A robot vezérlő runtime jelenleg nem fut.",
            "action_name": None,
            "action_parameters": {"max_v_mps": None, "max_omega_rad_s": None},
        })

    client = GeminiStructuredChatClient(
        api_key="secret",
        config=GeminiChatConfig(model="gemini-3.8-flash"),
        urlopen=urlopen,
    )
    decision = client.complete([
        {"role": "system", "content": "SYSTEM"},
        {"role": "system", "content": "CONTEXT"},
        {"role": "user", "content": "Szia"},
        {"role": "assistant", "content": "Szia"},
        {"role": "user", "content": "Mi az állapotod?"},
    ])

    assert decision.spoken_text == "A robot vezérlő runtime jelenleg nem fut."
    assert decision.robot_action is None
    assert captured["url"].endswith("/v1beta/interactions")
    assert captured["body"]["store"] is False
    assert captured["body"]["model"] == "gemini-3.8-flash"
    assert captured["body"]["response_format"]["mime_type"] == "application/json"
    assert captured["body"]["generation_config"]["thinking_level"] == "low"
    assert captured["body"]["system_instruction"] == "SYSTEM\n\nCONTEXT"
    assert captured["body"]["input"] == (
        "USER:\nSzia\n\nASSISTANT:\nSzia\n\nUSER:\nMi az állapotod?"
    )
    assert captured["headers"]["X-goog-api-key"] == "secret"


def test_gemini_parses_shadow_action():
    client = GeminiStructuredChatClient(
        api_key="secret",
        urlopen=lambda request, timeout: _response({
            "spoken_text": "Megpróbálok feléd fordulni.",
            "action_name": "v3.command.face_person",
            "action_parameters": {"max_v_mps": None, "max_omega_rad_s": 0.3},
        }),
    )
    decision = client.complete([{"role": "user", "content": "Fordulj felém"}])
    assert decision.robot_action.name == "v3.command.face_person"
    assert decision.robot_action.as_dict() == {"max_omega_rad_s": 0.3}


def test_gemini_rejects_invalid_decision():
    client = GeminiStructuredChatClient(
        api_key="secret",
        urlopen=lambda request, timeout: _response({
            "spoken_text": "x",
            "action_name": "v3.command.wheels",
            "action_parameters": {"max_v_mps": None, "max_omega_rad_s": None},
        }),
    )
    with pytest.raises(GeminiRequestError):
        client.complete([{"role": "user", "content": "x"}])
