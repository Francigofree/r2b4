import json

import pytest

from r2b4_voice.groq_llm import GroqChatConfig, GroqStructuredChatClient, LLMRequestError


class Response:
    def __init__(self, payload):
        self._payload = payload
    def read(self):
        return self._payload


def _urlopen_with(content):
    payload = json.dumps({"choices": [{"message": {"content": json.dumps(content)}}]}).encode()
    return lambda request, timeout: Response(payload)


def test_structured_llm_parses_text_only():
    client = GroqStructuredChatClient(
        api_key="secret",
        config=GroqChatConfig(model="openai/gpt-oss-20b"),
        urlopen=_urlopen_with({
            "spoken_text": "Szia.",
            "action_name": None,
            "action_parameters": {"max_v_mps": None, "max_omega_rad_s": None},
        }),
    )
    decision = client.complete([{"role": "user", "content": "Szia"}])
    assert decision.spoken_text == "Szia."
    assert decision.robot_action is None


def test_structured_llm_parses_action():
    client = GroqStructuredChatClient(
        api_key="secret",
        urlopen=_urlopen_with({
            "spoken_text": "Megpróbálok feléd fordulni.",
            "action_name": "v3.command.face_person",
            "action_parameters": {"max_v_mps": None, "max_omega_rad_s": 0.3},
        }),
    )
    decision = client.complete([{"role": "user", "content": "Fordulj felém"}])
    assert decision.robot_action.name == "v3.command.face_person"
    assert decision.robot_action.as_dict() == {"max_omega_rad_s": 0.3}


def test_structured_llm_rejects_parameters_without_action():
    client = GroqStructuredChatClient(
        api_key="secret",
        urlopen=_urlopen_with({
            "spoken_text": "Nem.",
            "action_name": None,
            "action_parameters": {"max_v_mps": 0.1, "max_omega_rad_s": None},
        }),
    )
    with pytest.raises(LLMRequestError):
        client.complete([{"role": "user", "content": "x"}])
