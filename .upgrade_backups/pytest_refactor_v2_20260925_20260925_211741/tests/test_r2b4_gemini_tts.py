import base64
import json

import pytest

from r2b4_voice.gemini_tts import GeminiTtsClient, GeminiTtsConfig, GeminiTtsError


class Response:
    def __init__(self, payload):
        self._payload = payload
    def read(self):
        return self._payload


def test_gemini_tts_parses_pcm_and_sends_audio_request():
    seen = {}
    pcm = (123).to_bytes(2, "little", signed=True) * 240
    payload = json.dumps({
        "candidates": [{
            "content": {"parts": [{"inlineData": {
                "mimeType": "audio/L16;codec=pcm;rate=24000",
                "data": base64.b64encode(pcm).decode("ascii"),
            }}]}
        }]
    }).encode()

    def urlopen(request, timeout):
        seen["url"] = request.full_url
        seen["body"] = json.loads(request.data.decode("utf-8"))
        seen["timeout"] = timeout
        return Response(payload)

    client = GeminiTtsClient(
        api_key="secret",
        config=GeminiTtsConfig(model="gemini-3.1-flash-tts-preview", voice="Kore"),
        urlopen=urlopen,
    )
    speech = client.synthesize("Szia Alba")
    assert speech.pcm == pcm
    assert speech.sample_rate_hz == 24000
    assert speech.voice == "Kore"
    assert "gemini-3.1-flash-tts-preview" in seen["url"]
    generation = seen["body"]["generationConfig"]
    assert generation["responseModalities"] == ["AUDIO"]
    assert generation["speechConfig"]["voiceConfig"]["prebuiltVoiceConfig"]["voiceName"] == "Kore"


def test_gemini_tts_rejects_missing_audio():
    payload = json.dumps({"candidates": [{"content": {"parts": [{"text": "no audio"}]}}]}).encode()
    client = GeminiTtsClient(api_key="secret", urlopen=lambda request, timeout: Response(payload))
    with pytest.raises(GeminiTtsError):
        client.synthesize("Szia")
