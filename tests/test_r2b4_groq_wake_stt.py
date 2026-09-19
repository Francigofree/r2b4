from __future__ import annotations

import json

from r2b4_voice.groq_stt import GroqWakeTranscriber
from r2b4_voice.wake_core import WakeUtterance


class _Response:
    def read(self):
        return json.dumps({"text": "Alba"}).encode("utf-8")


def test_groq_wake_request_is_hungarian_whisper_and_contains_wav():
    captured = {}

    def fake_urlopen(request, timeout):
        captured["request"] = request
        captured["timeout"] = timeout
        return _Response()

    utterance = WakeUtterance(
        first_sequence=1,
        last_sequence=2,
        started_monotonic_ns=0,
        ended_monotonic_ns=40_000_000,
        sample_rate_hz=48_000,
        sample_count=1920,
        pcm=b"\x00\x00" * 1920,
    )
    transcriber = GroqWakeTranscriber(api_key="secret", urlopen=fake_urlopen)
    assert transcriber.transcribe(utterance) == "Alba"
    request = captured["request"]
    assert request.get_header("Authorization") == "Bearer secret"
    assert request.full_url.endswith("/audio/transcriptions")
    body = request.data
    assert b"whisper-large-v3-turbo" in body
    assert b'name="language"' in body and b"hu" in body
    assert b"RIFF" in body and b"WAVE" in body
