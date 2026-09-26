from __future__ import annotations

from types import SimpleNamespace

from r2b4_er2 import speech as speech_module


class FakeTts:
    model = "piper-tts"
    voice = "hu_HU-anna-medium"

    def synthesize(self, text: str):
        return SimpleNamespace(model=self.model, voice=self.voice, text=text)


class FakePlayer:
    def play(self, speech):
        assert speech.text == "Teszt."
        return "pw-play"


def test_er2_speech_uses_shared_default_tts_factory(monkeypatch):
    created = []

    def fake_build(*, gemini_api_key=None):
        created.append(gemini_api_key)
        return FakeTts()

    monkeypatch.setattr(speech_module, "build_tts_client", fake_build)
    reporter = speech_module.Er2SpeechReporter(player=FakePlayer())

    result = reporter.speak("Teszt.")

    assert created == [None]
    assert result["model"] == "piper-tts"
    assert result["voice"] == "hu_HU-anna-medium"
