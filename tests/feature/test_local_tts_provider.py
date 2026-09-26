from __future__ import annotations

from types import SimpleNamespace

import pytest

from r2b4_voice.piper_tts import PiperTtsClient, PiperTtsConfig
from r2b4_voice.tts_provider import DEFAULT_PIPER_VOICE, resolve_tts_provider


class FakeVoice:
    def synthesize(self, text: str):
        assert text == "Szia Alba."
        return iter(
            (
                SimpleNamespace(
                    sample_rate=22050,
                    sample_width=2,
                    sample_channels=1,
                    audio_int16_bytes=b"\x01\x00\x02\x00",
                ),
                SimpleNamespace(
                    sample_rate=22050,
                    sample_width=2,
                    sample_channels=1,
                    audio_int16_bytes=b"\x03\x00",
                ),
            )
        )


def test_piper_client_returns_existing_pcm_contract(tmp_path):
    model = tmp_path / f"{DEFAULT_PIPER_VOICE}.onnx"
    model.write_bytes(b"fake-model")
    (tmp_path / f"{DEFAULT_PIPER_VOICE}.onnx.json").write_text("{}", encoding="utf-8")
    client = PiperTtsClient(
        PiperTtsConfig(model_path=model),
        voice_loader=lambda _path: FakeVoice(),
    )

    speech = client.synthesize("  Szia Alba.  ")

    assert speech.pcm == b"\x01\x00\x02\x00\x03\x00"
    assert speech.sample_rate_hz == 22050
    assert speech.channels == 1
    assert speech.sample_width_bytes == 2
    assert speech.model == "piper-tts"
    assert speech.voice == DEFAULT_PIPER_VOICE


def test_local_piper_is_default_provider():
    assert resolve_tts_provider(None) == "piper"
    assert resolve_tts_provider("local") == "piper"


def test_gemini_remains_explicit_optional_provider():
    assert resolve_tts_provider("gemini") == "gemini"
    assert resolve_tts_provider("cloud") == "gemini"


def test_unknown_tts_provider_fails_closed():
    with pytest.raises(ValueError, match="piper or gemini"):
        resolve_tts_provider("unknown")


def test_factory_builds_piper_by_default(monkeypatch, tmp_path):
    from r2b4_voice import tts_provider

    model = tmp_path / f"{DEFAULT_PIPER_VOICE}.onnx"
    monkeypatch.setenv("R2B4_PIPER_MODEL_PATH", str(model))
    monkeypatch.delenv("R2B4_TTS_PROVIDER", raising=False)
    captured = {}

    class StubPiper:
        def __init__(self, config):
            captured["config"] = config

    monkeypatch.setattr(tts_provider, "PiperTtsClient", StubPiper)
    monkeypatch.setattr(tts_provider, "ensure_piper_importable", lambda _path=None: object())
    client = tts_provider.build_tts_client(tmp_path, {})
    assert isinstance(client, StubPiper)
    assert captured["config"].voice == DEFAULT_PIPER_VOICE
    assert captured["config"].model_path == model.resolve()


def test_factory_keeps_gemini_as_explicit_provider(monkeypatch, tmp_path):
    from r2b4_voice import tts_provider

    monkeypatch.setenv("R2B4_TTS_PROVIDER", "gemini")
    captured = {}

    class StubGemini:
        def __init__(self, api_key=None, config=None):
            captured["api_key"] = api_key
            captured["config"] = config

    monkeypatch.setattr(tts_provider, "GeminiTtsClient", StubGemini)
    client = tts_provider.build_tts_client(tmp_path, {}, gemini_api_key="test-key")
    assert isinstance(client, StubGemini)
    assert captured["api_key"] == "test-key"


def test_factory_uses_dedicated_piper_target_from_project_env(monkeypatch, tmp_path):
    from r2b4_voice import tts_provider

    # Isolate this test from the installer's process-level override. Runtime
    # semantics intentionally give a real environment variable higher priority
    # than project_env.
    monkeypatch.delenv("R2B4_PIPER_PYTHON_DIR", raising=False)

    model = tmp_path / "voice.onnx"
    dependency_dir = tmp_path / "piper-python"
    captured = {}

    class StubPiper:
        def __init__(self, config):
            captured["config"] = config

    def fake_import(path=None):
        captured["dependency_dir"] = path
        return object()

    monkeypatch.setattr(tts_provider, "PiperTtsClient", StubPiper)
    monkeypatch.setattr(tts_provider, "ensure_piper_importable", fake_import)
    client = tts_provider.build_tts_client(
        tmp_path,
        {
            "R2B4_PIPER_MODEL_PATH": str(model),
            "R2B4_PIPER_PYTHON_DIR": str(dependency_dir),
        },
    )

    assert isinstance(client, StubPiper)
    assert captured["dependency_dir"] == dependency_dir.resolve()


def test_process_env_overrides_project_env_for_piper_target(monkeypatch, tmp_path):
    from r2b4_voice import tts_provider

    model = tmp_path / "voice.onnx"
    process_dir = tmp_path / "process-piper"
    project_dir = tmp_path / "project-piper"
    captured = {}

    class StubPiper:
        def __init__(self, config):
            captured["config"] = config

    def fake_import(path=None):
        captured["dependency_dir"] = path
        return object()

    monkeypatch.setenv("R2B4_PIPER_PYTHON_DIR", str(process_dir))
    monkeypatch.setattr(tts_provider, "PiperTtsClient", StubPiper)
    monkeypatch.setattr(tts_provider, "ensure_piper_importable", fake_import)

    client = tts_provider.build_tts_client(
        tmp_path,
        {
            "R2B4_PIPER_MODEL_PATH": str(model),
            "R2B4_PIPER_PYTHON_DIR": str(project_dir),
        },
    )

    assert isinstance(client, StubPiper)
    assert captured["dependency_dir"] == process_dir.resolve()
