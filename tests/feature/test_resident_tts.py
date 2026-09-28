from __future__ import annotations

import threading
import time
from pathlib import Path
from types import SimpleNamespace


def test_resident_tts_protocol_roundtrip(tmp_path):
    from r2b4_voice.gemini_tts import GeneratedSpeech
    from r2b4_voice.piper_tts import PiperTtsConfig
    from r2b4_voice.resident_tts import ResidentPiperTtsClient
    from r2b4_voice.tts_daemon import _serve

    model = tmp_path / "voice.onnx"
    model.write_bytes(b"fake")
    (tmp_path / "voice.onnx.json").write_text("{}", encoding="utf-8")
    socket_path = tmp_path / "tts.sock"

    class FakeTts:
        model = "piper-tts"
        voice = "hu-test"

        def synthesize(self, text):
            assert text == "Szia."
            return GeneratedSpeech(
                pcm=b"\x01\x00\x02\x00",
                sample_rate_hz=22050,
                channels=1,
                sample_width_bytes=2,
                model=self.model,
                voice=self.voice,
            )

    server = threading.Thread(target=_serve, args=(socket_path, FakeTts()), daemon=True)
    server.start()
    deadline = time.monotonic() + 2.0
    while not socket_path.exists() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert socket_path.exists()

    client = ResidentPiperTtsClient(
        PiperTtsConfig(model_path=model, voice="hu-test"),
        project_root=tmp_path,
        dependency_dir=tmp_path,
    )
    client._socket_path = socket_path

    assert client.warmup() is True
    speech = client.synthesize("Szia.")
    assert speech.pcm == b"\x01\x00\x02\x00"
    assert speech.sample_rate_hz == 22050
    assert speech.model == "piper-tts"
    assert speech.voice == "hu-test"


def test_tts_factory_resident_mode_does_not_load_piper_in_caller(monkeypatch, tmp_path):
    from r2b4_voice import tts_provider

    model = tmp_path / "voice.onnx"
    dependency_dir = tmp_path / "piper-python"
    captured = {}

    class StubResident:
        def __init__(self, config, *, project_root, dependency_dir):
            captured["config"] = config
            captured["project_root"] = project_root
            captured["dependency_dir"] = dependency_dir

    monkeypatch.setenv("R2B4_PIPER_MODEL_PATH", str(model))
    monkeypatch.setenv("R2B4_PIPER_PYTHON_DIR", str(dependency_dir))
    monkeypatch.delenv("R2B4_TTS_PROVIDER", raising=False)
    monkeypatch.setattr(tts_provider, "ResidentPiperTtsClient", StubResident)
    monkeypatch.setattr(
        tts_provider,
        "ensure_piper_importable",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("resident caller must not import/load Piper")
        ),
    )

    client = tts_provider.build_tts_client(tmp_path, {}, resident=True)

    assert isinstance(client, StubResident)
    assert captured["config"].model_path == model.resolve()
    assert captured["project_root"] == tmp_path.resolve()
    assert captured["dependency_dir"] == dependency_dir.resolve()


def test_plain_prompt_warms_tts_during_llm_and_speaks_bounded_chunks(tmp_path):
    from r2b4_voice.plain_llm import run_plain_prompt

    order = []
    warm_started = threading.Event()
    answer = ("Első rövid mondat. Második rövid mondat. " * 30).strip()
    synthesized = []

    class Client:
        def complete_text(self, prompt):
            assert prompt == "kérdés"
            assert warm_started.wait(1.0), "TTS warmup must start before Gemini completes"
            order.append("llm")
            return answer

    class Tts:
        model = "piper-tts"
        voice = "test"

        def warmup(self):
            order.append("warmup")
            warm_started.set()
            return True

        def synthesize(self, text):
            assert 0 < len(text) <= 320
            synthesized.append(text)
            return SimpleNamespace(text=text)

    class Player:
        def play(self, speech):
            order.append("play")
            return "fake"

    rc = run_plain_prompt(
        "kérdés",
        project_root=tmp_path,
        client=Client(),
        tts=Tts(),
        player=Player(),
    )

    assert rc == 0
    assert order.index("warmup") < order.index("llm") < order.index("play")
    assert len(synthesized) > 1
    assert " ".join(synthesized).replace("  ", " ") == answer
