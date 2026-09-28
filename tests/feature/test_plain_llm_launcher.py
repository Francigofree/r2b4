from __future__ import annotations

import io
import json


def test_plain_gemini_client_sends_unstructured_interaction():
    from r2b4_voice.gemini_llm import GeminiChatConfig
    from r2b4_voice.plain_llm import PlainGeminiClient

    seen = {}

    class Response:
        def read(self):
            return json.dumps(
                {
                    "steps": [
                        {"type": "thought", "summary": []},
                        {
                            "type": "model_output",
                            "content": [{"type": "text", "text": "Sima válasz."}],
                        },
                    ]
                }
            ).encode("utf-8")

    def fake_urlopen(request, timeout):
        seen["url"] = request.full_url
        seen["timeout"] = timeout
        seen["headers"] = dict(request.header_items())
        seen["body"] = json.loads(request.data.decode("utf-8"))
        return Response()

    client = PlainGeminiClient(
        api_key="test-key",
        config=GeminiChatConfig(model="gemini-3.5-flash-lite", thinking_level="low"),
        urlopen=fake_urlopen,
    )

    assert client.complete_text("Miért kék az ég?") == "Sima válasz."
    assert seen["body"] == {
        "model": "gemini-3.5-flash-lite",
        "input": "Miért kék az ég?",
        "store": False,
        "generation_config": {"thinking_level": "low"},
    }
    assert "response_format" not in seen["body"]
    assert seen["headers"]["X-goog-api-key"] == "test-key"


def test_run_plain_prompt_prints_and_speaks_full_long_answer(tmp_path):
    from r2b4_voice.plain_llm import run_plain_prompt

    answer = ("Ez egy hosszabb mondat. " * 120).strip()
    synthesized = []
    played = []

    class Client:
        def complete_text(self, prompt):
            assert prompt == "kérdés"
            return answer

    class Tts:
        def synthesize(self, text):
            assert 0 < len(text) <= 1800
            synthesized.append(text)
            return {"speech": text}

    class Player:
        def play(self, speech):
            played.append(speech)
            return "fake"

    output = io.StringIO()
    rc = run_plain_prompt(
        "kérdés",
        project_root=tmp_path,
        client=Client(),
        tts=Tts(),
        player=Player(),
        stdout=output,
    )

    assert rc == 0
    assert output.getvalue().strip() == answer
    assert " ".join(synthesized).replace("  ", " ") == answer
    assert len(synthesized) >= 2
    assert len(played) == len(synthesized)


def test_launcher_unknown_text_routes_to_plain_llm(monkeypatch, tmp_path):
    from v3 import launcher_cli, runtime_performance

    called = {}
    monkeypatch.setattr(launcher_cli, "project_root", lambda: tmp_path)
    monkeypatch.setattr(runtime_performance, "apply_host_affinity", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(launcher_cli, "_robot_catalog", lambda: [])

    def fake_plain(argv, root):
        called["argv"] = list(argv)
        called["root"] = root
        return 23

    monkeypatch.setattr(launcher_cli, "_plain_prompt", fake_plain)

    assert launcher_cli.main(["Miért kék az ég?"]) == 23
    assert called == {"argv": ["Miért kék az ég?"], "root": tmp_path}


def test_launcher_double_dash_forces_plain_prompt_even_for_known_command(monkeypatch, tmp_path):
    from v3 import launcher_cli, runtime_performance

    called = {}
    monkeypatch.setattr(launcher_cli, "project_root", lambda: tmp_path)
    monkeypatch.setattr(runtime_performance, "apply_host_affinity", lambda *_args, **_kwargs: None)

    def fake_plain(argv, root):
        called["argv"] = list(argv)
        called["root"] = root
        return 29

    monkeypatch.setattr(launcher_cli, "_plain_prompt", fake_plain)

    assert launcher_cli.main(["--", "s"]) == 29
    assert called == {"argv": ["s"], "root": tmp_path}


def test_launcher_known_robot_command_does_not_route_to_plain(monkeypatch, tmp_path):
    from v3 import launcher_cli, runtime_performance

    monkeypatch.setattr(launcher_cli, "project_root", lambda: tmp_path)
    monkeypatch.setattr(runtime_performance, "apply_host_affinity", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(launcher_cli, "_robot_catalog", lambda: [{"name": "s"}])
    monkeypatch.setattr(launcher_cli.interface_cli, "ALIASES", {})
    monkeypatch.setattr(launcher_cli.interface_cli, "main", lambda *_args, **_kwargs: 31)
    monkeypatch.setattr(
        launcher_cli,
        "_plain_prompt",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("plain path must not run")),
    )

    assert launcher_cli.main(["s"]) == 31
