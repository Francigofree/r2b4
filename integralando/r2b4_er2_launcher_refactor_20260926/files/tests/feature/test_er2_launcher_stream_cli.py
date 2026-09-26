from __future__ import annotations

import json
from types import SimpleNamespace

from r2b4_er2 import cli as er2_cli
from v3.launcher_cli import _normalize_er2_args


def test_er2_shorthand_defaults_to_stream_and_preserves_options() -> None:
    assert _normalize_er2_args(["er2", "fordulj 90 fokot"]) == [
        "er2", "stream", "fordulj 90 fokot",
    ]
    assert _normalize_er2_args([
        "er2", "fordulj 90 fokot", "--camera", "--tools", "--speak", "--json",
    ]) == [
        "er2", "stream", "fordulj 90 fokot", "--camera", "--tools", "--speak", "--json",
    ]
    assert _normalize_er2_args(["er2", "preview", "mit látsz?"]) == [
        "er2", "preview", "mit látsz?",
    ]


def test_stream_parser_accepts_full_launcher_surface() -> None:
    args = er2_cli._parser().parse_args([
        "stream",
        "fordulj 90 fokot",
        "--camera",
        "--tools",
        "--speak",
        "--json",
        "--seconds",
        "3",
    ])
    assert args.command == "stream"
    assert args.task == "fordulj 90 fokot"
    assert args.camera is True
    assert args.tools is True
    assert args.speak is True
    assert args.json is True
    assert args.seconds == 3.0


def test_stream_json_and_speech_use_same_stream_text(monkeypatch, tmp_path, capsys) -> None:
    stops: list[str] = []
    spoken: list[str] = []

    class Interface:
        def read(self, resource: str):
            if resource == "operator.status":
                return {"runtime_running": True}
            return {}

        def execute(self, action: str, **kwargs):
            raise AssertionError(f"unexpected execute: {action}")

    tools = SimpleNamespace(robot_stop=lambda: stops.append("STOP"))
    evidence = SimpleNamespace(emit=lambda *args, **kwargs: None)

    class FakeStreamingClient:
        def __init__(self, _tools, _media, _cfg, *, on_text, evidence):
            self.on_text = on_text

        def run(self, task: str, *, duration_s=None):
            assert task == "fordulj 90 fokot"
            self.on_text("Elfordultam ")
            self.on_text("90 fokot.")
            return SimpleNamespace(
                reconnect_count=0,
                latest_resumption_handle="resume-1",
                stopped_cleanly=True,
            )

    class FakeSpeechReporter:
        def __init__(self, *, evidence):
            pass

        def speak(self, text: str):
            spoken.append(text)
            return {"status": "ok"}

    monkeypatch.setattr(er2_cli, "RobotInterface", lambda **kwargs: Interface())
    monkeypatch.setattr(er2_cli.Er2RobotTools, "from_interface", lambda *args, **kwargs: tools)
    monkeypatch.setattr(er2_cli.Er2Evidence, "from_project_root", lambda root: evidence)
    monkeypatch.setattr(er2_cli, "VisionMediaClient", lambda **kwargs: object())
    monkeypatch.setattr(er2_cli, "Er2StreamingClient", FakeStreamingClient)
    monkeypatch.setattr(er2_cli, "Er2SpeechReporter", FakeSpeechReporter)

    rc = er2_cli.main([
        "stream",
        "fordulj 90 fokot",
        "--camera",
        "--tools",
        "--speak",
        "--json",
    ], project_root=tmp_path)

    assert rc == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["mode"] == "stream"
    assert payload["text"] == "Elfordultam 90 fokot."
    assert payload["camera"] is True
    assert payload["tools"] is True
    assert payload["stopped_cleanly"] is True
    assert spoken == ["Elfordultam 90 fokot."]
    assert stops == ["STOP"]
