from __future__ import annotations

import asyncio
from types import SimpleNamespace

from r2b4_er2 import cli as er2_cli
from r2b4_er2.config import Er2Config
from r2b4_er2.preview import Er2PreviewClient
from r2b4_er2.streaming import Er2StreamingClient
from v3.launcher_cli import command_catalog


class _Evidence:
    def __init__(self) -> None:
        self.events: list[tuple[str, dict[str, object]]] = []

    def emit(self, event_type: str, **fields: object):
        self.events.append((event_type, dict(fields)))


class _PreviewInteractions:
    def __init__(self) -> None:
        self.calls = 0

    def create(self, **kwargs):
        self.calls += 1
        if self.calls == 1:
            return SimpleNamespace(
                id="interaction-1",
                steps=(SimpleNamespace(type="function_call", name="robot_status", arguments={}, id="call-1"),),
                output_text=None,
            )
        return SimpleNamespace(id="interaction-2", steps=(), output_text="ok")


class _PreviewSdk:
    def __init__(self) -> None:
        self.interactions = _PreviewInteractions()


class _PreviewTools:
    def interaction_tools(self):
        return [{"type": "function", "name": "robot_status", "parameters": {"type": "object", "properties": {}}}]

    def execute(self, name, arguments):
        assert name == "robot_status"
        assert dict(arguments) == {}
        return {"status": "COMPLETED"}


class _StreamTools:
    config = Er2Config()

    def robot_stop(self):
        return {"status": "STOPPED"}


class _StreamMedia:
    pass


class _ReceiveSession:
    def __init__(self) -> None:
        self.calls = 0

    def receive(self):
        async def _gen():
            self.calls += 1
            if self.calls == 1:
                yield SimpleNamespace(
                    session_resumption_update=None,
                    go_away=None,
                    server_content=SimpleNamespace(
                        model_turn=SimpleNamespace(parts=(SimpleNamespace(text="hello"),)),
                        turn_complete=True,
                    ),
                    tool_call=None,
                )
        return _gen()


def test_bare_stream_defaults_enable_everything() -> None:
    args = er2_cli._parser().parse_args(["stream", "nézz körül"])
    assert args.camera is True
    assert args.tools is True
    assert args.speak is True
    assert args.json is True
    assert command_catalog()["er2"]["stream_defaults"] == ["camera", "tools", "speak", "json"]


def test_preview_communication_is_event_logged_without_raw_prompt_or_output() -> None:
    evidence = _Evidence()
    result = Er2PreviewClient(Er2Config(), client=_PreviewSdk(), evidence=evidence).run(
        "secret prompt text",
        tools=_PreviewTools(),
    )
    assert result.text == "ok"
    names = [name for name, _ in evidence.events]
    assert names == [
        "ER2_PREVIEW_START",
        "ER2_PREVIEW_REQUEST_TX",
        "ER2_PREVIEW_RESPONSE_RX",
        "ER2_PREVIEW_TOOL_CALL_RX",
        "ER2_PREVIEW_TOOL_RESULTS_TX",
        "ER2_PREVIEW_RESPONSE_RX",
        "ER2_PREVIEW_COMPLETE",
    ]
    assert all("prompt" not in fields and "text" not in fields for _, fields in evidence.events)


def test_stream_receive_text_and_turn_completion_are_event_logged_as_metadata() -> None:
    async def scenario() -> None:
        evidence = _Evidence()
        chunks: list[str] = []
        client = Er2StreamingClient(
            _StreamTools(),
            _StreamMedia(),
            Er2Config(),
            client=object(),
            on_text=chunks.append,
            evidence=evidence,
        )
        await client._receive_loop(
            _ReceiveSession(),
            object(),
            asyncio.Event(),
            asyncio.Event(),
            asyncio.Event(),
        )
        assert chunks == ["hello"]
        names = [name for name, _ in evidence.events]
        assert names == ["ER2_STREAM_TEXT_RX", "ER2_STREAM_TURN_COMPLETE_RX"]
        assert evidence.events[0][1]["chars"] == 5
        assert "text" not in evidence.events[0][1]

    asyncio.run(scenario())
