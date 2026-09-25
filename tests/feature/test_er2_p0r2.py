from __future__ import annotations

import asyncio
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from r2b4_er2.cli import _probe_media
from r2b4_er2.config import Er2Config
from r2b4_er2.evidence import Er2Evidence
from r2b4_er2.media import Er2MediaUnavailable
from r2b4_er2.preview import Er2PreviewClient
from r2b4_er2.speech import Er2SpeechReporter
from r2b4_er2.streaming import Er2StreamingClient
from r2b4_er2.tool_bridge import Er2RobotTools
from v3.external_gateway import ExternalRobotGateway, GatewayPolicy
from v3.hri_evidence import HRI_EVENT_SCHEMA, HriEventJournal

pytestmark = pytest.mark.providers


class _FakeInteractions:
    def __init__(self) -> None:
        self.calls: list[dict[str, object]] = []

    def create(self, **kwargs):
        self.calls.append(dict(kwargs))
        if len(self.calls) == 1:
            return SimpleNamespace(
                id="interaction-1",
                steps=(SimpleNamespace(type="function_call", name="robot_status", arguments={}, id="call-1"),),
                output_text=None,
            )
        return SimpleNamespace(id="interaction-2", steps=(), output_text="preview-ok")


class _FakePreviewSdk:
    def __init__(self) -> None:
        self.interactions = _FakeInteractions()


class _FakePreviewTools:
    def interaction_tools(self):
        return [{"type": "function", "name": "robot_status", "parameters": {"type": "object", "properties": {}}}]

    def execute(self, name, arguments):
        assert name == "robot_status"
        assert dict(arguments) == {}
        return {"status": "COMPLETED", "answer": 7}


def test_preview_tool_result_repeats_interaction_scoped_tools_and_content_blocks() -> None:
    sdk = _FakePreviewSdk()
    tools = _FakePreviewTools()
    result = Er2PreviewClient(Er2Config(), client=sdk).run("check", tools=tools)

    assert result.text == "preview-ok"
    assert result.tool_rounds == 1
    assert len(sdk.interactions.calls) == 2
    second = sdk.interactions.calls[1]
    assert second["previous_interaction_id"] == "interaction-1"
    assert second["tools"] == tools.interaction_tools()
    assert second["generation_config"] == {"thinking_level": "high"}
    row = second["input"][0]
    assert row["type"] == "function_result"
    assert row["result"][0]["type"] == "text"
    assert json.loads(row["result"][0]["text"])["answer"] == 7


class _FakeLiveTypes:
    class Blob:
        def __init__(self, *, data, mime_type):
            self.data = data
            self.mime_type = mime_type


class _FakeHeartbeatSession:
    def __init__(self) -> None:
        self.calls: list[dict[str, object]] = []

    async def send_realtime_input(self, **kwargs):
        self.calls.append(dict(kwargs))


class _FakeHeartbeatTools:
    config = Er2Config(heartbeat_s=0.90)

    def robot_status(self):
        return {"status": "COMPLETED"}

    def robot_stop(self):
        return {"status": "STOPPED"}


class _FakeMedia:
    async def latest_jpeg(self, *, stream_name="lores"):
        assert stream_name == "lores"
        return b"\xff\xd8x\xff\xd9"


def test_streaming_heartbeat_waits_initially_but_resumed_connection_sends_immediately() -> None:
    async def scenario() -> None:
        client = Er2StreamingClient(_FakeHeartbeatTools(), _FakeMedia(), Er2Config(heartbeat_s=0.90), client=object())

        initial_session = _FakeHeartbeatSession()
        initial_turn_done = asyncio.Event()
        stop = asyncio.Event()
        task = asyncio.create_task(
            client._heartbeat_loop(
                initial_session,
                _FakeLiveTypes,
                initial_turn_done,
                stop,
                wait_for_initial_turn=True,
            )
        )
        await asyncio.sleep(0.02)
        assert initial_session.calls == []
        initial_turn_done.set()
        for _ in range(50):
            if len(initial_session.calls) >= 2:
                break
            await asyncio.sleep(0.002)
        assert [set(call) for call in initial_session.calls[:2]] == [{"video"}, {"text"}]
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)

        resumed_session = _FakeHeartbeatSession()
        resumed_turn_done = asyncio.Event()
        task = asyncio.create_task(
            client._heartbeat_loop(
                resumed_session,
                _FakeLiveTypes,
                resumed_turn_done,
                stop,
                wait_for_initial_turn=False,
            )
        )
        for _ in range(50):
            if len(resumed_session.calls) >= 2:
                break
            await asyncio.sleep(0.002)
        assert [set(call) for call in resumed_session.calls[:2]] == [{"video"}, {"text"}]
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)

    asyncio.run(scenario())


class _ReadyMedia:
    socket_path = Path("/tmp/ready.sock")

    def latest_jpeg_sync(self, *, stream_name="lores"):
        assert stream_name == "lores"
        return b"\xff\xd8abc\xff\xd9"


class _MissingMedia:
    socket_path = Path("/tmp/missing.sock")

    def latest_jpeg_sync(self, *, stream_name="lores"):
        raise Er2MediaUnavailable("not running")


def test_status_media_probe_reports_real_jpeg_probe_and_failure() -> None:
    ready = _probe_media(_ReadyMedia())
    missing = _probe_media(_MissingMedia())
    assert ready["available"] is True and ready["jpeg_bytes"] == 7
    assert missing["available"] is False and "not running" in str(missing["error"])


class _FakeInterface:
    def capabilities(self):
        return {"schema": "fake", "capabilities": {}}

    def read(self, resource: str):
        if resource == "v3.status":
            return {"state": "RUNNING", "ready_for_active": True}
        if resource == "operator.status":
            return {"runtime_running": True, "capture_mode": "full", "capture_hz": 10}
        if resource == "v3.safety":
            return {"state": "RUNNING", "decision": "ALLOW"}
        raise RuntimeError(resource)

    def execute(self, action: str, **parameters: object):
        return {"command_id": "cmd-1", "action": action}

    def stop(self):
        return {"status": "STOPPED"}


class _FakeEvidence:
    def __init__(self) -> None:
        self.events: list[tuple[str, dict[str, object]]] = []

    def emit(self, event_type: str, **fields: object):
        self.events.append((event_type, dict(fields)))


def test_provider_tool_evidence_records_one_call_and_result_not_internal_reads() -> None:
    evidence = _FakeEvidence()
    gateway = ExternalRobotGateway(_FakeInterface(), policy=GatewayPolicy(allow_execute=True))
    tools = Er2RobotTools(gateway, Er2Config(), evidence=evidence)
    result = tools.execute("robot_status", {})
    assert result["status"] == "COMPLETED"
    assert [name for name, _ in evidence.events] == ["ER2_TOOL_CALL", "ER2_TOOL_RESULT"]
    assert evidence.events[1][1]["status"] == "COMPLETED"


def test_er2_evidence_uses_existing_hri_journal_contract(tmp_path: Path) -> None:
    path = tmp_path / "hri_events.ndjson"
    evidence = Er2Evidence(HriEventJournal(path), session_id="er2-test")
    evidence.emit("ER2_PREVIEW_START", model="test-model")
    row = json.loads(path.read_text(encoding="utf-8").strip())
    assert row["schema"] == HRI_EVENT_SCHEMA
    assert row["event_type"] == "ER2_PREVIEW_START"
    assert row["source"] == "ER2"
    assert row["session_id"] == "er2-test"


class _FakeTts:
    def synthesize(self, text: str):
        return SimpleNamespace(model="tts-model", voice="voice", text=text)


class _FakePlayer:
    def __init__(self) -> None:
        self.played = None

    def play(self, speech):
        self.played = speech
        return "fake-player"


def test_er2_spoken_report_reuses_host_tts_and_audio_without_robot_authority() -> None:
    evidence = _FakeEvidence()
    player = _FakePlayer()
    reporter = Er2SpeechReporter(evidence=evidence, tts=_FakeTts(), player=player)
    result = reporter.speak("Robot stopped safely.")
    assert result["player"] == "fake-player"
    assert player.played.text == "Robot stopped safely."
    assert [name for name, _ in evidence.events] == ["ER2_SPEECH_START", "ER2_SPEECH_COMPLETE"]

class _TwoDriveInteractions:
    def __init__(self) -> None:
        self.calls: list[dict[str, object]] = []

    def create(self, **kwargs):
        self.calls.append(dict(kwargs))
        if len(self.calls) == 1:
            args = {"v_mps": 0.15, "omega_rad_s": 0.0, "duration_s": 1.0}
            return SimpleNamespace(
                id="drive-round-1",
                steps=(
                    SimpleNamespace(type="function_call", name="robot_drive", arguments=args, id="drive-1"),
                    SimpleNamespace(type="function_call", name="robot_drive", arguments=args, id="drive-2"),
                ),
                output_text=None,
            )
        return SimpleNamespace(id="drive-round-2", steps=(), output_text="done")


class _RecordingDriveTools:
    def __init__(self) -> None:
        self.executed: list[str] = []

    def interaction_tools(self):
        return [{"type": "function", "name": "robot_drive", "parameters": {"type": "object", "properties": {}}}]

    def execute(self, name, arguments):
        self.executed.append(name)
        return {"status": "COMPLETED"}


def test_preview_executes_at_most_one_physical_drive_per_model_tool_round() -> None:
    sdk = SimpleNamespace(interactions=_TwoDriveInteractions())
    tools = _RecordingDriveTools()
    result = Er2PreviewClient(Er2Config(), client=sdk).run("two segments", tools=tools)
    assert result.text == "done"
    assert tools.executed == ["robot_drive"]
    returned = sdk.interactions.calls[1]["input"]
    assert returned[0]["call_id"] == "drive-1"
    assert json.loads(returned[0]["result"][0]["text"])["status"] == "COMPLETED"
    rejected = json.loads(returned[1]["result"][0]["text"])
    assert rejected["status"] == "ERROR"
    assert "ONE_ROBOT_DRIVE_PER_TOOL_ROUND" in rejected["error"]
