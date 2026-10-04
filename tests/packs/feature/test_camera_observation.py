from __future__ import annotations

import base64
import io
import json
import time
from pathlib import Path
from types import SimpleNamespace

from r2b4_orchestration.agent_contracts import AgentToolRequest
from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker
from r2b4_orchestration.agent_tools import build_default_agent_tools
from r2b4_voice.gemini_llm import GeminiStructuredChatClient
from r2b4_voice.openai_llm import OpenAIResponsesChatClient
from v3.adapters.vision_media_contracts import CameraJpegMetadata, VisionJpeg


def _observation() -> VisionJpeg:
    now = time.monotonic_ns()
    return VisionJpeg(
        b"\xff\xd8canonical-camera-image\xff\xd9",
        CameraJpegMetadata(
            source_sequence=7, sensor_timestamp_ns=now,
            measurement_monotonic_ns=now, completed_monotonic_ns=now,
            calibration_id="calibration-test", stream="lores", width=640, height=480,
            rectified_K=((500.0, 0.0, 320.0), (0.0, 500.0, 240.0), (0.0, 0.0, 1.0)),
            owner_generation="owner-test",
        ),
    )


def _assert_native_agent_image_without_text_payload(monkeypatch, tmp_path: Path) -> None:
    from v3.adapters.vision_media_socket import VisionClient
    observation = _observation()
    requests = []
    events = []
    monkeypatch.setattr(VisionClient, "observe", lambda self, **kwargs: observation)

    replies = [
        {"kind": "tool", "spoken_text": None, "tool_name": "vision.observe", "tool_arguments_json": "{}", "action_name": None, "action_parameters": {}},
        {"kind": "final", "spoken_text": "Látom a képet.", "tool_name": None, "tool_arguments_json": None, "action_name": None, "action_parameters": {}},
    ]
    encoded_image = base64.b64encode(observation.image_bytes).decode("ascii")

    for provider in ("openai", "gemini"):
        requests.clear()
        events.clear()

        def urlopen(request, **kwargs):
            body = json.loads(request.data)
            requests.append(body)
            reply = json.dumps(replies[len(requests) - 1])
            if provider == "openai":
                event = {"type": "response.completed", "response": {"output_text": reply}}
                return io.BytesIO(("data: " + json.dumps(event) + "\n\n").encode())
            return io.BytesIO(json.dumps({"candidates": [{"content": {"parts": [{"text": reply}]}}]}).encode())

        model = OpenAIResponsesChatClient(api_key="test", urlopen=urlopen) if provider == "openai" else GeminiStructuredChatClient(api_key="test", urlopen=urlopen)
        broker = AgentToolBroker(build_default_agent_tools(tmp_path))
        result = AgentCore(model, broker).run(
            [{"role": "user", "content": "Mit látsz?"}], (),
            event_sink=lambda name, payload: events.append((name, payload)),
        )
        assert result.spoken_text == "Látom a képet."
        assert len(requests) == 2
        second = requests[1]
        if provider == "openai":
            parts = second["input"][-1]["content"]
            image = next(part for part in parts if part["type"] == "input_image")
            assert image["image_url"] == "data:image/jpeg;base64," + encoded_image
            text = second["instructions"] + " " + parts[0]["text"]
        else:
            parts = second["contents"][0]["parts"]
            image = next(part for part in parts if "inlineData" in part)
            assert image["inlineData"] == {"mimeType": "image/jpeg", "data": encoded_image}
            text = second["systemInstruction"]["parts"][0]["text"] + " " + parts[0]["text"]
        assert '"source_sequence":7' in text
        assert '"owner_generation":"owner-test"' in text
        assert encoded_image not in text
        assert "canonical-camera-image" not in text
        assert encoded_image not in json.dumps(events)
        assert "canonical-camera-image" not in json.dumps(events)
        tool_result = broker.execute(AgentToolRequest("vision.observe", {}))
        assert "image_bytes" not in json.dumps(tool_result.to_jsonable())
        assert "canonical-camera-image" not in repr(tool_result)


def _assert_camera_only_er2_never_starts_v3(monkeypatch, tmp_path: Path) -> None:
    import r2b4_er2.executor as executor
    from r2b4_er2.cli import _parser, _probe_media
    observation = _observation()
    runtime_operations = []
    media_calls = []

    class Interface:
        def __init__(self, **kwargs): pass
        def read(self, resource):
            raise AssertionError("camera observation must not read or start V3")
        def execute(self, name, **kwargs):
            runtime_operations.append(name)
            raise AssertionError("camera observation must not command V3")

    class Media:
        socket_path = tmp_path / "vision.sock"
        def __init__(self, **kwargs): pass
        def observe_sync(self, **kwargs):
            media_calls.append("observe")
            return observation
        def status(self):
            media_calls.append("status")
            return {"running": False, "camera_state": "OFF", "detector_running": False}

    class Tools:
        motion_attempted = False
        @classmethod
        def from_interface(cls, *args, **kwargs): return cls()
        def robot_stop(self): raise AssertionError("camera-only request must not issue robot STOP")

    class Preview:
        def __init__(self, *args, **kwargs): pass
        def run(self, task, **kwargs):
            assert kwargs["image_bytes"] == observation.image_bytes
            return SimpleNamespace(text="kép", tool_rounds=0)

    class Stream:
        def __init__(self, tools, media, *args, **kwargs):
            assert tools is None
            assert media is not None
            kwargs["on_text"]("kép")
        def run(self, *args, **kwargs):
            return SimpleNamespace(reconnect_count=0, latest_resumption_handle=None, stopped_cleanly=True)

    monkeypatch.setattr(executor, "RobotInterface", Interface)
    monkeypatch.setattr(executor, "VisionMediaClient", Media)
    monkeypatch.setattr(executor, "Er2RobotTools", Tools)
    monkeypatch.setattr(executor, "Er2PreviewClient", Preview)
    monkeypatch.setattr(executor, "Er2StreamingClient", Stream)
    for mode in ("preview", "stream"):
        result = executor.run_er2_task("mit látsz?", project_root=tmp_path, mode=mode, camera=True, tools_enabled=False, duration_s=1.0)
        assert result.text == "kép"
    assert runtime_operations == []
    probe = _probe_media(Media())
    assert probe["camera_state"] == "OFF"
    assert media_calls == ["observe", "status"]
    parsed = _parser().parse_args(["stream", "mit látsz?", "--no-tools", "--no-camera"])
    assert parsed.tools is False and parsed.camera is False


def test_camera_observation_native_provider_attachments_and_v3_independence(monkeypatch, tmp_path: Path) -> None:
    with monkeypatch.context() as patch:
        _assert_native_agent_image_without_text_payload(patch, tmp_path / "agent")
    with monkeypatch.context() as patch:
        _assert_camera_only_er2_never_starts_v3(patch, tmp_path / "er2")
