#!/usr/bin/env python3
"""Focused offline validation for R2B4 Gemini Robotics ER2 P0."""
from __future__ import annotations

import asyncio
import os
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

# Executed as ``python tools/validate_er2_p0.py`` by the installer.  In that
# mode Python puts ``tools/`` rather than the repository root on sys.path.
# Bootstrap the R2B4 root explicitly so root-level packages such as r2b4_er2
# and v3 are importable independent of the caller's shell environment.
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_PROJECT_ROOT_STR = str(_PROJECT_ROOT)
if _PROJECT_ROOT_STR not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT_STR)

from r2b4_er2.config import PREVIEW_MODEL, STREAMING_MODEL, Er2Config
from r2b4_er2.media import VisionMediaClient
from r2b4_er2.preview import Er2PreviewClient
from r2b4_er2.streaming import Er2StreamingClient
from r2b4_er2.tool_bridge import Er2RobotTools
from v3.adapters.vision_media_contracts import CameraJpegMetadata, VisionJpeg
from v3.adapters.vision_media_socket import VisionMediaServer
from v3.adapters.vision_owner import CameraVisionOwner
from v3.external_gateway import ExternalRobotGateway, GatewayPolicy


class FakeInterface:
    def __init__(self) -> None:
        self.exec_calls: list[tuple[str, dict[str, object]]] = []
        self.stop_count = 0

    def capabilities(self):
        return {"schema": "fake", "capabilities": {}}

    def read(self, resource: str):
        if resource == "operator.status":
            return {"runtime_running": True, "capture_mode": "full", "capture_hz": 10}
        if resource == "v3.safety":
            return {"state": "RUNNING", "decision": "ALLOW", "reason": None}
        if resource == "v3.status":
            return {"state": "RUNNING", "ready_for_active": True, "safety_decision": "ALLOW"}
        raise RuntimeError(f"unexpected read {resource}")

    def execute(self, action: str, **parameters: object):
        self.exec_calls.append((action, dict(parameters)))
        return {"command_id": "fake-command", "action": action}

    def stop(self):
        self.stop_count += 1
        return {"status": "STOPPED"}


class Clock:
    def __init__(self) -> None:
        self.now = 0.0
    def monotonic(self) -> float:
        return self.now
    def sleep(self, seconds: float) -> None:
        self.now += float(seconds)


def fake_observation(*, stream_name="lores", sequence=1, generation="offline-fake-owner") -> VisionJpeg:
    now = time.monotonic_ns()
    width, height = (640, 360) if stream_name == "lores" else (1280, 720)
    metadata = CameraJpegMetadata(
        source_sequence=sequence, sensor_timestamp_ns=now,
        measurement_monotonic_ns=now, completed_monotonic_ns=now,
        calibration_id="offline-fake-calibration", stream=stream_name,
        width=width, height=height,
        rectified_K=((500.0, 0.0, width / 2), (0.0, 500.0, height / 2), (0.0, 0.0, 1.0)),
        owner_generation=generation,
    )
    return VisionJpeg(b"\xff\xd8R2B4-ER2-TEST\xff\xd9", metadata)


class FakeCamera:
    owner_generation = "offline-fake-owner"

    def __init__(self) -> None:
        self.running = False
        self.start_count = 0
        self.stop_count = 0
        self.sequence = 0
        self.photo = SimpleNamespace(last_output=None, last_error=None, last_metadata=None)

    def start(self) -> bool:
        self.start_count += 1
        self.running = True
        return True

    def stop(self) -> None:
        self.stop_count += 1
        self.running = False

    def get_runtime_status(self):
        return SimpleNamespace(running=self.running, last_error=None)

    def get_photo_status(self):
        return self.photo

    def request_jpeg(self, output, *, stream_name="lores") -> bool:
        assert self.running
        self.sequence += 1
        observation = fake_observation(stream_name=stream_name, sequence=self.sequence, generation=self.owner_generation)
        Path(output).write_bytes(observation.image_bytes)
        self.photo = SimpleNamespace(last_output=str(output), last_error=None, last_metadata=observation.metadata)
        return True


class FakeInteractions:
    def __init__(self) -> None:
        self.calls = []
    def create(self, **kwargs):
        self.calls.append(kwargs)
        if len(self.calls) == 1:
            fc = SimpleNamespace(type="function_call", name="robot_status", arguments={}, id="call-1")
            return SimpleNamespace(id="interaction-1", outputs=(fc,), output_text=None)
        return SimpleNamespace(id="interaction-2", outputs=(), output_text="preview-ok")


class FakePreviewClient:
    def __init__(self) -> None:
        self.interactions = FakeInteractions()


class FakeToolsForPreview:
    def interaction_tools(self):
        return [{"type": "function", "name": "robot_status", "parameters": {"type": "object", "properties": {}}}]
    def execute(self, name, arguments):
        assert name == "robot_status" and dict(arguments) == {}
        return {"status": "COMPLETED"}


class FakeLiveTypes:
    class FunctionResponse:
        def __init__(self, *, name, response, id=None):
            self.name = name; self.response = response; self.id = id


class FakeLiveSession:
    def __init__(self, message) -> None:
        self.message = message
        self.responses = []
    async def receive(self):
        if self.message is not None:
            message, self.message = self.message, None
            yield message
    async def send_tool_response(self, *, function_responses):
        self.responses.extend(function_responses)


class FakeLiveTools:
    config = Er2Config()
    def execute(self, name, args, *, cancel_event=None):
        assert name == "robot_stop"
        return {"status": "COMPLETED"}
    def robot_stop(self):
        return {"status": "STOPPED"}


class FakeMedia:
    async def observe(self, *, stream_name="lores") -> VisionJpeg:
        return fake_observation(stream_name=stream_name)


def validate_tools() -> None:
    cfg = Er2Config(max_segment_s=0.20, session_watchdog_s=2.0)
    interface = FakeInterface()
    gateway = ExternalRobotGateway(
        interface,
        policy=GatewayPolicy(allow_execute=True, session_owner_pid=os.getpid(), session_watchdog_s=2.0),
    )
    clock = Clock()
    tools = Er2RobotTools(gateway, cfg, sleep=clock.sleep, monotonic=clock.monotonic)
    declarations = tools.live_tools()[0]["function_declarations"]
    assert declarations and all(item["behavior"] == "BLOCKING" for item in declarations)
    result = tools.robot_drive(v_mps=0.10, omega_rad_s=0.0, duration_s=0.10)
    assert result["status"] == "COMPLETED"
    assert interface.stop_count >= 1
    action, params = interface.exec_calls[-1]
    assert action == "v3.command.teleop"
    assert params["capture"] is False
    assert params["capture_mode"] == "full" and params["capture_hz"] == 10
    assert params["session_owner_pid"] == os.getpid()
    assert params["session_watchdog_s"] == 2.0

    denied = ExternalRobotGateway(interface, policy=GatewayPolicy(allow_execute=False))
    try:
        Er2RobotTools(denied, cfg)
    except ValueError:
        pass
    else:
        raise AssertionError("ER2 tools must reject non-executing/shadow gateway")


def validate_media() -> None:
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "vision.sock"
        camera = FakeCamera()
        owner = CameraVisionOwner(lambda: camera, idle_grace_s=0.02)
        server = VisionMediaServer(owner, path, photo_timeout_s=0.3)
        server.start()
        try:
            client = VisionMediaClient(path, timeout_s=0.5)
            assert client.status()["camera_state"] == "OFF"
            assert camera.start_count == 0
            observation = client.observe_sync()
            assert observation.image_bytes == b"\xff\xd8R2B4-ER2-TEST\xff\xd9"
            assert observation.metadata == camera.photo.last_metadata
            assert observation.metadata.calibration_state == "CALIBRATED"
            assert observation.metadata.calibration_id == "offline-fake-calibration"
            assert observation.metadata.owner_generation == camera.owner_generation
            assert observation.metadata.source_sequence == 1
            assert observation.metadata.width == 640 and observation.metadata.height == 360
            assert observation.metadata.rectified_K[2] == (0.0, 0.0, 1.0)
            assert camera.start_count == 1
            deadline = time.monotonic() + 1.0
            while owner.status()["camera_state"] != "OFF" and time.monotonic() < deadline:
                time.sleep(0.01)
            assert owner.status()["camera_state"] == "OFF"
            assert camera.stop_count == 1
        finally:
            server.stop()
            owner.close()
        assert not path.exists()


def validate_preview() -> None:
    fake = FakePreviewClient()
    result = Er2PreviewClient(Er2Config(), client=fake).run("check robot", tools=FakeToolsForPreview())
    assert result.text == "preview-ok" and result.tool_rounds == 1
    assert fake.interactions.calls[0]["model"] == PREVIEW_MODEL
    second = fake.interactions.calls[1]
    assert second["previous_interaction_id"] == "interaction-1"
    assert second["input"][0]["type"] == "function_result"


def validate_stream_tool_return() -> None:
    call = SimpleNamespace(name="robot_stop", args={}, id="live-call-1")
    msg = SimpleNamespace(
        session_resumption_update=None,
        go_away=None,
        server_content=SimpleNamespace(model_turn=None, turn_complete=True),
        tool_call=SimpleNamespace(function_calls=(call,)),
    )
    session = FakeLiveSession(msg)
    client = Er2StreamingClient(FakeLiveTools(), FakeMedia(), Er2Config(), client=object(), on_text=lambda _t: None)
    async def run():
        await client._receive_loop(session, FakeLiveTypes, asyncio.Event(), asyncio.Event(), asyncio.Event())
    asyncio.run(run())
    assert len(session.responses) == 1
    assert session.responses[0].name == "robot_stop"
    assert session.responses[0].id == "live-call-1"


def main() -> int:
    assert PREVIEW_MODEL == "gemini-robotics-er-2-preview"
    assert STREAMING_MODEL == "gemini-robotics-er-2-streaming-preview"
    validate_tools()
    validate_media()
    validate_preview()
    validate_stream_tool_return()
    print("ER2 P0 offline validation: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
