#!/usr/bin/env python3
"""Focused offline validation for R2B4 Gemini Robotics ER2 P0."""
from __future__ import annotations

import asyncio
import os
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

from r2b4_er2.config import PREVIEW_MODEL, STREAMING_MODEL, Er2Config
from r2b4_er2.media import VisionMediaClient
from r2b4_er2.preview import Er2PreviewClient
from r2b4_er2.streaming import Er2StreamingClient
from r2b4_er2.tool_bridge import Er2RobotTools
from v3.adapters.vision_media_socket import VisionMediaServer
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


class FakeCamera:
    def request_jpeg(self, output, *, stream_name="lores") -> bool:
        Path(output).write_bytes(b"\xff\xd8R2B4-ER2-TEST\xff\xd9")
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
        yield self.message
    async def send_tool_response(self, *, function_responses):
        self.responses.extend(function_responses)


class FakeLiveTools:
    config = Er2Config()
    def execute(self, name, args):
        assert name == "robot_stop"
        return {"status": "COMPLETED"}
    def robot_stop(self):
        return {"status": "STOPPED"}


class FakeMedia:
    async def latest_jpeg(self, *, stream_name="lores"):
        return b"\xff\xd8x\xff\xd9"


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
        server = VisionMediaServer(FakeCamera(), path, photo_timeout_s=0.3)
        server.start()
        try:
            payload = VisionMediaClient(path, timeout_s=0.5).latest_jpeg_sync()
            assert payload.startswith(b"\xff\xd8") and payload.endswith(b"\xff\xd9")
        finally:
            server.stop()
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
