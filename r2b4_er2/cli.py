"""Human/agent CLI for real Gemini Robotics ER 2 Preview and Streaming."""
from __future__ import annotations

import argparse
import json
import os
from collections.abc import Mapping, Sequence
from pathlib import Path

from v3.robot_interface import RobotInterface

from .config import Er2Config, PREVIEW_MODEL, STREAMING_MODEL
from .evidence import Er2Evidence
from .media import Er2MediaUnavailable, VisionMediaClient
from .preview import Er2PreviewClient
from .speech import Er2SpeechReporter
from .streaming import Er2StreamingClient
from .tool_bridge import Er2RobotTools


class _RuntimeLease:
    def __init__(self, interface: RobotInterface) -> None:
        self.interface = interface
        self.owned = False

    def ensure(self) -> None:
        status = self.interface.read("operator.status")
        running = isinstance(status, dict) and status.get("runtime_running") is True
        if running:
            return
        self.interface.execute("operator.runtime.start", capture_mode="nincs", capture_hz=10)
        self.owned = True

    def close(self) -> None:
        if self.owned:
            self.interface.execute("operator.runtime.stop")
            self.owned = False


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="r er2", description="Gemini Robotics ER 2 integration")
    sub = parser.add_subparsers(dest="command", required=True)

    status = sub.add_parser("status", help="probe ER2 readiness and show concise R2B4 state")
    status.add_argument("--json", action="store_true", help="show full machine-readable state")

    preview = sub.add_parser("preview", help="one ER2 Preview Interactions turn")
    preview.add_argument("prompt")
    group = preview.add_mutually_exclusive_group()
    group.add_argument("--image", type=Path)
    group.add_argument("--camera", action="store_true")
    preview.add_argument("--tools", action="store_true", help="allow real bounded robot tool execution")
    preview.add_argument("--speak", action="store_true", help="speak the final ER2 text through the existing TTS/audio path")
    preview.add_argument("--json", action="store_true")

    stream = sub.add_parser("stream", help="live ER2 Streaming robot session")
    stream.add_argument("task")
    stream.add_argument("--seconds", type=float, default=None, help="optional bounded session duration")
    return parser


def _safe_read(interface: RobotInterface, resource: str) -> object:
    try:
        return interface.read(resource)
    except Exception as exc:
        return {"read_error": f"{type(exc).__name__}: {exc}"}


def _probe_media(media: VisionMediaClient) -> dict[str, object]:
    try:
        payload = media.latest_jpeg_sync(stream_name="lores")
        return {
            "available": True,
            "stream": "lores",
            "jpeg_bytes": len(payload),
            "socket": str(media.socket_path),
        }
    except Er2MediaUnavailable as exc:
        return {
            "available": False,
            "stream": "lores",
            "socket": str(media.socket_path),
            "error": str(exc),
        }
    except Exception as exc:
        return {
            "available": False,
            "stream": "lores",
            "socket": str(media.socket_path),
            "error": f"{type(exc).__name__}: {exc}",
        }


def _value(mapping: object, key: str, default: object = None) -> object:
    return mapping.get(key, default) if isinstance(mapping, Mapping) else default


def _status_payload(interface: RobotInterface, cfg: Er2Config) -> dict[str, object]:
    operator = _safe_read(interface, "operator.status")
    v3_status = _safe_read(interface, "v3.status")
    media = VisionMediaClient(timeout_s=cfg.media_timeout_s)
    media_probe = _probe_media(media)
    return {
        "preview_model": cfg.preview_model,
        "streaming_model": cfg.streaming_model,
        "api_key_configured": bool((os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY") or "").strip()),
        "media_probe": media_probe,
        "operator": operator,
        "v3_status": v3_status,
    }


def _print_status(data: Mapping[str, object]) -> None:
    operator = data.get("operator")
    robot = data.get("v3_status")
    media = data.get("media_probe")

    runtime_running = _value(operator, "runtime_running")
    runtime_pid = _value(operator, "runtime_pid", _value(operator, "pid"))
    capture_mode = _value(operator, "capture_mode", "?")
    capture_hz = _value(operator, "capture_hz", "?")

    robot_state = _value(robot, "state", "?")
    ready = _value(robot, "ready_for_active", "?")
    safety = _value(robot, "safety_decision", "?")
    fault = _value(robot, "fault_layer")

    media_ready = _value(media, "available") is True
    media_detail = (
        f"READY {_value(media, 'jpeg_bytes', '?')} B lores JPEG"
        if media_ready
        else f"UNAVAILABLE ({_value(media, 'error', 'unknown')})"
    )

    print(f"ER2 Preview:   {data['preview_model']}")
    print(f"ER2 Streaming: {data['streaming_model']}")
    print(f"API key:       {'configured' if data['api_key_configured'] else 'MISSING'}")
    print(f"Runtime:       {'RUNNING' if runtime_running is True else 'OFF'}" + (f" pid={runtime_pid}" if runtime_pid else ""))
    print(f"Robot:         state={robot_state} ready={ready} safety={safety} fault={fault or 'none'}")
    print(f"Capture:       {capture_mode} @ {capture_hz} Hz")
    print(f"Media:         {media_detail}")
    print(f"Media socket:  {_value(media, 'socket', '?')}")


def main(argv: Sequence[str] | None = None, *, project_root: str | Path | None = None) -> int:
    args = _parser().parse_args(list(argv) if argv is not None else None)
    root = Path(project_root).resolve() if project_root is not None else Path(__file__).resolve().parents[1]
    cfg = Er2Config.from_env()
    interface = RobotInterface(project_root=root)
    evidence = Er2Evidence.from_project_root(root)

    if args.command == "status":
        data = _status_payload(interface, cfg)
        evidence.emit(
            "ER2_STATUS_PROBE",
            api_key_configured=data["api_key_configured"],
            runtime_running=_value(data.get("operator"), "runtime_running"),
            media_available=_value(data.get("media_probe"), "available"),
        )
        if args.json:
            print(json.dumps(data, ensure_ascii=False, indent=2, default=str))
        else:
            _print_status(data)
        return 0

    tools = Er2RobotTools.from_interface(interface, cfg, evidence=evidence)
    lease = _RuntimeLease(interface)
    try:
        if args.command == "preview":
            image_bytes = None
            if args.image is not None:
                image_bytes = args.image.read_bytes()
            elif args.camera:
                lease.ensure()
                image_bytes = VisionMediaClient(timeout_s=cfg.media_timeout_s).latest_jpeg_sync()
            result = Er2PreviewClient(cfg, evidence=evidence).run(
                args.prompt,
                image_bytes=image_bytes,
                tools=tools if args.tools else None,
            )
            if args.json:
                print(json.dumps({"interaction_id": result.interaction_id, "text": result.text, "tool_rounds": result.tool_rounds}, ensure_ascii=False, indent=2))
            else:
                print(result.text)
            if args.speak:
                if not result.text.strip():
                    raise RuntimeError("ER2 returned no text to speak")
                Er2SpeechReporter(evidence=evidence).speak(result.text)
            return 0

        if args.command == "stream":
            lease.ensure()
            media = VisionMediaClient(timeout_s=cfg.media_timeout_s)
            result = Er2StreamingClient(tools, media, cfg, evidence=evidence).run(args.task, duration_s=args.seconds)
            print()
            print(json.dumps({"reconnect_count": result.reconnect_count, "resumption": bool(result.latest_resumption_handle), "stopped_cleanly": result.stopped_cleanly}, ensure_ascii=False))
            return 0
    finally:
        try:
            tools.robot_stop()
        except Exception:
            pass
        lease.close()
    return 1


__all__ = [
    "main",
    "PREVIEW_MODEL",
    "STREAMING_MODEL",
    "_probe_media",
    "_print_status",
    "_status_payload",
]
