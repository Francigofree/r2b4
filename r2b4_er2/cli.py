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

    stream = sub.add_parser(
        "stream",
        help="live ER2 Streaming robot session; camera, robot tools, speech and JSON are enabled by default",
    )
    stream.add_argument("task")
    stream.add_argument(
        "--camera",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="use the canonical live camera feed (enabled by default)",
    )
    stream.add_argument(
        "--tools",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="allow canonical bounded robot tools (enabled by default)",
    )
    stream.add_argument(
        "--speak",
        action="store_true",
        default=True,
        help="speak the accumulated ER2 text through the existing TTS/audio path (enabled by default)",
    )
    stream.add_argument(
        "--json",
        action="store_true",
        default=True,
        help="emit one machine-readable JSON result (enabled by default)",
    )
    stream.add_argument("--seconds", type=float, default=None, help="optional bounded session duration")
    return parser


def _safe_read(interface: RobotInterface, resource: str) -> object:
    try:
        return interface.read(resource)
    except Exception as exc:
        return {"read_error": f"{type(exc).__name__}: {exc}"}


def _probe_media(media: VisionMediaClient) -> dict[str, object]:
    try:
        status = media.status()
        return {
            **status,
            "available": status.get("owner_pid") is not None and not status.get("last_error"),
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
    media = VisionMediaClient(project_root=interface.root, timeout_s=cfg.media_timeout_s)
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
    media_error = _value(media, "error") or _value(media, "last_error")
    if media_ready:
        media_detail = f"READY camera={_value(media, 'camera_state', 'OFF')}"
    elif media_error:
        media_detail = f"UNAVAILABLE ({media_error})"
    else:
        media_detail = "IDLE; camera starts on observation demand"

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
    from v3.runtime_performance import apply_host_affinity
    apply_host_affinity(root, "er2")
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

    lease = _RuntimeLease(interface)
    tools = Er2RobotTools.from_interface(interface, cfg, evidence=evidence, ensure_runtime=lease.ensure)
    try:
        if args.command == "preview":
            image_bytes = None
            if args.image is not None:
                image_bytes = args.image.read_bytes()
            elif args.camera:
                observation = VisionMediaClient(project_root=root, timeout_s=cfg.media_timeout_s).observe_sync()
                image_bytes = observation.image_bytes
                evidence.emit("ER2_CAMERA_OBSERVATION", lineage=observation.metadata.to_jsonable())
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
            media = VisionMediaClient(project_root=root, timeout_s=cfg.media_timeout_s)
            text_chunks: list[str] = []

            def on_text(chunk: str) -> None:
                text_chunks.append(chunk)
                if not args.json:
                    print(chunk, end="", flush=True)

            evidence.emit(
                "ER2_STREAM_CLI_START",
                task_chars=len(args.task.strip()),
                camera_enabled=bool(args.camera),
                tools_enabled=bool(args.tools),
                speak_enabled=bool(args.speak),
                json_enabled=bool(args.json),
                bounded_duration_s=args.seconds,
            )
            result = Er2StreamingClient(
                tools if args.tools else None,
                media if args.camera else None,
                cfg,
                on_text=on_text,
                evidence=evidence,
            ).run(args.task, duration_s=args.seconds)
            text = "".join(text_chunks).strip()

            if args.speak:
                if not text:
                    raise RuntimeError("ER2 Streaming returned no text to speak")
                Er2SpeechReporter(evidence=evidence).speak(text)

            payload = {
                "mode": "stream",
                "text": text,
                "camera": bool(args.camera),
                "tools": bool(args.tools),
                "speak": bool(args.speak),
                "json": bool(args.json),
                "reconnect_count": result.reconnect_count,
                "resumption": bool(result.latest_resumption_handle),
                "stopped_cleanly": result.stopped_cleanly,
            }
            evidence.emit(
                "ER2_STREAM_CLI_COMPLETE",
                output_chars=len(text),
                camera_enabled=bool(args.camera),
                tools_enabled=bool(args.tools),
                speak_enabled=bool(args.speak),
                json_enabled=bool(args.json),
                reconnect_count=result.reconnect_count,
                resumable=bool(result.latest_resumption_handle),
                stopped_cleanly=result.stopped_cleanly,
            )
            if args.json:
                print(json.dumps(payload, ensure_ascii=False, indent=2))
            elif text_chunks:
                print()
            return 0
    finally:
        try:
            if tools.motion_attempted:
                tools.robot_stop()
        finally:
            lease.close()
    return 1


__all__ = [
    "main",
    "PREVIEW_MODEL",
    "STREAMING_MODEL",
    "_parser",
    "_probe_media",
    "_print_status",
    "_status_payload",
]
