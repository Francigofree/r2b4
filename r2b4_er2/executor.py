"""Programmatic ER2 executor for the system execution-mode router.

This is the non-CLI equivalent of the existing ER2 preview/stream workflows. It
returns text to the caller and never performs TTS, so voice and launcher callers
can use their own canonical response/output path.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from v3.robot_interface import RobotInterface

from .config import Er2Config
from .evidence import Er2Evidence
from .media import VisionMediaClient
from .preview import Er2PreviewClient
from .streaming import Er2StreamingClient
from .tool_bridge import Er2RobotTools


@dataclass(frozen=True, slots=True)
class Er2ExecutionResult:
    mode: str
    text: str
    reconnect_count: int = 0
    resumable: bool = False
    stopped_cleanly: bool = True


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


def run_er2_task(
    task: str,
    *,
    project_root: str | Path,
    mode: str = "stream",
    camera: bool = True,
    tools_enabled: bool = True,
    duration_s: float | None = None,
) -> Er2ExecutionResult:
    if not isinstance(task, str) or not task.strip():
        raise ValueError("ER2 task must be non-empty")
    normalized_mode = str(mode).strip().lower()
    if normalized_mode not in {"preview", "stream"}:
        raise ValueError("mode must be preview or stream")

    root = Path(project_root).expanduser().resolve()
    cfg = Er2Config.from_env()
    interface = RobotInterface(project_root=root)
    evidence = Er2Evidence.from_project_root(root)
    lease = _RuntimeLease(interface)
    tools = Er2RobotTools.from_interface(interface, cfg, evidence=evidence, ensure_runtime=lease.ensure)

    evidence.emit(
        "EXECUTION_ROUTER_ER2_START",
        mode=normalized_mode,
        task_chars=len(task.strip()),
        camera_enabled=bool(camera),
        tools_enabled=bool(tools_enabled),
        duration_s=duration_s,
    )

    try:
        if normalized_mode == "preview":
            image_bytes = None
            if camera:
                observation = VisionMediaClient(project_root=root, timeout_s=cfg.media_timeout_s).observe_sync()
                image_bytes = observation.image_bytes
                evidence.emit("ER2_CAMERA_OBSERVATION", lineage=observation.metadata.to_jsonable())
            result = Er2PreviewClient(cfg, evidence=evidence).run(
                task.strip(),
                image_bytes=image_bytes,
                tools=tools if tools_enabled else None,
            )
            text = result.text.strip()
            evidence.emit(
                "EXECUTION_ROUTER_ER2_COMPLETE",
                mode="preview",
                output_chars=len(text),
                tool_rounds=result.tool_rounds,
            )
            return Er2ExecutionResult(mode="preview", text=text)

        media = VisionMediaClient(project_root=root, timeout_s=cfg.media_timeout_s)
        chunks: list[str] = []

        def on_text(chunk: str) -> None:
            if isinstance(chunk, str):
                chunks.append(chunk)

        stream_result = Er2StreamingClient(
            tools if tools_enabled else None,
            media if camera else None,
            cfg,
            on_text=on_text,
            evidence=evidence,
        ).run(task.strip(), duration_s=duration_s)
        text = "".join(chunks).strip()
        evidence.emit(
            "EXECUTION_ROUTER_ER2_COMPLETE",
            mode="stream",
            output_chars=len(text),
            reconnect_count=stream_result.reconnect_count,
            resumable=bool(stream_result.latest_resumption_handle),
            stopped_cleanly=stream_result.stopped_cleanly,
        )
        return Er2ExecutionResult(
            mode="stream",
            text=text,
            reconnect_count=stream_result.reconnect_count,
            resumable=bool(stream_result.latest_resumption_handle),
            stopped_cleanly=stream_result.stopped_cleanly,
        )
    finally:
        try:
            if tools.motion_attempted:
                tools.robot_stop()
        finally:
            lease.close()


__all__ = ["Er2ExecutionResult", "run_er2_task"]
