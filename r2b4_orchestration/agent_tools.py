"""Explicit composition of R2B4 capabilities exposed to AgentCore."""
from __future__ import annotations

from pathlib import Path
from collections.abc import Mapping

from .agent_contracts import AgentToolResult, AgentToolSpec
from .agent_config_tools import build_config_tools
from .agent_evidence_tools import build_evidence_tools
from .agent_source_tools import build_source_tools


def _strict(value: Mapping[str, object], allowed: set[str]) -> dict[str, object]:
    args = dict(value)
    unknown = sorted(set(args) - allowed)
    if unknown:
        raise ValueError("unknown arguments: " + ", ".join(unknown))
    return args


def _er2_delegate(root: Path, value: Mapping[str, object]) -> object:
    args = _strict(value, {"task", "reason", "mode", "camera", "tools", "duration_s"})
    reason = args.get("reason")
    if reason not in ("visual_observation", "multi_step_physical", "continuous_feedback", "open_ended_spatial"):
        raise ValueError("reason must be visual_observation|multi_step_physical|continuous_feedback|open_ended_spatial")
    task = args.get("task")
    if not isinstance(task, str) or not task.strip() or len(task) > 4000:
        raise ValueError("task must be 1..4000 characters")
    mode = args.get("mode", "stream")
    if mode not in {"preview", "stream"}:
        raise ValueError("mode must be preview or stream")
    camera = args.get("camera", True)
    tools_enabled = args.get("tools", mode == "stream")
    if type(camera) is not bool or type(tools_enabled) is not bool:
        raise ValueError("camera and tools must be booleans")
    duration = args.get("duration_s")
    if duration is None and mode == "stream":
        duration = 20.0
    if duration is not None:
        if isinstance(duration, bool) or not isinstance(duration, (int, float)) or not 1.0 <= float(duration) <= 30.0:
            raise ValueError("duration_s must be within [1, 30]")
        duration = float(duration)

    from r2b4_er2.executor import run_er2_task
    result = run_er2_task(
        task.strip(),
        project_root=root,
        mode=mode,
        camera=camera,
        tools_enabled=tools_enabled,
        duration_s=duration,
    )
    return {
        "reason": reason,
        "mode": result.mode,
        "text": result.text,
        "reconnect_count": result.reconnect_count,
        "resumable": result.resumable,
        "stopped_cleanly": result.stopped_cleanly,
    }


def _vision_observe(root: Path, value: Mapping[str, object]) -> AgentToolResult:
    args = _strict(value, {"stream"})
    stream = args.get("stream", "lores")
    if stream not in {"lores", "main"}:
        raise ValueError("stream must be lores or main")
    from v3.adapters.vision_media_socket import VisionClient
    observation = VisionClient(root=root).observe(stream_name=stream)
    return AgentToolResult(
        "vision.observe", "COMPLETED",
        data={"lineage": observation.metadata.to_jsonable(), "image_attached": True},
        images=(observation,),
    )


def build_default_agent_tools(project_root: Path):
    """Return the explicit current AgentCore tool surface.

    There is intentionally no filesystem/plugin discovery. Future tuning tools
    should be added here only after they have their own canonical typed API.
    """
    root = Path(project_root).resolve()
    tools = [
        *build_source_tools(root),
        *build_evidence_tools(root),
        *build_config_tools(root),
        (
            AgentToolSpec(
                "vision.observe",
                "Observe one fresh calibrated camera image without starting V3. The image is attached natively to your next provider step; the JSON result contains exact frame lineage only.",
                "READ",
                {"stream": "optional lores|main, default lores"},
            ),
            lambda args: _vision_observe(root, args),
        ),
    ]
    tools.append((
        AgentToolSpec(
            "er2.delegate",
            "Delegate an explicit visual/spatial/complex robot task to canonical ER2 with a required reason. Use one canonical action whenever it represents the entire request, including metric move/turn. ER2 still executes only through ExternalRobotGateway -> RobotInterface -> V3 safety. Do not use for hypothetical/explanatory requests.",
            "ROBOTICS",
            {
                "task": "required explicit robot task text",
                "reason": "required visual_observation|multi_step_physical|continuous_feedback|open_ended_spatial; a single canonical motion is not an ER2 reason",
                "mode": "optional preview|stream; stream for physical robotics",
                "camera": "optional boolean, default true",
                "tools": "optional boolean, default true for stream",
                "duration_s": "optional bounded 1..30 seconds; stream default 20",
            },
        ),
        lambda args: _er2_delegate(root, args),
    ))
    return tuple(tools)


__all__ = ["build_default_agent_tools"]
