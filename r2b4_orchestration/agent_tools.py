"""Explicit composition of R2B4 capabilities exposed to AgentCore."""
from __future__ import annotations

from pathlib import Path
import threading
import time
from collections.abc import Mapping
from typing import Protocol

from v3.action_catalog import action_descriptor
from v3.adapters.vision_media_contracts import VisionJpeg

from .agent_contracts import AgentToolResult, AgentToolSpec
from .agent_config_tools import build_config_tools
from .agent_evidence_tools import build_evidence_tools
from .agent_source_tools import build_source_tools


class AgentRobotInterface(Protocol):
    def capabilities(self) -> Mapping[str, object]: ...
    def read(self, resource: str) -> object: ...
    def execute(self, action: str, **parameters: object) -> object: ...


_ROBOT_READ_RESOURCES = frozenset({
    "robot.state", "world.snapshot", "world.history", "behavior.state", "behavior.history",
    "operator.status", "v3.status", "v3.pose", "v3.health", "v3.safety", "camera.status",
})


def _robot_capabilities(interface: AgentRobotInterface, value: Mapping[str, object]) -> object:
    _strict(value, set())
    raw = interface.capabilities()
    caps = raw.get("capabilities")
    items = {}
    for name, capability in (caps.items() if isinstance(caps, Mapping) else ()):
        if not isinstance(name, str) or not isinstance(capability, Mapping):
            continue
        descriptor = action_descriptor(name)
        if (name in _ROBOT_READ_RESOURCES or name == "vision.observe"
                or name in {"behavior.room_cruise", "behavior.follow_person", "behavior.cancel"}
                or (descriptor is not None and descriptor.voice_exposed)):
            items[name] = dict(capability)
    return {"schema": raw.get("schema"), "capabilities": items}


def _robot_read(interface: AgentRobotInterface, value: Mapping[str, object]) -> object:
    args = _strict(value, {"resource"})
    resource = args.get("resource")
    if not isinstance(resource, str) or resource not in _ROBOT_READ_RESOURCES:
        raise ValueError("resource must be a published robot/world/behavior state resource")
    return interface.read(resource)


def _strict(value: Mapping[str, object], allowed: set[str]) -> dict[str, object]:
    args = dict(value)
    unknown = sorted(set(args) - allowed)
    if unknown:
        raise ValueError("unknown arguments: " + ", ".join(unknown))
    return args


def _er2_delegate(root: Path, value: Mapping[str, object], *,
                  cancel_event: threading.Event | None = None, deadline: float | None = None) -> object:
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
    if cancel_event is not None and cancel_event.is_set():
        raise TimeoutError("AGENT_TURN_CANCELLED")
    if deadline is not None:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("AGENT_TURN_EXPIRED")
        duration = remaining if duration is None else min(duration, remaining)
    result = run_er2_task(
        task.strip(),
        project_root=root,
        mode=mode,
        camera=camera,
        tools_enabled=tools_enabled,
        duration_s=duration,
        cancel_event=cancel_event,
    )
    return {
        "reason": reason,
        "mode": result.mode,
        "text": result.text,
        "reconnect_count": result.reconnect_count,
        "resumable": result.resumable,
        "stopped_cleanly": result.stopped_cleanly,
    }


def _vision_observe(interface: AgentRobotInterface, value: Mapping[str, object]) -> AgentToolResult:
    args = _strict(value, {"stream"})
    stream = args.get("stream", "lores")
    if stream not in {"lores", "main"}:
        raise ValueError("stream must be lores or main")
    observation = interface.execute("vision.observe", stream=stream)
    if not isinstance(observation, VisionJpeg):
        raise TypeError("vision.observe must return a canonical calibrated VisionJpeg")
    return AgentToolResult(
        "vision.observe", "COMPLETED",
        data={"lineage": observation.metadata.to_jsonable(), "image_attached": True},
        images=(observation,),
    )


def build_default_agent_tools(
    project_root: Path,
    *,
    interface: AgentRobotInterface | None = None,
    developer_mode: bool = False,
):
    """Return the explicit current AgentCore tool surface.

    Runtime turns receive public robot observation and canonical action tools.
    Source, evidence analysis and config writes require an explicit host-selected
    developer mode; a model reply or user turn cannot elevate the tool catalog.
    """
    root = Path(project_root).resolve()
    if type(developer_mode) is not bool:
        raise TypeError("developer_mode must be a boolean")
    if interface is None:
        from v3.robot_interface import RobotInterface
        interface = RobotInterface(project_root=root)
    tools = [
        (
            AgentToolSpec(
                "robot.capabilities",
                "Read the robot's currently published state and action capabilities. Availability and readiness are live observations, not permission to bypass V3 safety.",
                "READ",
            ),
            lambda args: _robot_capabilities(interface, args),
        ),
        (
            AgentToolSpec(
                "robot.read",
                "Read canonical robot/world/behavior state with temporal and confidence metadata. Public World Model knowledge does not override fresh V3 local geometry or safety.",
                "READ",
                {"resource": "required " + "|".join(sorted(_ROBOT_READ_RESOURCES))},
            ),
            lambda args: _robot_read(interface, args),
        ),
        (
            AgentToolSpec(
                "vision.observe",
                "Observe one fresh calibrated camera image without starting V3. The image is attached natively to your next provider step; the JSON result contains exact frame lineage only.",
                "READ",
                {"stream": "optional lores|main, default lores"},
            ),
            lambda args: _vision_observe(interface, args),
        ),
    ]
    if developer_mode:
        tools.extend((*build_source_tools(root), *build_evidence_tools(root), *build_config_tools(root)))
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
        lambda args, **control: _er2_delegate(root, args, **control),
    ))
    return tuple(tools)


__all__ = ["build_default_agent_tools"]
