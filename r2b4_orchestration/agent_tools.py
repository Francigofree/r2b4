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
from .world_model import WorldQuery, WorldQueryResult


class AgentRobotInterface(Protocol):
    def capabilities(self) -> Mapping[str, object]: ...
    def read(self, resource: str) -> object: ...
    def query(self, query: WorldQuery) -> WorldQueryResult: ...
    def execute(self, action: str, **parameters: object) -> object: ...


def _robot_capabilities(interface: AgentRobotInterface, value: Mapping[str, object]) -> object:
    _strict(value, set())
    return interface.capabilities()


def _robot_read(interface: AgentRobotInterface, value: Mapping[str, object]) -> object:
    args = _strict(value, {"resource"})
    resource = args.get("resource")
    if not isinstance(resource, str) or not resource:
        raise ValueError("resource must be a published resource name")
    return interface.read(resource)


def _robot_call(interface: AgentRobotInterface, value: Mapping[str, object]) -> object:
    args = _strict(value, {"name", "parameters"})
    name, parameters = args.get("name"), args.get("parameters", {})
    if not isinstance(name, str) or not name or not isinstance(parameters, Mapping):
        raise ValueError("robot.call requires name and parameters object")
    catalogue = interface.capabilities().get("capabilities", {})
    capability = catalogue.get(name) if isinstance(catalogue, Mapping) else None
    if not isinstance(capability, Mapping) or capability.get("supported") is not True:
        raise ValueError("CAPABILITY_NOT_PUBLISHED:" + name)
    descriptor = action_descriptor(name)
    if descriptor is not None or capability.get("requires_motion") is True:
        # Physical work still enters through Brain admission. The complete
        # public descriptor is discoverable without a second client allowlist.
        return AgentToolResult("robot.call", "PROPOSAL", data={"steps": [
            {"action": name, "parameters": dict(parameters)}]})
    if name == "skill.run":
        return AgentToolResult("robot.call", "PROPOSAL", data={
            "skill": parameters.get("name"), "parameters": parameters.get("parameters", {})})
    if isinstance(capability.get("skill_name"), str):
        return AgentToolResult("robot.call", "PROPOSAL", data={
            "skill": capability["skill_name"], "parameters": dict(parameters)})
    return interface.execute(name, **dict(parameters))


def _world_query(interface: AgentRobotInterface, value: Mapping[str, object]) -> object:
    query = WorldQuery.from_jsonable(value)
    result = interface.query(query)
    if not isinstance(result, WorldQueryResult):
        raise TypeError("world.query must return a canonical WorldQueryResult")
    return result.to_jsonable()


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
    tools_enabled = args.get("tools", False)
    if type(camera) is not bool or type(tools_enabled) is not bool:
        raise ValueError("camera and tools must be booleans")
    if tools_enabled:
        raise ValueError("ER2_SPECIALIST_HAS_NO_PHYSICAL_AUTHORITY")
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
    """Expose the common robot and K&F tools; developer_mode is compatibility input."""
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
                {"resource": "required published resource name from robot.capabilities"},
            ),
            lambda args: _robot_read(interface, args),
        ),
        (
            AgentToolSpec("robot.call", "Call any published software/compute capability; physical actions and skill.run become Brain proposals.",
                "READ", {"name": "required exact published name", "parameters": "optional parameters object"}),
            lambda args: _robot_call(interface, args),
        ),
        (
            AgentToolSpec(
                "world.query",
                "Read exact bounded semantic facts or task/observation episodes from the Brain's shared Public World Model without starting V3. Preserve returned freshness, uncertainty, scope, measurement time and lineage; a historical fact never authorizes motion.",
                "READ",
                {
                    "entity_id": "optional exact semantic entity ID",
                    "attribute": "optional exact fact attribute",
                    "domain": "optional published freshness domain",
                    "limit": "optional integer 1..64, default 32",
                    "require_current": "optional boolean, default false; true excludes stale, conflicting, unknown and scope-mismatched facts",
                    "scope": "optional object with current frame_id, runtime_pid and map_revision; all declared fact scope fields must match",
                    "kind": "optional facts|episodes, default facts; episodes are original evidence, not current execution targets",
                    "after_sequence": "optional non-negative integer episode cursor, default 0; inspect history_gap and truncated before claiming complete history",
                },
            ),
            lambda args: _world_query(interface, args),
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
    tools.extend((*build_source_tools(root), *build_evidence_tools(root), *build_config_tools(root)))
    for action in ("skill.create", "skill.update"):
        tools.append((AgentToolSpec(action, "Save a standard Python skill used by the common library and Brain.",
            "READ", {"name": "required skill name", "source": "required Python module; async def run(robot, **parameters)",
                "description": "optional short description", "test_source": "optional Python test module"}),
            lambda args, action=action: interface.execute(action, **dict(args))))
    tools.append((AgentToolSpec("skill.test", "Run this saved skill's optional Python tests through the canonical test launcher and return bounded feedback.",
        "READ", {"name": "required saved skill name"}),
        lambda args: interface.execute("skill.test", **dict(args))))
    tools.append((
        AgentToolSpec(
            "er2.delegate",
            "Use ER2 as a reasoning specialist only after an explicit ER2 trigger in the current user request. Return reasoning to Agent/Brain; physical tools are disabled. Prefer a canonical action or Brain plan for ordinary robot tasks.",
            "ROBOTICS",
            {
                "task": "required explicit robot task text",
                "reason": "required visual_observation|multi_step_physical|continuous_feedback|open_ended_spatial; a single canonical motion is not an ER2 reason",
                "mode": "optional preview|stream; stream for physical robotics",
                "camera": "optional boolean, default true",
                "tools": "optional false; physical tools are forbidden",
                "duration_s": "optional bounded 1..30 seconds; stream default 20",
            },
        ),
        lambda args, **control: _er2_delegate(root, args, **control),
    ))
    return tuple(tools)


__all__ = ["build_default_agent_tools"]
