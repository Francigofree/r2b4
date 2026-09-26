"""ER2 tools over the protocol-neutral ExternalRobotGateway.

The bridge owns no runtime, motor or safety authority. Physical calls are bounded
blocking operations, as required by Gemini Robotics ER 2 Streaming.
"""
from __future__ import annotations

import dataclasses
import math
import os
import threading
import time
from collections.abc import Mapping

from v3.external_gateway import ExternalRequest, ExternalRobotGateway, GatewayPolicy
from v3.robot_interface import RobotInterface

from .config import Er2Config
from .evidence import Er2Evidence


PHYSICAL_TOOLS = frozenset({
    "robot_navigate_to_pose", "robot_move_relative", "robot_turn_by", "robot_drive",
})


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise Er2ToolError(f"{name} must be finite numeric")
    return float(value)


def _wrap_yaw(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _jsonable(value: object) -> object:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if dataclasses.is_dataclass(value):
        return {str(k): _jsonable(v) for k, v in dataclasses.asdict(value).items()}
    if isinstance(value, Mapping):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set, frozenset)):
        return [_jsonable(item) for item in value]
    return str(value)


class Er2ToolError(RuntimeError):
    pass


class Er2SafetyError(Er2ToolError):
    """A physical tool could not confirm STOP; the provider must terminate."""


def _tool_result_metadata(result: Mapping[str, object]) -> dict[str, object]:
    out: dict[str, object] = {"status": result.get("status")}
    for key in ("mission_id", "reason"):
        if isinstance(result.get(key), str):
            out[key] = result[key]
    payload = result.get("result")
    if isinstance(payload, Mapping):
        for key in ("command_id", "mission_id", "action"):
            value = payload.get(key)
            if isinstance(value, (str, int, float, bool)) or value is None:
                out[key] = value
    command = result.get("command")
    if isinstance(command, Mapping):
        command_payload = command.get("result")
        if isinstance(command_payload, Mapping):
            for key in ("command_id", "mission_id", "action"):
                value = command_payload.get(key)
                if isinstance(value, (str, int, float, bool)) or value is None:
                    out[f"command_{key}"] = value
    return out


class Er2RobotTools:
    """Small, fail-closed ER2 tool surface with real execution only."""

    def __init__(
        self,
        gateway: ExternalRobotGateway,
        config: Er2Config | None = None,
        *,
        evidence: Er2Evidence | None = None,
        sleep=time.sleep,
        monotonic=time.monotonic,
        debug_drive: bool = False,
    ) -> None:
        if not isinstance(gateway, ExternalRobotGateway):
            raise TypeError("gateway must be ExternalRobotGateway")
        if gateway.policy.allow_execute is not True:
            raise ValueError("ER2 requires a gateway with real execution enabled")
        self.gateway = gateway
        self.config = config or Er2Config.from_env()
        self.evidence = evidence
        self._sleep = sleep
        self._monotonic = monotonic
        self._seq = 0
        self._debug_drive = debug_drive

    @classmethod
    def from_interface(
        cls,
        interface: RobotInterface,
        config: Er2Config | None = None,
        *,
        session_owner_pid: int | None = None,
        evidence: Er2Evidence | None = None,
    ) -> "Er2RobotTools":
        cfg = config or Er2Config.from_env()
        owner = os.getpid() if session_owner_pid is None else session_owner_pid
        gateway = ExternalRobotGateway(
            interface,
            policy=GatewayPolicy(
                allow_execute=True,
                session_owner_pid=owner,
                session_watchdog_s=cfg.session_watchdog_s,
            ),
        )
        return cls(gateway, cfg, evidence=evidence)

    def _emit(self, event_type: str, **fields: object) -> None:
        if self.evidence is not None:
            self.evidence.emit(event_type, **fields)

    def _request_id(self, label: str) -> str:
        self._seq += 1
        return f"er2-{label}-{os.getpid()}-{self._seq}"

    def _handle(self, operation: str, name: str | None = None, parameters: Mapping[str, object] | None = None) -> dict[str, object]:
        response = self.gateway.handle(
            ExternalRequest(
                request_id=self._request_id(operation),
                operation=operation,
                name=name,
                parameters={} if parameters is None else parameters,
            )
        )
        result = response.to_jsonable()
        result["result"] = _jsonable(result.get("result"))
        return result

    def robot_status(self) -> dict[str, object]:
        response = self._handle("read", "v3.status")
        if response["status"] != "COMPLETED":
            fallback = self._handle("read", "operator.status")
            return {"robot_status": response, "operator": fallback}
        return response

    def robot_stop(self) -> dict[str, object]:
        result = self._handle("stop")
        if result["status"] != "COMPLETED":
            raise Er2SafetyError(f"robot STOP failed: {result.get('error') or result['status']}")
        return result

    def robot_drive(
        self, *, v_mps: float, omega_rad_s: float, duration_s: float,
        cancel_event: threading.Event | None = None,
    ) -> dict[str, object]:
        for value, name in ((v_mps, "v_mps"), (omega_rad_s, "omega_rad_s"), (duration_s, "duration_s")):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                raise Er2ToolError(f"{name} must be finite numeric")
        v = float(v_mps)
        omega = float(omega_rad_s)
        duration = float(duration_s)
        if abs(v) > self.config.max_v_mps:
            raise Er2ToolError(f"abs(v_mps) exceeds ER2 bound {self.config.max_v_mps:g}")
        if abs(omega) > self.config.max_omega_rad_s:
            raise Er2ToolError(f"abs(omega_rad_s) exceeds ER2 bound {self.config.max_omega_rad_s:g}")
        if not 0.10 <= duration <= self.config.max_segment_s:
            raise Er2ToolError(f"duration_s must stay within [0.10, {self.config.max_segment_s:g}]")
        if abs(v) <= 1e-12 and abs(omega) <= 1e-12:
            return self.robot_stop()
        if cancel_event is not None and cancel_event.is_set():
            raise Er2ToolError("robot drive cancelled before execution")

        operator = self._handle("read", "operator.status")
        operator_payload = operator.get("result") if isinstance(operator, Mapping) else None
        current_mode = operator_payload.get("capture_mode") if isinstance(operator_payload, Mapping) else None
        current_hz = operator_payload.get("capture_hz") if isinstance(operator_payload, Mapping) else None
        interrupted: dict[str, object] | None = None
        try:
            if cancel_event is not None and cancel_event.is_set():
                raise Er2ToolError("robot drive cancelled before execution")
            command = self._handle(
                "execute",
                "v3.command.teleop",
                {
                    "v_mps": v,
                    "omega_rad_s": omega,
                    "max_v_mps": self.config.max_v_mps,
                    "max_omega_rad_s": self.config.max_omega_rad_s,
                    # Never force-restart a resident runtime merely to change capture.
                    "capture": False,
                    "capture_mode": current_mode if isinstance(current_mode, str) else "nincs",
                    "capture_hz": current_hz if isinstance(current_hz, int) else 10,
                },
            )
            if command["status"] not in {"ACCEPTED", "COMPLETED"}:
                raise Er2ToolError(f"robot drive rejected: {command.get('error') or command['status']}")
            started = self._monotonic()
            while True:
                if cancel_event is not None and cancel_event.is_set():
                    interrupted = {"reason": "CANCELLED"}
                    break
                elapsed = self._monotonic() - started
                if elapsed >= duration:
                    break
                self._sleep(min(0.05, duration - elapsed))
                safety = self._handle("read", "v3.safety")
                if safety["status"] != "COMPLETED":
                    interrupted = {"reason": "SAFETY_STATUS_UNAVAILABLE", "detail": safety}
                    break
                payload = safety.get("result")
                if isinstance(payload, Mapping):
                    decision = payload.get("decision")
                    state = payload.get("state")
                    if decision in {"STOP", "FAULT"} or state in {"FAULT", "ERROR"}:
                        interrupted = {"reason": "SAFETY_STOP", "detail": _jsonable(payload)}
                        break
        finally:
            stop = self.robot_stop()

        final = self.robot_status()
        return {
            "status": "COMPLETED" if interrupted is None else "INTERRUPTED",
            "requested": {"v_mps": v, "omega_rad_s": omega, "duration_s": duration},
            "elapsed_s": max(0.0, self._monotonic() - started),
            "command": command,
            "interrupted": interrupted,
            "stop": stop,
            "final": final,
        }

    def _pose_snapshot(self) -> Mapping[str, object]:
        response = self._handle("read", "v3.status")
        status = response.get("result")
        # V3ControlInterfaceAdapter only exposes fresh status from a live process.
        if response["status"] != "COMPLETED" or not isinstance(status, Mapping):
            raise Er2ToolError("fresh runtime status unavailable")
        if status.get("state") != "RUNNING" or status.get("fault_layer") or status.get("safety_decision") == "FAULT":
            raise Er2ToolError("runtime is not healthy")
        pose = status.get("estimate")
        if not isinstance(pose, Mapping) or not isinstance(pose.get("frame_id"), str) or not pose["frame_id"]:
            raise Er2ToolError("current localization frame unavailable")
        for key in ("x_m", "y_m", "yaw_rad"):
            _finite(pose.get(key), key)
        return pose

    def robot_navigate_to_pose(
        self, *, x_m: float, y_m: float, yaw_rad: float | None = None,
        max_v_mps: float | None = None, max_omega_rad_s: float | None = None,
        cancel_event: threading.Event | None = None,
    ) -> dict[str, object]:
        target = {"x_m": _finite(x_m, "x_m"), "y_m": _finite(y_m, "y_m")}
        if yaw_rad is not None:
            target["yaw_rad"] = _wrap_yaw(_finite(yaw_rad, "yaw_rad"))
        return self._navigate(target, self._pose_snapshot(), max_v_mps, max_omega_rad_s, cancel_event)

    def robot_move_relative(
        self, *, forward_m: float, left_m: float = 0.0,
        final_yaw_rad: float | None = None,
        max_v_mps: float | None = None, max_omega_rad_s: float | None = None,
        cancel_event: threading.Event | None = None,
    ) -> dict[str, object]:
        forward = _finite(forward_m, "forward_m")
        left = _finite(left_m, "left_m")
        yaw_delta = None if final_yaw_rad is None else _finite(final_yaw_rad, "final_yaw_rad")
        pose = self._pose_snapshot()
        yaw = float(pose["yaw_rad"])
        target = {
            "x_m": float(pose["x_m"]) + forward * math.cos(yaw) - left * math.sin(yaw),
            "y_m": float(pose["y_m"]) + forward * math.sin(yaw) + left * math.cos(yaw),
        }
        if yaw_delta is not None:
            target["yaw_rad"] = _wrap_yaw(yaw + yaw_delta)
        return self._navigate(target, pose, max_v_mps, max_omega_rad_s, cancel_event)

    def robot_turn_by(
        self, *, angle_deg: float, max_omega_rad_s: float | None = None,
        cancel_event: threading.Event | None = None,
    ) -> dict[str, object]:
        angle = _finite(angle_deg, "angle_deg")
        if not -180.0 <= angle <= 180.0:
            raise Er2ToolError("angle_deg must stay within [-180, 180]; pose goals use the shortest turn")
        pose = self._pose_snapshot()
        target = {"x_m": float(pose["x_m"]), "y_m": float(pose["y_m"]),
                  "yaw_rad": _wrap_yaw(float(pose["yaw_rad"]) + math.radians(angle))}
        return self._navigate(target, pose, None, max_omega_rad_s, cancel_event)

    def _navigate(
        self, target: dict[str, float], pose: Mapping[str, object],
        max_v_mps: float | None, max_omega_rad_s: float | None,
        cancel_event: threading.Event | None,
    ) -> dict[str, object]:
        parameters = dict(target)
        for name, requested, bound in (
            ("max_v_mps", max_v_mps, self.config.max_v_mps),
            ("max_omega_rad_s", max_omega_rad_s, self.config.max_omega_rad_s),
        ):
            value = bound if requested is None else _finite(requested, name)
            if not 0 < value <= bound:
                raise Er2ToolError(f"{name} must be >0 and <= {bound:g}")
            parameters[name] = value
        for name, value in target.items():
            _finite(value, name)
        operator = self._handle("read", "operator.status")
        capture = operator.get("result")
        if operator["status"] != "COMPLETED" or not isinstance(capture, Mapping):
            raise Er2ToolError("operator status unavailable")
        mode, hz = capture.get("capture_mode"), capture.get("capture_hz")
        if not isinstance(mode, str) or not isinstance(hz, int) or isinstance(hz, bool):
            raise Er2ToolError("current capture settings unavailable")

        started = self._monotonic()
        # Keep the independent producer watchdog as the final liveness bound.
        timeout_s = min(self.config.session_watchdog_s, self.gateway.policy.session_watchdog_s) - 0.5
        command: dict[str, object] | None = None
        mission_id: str | None = None
        final: Mapping[str, object] = {}
        reason = "CANCELLED"
        try:
            if cancel_event is None or not cancel_event.is_set():
                command = self._handle("execute", "v3.command.navigate", {
                    **parameters, "capture": False, "capture_mode": mode, "capture_hz": hz,
                })
                if command["status"] not in {"ACCEPTED", "COMPLETED"}:
                    raise Er2ToolError(f"navigation rejected: {command.get('error') or command['status']}")
                handle = command.get("result")
                command_id = handle.get("command_id") if isinstance(handle, Mapping) else None
                if not isinstance(command_id, str) or not command_id:
                    raise Er2ToolError("navigation returned no command identity")
                mission_id = f"mission-{command_id}"
                while True:
                    if cancel_event is not None and cancel_event.is_set():
                        reason = "CANCELLED"
                        break
                    remaining = timeout_s - (self._monotonic() - started)
                    if remaining <= 0:
                        reason = "TIMEOUT"
                        break
                    response = self._handle("read", "v3.status")
                    payload = response.get("result")
                    if response["status"] != "COMPLETED" or not isinstance(payload, Mapping):
                        reason = "STATUS_UNAVAILABLE"
                        break
                    final = payload
                    mission = payload.get("mission")
                    navigation = payload.get("navigation")
                    estimate = payload.get("estimate")
                    if payload.get("state") != "RUNNING" or payload.get("fault_layer") or payload.get("safety_decision") == "FAULT":
                        reason = "RUNTIME_FAULT"
                        break
                    if not isinstance(estimate, Mapping) or estimate.get("frame_id") != pose["frame_id"]:
                        reason = "LOCALIZATION_FRAME_CHANGED"
                        break
                    if not isinstance(mission, Mapping) or mission.get("mission_id") != mission_id:
                        reason = "MISSION_REPLACED"
                        break
                    if not isinstance(navigation, Mapping) or navigation.get("mission_id") != mission_id:
                        reason = "NAVIGATION_IDENTITY_MISMATCH"
                        break
                    if mission.get("lifecycle") in {"FAILED", "CANCELLED"}:
                        reason = "MISSION_" + str(mission["lifecycle"])
                        break
                    nav_status = navigation.get("status")
                    decision = payload.get("safety_decision")
                    if decision not in {"ALLOW", "STOP"}:
                        reason = "SAFETY_STATUS_UNAVAILABLE"
                        break
                    if decision == "STOP" and not (
                        nav_status == "COMPLETE" and payload.get("safety_reason") == "NOT_ACTIVE"
                    ):
                        reason = "SAFETY_STOP"
                        break
                    if mission.get("mode") != "NAVIGATE":
                        reason = "MISSION_INVALID"
                        break
                    if nav_status == "COMPLETE":
                        reason = "COMPLETE"
                        break
                    if nav_status in {"NO_PATH", "INVALIDATED"}:
                        reason = str(nav_status)
                        break
                    if nav_status not in {"ACTIVE", "PENDING", "IDLE"}:
                        reason = "NAVIGATION_STATUS_UNAVAILABLE"
                        break
                    self._sleep(min(0.05, remaining))
        finally:
            stop = self.robot_stop()

        navigation = final.get("navigation")
        navigation = navigation if isinstance(navigation, Mapping) else {}
        return {
            "status": "COMPLETED" if reason == "COMPLETE" else "INTERRUPTED",
            "reason": reason, "mission_id": mission_id, "requested": target,
            "elapsed_s": max(0.0, self._monotonic() - started),
            "final_pose": _jsonable(final.get("estimate")),
            "progress": navigation.get("progress"),
            "navigation_reason": navigation.get("reason"),
            "safety_reason": final.get("safety_reason"),
            "command": command, "stop": stop,
        }

    def execute(
        self, name: str, arguments: Mapping[str, object] | None = None,
        *, cancel_event: threading.Event | None = None,
    ) -> dict[str, object]:
        args = dict(arguments or {})
        self._emit("ER2_TOOL_CALL", tool_name=name, arguments=_jsonable(args))
        try:
            if name == "robot_status":
                if args:
                    raise Er2ToolError("robot_status takes no arguments")
                result = self.robot_status()
            elif name == "robot_stop":
                if args:
                    raise Er2ToolError("robot_stop takes no arguments")
                result = self.robot_stop()
            elif name in {"robot_navigate_to_pose", "robot_move_relative", "robot_turn_by"}:
                declaration = next(item for item in self.interaction_tools() if item["name"] == name)
                schema = declaration["parameters"]
                unknown = sorted(set(args) - set(schema["properties"]))
                missing = sorted(set(schema.get("required", ())) - set(args))
                if unknown or missing:
                    raise Er2ToolError(f"invalid {name} arguments: unknown={unknown}, missing={missing}")
                result = getattr(self, name)(**args, cancel_event=cancel_event)
            elif name == "robot_drive" and self._debug_drive:
                allowed = {"v_mps", "omega_rad_s", "duration_s"}
                unknown = sorted(set(args) - allowed)
                missing = sorted(allowed - set(args))
                if unknown:
                    raise Er2ToolError("unknown robot_drive arguments: " + ", ".join(unknown))
                if missing:
                    raise Er2ToolError("missing robot_drive arguments: " + ", ".join(missing))
                result = self.robot_drive(
                    v_mps=args["v_mps"],  # type: ignore[arg-type]
                    omega_rad_s=args["omega_rad_s"],  # type: ignore[arg-type]
                    duration_s=args["duration_s"],  # type: ignore[arg-type]
                    cancel_event=cancel_event,
                )
            else:
                raise Er2ToolError(f"unknown ER2 tool: {name}")
        except Exception as exc:
            self._emit(
                "ER2_TOOL_ERROR",
                tool_name=name,
                error_type=type(exc).__name__,
                error=str(exc)[:300],
            )
            raise
        self._emit("ER2_TOOL_RESULT", tool_name=name, **_tool_result_metadata(result))
        return result

    def interaction_tools(self) -> list[dict[str, object]]:
        speed = {
            "max_v_mps": {"type": "number", "description": "Optional positive linear speed limit.",
                          "maximum": self.config.max_v_mps},
            "max_omega_rad_s": {"type": "number", "description": "Optional positive angular speed limit.",
                               "maximum": self.config.max_omega_rad_s},
        }
        tools = [
            {
                "type": "function", "name": "robot_status",
                "description": "Read fresh R2B4 pose, mission, navigation, safety and health without moving.",
                "parameters": {"type": "object", "properties": {}},
            },
            {
                "type": "function", "name": "robot_stop",
                "description": "Stop R2B4 immediately through the canonical fail-safe command path.",
                "parameters": {"type": "object", "properties": {}},
            },
            {
                "type": "function", "name": "robot_navigate_to_pose",
                "description": "Navigate locally to x/y in the current localization frame, with optional final yaw. Blocks until this mission completes or fails, then confirms STOP. Inspect the result before choosing another movement.",
                "parameters": {"type": "object", "properties": {
                    "x_m": {"type": "number", "description": "Target x in metres in the current localization frame."},
                    "y_m": {"type": "number", "description": "Target y in metres in the current localization frame."},
                    "yaw_rad": {"type": "number", "description": "Optional absolute final heading in radians."},
                    **speed,
                }, "required": ["x_m", "y_m"]},
            },
            {
                "type": "function", "name": "robot_move_relative",
                "description": "Navigate to an offset from the current pose: forward and left in metres. Left is a destination offset, not strafing; R2B4 turns and drives to it. Blocks until completion or failure and confirms STOP.",
                "parameters": {"type": "object", "properties": {
                    "forward_m": {"type": "number"},
                    "left_m": {"type": "number", "description": "Leftward destination offset; defaults to zero."},
                    "final_yaw_rad": {"type": "number", "description": "Optional final heading offset in radians relative to the starting heading; positive is left."},
                    **speed,
                }, "required": ["forward_m"]},
            },
            {
                "type": "function", "name": "robot_turn_by",
                "description": "Turn at the current position using a closed-loop heading goal. Positive degrees turn left, negative right; 180 uses the canonical wrapped half-turn. Blocks until completion or failure and confirms STOP.",
                "parameters": {"type": "object", "properties": {
                    "angle_deg": {"type": "number", "minimum": -180, "maximum": 180},
                    "max_omega_rad_s": speed["max_omega_rad_s"],
                }, "required": ["angle_deg"]},
            },
        ]
        if self._debug_drive:
            tools.append(
                {
                    "type": "function",
                    "name": "robot_drive",
                    "description": "Debug fallback: one short bounded velocity segment, followed by confirmed STOP. Use the navigation tools for normal goals.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "v_mps": {"type": "number", "minimum": -self.config.max_v_mps, "maximum": self.config.max_v_mps},
                            "omega_rad_s": {"type": "number", "minimum": -self.config.max_omega_rad_s, "maximum": self.config.max_omega_rad_s},
                            "duration_s": {"type": "number", "minimum": 0.10, "maximum": self.config.max_segment_s},
                        },
                        "required": ["v_mps", "omega_rad_s", "duration_s"],
                    },
                }
            )
        return tools

    def live_tools(self) -> list[dict[str, object]]:
        declarations = []
        for item in self.interaction_tools():
            params = _upper_schema(item["parameters"])
            declarations.append(
                {
                    "name": item["name"],
                    "description": item["description"],
                    "behavior": "BLOCKING",
                    "parameters": params,
                }
            )
        return [{"function_declarations": declarations}]


def _upper_schema(value: object) -> object:
    if isinstance(value, Mapping):
        out: dict[str, object] = {}
        for key, item in value.items():
            if key == "type" and isinstance(item, str):
                out[str(key)] = item.upper()
            else:
                out[str(key)] = _upper_schema(item)
        return out
    if isinstance(value, list):
        return [_upper_schema(item) for item in value]
    return value


__all__ = ["Er2RobotTools", "Er2SafetyError", "Er2ToolError", "PHYSICAL_TOOLS"]
