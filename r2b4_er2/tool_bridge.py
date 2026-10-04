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
from collections.abc import Callable, Mapping

from v3.external_gateway import ExternalRequest, ExternalRobotGateway, GatewayPolicy
from v3.capture_rate import DEFAULT_CAPTURE_HZ
from v3.operator_controller import DEFAULT_CAPTURE_MODE
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
    for key in ("command_id", "mission_id", "reason"):
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
        ensure_runtime: Callable[[], None] | None = None,
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
        self._ensure_runtime = ensure_runtime
        self.motion_attempted = False

    @classmethod
    def from_interface(
        cls,
        interface: RobotInterface,
        config: Er2Config | None = None,
        *,
        session_owner_pid: int | None = None,
        evidence: Er2Evidence | None = None,
        ensure_runtime: Callable[[], None] | None = None,
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
        return cls(gateway, cfg, evidence=evidence, ensure_runtime=ensure_runtime)

    def _activate_motion(self, cancel_event: threading.Event | None) -> None:
        if cancel_event is not None and cancel_event.is_set():
            raise Er2ToolError("physical tool cancelled before runtime activation")
        self.motion_attempted = True
        if self._ensure_runtime is not None:
            self._ensure_runtime(cancel_event=cancel_event)

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

    def robot_navigate_to_pose(
        self, *, x_m: float, y_m: float, yaw_rad: float | None = None,
        max_v_mps: float | None = None, max_omega_rad_s: float | None = None,
        cancel_event: threading.Event | None = None,
        deadline: float | None = None,
    ) -> dict[str, object]:
        return self._finite_action("v3.command.navigate", {
            "x_m": x_m, "y_m": y_m, "yaw_rad": yaw_rad,
            "max_v_mps": max_v_mps, "max_omega_rad_s": max_omega_rad_s,
            "wait_for_completion": True,
        }, cancel_event, deadline)

    def robot_move_relative(
        self, *, forward_m: float, left_m: float = 0.0,
        final_yaw_rad: float | None = None,
        max_v_mps: float | None = None, max_omega_rad_s: float | None = None,
        cancel_event: threading.Event | None = None,
        deadline: float | None = None,
    ) -> dict[str, object]:
        return self._finite_action("v3.command.move_relative", {
            "forward_m": forward_m, "left_m": left_m, "final_yaw_rad": final_yaw_rad,
            "max_v_mps": max_v_mps, "max_omega_rad_s": max_omega_rad_s,
        }, cancel_event, deadline)

    def robot_turn_by(
        self, *, angle_deg: float, max_omega_rad_s: float | None = None,
        cancel_event: threading.Event | None = None,
        deadline: float | None = None,
    ) -> dict[str, object]:
        return self._finite_action("v3.command.turn_by", {
            "angle_deg": angle_deg, "max_omega_rad_s": max_omega_rad_s,
        }, cancel_event, deadline)

    def _finite_action(
        self, action: str, parameters: Mapping[str, object],
        cancel_event: threading.Event | None,
        deadline: float | None = None,
    ) -> dict[str, object]:
        # ER2 retains only its provider envelope and gateway adaptation. Goal
        # geometry and the entire mission/STOP lifecycle belong to V3.
        params = dict(parameters)
        for name, bound in (("max_v_mps", self.config.max_v_mps),
                            ("max_omega_rad_s", self.config.max_omega_rad_s)):
            if name == "max_v_mps" and action == "v3.command.turn_by":
                continue
            requested = params.get(name)
            value = bound if requested is None else _finite(requested, name)
            if not 0 < value <= bound:
                raise Er2ToolError(f"{name} must be >0 and <= {bound:g}")
            params[name] = value
        operator = self._handle("read", "operator.status")
        capture = operator.get("result")
        if operator["status"] != "COMPLETED" or not isinstance(capture, Mapping):
            raise Er2ToolError("operator status unavailable")
        timeout = self.config.session_watchdog_s - 0.5
        if deadline is not None:
            timeout = min(timeout, deadline - time.monotonic())
            if timeout <= 0:
                raise Er2ToolError("physical tool deadline reached")
        params.update(
            capture=False,
            capture_mode=capture.get("capture_mode") or DEFAULT_CAPTURE_MODE,
            capture_hz=capture.get("capture_hz") or DEFAULT_CAPTURE_HZ,
            cancel_event=cancel_event,
            finite_timeout_s=timeout,
        )
        response = self._handle("execute", action, params)
        result = response.get("result")
        if response["status"] not in {"ACCEPTED", "COMPLETED"}:
            error = str(response.get("error") or response["status"])
            if error.startswith("FiniteNavigationSafetyError:"):
                raise Er2SafetyError(error)
            raise Er2ToolError(f"navigation rejected: {error}")
        if not isinstance(result, Mapping) or result.get("status") not in {"COMPLETED", "INTERRUPTED"}:
            self.robot_stop()
            raise Er2ToolError("finite navigation returned no completion result")
        return dict(result)

    def execute(
        self, name: str, arguments: Mapping[str, object] | None = None,
        *, cancel_event: threading.Event | None = None,
        deadline: float | None = None,
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
                self._activate_motion(cancel_event)
                result = getattr(self, name)(**args, cancel_event=cancel_event, deadline=deadline)
            elif name == "robot_drive" and self._debug_drive:
                allowed = {"v_mps", "omega_rad_s", "duration_s"}
                unknown = sorted(set(args) - allowed)
                missing = sorted(allowed - set(args))
                if unknown:
                    raise Er2ToolError("unknown robot_drive arguments: " + ", ".join(unknown))
                if missing:
                    raise Er2ToolError("missing robot_drive arguments: " + ", ".join(missing))
                self._activate_motion(cancel_event)
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
