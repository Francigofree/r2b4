"""RobotInterface adapter for canonical V3 status and motion command ingress."""

from __future__ import annotations

from collections.abc import Mapping

from v3.action_catalog import ACTION_CATALOG, ActionDescriptor
from v3.capture_rate import DEFAULT_CAPTURE_HZ, validate_capture_hz
from v3.operator_controller import DEFAULT_CAPTURE_MODE, OperatorController


class V3ControlInterfaceAdapter:
    name = "v3_control"
    capability_names = frozenset({"v3.status", "v3.pose", "v3.safety", "v3.health"}) | frozenset(ACTION_CATALOG)

    def __init__(self, controller: OperatorController) -> None:
        self.controller = controller

    def capabilities(self) -> Mapping[str, Mapping[str, object]]:
        status = self.controller.status()
        runtime_running = bool(status.get("runtime_running"))
        runtime_status = self.controller.live_runtime_status()
        runtime_ready = bool(
            isinstance(runtime_status, Mapping)
            and runtime_status.get("state") == "RUNNING"
            and runtime_status.get("ready_for_active") is True
        )
        current = "RUNTIME_READY" if runtime_ready else (
            "RUNTIME_RUNNING_NOT_READY" if runtime_running else "RUNTIME_WILL_AUTO_START"
        )
        read_ready = isinstance(runtime_status, Mapping)
        result: dict[str, dict[str, object]] = {
            "v3.status": {
                "kind": "read", "supported": True, "available": read_ready, "ready": read_ready,
                "reason": None if read_ready else "NO_LIVE_RUNTIME_STATUS",
            },
            "v3.pose": {
                "kind": "read", "supported": True, "available": read_ready, "ready": read_ready,
                "reason": None if read_ready else "NO_LIVE_RUNTIME_STATUS",
            },
            "v3.safety": {
                "kind": "read", "supported": True, "available": read_ready, "ready": read_ready,
                "reason": None if read_ready else "NO_LIVE_RUNTIME_STATUS",
            },
            "v3.health": {
                "kind": "read", "supported": True, "available": read_ready, "ready": read_ready,
                "reason": None if read_ready else "NO_LIVE_RUNTIME_STATUS",
            },
        }
        for name, descriptor in ACTION_CATALOG.items():
            ready = runtime_running if name == "v3.command.stop" else True
            reason = "STOP_IS_SAFE_NOOP_WHEN_IDLE" if name == "v3.command.stop" else current
            result[name] = self._action(descriptor, True, ready, reason)
        return result

    @staticmethod
    def _action(descriptor: ActionDescriptor, available: bool, ready: bool, reason: str | None) -> dict[str, object]:
        result = descriptor.to_jsonable()
        result.update({
            "kind": "action",
            "supported": True,
            "available": available,
            "ready": ready,
            "reason": reason,
        })
        return result

    def read(self, resource: str) -> object:
        status = self.controller.live_runtime_status()
        if not isinstance(status, Mapping):
            raise RuntimeError("live resident V3 status is unavailable")
        if resource == "v3.status":
            return status
        if resource == "v3.pose":
            estimate = status.get("estimate")
            return dict(estimate) if isinstance(estimate, Mapping) else None
        if resource == "v3.safety":
            return {
                "state": status.get("state"),
                "tick_id": status.get("tick_id"),
                "ready_for_active": status.get("ready_for_active"),
                "decision": status.get("safety_decision"),
                "reason": status.get("safety_reason"),
                "fault_layer": status.get("fault_layer"),
                "enabled": status.get("enabled"),
                "left_output": status.get("left_output"),
                "right_output": status.get("right_output"),
            }
        if resource == "v3.health":
            health = status.get("source_health")
            return list(health) if isinstance(health, list) else []
        raise KeyError(resource)

    def execute(self, action: str, **parameters: object) -> object:
        params = dict(parameters)
        if action == "v3.command.stop":
            self._reject_unknown(params, set())
            self.controller.stop()
            return {"status": "STOPPED"}

        capture = bool(params.pop("capture", True))
        capture_mode = str(params.pop("capture_mode", DEFAULT_CAPTURE_MODE))
        capture_hz = validate_capture_hz(params.pop("capture_hz", DEFAULT_CAPTURE_HZ))
        session = {
            "session_owner_pid": params.pop("session_owner_pid", None),
            "session_watchdog_s": params.pop("session_watchdog_s", None),
        }
        if action == "v3.command.forward":
            speed = params.pop("speed_mps", 0.15)
            self._reject_unknown(params, set())
            return self.controller.forward(speed, capture=capture, capture_mode=capture_mode, capture_hz=capture_hz, **session)
        if action == "v3.command.backward":
            speed = params.pop("speed_mps", 0.15)
            self._reject_unknown(params, set())
            return self.controller.backward(speed, capture=capture, capture_mode=capture_mode, capture_hz=capture_hz, **session)
        if action == "v3.command.teleop":
            v_mps = self._required(params, "v_mps")
            omega_rad_s = self._required(params, "omega_rad_s")
            max_v = params.pop("max_v_mps", 0.50)
            max_omega = params.pop("max_omega_rad_s", 1.20)
            self._reject_unknown(params, set())
            return self.controller.start_teleop(
                v_mps=v_mps, omega_rad_s=omega_rad_s, max_v_mps=max_v,
                max_omega_rad_s=max_omega, capture=capture, capture_mode=capture_mode, capture_hz=capture_hz, **session,
            )
        if action == "v3.command.wheels":
            left = self._required(params, "left_mps")
            right = self._required(params, "right_mps")
            self._reject_unknown(params, set())
            return self.controller.wheels(left, right, capture=capture, capture_mode=capture_mode, capture_hz=capture_hz, **session)
        if action == "v3.command.explore":
            self._reject_unknown(params, set())
            return self.controller.roomcruise(capture=capture, capture_mode=capture_mode, capture_hz=capture_hz, **session)
        if action == "v3.command.face_person":
            max_omega = params.pop("max_omega_rad_s", 0.50)
            self._reject_unknown(params, set())
            return self.controller.faceperson(
                max_omega_rad_s=max_omega, capture=capture, capture_mode=capture_mode, capture_hz=capture_hz, **session,
            )
        if action == "v3.command.follow_person":
            max_v = params.pop("max_v_mps", 0.15)
            max_omega = params.pop("max_omega_rad_s", 0.30)
            self._reject_unknown(params, set())
            return self.controller.followperson(
                max_v_mps=max_v, max_omega_rad_s=max_omega,
                capture=capture, capture_mode=capture_mode, capture_hz=capture_hz, **session,
            )
        raise KeyError(action)

    @staticmethod
    def _required(parameters: dict[str, object], name: str) -> object:
        if name not in parameters:
            raise ValueError(f"missing required parameter: {name}")
        return parameters.pop(name)

    @staticmethod
    def _reject_unknown(parameters: Mapping[str, object], allowed: set[str]) -> None:
        unknown = sorted(set(parameters) - allowed)
        if unknown:
            raise ValueError(f"unknown parameters: {', '.join(unknown)}")


__all__ = ["V3ControlInterfaceAdapter"]
