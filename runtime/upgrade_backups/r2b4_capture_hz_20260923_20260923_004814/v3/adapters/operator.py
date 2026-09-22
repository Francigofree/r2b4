"""RobotInterface adapter for host/runtime and capture lifecycle orchestration."""

from __future__ import annotations

from collections.abc import Mapping

from v3.operator_controller import DEFAULT_CAPTURE_MODE, OperatorController


class OperatorInterfaceAdapter:
    name = "operator"
    capability_names = frozenset({
        "operator.status", "operator.diagnostics", "capture.status",
        "operator.runtime.start", "operator.runtime.stop", "operator.shutdown",
        "operator.panic", "capture.start", "capture.stop", "operator.proba",
    })

    def __init__(self, controller: OperatorController) -> None:
        self.controller = controller

    def capabilities(self) -> Mapping[str, Mapping[str, object]]:
        status = self.controller.status()
        running = bool(status.get("runtime_running"))
        capture_mode = self.controller.current_capture_mode()
        capture_path = self.controller.current_capture_path()
        capture_state = (
            "OFF" if capture_mode == "nincs"
            else "ARMED" if capture_path is not None
            else "NOT_ARMED"
        )
        return {
            "operator.status": self._read(True),
            "operator.diagnostics": self._read(True),
            "capture.status": self._read(True),
            "operator.runtime.start": self._action(True, True),
            "operator.runtime.stop": self._action(True, running, None if running else "ALREADY_STOPPED"),
            "operator.shutdown": self._action(True, running, None if running else "ALREADY_STOPPED"),
            "operator.panic": self._action(True, True),
            "capture.start": self._action(True, True, capture_state),
            "capture.stop": self._action(True, capture_state not in {"NOT_ARMED", "OFF"}, capture_state),
            "operator.proba": self._action(True, True),
        }

    @staticmethod
    def _read(ready: bool) -> dict[str, object]:
        return {"kind": "read", "supported": True, "available": True, "ready": ready}

    @staticmethod
    def _action(available: bool, ready: bool, reason: str | None = None) -> dict[str, object]:
        return {
            "kind": "action",
            "supported": True,
            "available": available,
            "ready": ready,
            "reason": reason,
        }

    def read(self, resource: str) -> object:
        if resource == "operator.status":
            return self.controller.status()
        if resource == "operator.diagnostics":
            return self.controller.diagnostics()
        if resource == "capture.status":
            return self.controller.capture_status()
        raise KeyError(resource)

    def execute(self, action: str, **parameters: object) -> object:
        params = dict(parameters)
        if action == "operator.runtime.start":
            mode = str(params.pop("capture_mode", DEFAULT_CAPTURE_MODE))
            self._reject_unknown(params)
            return {"pid": self.controller.ensure_runtime(mode), "capture_mode": mode}
        if action in {"operator.runtime.stop", "operator.shutdown"}:
            self._reject_unknown(params)
            self.controller.runtime_stop()
            return {"status": "STOPPED"}
        if action == "operator.panic":
            self._reject_unknown(params)
            self.controller.panic()
            return {"status": "STOPPED"}
        if action == "capture.start":
            mode = params.pop("capture_mode", None)
            self._reject_unknown(params)
            path = self.controller.capture_start(None if mode is None else str(mode))
            return {"status": "STARTED", "path": str(path) if path else None}
        if action == "capture.stop":
            self._reject_unknown(params)
            return self.controller.capture_stop()
        if action == "operator.proba":
            mode = str(params.pop("capture_mode", DEFAULT_CAPTURE_MODE))
            self._reject_unknown(params)
            self.controller.run_proba(capture_mode=mode)
            return {"status": "PASS"}
        raise KeyError(action)

    @staticmethod
    def _reject_unknown(parameters: Mapping[str, object]) -> None:
        if parameters:
            raise ValueError(f"unknown parameters: {', '.join(sorted(parameters))}")


__all__ = ["OperatorInterfaceAdapter"]
