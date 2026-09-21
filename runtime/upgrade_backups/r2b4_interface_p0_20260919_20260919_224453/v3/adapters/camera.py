"""RobotInterface adapter for exclusive physical camera diagnostics."""

from __future__ import annotations

from collections.abc import Mapping

from v3.adapters.camera_media import capture_h264_video, capture_photo
from v3.operator_controller import OperatorController


class CameraInterfaceAdapter:
    name = "camera"
    capability_names = frozenset({"camera.photo", "camera.video"})

    def __init__(self, controller: OperatorController) -> None:
        self.controller = controller

    def capabilities(self) -> Mapping[str, Mapping[str, object]]:
        running = bool(self.controller.status().get("runtime_running"))
        reason = "OWNED_BY_V3_RUNTIME" if running else None
        return {
            "camera.photo": {
                "kind": "action",
                "supported": True,
                "available": True,
                "ready": not running,
                "reason": reason,
            },
            "camera.video": {
                "kind": "action",
                "supported": True,
                "available": True,
                "ready": not running,
                "reason": reason,
            },
        }

    def read(self, resource: str) -> object:
        raise KeyError(resource)

    def execute(self, action: str, **parameters: object) -> object:
        if self.controller.status().get("runtime_running"):
            raise RuntimeError("camera is owned by the resident V3 runtime")
        params = dict(parameters)
        if action == "camera.photo":
            output = self._required(params, "output")
            warmup = params.pop("warmup_s", 2.0)
            self._reject_unknown(params)
            return capture_photo(str(output), warmup_s=float(warmup))
        if action == "camera.video":
            output = self._required(params, "output")
            duration = self._required(params, "duration_s")
            bitrate = params.pop("bitrate", 4_000_000)
            warmup = params.pop("warmup_s", 1.0)
            self._reject_unknown(params)
            return capture_h264_video(
                str(output),
                float(duration),
                bitrate=int(bitrate),
                warmup_s=float(warmup),
            )
        raise KeyError(action)

    @staticmethod
    def _required(parameters: dict[str, object], name: str) -> object:
        if name not in parameters:
            raise ValueError(f"missing required parameter: {name}")
        return parameters.pop(name)

    @staticmethod
    def _reject_unknown(parameters: Mapping[str, object]) -> None:
        if parameters:
            raise ValueError(f"unknown parameters: {', '.join(sorted(parameters))}")


__all__ = ["CameraInterfaceAdapter"]
