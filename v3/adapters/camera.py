"""RobotInterface consumer of the independent calibrated vision owner."""

from __future__ import annotations

from collections.abc import Mapping

from v3.adapters.camera_media import capture_h264_video, capture_photo
from v3.adapters.vision_media_socket import VisionClient
from v3.operator_controller import OperatorController


class CameraInterfaceAdapter:
    name = "camera"
    capability_names = frozenset({
        "camera.status", "camera.on", "camera.off", "camera.photo", "camera.video",
    })

    def __init__(self, controller: OperatorController) -> None:
        self.controller = controller

    def capabilities(self) -> Mapping[str, Mapping[str, object]]:
        status = VisionClient(root=self.controller.root).status()
        ready = status.get("camera_state") == "ON"
        return {
            name: {
                "kind": "read" if name == "camera.status" else "action",
                "supported": True,
                "available": True,
                "ready": ready or name in {"camera.status", "camera.off"},
                "reason": "CALIBRATED_DEMAND_DRIVEN",
            }
            for name in self.capability_names
        }

    def read(self, resource: str) -> object:
        if resource == "camera.status":
            return VisionClient(root=self.controller.root).status()
        raise KeyError(resource)

    def execute(self, action: str, **parameters: object) -> object:
        params = dict(parameters)
        if action in {"camera.on", "camera.off"}:
            self._reject_unknown(params)
            return VisionClient(root=self.controller.root).set_manual_demand(action == "camera.on")
        if action == "camera.photo":
            output = self._required(params, "output")
            self._reject_unknown(params)
            return capture_photo(str(output), root=self.controller.root)
        if action == "camera.video":
            output = self._required(params, "output")
            duration = self._required(params, "duration_s")
            bitrate = params.pop("bitrate", 4_000_000)
            self._reject_unknown(params)
            return capture_h264_video(
                str(output), float(duration), bitrate=int(bitrate), root=self.controller.root,
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
