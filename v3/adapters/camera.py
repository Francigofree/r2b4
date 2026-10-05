"""RobotInterface consumer of the independent calibrated vision owner."""

from __future__ import annotations

from collections.abc import Mapping

from v3.adapters.camera_media import capture_h264_video, capture_photo
from v3.adapters.vision_media_socket import VisionClient
from v3.operator_controller import OperatorController


class CameraInterfaceAdapter:
    name = "camera"
    capability_names = frozenset({
        "camera.status", "camera.on", "camera.off", "camera.photo", "camera.video", "vision.observe",
    })

    def __init__(self, controller: OperatorController, *, vision_client: VisionClient | None = None) -> None:
        self.controller = controller
        self.vision_client = vision_client

    def _client(self, *, timeout_s: float | None = None) -> VisionClient:
        if self.vision_client is not None:
            return self.vision_client
        return VisionClient(root=self.controller.root, **({} if timeout_s is None else {"timeout_s": timeout_s}))

    def capabilities(self) -> Mapping[str, Mapping[str, object]]:
        status = self._client().status()
        ready = status.get("camera_state") == "ON"
        failed = status.get("camera_state") == "FAILED"
        return {
            name: {
                "kind": "read" if name == "camera.status" else "action",
                "supported": True,
                "available": not failed or name in {"camera.status", "camera.on", "camera.off"},
                "ready": ready or name in {"camera.status", "camera.off"},
                "reason": status.get("last_error") if failed else "CALIBRATED_DEMAND_DRIVEN",
            }
            for name in self.capability_names
        }

    def read(self, resource: str) -> object:
        if resource == "camera.status":
            return self._client().status()
        raise KeyError(resource)

    def execute(self, action: str, **parameters: object) -> object:
        params = dict(parameters)
        if action == "vision.observe":
            stream = params.pop("stream", "lores")
            deadline = params.pop("deadline", None)
            cancel_event = params.pop("cancel_event", None)
            timeout_s = params.pop("timeout_s", None)
            self._reject_unknown(params)
            if stream not in {"lores", "main"}:
                raise ValueError("stream must be lores or main")
            if timeout_s is not None and (type(timeout_s) not in (int, float) or not 0.1 <= timeout_s <= 30):
                raise ValueError("timeout_s must be within [0.1, 30]")
            # The image travels directly from its independent owner to the
            # requesting host consumer, never through the V3 control process.
            return self._client(timeout_s=timeout_s).observe(
                stream_name=stream, deadline=deadline, cancel_event=cancel_event,
            )
        if action in {"camera.on", "camera.off"}:
            self._reject_unknown(params)
            return self._client().set_manual_demand(action == "camera.on")
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
