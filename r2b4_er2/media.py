"""ER2 consumer of the independent canonical calibrated vision capability."""
from __future__ import annotations

import asyncio
import threading
from pathlib import Path

from v3.adapters.vision_media_contracts import VisionJpeg
from v3.adapters.vision_media_socket import VisionClient, default_vision_media_socket_path


class Er2MediaUnavailable(RuntimeError):
    pass


class VisionMediaClient:
    """Read calibrated vision through RobotInterface, independently of V3."""

    def __init__(
        self,
        socket_path: str | Path | None = None,
        *,
        project_root: str | Path | None = None,
        timeout_s: float = 15.0,
        interface: object | None = None,
    ) -> None:
        self.socket_path = Path(socket_path) if socket_path is not None else default_vision_media_socket_path()
        self.timeout_s = timeout_s
        if interface is None or socket_path is not None:
            from v3.adapters.camera import CameraInterfaceAdapter
            from v3.operator_controller import OperatorController
            from v3.robot_interface import RobotInterface
            root = Path(project_root) if project_root is not None else Path(__file__).resolve().parents[1]
            controller = OperatorController(root)
            camera = CameraInterfaceAdapter(controller, vision_client=VisionClient(
                self.socket_path, root=root, timeout_s=timeout_s,
            ))
            interface = RobotInterface(project_root=root, controller=controller, adapters=(camera,))
        self._interface = interface

    def status(self) -> dict[str, object]:
        """Read owner readiness without creating camera demand."""
        return self._interface.read("camera.status")

    def observe_sync(self, *, stream_name: str = "lores", deadline=None, cancel_event=None) -> VisionJpeg:
        try:
            observation = self._interface.execute(
                "vision.observe", stream=stream_name, deadline=deadline, cancel_event=cancel_event,
                timeout_s=self.timeout_s,
            )
            if not isinstance(observation, VisionJpeg):
                raise ValueError("vision.observe must return a calibrated VisionJpeg")
            return observation
        except (EOFError, OSError, ValueError, RuntimeError) as exc:
            raise Er2MediaUnavailable(f"vision observation unavailable: {exc}") from exc

    async def observe(self, *, stream_name: str = "lores", deadline=None) -> VisionJpeg:
        cancelled = threading.Event()
        worker = asyncio.create_task(asyncio.to_thread(self.observe_sync, stream_name=stream_name,
                                                     deadline=deadline, cancel_event=cancelled))
        try:
            return await asyncio.shield(worker)
        except asyncio.CancelledError:
            cancelled.set()
            try:
                await worker
            except Er2MediaUnavailable:
                pass
            raise

__all__ = ["Er2MediaUnavailable", "VisionMediaClient"]
