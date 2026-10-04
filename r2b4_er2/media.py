"""ER2 consumer of the independent canonical calibrated vision capability."""
from __future__ import annotations

import asyncio
import threading
from pathlib import Path

from v3.adapters.vision_media_contracts import VisionJpeg
from v3.adapters.vision_media_socket import VisionClient


class Er2MediaUnavailable(RuntimeError):
    pass


class VisionMediaClient:
    """Acquire calibrated images directly from the owner, independently of V3."""

    def __init__(
        self,
        socket_path: str | Path | None = None,
        *,
        project_root: str | Path | None = None,
        timeout_s: float = 15.0,
    ) -> None:
        self._client = VisionClient(socket_path, root=project_root, timeout_s=timeout_s)
        self.socket_path = self._client.socket_path

    def status(self) -> dict[str, object]:
        """Read owner readiness without creating camera demand."""
        return self._client.status()

    def observe_sync(self, *, stream_name: str = "lores", deadline=None, cancel_event=None) -> VisionJpeg:
        try:
            return self._client.observe(stream_name=stream_name, deadline=deadline, cancel_event=cancel_event)
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
