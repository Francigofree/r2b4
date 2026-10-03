"""ER2 consumer of the independent canonical calibrated vision capability."""
from __future__ import annotations

import asyncio
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

    def observe_sync(self, *, stream_name: str = "lores") -> VisionJpeg:
        try:
            return self._client.observe(stream_name=stream_name)
        except (OSError, ValueError, RuntimeError) as exc:
            raise Er2MediaUnavailable(f"vision observation unavailable: {exc}") from exc

    async def observe(self, *, stream_name: str = "lores") -> VisionJpeg:
        return await asyncio.to_thread(self.observe_sync, stream_name=stream_name)

    def latest_jpeg_sync(self, *, stream_name: str = "lores") -> bytes:
        return self.observe_sync(stream_name=stream_name).image_bytes

    async def latest_jpeg(self, *, stream_name: str = "lores") -> bytes:
        return (await self.observe(stream_name=stream_name)).image_bytes


__all__ = ["Er2MediaUnavailable", "VisionMediaClient"]
