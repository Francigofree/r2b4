"""Client for the producer-side R2B4 vision media socket."""
from __future__ import annotations

import asyncio
import socket
from pathlib import Path

from v3.adapters.vision_media_socket import default_vision_media_socket_path

_MAX_JPEG_BYTES = 4 * 1024 * 1024


class Er2MediaUnavailable(RuntimeError):
    pass


def _recv_line(sock: socket.socket, limit: int = 512) -> bytes:
    data = bytearray()
    while len(data) < limit:
        chunk = sock.recv(1)
        if not chunk:
            break
        data += chunk
        if chunk == b"\n":
            return bytes(data)
    raise Er2MediaUnavailable("invalid vision media response")


class VisionMediaClient:
    """Request one latest JPEG without routing image payload through control."""

    def __init__(self, socket_path: str | Path | None = None, *, timeout_s: float = 1.25) -> None:
        self.socket_path = Path(socket_path) if socket_path is not None else default_vision_media_socket_path()
        self.timeout_s = float(timeout_s)
        if self.timeout_s <= 0:
            raise ValueError("timeout_s must be positive")

    def latest_jpeg_sync(self, *, stream_name: str = "lores") -> bytes:
        if stream_name not in {"lores", "main"}:
            raise ValueError("stream_name must be lores or main")
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.settimeout(self.timeout_s)
                sock.connect(str(self.socket_path))
                sock.sendall(f"R2B4JPEG1 {stream_name}\n".encode("ascii"))
                header = _recv_line(sock)
                if header.startswith(b"ERR "):
                    raise Er2MediaUnavailable(header[4:].decode("utf-8", "replace").strip())
                parts = header.decode("ascii", "strict").strip().split()
                if len(parts) != 2 or parts[0] != "OK":
                    raise Er2MediaUnavailable("invalid vision media header")
                size = int(parts[1])
                if not 1 <= size <= _MAX_JPEG_BYTES:
                    raise Er2MediaUnavailable("vision JPEG size outside bounded range")
                data = bytearray()
                while len(data) < size:
                    chunk = sock.recv(min(65536, size - len(data)))
                    if not chunk:
                        raise Er2MediaUnavailable("vision media connection closed early")
                    data += chunk
        except (OSError, ValueError) as exc:
            if isinstance(exc, Er2MediaUnavailable):
                raise
            raise Er2MediaUnavailable(f"vision media unavailable: {exc}") from exc
        payload = bytes(data)
        if not payload.startswith(b"\xff\xd8") or not payload.endswith(b"\xff\xd9"):
            raise Er2MediaUnavailable("vision media payload is not a complete JPEG")
        return payload

    async def latest_jpeg(self, *, stream_name: str = "lores") -> bytes:
        return await asyncio.to_thread(self.latest_jpeg_sync, stream_name=stream_name)


__all__ = ["Er2MediaUnavailable", "VisionMediaClient"]
