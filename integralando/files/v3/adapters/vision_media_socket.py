"""Capability-specific producer-side JPEG egress for the process vision owner.

Large camera bytes stay in the vision process and leave directly through a local
Unix socket. The control interpreter sees only the existing bounded metadata.
"""
from __future__ import annotations

import os
import secrets
import socket
import tempfile
import threading
import time
from pathlib import Path
from typing import Protocol

_MAX_REQUEST_BYTES = 128
_MAX_JPEG_BYTES = 4 * 1024 * 1024


class CameraJpegRequestPort(Protocol):
    def request_jpeg(self, output: str | Path, *, stream_name: str = "lores") -> bool: ...


def default_vision_media_socket_path() -> Path:
    raw = os.environ.get("R2B4_VISION_MEDIA_SOCKET", "").strip()
    if raw:
        return Path(raw)
    return Path(tempfile.gettempdir()) / f"r2b4_vision_media_{os.getuid()}.sock"


def _ram_tmp_dir() -> Path:
    shm = Path("/dev/shm")
    if shm.is_dir() and os.access(shm, os.W_OK | os.X_OK):
        return shm
    return Path(tempfile.gettempdir())


class VisionMediaServer:
    """Serve one requested JPEG at a time from the existing camera owner."""

    def __init__(
        self,
        camera: CameraJpegRequestPort,
        socket_path: str | Path | None = None,
        *,
        photo_timeout_s: float = 1.0,
    ) -> None:
        if not callable(getattr(camera, "request_jpeg", None)):
            raise TypeError("camera must provide request_jpeg")
        self.camera = camera
        self.socket_path = Path(socket_path) if socket_path is not None else default_vision_media_socket_path()
        self.photo_timeout_s = float(photo_timeout_s)
        if not 0.1 <= self.photo_timeout_s <= 5.0:
            raise ValueError("photo_timeout_s must stay within [0.1, 5.0]")
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._server: socket.socket | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            self.socket_path.unlink()
        except FileNotFoundError:
            pass
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(str(self.socket_path))
        os.chmod(self.socket_path, 0o600)
        server.listen(2)
        server.settimeout(0.20)
        self._server = server
        self._stop.clear()
        thread = threading.Thread(target=self._serve, name="r2b4-vision-media", daemon=True)
        self._thread = thread
        thread.start()

    def stop(self) -> None:
        self._stop.set()
        server = self._server
        if server is not None:
            try:
                server.close()
            except OSError:
                pass
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._thread = None
        self._server = None
        try:
            self.socket_path.unlink()
        except FileNotFoundError:
            pass

    def _serve(self) -> None:
        server = self._server
        if server is None:
            return
        while not self._stop.is_set():
            try:
                conn, _ = server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with conn:
                conn.settimeout(self.photo_timeout_s + 0.5)
                try:
                    request = self._recv_line(conn)
                    parts = request.decode("ascii", "strict").strip().split()
                    if len(parts) != 2 or parts[0] != "R2B4JPEG1" or parts[1] not in {"lores", "main"}:
                        raise ValueError("expected: R2B4JPEG1 lores|main")
                    payload = self._capture(parts[1])
                    conn.sendall(f"OK {len(payload)}\n".encode("ascii"))
                    conn.sendall(payload)
                except Exception as exc:
                    message = f"{type(exc).__name__}:{exc}".replace("\n", " ")[:300]
                    try:
                        conn.sendall(f"ERR {message}\n".encode("utf-8", "replace"))
                    except OSError:
                        pass

    @staticmethod
    def _recv_line(conn: socket.socket) -> bytes:
        data = bytearray()
        while len(data) < _MAX_REQUEST_BYTES:
            chunk = conn.recv(1)
            if not chunk:
                break
            data += chunk
            if chunk == b"\n":
                return bytes(data)
        raise ValueError("media request too long or incomplete")

    def _capture(self, stream_name: str) -> bytes:
        target = _ram_tmp_dir() / (
            f"r2b4-er2-{os.getpid()}-{secrets.token_hex(8)}.jpg"
        )
        try:
            if not self.camera.request_jpeg(target, stream_name=stream_name):
                raise RuntimeError("camera JPEG request busy")
            deadline = time.monotonic() + self.photo_timeout_s
            last_size = -1
            stable = 0
            while time.monotonic() < deadline:
                try:
                    size = target.stat().st_size
                except FileNotFoundError:
                    size = 0
                if size > 0 and size == last_size:
                    stable += 1
                    if stable >= 2:
                        break
                else:
                    stable = 0
                last_size = size
                time.sleep(0.010)
            if not target.is_file():
                raise TimeoutError("camera JPEG was not produced")
            payload = target.read_bytes()
            if not 1 <= len(payload) <= _MAX_JPEG_BYTES:
                raise RuntimeError("camera JPEG outside bounded size")
            if not payload.startswith(b"\xff\xd8") or not payload.endswith(b"\xff\xd9"):
                raise RuntimeError("camera output is not a complete JPEG")
            return payload
        finally:
            try:
                target.unlink()
            except FileNotFoundError:
                pass


__all__ = [
    "CameraJpegRequestPort",
    "VisionMediaServer",
    "default_vision_media_socket_path",
]
