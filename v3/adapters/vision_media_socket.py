"""Bounded vision sessions and direct calibrated-image egress, outside control."""
from __future__ import annotations

import json
import os
import secrets
import select
import socket
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

from .vision_media_contracts import CameraJpegMetadata, VisionJpeg, MAX_JPEG_BYTES, MAX_METADATA_BYTES

MAX_STATE_BYTES = 65536


def default_vision_media_socket_path() -> Path:
    raw = os.environ.get("R2B4_VISION_MEDIA_SOCKET", "").strip()
    return Path(raw) if raw else Path(tempfile.gettempdir()) / f"r2b4_vision_media_{os.getuid()}.sock"


def recv_line(conn: socket.socket, limit: int = MAX_STATE_BYTES) -> bytes:
    data = bytearray()
    while len(data) < limit:
        part = conn.recv(1)
        if not part:
            raise EOFError("vision owner disconnected")
        if part == b"\n":
            return bytes(data)
        data.extend(part)
    raise ValueError("vision transport exceeded its bound")


def recv_json_line(conn: socket.socket, *, reader=None) -> dict:
    if reader is None:
        payload = recv_line(conn)
    else:
        payload = reader.readline(MAX_STATE_BYTES + 1)
        if not payload:
            raise EOFError("vision owner disconnected")
        if len(payload) > MAX_STATE_BYTES or not payload.endswith(b"\n"):
            raise ValueError("vision transport exceeded its bound")
    value = json.loads(payload)
    if not isinstance(value, dict):
        raise ValueError("invalid vision state")
    if value.get("error"):
        raise RuntimeError(str(value["error"]))
    return value


def send_json_line(conn: socket.socket, value: dict) -> None:
    payload = json.dumps(value, separators=(",", ":"), allow_nan=False).encode()
    if len(payload) > MAX_STATE_BYTES:
        raise ValueError("vision state exceeded its bound")
    conn.sendall(payload + b"\n")


def _recv_exact(conn: socket.socket, size: int) -> bytes:
    payload = bytearray()
    while len(payload) < size:
        part = conn.recv(min(65536, size - len(payload)))
        if not part:
            raise EOFError("incomplete vision payload")
        payload.extend(part)
    return bytes(payload)


class VisionClient:
    """A consumer; owner launch is lazy and independent of robot lifecycle."""

    def __init__(self, socket_path: str | Path | None = None, *, root: str | Path | None = None,
                 timeout_s: float = 15.0) -> None:
        self.socket_path = Path(socket_path) if socket_path is not None else default_vision_media_socket_path()
        self.root = Path(root) if root is not None else Path(__file__).resolve().parents[2]
        self.timeout_s = float(timeout_s)
        if not 0.1 <= self.timeout_s <= 30:
            raise ValueError("vision timeout must be within [0.1, 30]")

    def _connect(self) -> socket.socket:
        conn = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        conn.settimeout(self.timeout_s)
        try:
            conn.connect(str(self.socket_path))
        except BaseException:
            conn.close()
            raise
        return conn

    def connect(self, *, launch: bool = True) -> socket.socket:
        try:
            return self._connect()
        except (FileNotFoundError, ConnectionRefusedError):
            if not launch:
                raise
        # The service performs single-owner election before binding; simultaneous
        # first consumers therefore never open two physical devices.
        subprocess.Popen(
            [sys.executable, "-m", "v3.adapters.vision_owner", "--root", str(self.root),
             "--socket", str(self.socket_path)],
            cwd=self.root, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, start_new_session=True, close_fds=True,
        )
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            try:
                return self._connect()
            except (FileNotFoundError, ConnectionRefusedError):
                time.sleep(0.025)
        raise RuntimeError("VISION_OWNER_UNAVAILABLE")

    def status(self) -> dict[str, object]:
        try:
            conn = self.connect(launch=False)
        except (FileNotFoundError, ConnectionRefusedError):
            return {"running": False, "camera_state": "OFF", "detector_running": False,
                    "consumers": 0, "manual_demand": False, "owner_generation": "",
                    "last_error": None, "owner_pid": None}
        with conn:
            try:
                conn.sendall(b"R2B4VISION1 STATUS\n")
                return recv_json_line(conn)
            except (OSError, EOFError, RuntimeError, ValueError) as exc:
                return {"running": False, "camera_state": "FAILED", "detector_running": False,
                        "consumers": 0, "manual_demand": False, "owner_generation": "",
                        "last_error": f"VISION_UNAVAILABLE:{type(exc).__name__}:{exc}"[:256], "owner_pid": None}

    def set_manual_demand(self, active: bool) -> dict[str, object]:
        if type(active) is not bool:
            raise TypeError("active must be bool")
        if not active:
            try:
                conn = self.connect(launch=False)
            except (FileNotFoundError, ConnectionRefusedError):
                return self.status()
        else:
            conn = self.connect()
        with conn:
            conn.sendall(b"R2B4VISION1 ON\n" if active else b"R2B4VISION1 OFF\n")
            return recv_json_line(conn)

    def session(self) -> VisionSession:
        return VisionSession(self)

    def observe(self, *, stream_name: str = "lores") -> VisionJpeg:
        with self.session() as session:
            return session.observe(stream_name=stream_name)


class VisionSession:
    """Socket lifetime holds one image demand; disconnect releases it."""

    def __init__(self, client: VisionClient) -> None:
        self.client = client
        self.conn: socket.socket | None = None
        self.generation = ""
        self.maximum_age_ns = 250_000_000

    def __enter__(self) -> VisionSession:
        conn = self.client.connect()
        try:
            conn.sendall(b"R2B4VISION1 IMAGE\n")
            hello = recv_json_line(conn)
            self.generation = str(hello["owner_generation"])
            self.maximum_age_ns = int(hello["maximum_age_ns"])
            if not self.generation or self.maximum_age_ns <= 0:
                raise ValueError("invalid vision session")
            self.conn = conn
            return self
        except BaseException:
            conn.close()
            raise

    def observe(self, *, stream_name: str = "lores") -> VisionJpeg:
        if stream_name not in {"lores", "main"}:
            raise ValueError("stream_name must be lores or main")
        conn = self.conn
        if conn is None:
            raise RuntimeError("vision session is closed")
        conn.sendall(f"OBSERVE {stream_name}\n".encode())
        header = recv_line(conn, 1024).decode()
        if header.startswith("ERR "):
            raise RuntimeError(header[4:])
        parts = header.split()
        if len(parts) != 3 or parts[0] != "OK":
            raise ValueError("invalid vision image header")
        metadata_size, image_size = map(int, parts[1:])
        if not 1 <= metadata_size <= MAX_METADATA_BYTES or not 1 <= image_size <= MAX_JPEG_BYTES:
            raise ValueError("vision image exceeded its bound")
        metadata = CameraJpegMetadata.from_jsonable(json.loads(_recv_exact(conn, metadata_size)))
        payload = _recv_exact(conn, image_size)
        metadata.require_fresh(time.monotonic_ns(), generation=self.generation,
                               maximum_age_ns=self.maximum_age_ns)
        if metadata.stream != stream_name:
            raise RuntimeError("VISION_STREAM_MISMATCH")
        return VisionJpeg(payload, metadata)

    def __exit__(self, *_: object) -> None:
        if self.conn is not None:
            self.conn.close()
            self.conn = None


class VisionMediaServer:
    """Serve bounded sessions on the sole independent camera owner."""

    def __init__(self, owner, socket_path: str | Path | None = None, *, photo_timeout_s: float = 5.0) -> None:
        self.owner = owner
        self.socket_path = Path(socket_path) if socket_path is not None else default_vision_media_socket_path()
        self.photo_timeout_s = float(photo_timeout_s)
        self._stop = threading.Event()
        self._server: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._clients = threading.BoundedSemaphore(12)
        self._capture_lock = threading.Lock()
        self._connections: set[socket.socket] = set()
        self._connections_lock = threading.Lock()

    def start(self) -> None:
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
        self.socket_path.unlink(missing_ok=True)
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(str(self.socket_path))
        os.chmod(self.socket_path, 0o600)
        server.listen(12)
        server.settimeout(0.2)
        self._server = server
        self._thread = threading.Thread(target=self._serve, name="vision-media", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._server is not None:
            self._server.close()
        with self._connections_lock:
            for conn in tuple(self._connections):
                try:
                    conn.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
        if self._thread is not None:
            self._thread.join(timeout=1)
        self.socket_path.unlink(missing_ok=True)

    def _serve(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _ = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            if not self._clients.acquire(blocking=False):
                conn.close()
                continue
            with self._connections_lock:
                self._connections.add(conn)
            threading.Thread(target=self._handle, args=(conn,), daemon=True).start()

    def _handle(self, conn: socket.socket) -> None:
        demand = None
        image_mode = False
        try:
            conn.settimeout(15)
            parts = recv_line(conn, 128).decode("ascii").split()
            if len(parts) != 2 or parts[0] != "R2B4VISION1":
                raise ValueError("expected canonical vision request")
            command = parts[1]
            if command == "STATUS":
                send_json_line(conn, self.owner.status())
                return
            if command in {"ON", "OFF"}:
                self.owner.set_manual_demand(command == "ON")
                send_json_line(conn, self.owner.status())
                return
            if command not in {"PERSON", "IMAGE"}:
                raise ValueError("unknown vision request")
            demand = self.owner.acquire(person=command == "PERSON")
            generation = self.owner.generation
            if command == "PERSON":
                while not self._stop.is_set():
                    send_json_line(conn, self.owner.control_state(generation))
                    readable, _, _ = select.select([conn], [], [], 0.05)
                    if readable:
                        if not conn.recv(1):
                            return
                        raise ValueError("person session is receive-only")
                return
            image_mode = True
            send_json_line(conn, {"owner_generation": generation,
                                  "maximum_age_ns": self.owner.maximum_age_ns})
            # An image session may wait between observations, holding demand
            # until its consumer disconnects (video and provider sessions).
            conn.settimeout(None)
            while not self._stop.is_set():
                request = recv_line(conn, 128).decode("ascii").split()
                if len(request) != 2 or request[0] != "OBSERVE" or request[1] not in {"lores", "main"}:
                    raise ValueError("invalid observation request")
                result = self._capture(request[1], generation)
                metadata = json.dumps(result.metadata.to_jsonable(), separators=(",", ":")).encode()
                if len(metadata) > MAX_METADATA_BYTES:
                    raise ValueError("image metadata exceeded its bound")
                conn.settimeout(5)
                conn.sendall(f"OK {len(metadata)} {len(result.image_bytes)}\n".encode())
                conn.sendall(metadata)
                conn.sendall(result.image_bytes)
                conn.settimeout(None)
        except (EOFError, BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:
            try:
                message = f"{type(exc).__name__}:{exc}".replace("\n", " ")[:300]
                if image_mode:
                    conn.sendall(f"ERR {message}\n".encode())
                else:
                    send_json_line(conn, {"error": message})
            except OSError:
                pass
        finally:
            try:
                if demand is not None:
                    self.owner.release(demand)
            finally:
                conn.close()
                with self._connections_lock:
                    self._connections.discard(conn)
                self._clients.release()

    def _capture(self, stream_name: str, generation: str) -> VisionJpeg:
        if not self._capture_lock.acquire(timeout=self.photo_timeout_s):
            raise RuntimeError("vision observation busy")
        target = Path(tempfile.gettempdir()) / f"r2b4-vision-{os.getpid()}-{secrets.token_hex(8)}.jpg"
        try:
            camera = self.owner.camera_for_generation(generation)
            if not camera.request_jpeg(target, stream_name=stream_name):
                raise RuntimeError("camera JPEG request busy")
            deadline = time.monotonic() + self.photo_timeout_s
            while time.monotonic() < deadline:
                status = camera.get_photo_status()
                if status.last_error:
                    raise RuntimeError(status.last_error)
                if status.last_output == str(target) and status.last_metadata is not None:
                    metadata = status.last_metadata
                    self.owner.camera_for_generation(generation)
                    metadata.require_fresh(time.monotonic_ns(), generation=generation,
                                           maximum_age_ns=self.owner.maximum_age_ns)
                    return VisionJpeg(target.read_bytes(), metadata)
                time.sleep(0.01)
            raise TimeoutError("camera JPEG was not produced")
        finally:
            cancel = getattr(locals().get("camera"), "cancel_jpeg", None)
            if callable(cancel):
                cancel(target)
            target.unlink(missing_ok=True)
            self._capture_lock.release()


__all__ = ["VisionClient", "VisionSession", "VisionMediaServer", "default_vision_media_socket_path"]
