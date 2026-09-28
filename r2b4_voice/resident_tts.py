"""Resident local Piper synthesis client for short-lived R2B4 host commands.

The normal ``r`` launcher is one process per invocation. Loading the ONNX voice
inside every process adds avoidable latency. This module keeps the existing
GeneratedSpeech contract while moving only Piper model ownership into a small,
user-local Unix-socket sidecar. If the sidecar cannot be started or reached,
the client falls back to the existing in-process Piper implementation.
"""
from __future__ import annotations

import fcntl
import hashlib
import json
import os
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from .gemini_tts import GeneratedSpeech
from .piper_tts import PiperTtsClient, PiperTtsConfig, ensure_piper_importable

_PROTOCOL = 1
_MAX_HEADER_BYTES = 64 * 1024
_DEFAULT_CONNECT_TIMEOUT_S = 1.0
_DEFAULT_REQUEST_TIMEOUT_S = 90.0
_DEFAULT_STARTUP_TIMEOUT_S = 20.0


class ResidentTtsError(RuntimeError):
    pass


def _runtime_dir() -> Path:
    configured = os.environ.get("XDG_RUNTIME_DIR", "").strip()
    if configured:
        path = Path(configured).expanduser().resolve()
        if path.is_dir():
            return path
    return Path("/tmp")


def _config_fingerprint(config: PiperTtsConfig, dependency_dir: Path) -> str:
    try:
        stat = config.model_path.stat()
        revision = f"{stat.st_size}:{stat.st_mtime_ns}"
    except OSError:
        revision = "missing"
    raw = "\0".join((str(config.model_path), config.voice, str(dependency_dir), revision))
    return hashlib.sha256(raw.encode("utf-8")).hexdigest()[:16]


def resident_socket_path(config: PiperTtsConfig, dependency_dir: Path) -> Path:
    name = f"r2b4-tts-{os.getuid()}-{_config_fingerprint(config, dependency_dir)}.sock"
    return _runtime_dir() / name


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ResidentTtsError("resident TTS connection closed early")
        data.extend(chunk)
    return bytes(data)


def _send_message(sock: socket.socket, payload: dict[str, object]) -> None:
    encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    if len(encoded) > _MAX_HEADER_BYTES:
        raise ResidentTtsError("resident TTS request is too large")
    sock.sendall(struct.pack("!I", len(encoded)) + encoded)


def _recv_response(sock: socket.socket) -> tuple[dict[str, Any], bytes]:
    header_size = struct.unpack("!I", _recv_exact(sock, 4))[0]
    if header_size <= 0 or header_size > _MAX_HEADER_BYTES:
        raise ResidentTtsError("resident TTS returned an invalid header size")
    try:
        header = json.loads(_recv_exact(sock, header_size).decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ResidentTtsError("resident TTS returned an invalid response header") from exc
    if not isinstance(header, dict):
        raise ResidentTtsError("resident TTS response header is not an object")
    pcm_size = header.get("pcm_bytes", 0)
    if not isinstance(pcm_size, int) or isinstance(pcm_size, bool) or pcm_size < 0:
        raise ResidentTtsError("resident TTS returned an invalid PCM size")
    pcm = _recv_exact(sock, pcm_size) if pcm_size else b""
    return header, pcm


class ResidentPiperTtsClient:
    """Piper-compatible client backed by one resident ONNX model process."""

    def __init__(
        self,
        config: PiperTtsConfig,
        *,
        project_root: Path | str,
        dependency_dir: Path | str,
        connect_timeout_s: float = _DEFAULT_CONNECT_TIMEOUT_S,
        request_timeout_s: float = _DEFAULT_REQUEST_TIMEOUT_S,
        startup_timeout_s: float = _DEFAULT_STARTUP_TIMEOUT_S,
    ) -> None:
        if not isinstance(config, PiperTtsConfig):
            raise TypeError("config must be PiperTtsConfig")
        self._config = config
        self._project_root = Path(project_root).expanduser().resolve()
        self._dependency_dir = Path(dependency_dir).expanduser().resolve()
        self._socket_path = resident_socket_path(config, self._dependency_dir)
        self._connect_timeout_s = float(connect_timeout_s)
        self._request_timeout_s = float(request_timeout_s)
        self._startup_timeout_s = float(startup_timeout_s)
        if min(self._connect_timeout_s, self._request_timeout_s, self._startup_timeout_s) <= 0:
            raise ValueError("resident TTS timeouts must be positive")
        self._fallback: PiperTtsClient | None = None

    @property
    def model(self) -> str:
        return "piper-tts"

    @property
    def voice(self) -> str:
        return self._config.voice

    @property
    def socket_path(self) -> Path:
        return self._socket_path

    def warmup(self) -> bool:
        """Ensure the sidecar is ready without synthesizing speech."""
        try:
            self._ensure_daemon()
            header, _ = self._request({"op": "ping", "protocol": _PROTOCOL})
            return header.get("ok") is True
        except Exception:
            return False

    def synthesize(self, text: str) -> GeneratedSpeech:
        if not isinstance(text, str) or not text.strip():
            raise ValueError("TTS text must be non-empty")
        spoken = text.strip()
        if len(spoken) > self._config.max_text_chars:
            raise ValueError("TTS text exceeds configured maximum")

        try:
            self._ensure_daemon()
            header, pcm = self._request(
                {"op": "synthesize", "protocol": _PROTOCOL, "text": spoken}
            )
            if header.get("ok") is not True:
                error_type = str(header.get("error_type") or "ResidentTtsError")
                detail = str(header.get("error") or "resident Piper synthesis failed")
                raise ResidentTtsError(f"{error_type}: {detail}")
            return GeneratedSpeech(
                pcm=pcm,
                sample_rate_hz=int(header["sample_rate_hz"]),
                channels=int(header["channels"]),
                sample_width_bytes=int(header["sample_width_bytes"]),
                model=str(header["model"]),
                voice=str(header["voice"]),
            )
        except (OSError, KeyError, TypeError, ValueError, ResidentTtsError):
            # Availability must fail open to the already-proven local path. The
            # fallback is cached for the remainder of this short-lived process.
            if self._fallback is None:
                ensure_piper_importable(self._dependency_dir)
                self._fallback = PiperTtsClient(self._config)
            return self._fallback.synthesize(spoken)

    def _request(self, payload: dict[str, object]) -> tuple[dict[str, Any], bytes]:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(self._connect_timeout_s)
            sock.connect(str(self._socket_path))
            sock.settimeout(self._request_timeout_s)
            _send_message(sock, payload)
            return _recv_response(sock)

    def _ping(self) -> bool:
        try:
            header, _ = self._request({"op": "ping", "protocol": _PROTOCOL})
            return (
                header.get("ok") is True
                and header.get("protocol") == _PROTOCOL
                and header.get("voice") == self.voice
            )
        except Exception:
            return False

    def _ensure_daemon(self) -> None:
        if self._ping():
            return
        lock_path = self._socket_path.with_suffix(self._socket_path.suffix + ".lock")
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        with lock_path.open("a+b") as lock_file:
            os.chmod(lock_path, 0o600)
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
            if self._ping():
                return
            try:
                self._socket_path.unlink()
            except FileNotFoundError:
                pass
            self._spawn_daemon()
            deadline = time.monotonic() + self._startup_timeout_s
            while time.monotonic() < deadline:
                if self._ping():
                    return
                time.sleep(0.05)
            raise ResidentTtsError(
                f"resident Piper sidecar did not become ready: {self._socket_path}"
            )

    def _spawn_daemon(self) -> None:
        fingerprint = _config_fingerprint(self._config, self._dependency_dir)
        log_path = Path("/tmp") / f"r2b4-tts-{os.getuid()}-{fingerprint}.log"
        env = dict(os.environ)
        env["R2B4_ROOT"] = str(self._project_root)
        env["PYTHONUNBUFFERED"] = "1"
        argv = [
            sys.executable,
            "-m",
            "r2b4_voice.tts_daemon",
            "--socket",
            str(self._socket_path),
            "--model",
            str(self._config.model_path),
            "--voice",
            self._config.voice,
            "--dependency-dir",
            str(self._dependency_dir),
        ]
        with log_path.open("ab", buffering=0) as log:
            subprocess.Popen(
                argv,
                cwd=str(self._project_root),
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=log,
                start_new_session=True,
                close_fds=True,
            )


__all__ = ["ResidentPiperTtsClient", "ResidentTtsError", "resident_socket_path"]
