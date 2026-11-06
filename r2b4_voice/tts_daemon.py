"""User-local resident Piper synthesis sidecar for R2B4.

The process owns only the local TTS model. It owns no microphone, robot state,
V3 authority, or audio playback. Requests are serialized over a mode-0600 Unix
socket and return the existing GeneratedSpeech PCM fields plus raw PCM bytes.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import struct
import threading
from pathlib import Path

from .piper_tts import PiperTtsClient, PiperTtsConfig, ensure_piper_importable

_PROTOCOL = 1
_MAX_HEADER_BYTES = 64 * 1024


def _recv_exact(conn: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = conn.recv(size - len(data))
        if not chunk:
            raise ConnectionError("client closed connection early")
        data.extend(chunk)
    return bytes(data)


def _recv_request(conn: socket.socket) -> dict[str, object]:
    size = struct.unpack("!I", _recv_exact(conn, 4))[0]
    if size <= 0 or size > _MAX_HEADER_BYTES:
        raise ValueError("invalid request header size")
    decoded = json.loads(_recv_exact(conn, size).decode("utf-8"))
    if not isinstance(decoded, dict):
        raise ValueError("request must be an object")
    return decoded


def _send_response(conn: socket.socket, header: dict[str, object], pcm: bytes = b"") -> None:
    body = dict(header)
    body["pcm_bytes"] = len(pcm)
    encoded = json.dumps(body, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    if len(encoded) > _MAX_HEADER_BYTES:
        raise ValueError("response header is too large")
    conn.sendall(struct.pack("!I", len(encoded)) + encoded + pcm)


def _serve(socket_path: Path, tts: PiperTtsClient) -> int:
    stop = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop
        stop = True

    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGTERM, request_stop)
        signal.signal(signal.SIGINT, request_stop)
    os.umask(0o077)
    socket_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        socket_path.unlink()
    except FileNotFoundError:
        pass

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        server.bind(str(socket_path))
        os.chmod(socket_path, 0o600)
        server.listen(8)
        server.settimeout(1.0)
        while not stop:
            try:
                conn, _ = server.accept()
            except socket.timeout:
                continue
            with conn:
                conn.settimeout(90.0)
                try:
                    request = _recv_request(conn)
                    if request.get("protocol") != _PROTOCOL:
                        raise ValueError("unsupported resident TTS protocol")
                    op = request.get("op")
                    if op == "ping":
                        _send_response(
                            conn,
                            {
                                "ok": True,
                                "protocol": _PROTOCOL,
                                "model": tts.model,
                                "voice": tts.voice,
                            },
                        )
                        continue
                    if op != "synthesize":
                        raise ValueError("unsupported resident TTS operation")
                    text = request.get("text")
                    if not isinstance(text, str) or not text.strip():
                        raise ValueError("text must be non-empty")
                    speech = tts.synthesize(text)
                    _send_response(
                        conn,
                        {
                            "ok": True,
                            "protocol": _PROTOCOL,
                            "sample_rate_hz": speech.sample_rate_hz,
                            "channels": speech.channels,
                            "sample_width_bytes": speech.sample_width_bytes,
                            "model": speech.model,
                            "voice": speech.voice,
                        },
                        speech.pcm,
                    )
                except Exception as exc:
                    try:
                        _send_response(
                            conn,
                            {
                                "ok": False,
                                "protocol": _PROTOCOL,
                                "error_type": type(exc).__name__,
                                "error": str(exc)[:500],
                            },
                        )
                    except Exception:
                        pass
        return 0
    finally:
        server.close()
        try:
            socket_path.unlink()
        except FileNotFoundError:
            pass


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="R2B4 resident local Piper TTS sidecar")
    parser.add_argument("--socket", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--voice", required=True)
    parser.add_argument("--dependency-dir", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    ensure_piper_importable(args.dependency_dir)
    tts = PiperTtsClient(PiperTtsConfig(model_path=args.model, voice=args.voice))
    return _serve(args.socket.expanduser().resolve(), tts)


if __name__ == "__main__":
    raise SystemExit(main())
