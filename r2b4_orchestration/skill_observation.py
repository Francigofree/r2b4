"""Compact host proxy for connection-owned sessions in the vision owner."""
from __future__ import annotations

from pathlib import Path
import math
import socket
import threading

from v3.adapters.vision_media_socket import VisionClient, recv_json_line, send_json_line


_OPERATIONS = {
    "observe.open": ("Open the installed person detector in an image-space region", {"region": "normalized bbox or null", "object_kind": "person", "ready_timeout_s": "seconds"}),
    "observe.status": ("Read observation readiness, source lineage and interval state", {"handle": "observer handle"}),
    "observe.events": ("Read bounded sequenced detector events and explicit delivery loss", {"handle": "observer handle", "after_sequence": "integer", "limit": "1..32"}),
    "observe.finish": ("Drain measurements through a monotonic deadline; timeout reports a gap", {"handle": "observer handle", "until_monotonic_ns": "measurement deadline", "timeout_s": "seconds", "after_sequence": "integer"}),
    "observe.close": ("Release this observer and its recorder demands", {"handle": "observer handle"}),
    "media.recorder.open": ("Open an asynchronous calibrated event recorder", {"observer": "observer handle", "pre_s": "seconds", "post_s": "seconds", "fps": "frames/s", "output_dir": "optional new artifact directory"}),
    "media.recorder.start_event_clip": ("Accept a qualified event clip; completion is returned separately", {"handle": "recorder handle", "event": "qualified observer event"}),
    "media.recorder.status": ("Read clip completion, failure and media references", {"handle": "recorder handle"}),
    "media.recorder.finish": ("Finish pending clips within a bounded wait or return partial results", {"handle": "recorder handle", "timeout_s": "seconds"}),
    "media.recorder.close": ("Cancel outstanding recorder work and release buffered frames", {"handle": "recorder handle"}),
}


class _Connection:
    def __init__(self, client):
        self.lock = threading.Lock()
        self.timeout_s = client.timeout_s
        self.conn = client.connect()
        try:
            self.conn.sendall(b"R2B4VISION1 SKILL\n")
            recv_json_line(self.conn)
        except BaseException:
            self.conn.close()
            raise

    def call(self, name, parameters):
        with self.lock:
            wait_s = parameters.get("ready_timeout_s", parameters.get("timeout_s", 0))
            timeout = self.timeout_s
            if type(wait_s) in (int, float) and math.isfinite(wait_s) and 0 <= wait_s <= 30:
                timeout = max(timeout, float(wait_s) + 1)
            self.conn.settimeout(timeout)
            send_json_line(self.conn, {"name": name, "parameters": parameters})
            value = recv_json_line(self.conn)
            if value.get("failure"):
                raise RuntimeError(value["failure"])
            return value["result"]

    def close(self):
        # Do not wait behind an outstanding bounded finish/readiness operation.
        try:
            self.conn.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.conn.close()


class SkillObservationAdapter:
    capability_names = frozenset(_OPERATIONS)

    def __init__(self, root: str | Path, *, client=None):
        self.client = client or VisionClient(root=root)
        self._lock = threading.Lock()
        self._sessions = {}
        self._pending = {}

    def descriptors(self):
        status = self.client.status()
        failed = status.get("camera_state") == "FAILED"
        return {name: {"name": name, "description": description, "parameters": parameters,
            "result": "compact session state, semantic events or media file references",
            "kind": "action", "supported": True, "available": not failed,
            "ready": not failed, "owner": "vision-owner", "reason": "VISION_UNAVAILABLE" if failed else None}
            for name, (description, parameters) in _OPERATIONS.items()}

    def call(self, name, parameters, invocation_id=None):
        if name not in _OPERATIONS:
            raise KeyError(name)
        key = invocation_id or "public"
        with self._lock:
            session = self._sessions.get(key)
            if session is None:
                marker = self._pending.get(key)
                creating = marker is None
                if creating and len(self._sessions) + len(self._pending) >= 8:
                    raise RuntimeError("OBSERVATION_CLIENT_CAPACITY_EXCEEDED")
                if creating:
                    marker = threading.Event()
                    self._pending[key] = marker
        if session is None and not creating:
            marker.wait(self.client.timeout_s)
            with self._lock:
                session = self._sessions.get(key)
            if session is None:
                raise RuntimeError("OBSERVATION_SESSION_UNAVAILABLE_OR_REVOKED")
        elif session is None:
            try:
                new_session = _Connection(self.client)
            except BaseException:
                with self._lock:
                    if self._pending.get(key) is marker:
                        self._pending.pop(key)
                    marker.set()
                raise
            with self._lock:
                revoked = self._pending.get(key) is not marker
                if not revoked:
                    self._pending.pop(key)
                    session = self._sessions.setdefault(key, new_session)
                marker.set()
            if revoked or session is not new_session:
                new_session.close()
            if revoked:
                raise RuntimeError("OBSERVATION_SESSION_REVOKED")
        try:
            return session.call(name, parameters)
        except (OSError, EOFError):
            with self._lock:
                if self._sessions.get(key) is session:
                    self._sessions.pop(key)
            session.close()
            raise

    def close_invocation(self, invocation_id=None):
        with self._lock:
            marker = self._pending.pop(invocation_id or "public", None)
            if marker is not None:
                marker.set()
            session = self._sessions.pop(invocation_id or "public", None)
        if session is not None:
            session.close()

    def close(self):
        with self._lock:
            for marker in self._pending.values():
                marker.set()
            self._pending.clear()
            sessions, self._sessions = tuple(self._sessions.values()), {}
        for session in sessions:
            session.close()
