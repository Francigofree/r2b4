"""Append-only NDJSON journal for R2B4 conversation sessions."""

from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path
from typing import Mapping


class ConversationJournal:
    def __init__(self, directory: Path | str, session_id: str | None = None) -> None:
        root = Path(directory)
        root.mkdir(parents=True, exist_ok=True)
        try:
            root.chmod(0o700)
        except OSError:
            pass
        if session_id is None:
            session_id = time.strftime("voice_%Y%m%d_%H%M%S") + f"_{os.getpid()}"
        if not session_id or any(ch not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for ch in session_id):
            raise ValueError("invalid conversation session_id")
        self.session_id = session_id
        self.path = root / f"{session_id}.ndjson"
        self._lock = threading.Lock()
        self._sequence = 0

    @property
    def sequence(self) -> int:
        with self._lock:
            return self._sequence

    def append(self, event_type: str, payload: Mapping[str, object], *, monotonic_ns: int | None = None) -> int:
        if not isinstance(event_type, str) or not event_type.strip():
            raise ValueError("event_type must be non-empty")
        record_time = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        if not isinstance(record_time, int) or isinstance(record_time, bool) or record_time < 0:
            raise ValueError("monotonic_ns must be non-negative")
        with self._lock:
            self._sequence += 1
            record = {
                "seq": self._sequence,
                "type": event_type.strip(),
                "session_id": self.session_id,
                "monotonic_ns": record_time,
                **dict(payload),
            }
            data = (json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
            fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
            try:
                try:
                    os.fchmod(fd, 0o600)
                except OSError:
                    pass
                written = 0
                while written < len(data):
                    written += os.write(fd, data[written:])
                os.fsync(fd)
            finally:
                os.close(fd)
            return self._sequence


__all__ = ["ConversationJournal"]
