"""Read-only, bounded journal edge for upper-runtime semantic evidence.

The public runtime appends completed event envelopes. Capture follows only
events produced during its session, without importing the world/behavior owner
or sending anything back to the control interpreter.
"""

from __future__ import annotations

import json
import os
import stat
import time
from pathlib import Path


PUBLIC_RUNTIME_EVENT_TOPIC = "v3.public_runtime_event"
MAX_PUBLIC_RUNTIME_EVENT_BYTES = 1_048_576
_READ_BATCH_BYTES = 262_144
_EVENT_BATCH_COUNT = 64
_READ_CHUNK_BYTES = 65_536
_MAX_SESSION_EVENTS = 100_000
_MAX_PRODUCERS = 64


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate event field")
        result[key] = value
    return result


def _decode(line: bytes) -> dict[str, object]:
    def invalid_constant(_):
        raise ValueError("non-finite event value")

    row = json.loads(line, object_pairs_hook=_object, parse_constant=invalid_constant)
    if not isinstance(row, dict):
        raise ValueError("event envelope must be an object")
    for name in ("schema", "kind", "clock_epoch", "producer_id"):
        if not isinstance(row.get(name), str) or not row[name] or len(row[name]) > 256:
            raise ValueError("event identity must be bounded text")
    for name in ("publication_time_ns", "event_sequence", "evidence_dropped"):
        if type(row.get(name)) is not int or row[name] < 0:
            raise ValueError("event time and counters must be non-negative integers")
    if row["event_sequence"] == 0 or "value" not in row:
        raise ValueError("event sequence and value are required")
    return row


class PublicRuntimeEventFollower:
    """One capture session's incremental reader; it never changes the journal.

    Each drain bounds bytes read and events decoded. A final drain is bounded
    to the EOF observed at close, so a still-running producer cannot hold capture
    finalization open indefinitely. Invalid or lost evidence becomes an ordinary
    capture-transport observation with an explicit integrity reason.
    """

    def __init__(self, project_root: str | Path):
        self.path = Path(project_root) / "runtime" / "public_world" / "events.ndjson"
        self._stream = None
        self._identity = None
        self._pending = b""
        self._discard_line = False
        self._started = False
        self._disabled = False
        self._previous: dict[str, tuple[int, int]] = {}
        self._initial_losses: list[tuple[str, object]] = []
        self._final_offset = None
        self._rows_read = 0

    def _loss(self, reason: str) -> tuple[str, object]:
        return "v3.capture_transport", {
            "event_type": "public_runtime_evidence_loss",
            "integrity_reason": reason,
            "drop_count": 1,
            "monotonic_ns": time.monotonic_ns(),
        }

    def _open(self, *, baseline: bool) -> None:
        try:
            descriptor = os.open(self.path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        except FileNotFoundError:
            return
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode):
            os.close(descriptor)
            raise OSError("public event journal must be a regular file")
        self._stream = os.fdopen(descriptor, "rb")
        self._identity = info.st_dev, info.st_ino
        if baseline:
            # Only the last bounded row is needed to qualify session-local gaps
            # and cumulative loss. Historic payloads are never imported.
            start = max(0, info.st_size - MAX_PUBLIC_RUNTIME_EVENT_BYTES - 1)
            self._stream.seek(start)
            tail = self._stream.read(MAX_PUBLIC_RUNTIME_EVENT_BYTES + 1)
            if tail.endswith(b"\n"):
                last = tail.rstrip(b"\n").rsplit(b"\n", 1)[-1]
                try:
                    row = _decode(last)
                except (ValueError, UnicodeError, RecursionError):
                    pass  # Historical files may predate the event envelope.
                else:
                    self._previous[row["producer_id"]] = row["event_sequence"], row["evidence_dropped"]
            elif tail:
                # Ignore the rest of a row already in flight at session start.
                self._discard_line = True
            self._stream.seek(info.st_size)

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        try:
            self._open(baseline=True)
        except OSError:
            self._disabled = True
            self._initial_losses.append(self._loss("PUBLIC_RUNTIME_JOURNAL_UNREADABLE"))

    def drain(self) -> tuple[tuple[str, object], ...]:
        if not self._started:
            self.start()
        result = self._initial_losses
        self._initial_losses = []
        if self._disabled:
            return tuple(result)
        try:
            if self._stream is None:
                self._open(baseline=False)
                if self._stream is None:
                    return tuple(result)
            info = self.path.stat(follow_symlinks=False)
            if (info.st_dev, info.st_ino) != self._identity or info.st_size < self._stream.tell():
                self._disabled = True
                result.append(self._loss("PUBLIC_RUNTIME_JOURNAL_CHANGED"))
                return tuple(result)
            remaining = _READ_BATCH_BYTES
            while len(result) < _EVENT_BATCH_COUNT:
                newline = self._pending.find(b"\n")
                if newline >= 0:
                    line, self._pending = self._pending[:newline], self._pending[newline + 1:]
                    if self._discard_line:
                        self._discard_line = False
                        continue
                    self._rows_read += 1
                    if self._rows_read > _MAX_SESSION_EVENTS:
                        self._disabled = True
                        result.append(self._loss("PUBLIC_RUNTIME_EVENT_LIMIT"))
                        break
                    if len(line) > MAX_PUBLIC_RUNTIME_EVENT_BYTES:
                        result.append(self._loss("PUBLIC_RUNTIME_EVENT_OVERSIZED"))
                        continue
                    try:
                        row = _decode(line)
                    except (ValueError, UnicodeError, RecursionError):
                        result.append(self._loss("PUBLIC_RUNTIME_EVENT_INVALID"))
                        continue
                    if row["publication_time_ns"] > time.monotonic_ns():
                        result.append(self._loss("PUBLIC_RUNTIME_EVENT_FUTURE_TIME"))
                        continue
                    previous = self._previous.get(row["producer_id"])
                    if previous is None and len(self._previous) >= _MAX_PRODUCERS:
                        self._disabled = True
                        result.append(self._loss("PUBLIC_RUNTIME_PRODUCER_LIMIT"))
                        break
                    if previous is not None:
                        sequence, dropped = previous
                        if row["event_sequence"] != sequence + 1:
                            result.append(self._loss("PUBLIC_RUNTIME_EVENT_SEQUENCE_GAP"))
                        if row["evidence_dropped"] != dropped:
                            result.append(self._loss("PUBLIC_RUNTIME_PRODUCER_LOSS"))
                    elif row["evidence_dropped"]:
                        result.append(self._loss("PUBLIC_RUNTIME_PRODUCER_LOSS"))
                    self._previous[row["producer_id"]] = row["event_sequence"], row["evidence_dropped"]
                    # Preserve arbitrary future kind/value fields unchanged.
                    result.append((PUBLIC_RUNTIME_EVENT_TOPIC, row))
                    continue
                if len(self._pending) > MAX_PUBLIC_RUNTIME_EVENT_BYTES:
                    if not self._discard_line:
                        result.append(self._loss("PUBLIC_RUNTIME_EVENT_OVERSIZED"))
                    self._pending = b""
                    self._discard_line = True
                if remaining <= 0:
                    break
                size = min(_READ_CHUNK_BYTES, remaining)
                if self._final_offset is not None:
                    size = min(size, self._final_offset - self._stream.tell())
                    if size <= 0:
                        break
                data = self._stream.read(size)
                if not data:
                    break
                remaining -= len(data)
                self._pending += data
        except OSError:
            self._disabled = True
            result.append(self._loss("PUBLIC_RUNTIME_JOURNAL_UNREADABLE"))
        return tuple(result)

    def finish(self):
        """Yield bounded batches up to close-time EOF, then close the reader."""
        if not self._started:
            self.start()
        try:
            if self._stream is None and not self._disabled:
                self._open(baseline=False)
            if self._stream is not None:
                self._final_offset = os.fstat(self._stream.fileno()).st_size
            while True:
                position = self._stream.tell() if self._stream is not None else None
                pending = len(self._pending)
                batch = self.drain()
                if batch:
                    yield batch
                if self._disabled or self._stream is None:
                    break
                if self._stream.tell() == position and len(self._pending) == pending:
                    break
            if self._pending or self._discard_line:
                yield (self._loss("PUBLIC_RUNTIME_EVENT_PARTIAL"),)
        except OSError:
            yield (self._loss("PUBLIC_RUNTIME_JOURNAL_UNREADABLE"),)
        finally:
            self.close()

    def close(self) -> None:
        if self._stream is not None:
            self._stream.close()
            self._stream = None
