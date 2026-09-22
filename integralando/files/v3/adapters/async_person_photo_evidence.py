"""Bounded asynchronous handoff for passive person-photo evidence.

The V3 control observer only enqueues immutable completed L2/L5 values.  Wall
clock formatting, filesystem-derived output naming and camera JPEG request work
remain on this passive worker and can never grant positive actuation authority.
"""
from __future__ import annotations

import queue
import threading
from dataclasses import dataclass

from v3.async_capability import TransportSemantics
from v3.contracts import AdmittedFrame, MissionIntent

from .person_photo_evidence import PersonPhotoEvidenceRecorder, PersonPhotoEvidenceStatus


@dataclass(frozen=True, slots=True)
class AsyncPersonPhotoEvidenceStatus:
    recorder: PersonPhotoEvidenceStatus
    enqueued_count: int
    processed_count: int
    superseded_count: int
    worker_error: str | None


class AsyncPersonPhotoEvidenceRecorder:
    """Passive bounded queue in front of ``PersonPhotoEvidenceRecorder``."""

    transport_semantics = TransportSemantics.EVIDENCE_STREAM

    __slots__ = (
        "_closed",
        "_enqueued_count",
        "_lock",
        "_processed_count",
        "_queue",
        "_recorder",
        "_stop",
        "_superseded_count",
        "_thread",
        "_worker_error",
    )

    def __init__(
        self,
        recorder: PersonPhotoEvidenceRecorder,
        *,
        capacity: int = 16,
    ) -> None:
        if not isinstance(recorder, PersonPhotoEvidenceRecorder):
            raise TypeError("recorder must be PersonPhotoEvidenceRecorder")
        if (
            not isinstance(capacity, int)
            or isinstance(capacity, bool)
            or capacity <= 0
        ):
            raise ValueError("capacity must be a positive integer")
        self._recorder = recorder
        self._queue: queue.Queue[tuple[AdmittedFrame, MissionIntent]] = queue.Queue(
            maxsize=capacity
        )
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._closed = False
        self._enqueued_count = 0
        self._processed_count = 0
        self._superseded_count = 0
        self._worker_error: str | None = None
        # Passive evidence must not be able to hold process shutdown hostage.
        self._thread = threading.Thread(
            target=self._run,
            name="r2b4-person-photo-evidence",
            daemon=True,
        )
        self._thread.start()

    @property
    def status(self) -> AsyncPersonPhotoEvidenceStatus:
        with self._lock:
            return AsyncPersonPhotoEvidenceStatus(
                recorder=self._recorder.status,
                enqueued_count=self._enqueued_count,
                processed_count=self._processed_count,
                superseded_count=self._superseded_count,
                worker_error=self._worker_error,
            )

    def observe(self, admitted: AdmittedFrame, mission: MissionIntent) -> bool:
        """O(1)-bounded publication from the completed control tick."""
        if not isinstance(admitted, AdmittedFrame):
            raise TypeError("admitted must be AdmittedFrame")
        if not isinstance(mission, MissionIntent):
            raise TypeError("mission must be MissionIntent")
        if admitted.context != mission.context:
            raise ValueError("L2 and L5 evidence contexts must match")
        with self._lock:
            if self._closed:
                return False
        item = (admitted, mission)
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            # Photo evidence is best-effort and has no safety authority.  Keep
            # the newest completed semantic state and expose supersede telemetry.
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            else:
                with self._lock:
                    self._superseded_count += 1
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                with self._lock:
                    self._superseded_count += 1
                return False
        with self._lock:
            self._enqueued_count += 1
        return True

    def _run(self) -> None:
        while not self._stop.is_set() or not self._queue.empty():
            try:
                admitted, mission = self._queue.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                self._recorder.observe(admitted, mission)
            except BaseException as exc:
                with self._lock:
                    self._worker_error = f"{type(exc).__name__}:{exc}"[:256]
            finally:
                with self._lock:
                    self._processed_count += 1

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        self._stop.set()
        self._thread.join(timeout=0.5)
        if self._thread.is_alive():
            with self._lock:
                self._worker_error = self._worker_error or "PERSON_EVIDENCE_WORKER_STOP_TIMEOUT"


__all__ = [
    "AsyncPersonPhotoEvidenceRecorder",
    "AsyncPersonPhotoEvidenceStatus",
]
