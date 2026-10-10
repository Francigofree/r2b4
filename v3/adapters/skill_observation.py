"""Connection-owned observation and event clips in the existing vision edge.

Only compact events and file references leave this process. The detector and
calibrated JPEG producer are the existing camera owner's capabilities.
"""
from __future__ import annotations

from collections import deque
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
import uuid


def _seconds(value, name, maximum=30.0):
    if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= maximum:
        raise ValueError(f"{name} must be finite and within [0, {maximum}]")
    return float(value)


def _region(value):
    if value is None:
        return {"xmin": 0.0, "ymin": 0.0, "xmax": 1.0, "ymax": 1.0}
    if not isinstance(value, dict) or set(value) != {"xmin", "ymin", "xmax", "ymax"}:
        raise RuntimeError("OBSERVATION_REGION_UNAVAILABLE: use a normalized image-space box")
    result = {name: _seconds(value[name], name, 1.0) for name in value}
    if result["xmin"] >= result["xmax"] or result["ymin"] >= result["ymax"]:
        raise ValueError("observation region must have positive area")
    return result


class PersonObserver:
    def __init__(self, owner, *, region=None, object_kind="person", ready_timeout_s=5.0):
        if object_kind != "person":
            raise RuntimeError(f"DETECTOR_UNAVAILABLE:{object_kind}")
        self.owner = owner
        self.region = _region(region)
        self.handle = uuid.uuid4().hex
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._events = deque(maxlen=256)
        self._sequence = 0
        self._source_sequence = 0
        self._last_measurement = 0
        self._last_source_time = time.monotonic()
        self._present = False
        self._gap = False
        self._until = None
        self._finished = False
        self._partial = False
        self._failed = None
        self._closed = False
        self._ready = threading.Event()
        self._started_ns = time.monotonic_ns()
        self._demand = owner.acquire(person=True)
        self.generation = owner.generation
        self._thread = threading.Thread(target=self._run, name="skill-person-observer", daemon=True)
        self._thread.start()
        if not self._ready.wait(_seconds(ready_timeout_s, "ready_timeout_s")):
            self.close()
            raise RuntimeError("OBSERVER_NOT_READY: no fresh detector measurement")
        if self._failed:
            self.close()
            raise RuntimeError(self._failed)

    def _emit(self, kind, measurement_ns, **details):
        self._sequence += 1
        self._events.append({"kind": kind, "sequence": self._sequence,
            "event_id": f"{self.handle}:{self._sequence}", "observer": self.handle,
            "measurement_monotonic_ns": measurement_ns,
            "source_sequence": self._source_sequence, "owner_generation": self.generation,
            **details})
        self._condition.notify_all()

    def _run(self):
        try:
            while not self._stop.is_set():
                state = self.owner.control_state(self.generation)
                status = state["detection_status"]
                if not status["running"] or status.get("last_error"):
                    raise RuntimeError(status.get("last_error") or "PERSON_DETECTOR_STOPPED")
                detection = state.get("detection")
                with self._condition:
                    if detection and detection["sequence"] > self._source_sequence:
                        measured = detection["measurement_monotonic_ns"]
                        if detection.get("owner_generation") != self.generation:
                            raise RuntimeError("VISION_GENERATION_MISMATCH")
                        if self._until is not None and measured > self._until:
                            self._finished = True
                            self._condition.notify_all()
                            return
                        skipped = max(0, detection["sequence"] - self._source_sequence - 1)
                        had_source = self._source_sequence > 0
                        self._source_sequence = detection["sequence"]
                        if had_source and skipped:
                            self._emit("coverage_gap", measured, reason="DETECTOR_RESULTS_SUPERSEDED",
                                       lost_results=skipped)
                        self._last_measurement = measured
                        self._last_source_time = time.monotonic()
                        fresh = 0 <= time.monotonic_ns() - measured <= self.owner.maximum_age_ns
                        if not fresh or measured < self._started_ns:
                            if not self._gap:
                                self._emit("coverage_gap", measured, reason="DETECTOR_MEASUREMENT_STALE")
                            self._gap = True
                            self._present = False
                        else:
                            self._gap = False
                            self._ready.set()
                            qualified = []
                            for item in detection["detections"]:
                                box = item["box"]
                                x = (box["xmin"] + box["xmax"]) / 2
                                y = (box["ymin"] + box["ymax"]) / 2
                                if (self.region["xmin"] <= x <= self.region["xmax"]
                                        and self.region["ymin"] <= y <= self.region["ymax"]):
                                    qualified.append(item)
                            if qualified and not self._present:
                                self._emit("qualified_presence", measured, object_kind="person",
                                    detections=qualified[:4], source_frame_sequence=detection["source_frame_sequence"],
                                    completed_monotonic_ns=detection["completed_monotonic_ns"],
                                    geometry_state=detection.get("geometry_state"), region=self.region)
                            self._present = bool(qualified)
                    elif time.monotonic() - self._last_source_time > 1.0 and not self._gap:
                        self._gap = True
                        self._present = False
                        self._emit("coverage_gap", time.monotonic_ns(), reason="DETECTOR_MEASUREMENT_MISSING")
                self._stop.wait(0.02)
        except Exception as exc:
            with self._condition:
                self._failed = f"{type(exc).__name__}:{exc}"[:256]
                self._emit("capability_failed", time.monotonic_ns(), reason=self._failed)
                self._ready.set()
                self._finished = True

    def page(self, after_sequence=0, limit=32):
        if type(after_sequence) is not int or after_sequence < 0:
            raise ValueError("after_sequence must be a nonnegative integer")
        if type(limit) is not int or not 1 <= limit <= 32:
            raise ValueError("event limit must be within [1, 32]")
        with self._condition:
            latest_sequence = (max((event["sequence"] for event in self._events), default=0)
                               if self._finished and self._until is not None else self._sequence)
            oldest = self._events[0]["sequence"] if self._events else latest_sequence + 1
            return {"handle": self.handle, "status": "FAILED" if self._failed else "CLOSED" if self._closed
                    else "FINISHED" if self._finished else "READY",
                "owner_generation": self.generation, "monotonic_ns": time.monotonic_ns(),
                "clock_domain": "host_monotonic",
                "events": [dict(event) for event in self._events if event["sequence"] > after_sequence][:limit],
                "latest_sequence": latest_sequence, "lost_events": max(0, oldest - after_sequence - 1),
                "finished": self._finished, "reason": self._failed, "region": self.region,
                "object_kind": "person", "last_measurement_monotonic_ns": self._last_measurement}

    def finish(self, *, until_monotonic_ns, timeout_s=5.0, after_sequence=0, limit=32):
        if type(until_monotonic_ns) is not int or until_monotonic_ns < self._started_ns:
            raise ValueError("observer deadline must be a monotonic measurement time after open")
        deadline = time.monotonic() + _seconds(timeout_s, "timeout_s")
        with self._condition:
            if self._until is not None and self._until != until_monotonic_ns:
                raise ValueError("observer interval has already been closed")
            self._until = until_monotonic_ns
            # Remove post-deadline events generated before the finish request.
            self._events = deque((event for event in self._events
                if event["measurement_monotonic_ns"] <= self._until), maxlen=256)
            if self._last_measurement > self._until:
                self._finished = True
                self._stop.set()
            while not self._finished:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self._emit("coverage_gap", self._until, reason="OBSERVER_FINISH_TIMEOUT")
                    self._partial = True
                    self._finished = True
                    self._stop.set()
                    break
                self._condition.wait(remaining)
        result = self.page(after_sequence, limit)
        result["partial"] = self._partial
        return result

    def require_event(self, event):
        if not isinstance(event, dict):
            raise ValueError("event must be an observer event")
        with self._condition:
            original = next((item for item in self._events if item["event_id"] == event.get("event_id")), None)
            if original is None or original != event or original["kind"] != "qualified_presence":
                raise RuntimeError("EVENT_EVIDENCE_EXPIRED_OR_INVALID")
            return dict(original)

    def close(self):
        self._stop.set()
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._finished = True
            self._condition.notify_all()
        self._thread.join(timeout=0.25)
        self.owner.release(self._demand)


class EventRecorder:
    """Small bounded prebuffer and clips; encoding never returns image bytes."""

    def __init__(self, observer, capture, *, pre_s=2.0, post_s=3.0, fps=5.0,
                 output_dir=None, ready_timeout_s=5.0):
        self.observer = observer
        self.capture = capture
        self.pre_s = _seconds(pre_s, "pre_s", 10)
        self.post_s = _seconds(post_s, "post_s", 30)
        self.fps = _seconds(fps, "fps", 10)
        if self.fps < 0.1:
            raise ValueError("fps must be within [0.1, 10]")
        self._encoder = shutil.which("ffmpeg")
        if self._encoder is None:
            raise RuntimeError("MEDIA_ENCODER_UNAVAILABLE: ffmpeg is required")
        self.handle = uuid.uuid4().hex
        self.output_dir = (Path(output_dir).expanduser().resolve() if output_dir else
                           Path(tempfile.gettempdir()) / f"r2b4-skill-media-{self.handle}")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._buffer = deque(maxlen=max(2, math.ceil(self.pre_s * self.fps) + 2))
        self._clips = {}
        self._accepted_events = {}
        self._total_clips = 0
        self._completed_clips = 0
        self._failed_clips = 0
        self._partial_clips = 0
        self._results_path = self.output_dir / f"{self.handle}-clips.jsonl"
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._changed = threading.Condition(self._lock)
        self._finishing = False
        self._error = None
        self._process = None
        self._last_sequence = 0
        self._last_measurement = 0
        self._geometry = None
        self._coverage_gaps = 0
        self._thread = threading.Thread(target=self._run, name="skill-event-recorder", daemon=True)
        self._encode_thread = threading.Thread(target=self._encode_loop, name="skill-clip-encoder", daemon=True)
        self._thread.start()
        self._encode_thread.start()
        if not self._ready.wait(_seconds(ready_timeout_s, "ready_timeout_s")):
            self.close()
            raise RuntimeError("RECORDER_NOT_READY")
        if self._error:
            self.close()
            raise RuntimeError(self._error)

    def start(self, event):
        original = self.observer.require_event(event)
        with self._lock:
            if self._finishing or self._stop.is_set() or self._error:
                raise RuntimeError(self._error or "RECORDER_CLOSED")
            existing = self._accepted_events.get(original["event_id"])
            if existing:
                return {"status": "ACCEPTED", "clip_id": existing, "handle": self.handle}
            if sum(clip["status"] in {"RECORDING", "ENCODING", "QUEUED"}
                   for clip in self._clips.values()) >= 2:
                raise RuntimeError("RECORDER_CLIP_CAPACITY_EXCEEDED")
            if len(self._clips) >= 64:
                oldest = next(key for key, clip in self._clips.items()
                              if clip["status"] not in {"RECORDING", "ENCODING", "QUEUED"})
                del self._clips[oldest]
            clip_id = uuid.uuid4().hex
            measured = original["measurement_monotonic_ns"]
            start_ns = measured - round(self.pre_s * 1e9)
            end_ns = measured + round(self.post_s * 1e9)
            frames = [image for image in self._buffer
                      if start_ns <= image.metadata.measurement_monotonic_ns <= end_ns]
            clip = {"clip_id": clip_id, "event": original, "status": "RECORDING",
                    "start_ns": start_ns, "end_ns": end_ns, "images": frames,
                    "pre_partial": not frames or frames[0].metadata.measurement_monotonic_ns > start_ns + round(1e9/self.fps),
                    "post_partial": False, "reason": None}
            self._clips[clip_id] = clip
            self._accepted_events[original["event_id"]] = clip_id
            if len(self._accepted_events) > 256:
                del self._accepted_events[next(iter(self._accepted_events))]
            self._total_clips += 1
            if time.monotonic_ns() >= end_ns:
                clip["status"] = "QUEUED"
            self._changed.notify_all()
            return {"status": "ACCEPTED", "clip_id": clip_id, "handle": self.handle,
                    "event_id": original["event_id"]}

    def _run(self):
        try:
            while not self._stop.is_set():
                image = self.capture("lores", self.observer.generation)
                if self._stop.is_set():
                    return
                metadata = image.metadata
                with self._lock:
                    if metadata.owner_generation != self.observer.generation:
                        raise RuntimeError("VISION_GENERATION_MISMATCH")
                    geometry = (metadata.width, metadata.height, metadata.calibration_id)
                    if self._geometry is not None and self._geometry != geometry:
                        raise RuntimeError("CALIBRATED_CAMERA_GEOMETRY_CHANGED")
                    self._geometry = geometry
                    if metadata.source_sequence > self._last_sequence:
                        frame_gap = (self._last_measurement > 0 and
                                     metadata.measurement_monotonic_ns - self._last_measurement > 2e9/self.fps)
                        if frame_gap:
                            self._coverage_gaps += 1
                        self._last_measurement = metadata.measurement_monotonic_ns
                        self._last_sequence = metadata.source_sequence
                        self._buffer.append(image)
                        # Bound the raw prebuffer independently of frame count.
                        while sum(len(item.image_bytes) for item in self._buffer) > 8 * 1024 * 1024:
                            self._buffer.popleft()
                        for clip in self._clips.values():
                            if clip["status"] != "RECORDING":
                                continue
                            if frame_gap:
                                clip["post_partial"] = True
                                clip["reason"] = "MEDIA_MEASUREMENT_GAP"
                            if clip["start_ns"] <= metadata.measurement_monotonic_ns <= clip["end_ns"]:
                                clip["images"].append(image)
                                if sum(len(item.image_bytes) for item in clip["images"]) > 16 * 1024 * 1024:
                                    clip["post_partial"] = True
                                    clip["reason"] = "CLIP_BUFFER_CAPACITY_EXCEEDED"
                                    clip["status"] = "QUEUED"
                            elif metadata.measurement_monotonic_ns > clip["end_ns"]:
                                clip["status"] = "QUEUED"
                        self._ready.set()
                        self._changed.notify_all()
                    active = any(clip["status"] == "RECORDING" for clip in self._clips.values())
                    if self._finishing and not active:
                        return
                self._stop.wait(1 / self.fps)
        except Exception as exc:
            with self._lock:
                self._error = f"{type(exc).__name__}:{exc}"[:256]
                for clip in self._clips.values():
                    if clip["status"] == "RECORDING":
                        clip["post_partial"] = True
                        clip["reason"] = self._error
                        clip["status"] = "QUEUED"
                self._ready.set()
                self._changed.notify_all()

    def _encode_loop(self):
        while not self._stop.is_set():
            with self._changed:
                clip = next((item for item in self._clips.values() if item["status"] == "QUEUED"), None)
                if clip is None:
                    self._changed.wait(0.05)
                    continue
                clip["status"] = "ENCODING"
            try:
                self._encode(clip)
            except Exception as exc:
                with self._lock:
                    clip["status"] = "FAILED"
                    clip["reason"] = f"{type(exc).__name__}:{exc}"[:256]
            finally:
                with self._changed:
                    clip.pop("images", None)
                    self._process = None
                    if clip["status"] == "COMPLETED":
                        self._completed_clips += 1
                    elif clip["status"] == "PARTIAL":
                        self._partial_clips += 1
                    else:
                        self._failed_clips += 1
                    result = {key: value for key, value in clip.items() if key not in {"start_ns", "end_ns"}}
                try:
                    with self._results_path.open("a", encoding="utf-8") as output:
                        output.write(json.dumps(result, allow_nan=False, separators=(",", ":")) + "\n")
                except OSError as exc:
                    with self._lock:
                        self._error = f"MEDIA_RESULT_SAVE_FAILED:{type(exc).__name__}:{exc}"[:256]
                with self._changed:
                    self._changed.notify_all()

    def _encode(self, clip):
        from .camera_media import _write_encoder_frame
        images = clip["images"]
        if not images:
            raise RuntimeError("CLIP_HAS_NO_FRAMES")
        target = self.output_dir / f"{clip['clip_id']}.mp4"
        # Reserve the output exclusively; never overwrite existing evidence.
        with target.open("xb"):
            pass
        fd, name = tempfile.mkstemp(prefix=".clip-", suffix=".mp4", dir=self.output_dir)
        os.close(fd)
        staging = Path(name)
        try:
            with tempfile.TemporaryFile() as errors:
                process = subprocess.Popen([self._encoder, "-hide_banner", "-loglevel", "error", "-y",
                    "-f", "image2pipe", "-vcodec", "mjpeg", "-framerate", str(self.fps), "-i", "pipe:0",
                    "-an", "-c:v", "libx264", "-threads", "1", "-preset", "ultrafast",
                    "-pix_fmt", "yuv420p", str(staging)], stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL, stderr=errors)
                with self._lock:
                    self._process = process
                os.set_blocking(process.stdin.fileno(), False)
                try:
                    for image in images:
                        if self._stop.is_set():
                            raise RuntimeError("RECORDER_CANCELLED")
                        _write_encoder_frame(process.stdin.fileno(), image.image_bytes, 0.25)
                    process.stdin.close()
                    code = process.wait(timeout=5)
                    if code or not staging.stat().st_size:
                        errors.seek(0)
                        raise RuntimeError("CLIP_ENCODER_FAILED:" + errors.read(500).decode("utf-8", "replace"))
                    with target.open("r+b") as destination, staging.open("rb") as source:
                        shutil.copyfileobj(source, destination)
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.wait(timeout=1)
                    if process.stdin is not None and not process.stdin.closed:
                        process.stdin.close()
            with self._lock:
                clip.update(status="PARTIAL" if clip["pre_partial"] or clip["post_partial"] else "COMPLETED",
                    output=str(target), bytes=target.stat().st_size, frames=len(images),
                    first_frame=images[0].metadata.to_jsonable(), last_frame=images[-1].metadata.to_jsonable())
        finally:
            staging.unlink(missing_ok=True)

    def status(self):
        with self._lock:
            clips = [{key: value for key, value in clip.items() if key not in {"images", "start_ns", "end_ns"}}
                     for clip in self._clips.values()]
            pending = any(clip["status"] in {"RECORDING", "QUEUED", "ENCODING"} for clip in clips)
            partial = bool(pending or self._partial_clips or self._failed_clips or self._error)
            return {"handle": self.handle, "status": "FAILED" if self._error else "PARTIAL" if self._finishing and partial
                    else "FINISHED" if self._finishing else "READY", "partial": partial,
                    "reason": self._error, "clips": clips[-8:], "clips_total": self._total_clips,
                    "clips_completed": self._completed_clips, "clips_partial": self._partial_clips,
                    "clips_failed": self._failed_clips, "clip_results": str(self._results_path),
                    "coverage_gaps": self._coverage_gaps,
                    "clips_omitted": max(0, len(clips)-8), "owner_generation": self.observer.generation}

    def finish(self, timeout_s=5.0):
        deadline = time.monotonic() + _seconds(timeout_s, "timeout_s")
        with self._changed:
            self._finishing = True
            while any(clip["status"] in {"RECORDING", "QUEUED", "ENCODING"} for clip in self._clips.values()):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._changed.wait(remaining)
        return self.status()

    def close(self):
        self._stop.set()
        with self._changed:
            process = self._process
            if process is not None and process.poll() is None:
                process.kill()
            self._buffer.clear()
            for clip in self._clips.values():
                if clip["status"] in {"RECORDING", "QUEUED"}:
                    clip["status"] = "CANCELLED"
                    clip["reason"] = "RECORDER_CLOSED"
                    clip.pop("images", None)
            self._changed.notify_all()
        self._thread.join(timeout=0.25)
        self._encode_thread.join(timeout=0.25)


class ObservationSessions:
    """One socket's sessions; disconnect always cleans its own demands."""

    def __init__(self, owner, capture):
        self.owner = owner
        self.capture = capture
        self.observers = {}
        self.recorders = {}

    def call(self, name, parameters):
        params = dict(parameters)
        if name == "observe.open":
            if len(self.observers) >= 4:
                raise RuntimeError("OBSERVER_SESSION_CAPACITY_EXCEEDED")
            observer = PersonObserver(self.owner, **params)
            self.observers[observer.handle] = observer
            return observer.page()
        if name == "media.recorder.open":
            observer = self.observers[params.pop("observer")]
            if len(self.recorders) >= 1:
                raise RuntimeError("RECORDER_SESSION_CAPACITY_EXCEEDED")
            recorder = EventRecorder(observer, self.capture, **params)
            self.recorders[recorder.handle] = recorder
            return recorder.status()
        handle = params.pop("handle")
        if name.startswith("observe."):
            observer = self.observers[handle]
            if name in {"observe.events", "observe.status"}:
                return observer.page(**params)
            if name == "observe.finish":
                return observer.finish(**params)
            if name == "observe.close":
                if params:
                    raise ValueError("observe.close accepts only handle")
                for recorder_id, recorder in tuple(self.recorders.items()):
                    if recorder.observer is observer:
                        recorder.close()
                        del self.recorders[recorder_id]
                observer.close()
                del self.observers[handle]
                return {"handle": handle, "status": "CLOSED"}
        elif name.startswith("media.recorder."):
            recorder = self.recorders[handle]
            if name == "media.recorder.start_event_clip":
                return recorder.start(**params)
            if name == "media.recorder.finish":
                return recorder.finish(**params)
            if name == "media.recorder.status" and not params:
                return recorder.status()
            if name == "media.recorder.close" and not params:
                recorder.close()
                del self.recorders[handle]
                return {"handle": handle, "status": "CLOSED"}
        raise ValueError("unknown observation operation")

    def close(self):
        errors = []
        for recorder in self.recorders.values():
            try:
                recorder.close()
            except Exception as exc:
                errors.append(exc)
        for observer in self.observers.values():
            try:
                observer.close()
            except Exception as exc:
                errors.append(exc)
        self.recorders.clear()
        self.observers.clear()
        if errors:
            raise errors[0]
