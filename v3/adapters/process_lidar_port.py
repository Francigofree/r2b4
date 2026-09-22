"""Process-isolated production LiDAR owner with compact control transport.

The child owns pyserial, the RPLIDAR driver, scan pump and matcher.  Two bounded
transport lanes are exposed:
- compact control state: safety/localization plus <= configured local points;
- full raw scan: revision-only evidence lane for passive capture.
The control reader never needs to receive/rebuild the full physical scan.
"""
from __future__ import annotations

import math
import multiprocessing
import queue
import threading
import time
from collections import deque
from collections.abc import Mapping
from typing import Any, Callable

from v3.async_capability import TransportSemantics, latest_state_snapshot
from v3.runtime_performance import apply_current_affinity, temporary_current_affinity

from .latest_lidar import MATCHER_CONFIDENCE_MODEL, MATCHER_CONTRACT_ID, MATCHER_TRANSPORT
from .native_lidar_port import (
    NativeLidarPortConfig,
    NativeMatcherResult,
    NativeRawLidarSnapshot,
    TimedPoseReference,
    open_native_lidar_port,
)
from .rplidar_c1 import RplidarPoint

_PROCESS_START_METHOD = "spawn"
_POSE_HISTORY_CAPACITY = 64
_STATE_QUEUE_CAPACITY = 2
_RAW_QUEUE_CAPACITY = 2
_STATE_HEARTBEAT_NS = 100_000_000
_READY_TIMEOUT_S = 12.0
_STOP_TIMEOUT_S = 3.0


def _plain(value: object) -> object:
    if isinstance(value, Mapping):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return tuple(_plain(item) for item in value)
    return value


def _bounded_control_points(
    points: tuple[RplidarPoint, ...],
    *,
    minimum_range_m: float,
    maximum_range_m: float,
    maximum_points: int,
) -> tuple[RplidarPoint, ...]:
    filtered = tuple(
        sorted(
            (
                point
                for point in points
                if minimum_range_m <= float(point.distance_m) <= maximum_range_m
            ),
            key=lambda point: (float(point.angle_deg) % 360.0, float(point.distance_m), -int(point.quality)),
        )
    )
    if len(filtered) <= maximum_points:
        return filtered
    bucket_width_deg = 360.0 / maximum_points
    selected: dict[int, int] = {}
    for index, point in enumerate(filtered):
        angle = float(point.angle_deg) % 360.0
        bucket = min(maximum_points - 1, int(angle / bucket_width_deg))
        previous = selected.get(bucket)
        if previous is None:
            selected[bucket] = index
            continue
        old = filtered[previous]
        if (float(point.distance_m), -int(point.quality), angle, index) < (
            float(old.distance_m), -int(old.quality), float(old.angle_deg) % 360.0, previous
        ):
            selected[bucket] = index
    selected_indices = set(selected.values())
    if len(selected_indices) < maximum_points:
        remaining = [index for index in range(len(filtered)) if index not in selected_indices]
        missing = maximum_points - len(selected_indices)
        if missing >= len(remaining):
            selected_indices.update(remaining)
        elif missing == 1:
            selected_indices.add(remaining[len(remaining) // 2])
        elif remaining:
            last = len(remaining) - 1
            selected_indices.update(remaining[round(slot * last / (missing - 1))] for slot in range(missing))
    return tuple(
        sorted(
            (filtered[index] for index in selected_indices),
            key=lambda point: (float(point.angle_deg) % 360.0, float(point.distance_m), -int(point.quality)),
        )
    )


def _wire_raw(snapshot: NativeRawLidarSnapshot | None) -> object | None:
    if snapshot is None:
        return None
    return (
        snapshot.raw_scan_id,
        snapshot.raw_scan_timestamp,
        snapshot.scan_start_monotonic_ns,
        snapshot.scan_end_monotonic_ns,
        snapshot.measurement_monotonic_ns,
        snapshot.health,
        tuple((p.angle_deg, p.distance_m, p.quality) for p in snapshot.raw_scan),
        _plain(snapshot.summary),
        snapshot.observed_monotonic_ns,
    )


def _wire_control_raw(
    snapshot: NativeRawLidarSnapshot | None,
    *,
    minimum_range_m: float,
    maximum_range_m: float,
    maximum_points: int,
) -> object | None:
    if snapshot is None:
        return None
    bounded = _bounded_control_points(
        tuple(snapshot.raw_scan),
        minimum_range_m=minimum_range_m,
        maximum_range_m=maximum_range_m,
        maximum_points=maximum_points,
    )
    return (
        snapshot.raw_scan_id,
        snapshot.raw_scan_timestamp,
        snapshot.scan_start_monotonic_ns,
        snapshot.scan_end_monotonic_ns,
        snapshot.measurement_monotonic_ns,
        snapshot.health,
        tuple((p.angle_deg, p.distance_m, p.quality) for p in bounded),
        _plain(snapshot.summary),
        snapshot.observed_monotonic_ns,
    )


def _unwire_raw(value: object | None) -> NativeRawLidarSnapshot | None:
    if value is None:
        return None
    (
        raw_scan_id,
        raw_scan_timestamp,
        scan_start_ns,
        scan_end_ns,
        measurement_ns,
        health,
        points,
        summary,
        observed_ns,
    ) = value  # type: ignore[misc]
    return NativeRawLidarSnapshot(
        raw_scan_id=int(raw_scan_id),
        raw_scan_timestamp=float(raw_scan_timestamp),
        scan_start_monotonic_ns=int(scan_start_ns),
        scan_end_monotonic_ns=int(scan_end_ns),
        measurement_monotonic_ns=int(measurement_ns),
        health=str(health),
        raw_scan=tuple(
            RplidarPoint(float(angle), float(distance), int(quality))
            for angle, distance, quality in points
        ),
        summary=dict(summary),
        observed_monotonic_ns=(None if observed_ns is None else int(observed_ns)),
    )


def _wire_matcher(result: NativeMatcherResult | None) -> object | None:
    if result is None:
        return None
    return (
        result.matcher_result_id,
        result.candidate_id,
        result.source_raw_scan_id,
        result.source_raw_scan_timestamp,
        result.scan_start_monotonic_ns,
        result.scan_end_monotonic_ns,
        result.measurement_monotonic_ns,
        result.pose_reference_monotonic_ns,
        result.timestamp,
        _plain(result.summary),
    )


def _unwire_matcher(value: object | None) -> NativeMatcherResult | None:
    if value is None:
        return None
    (
        matcher_result_id,
        candidate_id,
        source_raw_scan_id,
        source_raw_scan_timestamp,
        scan_start_ns,
        scan_end_ns,
        measurement_ns,
        pose_reference_ns,
        timestamp,
        summary,
    ) = value  # type: ignore[misc]
    return NativeMatcherResult(
        matcher_result_id=int(matcher_result_id),
        candidate_id=int(candidate_id),
        source_raw_scan_id=int(source_raw_scan_id),
        source_raw_scan_timestamp=float(source_raw_scan_timestamp),
        scan_start_monotonic_ns=int(scan_start_ns),
        scan_end_monotonic_ns=int(scan_end_ns),
        measurement_monotonic_ns=int(measurement_ns),
        pose_reference_monotonic_ns=int(pose_reference_ns),
        timestamp=float(timestamp),
        summary=dict(summary),
    )


class _SharedPoseHistory:
    __slots__ = ("_capacity", "_lock", "_sequence", "_times", "_values")

    def __init__(self, capacity: int, lock: Any, sequence: Any, times: Any, values: Any) -> None:
        self._capacity = capacity
        self._lock = lock
        self._sequence = sequence
        self._times = times
        self._values = values

    def publish(self, pose: TimedPoseReference) -> None:
        if not isinstance(pose, TimedPoseReference):
            raise TypeError("pose must be TimedPoseReference")
        if not self._lock.acquire(timeout=0.05):
            raise RuntimeError("LiDAR pose history publisher lock timed out")
        try:
            sequence = int(self._sequence.value)
            index = sequence % self._capacity
            self._times[index] = int(pose.monotonic_ns)
            base = index * 3
            self._values[base] = float(pose.x_m)
            self._values[base + 1] = float(pose.y_m)
            self._values[base + 2] = float(pose.yaw_rad)
            self._sequence.value = sequence + 1
        finally:
            self._lock.release()

    def lookup(self, monotonic_ns: int) -> TimedPoseReference | None:
        if not isinstance(monotonic_ns, int) or isinstance(monotonic_ns, bool) or monotonic_ns < 0:
            raise ValueError("monotonic_ns must be a non-negative integer")
        with self._lock:
            sequence = int(self._sequence.value)
            count = min(sequence, self._capacity)
            first = sequence - count
            poses = []
            for logical in range(first, sequence):
                index = logical % self._capacity
                base = index * 3
                poses.append(
                    TimedPoseReference(
                        int(self._times[index]),
                        float(self._values[base]),
                        float(self._values[base + 1]),
                        float(self._values[base + 2]),
                    )
                )
        if not poses or monotonic_ns < poses[0].monotonic_ns or monotonic_ns > poses[-1].monotonic_ns:
            return None
        lo, hi = 0, len(poses)
        while lo < hi:
            mid = (lo + hi) // 2
            if poses[mid].monotonic_ns < monotonic_ns:
                lo = mid + 1
            else:
                hi = mid
        index = lo
        if index < len(poses) and poses[index].monotonic_ns == monotonic_ns:
            return poses[index]
        if index == 0 or index >= len(poses):
            return None
        before, after = poses[index - 1], poses[index]
        span = after.monotonic_ns - before.monotonic_ns
        if span <= 0:
            return None
        fraction = (monotonic_ns - before.monotonic_ns) / span
        yaw_delta = math.atan2(math.sin(after.yaw_rad - before.yaw_rad), math.cos(after.yaw_rad - before.yaw_rad))
        yaw = before.yaw_rad + fraction * yaw_delta
        return TimedPoseReference(
            monotonic_ns,
            before.x_m + fraction * (after.x_m - before.x_m),
            before.y_m + fraction * (after.y_m - before.y_m),
            math.atan2(math.sin(yaw), math.cos(yaw)),
        )


def _put_latest(target: Any, payload: object) -> None:
    try:
        target.put_nowait(payload)
        return
    except queue.Full:
        pass
    try:
        target.get_nowait()
    except queue.Empty:
        pass
    try:
        target.put_nowait(payload)
    except queue.Full:
        pass


def _put_raw_evidence(target: Any, payload: object, superseded_count: int) -> int:
    try:
        target.put_nowait(payload)
        return superseded_count
    except queue.Full:
        pass
    try:
        target.get_nowait()
    except queue.Empty:
        pass
    else:
        superseded_count += 1
    try:
        target.put_nowait(payload)
    except queue.Full:
        superseded_count += 1
    return superseded_count


def _put_raw_end(target: Any, *, last_revision: int, produced_count: int,
                 superseded_count: int) -> None:
    marker = ("raw_end", int(last_revision), int(produced_count), int(superseded_count))
    try:
        target.put(marker, timeout=0.5)
        return
    except queue.Full:
        pass
    try:
        target.get_nowait()
    except queue.Empty:
        pass
    else:
        superseded_count += 1
    marker = ("raw_end", int(last_revision), int(produced_count), int(superseded_count))
    try:
        target.put(marker, timeout=0.5)
    except queue.Full:
        return


def _lidar_owner_process_main(
    config: NativeLidarPortConfig,
    pose_lock: Any,
    pose_sequence: Any,
    pose_times: Any,
    pose_values: Any,
    state_queue: Any,
    raw_queue: Any,
    ready_event: Any,
    stop_event: Any,
    worker_cpu: int | None,
    strict_affinity: bool,
    control_minimum_range_m: float,
    control_maximum_range_m: float,
    control_maximum_points: int,
) -> None:
    port = None
    raw_produced_count = 0
    raw_superseded_count = 0
    last_raw_revision = 0
    try:
        if worker_cpu is not None:
            apply_current_affinity(worker_cpu, role="lidar-owner-process", strict=strict_affinity)
        import serial

        pose_history = _SharedPoseHistory(_POSE_HISTORY_CAPACITY, pose_lock, pose_sequence, pose_times, pose_values)
        port = open_native_lidar_port(config, pose_history.lookup, serial.Serial)
        initial_raw = port.get_raw_scan_snapshot()
        initial_matcher = port.get_matcher_result()
        initial_status = dict(port.get_runtime_status())
        control_raw = _wire_control_raw(
            initial_raw,
            minimum_range_m=control_minimum_range_m,
            maximum_range_m=control_maximum_range_m,
            maximum_points=control_maximum_points,
        )
        _put_latest(
            state_queue,
            (
                "state",
                control_raw,
                _wire_matcher(initial_matcher),
                initial_status,
            ),
        )
        if initial_raw is not None:
            raw_produced_count += 1
            raw_superseded_count = _put_raw_evidence(
                raw_queue, ("raw", _wire_raw(initial_raw)), raw_superseded_count
            )
        ready_event.set()
        last_raw_revision = int(getattr(initial_raw, "raw_scan_id", 0) or 0)
        last_matcher_revision = int(getattr(initial_matcher, "matcher_result_id", 0) or 0)
        last_status_ns = time.monotonic_ns()
        while not stop_event.is_set():
            raw = port.get_raw_scan_snapshot()
            matcher = port.get_matcher_result()
            now_ns = time.monotonic_ns()
            raw_revision = int(getattr(raw, "raw_scan_id", 0) or 0)
            matcher_revision = int(getattr(matcher, "matcher_result_id", 0) or 0)
            raw_changed = raw_revision != last_raw_revision
            if raw_changed and raw is not None:
                raw_produced_count += 1
                raw_superseded_count = _put_raw_evidence(
                    raw_queue, ("raw", _wire_raw(raw)), raw_superseded_count
                )
                control_raw = _wire_control_raw(
                    raw,
                    minimum_range_m=control_minimum_range_m,
                    maximum_range_m=control_maximum_range_m,
                    maximum_points=control_maximum_points,
                )
            if raw_changed or matcher_revision != last_matcher_revision or now_ns - last_status_ns >= _STATE_HEARTBEAT_NS:
                status = dict(port.get_runtime_status())
                _put_latest(
                    state_queue,
                    (
                        "state",
                        # Latest-only delivery can discard any older message.
                        # Every state must therefore carry a complete snapshot,
                        # including matcher-only and heartbeat publications.
                        control_raw,
                        _wire_matcher(matcher),
                        status,
                    ),
                )
                last_raw_revision = raw_revision
                last_matcher_revision = matcher_revision
                last_status_ns = now_ns
            stop_event.wait(0.005)
    except BaseException as exc:
        _put_latest(state_queue, ("error", type(exc).__name__, str(exc)))
        ready_event.set()
    finally:
        if port is not None:
            try:
                port.stop()
            except BaseException as exc:
                _put_latest(state_queue, ("error", type(exc).__name__, str(exc)))
        _put_raw_end(
            raw_queue,
            last_revision=last_raw_revision,
            produced_count=raw_produced_count,
            superseded_count=raw_superseded_count,
        )


class ProcessLidarPort:
    """Latest compact-control proxy plus revision-only full-raw capture lane."""

    transport_semantics = TransportSemantics.LATEST_STATE

    __slots__ = (
        "_config", "_fatal_error", "_matcher_result", "_pose_history", "_process",
        "_raw_snapshot", "_capture_raw_snapshot", "_capture_raw_revision",
        "_ready_event", "_state_queue", "_raw_queue", "_status", "_stop_event",
        "_stopped", "_collector", "_collector_stop", "_pending_poses",
        "_external_raw_queue",
    )

    def __init__(
        self,
        config: NativeLidarPortConfig,
        pose_provider: Callable[[int], TimedPoseReference | None],
        *,
        worker_cpu: int | None = None,
        strict_affinity: bool = False,
        control_minimum_range_m: float = 0.08,
        control_maximum_range_m: float = 2.5,
        control_maximum_points: int = 96,
        capture_raw_queue: Any | None = None,
    ) -> None:
        if not isinstance(config, NativeLidarPortConfig):
            raise TypeError("config must be NativeLidarPortConfig")
        if not callable(pose_provider):
            raise TypeError("pose_provider must be callable")
        if worker_cpu is not None and (not isinstance(worker_cpu, int) or isinstance(worker_cpu, bool) or worker_cpu < 0):
            raise ValueError("worker_cpu must be non-negative or None")
        if type(strict_affinity) is not bool:
            raise TypeError("strict_affinity must be bool")
        for value, name in ((control_minimum_range_m, "control_minimum_range_m"), (control_maximum_range_m, "control_maximum_range_m")):
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if control_minimum_range_m < 0 or control_maximum_range_m <= control_minimum_range_m:
            raise ValueError("control range must be increasing")
        if not isinstance(control_maximum_points, int) or isinstance(control_maximum_points, bool) or control_maximum_points <= 0:
            raise ValueError("control_maximum_points must be positive")

        context = multiprocessing.get_context(_PROCESS_START_METHOD)
        pose_lock = context.Lock()
        pose_sequence = context.RawValue("Q", 0)
        pose_times = context.RawArray("q", _POSE_HISTORY_CAPACITY)
        pose_values = context.RawArray("d", _POSE_HISTORY_CAPACITY * 3)
        self._pose_history = _SharedPoseHistory(_POSE_HISTORY_CAPACITY, pose_lock, pose_sequence, pose_times, pose_values)
        self._config = config
        self._state_queue = context.Queue(maxsize=_STATE_QUEUE_CAPACITY)
        self._external_raw_queue = capture_raw_queue is not None
        self._raw_queue = (
            capture_raw_queue if self._external_raw_queue
            else context.Queue(maxsize=_RAW_QUEUE_CAPACITY)
        )
        self._ready_event = context.Event()
        self._stop_event = context.Event()
        self._process = context.Process(
            target=_lidar_owner_process_main,
            args=(
                config, pose_lock, pose_sequence, pose_times, pose_values,
                self._state_queue, self._raw_queue, self._ready_event, self._stop_event,
                worker_cpu, strict_affinity,
                float(control_minimum_range_m), float(control_maximum_range_m), int(control_maximum_points),
            ),
            name="v3-lidar-owner-process",
            daemon=False,
        )
        self._raw_snapshot: NativeRawLidarSnapshot | None = None
        self._capture_raw_snapshot: NativeRawLidarSnapshot | None = None
        self._capture_raw_revision = 0
        self._matcher_result: NativeMatcherResult | None = None
        self._status: dict[str, object] = {
            "matcher_contract_id": MATCHER_CONTRACT_ID,
            "matcher_confidence_model": MATCHER_CONFIDENCE_MODEL,
            "matcher_transport": MATCHER_TRANSPORT,
            "running": False,
            "matcher_process_alive": False,
            "driver_connected": False,
            "health": "STALE",
            "process_isolated": True,
            "compact_control_transport": True,
        }
        self._fatal_error = ""
        self._stopped = False
        self._collector_stop = threading.Event()
        self._pending_poses: deque[TimedPoseReference] = deque(maxlen=_POSE_HISTORY_CAPACITY)
        self._collector: threading.Thread | None = None
        self._process.start()
        # Parent receives only compact state when evidence is sent to a sidecar.
        try:
            self._state_queue._writer.close()
            if not self._external_raw_queue:
                self._raw_queue._writer.close()
        except (AttributeError, OSError):
            pass
        ready_timeout = max(_READY_TIMEOUT_S, config.process_ready_timeout_s)
        if not self._ready_event.wait(ready_timeout):
            self.stop()
            raise RuntimeError("process-isolated LiDAR owner did not become ready")
        try:
            first = self._state_queue.get(timeout=ready_timeout)
        except queue.Empty:
            first = None
        if first is not None:
            if first[0] == "error":
                self._fatal_error = f"{first[1]}:{first[2]}"
            elif first[0] == "state":
                self._raw_snapshot = _unwire_raw(first[1])
                self._matcher_result = _unwire_matcher(first[2])
                self._status = dict(first[3])
                self._status["process_isolated"] = True
                self._status["compact_control_transport"] = True
            else:
                self._fatal_error = "UNKNOWN_LIDAR_PROCESS_MESSAGE"
        self._drain_state()
        if self._fatal_error:
            error = self._fatal_error
            self.stop()
            raise RuntimeError(f"process-isolated LiDAR startup failed: {error}")
        if not self._process.is_alive():
            self.stop()
            raise RuntimeError("process-isolated LiDAR owner exited during startup")
        self._collector = threading.Thread(target=self._collect_state, name="r2b4-lidar-ipc", daemon=True)
        with temporary_current_affinity(worker_cpu, role="lidar-ipc", strict=strict_affinity):
            self._collector.start()

    @property
    def config(self) -> NativeLidarPortConfig:
        return self._config

    def publish_pose_reference(self, pose: TimedPoseReference) -> None:
        if not isinstance(pose, TimedPoseReference):
            raise TypeError("pose must be TimedPoseReference")
        if not self._stopped:
            self._pending_poses.append(pose)

    def _collect_state(self) -> None:
        try:
            while not self._collector_stop.is_set():
                for _ in range(_POSE_HISTORY_CAPACITY):
                    try:
                        pose = self._pending_poses.popleft()
                    except IndexError:
                        break
                    self._pose_history.publish(pose)
                self._drain_state()
                self._collector_stop.wait(0.005)
        except Exception as exc:
            self._fatal_error = f"LIDAR_COLLECTOR_FAILED:{type(exc).__name__}:{exc}"

    def _drain_state(self) -> None:
        newest: object | None = None
        for _ in range(_STATE_QUEUE_CAPACITY):
            try:
                message = self._state_queue.get_nowait()
            except queue.Empty:
                break
            if message[0] == "error":
                self._fatal_error = f"{message[1]}:{message[2]}"
            elif message[0] != "state":
                self._fatal_error = f"UNKNOWN_LIDAR_PROCESS_MESSAGE:{message[0]}"
            else:
                newest = message
        if newest is None:
            if not self._process.is_alive() and not self._stopped:
                self._fatal_error = self._fatal_error or "LIDAR_OWNER_PROCESS_EXITED"
            return
        if newest[1] is not None and (  # type: ignore[index]
            self._raw_snapshot is None
            or newest[1][0] != self._raw_snapshot.raw_scan_id  # type: ignore[index]
        ):
            self._raw_snapshot = _unwire_raw(newest[1])  # type: ignore[index]
        self._matcher_result = _unwire_matcher(newest[2])  # type: ignore[index]
        status = dict(newest[3])  # type: ignore[index]
        status["process_isolated"] = True
        status["compact_control_transport"] = True
        status["owner_process_alive"] = self._process.is_alive()
        self._status = status

    def _drain_capture_raw(self) -> None:
        newest: object | None = None
        for _ in range(_RAW_QUEUE_CAPACITY):
            try:
                newest = self._raw_queue.get_nowait()
            except queue.Empty:
                break
        if newest is None:
            return
        if not isinstance(newest, tuple) or not newest:
            self._fatal_error = "LIDAR_RAW_TRANSPORT_INVALID"
            return
        if newest[0] == "raw_end":
            if len(newest) == 4:
                self._status["capture_raw_transport_end"] = True
                self._status["capture_raw_produced_count"] = int(newest[2])
                self._status["capture_raw_superseded_count"] = int(newest[3])
                return
            self._fatal_error = "LIDAR_RAW_TRANSPORT_INVALID"
            return
        if len(newest) != 2 or newest[0] != "raw":
            self._fatal_error = "LIDAR_RAW_TRANSPORT_INVALID"
            return
        snapshot = _unwire_raw(newest[1])
        if snapshot is not None:
            self._capture_raw_snapshot = snapshot
            self._capture_raw_revision = int(snapshot.raw_scan_id)

    def get_raw_scan_snapshot(self) -> NativeRawLidarSnapshot | None:
        """Compact physical scan used by control/local perception only."""
        return self._raw_snapshot

    def get_capture_raw_scan_snapshot(self) -> NativeRawLidarSnapshot | None:
        """Full latest physical scan for passive capture; never used by L0 control."""
        if self._external_raw_queue:
            return None
        self._drain_capture_raw()
        return self._capture_raw_snapshot

    def get_matcher_result(self) -> NativeMatcherResult | None:
        return self._matcher_result

    def capability_snapshot(
        self,
        observed_monotonic_ns: int,
        *,
        stale_after_ns: int = 250_000_000,
    ):
        raw = self._raw_snapshot
        status = self.get_runtime_status()
        source_sequence = None if raw is None else int(raw.raw_scan_id)
        source_ns = None if raw is None else int(raw.scan_end_monotonic_ns)
        error = self._fatal_error or (
            "LIDAR_NOT_RUNNING" if status.get("health") == "ERROR" else None
        )
        return latest_state_snapshot(
            name="lidar.control",
            observed_monotonic_ns=observed_monotonic_ns,
            source_sequence=source_sequence,
            source_monotonic_ns=source_ns,
            stale_after_ns=stale_after_ns,
            running=bool(status.get("running", False)),
            error=error,
            degraded=status.get("health") == "DEGRADED",
        )

    def get_runtime_status(self) -> dict[str, object]:
        status = dict(self._status)
        alive = self._process.is_alive() and not self._stopped
        status["owner_process_alive"] = alive
        status["capture_raw_revision"] = self._capture_raw_revision
        if not alive:
            status["running"] = False
            status["matcher_process_alive"] = False
            status["driver_connected"] = False
            status["health"] = "ERROR"
        if self._fatal_error:
            status["fatal_error"] = self._fatal_error
            status["running"] = False
        return status

    def stop(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        self._stop_event.set()
        # Keep draining compact state while the child flushes its queue feeders.
        # Stopping the reader first can strand the child in Queue finalization.
        self._process.join(timeout=_STOP_TIMEOUT_S)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=_STOP_TIMEOUT_S)
        self._collector_stop.set()
        if self._process.is_alive():
            raise RuntimeError("process-isolated LiDAR owner did not stop")
        if self._collector is not None:
            self._collector.join(timeout=_STOP_TIMEOUT_S)
            if self._collector.is_alive():
                raise RuntimeError("LiDAR state collector did not stop")
        owned_queues = (
            (self._state_queue,) if self._external_raw_queue
            else (self._state_queue, self._raw_queue)
        )
        for item in owned_queues:
            try:
                item.close()
            except (OSError, ValueError):
                pass


def open_process_lidar_port(
    config: NativeLidarPortConfig,
    pose_provider: Callable[[int], TimedPoseReference | None],
    *,
    worker_cpu: int | None = None,
    strict_affinity: bool = False,
    control_minimum_range_m: float = 0.08,
    control_maximum_range_m: float = 2.5,
    control_maximum_points: int = 96,
    capture_raw_queue: Any | None = None,
) -> ProcessLidarPort:
    return ProcessLidarPort(
        config,
        pose_provider,
        worker_cpu=worker_cpu,
        strict_affinity=strict_affinity,
        control_minimum_range_m=control_minimum_range_m,
        control_maximum_range_m=control_maximum_range_m,
        control_maximum_points=control_maximum_points,
        capture_raw_queue=capture_raw_queue,
    )


__all__ = ["ProcessLidarPort", "open_process_lidar_port"]
