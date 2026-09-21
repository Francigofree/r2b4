"""Process-isolated production LiDAR owner with a latest-only parent proxy.

The child owns pyserial, the RPLIDAR driver, the native pump thread and the
existing matcher process.  The control process only publishes completed L3 pose
references into a tiny shared-memory ring and reads immutable latest snapshots.
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

from v3.runtime_performance import apply_current_affinity, temporary_current_affinity

from .latest_lidar import (
    MATCHER_CONFIDENCE_MODEL,
    MATCHER_CONTRACT_ID,
    MATCHER_TRANSPORT,
)
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
_STATE_HEARTBEAT_NS = 100_000_000
_READY_TIMEOUT_S = 12.0
_STOP_TIMEOUT_S = 3.0


def _plain(value: object) -> object:
    if isinstance(value, Mapping):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return tuple(_plain(item) for item in value)
    return value


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
        lo = 0
        hi = len(poses)
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
        before = poses[index - 1]
        after = poses[index]
        span = after.monotonic_ns - before.monotonic_ns
        if span <= 0:
            return None
        fraction = (monotonic_ns - before.monotonic_ns) / span
        yaw_delta = math.atan2(
            math.sin(after.yaw_rad - before.yaw_rad),
            math.cos(after.yaw_rad - before.yaw_rad),
        )
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


def _lidar_owner_process_main(
    config: NativeLidarPortConfig,
    pose_lock: Any,
    pose_sequence: Any,
    pose_times: Any,
    pose_values: Any,
    state_queue: Any,
    ready_event: Any,
    stop_event: Any,
    worker_cpu: int | None,
    strict_affinity: bool,
) -> None:
    port = None
    try:
        if worker_cpu is not None:
            apply_current_affinity(
                worker_cpu,
                role="lidar-owner-process",
                strict=strict_affinity,
            )
        import serial

        pose_history = _SharedPoseHistory(
            _POSE_HISTORY_CAPACITY,
            pose_lock,
            pose_sequence,
            pose_times,
            pose_values,
        )
        port = open_native_lidar_port(config, pose_history.lookup, serial.Serial)
        initial_raw = port.get_raw_scan_snapshot()
        initial_matcher = port.get_matcher_result()
        initial_status = dict(port.get_runtime_status())
        _put_latest(
            state_queue,
            (
                "state",
                _wire_raw(initial_raw),
                _wire_matcher(initial_matcher),
                initial_status,
            ),
        )
        ready_event.set()
        last_raw_revision = int(getattr(initial_raw, "raw_scan_id", 0) or 0)
        last_matcher_revision = int(
            getattr(initial_matcher, "matcher_result_id", 0) or 0
        )
        last_status_ns = time.monotonic_ns()
        while not stop_event.is_set():
            raw = port.get_raw_scan_snapshot()
            matcher = port.get_matcher_result()
            now_ns = time.monotonic_ns()
            raw_revision = int(getattr(raw, "raw_scan_id", 0) or 0)
            matcher_revision = int(getattr(matcher, "matcher_result_id", 0) or 0)
            if (
                raw_revision != last_raw_revision
                or matcher_revision != last_matcher_revision
                or now_ns - last_status_ns >= _STATE_HEARTBEAT_NS
            ):
                status = dict(port.get_runtime_status())
                _put_latest(
                    state_queue,
                    (
                        "state",
                        _wire_raw(raw),
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


class ProcessLidarPort:
    """Read-only latest-result proxy for a fully separate LiDAR owner process."""

    __slots__ = (
        "_config",
        "_fatal_error",
        "_matcher_result",
        "_pose_history",
        "_process",
        "_raw_snapshot",
        "_ready_event",
        "_state_queue",
        "_status",
        "_stop_event",
        "_stopped",
        "_collector",
        "_collector_stop",
        "_pending_poses",
    )

    def __init__(
        self,
        config: NativeLidarPortConfig,
        pose_provider: Callable[[int], TimedPoseReference | None],
        *,
        worker_cpu: int | None = None,
        strict_affinity: bool = False,
    ) -> None:
        if not isinstance(config, NativeLidarPortConfig):
            raise TypeError("config must be NativeLidarPortConfig")
        if not callable(pose_provider):
            raise TypeError("pose_provider must be callable")
        if worker_cpu is not None and (
            not isinstance(worker_cpu, int) or isinstance(worker_cpu, bool) or worker_cpu < 0
        ):
            raise ValueError("worker_cpu must be non-negative or None")
        if type(strict_affinity) is not bool:
            raise TypeError("strict_affinity must be bool")

        context = multiprocessing.get_context(_PROCESS_START_METHOD)
        pose_lock = context.Lock()
        pose_sequence = context.RawValue("Q", 0)
        pose_times = context.RawArray("q", _POSE_HISTORY_CAPACITY)
        pose_values = context.RawArray("d", _POSE_HISTORY_CAPACITY * 3)
        self._pose_history = _SharedPoseHistory(
            _POSE_HISTORY_CAPACITY,
            pose_lock,
            pose_sequence,
            pose_times,
            pose_values,
        )
        self._config = config
        self._state_queue = context.Queue(maxsize=_STATE_QUEUE_CAPACITY)
        self._ready_event = context.Event()
        self._stop_event = context.Event()
        self._process = context.Process(
            target=_lidar_owner_process_main,
            args=(
                config,
                pose_lock,
                pose_sequence,
                pose_times,
                pose_values,
                self._state_queue,
                self._ready_event,
                self._stop_event,
                worker_cpu,
                strict_affinity,
            ),
            name="v3-lidar-owner-process",
            daemon=False,
        )
        self._raw_snapshot: NativeRawLidarSnapshot | None = None
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
        }
        self._fatal_error = ""
        self._stopped = False
        self._collector_stop = threading.Event()
        self._pending_poses: deque[TimedPoseReference] = deque(maxlen=_POSE_HISTORY_CAPACITY)
        self._collector: threading.Thread | None = None
        self._process.start()
        # This process only receives state. Closing its unused writer lets a
        # partial receive terminate on EOF if the child exits mid-message.
        self._state_queue._writer.close()
        if not self._ready_event.wait(max(_READY_TIMEOUT_S, config.process_ready_timeout_s)):
            self.stop()
            raise RuntimeError("process-isolated LiDAR owner did not become ready")
        self._drain_state()
        if self._fatal_error:
            self.stop()
            raise RuntimeError(f"process-isolated LiDAR startup failed: {self._fatal_error}")
        if not self._process.is_alive():
            self.stop()
            raise RuntimeError("process-isolated LiDAR owner exited during startup")
        self._collector = threading.Thread(
            target=self._collect_state,
            name="r2b4-lidar-ipc",
            daemon=True,
        )
        with temporary_current_affinity(
            worker_cpu, role="lidar-ipc", strict=strict_affinity
        ):
            self._collector.start()

    @property
    def config(self) -> NativeLidarPortConfig:
        return self._config

    def publish_pose_reference(self, pose: TimedPoseReference) -> None:
        if not isinstance(pose, TimedPoseReference):
            raise TypeError("pose must be TimedPoseReference")
        if self._stopped:
            return
        # The control observer never waits for the child-held pose-history lock.
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
                newest = self._state_queue.get_nowait()
            except queue.Empty:
                break
            if newest[0] == "error":
                self._fatal_error = f"{newest[1]}:{newest[2]}"
                return
        if newest is None:
            if not self._process.is_alive() and not self._stopped:
                self._fatal_error = self._fatal_error or "LIDAR_OWNER_PROCESS_EXITED"
            return
        kind = newest[0]  # type: ignore[index]
        if kind == "error":
            self._fatal_error = f"{newest[1]}:{newest[2]}"  # type: ignore[index]
            return
        if kind != "state":
            self._fatal_error = f"UNKNOWN_LIDAR_PROCESS_MESSAGE:{kind}"
            return
        raw = _unwire_raw(newest[1])  # type: ignore[index]
        matcher = _unwire_matcher(newest[2])  # type: ignore[index]
        status = dict(newest[3])  # type: ignore[index]
        status["process_isolated"] = True
        status["owner_process_alive"] = self._process.is_alive()
        self._raw_snapshot = raw
        self._matcher_result = matcher
        self._status = status

    def get_raw_scan_snapshot(self) -> NativeRawLidarSnapshot | None:
        return self._raw_snapshot

    def get_matcher_result(self) -> NativeMatcherResult | None:
        return self._matcher_result

    def get_runtime_status(self) -> dict[str, object]:
        status = dict(self._status)
        alive = self._process.is_alive() and not self._stopped
        status["owner_process_alive"] = alive
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
        self._collector_stop.set()
        self._stop_event.set()
        self._process.join(timeout=_STOP_TIMEOUT_S)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=_STOP_TIMEOUT_S)
        if self._process.is_alive():
            raise RuntimeError("process-isolated LiDAR owner did not stop")
        if self._collector is not None:
            self._collector.join(timeout=_STOP_TIMEOUT_S)
            if self._collector.is_alive():
                raise RuntimeError("LiDAR state collector did not stop")
        close = getattr(self._state_queue, "close", None)
        if callable(close):
            close()


def open_process_lidar_port(
    config: NativeLidarPortConfig,
    pose_provider: Callable[[int], TimedPoseReference | None],
    *,
    worker_cpu: int | None = None,
    strict_affinity: bool = False,
) -> ProcessLidarPort:
    """Open the production LiDAR process proxy or fail closed."""

    return ProcessLidarPort(
        config,
        pose_provider,
        worker_cpu=worker_cpu,
        strict_affinity=strict_affinity,
    )


__all__ = [
    "ProcessLidarPort",
    "open_process_lidar_port",
]
