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
import os
import queue
import threading
import time
from collections import deque
from collections.abc import Mapping
from pathlib import Path
from types import MappingProxyType
from typing import Any, Callable

from v3.config_types import LidarProcessConfig
from v3.contracts import QualityState, RobotEstimate
from v3.async_capability import TransportSemantics, latest_state_snapshot
from v3.runtime_performance import CpuSet, normalize_cpus, apply_process_cpuset, temporary_current_affinity
from v3.navigation_map_capture import navigation_map_observation

from .latest_lidar import MATCHER_CONFIDENCE_MODEL, MATCHER_CONTRACT_ID, MATCHER_TRANSPORT
from .live_lidar import LidarRelativeMotionReading
from .native_lidar_port import (
    NativeLidarPortConfig,
    NativeMatcherResult,
    NativeRawLidarSnapshot,
    TimedPoseReference,
    open_native_lidar_port,
    rotation_deskew_points,
)
from .rplidar_c1 import RplidarPoint
from .slam_toolbox_transport import (
    SlamLocalizationReceiver, SlamOdometryPacket, SlamToolboxSender,
)

_PROCESS_START_METHOD = "spawn"


def _slam_matcher_result(receiver: SlamLocalizationReceiver, now_ns: int) -> NativeMatcherResult | None:
    """Adapt an accepted scan-bound pose to the existing compact L1 input."""
    from .latest_lidar import (
        POSE_FRAME_ID, POSE_FRAME_OWNER, POSE_FRAME_YAW,
        SLAM_CONTRACT_ID, SLAM_CONFIDENCE_MODEL, SLAM_TRANSPORT,
    )
    pose = receiver.last_pose
    timing = receiver.last_scan_timing
    if pose is None or timing is None or receiver.error:
        return None
    start, end, measured = timing
    confidence = 1.0 / (1.0 + max(pose.variance_x_m2, pose.variance_y_m2, pose.variance_yaw_rad2))
    summary = MappingProxyType({
        "matcher_contract_id": SLAM_CONTRACT_ID,
        "matcher_confidence_model": SLAM_CONFIDENCE_MODEL,
        "matcher_transport": SLAM_TRANSPORT,
        "map_frame_id": POSE_FRAME_ID,
        "map_frame_owner": POSE_FRAME_OWNER,
        "yaw_convention": POSE_FRAME_YAW,
        "lidar_pose_x": pose.x_m, "lidar_pose_y": pose.y_m,
        "lidar_pose_theta": pose.yaw_rad, "lidar_pose_confidence": confidence,
        "variance_x_m2": pose.variance_x_m2,
        "variance_y_m2": pose.variance_y_m2,
        "variance_yaw_rad2": pose.variance_yaw_rad2,
        "tracking_ready": True,
        "matcher_reason": "SLAM_POSE_ACCEPTED",
        "matcher_confidence": confidence,
        "slam_generation": pose.generation,
        "slam_bridge_generation": pose.bridge_generation,
        "slam_session_id": pose.session_id,
        "source_scan_revision": pose.source_sequence,
        "scan_start_monotonic_ns": start, "scan_end_monotonic_ns": end,
        "measurement_monotonic_ns": measured,
        "pose_reference_monotonic_ns": measured,
        "scan_pose_alignment_delta_ns": 0,
    })
    return NativeMatcherResult(
        receiver.accepted_packets, receiver.accepted_packets, pose.source_sequence,
        end / 1_000_000_000.0, start, end, measured, measured,
        now_ns / 1_000_000_000.0, summary,
    )


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
        tuple((p.angle_deg, p.distance_m, p.quality, p.received_monotonic_ns)
              for p in snapshot.raw_scan),
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
            RplidarPoint(float(point[0]), float(point[1]), int(point[2]),
                         None if len(point) == 3 or point[3] is None else int(point[3]))
            for point in points
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


def _wire_relative(result: LidarRelativeMotionReading | None) -> object | None:
    if result is None:
        return None
    return tuple(getattr(result, name) for name in result.__dataclass_fields__)


def _unwire_relative(value: object | None) -> LidarRelativeMotionReading | None:
    return None if value is None else LidarRelativeMotionReading(*value)


class _SharedPoseHistory:
    __slots__ = ("_capacity", "_lock", "_sequence", "_times", "_values", "_lock_timeout_s")

    def __init__(self, capacity: int, lock: Any, sequence: Any, times: Any, values: Any, lock_timeout_s: float) -> None:
        self._capacity = capacity
        self._lock_timeout_s = lock_timeout_s
        self._lock = lock
        self._sequence = sequence
        self._times = times
        self._values = values

    def publish(self, pose: TimedPoseReference) -> None:
        if not isinstance(pose, TimedPoseReference):
            raise TypeError("pose must be TimedPoseReference")
        if not self._lock.acquire(timeout=self._lock_timeout_s):
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
        if not 0 < span <= 100_000_000:
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
                 superseded_count: int, timeout_s: float) -> None:
    marker = ("raw_end", int(last_revision), int(produced_count), int(superseded_count))
    try:
        target.put(marker, timeout=timeout_s)
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
        target.put(marker, timeout=timeout_s)
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
    worker_cpus: int | CpuSet | None,
    matcher_cpus: CpuSet | None,
    strict_affinity: bool,
    control_minimum_range_m: float,
    control_maximum_range_m: float,
    control_maximum_points: int,
    process_config: LidarProcessConfig,
    capture_map_hz: int = 0,
    slam_socket_path: str | None = None,
    slam_session_id: int = 0,
    slam_odometry_state: Any | None = None,
) -> None:
    port = None
    raw_produced_count = 0
    raw_superseded_count = 0
    last_raw_revision = 0
    last_map_measurement_ns = None
    slam_enabled = config.global_backend == "slam_toolbox"
    slam_sender = SlamToolboxSender(
        slam_socket_path, reply_session_id=slam_session_id if slam_enabled else None)
    slam_receiver = SlamLocalizationReceiver(slam_session_id, config.maximum_result_age_ns)
    slam_result = None
    deskew_reason = "NO_SCAN"
    deskew_applied_scans = 0
    deskew_uncompensated_scans = 0

    pending_geometry = None

    def derived_scan(raw, now_ns):
        nonlocal deskew_reason, deskew_applied_scans, deskew_uncompensated_scans
        if raw is None or not slam_enabled:
            return raw, False
        from dataclasses import replace
        points, deskew_reason = rotation_deskew_points(
            raw.raw_scan, scan_start_ns=raw.scan_start_monotonic_ns,
            scan_end_ns=raw.scan_end_monotonic_ns,
            measurement_ns=raw.measurement_monotonic_ns,
            pose_provider=pose_history.lookup,
            lidar_x_m=config.matcher.lidar_x_offset_m,
            lidar_y_m=config.matcher.lidar_y_offset_m)
        pending = (deskew_reason == "HEADING_BRACKET_UNAVAILABLE"
                   and now_ns-raw.scan_end_monotonic_ns <= 40_000_000)
        unavailable = deskew_reason not in {"APPLIED", "POINT_TIMING_UNAVAILABLE"}
        if deskew_reason == "HEADING_BRACKET_UNAVAILABLE" and not pending:
            deskew_reason = "HEADING_BRACKET_TIMEOUT"
        if not pending:
            deskew_applied_scans += int(deskew_reason == "APPLIED")
            deskew_uncompensated_scans += int(deskew_reason != "APPLIED")
        return replace(raw, raw_scan=() if unavailable else points, summary=MappingProxyType({
            **raw.summary, "rotation_deskew": deskew_reason,
            "local_geometry_pending": unavailable,
            "deskew_time_basis": "HOST_PACKET_RECEIPT",
            "serial_chunk_wire_time_ns": math.ceil(
                config.driver.read_chunk_size * 10 * 1e9 / config.driver.baudrate),
        })), pending

    def publish_slam_scan(raw, now_ns):
        if slam_session_id and raw is not None and not raw.summary.get("local_geometry_pending", False):
            sent = slam_sender.publish_scan(
                raw, now_ns=now_ns, session_id=slam_session_id, source_pid=os.getpid(),
                minimum_range_m=config.driver.minimum_distance_m,
                maximum_range_m=config.driver.maximum_distance_m,
                lidar_x_m=config.matcher.lidar_x_offset_m,
                lidar_y_m=config.matcher.lidar_y_offset_m,
                maximum_age_ns=config.maximum_result_age_ns,
            )
            if sent and slam_enabled:
                slam_receiver.remember_scan(raw)

    def receive_slam_pose(now_ns):
        nonlocal slam_result
        if not slam_enabled or slam_odometry_state is None:
            return
        sequence = slam_odometry_state[0]
        generation, odometry_ns, valid = slam_odometry_state[1:4]
        coherent = not sequence % 2 and sequence == slam_odometry_state[0]
        if (slam_receiver.error or (coherent and slam_receiver.last_pose is not None
                and generation != slam_receiver.last_pose.generation)):
            # A global pose from another local-odometry generation must not
            # survive until a later reply or an ordinary freshness timeout.
            slam_result = None
        for payload in slam_sender.receive():
            pose = slam_receiver.accept(
                payload, now_ns=now_ns,
                current_generation=generation if coherent else None,
                odometry_ns=odometry_ns, odometry_valid=bool(valid) and coherent,
            )
            if pose is not None or slam_receiver.error:
                slam_result = _slam_matcher_result(slam_receiver, now_ns)

    def runtime_status():
        status = dict(port.get_runtime_status())
        status["slam_scan_sent"] = slam_sender.sent_packets
        status["slam_scan_dropped"] = slam_sender.dropped_packets
        if slam_enabled:
            from .latest_lidar import SLAM_CONTRACT_ID, SLAM_CONFIDENCE_MODEL, SLAM_TRANSPORT
            status.update(
                matcher_contract_id=SLAM_CONTRACT_ID,
                matcher_confidence_model=SLAM_CONFIDENCE_MODEL,
                matcher_transport=SLAM_TRANSPORT,
                matcher_error=status.get("matcher_error") or slam_receiver.error,
                slam_pose_reason=slam_receiver.last_reason,
                slam_pose_accepted=slam_receiver.accepted_packets,
                slam_pose_dropped=slam_receiver.dropped_packets,
                rotation_deskew=deskew_reason,
                rotation_deskew_applied_scans=deskew_applied_scans,
                rotation_deskew_uncompensated_scans=deskew_uncompensated_scans,
                rotation_deskew_time_basis="HOST_PACKET_RECEIPT",
                serial_chunk_wire_time_ns=math.ceil(
                    config.driver.read_chunk_size * 10 * 1e9 / config.driver.baudrate),
            )
        return status

    def publish_evidence(raw, compact):
        nonlocal raw_produced_count, raw_superseded_count, last_map_measurement_ns
        if raw_queue is None or raw is None:
            return
        if capture_map_hz:
            if compact is None or compact[7].get("local_geometry_pending", False):
                return
            measured = raw.measurement_monotonic_ns
            period = 1_000_000_000 // capture_map_hz
            if (last_map_measurement_ns is not None
                    and (measured <= last_map_measurement_ns
                         or measured // period == last_map_measurement_ns // period)):
                return
            last_map_measurement_ns = measured
            observation = navigation_map_observation(
                raw, compact[6], minimum_range_m=control_minimum_range_m,
                maximum_range_m=control_maximum_range_m,
                x_offset_m=config.matcher.lidar_x_offset_m,
                y_offset_m=config.matcher.lidar_y_offset_m)
            observation["rotation_deskew"] = compact[7].get("rotation_deskew", "NOT_REQUESTED")
            observation["point_timing_basis"] = compact[7].get("deskew_time_basis", "UNAVAILABLE")
            payload = ("navigation_map", observation)
        else:
            payload = ("raw", _wire_raw(raw))
        raw_produced_count += 1
        raw_superseded_count = _put_raw_evidence(raw_queue, payload, raw_superseded_count)
    try:
        if worker_cpus is not None:
            apply_process_cpuset(worker_cpus, role="lidar-owner-process", strict=strict_affinity)
        import serial

        pose_history = _SharedPoseHistory(process_config.pose_history_capacity, pose_lock, pose_sequence, pose_times, pose_values, process_config.pose_lock_timeout_s)
        port = open_native_lidar_port(
            config, pose_history.lookup, serial.Serial,
            matcher_cpus=matcher_cpus, strict_affinity=strict_affinity,
        )
        initial_raw = port.get_raw_scan_snapshot()
        initial_geometry, geometry_pending = derived_scan(initial_raw, time.monotonic_ns())
        pending_geometry = initial_raw if geometry_pending else None
        initial_matcher = slam_result if slam_enabled else port.get_matcher_result()
        initial_relative = port.get_relative_result()
        initial_status = runtime_status()
        control_raw = _wire_control_raw(
            initial_geometry,
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
                _wire_relative(initial_relative),
            ),
        )
        publish_evidence(initial_raw, control_raw)
        publish_slam_scan(initial_geometry, time.monotonic_ns())
        ready_event.set()
        last_raw_revision = int(getattr(initial_raw, "raw_scan_id", 0) or 0)
        last_matcher_revision = int(getattr(initial_matcher, "matcher_result_id", 0) or 0)
        last_relative_revision = int(getattr(initial_relative, "source_sequence", 0) or 0)
        last_status_ns = time.monotonic_ns()
        while not stop_event.is_set():
            raw = port.get_raw_scan_snapshot()
            matcher = slam_result if slam_enabled else port.get_matcher_result()
            relative = port.get_relative_result()
            now_ns = time.monotonic_ns()
            raw_revision = int(getattr(raw, "raw_scan_id", 0) or 0)
            matcher_revision = int(getattr(matcher, "matcher_result_id", 0) or 0)
            relative_revision = int(getattr(relative, "source_sequence", 0) or 0)
            raw_changed = raw_revision != last_raw_revision
            geometry_changed = False
            if raw_changed and raw is not None:
                geometry, geometry_pending = derived_scan(raw, now_ns)
                pending_geometry = raw if geometry_pending else None
                publish_slam_scan(geometry, now_ns)
                control_raw = _wire_control_raw(
                    geometry,
                    minimum_range_m=control_minimum_range_m,
                    maximum_range_m=control_maximum_range_m,
                    maximum_points=control_maximum_points,
                )
                publish_evidence(raw, control_raw)
            elif pending_geometry is not None:
                geometry, geometry_pending = derived_scan(pending_geometry, now_ns)
                if not geometry_pending:
                    control_raw = _wire_control_raw(
                        geometry, minimum_range_m=control_minimum_range_m,
                        maximum_range_m=control_maximum_range_m,
                        maximum_points=control_maximum_points)
                    geometry_changed = True
                    publish_slam_scan(geometry, now_ns)
                    if capture_map_hz:
                        publish_evidence(pending_geometry, control_raw)
                    pending_geometry = None
            receive_slam_pose(now_ns)
            if slam_enabled:
                matcher = slam_result
                matcher_revision = int(getattr(matcher, "matcher_result_id", 0) or 0)
            if (raw_changed or geometry_changed or matcher_revision != last_matcher_revision
                    or relative_revision != last_relative_revision
                    or now_ns - last_status_ns >= process_config.state_heartbeat_ns):
                status = runtime_status()
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
                        _wire_relative(relative),
                    ),
                )
                last_raw_revision = raw_revision
                last_matcher_revision = matcher_revision
                last_relative_revision = relative_revision
                last_status_ns = now_ns
            stop_event.wait(0.005)
    except BaseException as exc:
        _put_latest(state_queue, ("error", type(exc).__name__, str(exc)))
        ready_event.set()
    finally:
        slam_sender.close()
        # The publication loop is closed. Seal this evidence lane before
        # driver/matcher cleanup can exhaust the parent's bounded stop budget.
        # No raw scan is published after this marker, even while the driver
        # finishes acquiring internally.
        try:
            if raw_queue is not None:
                _put_raw_end(
                    raw_queue,
                    last_revision=last_raw_revision,
                    produced_count=raw_produced_count,
                    timeout_s=process_config.raw_end_timeout_s,
                    superseded_count=raw_superseded_count,
                )
        finally:
            if port is not None:
                try:
                    port.stop()
                except BaseException as exc:
                    _put_latest(state_queue, ("error", type(exc).__name__, str(exc)))


class ProcessLidarPort:
    """Latest compact-control proxy plus revision-only full-raw capture lane."""

    transport_semantics = TransportSemantics.LATEST_STATE

    __slots__ = (
        "_config", "_fatal_error", "_matcher_result", "_relative_result", "_pose_history", "_process",
        "_raw_snapshot",
        "_ready_event", "_state_queue", "_raw_queue", "_status", "_stop_event",
        "_stopped", "_collector", "_collector_stop", "_pending_poses",
        "_process_config",
        "_slam_sender", "_slam_session_id", "_slam_source_pid", "_slam_active",
        "_pending_slam_odometry",
        "_slam_enabled", "_slam_odometry_state",
    )

    def __init__(
        self,
        config: NativeLidarPortConfig,
        pose_provider: Callable[[int], TimedPoseReference | None],
        *,
        worker_cpus: int | CpuSet | None = None,
        matcher_cpus: CpuSet | None = None,
        strict_affinity: bool = False,
        control_minimum_range_m: float = 0.08,
        control_maximum_range_m: float = 2.5,
        control_maximum_points: int = 96,
        capture_raw_queue: Any | None = None,
        capture_map_hz: int = 0,
        process_config: LidarProcessConfig,
        slam_socket_path: str | Path | None = None,
    ) -> None:
        if not isinstance(config, NativeLidarPortConfig):
            raise TypeError("config must be NativeLidarPortConfig")
        if not callable(pose_provider):
            raise TypeError("pose_provider must be callable")
        if worker_cpus is not None:
            worker_cpus = normalize_cpus(worker_cpus)
        if matcher_cpus is not None:
            matcher_cpus = normalize_cpus(matcher_cpus)
        if type(strict_affinity) is not bool:
            raise TypeError("strict_affinity must be bool")
        if type(capture_map_hz) is not int or not 0 <= capture_map_hz <= 5:
            raise ValueError("capture_map_hz must be an integer from 0 to 5")
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
        pose_times = context.RawArray("q", process_config.pose_history_capacity)
        pose_values = context.RawArray("d", process_config.pose_history_capacity * 3)
        self._pose_history = _SharedPoseHistory(process_config.pose_history_capacity, pose_lock, pose_sequence, pose_times, pose_values, process_config.pose_lock_timeout_s)
        self._config = config
        self._process_config = process_config
        self._state_queue = context.Queue(maxsize=process_config.state_queue_capacity)
        # No fallback raw mailbox: without an explicit sidecar consumer there
        # is no raw encoding, queue feeder, drain or capture work at all.
        self._raw_queue = capture_raw_queue
        self._ready_event = context.Event()
        self._stop_event = context.Event()
        # One shared lifetime identity ties child scans to parent local odometry.
        # A replacement port receives a new identity even within the same PID.
        self._slam_session_id = time.monotonic_ns()
        self._slam_source_pid = os.getpid()
        self._slam_enabled = config.global_backend == "slam_toolbox"
        # Collector-only writer, child reader. Sequence brackets the three
        # scalars so a generation transition cannot produce a mixed snapshot.
        self._slam_odometry_state = context.RawArray("Q", 4)
        self._slam_sender = SlamToolboxSender(slam_socket_path)
        self._slam_active = False
        self._pending_slam_odometry: deque[tuple] = deque(maxlen=1)
        self._process = context.Process(
            target=_lidar_owner_process_main,
            args=(
                config, pose_lock, pose_sequence, pose_times, pose_values,
                self._state_queue, self._raw_queue, self._ready_event, self._stop_event,
                worker_cpus, matcher_cpus, strict_affinity,
                float(control_minimum_range_m), float(control_maximum_range_m), int(control_maximum_points),
                process_config, capture_map_hz,
                None if slam_socket_path is None else os.fspath(slam_socket_path),
                self._slam_session_id,
                self._slam_odometry_state,
            ),
            name="v3-lidar-owner-process",
            daemon=False,
        )
        self._raw_snapshot: NativeRawLidarSnapshot | None = None
        self._matcher_result: NativeMatcherResult | None = None
        self._relative_result: LidarRelativeMotionReading | None = None
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
        self._pending_poses: deque[TimedPoseReference] = deque(maxlen=process_config.pose_history_capacity)
        self._collector: threading.Thread | None = None
        with temporary_current_affinity(worker_cpus, role="worker-start", strict=strict_affinity):
            self._process.start()
        # Parent receives only compact state when evidence is sent to a sidecar.
        try:
            self._state_queue._writer.close()
        except (AttributeError, OSError):
            pass
        ready_timeout = max(process_config.ready_timeout_s, config.process_ready_timeout_s)
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
                self._relative_result = _unwire_relative(first[4] if len(first) > 4 else None)
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
        with temporary_current_affinity(worker_cpus, role="lidar-ipc", strict=strict_affinity):
            self._collector.start()

    @property
    def config(self) -> NativeLidarPortConfig:
        return self._config

    def publish_pose_reference(self, pose: TimedPoseReference) -> None:
        if not isinstance(pose, TimedPoseReference):
            raise TypeError("pose must be TimedPoseReference")
        if not self._stopped:
            self._pending_poses.append(pose)

    def publish_slam_odometry(self, estimate: RobotEstimate) -> None:
        """Queue only completed local-pose scalars; no I/O in the control tick."""
        if self._stopped or not (self._slam_active or self._slam_enabled):
            return
        pose = estimate.local_pose
        if pose is None:
            return
        quality = estimate.localization_quality
        continuous = quality.local_pose_continuous and not quality.pose_discontinuity
        self._pending_slam_odometry.append((
            estimate.context.tick_id, estimate.context.monotonic_ns, quality.generation,
            pose.x_m, pose.y_m, pose.yaw_rad,
            continuous and quality.local_translation is not QualityState.LOST,
            continuous and quality.heading is not QualityState.LOST,
        ))

    def _collect_state(self) -> None:
        try:
            while not self._collector_stop.is_set():
                for _ in range(self._process_config.pose_history_capacity):
                    try:
                        pose = self._pending_poses.popleft()
                    except IndexError:
                        break
                    self._pose_history.publish(pose)
                self._drain_state()
                # Reuse this existing collector. Without a consumer the probe
                # is at most 1 Hz and no scan or odometry payload is created.
                now_ns = time.monotonic_ns()
                self._slam_active = self._slam_sender.available(now_ns)
                try:
                    odometry = self._pending_slam_odometry.popleft()
                except IndexError:
                    odometry = None
                if odometry is not None:
                    state = self._slam_odometry_state
                    state[0] += 1
                    state[1], state[2], state[3] = odometry[2], odometry[1], int(odometry[6] and odometry[7])
                    state[0] += 1
                if odometry is not None and self._slam_active:
                    try:
                        packet = SlamOdometryPacket(
                            self._slam_session_id, self._slam_source_pid, *odometry)
                        self._slam_sender.publish_odometry(packet, now_ns=now_ns)
                    except (ValueError, TypeError, OverflowError):
                        self._slam_sender.dropped_packets += 1
                    self._slam_active = self._slam_sender.active
                self._collector_stop.wait(0.005)
        except Exception as exc:
            self._fatal_error = f"LIDAR_COLLECTOR_FAILED:{type(exc).__name__}:{exc}"
        finally:
            self._slam_active = False
            self._slam_sender.close()

    def _drain_state(self) -> None:
        newest: object | None = None
        for _ in range(self._process_config.state_queue_capacity):
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
        wire = newest[1]  # type: ignore[index]
        if wire is not None:
            previous = self._raw_snapshot
            same_scan = previous is not None and wire[0] == previous.raw_scan_id
            if same_scan and tuple(wire[1:5]) != (
                previous.raw_scan_timestamp, previous.scan_start_monotonic_ns,
                previous.scan_end_monotonic_ns, previous.measurement_monotonic_ns,
            ):
                self._fatal_error = "LIDAR_PHYSICAL_SCAN_IDENTITY_REWRITTEN"
                return
            geometry_completed = (same_scan
                and previous.summary.get("local_geometry_pending", False)
                and not wire[7].get("local_geometry_pending", False))
            if not same_scan or geometry_completed:
                # Physical scan identity is immutable, but a deferred first
                # local geometry can complete after immediate raw safety. The
                # first local sample must reach L2 under that original identity.
                self._raw_snapshot = _unwire_raw(wire)
        self._matcher_result = _unwire_matcher(newest[2])  # type: ignore[index]
        self._relative_result = _unwire_relative(newest[4] if len(newest) > 4 else None)  # type: ignore[arg-type,index]
        status = dict(newest[3])  # type: ignore[index]
        status["process_isolated"] = True
        status["compact_control_transport"] = True
        status["owner_process_alive"] = self._process.is_alive()
        self._status = status

    def get_raw_scan_snapshot(self) -> NativeRawLidarSnapshot | None:
        """Compact physical scan used by control/local perception only."""
        return self._raw_snapshot

    def get_capture_raw_scan_snapshot(self) -> NativeRawLidarSnapshot | None:
        """Raw evidence goes directly to its requested sidecar, never control."""
        return None

    def get_matcher_result(self) -> NativeMatcherResult | None:
        return self._matcher_result

    def get_relative_result(self) -> LidarRelativeMotionReading | None:
        return self._relative_result

    def capability_snapshot(
        self,
        observed_monotonic_ns: int,
        *,
        stale_after_ns: int = 250_000_000,
    ):
        raw = self._raw_snapshot
        status = self.get_runtime_status()
        source_sequence = None if raw is None else int(raw.raw_scan_id)
        source_ns = None if raw is None else int(raw.measurement_monotonic_ns)
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
            stale=raw is not None and raw.health == "STALE",
        )

    def localization_capability_snapshot(self, observed_monotonic_ns: int, *, stale_after_ns: int = 250_000_000):
        result = self._matcher_result
        status = self.get_runtime_status()
        error = self._fatal_error or status.get("matcher_error") or None
        if not self._stopped and not status.get("matcher_process_alive", False):
            error = error or "LIDAR_MATCHER_NOT_RUNNING"
        return latest_state_snapshot(
            name="lidar.localization", observed_monotonic_ns=observed_monotonic_ns,
            source_sequence=None if result is None else result.matcher_result_id,
            source_monotonic_ns=None if result is None else result.measurement_monotonic_ns,
            stale_after_ns=stale_after_ns, running=not self._stopped, error=error,
            degraded=status.get("health") not in ("OK", "STALE"),
        )

    def get_runtime_status(self) -> dict[str, object]:
        status = dict(self._status)
        status["slam_odometry_sent"] = self._slam_sender.sent_packets
        status["slam_odometry_dropped"] = self._slam_sender.dropped_packets
        alive = self._process.is_alive() and not self._stopped
        status["owner_process_alive"] = alive
        status["capture_raw_enabled"] = self._raw_queue is not None
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
        self._process.join(timeout=self._process_config.stop_timeout_s)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=self._process_config.stop_timeout_s)
        self._collector_stop.set()
        if self._process.is_alive():
            raise RuntimeError("process-isolated LiDAR owner did not stop")
        if self._collector is not None:
            self._collector.join(timeout=self._process_config.stop_timeout_s)
            if self._collector.is_alive():
                raise RuntimeError("LiDAR state collector did not stop")
        self._slam_sender.close()
        for item in (self._state_queue,):
            try:
                item.close()
            except (OSError, ValueError):
                pass


def open_process_lidar_port(
    config: NativeLidarPortConfig,
    pose_provider: Callable[[int], TimedPoseReference | None],
    *,
    worker_cpus: int | CpuSet | None = None,
    matcher_cpus: CpuSet | None = None,
    strict_affinity: bool = False,
    control_minimum_range_m: float = 0.08,
    control_maximum_range_m: float = 2.5,
    control_maximum_points: int = 96,
    capture_raw_queue: Any | None = None,
    capture_map_hz: int = 0,
    process_config: LidarProcessConfig,
    slam_socket_path: str | Path | None = None,
) -> ProcessLidarPort:
    return ProcessLidarPort(
        config,
        pose_provider,
        worker_cpus=worker_cpus,
        matcher_cpus=matcher_cpus,
        strict_affinity=strict_affinity,
        control_minimum_range_m=control_minimum_range_m,
        control_maximum_range_m=control_maximum_range_m,
        control_maximum_points=control_maximum_points,
        capture_raw_queue=capture_raw_queue,
        capture_map_hz=capture_map_hz,
        process_config=process_config,
        slam_socket_path=slam_socket_path,
    )


__all__ = ["ProcessLidarPort", "open_process_lidar_port"]
