"""Bounded R2B4 scan, odometry and localization datagrams for SLAM Toolbox.

Only the physical LiDAR owner encodes scans. Completed L3 local odometry is
encoded by the existing IPC collector; the control tick only queues scalars.
Only validated scan-bound poses return through the existing LiDAR/L2/L3 path.
The sidecar has no command, motor, local odometry or world-model authority.
"""
from __future__ import annotations

import math
import os
from pathlib import Path
import socket
import stat
import struct
from collections import OrderedDict
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .native_lidar_port import NativeRawLidarSnapshot

DEFAULT_SLAM_SOCKET = Path(__file__).resolve().parents[2] / "runtime" / "slam_toolbox.sock"
MAX_PACKET_BYTES = 4096
SCAN_BINS = 720
_PROBE_PERIOD_NS = 1_000_000_000
_MAGIC = b"R2ST"
_VERSION = 1
_SCAN = 1
_ODOMETRY = 2
_POSE = 3
_SCAN_HEADER = struct.Struct("<4sBBHQIQQQQffff")
_ODOMETRY_PACKET = struct.Struct("<4sBBHQIQQQddd")
_POSE_PACKET = struct.Struct("<4sBBHQQQQQdddddd")


def _uint(value: int, bits: int, name: str, *, positive: bool = False) -> None:
    if type(value) is not int or not int(positive) <= value < 2**bits:
        raise ValueError(f"invalid {name}")


@dataclass(frozen=True, slots=True)
class SlamScanPacket:
    session_id: int
    source_pid: int
    source_sequence: int
    scan_start_ns: int
    scan_end_ns: int
    measurement_ns: int
    minimum_range_m: float
    maximum_range_m: float
    lidar_x_m: float
    lidar_y_m: float
    # Bin zero faces +X; subsequent bins advance CCW in the sensor frame.
    # Zero means unobserved, below the sensor minimum and not a free ray.
    ranges_m: tuple[float, ...]

    def __post_init__(self) -> None:
        for name in ("session_id", "source_sequence"):
            _uint(getattr(self, name), 64, name, positive=True)
        _uint(self.source_pid, 32, "source_pid", positive=True)
        for name in ("scan_start_ns", "scan_end_ns", "measurement_ns"):
            _uint(getattr(self, name), 64, name)
        if (self.scan_start_ns > self.scan_end_ns or self.measurement_ns
                != self.scan_start_ns + (self.scan_end_ns-self.scan_start_ns)//2):
            raise ValueError("scan measurement must be the acquisition midpoint")
        if any(not math.isfinite(value) for value in (
                self.minimum_range_m, self.maximum_range_m, self.lidar_x_m, self.lidar_y_m)):
            raise ValueError("scan geometry must be finite")
        if not 0 <= self.minimum_range_m < self.maximum_range_m:
            raise ValueError("invalid scan range")
        if not isinstance(self.ranges_m, tuple) or not 1 <= len(self.ranges_m) <= SCAN_BINS:
            raise ValueError("invalid scan bin count")
        if any(value != 0.0 and (not math.isfinite(value)
               or not self.minimum_range_m <= value <= self.maximum_range_m)
               for value in self.ranges_m):
            raise ValueError("invalid scan range value")


@dataclass(frozen=True, slots=True)
class SlamOdometryPacket:
    session_id: int
    source_pid: int
    tick_id: int
    monotonic_ns: int
    generation: int
    x_m: float
    y_m: float
    yaw_rad: float
    local_translation_valid: bool
    heading_valid: bool

    def __post_init__(self) -> None:
        _uint(self.session_id, 64, "session_id", positive=True)
        _uint(self.source_pid, 32, "source_pid", positive=True)
        for name in ("tick_id", "monotonic_ns", "generation"):
            _uint(getattr(self, name), 64, name)
        if any(not math.isfinite(value) for value in (self.x_m, self.y_m, self.yaw_rad)):
            raise ValueError("odometry pose must be finite")
        if type(self.local_translation_valid) is not bool or type(self.heading_valid) is not bool:
            raise ValueError("odometry quality must be boolean")


@dataclass(frozen=True, slots=True)
class SlamPosePacket:
    session_id: int
    source_sequence: int
    measurement_ns: int
    generation: int
    x_m: float
    y_m: float
    yaw_rad: float
    variance_x_m2: float
    variance_y_m2: float
    variance_yaw_rad2: float
    bridge_generation: int

    def __post_init__(self) -> None:
        for name in ("session_id", "source_sequence", "bridge_generation"):
            _uint(getattr(self, name), 64, name, positive=True)
        for name in ("measurement_ns", "generation"):
            _uint(getattr(self, name), 64, name)
        if any(not math.isfinite(value) for value in (
                self.x_m, self.y_m, self.yaw_rad, self.variance_x_m2,
                self.variance_y_m2, self.variance_yaw_rad2)):
            raise ValueError("SLAM pose/covariance must be finite")
        if min(self.variance_x_m2, self.variance_y_m2, self.variance_yaw_rad2) <= 0:
            raise ValueError("SLAM covariance must be positive")


def encode_scan(
    snapshot: NativeRawLidarSnapshot, *, session_id: int, source_pid: int,
    minimum_range_m: float, maximum_range_m: float,
    lidar_x_m: float, lidar_y_m: float, bins: int = SCAN_BINS,
) -> bytes:
    """Convert one healthy physical scan, retaining closest returns per bin."""
    if snapshot.health != "OK":
        raise ValueError("only healthy physical scans may enter passive SLAM")
    if type(bins) is not int or not 1 <= bins <= SCAN_BINS:
        raise ValueError("invalid scan bin count")
    ranges = [0.0] * bins
    for point in snapshot.raw_scan:
        if (point.quality <= 0 or not math.isfinite(point.distance_m)
                or not minimum_range_m <= point.distance_m <= maximum_range_m
                or not math.isfinite(point.angle_deg)):
            continue
        # RPLIDAR turns clockwise; ROS LaserScan bins turn counterclockwise.
        index = int(((-point.angle_deg) % 360.0) * bins / 360.0 + 0.5) % bins
        if ranges[index] == 0.0 or point.distance_m < ranges[index]:
            ranges[index] = point.distance_m
    packet = SlamScanPacket(
        session_id, source_pid, snapshot.raw_scan_id,
        snapshot.scan_start_monotonic_ns, snapshot.scan_end_monotonic_ns,
        snapshot.measurement_monotonic_ns, minimum_range_m, maximum_range_m,
        lidar_x_m, lidar_y_m, tuple(ranges),
    )
    return _SCAN_HEADER.pack(
        _MAGIC, _VERSION, _SCAN, bins, packet.session_id, packet.source_pid,
        packet.source_sequence, packet.scan_start_ns, packet.scan_end_ns,
        packet.measurement_ns, packet.minimum_range_m, packet.maximum_range_m,
        packet.lidar_x_m, packet.lidar_y_m,
    ) + struct.pack(f"<{bins}f", *packet.ranges_m)


def encode_odometry(packet: SlamOdometryPacket) -> bytes:
    flags = int(packet.local_translation_valid) | int(packet.heading_valid) << 1
    return _ODOMETRY_PACKET.pack(
        _MAGIC, _VERSION, _ODOMETRY, flags, packet.session_id, packet.source_pid,
        packet.tick_id, packet.monotonic_ns, packet.generation,
        packet.x_m, packet.y_m, packet.yaw_rad,
    )


def encode_pose(packet: SlamPosePacket) -> bytes:
    return _POSE_PACKET.pack(
        _MAGIC, _VERSION, _POSE, 0, packet.session_id, packet.source_sequence,
        packet.measurement_ns, packet.generation, packet.bridge_generation,
        packet.x_m, packet.y_m, packet.yaw_rad,
        packet.variance_x_m2, packet.variance_y_m2, packet.variance_yaw_rad2,
    )


def decode_packet(data: bytes) -> SlamScanPacket | SlamOdometryPacket | SlamPosePacket:
    """Decode exact-size, versioned packets; malformed input is rejected."""
    if not 8 <= len(data) <= MAX_PACKET_BYTES:
        raise ValueError("invalid SLAM datagram size")
    magic, version, kind, count = struct.unpack_from("<4sBBH", data)
    if magic != _MAGIC or version != _VERSION:
        raise ValueError("unknown SLAM datagram contract")
    if kind == _SCAN:
        if not 1 <= count <= SCAN_BINS or len(data) != _SCAN_HEADER.size + count * 4:
            raise ValueError("invalid scan datagram size")
        fields = _SCAN_HEADER.unpack_from(data)[4:]
        ranges = struct.unpack_from(f"<{count}f", data, _SCAN_HEADER.size)
        return SlamScanPacket(*fields, ranges)
    if kind == _ODOMETRY:
        if count & ~3 or len(data) != _ODOMETRY_PACKET.size:
            raise ValueError("invalid odometry datagram")
        return SlamOdometryPacket(*_ODOMETRY_PACKET.unpack(data)[4:], bool(count & 1), bool(count & 2))
    if kind == _POSE:
        if count or len(data) != _POSE_PACKET.size:
            raise ValueError("invalid pose datagram")
        session, sequence, measured, generation, bridge, *pose = _POSE_PACKET.unpack(data)[4:]
        return SlamPosePacket(session, sequence, measured, generation, *pose, bridge)
    raise ValueError("unknown SLAM datagram kind")


class SlamToolboxSender:
    """A single optional nonblocking observation destination, never a queue."""

    def __init__(self, socket_path: str | Path | None = None, *, reply_session_id: int | None = None) -> None:
        self.path = os.fspath(DEFAULT_SLAM_SOCKET if socket_path is None else socket_path)
        self._socket: socket.socket | None = None
        self._reply_address = (None if reply_session_id is None
                               else f"\0r2b4-slam-{os.getpid()}-{reply_session_id}")
        self._next_probe_ns = 0
        self._last_scan_measurement_ns: int | None = None
        self.active = False
        self.sent_packets = 0
        self.dropped_packets = 0

    def available(self, now_ns: int) -> bool:
        if now_ns < self._next_probe_ns:
            return self.active
        self._next_probe_ns = now_ns + _PROBE_PERIOD_NS
        try:
            self.active = stat.S_ISSOCK(os.stat(self.path).st_mode)
            if self.active and self._socket is None:
                self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
                self._socket.setblocking(False)
                if self._reply_address is not None:
                    self._socket.bind(self._reply_address)
        except OSError:
            self.active = False
        if not self.active:
            self.close()
        return self.active

    def _send(self, payload: bytes) -> bool:
        try:
            if self._socket is None or self._socket.sendto(payload, self.path) != len(payload):
                raise OSError("SLAM datagram not delivered")
        except OSError:
            self.dropped_packets += 1
            self.close()
            return False
        self.sent_packets += 1
        return True

    def publish_scan(self, snapshot: NativeRawLidarSnapshot | None, *, now_ns: int,
                     session_id: int, source_pid: int, minimum_range_m: float,
                     maximum_range_m: float, lidar_x_m: float, lidar_y_m: float,
                     maximum_age_ns: int) -> bool:
        if snapshot is None or not self.available(now_ns):
            return False
        measured = snapshot.measurement_monotonic_ns
        if (self._last_scan_measurement_ns is not None
                and measured <= self._last_scan_measurement_ns):
            return False
        if snapshot.health != "OK" or not 0 <= now_ns-snapshot.measurement_monotonic_ns <= maximum_age_ns:
            self.dropped_packets += 1
            return False
        try:
            payload = encode_scan(
                snapshot, session_id=session_id, source_pid=source_pid,
                minimum_range_m=minimum_range_m, maximum_range_m=maximum_range_m,
                lidar_x_m=lidar_x_m, lidar_y_m=lidar_y_m,
            )
        except (ValueError, TypeError, OverflowError, struct.error):
            self.dropped_packets += 1
            return False
        sent = self._send(payload)
        if sent:
            self._last_scan_measurement_ns = measured
        return sent

    def publish_odometry(self, packet: SlamOdometryPacket, *, now_ns: int) -> bool:
        if not self.available(now_ns):
            return False
        try:
            payload = encode_odometry(packet)
        except (ValueError, TypeError, OverflowError, struct.error):
            self.dropped_packets += 1
            return False
        return self._send(payload)

    def receive(self, *, maximum_packets: int = 4) -> tuple[bytes, ...]:
        """Bounded replies from the same sidecar socket; never wait for one."""
        packets = []
        if self._socket is not None:
            for _ in range(maximum_packets):
                try:
                    payload, address = self._socket.recvfrom(MAX_PACKET_BYTES + 1)
                except BlockingIOError:
                    break
                except OSError:
                    self.dropped_packets += 1
                    self.close()
                    break
                if address == self.path:
                    packets.append(payload)
                else:
                    self.dropped_packets += 1
        return tuple(packets)

    def close(self) -> None:
        self.active = False
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None


class SlamLocalizationReceiver:
    """Validate compact poses against successfully exported physical scans."""

    def __init__(self, session_id: int, maximum_age_ns: int, *, maximum_odometry_age_ns: int | None = None) -> None:
        self.session_id = session_id
        self.maximum_age_ns = maximum_age_ns
        self.maximum_odometry_age_ns = maximum_age_ns if maximum_odometry_age_ns is None else maximum_odometry_age_ns
        self._scans: OrderedDict[int, tuple[int, int, int]] = OrderedDict()
        self._last_sequence = 0
        self.bridge_generation: int | None = None
        self.error = ""
        self.last_reason = "SLAM_POSE_UNAVAILABLE"
        self.accepted_packets = 0
        self.dropped_packets = 0
        self.last_pose: SlamPosePacket | None = None
        self.last_scan_timing: tuple[int, int, int] | None = None

    def remember_scan(self, snapshot: NativeRawLidarSnapshot) -> None:
        self._scans[snapshot.raw_scan_id] = (
            snapshot.scan_start_monotonic_ns, snapshot.scan_end_monotonic_ns,
            snapshot.measurement_monotonic_ns,
        )
        while len(self._scans) > 32:
            self._scans.popitem(last=False)

    def accept(self, data: bytes, *, now_ns: int, current_generation: int | None,
               odometry_ns: int, odometry_valid: bool) -> SlamPosePacket | None:
        if self.error:
            self.dropped_packets += 1
            return None
        try:
            packet = decode_packet(data)
        except (ValueError, TypeError, struct.error):
            self.last_reason = "SLAM_INVALID_PACKET"
            self.dropped_packets += 1
            return None
        reason = ""
        timing = self._scans.get(getattr(packet, "source_sequence", -1))
        if not isinstance(packet, SlamPosePacket) or packet.session_id != self.session_id:
            reason = "SLAM_SOURCE_SESSION_MISMATCH"
        elif timing is None or timing[2] != packet.measurement_ns:
            reason = "SLAM_UNKNOWN_SCAN"
        elif packet.source_sequence <= self._last_sequence:
            reason = "SLAM_DUPLICATE_OR_OUT_OF_ORDER"
        elif not 0 <= now_ns-packet.measurement_ns <= self.maximum_age_ns:
            reason = "SLAM_POSE_STALE"
        elif (not odometry_valid or current_generation is None
              or packet.generation != current_generation
              or not 0 <= now_ns-odometry_ns <= self.maximum_odometry_age_ns):
            reason = "SLAM_ODOMETRY_GENERATION_INVALID"
        elif self.bridge_generation is not None and packet.bridge_generation != self.bridge_generation:
            self.error = reason = "SLAM_BRIDGE_RESTART_REQUIRES_RUNTIME_RESTART"
        elif max(packet.variance_x_m2, packet.variance_y_m2, packet.variance_yaw_rad2) > 0.25:
            reason = "SLAM_POSE_COVARIANCE_REJECTED"
        if reason:
            self.last_reason = reason
            self.dropped_packets += 1
            return None
        self.bridge_generation = packet.bridge_generation
        self._last_sequence = packet.source_sequence
        self.last_pose = packet
        self.last_scan_timing = timing
        self.accepted_packets += 1
        self.last_reason = "SLAM_POSE_ACCEPTED"
        return packet


__all__ = [
    "DEFAULT_SLAM_SOCKET", "MAX_PACKET_BYTES", "SCAN_BINS",
    "SlamScanPacket", "SlamOdometryPacket", "SlamPosePacket", "SlamToolboxSender",
    "SlamLocalizationReceiver", "encode_scan", "encode_odometry", "encode_pose", "decode_packet",
]
