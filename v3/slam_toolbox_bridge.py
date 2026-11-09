"""ROS-only SLAM Toolbox edge. Imported by its isolated interpreter, never L0–L12.

Scan bytes arrive directly from the LiDAR owner. Only correlated, scan-time
base poses return to that owner; global map geometry stays in this process.
"""
from __future__ import annotations

import argparse
from collections import OrderedDict, deque
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import socket
import time

from v3.adapters.slam_toolbox_transport import (
    MAX_PACKET_BYTES, SlamOdometryPacket, SlamScanPacket, SlamPosePacket,
    decode_packet, encode_pose,
)

MAP_FRAME = "R2B4_BOOT_ROBOT_MAP"
ODOM_FRAME = "R2B4_ODOM_LOCAL"
BASE_FRAME = "base_link"
LASER_FRAME = "r2b4_laser"
MAX_SOURCE_AGE_NS = 250_000_000
MAX_ODOM_BRACKET_NS = 100_000_000


@dataclass(frozen=True, slots=True)
class ScanPose:
    generation: int
    x_m: float
    y_m: float
    yaw_rad: float


def odometry_twist(previous: SlamOdometryPacket | None, current: SlamOdometryPacket):
    """Body twist from qualified same-generation measurements, never receipt time."""
    if (previous is None or previous.session_id != current.session_id
            or previous.generation != current.generation
            or not all((previous.local_translation_valid, previous.heading_valid,
                        current.local_translation_valid, current.heading_valid))):
        return None
    elapsed = current.monotonic_ns-previous.monotonic_ns
    if not 0 < elapsed <= MAX_ODOM_BRACKET_NS:
        return None
    dt = elapsed/1e9
    yaw_delta = math.atan2(math.sin(current.yaw_rad-previous.yaw_rad),
                          math.cos(current.yaw_rad-previous.yaw_rad))
    middle_yaw = previous.yaw_rad+yaw_delta/2
    velocity = ((current.x_m-previous.x_m)*math.cos(middle_yaw)
                +(current.y_m-previous.y_m)*math.sin(middle_yaw))/dt
    return velocity, yaw_delta/dt


def ros_scan_ranges(scan: SlamScanPacket) -> list[float]:
    """Keep missing wire bins out of Karto's unfiltered scan matcher.

    Zero is the compact transport's missing-return sentinel. Karto's matcher
    does not apply LaserScan.range_min to its lookup points, so forwarding zero
    would create repeated points at the laser origin. NaN is explicitly ignored
    by both its matching and occupancy paths.
    """
    return [value if value != 0.0 else math.nan for value in scan.ranges_m]


def align_scan(scan: SlamScanPacket, history) -> ScanPose | None:
    """Interpolate qualified local odometry; never extrapolate across a reset."""
    previous = None
    for current in history:
        if current.session_id != scan.session_id:
            continue
        if current.monotonic_ns == scan.measurement_ns:
            if current.local_translation_valid and current.heading_valid:
                return ScanPose(current.generation, current.x_m, current.y_m, current.yaw_rad)
            return None
        if current.monotonic_ns > scan.measurement_ns:
            if (previous is None or previous.generation != current.generation
                    or not all((previous.local_translation_valid, previous.heading_valid,
                                current.local_translation_valid, current.heading_valid))
                    or current.monotonic_ns - previous.monotonic_ns > MAX_ODOM_BRACKET_NS):
                return None
            fraction = ((scan.measurement_ns - previous.monotonic_ns)
                        / (current.monotonic_ns - previous.monotonic_ns))
            delta = math.atan2(math.sin(current.yaw_rad - previous.yaw_rad),
                               math.cos(current.yaw_rad - previous.yaw_rad))
            yaw = previous.yaw_rad + fraction * delta
            return ScanPose(current.generation,
                previous.x_m + fraction * (current.x_m - previous.x_m),
                previous.y_m + fraction * (current.y_m - previous.y_m),
                math.atan2(math.sin(yaw), math.cos(yaw)))
        previous = current
    return None


def conservative_covariance(covariance) -> tuple[float, float, float]:
    """Validate planar ROS covariance and bound its full correlated matrix.

    Positive diagonal scaling produces a unit-diagonal correlation matrix,
    making the symmetry/minor tolerances independent of variance magnitude.
    Absolute row sums provide a diagonal upper bound in the PSD ordering;
    dropping cross-covariances would not provide that guarantee.
    """
    try:
        if len(covariance) != 36:
            raise ValueError("SLAM covariance must contain 36 ROS entries")
        planar = tuple(float(covariance[index]) for index in (0, 1, 5, 6, 7, 11, 30, 31, 35))
    except (TypeError, IndexError, OverflowError) as exc:
        raise ValueError("SLAM covariance must contain finite numeric entries") from exc
    if any(not math.isfinite(value) for value in planar):
        raise ValueError("SLAM planar covariance must be finite")
    diagonal = planar[0], planar[4], planar[8]
    if min(diagonal) <= 0:
        raise ValueError("SLAM covariance diagonal must be positive")
    roots = tuple(math.sqrt(value) for value in diagonal)
    correlations = []
    envelopes = []
    for row, column in ((0, 1), (0, 2), (1, 2)):
        forward, reverse = planar[3*row+column], planar[3*column+row]
        large, small = max(roots[row], roots[column]), min(roots[row], roots[column])
        normalized_forward, normalized_reverse = (forward / large) / small, (reverse / large) / small
        if (not math.isfinite(normalized_forward) or not math.isfinite(normalized_reverse)
                or abs(normalized_forward-normalized_reverse)
                > 1e-7 * max(1., abs(normalized_forward), abs(normalized_reverse))):
            raise ValueError("SLAM covariance must be symmetric")
        correlation = (normalized_forward + normalized_reverse) / 2
        if not math.isfinite(correlation) or 1 - correlation*correlation < -1e-10:
            raise ValueError("SLAM covariance has a negative principal minor")
        correlations.append(correlation)
        envelopes.append(max(abs(forward), abs(reverse)))
    xy, xyaw, yyaw = correlations
    determinant = 1 + 2*xy*xyaw*yyaw - xy*xy - xyaw*xyaw - yyaw*yyaw
    if determinant < -1e-10:
        raise ValueError("SLAM covariance is not positive semidefinite")
    exy, exyaw, eyyaw = envelopes
    try:
        rows = ((diagonal[0], exy, exyaw), (diagonal[1], exy, eyyaw),
                (diagonal[2], exyaw, eyyaw))
        # A correctly rounded sum can still round downward. With strongly
        # unequal scales that could erase a cross-term from the upper bound.
        bounded = tuple(math.nextafter(math.fsum(row), math.inf) if any(row[1:]) else row[0]
                        for row in rows)
    except OverflowError as exc:
        raise ValueError("SLAM covariance bound overflow") from exc
    if any(not math.isfinite(value) for value in bounded):
        raise ValueError("SLAM covariance bound must be finite")
    return bounded


def _write_status(path: Path, value: dict) -> None:
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, sort_keys=True, allow_nan=False) + "\n")
    os.chmod(temporary, 0o600)
    temporary.replace(path)


def run(args) -> int:
    # ROS and its numpy/transport dependencies exist exclusively in this edge.
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid, Odometry
    from lifecycle_msgs.srv import ChangeState
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

    path = Path(args.socket)
    status_path = Path(args.status_path)
    status_path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise RuntimeError(f"SLAM socket is already owned: {path}")
    ingress = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    ingress.setblocking(False)
    ingress.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 64 * MAX_PACKET_BYTES)
    ingress.bind(str(path))
    os.chmod(path, 0o600)
    bridge_generation = time.monotonic_ns()
    # One reversible clock-domain translation. Receipt time never substitutes
    # for measurement time; both the original ns and offset are observable.
    ros_offset_ns = time.time_ns() - time.monotonic_ns()
    state = {"state": "STARTING", "pid": os.getpid(),
             "bridge_generation": bridge_generation, "ros_offset_ns": ros_offset_ns,
             "scans_received": 0, "scans_published": 0, "poses_received": 0,
             "poses_returned": 0, "rejected": 0, "superseded": 0,
             "rejected_by_reason": {}, "last_rejection": None,
             "map_messages": 0, "map_frame": MAP_FRAME,
             "max_scan_age_ns": 0, "max_pose_age_ns": 0,
             "session_id": None, "generation": None, "error": None}
    stopped = False

    def stop(_number, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node("r2b4_slam_bridge")
    broadcaster = TransformBroadcaster(node)
    static_broadcaster = StaticTransformBroadcaster(node)
    publisher = node.create_publisher(LaserScan, "/r2b4/slam/scan", 1)
    odometry_publisher = node.create_publisher(Odometry, "/r2b4/odom", 1)
    change_state = node.create_client(ChangeState, "/slam_toolbox/change_state")
    lifecycle_future = None
    lifecycle_transition = 1  # CONFIGURE then ACTIVATE (3)
    lifecycle_active = False
    history = deque(maxlen=128)
    associations = OrderedDict()
    pending = None
    geometry = None
    last_sequence = 0
    last_status_ns = 0

    def stamp(message, monotonic_ns):
        ros_ns = monotonic_ns + ros_offset_ns
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(ros_ns, 1_000_000_000)

    def reject(reason):
        state["rejected"] += 1
        counts = state["rejected_by_reason"]
        counts[reason] = counts.get(reason, 0) + 1
        state["last_rejection"] = reason

    def pose_callback(message):
        state["poses_received"] += 1
        ros_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        context = associations.pop(ros_ns, None)
        if context is None:
            reject("POSE_SCAN_ASSOCIATION_MISSING")
            return
        if message.header.frame_id != MAP_FRAME:
            reject("POSE_FRAME_MISMATCH")
            return
        scan, aligned, address = context
        age = time.monotonic_ns() - scan.measurement_ns
        q = message.pose.pose.orientation
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if not 0 <= age <= MAX_SOURCE_AGE_NS:
            reject("POSE_MEASUREMENT_STALE")
            return
        if (not address or not math.isfinite(norm) or abs(norm - 1.0) > 0.01
                or abs(q.x) > 0.01 or abs(q.y) > 0.01):
            reject("POSE_ORIENTATION_OR_REPLY_INVALID")
            return
        p = message.pose.pose.position
        c = message.pose.covariance
        try:
            variance_x, variance_y, variance_yaw = conservative_covariance(c)
        except ValueError:
            reject("POSE_COVARIANCE_INVALID")
            return
        try:
            # SLAM Toolbox publishes Karto GetCorrectedPose, the robot base
            # pose; the laser extrinsics have already been applied by Karto.
            result = SlamPosePacket(scan.session_id, scan.source_sequence,
                scan.measurement_ns, aligned.generation, p.x, p.y,
                math.atan2(2*q.w*q.z, 1 - 2*q.z*q.z), variance_x, variance_y, variance_yaw,
                bridge_generation)
            ingress.sendto(encode_pose(result), address)
        except ValueError:
            reject("POSE_VALUE_INVALID")
            return
        except OSError:
            reject("POSE_REPLY_TRANSPORT_FAILED")
            return
        state["poses_returned"] += 1
        state["last_pose_measurement_ns"] = scan.measurement_ns
        state["last_pose_sequence"] = scan.source_sequence
        state["last_pose"] = [p.x, p.y, result.yaw_rad]
        state["last_pose_covariance"] = [variance_x, variance_y, variance_yaw]
        state["last_pose_source_covariance"] = [float(c[index]) for index in (0, 1, 5, 6, 7, 11, 30, 31, 35)]
        state["max_pose_age_ns"] = max(state["max_pose_age_ns"], age)

    def map_callback(message):
        cells = message.info.width * message.info.height
        if (message.header.frame_id != MAP_FRAME or cells != len(message.data)
                or cells > args.max_map_cells):
            state["error"] = "MAP_RESOURCE_OR_FRAME_LIMIT"
            return
        state["map_messages"] += 1
        state["map"] = {"width": message.info.width, "height": message.info.height,
                        "resolution_m": message.info.resolution, "cells": cells,
                        "observed_cells": sum(v >= 0 for v in message.data)}

    node.create_subscription(PoseWithCovarianceStamped, "/pose", pose_callback, 10)
    map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, "/map", map_callback, map_qos)
    try:
        while not stopped and rclpy.ok():
            now = time.monotonic_ns()
            if now - bridge_generation > args.max_duration_s * 1_000_000_000:
                raise RuntimeError("SLAM_SESSION_DURATION_LIMIT")
            if state["error"]:
                raise RuntimeError(state["error"])
            if not lifecycle_active:
                if lifecycle_future is not None and lifecycle_future.done():
                    if not lifecycle_future.result().success:
                        raise RuntimeError("SLAM_LIFECYCLE_TRANSITION_FAILED")
                    lifecycle_future = None
                    if lifecycle_transition == 3:
                        lifecycle_active = True
                        state["state"] = "ACTIVE"
                    else:
                        lifecycle_transition = 3
                if not lifecycle_active and lifecycle_future is None and change_state.service_is_ready():
                    request = ChangeState.Request()
                    request.transition.id = lifecycle_transition
                    lifecycle_future = change_state.call_async(request)
                if now - bridge_generation > 20_000_000_000:
                    raise RuntimeError("SLAM_LIFECYCLE_STARTUP_TIMEOUT")
            for _ in range(64):
                try:
                    data, address = ingress.recvfrom(MAX_PACKET_BYTES + 1)
                except BlockingIOError:
                    break
                try:
                    packet = decode_packet(data)
                except ValueError:
                    reject("INGRESS_PACKET_INVALID")
                    continue
                if not isinstance(packet, (SlamScanPacket, SlamOdometryPacket)):
                    reject("INGRESS_PACKET_KIND_INVALID")
                    continue
                if state["session_id"] is None:
                    state["session_id"] = packet.session_id
                elif state["session_id"] != packet.session_id:
                    raise RuntimeError("ROBOT_SESSION_CHANGED_RESTART_SLAM")
                if isinstance(packet, SlamOdometryPacket):
                    if state["generation"] is None:
                        state["generation"] = packet.generation
                    elif state["generation"] != packet.generation:
                        raise RuntimeError("ODOMETRY_GENERATION_CHANGED_RESTART_SLAM")
                    if 0 <= time.monotonic_ns() - packet.monotonic_ns <= 2_000_000_000:
                        if not history or packet.monotonic_ns > history[-1].monotonic_ns:
                            previous = history[-1] if history else None
                            history.append(packet)
                            if packet.local_translation_valid and packet.heading_valid:
                                transform = TransformStamped()
                                stamp(transform, packet.monotonic_ns)
                                transform.header.frame_id, transform.child_frame_id = ODOM_FRAME, BASE_FRAME
                                transform.transform.translation.x, transform.transform.translation.y = packet.x_m, packet.y_m
                                transform.transform.rotation.z = math.sin(packet.yaw_rad/2)
                                transform.transform.rotation.w = math.cos(packet.yaw_rad/2)
                                broadcaster.sendTransform(transform)
                                twist = odometry_twist(previous, packet)
                                if twist is not None:
                                    odometry = Odometry()
                                    stamp(odometry, packet.monotonic_ns)
                                    odometry.header.frame_id, odometry.child_frame_id = ODOM_FRAME, BASE_FRAME
                                    odometry.pose.pose.position.x, odometry.pose.pose.position.y = packet.x_m, packet.y_m
                                    odometry.pose.pose.orientation = transform.transform.rotation
                                    odometry.twist.twist.linear.x, odometry.twist.twist.angular.z = twist
                                    odometry_publisher.publish(odometry)
                                    state["last_odometry_measurement_ns"] = packet.monotonic_ns
                else:
                    state["scans_received"] += 1
                    if packet.source_sequence > last_sequence:
                        if pending is not None:
                            state["superseded"] += 1
                        pending = packet, address
                        last_sequence = packet.source_sequence
            if pending is not None and lifecycle_active:
                scan, address = pending
                age = time.monotonic_ns() - scan.measurement_ns
                aligned = align_scan(scan, history)
                if age < 0 or age > MAX_SOURCE_AGE_NS:
                    reject("SCAN_MEASUREMENT_STALE_OR_POSE_UNALIGNED")
                    pending = None
                elif aligned is not None:
                    new_geometry = (len(scan.ranges_m), scan.minimum_range_m,
                        scan.maximum_range_m, scan.lidar_x_m, scan.lidar_y_m)
                    if geometry is not None and geometry != new_geometry:
                        raise RuntimeError("LIDAR_GEOMETRY_CHANGED_RESTART_SLAM")
                    if geometry is None:
                        geometry = new_geometry
                        transform = TransformStamped()
                        stamp(transform, scan.measurement_ns)
                        transform.header.frame_id = BASE_FRAME
                        transform.child_frame_id = LASER_FRAME
                        transform.transform.translation.x = scan.lidar_x_m
                        transform.transform.translation.y = scan.lidar_y_m
                        transform.transform.rotation.w = 1.0
                        static_broadcaster.sendTransform(transform)
                    message = LaserScan()
                    stamp(message, scan.measurement_ns)
                    message.header.frame_id = LASER_FRAME
                    message.angle_min = 0.0
                    message.angle_increment = 2 * math.pi / len(scan.ranges_m)
                    message.angle_max = (len(scan.ranges_m) - 1) * message.angle_increment
                    message.range_min = scan.minimum_range_m
                    message.range_max = scan.maximum_range_m
                    message.scan_time = (scan.scan_end_ns - scan.scan_start_ns) / 1e9
                    # Producer rotation compensation, when available, already
                    # expresses returns at the canonical midpoint. Angular
                    # rebinning has no per-ray ordering to expose to ROS.
                    message.time_increment = 0.0
                    message.ranges = ros_scan_ranges(scan)
                    ros_ns = scan.measurement_ns + ros_offset_ns
                    associations[ros_ns] = scan, aligned, address
                    while len(associations) > 32:
                        associations.popitem(last=False)
                    publisher.publish(message)
                    state["scans_published"] += 1
                    state["last_scan_measurement_ns"] = scan.measurement_ns
                    state["last_scan_sequence"] = scan.source_sequence
                    state["max_scan_age_ns"] = max(state["max_scan_age_ns"], age)
                    pending = None
                    if state["scans_published"] >= args.max_scans:
                        raise RuntimeError("SLAM_SESSION_SCAN_LIMIT_SAVE_AND_RESTART")
            rclpy.spin_once(node, timeout_sec=0.005)
            if now - last_status_ns >= 1_000_000_000:
                state["observed_monotonic_ns"] = now
                _write_status(status_path, state)
                last_status_ns = now
        state["state"] = "STOPPED"
        return 0
    except Exception as exc:
        state["state"] = "FAILED"
        state["error"] = f"{type(exc).__name__}:{exc}"
        raise
    finally:
        _write_status(status_path, state)
        node.destroy_node()
        rclpy.shutdown()
        ingress.close()
        path.unlink(missing_ok=True)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True)
    parser.add_argument("--status-path", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--max-duration-s", type=int, default=7200)
    parser.add_argument("--max-scans", type=int, default=30000)
    parser.add_argument("--max-map-cells", type=int, default=1_000_000)
    args = parser.parse_args(argv)
    if min(args.max_duration_s, args.max_scans, args.max_map_cells) <= 0:
        parser.error("resource budgets must be positive")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
