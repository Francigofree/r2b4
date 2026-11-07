"""ROS-only Planner Server bridge: global path -> one compact local suggestion."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import time

from v3.adapters.nav2_planner import MAX_PACKET_BYTES, decode_packet, encode_packet
from v3.contracts import GLOBAL_FRAME_ID, LOCAL_FRAME_ID, Pose2D


def local_waypoint(path, start, transform, lookahead_m, goal_yaw=None):
    """Walk the first path segment; never jump across a bend or return raw path."""
    if not 1 < len(path) <= 16384 or not math.isfinite(lookahead_m) or lookahead_m <= 0:
        raise ValueError("invalid global path bounds")
    if any(len(point) != 2 or any(not math.isfinite(v) for v in point) for point in path):
        raise ValueError("invalid global path coordinates")
    if math.hypot(path[0][0]-start[0], path[0][1]-start[1]) > .20:
        raise ValueError("global path start differs from request")
    remaining = lookahead_m
    previous = start[:2]
    target = previous
    terminal = True
    for point in path:
        distance = math.hypot(point[0]-previous[0], point[1]-previous[1])
        if distance > remaining and distance > 0:
            fraction = remaining / distance
            target = (previous[0] + fraction*(point[0]-previous[0]),
                      previous[1] + fraction*(point[1]-previous[1]))
            terminal = False
            break
        remaining -= distance
        previous = target = point
    return list(transform.inverse().apply(*target, goal_yaw if terminal else None))


def validate_request(packet, now_ns):
    for key in ("request_id", "source_ns", "tick_id", "odom_generation", "transform_revision",
                "deadline_ns", "max_map_age_ns"):
        if type(packet.get(key)) is not int or packet[key] < 0:
            raise ValueError("invalid request identity/deadline")
    if (packet.get("frame_id") != GLOBAL_FRAME_ID
            or not 0 <= now_ns-packet["source_ns"] <= 350_000_000
            or not now_ns < packet["deadline_ns"] <= now_ns+1_000_000_000
            or not 0 < packet["max_map_age_ns"] <= 30_000_000_000):
        raise ValueError("stale request or frame mismatch")
    for key in ("start", "goal", "map_to_odom"):
        values = packet.get(key)
        if (not isinstance(values, list) or len(values) != 3
                or any((value is None and (key != "goal" or index != 2))
                       or (value is not None and (isinstance(value, bool)
                           or not isinstance(value, (int, float)) or not math.isfinite(value)))
                       for index, value in enumerate(values))):
            raise ValueError("invalid request pose")
    distance = packet.get("lookahead_m")
    if isinstance(distance, bool) or not isinstance(distance, (int, float)) or not 0 < distance <= 5.:
        raise ValueError("invalid local waypoint distance")


def run(args):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import ComputePathToPose
    from lifecycle_msgs.srv import ChangeState

    path, status_path = Path(args.socket), Path(args.status_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise RuntimeError(f"Nav2 socket already owned: {path}")
    ingress = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    ingress.setblocking(False)
    ingress.bind(str(path))
    os.chmod(path, 0o600)
    state = {"state": "STARTING", "pid": os.getpid(), "requests": 0, "completed": 0,
             "rejected": 0, "timeouts": 0, "map_revision": 0, "motor_authority": False}
    stopped = False

    def stop(_signal, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node("r2b4_nav2_planner_bridge")
    planner = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
    lifecycle = node.create_client(ChangeState, "/planner_server/change_state")
    lifecycle_future, transition, active = None, 1, False
    map_stamp_ns = 0
    map_received_ns = 0
    map_valid = False
    pending = None
    last_status_ns = 0

    def map_received(message):
        nonlocal map_stamp_ns, map_received_ns, map_valid
        state["map_revision"] += 1
        cells = message.info.width * message.info.height
        map_valid = (message.header.frame_id == GLOBAL_FRAME_ID and 0 < cells <= args.max_map_cells
                     and cells == len(message.data) and message.info.resolution > 0)
        map_stamp_ns = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
        map_received_ns = time.monotonic_ns()

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, "/map", map_received, qos)

    def reply(packet, address, status, **extra):
        result = {key: packet[key] for key in ("request_id", "source_ns", "tick_id",
                  "odom_generation", "transform_revision")}
        result.update(frame_id=LOCAL_FRAME_ID, status=status, **extra)
        try:
            ingress.sendto(encode_packet(result), address)
        except OSError:
            state["rejected"] += 1

    def pose(values):
        result = PoseStamped()
        result.header.frame_id = GLOBAL_FRAME_ID
        result.header.stamp = node.get_clock().now().to_msg()
        result.pose.position.x, result.pose.position.y = float(values[0]), float(values[1])
        yaw = values[2] or 0.
        result.pose.orientation.z, result.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        return result

    try:
        while not stopped and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.002)
            now = time.monotonic_ns()
            if not active:
                if lifecycle_future is not None and lifecycle_future.done():
                    if not lifecycle_future.result().success:
                        raise RuntimeError("NAV2_LIFECYCLE_TRANSITION_FAILED")
                    lifecycle_future = None
                    if transition == 3:
                        active = True
                        state["state"] = "RUNNING"
                    transition = 3
                if not active and lifecycle_future is None and lifecycle.service_is_ready():
                    request = ChangeState.Request()
                    request.transition.id = transition
                    lifecycle_future = lifecycle.call_async(request)
            if pending is not None:
                packet, address, future, handle, revision, stamp = pending
                if now > packet["deadline_ns"]:
                    if handle is not None:
                        handle.cancel_goal_async()
                    else:
                        # Cancellation after late acceptance prevents an orphan
                        # action occupying the next logical request.
                        def cancel_late(done):
                            accepted = done.result()
                            if accepted is not None and accepted.accepted:
                                accepted.cancel_goal_async()
                        future.add_done_callback(cancel_late)
                    reply(packet, address, "TIMEOUT")
                    state["timeouts"] += 1
                    pending = None
                elif future.done():
                    if handle is None:
                        handle = future.result()
                        if handle is None or not handle.accepted:
                            reply(packet, address, "NO_PATH")
                            pending = None
                        else:
                            pending = (packet, address, handle.get_result_async(), handle, revision, stamp)
                    else:
                        envelope = future.result()
                        result = envelope.result
                        if (envelope.status != 4 or result.error_code != 0 or not result.path.poses):
                            reply(packet, address, "NO_PATH")
                        else:
                            try:
                                if (result.path.header.frame_id != GLOBAL_FRAME_ID
                                        or any(item.header.frame_id not in ("", GLOBAL_FRAME_ID)
                                               for item in result.path.poses)):
                                    raise ValueError("Nav2 path frame mismatch")
                                points = [(item.pose.position.x, item.pose.position.y) for item in result.path.poses]
                                if math.hypot(points[-1][0]-packet["goal"][0], points[-1][1]-packet["goal"][1]) > .15:
                                    raise ValueError("Nav2 path does not reach goal")
                                waypoint = local_waypoint(points, packet["start"],
                                    Pose2D(GLOBAL_FRAME_ID, *packet["map_to_odom"]), packet["lookahead_m"], packet["goal"][2])
                                reply(packet, address, "READY", local_goal=waypoint,
                                      map_revision=revision, map_stamp_ns=stamp)
                                state["completed"] += 1
                            except ValueError:
                                reply(packet, address, "INVALID")
                        pending = None
            # Bound ingress drain even under malformed client traffic.
            for _ in range(4):
                try:
                    data, address = ingress.recvfrom(MAX_PACKET_BYTES+1)
                except BlockingIOError:
                    break
                try:
                    packet = decode_packet(data)
                    validate_request(packet, now)
                except (ValueError, KeyError, TypeError, OverflowError):
                    state["rejected"] += 1
                    continue
                state["requests"] += 1
                if pending is not None or not active or not planner.server_is_ready():
                    reply(packet, address, "UNAVAILABLE")
                    continue
                ros_now = node.get_clock().now().nanoseconds
                if (not map_valid or not map_stamp_ns or not map_received_ns
                        or not 0 <= ros_now-map_stamp_ns <= packet["max_map_age_ns"]
                        or now-map_received_ns > packet["max_map_age_ns"]):
                    reply(packet, address, "STALE")
                    continue
                goal = ComputePathToPose.Goal()
                goal.start, goal.goal = pose(packet["start"]), pose(packet["goal"])
                goal.planner_id, goal.use_start = "GridBased", True
                pending = (packet, address, planner.send_goal_async(goal), None,
                           state["map_revision"], map_stamp_ns)
            if now-last_status_ns > 250_000_000:
                last_status_ns = now
                state["map_stamp_ns"], state["pending"] = map_stamp_ns, pending is not None
                temporary = status_path.with_suffix(".tmp")
                temporary.write_text(json.dumps(state, sort_keys=True)+"\n")
                temporary.replace(status_path)
    finally:
        if pending is not None and pending[3] is not None:
            pending[3].cancel_goal_async()
        ingress.close()
        path.unlink(missing_ok=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True)
    parser.add_argument("--status-path", required=True)
    parser.add_argument("--max-map-cells", type=int, default=250000)
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
