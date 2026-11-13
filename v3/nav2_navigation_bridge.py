"""ROS-only full Nav2 edge. No command gateway, GPIO or motor capability."""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import socket
import time

from v3.adapters.nav2_navigation import (
    COMMAND_TTL_NS, HEARTBEAT_MAX_AGE_NS, MAX_PACKET_BYTES, SENSOR_MAX_AGE_NS,
    decode_packet, encode_packet,
)
from v3.contracts import GLOBAL_FRAME_ID, LOCAL_FRAME_ID
from v3.nav2_diagnostics import NAV2_SAMPLE_PERIOD_NS, Nav2Diagnostics


def validate_request(packet, now_ns):
    for key in ("mission_id", "goal_id"):
        if not isinstance(packet.get(key), str) or not 0 < len(packet[key]) <= 256:
            raise ValueError("invalid navigation identity")
    if packet.get("operation") == "CANCEL":
        return
    if packet.get("operation") != "NAVIGATE" or packet.get("frame_id") != GLOBAL_FRAME_ID:
        raise ValueError("invalid navigation operation/frame")
    for key in ("source_ns", "source_tick_id", "odom_generation", "transform_revision"):
        if type(packet.get(key)) is not int or packet[key] < 0:
            raise ValueError("invalid navigation lineage")
    if not 0 <= now_ns-packet["source_ns"] <= HEARTBEAT_MAX_AGE_NS:
        raise ValueError("navigation heartbeat expired")
    for key, maximum in (("max_v_mps", 2.), ("max_omega_rad_s", 8.)):
        value = packet.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= maximum:
            raise ValueError("invalid mission envelope")
    for key in ("goal_tolerance_m", "yaw_tolerance_rad"):
        value = packet.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError("invalid mission goal tolerance")
    goal = packet.get("goal")
    if packet.get("mode") == "EXPLORE":
        if goal is not None:
            raise ValueError("EXPLORE goal belongs to map selector")
    elif packet.get("mode") == "NAVIGATE":
        if (not isinstance(goal, list) or len(goal) != 3 or any(
                (value is None and index != 2) or (value is not None and
                    (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)))
                for index, value in enumerate(goal))):
            raise ValueError("invalid navigation goal")
    else:
        raise ValueError("invalid navigation mode")


def velocity_lifetime(packet, *, now_ns, produced_ns, accepted_ns, scan_ns, odom_ns, tf_ns):
    """A fresh ROS stamp cannot renew old physical evidence or an old goal."""
    if not (accepted_ns <= produced_ns <= now_ns and packet["source_ns"] <= produced_ns
            and 0 <= now_ns-packet["source_ns"] <= HEARTBEAT_MAX_AGE_NS
            and all(0 < stamp <= produced_ns and now_ns-stamp <= SENSOR_MAX_AGE_NS
                    for stamp in (scan_ns, odom_ns, tf_ns))):
        return None
    deadline = min(produced_ns+COMMAND_TTL_NS, scan_ns+SENSOR_MAX_AGE_NS,
                   odom_ns+SENSOR_MAX_AGE_NS, tf_ns+SENSOR_MAX_AGE_NS)
    return deadline if now_ns < deadline else None


def controller_goal_parameters(packet):
    cruising = packet["mode"] == "EXPLORE"
    # Cruise destinations are intermediate room sectors, not final pose goals.
    # Requiring centimetre/yaw convergence here causes avoidable stop/spin cycles.
    return (.18 if cruising else packet["goal_tolerance_m"]*.5,
        math.pi if cruising else packet["yaw_tolerance_rad"]*.5,
        min(1.2, packet["max_omega_rad_s"]), packet["max_v_mps"], packet["max_omega_rad_s"])


def fallback_status(now_ns, active_until_ns, *, terminal_status, fresh,
                    lifecycle_active, action_active):
    """Do not overwrite an unexpired proposal with a heartbeat-only PENDING.

    Silence retains exactly the original source command and deadline in V3.
    Terminal/cancel/unavailable evidence always revokes it immediately.
    """
    if terminal_status is not None:
        return terminal_status
    if fresh and lifecycle_active and action_active and now_ns < active_until_ns:
        return None
    return "PENDING" if fresh and lifecycle_active else "STALE"


def run(args):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
    from nav2_msgs.msg import BehaviorTreeLog
    from sensor_msgs.msg import LaserScan
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.msg import SpeedLimit
    from v3.adapters.nav2_lifecycle import Nav2LifecycleMonitor
    from rcl_interfaces.srv import SetParametersAtomically
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from tf2_ros import Buffer, TransformListener, TransformException
    from rclpy.time import Time
    from v3.adapters.ros2_frontier import Ros2FrontierExplorer
    from v3.adapters.room_cruise import RoomCruiseGoals

    path, status_path = Path(args.socket), Path(args.status_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise RuntimeError(f"Nav2 navigation socket already owned: {path}")
    ingress = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    ingress.setblocking(False)
    ingress.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * MAX_PACKET_BYTES)
    ingress.bind(str(path))
    os.chmod(path, 0o600)
    generation = time.monotonic_ns()
    ros_offset_ns = time.time_ns()-generation
    state = dict(state="STARTING", pid=os.getpid(), worker_generation=generation,
        motor_authority=False, requests=0, rejected=0, commands=0, cancellations=0,
        map_revision=0, child_goal_id=0, error=None, lifecycle_owner="nav2_lifecycle_manager")
    stopped = False

    def stop(_signal, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node("r2b4_nav2_navigation_bridge")
    frontier = Ros2FrontierExplorer(node, args.frontier_executable, args.frontier_params,
        status_path.parent / "frontier.log", ros_offset_ns)
    cruise = RoomCruiseGoals()
    navigator = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    speed_publisher = node.create_publisher(SpeedLimit, "/speed_limit", 1)
    goal_parameters = node.create_client(SetParametersAtomically, "/controller_server/set_parameters_atomically")
    parameter_future, parameter_values, applied_tolerances, parameter_started_ns = None, None, None, 0
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    diagnostics = Nav2Diagnostics(node, generation, ros_offset_ns, **args.robot_geometry)
    lifecycle = Nav2LifecycleMonitor(node, generation)
    active = False
    current = None
    # All action state remains ROS-local. V3 mission identity is echoed only.
    sending, handle, result_future, cancel_future = None, None, None, None
    cancelling, cancel_started_ns, accepted_ns, settle_until_ns = False, 0, 0, 0
    terminal_status = None
    selected_goal = None
    frontier_goal = None
    map_valid, map_received_ns, map_stamp_ns = False, 0, 0
    scan_ns, odom_ns, tf_ns = 0, 0, 0
    scan_history, odom_history, tf_history = deque(maxlen=32), deque(maxlen=32), deque(maxlen=32)
    sequence, last_status_ns, last_reply_ns = 0, 0, 0
    last_active_until_ns = 0
    latest_source_ns = -1

    def stamp_ns(message):
        return message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec-ros_offset_ns

    def reply(status, *, produced_ns=None, valid_until_ns=None, v=0., omega=0., evidence=None):
        nonlocal sequence, last_reply_ns, last_active_until_ns
        if current is None:
            return
        packet, address = current
        now = time.monotonic_ns()
        produced_ns = now if produced_ns is None else produced_ns
        if produced_ns < packet["source_ns"]:
            return
        sequence += 1
        output = {key: packet[key] for key in ("mission_id", "goal_id", "source_ns",
            "source_tick_id", "odom_generation", "transform_revision")}
        scan_evidence, odom_evidence = (scan_ns, odom_ns) if evidence is None else evidence
        output.update(frame_id=LOCAL_FRAME_ID, status=status, worker_generation=generation,
            command_sequence=sequence, produced_ns=produced_ns,
            valid_until_ns=(max(produced_ns, min(produced_ns+COMMAND_TTL_NS,
                packet["source_ns"]+HEARTBEAT_MAX_AGE_NS))
                if valid_until_ns is None else valid_until_ns),
            scan_measurement_ns=min(scan_evidence, produced_ns), odom_measurement_ns=min(odom_evidence, produced_ns),
            v_mps=v, omega_rad_s=omega, selected_goal=selected_goal,
            map_revision=state["map_revision"], child_goal_id=state["child_goal_id"])
        try:
            ingress.sendto(encode_packet(output), address)
            last_reply_ns = now
            last_active_until_ns = output["valid_until_ns"] if status == "ACTIVE" else 0
        except OSError:
            state["rejected"] += 1

    def map_callback(message):
        nonlocal map_valid, map_received_ns, map_stamp_ns
        measured = stamp_ns(message)
        cells = message.info.width * message.info.height
        if (message.header.frame_id != GLOBAL_FRAME_ID or not 0 < cells <= args.max_map_cells
                or cells != len(message.data) or not math.isfinite(message.info.resolution)
                or message.info.resolution <= 0 or not 0 <= time.monotonic_ns()-measured <= 6_000_000_000):
            map_valid = False
            return
        p, q = message.info.origin.position, message.info.origin.orientation
        if (not all(math.isfinite(value) for value in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                or abs(sum(value*value for value in (q.x, q.y, q.z, q.w))-1.) > .01):
            map_valid = False
            return
        # Only freshness/revision scalars of the SLAM map stay here. Frontier
        # search and all path/collision work remain with ROS2/Nav2.
        map_valid = True
        map_received_ns = time.monotonic_ns()
        map_stamp_ns = measured
        state["map_revision"] += 1
        diagnostics.observe_map(message, measured, "slam")

    def coverage_costmap_callback(message):
        measured = stamp_ns(message)
        cells = message.info.width*message.info.height
        p, q = message.info.origin.position, message.info.origin.orientation
        if (message.header.frame_id == GLOBAL_FRAME_ID and 0 < cells <= args.max_map_cells
                and cells == len(message.data) and math.isfinite(message.info.resolution)
                and message.info.resolution > 0 and 0 <= time.monotonic_ns()-measured <= 3_000_000_000
                and all(math.isfinite(value) for value in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                and abs(q.x)+abs(q.y) <= 1e-6 and abs(p.z) <= 1e-6
                and abs(q.z*q.z+q.w*q.w-1.) <= .01):
            cruise.update_costmap(message, measured)
            diagnostics.observe_map(message, measured, "global_costmap")

    def local_costmap_callback(message):
        measured = stamp_ns(message)
        cells = message.info.width*message.info.height
        p, q = message.info.origin.position, message.info.origin.orientation
        if (message.header.frame_id == LOCAL_FRAME_ID and 0 < cells <= args.max_map_cells
                and cells == len(message.data) and math.isfinite(message.info.resolution)
                and message.info.resolution > 0 and 0 <= time.monotonic_ns()-measured <= 3_000_000_000
                and all(math.isfinite(value) for value in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                and abs(q.x)+abs(q.y) <= 1e-6 and abs(p.z) <= 1e-6
                and abs(q.z*q.z+q.w*q.w-1.) <= .01):
            diagnostics.observe_map(message, measured, "local_costmap")

    def scan_callback(message):
        nonlocal scan_ns
        measured = stamp_ns(message)
        if (message.header.frame_id == "r2b4_laser"
                and 0 <= time.monotonic_ns()-measured <= SENSOR_MAX_AGE_NS
                and 16 <= len(message.ranges) <= 2048
                and math.isfinite(message.range_min) and math.isfinite(message.range_max)
                and 0 < message.range_min < message.range_max
                and math.isfinite(message.angle_increment) and message.angle_increment > 0
                and all(math.isnan(value) or value == math.inf or message.range_min <= value <= message.range_max
                        for value in message.ranges)):
            scan_ns = max(scan_ns, measured)
            diagnostics.observe_scan(message, measured)
            if not scan_history or measured > scan_history[-1]:
                scan_history.append(measured)

    def odom_callback(message):
        nonlocal odom_ns
        measured = stamp_ns(message)
        p, q, twist = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
        if (message.header.frame_id == LOCAL_FRAME_ID and message.child_frame_id == "base_link"
                and 0 <= time.monotonic_ns()-measured <= SENSOR_MAX_AGE_NS
                and all(math.isfinite(value) for value in (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                    twist.linear.x, twist.angular.z))
                and abs(sum(value*value for value in (q.x, q.y, q.z, q.w))-1.) <= .01):
            if measured >= odom_ns:
                diagnostics.local_pose = (measured, (p.x, p.y,
                    math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
            odom_ns = max(odom_ns, measured)
            diagnostics.observe_odometry(message, measured)
            if not odom_history or measured > odom_history[-1]:
                odom_history.append(measured)

    def velocity_callback(message):
        diagnostics.observe_command(message, stamp_ns(message), time.monotonic_ns())
        if current is None or handle is None or cancelling or not active or terminal_status is not None:
            return
        if current[0]["mode"] == "EXPLORE" and cruise.phase == "FRONTIER" and (frontier_goal is None
                or frontier_goal is not frontier.proposal or frontier_goal.completion.done()):
            return
        now, produced = time.monotonic_ns(), stamp_ns(message)
        # ROS queues can deliver a newer sensor callback before an older cmd.
        # Use already-observed measurements that existed at command production.
        evidence = [next((stamp for stamp in reversed(history) if stamp <= produced), 0)
                    for history in (scan_history, odom_history, tf_history)]
        deadline = velocity_lifetime(current[0], now_ns=now, produced_ns=produced,
            accepted_ns=accepted_ns, scan_ns=evidence[0], odom_ns=evidence[1], tf_ns=evidence[2])
        v, omega = message.twist.linear.x, message.twist.angular.z
        if (deadline is None or message.header.frame_id not in ("", "base_link")
                or not math.isfinite(v) or not math.isfinite(omega)
                or any(abs(value) > 1e-8 for value in (message.twist.linear.y,
                    message.twist.linear.z, message.twist.angular.x, message.twist.angular.y))
                or abs(v) > current[0]["max_v_mps"]+1e-6
                or abs(omega) > current[0]["max_omega_rad_s"]+1e-6):
            reply("STALE")
            return
        reply("ACTIVE", produced_ns=produced, valid_until_ns=deadline, v=v, omega=omega,
              evidence=evidence[:2])
        state["commands"] += 1

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, "/map", map_callback, qos)
    node.create_subscription(OccupancyGrid, "/global_costmap/costmap", coverage_costmap_callback, qos)
    node.create_subscription(OccupancyGrid, "/local_costmap/costmap", local_costmap_callback, qos)
    node.create_subscription(LaserScan, "/r2b4/slam/scan", scan_callback, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/r2b4/odom", odom_callback, qos_profile_sensor_data)
    node.create_subscription(TwistStamped, "/r2b4/nav2/cmd_vel", velocity_callback, 1)
    node.create_subscription(RosPath, "/plan",
        lambda message: diagnostics.observe_path(message, stamp_ns(message)), 1)
    node.create_subscription(BehaviorTreeLog, "/behavior_tree_log",
        lambda message: diagnostics.observe_recovery(message, time.monotonic_ns()), 4)

    def navigation_feedback(message, child_id, mission_id):
        # ActionClient dispatch already correlates feedback to its goal UUID.
        feedback = message.feedback
        diagnostics.recovery_feedback = {
            "received_monotonic_ns": time.monotonic_ns(),
            "child_goal_id": child_id, "mission_id": mission_id,
            "goal_uuid": bytes(message.goal_id.uuid).hex(),
            "number_of_recoveries": int(feedback.number_of_recoveries),
            "distance_remaining_m": float(feedback.distance_remaining)}

    def begin_cancel(now, *, stop_frontier=True):
        nonlocal cancelling, cancel_started_ns, cancel_future, last_active_until_ns
        if last_active_until_ns:
            # Revoke an internal child's cached velocity before cancelling the
            # ROS action. Explicit CANCELLED/STALE replies already clear this.
            reply("PENDING")
        last_active_until_ns = 0
        if stop_frontier:
            frontier.stop()
        if not cancelling and (sending is not None or handle is not None):
            cancelling, cancel_started_ns = True, now
            state["cancellations"] += 1
            if handle is not None:
                cancel_future = handle.cancel_goal_async()

    try:
        while not stopped and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.002)
            now = time.monotonic_ns()
            frontier.poll()
            active = lifecycle.poll(now)
            state["state"] = "RUNNING" if active else "STARTING"
            robot_pose = None
            try:
                transform = tf_buffer.lookup_transform(GLOBAL_FRAME_ID, "base_link", Time())
                tf_ns = stamp_ns(transform)
                if not tf_history or tf_ns > tf_history[-1]:
                    tf_history.append(tf_ns)
                p = transform.transform.translation
                q = transform.transform.rotation
                robot_pose = p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            except TransformException:
                tf_ns = 0
            if 0 < tf_ns <= now and now-tf_ns <= SENSOR_MAX_AGE_NS:
                cruise.observe_pose(robot_pose, now)
            # Only four small packets are decoded per ROS turn.
            for _ in range(4):
                try:
                    data, address = ingress.recvfrom(MAX_PACKET_BYTES+1)
                except BlockingIOError:
                    break
                try:
                    packet = decode_packet(data)
                    # TF/ROS work above may have yielded the CPU since the
                    # loop timestamp. Validate against actual receipt time.
                    now = time.monotonic_ns()
                    validate_request(packet, now)
                except (ValueError, KeyError, TypeError, OverflowError) as exc:
                    state["rejected"] += 1
                    state["last_rejection"] = str(exc)[:128]
                    continue
                state["requests"] += 1
                if packet["operation"] == "CANCEL":
                    if current is not None and all(packet[key] == current[0][key] for key in ("mission_id", "goal_id")):
                        reply("CANCELLED")
                        begin_cancel(now)
                        cruise.stop()
                        current, selected_goal, terminal_status, frontier_goal = None, None, None, None
                    continue
                if packet["source_ns"] <= latest_source_ns:
                    continue
                latest_source_ns = packet["source_ns"]
                same = current is not None and all(packet[key] == current[0][key]
                    for key in ("mission_id", "goal_id", "odom_generation", "mode", "goal",
                                "goal_tolerance_m", "yaw_tolerance_rad", "max_v_mps", "max_omega_rad_s"))
                if same and packet["source_ns"] <= current[0]["source_ns"]:
                    continue
                if not same:
                    begin_cancel(now)
                    cruise.stop()
                    selected_goal, terminal_status, frontier_goal = None, None, None
                current = packet, address
                if packet["mode"] == "EXPLORE":
                    session = tuple(packet[key] for key in ("mission_id", "goal_id", "odom_generation"))
                    cruise.begin(session)
                    # Overlap ROS discovery/costmap warmup with mission
                    # admission. Actual Nav2 dispatch below still requires
                    # fresh sensors/map/TF and applied mission tolerances.
                    if cruise.phase == "FRONTIER":
                        frontier.start(session)
                limit = SpeedLimit()
                limit.header.stamp = node.get_clock().now().to_msg()
                limit.percentage, limit.speed_limit = False, float(packet["max_v_mps"])
                speed_publisher.publish(limit)
            if current is not None and now-current[0]["source_ns"] > HEARTBEAT_MAX_AGE_NS:
                reply("STALE")
                begin_cancel(now)
                cruise.stop()
                current, selected_goal, terminal_status, frontier_goal = None, None, None, None
            if (current is not None and current[0]["mode"] == "EXPLORE"
                    and cruise.phase == "FRONTIER" and frontier.complete):
                # A completed map is normal: frontier discovery gives way to
                # persistent known-room coverage, without a V3 NO_PATH fault.
                cruise.enter_coverage()
                begin_cancel(now)
                selected_goal, frontier_goal, terminal_status = None, None, None
            if (current is not None and current[0]["mode"] == "EXPLORE"
                    and cruise.phase == "FRONTIER"
                    and frontier_goal is not None and frontier_goal is not frontier.proposal):
                # Upstream preemption/cancel revokes the old child's velocity
                # immediately. Wait for real Nav2 cancellation before dispatch.
                begin_cancel(now, stop_frontier=False)
                selected_goal, frontier_goal = None, None
            if sending is not None and sending.done():
                handle = sending.result()
                sending = None
                if handle is None or not handle.accepted:
                    if not cancelling:
                        diagnostics.record_error(now, "GOAL_REJECTED", 900,
                            goal_id=state["child_goal_id"])
                    handle, terminal_status = None, None if cancelling else "NO_PATH"
                    if cancelling:
                        cancelling, settle_until_ns = False, now+150_000_000
                    elif current is not None and current[0]["mode"] == "EXPLORE":
                        if cruise.phase == "COVERAGE":
                            cruise.finish(selected_goal, False, now)
                        frontier.finish(frontier_goal, "ABORTED", 900)
                        selected_goal, frontier_goal, terminal_status = None, None, None
                        settle_until_ns = now+150_000_000
                else:
                    accepted_ns = now
                    last_active_until_ns = 0
                    result_future = handle.get_result_async()
                    if cancelling:
                        cancel_future = handle.cancel_goal_async()
            if result_future is not None and result_future.done():
                envelope = result_future.result()
                handle, result_future, cancel_future = None, None, None
                if cancelling:
                    cancelling, settle_until_ns = False, now+150_000_000
                elif current is not None:
                    terminal_status = "SUCCEEDED" if envelope.status == 4 and envelope.result.error_code == 0 else "FAILED"
                    if terminal_status == "FAILED":
                        diagnostics.record_error(now, "NAVIGATION_FAILED", envelope.result.error_code,
                            getattr(envelope.result, "error_msg", ""), state["child_goal_id"])
                    reply("PENDING" if current[0]["mode"] == "EXPLORE" else terminal_status)
                    if current[0]["mode"] == "EXPLORE":
                        if cruise.phase == "COVERAGE":
                            cruise.finish(selected_goal, terminal_status == "SUCCEEDED", now)
                        frontier.finish(frontier_goal, "SUCCEEDED" if terminal_status == "SUCCEEDED"
                            else "ABORTED", envelope.result.error_code or (900 if terminal_status != "SUCCEEDED" else 0))
                        # Successful child completion already proves the old
                        # action is terminal; no artificial inter-goal pause.
                        settle_until_ns = now if terminal_status == "SUCCEEDED" else now+150_000_000
                        selected_goal, terminal_status, frontier_goal = None, None, None
            if cancelling and now-cancel_started_ns > 2_000_000_000:
                raise RuntimeError("NAV2_CANCELLATION_TIMEOUT")
            if parameter_future is not None:
                if parameter_future.done():
                    response = parameter_future.result()
                    if response is None or not response.result.successful:
                        raise RuntimeError("NAV2_GOAL_TOLERANCE_REJECTED")
                    applied_tolerances, parameter_future = parameter_values, None
                elif now-parameter_started_ns > 2_000_000_000:
                    raise RuntimeError("NAV2_GOAL_TOLERANCE_TIMEOUT")
            desired = None if current is None else controller_goal_parameters(current[0])
            if (current is not None and active and not cancelling and sending is None and handle is None
                    and terminal_status is None and now >= settle_until_ns):
                if parameter_future is None and applied_tolerances != desired and goal_parameters.service_is_ready():
                    parameters = SetParametersAtomically.Request()
                    parameters.parameters = [Parameter(name=name,
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))
                        for name, value in zip(("goal_checker.xy_goal_tolerance", "goal_checker.yaw_goal_tolerance",
                            "FollowPath.rotate_to_heading_angular_vel", "FollowPath.mission_max_v_mps",
                            "FollowPath.mission_max_omega_rad_s"), desired)]
                    parameter_values, parameter_started_ns = desired, now
                    parameter_future = goal_parameters.call_async(parameters)
            fresh = (map_valid and 0 <= now-map_received_ns <= 6_000_000_000
                and 0 <= now-map_stamp_ns <= 6_000_000_000
                and all(0 < stamp <= now and now-stamp <= SENSOR_MAX_AGE_NS for stamp in (scan_ns, odom_ns, tf_ns)))
            if (current is not None and active and not cancelling and sending is None and handle is None
                    and terminal_status is None and parameter_future is None
                    and applied_tolerances == desired
                    and now >= settle_until_ns and navigator.server_is_ready() and fresh):
                packet = current[0]
                if selected_goal is None:
                    if packet["mode"] == "EXPLORE":
                        if cruise.phase == "COVERAGE":
                            selected_goal = cruise.select(robot_pose, now)
                        else:
                            session = tuple(packet[key] for key in ("mission_id", "goal_id", "odom_generation"))
                            if frontier.start(session):
                                frontier_goal = frontier.proposal
                                selected_goal = None if frontier_goal is None else list(frontier_goal.goal)
                    else:
                        selected_goal = packet["goal"]
                    if selected_goal is not None:
                        state["child_goal_id"] += 1
                if selected_goal is not None:
                    # Goal selection can consume an edge turn; never dispatch
                    # with the timestamp captured before that computation.
                    dispatch_ns = time.monotonic_ns()
                    if (dispatch_ns-packet["source_ns"] > HEARTBEAT_MAX_AGE_NS
                            or any(dispatch_ns-stamp > SENSOR_MAX_AGE_NS for stamp in (scan_ns, odom_ns, tf_ns))):
                        continue
                    goal = NavigateToPose.Goal()
                    goal.pose = PoseStamped()
                    goal.pose.header.frame_id = GLOBAL_FRAME_ID
                    goal.pose.header.stamp = node.get_clock().now().to_msg()
                    goal.pose.pose.position.x, goal.pose.pose.position.y = map(float, selected_goal[:2])
                    yaw = selected_goal[2]
                    if yaw is None and robot_pose is not None:
                        yaw = math.atan2(selected_goal[1]-robot_pose[1], selected_goal[0]-robot_pose[0])
                    yaw = yaw or 0.
                    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                    goal.behavior_tree = args.behavior_tree
                    diagnostics.recovery_feedback = None
                    sending = navigator.send_goal_async(goal, feedback_callback=(
                        lambda message, child_id=state["child_goal_id"], mission_id=packet["mission_id"]:
                            navigation_feedback(message, child_id, mission_id)))
                    reply("PENDING")
                elif now-last_reply_ns >= 100_000_000:
                    reply("PENDING")
            if current is not None and now-last_reply_ns >= 100_000_000:
                status = fallback_status(now, last_active_until_ns,
                    terminal_status=terminal_status, fresh=fresh, lifecycle_active=active,
                    action_active=handle is not None and not cancelling)
                if status is not None:
                    reply(status)
            if now-last_status_ns >= NAV2_SAMPLE_PERIOD_NS:
                # The map->odom TF head is a transform horizon, not a physical
                # measurement. Observe its coverage of odometry without waits.
                try:
                    map_tf = tf_buffer.lookup_transform(GLOBAL_FRAME_ID, LOCAL_FRAME_ID, Time())
                    head = stamp_ns(map_tf)
                    state["transform_observation"] = {
                        "available": True, "observed_monotonic_ns": now,
                        "map_to_odom_head_ns": head, "odom_measurement_ns": odom_ns,
                        "horizon_from_observation_ns": head-now,
                        "odom_headroom_ns": head-odom_ns}
                except TransformException:
                    state["transform_observation"] = {"available": False, "observed_monotonic_ns": now}
                diagnostics.scan_map_pose = None
                if diagnostics.scan is not None:
                    measured = diagnostics.scan[0]
                    try:
                        scan_transform = tf_buffer.lookup_transform(GLOBAL_FRAME_ID, "base_link",
                            Time(nanoseconds=measured+ros_offset_ns))
                        p, q = scan_transform.transform.translation, scan_transform.transform.rotation
                        diagnostics.scan_map_pose = (measured, (p.x, p.y,
                            math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
                    except TransformException:
                        pass
                state["lifecycle_status"] = lifecycle.snapshot(now)
                state.update(observed_monotonic_ns=now, pending=sending is not None,
                    action_active=handle is not None, cancelling=cancelling,
                    mission_id=None if current is None else current[0]["mission_id"],
                    selected_goal=selected_goal, scan_measurement_ns=scan_ns,
                    odom_measurement_ns=odom_ns, tf_measurement_ns=tf_ns,
                    goal_provider=("nav2_costmap_coverage" if cruise.phase == "COVERAGE" else "ros2_explore_lite"),
                    frontier_status=frontier.status,
                    frontier_running=frontier.session is not None,
                    frontier_pid=None if frontier.process is None else frontier.process.pid,
                    frontier_goals=frontier.accepted, frontier_rejected=frontier.rejected)
                state["frontier_startup"] = frontier.startup_timing
                state.update(**cruise.snapshot())
                state["goal_checker_tolerances"] = applied_tolerances
                state["nav2_observation"] = diagnostics.snapshot(now,
                    scan_ns=scan_ns, odom_ns=odom_ns, tf_ns=tf_ns,
                    active=handle is not None and not cancelling,
                    robot_pose=robot_pose, state=state)
                temporary = status_path.with_suffix(".tmp")
                temporary.write_text(json.dumps(state, allow_nan=False, sort_keys=True)+"\n")
                temporary.replace(status_path)
                last_status_ns = now
    except BaseException as exc:
        state.update(state="FAILED", error=f"{type(exc).__name__}:{exc}"[:256])
        state["lifecycle_status"] = lifecycle.snapshot(time.monotonic_ns())
        diagnostics.record_error(time.monotonic_ns(), "BRIDGE_FAILED", message=state["error"])
        raise
    finally:
        if handle is not None:
            handle.cancel_goal_async()
        if current is not None:
            reply("UNAVAILABLE")
        frontier.close()
        ingress.close()
        path.unlink(missing_ok=True)
        if state["state"] != "FAILED":
            state["state"] = "STOPPED"
        final_observed_ns = time.monotonic_ns()
        state["observed_monotonic_ns"] = final_observed_ns
        state["nav2_observation"] = diagnostics.snapshot(final_observed_ns,
            scan_ns=scan_ns, odom_ns=odom_ns, tf_ns=tf_ns, active=False,
            robot_pose=None, state=state)
        temporary = status_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(state, allow_nan=False, sort_keys=True)+"\n")
        temporary.replace(status_path)
        node.destroy_node()
        rclpy.shutdown()
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True)
    parser.add_argument("--status-path", required=True)
    parser.add_argument("--robot-geometry", required=True, type=json.loads)
    parser.add_argument("--behavior-tree", required=True)
    parser.add_argument("--frontier-executable", required=True)
    parser.add_argument("--frontier-params", required=True)
    parser.add_argument("--max-map-cells", type=int, default=250000)
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
