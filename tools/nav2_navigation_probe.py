"""Motorless real-Nav2 integration probe in a separate ROS domain.

Publishes a synthetic room, scan, TF and planar odometry. Compact bridge
velocities move only the simulated pose. No robot runtime, command gateway,
device handle or motor writer is created. Logs and JSON evidence stay under a
unique output directory; domain 74 is refused to protect the live ROS stack.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time


MAP_FRAME = "R2B4_BOOT_ROBOT_MAP"
ODOM_FRAME = "R2B4_ODOM_LOCAL"
BASE_FRAME = "base_link"
LASER_FRAME = "r2b4_laser"
ROOM = (-1.5, -2.5, 5.5, 2.5)
OBSTACLE = (2., -.55, 2.45, .55)


def _ray_box(x, y, dx, dy, box):
    near, far = -math.inf, math.inf
    for value, direction, low, high in ((x, dx, box[0], box[2]), (y, dy, box[1], box[3])):
        if abs(direction) < 1e-12:
            if not low <= value <= high:
                return math.inf
        else:
            first, second = (low - value) / direction, (high - value) / direction
            near, far = max(near, min(first, second)), min(far, max(first, second))
    if far < max(near, 0.):
        return math.inf
    return near if near >= 0. else far


def _clearance(x, y, radius, obstacle):
    result = min(x - ROOM[0], ROOM[2] - x, y - ROOM[1], ROOM[3] - y) - radius
    if obstacle:
        dx = max(OBSTACLE[0] - x, 0., x - OBSTACLE[2])
        dy = max(OBSTACLE[1] - y, 0., y - OBSTACLE[3])
        result = min(result, math.hypot(dx, dy) - radius)
    return result


def _worker(args, root, output):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
    from sensor_msgs.msg import LaserScan
    from action_msgs.msg import GoalStatusArray
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    from tools.nav2_planner import _paths, _commands
    from v3.adapters.nav2_navigation import decode_guidance
    from v3.contracts import (LocalizationQuality, MissionConstraints, MotionIntent,
                              QualityState, RobotEstimate, TickContext)
    from v3.layers.l9_operational_constraints import OperationalConstraintLayer, OperationalConstraintsConfig
    from v3.wheel_motion import WheelMotionLimits

    bridge_socket, bridge_status = output / "bridge.sock", output / "bridge_status.json"
    if len(str(bridge_socket).encode()) > 100:
        raise ValueError("Probe output directory is too long for its Unix socket")
    paths = _paths(root)
    paths.update(work=output, socket=bridge_socket, status=bridge_status, geometry=output / "geometry.yaml")
    radius = args.robot_radius
    simulation_config = json.loads((output / "simulation_config.json").read_text())
    operational = simulation_config["operational"]
    wheel_limits = WheelMotionLimits(**operational.pop("wheel_limits"))
    operational = OperationalConstraintsConfig(**operational, wheel_limits=wheel_limits)
    constraints = MissionConstraints(**simulation_config["mission"])
    limiter = OperationalConstraintLayer(operational)
    quality = LocalizationQuality(local_translation=QualityState.GOOD, heading=QualityState.GOOD,
        global_position=QualityState.GOOD, local_sigma_m=.01, global_sigma_m=.01, yaw_sigma_rad=.01,
        encoder_age_ns=0, imu_age_ns=0, lidar_age_ns=0, global_fix_age_ns=0, relative_age_ns=0,
        local_pose_continuous=True, observability=1.)
    channel = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    channel.setblocking(False)
    channel.bind(f"\0r2b4-nav2-probe-{os.getpid()}")
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node("r2b4_motorless_navigation_probe")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    map_publisher = node.create_publisher(OccupancyGrid, "/map", qos)
    odom_publisher = node.create_publisher(Odometry, "/r2b4/odom", qos_profile_sensor_data)
    scan_publisher = node.create_publisher(LaserScan, "/r2b4/slam/scan", qos_profile_sensor_data)
    tf_publisher, static_publisher = TransformBroadcaster(node), StaticTransformBroadcaster(node)
    pose = [0., 0., 0.]
    obstacle = False
    plans = dict(total=0, after_obstacle=0, detour_after_obstacle=0)
    action_ids, explore_baseline_ids, explore_succeeded_ids = set(), set(), set()
    explore_children = {}

    def plan_received(message):
        plans["total"] += 1
        if obstacle:
            plans["after_obstacle"] += 1
            if any(abs(item.pose.position.y) > OBSTACLE[3] + radius for item in message.poses):
                plans["detour_after_obstacle"] += 1

    node.create_subscription(RosPath, "/plan", plan_received, 1)

    def action_status_received(message):
        for item in message.status_list:
            identity = bytes(item.goal_info.goal_id.uuid).hex()
            action_ids.add(identity)
            if phase == "EXPLORE" and identity not in explore_baseline_ids and item.status == 4:
                explore_succeeded_ids.add(identity)

    node.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", action_status_received, qos)

    def transform(parent, child, stamp, x=0., y=0., yaw=0.):
        value = TransformStamped()
        value.header.frame_id, value.child_frame_id, value.header.stamp = parent, child, stamp
        value.transform.translation.x, value.transform.translation.y = float(x), float(y)
        value.transform.rotation.z, value.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return value

    stamp = node.get_clock().now().to_msg()
    static_publisher.sendTransform([transform(MAP_FRAME, ODOM_FRAME, stamp),
                                   transform(BASE_FRAME, LASER_FRAME, stamp)])

    def publish_map():
        message = OccupancyGrid()
        message.header.frame_id, message.header.stamp = MAP_FRAME, node.get_clock().now().to_msg()
        message.info.resolution = .05
        message.info.width, message.info.height = 160, 120
        message.info.origin.position.x, message.info.origin.position.y = -2., -3.
        message.info.origin.orientation.w = 1.
        cells = []
        for row in range(message.info.height):
            y = -3. + (row + .5) * .05
            for col in range(message.info.width):
                x = -2. + (col + .5) * .05
                occupied = (not ROOM[0] < x < ROOM[2] or not ROOM[1] < y < ROOM[3]
                            or obstacle and OBSTACLE[0] <= x <= OBSTACLE[2] and OBSTACLE[1] <= y <= OBSTACLE[3])
                cells.append(100 if occupied else 0)
        message.data = cells
        map_publisher.publish(message)

    def publish_sensors(v, omega, *, scan):
        stamp = node.get_clock().now().to_msg()
        tf_publisher.sendTransform(transform(ODOM_FRAME, BASE_FRAME, stamp, *pose))
        message = Odometry()
        message.header.frame_id, message.child_frame_id, message.header.stamp = ODOM_FRAME, BASE_FRAME, stamp
        message.pose.pose.position.x, message.pose.pose.position.y = pose[:2]
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        message.twist.twist.linear.x, message.twist.twist.angular.z = float(v), float(omega)
        odom_publisher.publish(message)
        if scan:
            laser = LaserScan()
            laser.header.frame_id, laser.header.stamp = LASER_FRAME, stamp
            laser.angle_min, laser.angle_max = -math.pi, math.pi
            laser.angle_increment, laser.scan_time = 2 * math.pi / 180, .1
            laser.range_min, laser.range_max = .05, 3.5
            ranges = []
            for beam in range(181):
                angle = pose[2] - math.pi + beam * laser.angle_increment
                dx, dy = math.cos(angle), math.sin(angle)
                distance = _ray_box(*pose[:2], dx, dy, ROOM)
                if obstacle:
                    distance = min(distance, _ray_box(*pose[:2], dx, dy, OBSTACLE))
                ranges.append(float(distance) if .05 <= distance <= 3.5 else math.inf)
            laser.ranges = ranges
            scan_publisher.publish(laser)

    commands = _commands(paths, full=True, radius=radius)
    children, handles = [], []
    report = dict(schema="R2B4_NAV2_MOTORLESS_PROBE_V1", status="FAILED", motor_authority=False,
        ros_domain_id=args.ros_domain_id, output=str(output), radius_m=radius,
        fixed_goal_reached=False, obstacle_goal_reached=False, heartbeat_stop=False,
        explore_parent_stable=False, explore_child_goals=0, explore_succeeded_goals=0,
        simulated_motion="production_L9_with_synthetic_localization", unrealizable_proposals=0,
        projected_proposals=0, max_normal_wheel_acceleration_mps2=0., max_raw_omega_rad_s=0.,
        max_obstacle_detour_m=0., min_obstacle_clearance_m=None, plans=plans,
        active_commands=0, expired_commands_rejected=0, error=None)
    trace = (output / "trace.jsonl").open("w")
    start = time.monotonic()
    last_motion_ns = time.monotonic_ns()
    last_sensor = last_scan = last_map = last_heartbeat = last_trace = 0.
    phase, phase_started, obstacle_inserted = "STARTING", start, None
    phase_id, tick = 0, 0
    target, latest, mode = None, None, "NAVIGATE"
    heartbeat = True
    heartbeat_stopped_ns = None
    stop_pose = None
    ttl_positive_seen = False
    v = omega = raw_v = raw_omega = 0.
    control_tick = 0
    try:
        publish_map()
        publish_sensors(0., 0., scan=True)
        for name, command in commands:
            log = (output / (name + ".log")).open("w")
            handles.append(log)
            children.append(subprocess.Popen(command, cwd=root, stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        while time.monotonic() - start < args.timeout_s:
            rclpy.spin_once(node, timeout_sec=.002)
            now = time.monotonic()
            now_ns = time.monotonic_ns()
            for child, (name, _) in zip(children, commands):
                if child.poll() is not None:
                    raise RuntimeError(f"{name} exited with {child.returncode}; inspect its log")
            for _ in range(32):
                try:
                    payload, address = channel.recvfrom(4097)
                except BlockingIOError:
                    break
                if address != str(bridge_socket) or len(payload) > 4096:
                    raise RuntimeError("invalid bridge sender or packet size")
                packet = json.loads(payload)
                guidance = decode_guidance(packet)
                if (packet.get("status") == "SUCCEEDED"
                        and not guidance.produced_ns < time.monotonic_ns() < guidance.valid_until_ns):
                    raise RuntimeError("SUCCEEDED guidance is not fresh at receipt")
                if packet.get("mission_id") == f"motorless-{phase_id}":
                    if packet.get("status") == "SUCCEEDED" and mode == "NAVIGATE":
                        error = math.hypot(target[0] - pose[0], target[1] - pose[1])
                        yaw_error = abs(math.atan2(math.sin(target[2] - pose[2]), math.cos(target[2] - pose[2])))
                        if error > constraints.goal_tolerance_m or yaw_error > constraints.yaw_tolerance_rad:
                            raise RuntimeError("Nav2 success exceeded the requested mission tolerance")
                    latest = packet
                    if phase == "EXPLORE" and packet.get("selected_goal") is not None:
                        if packet.get("goal_id") != "motorless-goal-3":
                            raise RuntimeError("EXPLORE parent goal identity changed")
                        child = packet["child_goal_id"]
                        selected = tuple(packet["selected_goal"])
                        if child in explore_children and explore_children[child] != selected:
                            raise RuntimeError("EXPLORE changed a retained child goal")
                        explore_children[child] = selected
                    if packet.get("status") == "ACTIVE":
                        report["active_commands"] += 1
                        if (heartbeat_stopped_ns is not None and packet["produced_ns"] > heartbeat_stopped_ns + 350_000_000
                                and abs(packet["v_mps"]) + abs(packet["omega_rad_s"]) > 1e-8):
                            raise RuntimeError("positive command after heartbeat deadline")
            raw_v = raw_omega = 0.
            active_command = False
            if latest is not None and latest.get("status") == "ACTIVE":
                if latest["produced_ns"] <= now_ns < latest["valid_until_ns"]:
                    raw_v, raw_omega = latest["v_mps"], latest["omega_rad_s"]
                    active_command = abs(raw_v) + abs(raw_omega) > 1e-12
                else:
                    report["expired_commands_rejected"] += 1
            if now_ns - last_motion_ns >= 20_000_000:
                dt, last_motion_ns = (now_ns - last_motion_ns) / 1e9, now_ns
                context = TickContext(control_tick, now_ns)
                control_tick += 1
                estimate = RobotEstimate(context, ODOM_FRAME, *pose, v, omega, (0.,) * 25,
                                         localization_quality=quality)
                motion = MotionIntent(context, raw_v, raw_omega, 200_000_000, constraints,
                    stop_reason=None if active_command else "NAV2_INPUT_UNAVAILABLE",
                    transition_allowed=active_command)
                previous_wheels = wheel_limits.wheels(v, omega)
                realized = limiter.evaluate(motion, estimate)
                v, omega = realized.allowed_v_mps, realized.allowed_omega_rad_s
                report["max_raw_omega_rad_s"] = max(report["max_raw_omega_rad_s"], abs(raw_omega))
                if active_command:
                    steady = wheel_limits.constrain(raw_v, raw_omega,
                        max_v_mps=min(constraints.max_v_mps, operational.max_v_mps),
                        max_omega_rad_s=min(constraints.max_omega_rad_s, operational.max_omega_rad_s),
                        max_curvature_rad_per_m=operational.max_curvature_rad_per_m)
                    if abs(steady[0]) + abs(steady[1]) < 1e-12:
                        report["unrealizable_proposals"] += 1
                        report["last_unrealizable_proposal"] = [raw_v, raw_omega]
                    if steady != (raw_v, raw_omega):
                        report["projected_proposals"] += 1
                    acceleration = max(abs(current - previous) / dt for current, previous
                        in zip(wheel_limits.wheels(v, omega), previous_wheels))
                    report["max_normal_wheel_acceleration_mps2"] = max(
                        report["max_normal_wheel_acceleration_mps2"], acceleration)
                mid_yaw = pose[2] + omega * dt / 2
                pose[0] += v * math.cos(mid_yaw) * dt
                pose[1] += v * math.sin(mid_yaw) * dt
                pose[2] = math.atan2(math.sin(pose[2] + omega * dt), math.cos(pose[2] + omega * dt))
            clearance = _clearance(*pose[:2], radius, obstacle)
            if clearance < -.001:
                raise RuntimeError(f"simulated footprint collision: {clearance:.4f}m")
            if obstacle:
                previous = report["min_obstacle_clearance_m"]
                report["min_obstacle_clearance_m"] = clearance if previous is None else min(previous, clearance)
                report["max_obstacle_detour_m"] = max(report["max_obstacle_detour_m"], abs(pose[1]))
            if now - last_sensor >= .02:
                publish_sensors(v, omega, scan=now - last_scan >= .1)
                if now - last_scan >= .1:
                    last_scan = now
                last_sensor = now
            if now - last_map >= .5:
                publish_map()
                last_map = now
            if phase == "STARTING":
                try:
                    ready = json.loads(bridge_status.read_text()).get("state") == "RUNNING"
                except (OSError, ValueError):
                    ready = False
                if ready:
                    phase, phase_started, phase_id, target = "FIXED_GOAL", now, 1, [1., 0., 0.]
                    report["startup_s"] = now - start
                elif now - start > 30.:
                    raise TimeoutError("Nav2 lifecycle startup timeout")
            elif phase == "FIXED_GOAL" and latest is not None and latest.get("status") == "SUCCEEDED":
                report["fixed_goal_reached"], report["fixed_goal_pose"] = True, list(pose)
                phase, phase_started, phase_id, target, latest = "OBSTACLE", now, 2, [3.5, 0., 0.], None
            elif phase == "OBSTACLE":
                if not obstacle and now - phase_started >= .8:
                    obstacle, obstacle_inserted = True, now
                    publish_map()
                if latest is not None and latest.get("status") == "SUCCEEDED":
                    report["obstacle_goal_reached"], report["obstacle_goal_pose"] = True, list(pose)
                    if plans["detour_after_obstacle"] < 1 or report["max_obstacle_detour_m"] <= OBSTACLE[3] + radius:
                        raise RuntimeError("obstacle detour/replan was not observed")
                    phase, phase_started, phase_id, target, latest, mode = "EXPLORE", now, 3, None, None, "EXPLORE"
                    explore_baseline_ids = set(action_ids)
            elif phase == "EXPLORE":
                report["explore_child_goals"] = len(explore_children)
                report["explore_succeeded_goals"] = len(explore_succeeded_ids)
                if (len(explore_children) >= 2 and explore_succeeded_ids and latest is not None
                        and latest.get("status") == "ACTIVE"):
                    report["explore_parent_stable"] = True
                    report["explore_goals"] = list(explore_children.values())
                    phase, phase_started, phase_id, target, latest, mode = "HEARTBEAT_STOP", now, 4, [4.5, 0., 0.], None, "NAVIGATE"
            elif phase == "HEARTBEAT_STOP":
                ttl_positive_seen = ttl_positive_seen or abs(v) + abs(omega) > 1e-5
                if heartbeat and ttl_positive_seen and now - phase_started >= .8:
                    heartbeat = False
                    heartbeat_stopped_ns = now_ns
                if heartbeat_stopped_ns is not None and now_ns - heartbeat_stopped_ns > 600_000_000:
                    if stop_pose is None:
                        stop_pose = list(pose)
                    if abs(v) + abs(omega) > 1e-8:
                        raise RuntimeError("velocity did not expire after heartbeat loss")
                    if now_ns - heartbeat_stopped_ns > 1_000_000_000:
                        report["heartbeat_stop"] = math.hypot(pose[0] - stop_pose[0], pose[1] - stop_pose[1]) < .001
                        report["heartbeat_stop_pose"] = list(pose)
                        report["status"] = "PASS" if report["heartbeat_stop"] else "FAILED"
                        break
            if phase != "STARTING" and heartbeat and now - last_heartbeat >= .05:
                request = dict(operation="NAVIGATE", mission_id=f"motorless-{phase_id}",
                    goal_id=f"motorless-goal-{phase_id}", source_ns=now_ns, source_tick_id=tick,
                    odom_generation=0, transform_revision=0, frame_id=MAP_FRAME, mode=mode,
                    goal=target, max_v_mps=constraints.max_v_mps, max_omega_rad_s=constraints.max_omega_rad_s)
                request.update(goal_tolerance_m=constraints.goal_tolerance_m,
                               yaw_tolerance_rad=constraints.yaw_tolerance_rad)
                channel.sendto(json.dumps(request, allow_nan=False, separators=(",", ":")).encode(), str(bridge_socket))
                last_heartbeat = now
                tick += 1
            if latest is not None and latest.get("status") in ("FAILED", "NO_PATH", "UNAVAILABLE"):
                raise RuntimeError("Nav2 terminal status: " + latest["status"])
            if now - last_trace >= .1:
                trace.write(json.dumps(dict(t_s=now - start, phase=phase, pose=pose, v=v, omega=omega,
                    raw_v=raw_v, raw_omega=raw_omega,
                    status=None if latest is None else latest.get("status"), obstacle=obstacle)) + "\n")
                trace.flush()
                last_trace = now
        else:
            raise TimeoutError(f"probe deadline during {phase}")
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}:{exc}"[:500]
        if isinstance(exc, (KeyboardInterrupt, SystemExit)):
            raise
    finally:
        report.update(duration_s=time.monotonic() - start, final_pose=list(pose), phase=phase,
                      obstacle_inserted_s=None if obstacle_inserted is None else obstacle_inserted - start)
        try:
            report["bridge"] = json.loads(bridge_status.read_text())
            applied = report["bridge"].get("goal_checker_tolerances")
            report["goal_checker_tolerances_applied"] = (isinstance(applied, list)
                and applied[:2] == [constraints.goal_tolerance_m / 2, constraints.yaw_tolerance_rad / 2])
            if report["status"] == "PASS" and not report["goal_checker_tolerances_applied"]:
                report.update(status="FAILED", error="requested goal checker tolerances were not applied")
        except (OSError, ValueError):
            pass
        for child in reversed(children):
            if child.poll() is None:
                child.terminate()
        for child in reversed(children):
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=3)
        for handle in handles:
            handle.close()
        trace.close()
        channel.close()
        node.destroy_node()
        rclpy.shutdown()
        (output / "summary.json").write_text(json.dumps(report, indent=2, allow_nan=False, sort_keys=True) + "\n")
    print(json.dumps(report, sort_keys=True))
    return 0 if report["status"] == "PASS" else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--ros-domain-id", type=int, default=75)
    parser.add_argument("--timeout-s", type=float, default=100.)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--robot-radius", type=float, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if not 0 <= args.ros_domain_id <= 232 or args.ros_domain_id == 74:
        parser.error("Use a valid isolated ROS domain other than live domain 74")
    if not math.isfinite(args.timeout_s) or not 10. <= args.timeout_s <= 180.:
        parser.error("timeout must be 10..180 seconds")
    root = Path(__file__).resolve().parents[1]
    if args.worker:
        if (args.output is None or os.environ.get("ROS_DOMAIN_ID") != str(args.ros_domain_id)
                or args.robot_radius is None or not math.isfinite(args.robot_radius) or args.robot_radius <= 0):
            parser.error("worker requires its isolated domain and output")
        return _worker(args, root, args.output.resolve())
    (root / "tmp").mkdir(exist_ok=True)
    output = (Path(tempfile.mkdtemp(prefix="nav2_probe_", dir=root / "tmp")) if args.output is None
              else args.output.resolve())
    if args.output is not None:
        output.mkdir(parents=True, exist_ok=False)
    from tools.nav2_planner import _paths, _resolved_control, _write_geometry, _navigation_environment
    paths = _paths(root)
    paths.update(work=output, socket=output / "bridge.sock", status=output / "bridge_status.json",
                 geometry=output / "geometry.yaml")
    control = _resolved_control(root)
    radius = _write_geometry(paths, control)
    mission = asdict(control.mission.default_constraints)
    mission["max_v_mps"] = min(.25, mission["max_v_mps"], control.operational_constraints.max_v_mps)
    mission["max_omega_rad_s"] = min(1.4, mission["max_omega_rad_s"], control.operational_constraints.max_omega_rad_s)
    (output / "simulation_config.json").write_text(json.dumps(dict(
        operational=asdict(control.operational_constraints), mission=mission), sort_keys=True) + "\n")
    prefix = root / "runtime/deps/nav2-planner"
    environment = _navigation_environment(paths, root)
    environment.update(ROS_DOMAIN_ID=str(args.ros_domain_id), ROS_LOG_DIR=str(output / "ros_logs"))
    command = [str(prefix / "bin/python"), "-m", "tools.nav2_navigation_probe", "--worker",
               "--output", str(output), "--ros-domain-id", str(args.ros_domain_id), "--timeout-s", str(args.timeout_s),
               "--robot-radius", str(radius)]
    return subprocess.run(command, cwd=root, env=environment, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
