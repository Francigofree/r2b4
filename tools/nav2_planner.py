"""Host lifecycle for isolated Nav2 compute; V3 retains all motor authority."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import fcntl
from functools import partial
import math
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import time

from tools.slam_toolbox import _read_json, _write_json, _process_start, _ros_environment, _terminate_with_parent
from v3.adapters.nav2_lifecycle import Nav2LifecycleMonitor


def _bridge_ready(bridge, now_ns, *, pid):
    lifecycle = bridge.get("lifecycle_status", {})
    checked = lifecycle.get("last_response_checked_ns")
    return (type(pid) is int and pid > 1 and bridge.get("pid") == pid and bridge.get("state") == "RUNNING"
            and lifecycle.get("active") is True and type(checked) is int
            and 0 < checked <= now_ns and now_ns-checked < Nav2LifecycleMonitor.FRESHNESS_NS)


def _socket_identity(path):
    try:
        value = path.stat()
        return [value.st_dev, value.st_ino] if stat.S_ISSOCK(value.st_mode) else None
    except FileNotFoundError:
        return None


def _children_alive(state):
    return any(started is not None and _process_start(int(pid)) == started
               for pid, started in state.get("child_process_starts", {}).items())


def _remove_owned_socket(paths, state, root):
    """Recover only the exact endpoint recorded by a now-dead own session."""
    children = state.get("child_process_starts", {})
    identity = state.get("socket_identity")
    if (state.get("root") != str(root) or _process_start(state.get("pid")) is not None
            or not identity or not children or any(value is None for value in children.values())
            or _children_alive(state)
            or _socket_identity(paths["socket"]) != identity):
        return False
    paths["socket"].unlink(missing_ok=True)
    return True


def _paths(root):
    work = root / "runtime" / "nav2_planner"
    full = _read_json(root / "conf" / "vezerles.json").get("runtime", {}).get("nav2_navigation") is not None
    return {"work": work, "state": work / "supervisor.json", "status": work / "bridge_status.json",
            "socket": root / "runtime" / ("nav2_navigation.sock" if full else "nav2_planner.sock"),
            "prefix": root / "runtime" / "deps" / "nav2-planner",
            "overlay": root / "runtime" / "deps" / "nav2-wheel-adapter",
            "frontier": root / "runtime" / "deps" / "frontier-exploration",
            "frontier_params": root / "conf" / "frontier_exploration.yaml",
            "params": root / "conf" / "nav2_planner.yaml", "geometry": work / "geometry.yaml",
            "bt": root / "conf" / "nav2_navigate.xml"}


def _resolved_config(root):
    from v3.config import ConfigResolver
    conf = root / "conf"
    return ConfigResolver(conf/"hardver.json", conf/"fizika.json", conf/"speed_map.json",
                          conf/"vezerles.json").resolve()


def _resolved_control(root):
    return _resolved_config(root).runtime.composition.live_control.control


def _navigation_environment(paths, root):
    env = _ros_environment(root, paths["prefix"])
    # Bound FastDDS service/action response discovery waits in this edge only.
    # RMW 8.4.4 loads the service profile without enabling XML overrides for its
    # history-memory policy, so retain the normal RMW allocation defaults.
    env["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(root / "conf" / "nav2_fastdds.xml")
    env["AMENT_PREFIX_PATH"] = str(paths["overlay"]) + os.pathsep + env.get("AMENT_PREFIX_PATH", "")
    env["LD_LIBRARY_PATH"] = str(paths["overlay"] / "lib") + os.pathsep + env.get("LD_LIBRARY_PATH", "")
    frontier = paths["frontier"]
    env["AMENT_PREFIX_PATH"] = str(frontier) + os.pathsep + env["AMENT_PREFIX_PATH"]
    env["LD_LIBRARY_PATH"] = str(frontier / "lib") + os.pathsep + env["LD_LIBRARY_PATH"]
    env["PYTHONPATH"] = str(frontier / "lib/python3.12/site-packages") + os.pathsep + env.get("PYTHONPATH", "")
    return env


def _write_geometry(paths, control):
    import json
    navigation = control.navigation
    rear = navigation.footprint_center_x_m - navigation.footprint_length_m/2
    front = navigation.footprint_center_x_m + navigation.footprint_length_m/2
    half_width = navigation.footprint_width_m/2
    padding = max(navigation.footprint_safety_margin_m,
                  control.lidar_safety.minimum_clearance_m)
    footprint = [[rear, -half_width], [front, -half_width],
                 [front, half_width], [rear, half_width]]
    radius = math.hypot(max(abs(rear), abs(front)) + padding, half_width + padding)
    text = ""
    for name in ("global_costmap", "local_costmap"):
        text += (f"{name}:\n  {name}:\n    ros__parameters:\n"
                 f"      footprint: {json.dumps(json.dumps(footprint))}\n"
                 f"      footprint_padding: {padding}\n")
    limits = control.operational_constraints
    minimum = limits.wheel_limits.target_minimum_mps
    constraints = control.mission.default_constraints
    # RPP proposes steady targets from measured odometry. Its own startup ramp
    # must cross the wheel deadband in one controller step; the physical ramp
    # remains exclusively limited by V3 L9, using the unchanged platform limit.
    proposal_acceleration = max(limits.max_angular_acceleration_rad_s2,
                                limits.wheel_limits.target_center_spin_rad_s * 20.0 * 1.1)
    text += ("controller_server:\n  ros__parameters:\n    goal_checker:\n"
             f"      xy_goal_tolerance: {constraints.goal_tolerance_m / 2}\n"
             f"      yaw_goal_tolerance: {constraints.yaw_tolerance_rad / 2}\n"
             "    FollowPath:\n"
             f"      desired_linear_vel: {min(.30, limits.max_v_mps)}\n"
             f"      regulated_linear_scaling_min_speed: {minimum}\n"
             f"      min_approach_linear_velocity: {minimum}\n"
             f"      rotate_to_heading_angular_vel: {max(limits.wheel_limits.target_center_spin_rad_s, min(1.2, limits.max_omega_rad_s))}\n"
             f"      max_angular_accel: {proposal_acceleration}\n"
             f"      track_width_m: {limits.wheel_limits.track_width_m}\n"
             f"      wheel_minimum_mps: {limits.wheel_limits.minimum_mps}\n"
             f"      wheel_maximum_mps: {limits.wheel_limits.maximum_mps}\n"
             f"      platform_max_v_mps: {limits.max_v_mps}\n"
             f"      platform_max_omega_rad_s: {limits.max_omega_rad_s}\n"
             f"      max_curvature_rad_per_m: {limits.max_curvature_rad_per_m}\n")
    # Recovery proposals cross the same physical wheel and L9/L12 boundaries.
    text += ("behavior_server:\n  ros__parameters:\n"
             f"    min_rotational_vel: {limits.wheel_limits.target_center_spin_rad_s}\n"
             f"    max_rotational_vel: {max(limits.wheel_limits.target_center_spin_rad_s, min(1.2, limits.max_omega_rad_s))}\n"
             f"    rotational_acc_lim: {limits.max_angular_acceleration_rad_s2}\n")
    # Load the maintained tree during lifecycle activation, before admission.
    text += ("bt_navigator:\n  ros__parameters:\n"
             f"    default_nav_to_pose_bt_xml: {json.dumps(str(paths['bt']))}\n")
    paths["geometry"].write_text(text)
    return radius


def _commands(paths, *, full, radius, robot_geometry=None):
    servers = ("nav2_planner", "planner_server"),
    if full:
        servers += (("nav2_controller", "controller_server"),
                    ("nav2_behaviors", "behavior_server"), ("nav2_bt_navigator", "bt_navigator"))
    commands = []
    for package, executable in servers:
        command = [str(paths["prefix"] / "lib" / package / executable), "--ros-args",
                   "--params-file", str(paths["params"]), "--params-file", str(paths["geometry"])]
        if executable in ("controller_server", "behavior_server"):
            command += ["-r", "cmd_vel:=/r2b4/nav2/cmd_vel"]
        commands.append((executable, command))
    # The manager is the sole ROS lifecycle owner. Process supervision remains
    # here; disable manager respawn so two independent recovery loops cannot
    # reconfigure the same servers.
    managed = [executable for _, executable in servers]
    commands.append(("lifecycle_manager", [str(paths["prefix"] / "lib" /
        "nav2_lifecycle_manager" / "lifecycle_manager"), "--ros-args",
        "-r", "__node:=lifecycle_manager_navigation",
        "-p", "autostart:=true", "-p", "node_names:=[" + ",".join(managed) + "]",
        "-p", "bond_timeout:=2.0", "-p", "attempt_respawn_reconnection:=false",
        "-p", "diagnostic_updater.period:=0.2"]))
    command = [str(paths["prefix"] / "bin" / "python"), "-m",
               "v3.nav2_navigation_bridge" if full else "v3.nav2_planner_bridge",
               "--socket", str(paths["socket"]), "--status-path", str(paths["status"])]
    if full:
        if robot_geometry is None:
            raise ValueError("Nav2 diagnostics requires resolved robot geometry")
        command += ["--behavior-tree", str(paths["bt"]),
                    "--frontier-executable", str(paths["frontier"] / "lib/explore_lite/explore"),
                    "--frontier-params", str(paths["frontier_params"]),
                    "--robot-geometry", json.dumps(robot_geometry, allow_nan=False, separators=(",", ":"))]
    return commands + [("bridge", command)]


def _owned(state, root):
    pid = state.get("pid")
    if type(pid) is not int or pid <= 1 or state.get("root") != str(root) or _process_start(pid) != state.get("process_start"):
        return False
    try:
        args = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
        return b"tools.nav2_planner" in args and b"__supervise" in args and os.getpgid(pid) == pid
    except OSError:
        return False


@contextmanager
def _lock(paths):
    paths["work"].mkdir(parents=True, exist_ok=True, mode=0o700)
    with (paths["work"] / "command.lock").open("a") as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        yield


def status(root):
    paths = _paths(root)
    state = _read_json(paths["state"])
    running = _owned(state, root)
    result = {**state, "running": running, "motor_authority": False,
              "state": state.get("state", "STOPPED"), "bridge": _read_json(paths["status"])}
    if not running and result["state"] in ("RUNNING", "STARTING"):
        result["state"] = "PROCESS_LOST"
    result["ready"] = (running and result["state"] == "RUNNING"
                       and _bridge_ready(result["bridge"], time.monotonic_ns(),
                                         pid=state.get("bridge_pid")))
    return result


def _wait_ready(root, pid, child=None):
    deadline = time.monotonic()+30
    while time.monotonic() < deadline:
        current = status(root)
        if current.get("pid") == pid:
            if current["ready"]:
                return current
            if not current["running"] or current["state"] == "FAILED":
                raise RuntimeError(f"Nav2 startup failed: {current.get('error', current['state'])}")
            bridge = current["bridge"]
            if bridge.get("pid") == current.get("bridge_pid") and bridge.get("state") == "FAILED":
                raise RuntimeError(f"Nav2 bridge failed: {bridge.get('error')}")
        elif child is None:
            raise RuntimeError("Nav2 supervisor identity changed while waiting for readiness")
        if child is not None and child.poll() is not None:
            raise RuntimeError(f"Nav2 startup failed; see {_paths(root)['work']}")
        time.sleep(.1)
    raise RuntimeError(f"Nav2 readiness timeout; see {_paths(root)['work']}")


def start(root, *, ros_domain_id=74):
    paths = _paths(root)
    control = _resolved_control(root)
    full = control.navigation.full_nav2_enabled
    with _lock(paths):
        previous = _read_json(paths["state"])
        if _owned(previous, root):
            if previous.get("ros_domain_id") != ros_domain_id:
                raise RuntimeError("Nav2 is already running in a different ROS domain")
            return _wait_ready(root, previous["pid"])
        if previous.get("root") == str(root) and _children_alive(previous):
            raise RuntimeError("Nav2 previous session still has live children; refusing duplicate start")
        dependencies = [paths["prefix"] / "bin" / "python", paths["params"],
                        root / "conf" / "nav2_fastdds.xml",
                        paths["prefix"] / "lib" / "nav2_planner" / "planner_server",
                        paths["prefix"] / "lib" / "nav2_lifecycle_manager" / "lifecycle_manager"]
        if full:
            dependencies += [paths["bt"], paths["prefix"] / "lib" / "nav2_controller" / "controller_server",
                             paths["prefix"] / "lib" / "nav2_behaviors" / "behavior_server",
                             paths["prefix"] / "lib" / "nav2_bt_navigator" / "bt_navigator",
                             paths["prefix"] / "lib" / "libnav2_regulated_pure_pursuit_controller.so",
                             paths["overlay"] / "lib" / "libr2b4_nav2_wheel_adapter.so",
                             paths["frontier"] / "lib/explore_lite/explore", paths["frontier_params"]]
        for dependency in dependencies:
            if not dependency.is_file():
                raise RuntimeError(f"Nav2 dependency missing: {dependency}")
        if paths["socket"].exists() and not _remove_owned_socket(paths, previous, root):
            raise RuntimeError(f"Nav2 socket already exists: {paths['socket']}")
        paths["status"].unlink(missing_ok=True)
        with (paths["work"] / "supervisor.log").open("a") as log:
            child = subprocess.Popen([sys.executable, "-m", "tools.nav2_planner", "__supervise",
                "--root", str(root), "--ros-domain-id", str(ros_domain_id)], cwd=root,
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            return _wait_ready(root, child.pid, child)
        except BaseException:
            if child.poll() is None:
                current = _read_json(paths["state"])
                if current.get("pid") == child.pid and _owned(current, root):
                    _stop_owned(paths, current, root)
                else:
                    child.terminate()
                    try:
                        child.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        # Popen created this dedicated group in this call.
                        os.killpg(child.pid, signal.SIGKILL)
            child.wait(timeout=1)
            raise


def _stop_owned(paths, state, root):
    if not _owned(state, root):
        return
    pid = state["pid"]
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    deadline = time.monotonic()+5
    while time.monotonic() < deadline and _owned(state, root):
        time.sleep(.05)
    latest = _read_json(paths["state"])
    if latest.get("pid") == pid and latest.get("process_start") == state.get("process_start"):
        state = latest
    forced = _owned(state, root)
    if forced:
        # Rechecked identity includes the dedicated process group.
        try:
            os.killpg(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    deadline = time.monotonic()+1
    while time.monotonic() < deadline:
        if not _owned(state, root) and not _children_alive(state):
            break
        time.sleep(.05)
    else:
        raise RuntimeError("Nav2 shutdown deadline exceeded")
    _remove_owned_socket(paths, state, root)
    if forced:
        current = _read_json(paths["state"])
        if current.get("pid") == pid and current.get("process_start") == state.get("process_start"):
            current.update(state="STOPPED", forced_shutdown=True)
            _write_json(paths["state"], current)


def stop(root):
    paths = _paths(root)
    with _lock(paths):
        state = _read_json(paths["state"])
        if _owned(state, root):
            _stop_owned(paths, state, root)
        elif state.get("state") in ("RUNNING", "STARTING"):
            raise RuntimeError("Nav2 process identity lost; refusing to signal unverified process")
        else:
            _remove_owned_socket(paths, state, root)
        return status(root)


def _stop_children(children):
    """One grace period for the whole group, then one bounded reap period."""
    for child in reversed(children):
        if child.poll() is None:
            child.terminate()
    deadline = time.monotonic()+2
    for child in reversed(children):
        try:
            child.wait(timeout=max(0., deadline-time.monotonic()))
        except subprocess.TimeoutExpired:
            pass
    for child in reversed(children):
        if child.poll() is None:
            child.kill()
    deadline = time.monotonic()+1
    pending = []
    for child in reversed(children):
        try:
            child.wait(timeout=max(0., deadline-time.monotonic()))
        except subprocess.TimeoutExpired:
            pending.append(child.pid)
    return pending


def _supervise(root, ros_domain_id):
    paths = _paths(root)
    stopped = False

    def request_stop(_signal, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    state = {"state": "STARTING", "pid": os.getpid(), "process_start": _process_start(os.getpid()),
             "root": str(root), "ros_domain_id": ros_domain_id, "motor_authority": False,
             "lifecycle_owner": "nav2_lifecycle_manager", "children": [], "child_process_starts": {}}
    _write_json(paths["state"], state)
    children, logs = [], []
    try:
        os.nice(10)
        allowed = os.sched_getaffinity(0)
        control = _read_json(root / "conf" / "vezerles.json").get("runtime_affinity", {}).get("control_cpus", [3])
        cpus = (allowed & {1, 2}) - set(control)
        if not cpus:
            cpus = allowed - set(control)
        if not cpus:
            raise RuntimeError("No CPU available outside the configured control CPU")
        cpus = {min(cpus)}
        os.sched_setaffinity(0, cpus)
        state["cpus"] = sorted(cpus)
        resolved = _resolved_config(root)
        control = resolved.runtime.composition.live_control.control
        full = control.navigation.full_nav2_enabled
        state["mode"] = "NAVIGATION" if full else "PLANNER"
        radius = _write_geometry(paths, control)
        env = _navigation_environment(paths, root)
        env.update(ROS_DOMAIN_ID=str(ros_domain_id), ROS_LOG_DIR=str(paths["work"] / "ros_logs"))
        robot_geometry = {"body_bounds_m": resolved.lidar.safety_body_bounds_m,
                          "lidar_xy_m": [resolved.lidar.matcher.lidar_x_offset_m,
                                         resolved.lidar.matcher.lidar_y_offset_m]}
        for name, command in _commands(paths, full=full, radius=radius, robot_geometry=robot_geometry):
            log = (paths["work"] / f"{name}.log").open("a")
            logs.append(log)
            children.append(subprocess.Popen(command, cwd=root, env=env, stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT,
                preexec_fn=partial(_terminate_with_parent, os.getpid())))
            child = children[-1]
            state["children"].append(child.pid)
            state["child_process_starts"][str(child.pid)] = _process_start(child.pid)
            if name == "bridge":
                state["bridge_pid"] = child.pid
        _write_json(paths["state"], state)
        while not stopped:
            if any(child.poll() is not None for child in children):
                raise RuntimeError("NAV2_CHILD_EXITED")
            if "socket_identity" not in state and paths["socket"].exists():
                state["socket_identity"] = _socket_identity(paths["socket"])
                _write_json(paths["state"], state)
            if (_bridge_ready(_read_json(paths["status"]), time.monotonic_ns(), pid=state["bridge_pid"])
                    and state["state"] != "RUNNING"):
                state["state"] = "RUNNING"
                _write_json(paths["state"], state)
            time.sleep(.1)
    except BaseException as exc:
        state["state"], state["error"] = "FAILED", f"{type(exc).__name__}:{exc}"[:256]
        raise
    finally:
        pending = _stop_children(children)
        for log in logs:
            log.close()
        if pending:
            state.update(state="FAILED", error=f"NAV2_CHILD_SHUTDOWN_TIMEOUT:{pending}")
        else:
            # Children have exited; no still-running bridge owns the endpoint.
            paths["socket"].unlink(missing_ok=True)
        if state["state"] != "FAILED":
            state["state"] = "STOPPED"
        _write_json(paths["state"], state)
    return 0


def main(argv=None, *, project_root=None):
    import json
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("start", "stop", "status", "__supervise"))
    parser.add_argument("--root", type=Path, default=project_root or Path(__file__).resolve().parents[1])
    parser.add_argument("--ros-domain-id", type=int, default=74)
    args = parser.parse_args(argv)
    root = args.root.resolve()
    if args.operation == "__supervise":
        return _supervise(root, args.ros_domain_id)
    result = start(root, ros_domain_id=args.ros_domain_id) if args.operation == "start" else globals()[args.operation](root)
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
