"""Host lifecycle for isolated Nav2 compute; V3 retains all motor authority."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import fcntl
from functools import partial
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from tools.slam_toolbox import _read_json, _write_json, _process_start, _ros_environment, _terminate_with_parent


def _paths(root):
    work = root / "runtime" / "nav2_planner"
    full = _read_json(root / "conf" / "vezerles.json").get("runtime", {}).get("nav2_navigation") is not None
    return {"work": work, "state": work / "supervisor.json", "status": work / "bridge_status.json",
            "socket": root / "runtime" / ("nav2_navigation.sock" if full else "nav2_planner.sock"),
            "prefix": root / "runtime" / "deps" / "nav2-planner",
            "overlay": root / "runtime" / "deps" / "nav2-wheel-adapter",
            "params": root / "conf" / "nav2_planner.yaml", "geometry": work / "geometry.yaml",
            "bt": root / "conf" / "nav2_navigate.xml"}


def _resolved_control(root):
    from v3.config import ConfigResolver
    conf = root / "conf"
    return ConfigResolver(conf/"hardver.json", conf/"fizika.json", conf/"speed_map.json",
                          conf/"vezerles.json").resolve().runtime.composition.live_control.control


def _navigation_environment(paths, root):
    env = _ros_environment(root, paths["prefix"])
    env["AMENT_PREFIX_PATH"] = str(paths["overlay"]) + os.pathsep + env.get("AMENT_PREFIX_PATH", "")
    env["LD_LIBRARY_PATH"] = str(paths["overlay"] / "lib") + os.pathsep + env.get("LD_LIBRARY_PATH", "")
    return env


def _write_geometry(paths, control):
    navigation = control.navigation
    radius = math.hypot(navigation.footprint_length_m/2 + abs(navigation.footprint_center_x_m),
                        navigation.footprint_width_m/2) + navigation.footprint_safety_margin_m
    text = ""
    for name in ("global_costmap", "local_costmap"):
        text += (f"{name}:\n  {name}:\n    ros__parameters:\n"
                 f"      robot_radius: {radius}\n      inflation_layer:\n"
                 f"        inflation_radius: {radius+.10}\n")
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
    # The default tree is loaded during lifecycle activation, before the first
    # goal can supply its own XML. It must not require unused recovery servers.
    import json
    text += ("bt_navigator:\n  ros__parameters:\n"
             f"    default_nav_to_pose_bt_xml: {json.dumps(str(paths['bt']))}\n")
    paths["geometry"].write_text(text)
    return radius


def _commands(paths, *, full, radius):
    servers = ("nav2_planner", "planner_server"),
    if full:
        servers += (("nav2_controller", "controller_server"), ("nav2_bt_navigator", "bt_navigator"))
    commands = []
    for package, executable in servers:
        command = [str(paths["prefix"] / "lib" / package / executable), "--ros-args",
                   "--params-file", str(paths["params"]), "--params-file", str(paths["geometry"])]
        if executable == "controller_server":
            command += ["-r", "cmd_vel:=/r2b4/nav2/cmd_vel"]
        commands.append((executable, command))
    command = [str(paths["prefix"] / "bin" / "python"), "-m",
               "v3.nav2_navigation_bridge" if full else "v3.nav2_planner_bridge",
               "--socket", str(paths["socket"]), "--status-path", str(paths["status"])]
    if full:
        command += ["--behavior-tree", str(paths["bt"]), "--robot-radius", str(radius)]
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
    return result


def start(root, *, ros_domain_id=74):
    paths = _paths(root)
    control = _resolved_control(root)
    full = control.navigation.full_nav2_enabled
    with _lock(paths):
        if _owned(_read_json(paths["state"]), root):
            return status(root)
        dependencies = [paths["prefix"] / "bin" / "python", paths["params"],
                        paths["prefix"] / "lib" / "nav2_planner" / "planner_server"]
        if full:
            dependencies += [paths["bt"], paths["prefix"] / "lib" / "nav2_controller" / "controller_server",
                             paths["prefix"] / "lib" / "nav2_bt_navigator" / "bt_navigator",
                             paths["prefix"] / "lib" / "libnav2_regulated_pure_pursuit_controller.so",
                             paths["overlay"] / "lib" / "libr2b4_nav2_wheel_adapter.so"]
        for dependency in dependencies:
            if not dependency.is_file():
                raise RuntimeError(f"Nav2 dependency missing: {dependency}")
        if paths["socket"].exists():
            raise RuntimeError(f"Nav2 socket already exists: {paths['socket']}")
        paths["status"].unlink(missing_ok=True)
        with (paths["work"] / "supervisor.log").open("a") as log:
            child = subprocess.Popen([sys.executable, "-m", "tools.nav2_planner", "__supervise",
                "--root", str(root), "--ros-domain-id", str(ros_domain_id)], cwd=root,
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+30
        try:
            while time.monotonic() < deadline:
                current = status(root)
                if current.get("pid") == child.pid and current["state"] == "RUNNING":
                    return current
                if child.poll() is not None:
                    raise RuntimeError(f"Nav2 startup failed; see {paths['work']}")
                time.sleep(.1)
            raise RuntimeError(f"Nav2 startup timeout; see {paths['work']}")
        except BaseException:
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
            raise


def stop(root):
    paths = _paths(root)
    with _lock(paths):
        state = _read_json(paths["state"])
        if _owned(state, root):
            os.kill(state["pid"], signal.SIGTERM)
            deadline = time.monotonic()+6
            while time.monotonic() < deadline and _owned(state, root):
                time.sleep(.1)
            if _owned(state, root):
                os.killpg(state["pid"], signal.SIGKILL)
        elif state.get("state") in ("RUNNING", "STARTING"):
            raise RuntimeError("Nav2 process identity lost; refusing to signal unverified process")
        return status(root)


def _supervise(root, ros_domain_id):
    paths = _paths(root)
    stopped = False

    def request_stop(_signal, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    state = {"state": "STARTING", "pid": os.getpid(), "process_start": _process_start(os.getpid()),
             "root": str(root), "ros_domain_id": ros_domain_id, "motor_authority": False}
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
        control = _resolved_control(root)
        full = control.navigation.full_nav2_enabled
        state["mode"] = "NAVIGATION" if full else "PLANNER"
        radius = _write_geometry(paths, control)
        env = _navigation_environment(paths, root)
        env.update(ROS_DOMAIN_ID=str(ros_domain_id), ROS_LOG_DIR=str(paths["work"] / "ros_logs"))
        for name, command in _commands(paths, full=full, radius=radius):
            log = (paths["work"] / f"{name}.log").open("a")
            logs.append(log)
            children.append(subprocess.Popen(command, cwd=root, env=env, stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT,
                preexec_fn=partial(_terminate_with_parent, os.getpid())))
        state["children"] = [child.pid for child in children]
        while not stopped:
            if any(child.poll() is not None for child in children):
                raise RuntimeError("NAV2_CHILD_EXITED")
            if _read_json(paths["status"]).get("state") == "RUNNING" and state["state"] != "RUNNING":
                state["state"] = "RUNNING"
                _write_json(paths["state"], state)
            time.sleep(.1)
    except BaseException as exc:
        state["state"], state["error"] = "FAILED", f"{type(exc).__name__}:{exc}"[:256]
        raise
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.terminate()
        for child in reversed(children):
            try:
                child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=2)
        for log in logs:
            log.close()
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
