"""Host lifecycle for the isolated, non-actuating Nav2 Planner Server."""
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
    return {"work": work, "state": work / "supervisor.json", "status": work / "bridge_status.json",
            "socket": root / "runtime" / "nav2_planner.sock", "prefix": root / "runtime" / "deps" / "nav2-planner",
            "params": root / "conf" / "nav2_planner.yaml", "geometry": work / "geometry.yaml"}


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
    with _lock(paths):
        if _owned(_read_json(paths["state"]), root):
            return status(root)
        for dependency in (paths["prefix"] / "bin" / "python", paths["params"],
                           paths["prefix"] / "lib" / "nav2_planner" / "planner_server"):
            if not dependency.is_file():
                raise RuntimeError(f"Nav2 dependency missing: {dependency}")
        if paths["socket"].exists():
            raise RuntimeError(f"Nav2 socket already exists: {paths['socket']}")
        paths["status"].unlink(missing_ok=True)
        with (paths["work"] / "supervisor.log").open("a") as log:
            child = subprocess.Popen([sys.executable, "-m", "tools.nav2_planner", "__supervise",
                "--root", str(root), "--ros-domain-id", str(ros_domain_id)], cwd=root,
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+15
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
        from v3.config import ConfigResolver
        conf = root / "conf"
        navigation = ConfigResolver(conf/"hardver.json", conf/"fizika.json", conf/"speed_map.json", conf/"vezerles.json").resolve().runtime.composition.live_control.control.navigation
        radius = math.hypot(navigation.footprint_length_m/2 + abs(navigation.footprint_center_x_m),
                            navigation.footprint_width_m/2) + navigation.footprint_safety_margin_m
        paths["geometry"].write_text("global_costmap:\n  global_costmap:\n    ros__parameters:\n"
            f"      robot_radius: {radius}\n      inflation_layer:\n        inflation_radius: {radius+.10}\n")
        env = _ros_environment(root, paths["prefix"])
        env.update(ROS_DOMAIN_ID=str(ros_domain_id), ROS_LOG_DIR=str(paths["work"] / "ros_logs"))
        commands = [
            [str(paths["prefix"] / "lib" / "nav2_planner" / "planner_server"), "--ros-args",
             "--params-file", str(paths["params"]), "--params-file", str(paths["geometry"])],
            [str(paths["prefix"] / "bin" / "python"), "-m", "v3.nav2_planner_bridge",
             "--socket", str(paths["socket"]), "--status-path", str(paths["status"])],
        ]
        for name, command in zip(("planner", "bridge"), commands):
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
