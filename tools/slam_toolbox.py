"""Host lifecycle for SLAM Toolbox; never sends robot motion commands."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import ctypes
import datetime as dt
import fcntl
from functools import partial
import json
import math
import os
from pathlib import Path
import re
import signal
import stat
import subprocess
import sys
import time


def _paths(root: Path) -> dict[str, Path]:
    work = root / "runtime" / "slam_toolbox"
    return {"work": work, "state": work / "supervisor.json",
            "bridge": work / "bridge_status.json", "socket": root / "runtime" / "slam_toolbox.sock",
            "prefix": root / "runtime" / "deps" / "slam-toolbox",
            "solver": root / "runtime" / "deps" / "slam-toolbox-solver",
            "params": root / "conf" / "slam_toolbox.yaml", "maps": work / "maps"}


def _read_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def _write_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        os.chmod(temporary, 0o600)
        json.dump(value, handle, sort_keys=True, allow_nan=False)
        handle.write("\n")
    temporary.replace(path)


def _process_start(pid: int) -> str | None:
    try:
        value = Path(f"/proc/{pid}/stat").read_text()
        fields = value[value.rfind(")") + 2:].split()
        return None if fields[0] == "Z" else fields[19]
    except (OSError, IndexError):
        return None


def _resident_bytes(pid: int) -> int:
    try:
        return int(Path(f"/proc/{pid}/statm").read_text().split()[1]) * os.sysconf("SC_PAGE_SIZE")
    except (OSError, ValueError, IndexError):
        return 0


def _owned_supervisor(state: dict, root: Path) -> bool:
    pid = state.get("pid")
    if type(pid) is not int or pid <= 1 or state.get("root") != str(root):
        return False
    if _process_start(pid) != state.get("process_start"):
        return False
    try:
        args = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
        return (b"tools.slam_toolbox" in args and b"__supervise" in args
                and os.getpgid(pid) == pid)
    except OSError:
        return False


@contextmanager
def _command_lock(paths: dict[str, Path]):
    paths["work"].mkdir(parents=True, exist_ok=True, mode=0o700)
    with (paths["work"] / "command.lock").open("a") as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        yield


def _ros_environment(root: Path, prefix: Path) -> dict[str, str]:
    """Activate only the isolated dependency prefix, never the control Python."""
    env = os.environ.copy()
    env.update({"CONDA_PREFIX": str(prefix), "PATH": str(prefix / "bin") + os.pathsep + env.get("PATH", ""),
                "LD_LIBRARY_PATH": str(prefix / "lib") + os.pathsep + env.get("LD_LIBRARY_PATH", "")})
    setup = next((prefix / name for name in ("setup.bash", "setup.sh") if (prefix / name).is_file()), None)
    script = ('set -e; if [[ -n "$3" ]]; then source "$3" >/dev/null; '
              'else for hook in "$1"/etc/conda/activate.d/*.sh; do '
              'if [[ -f "$hook" ]]; then source "$hook" >/dev/null; fi; done; fi; '
              '"$2" -c \'import json, os; print(json.dumps(dict(os.environ)))\'')
    result = subprocess.run(["bash", "--noprofile", "--norc", "-c", script, "slam-env",
                             str(prefix), str(prefix / "bin" / "python"), str(setup) if setup else ""],
                            env=env, capture_output=True, text=True, timeout=10, check=True)
    env = json.loads(result.stdout)
    overlay = root / "runtime" / "deps" / "slam-toolbox-solver"
    env["AMENT_PREFIX_PATH"] = str(overlay) + os.pathsep + env.get("AMENT_PREFIX_PATH", "")
    env.update({"PYTHONPATH": str(root) + os.pathsep + env.get("PYTHONPATH", ""),
                "ROS_LOCALHOST_ONLY": "1", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
                "ROS_DOMAIN_ID": "74", "ROS_LOG_DIR": str(root / "runtime" / "slam_toolbox" / "ros_logs"),
                "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1",
                "NUMEXPR_NUM_THREADS": "1", "PYTHONUNBUFFERED": "1"})
    return env


def status(root: Path) -> dict:
    paths = _paths(root)
    state = _read_json(paths["state"])
    running = _owned_supervisor(state, root)
    result = {**state, "running": running, "mode": "SLAM_TOOLBOX", "motor_authority": False,
              "state": state.get("state", "STOPPED")}
    if not running and result["state"] in {"STARTING", "RUNNING"}:
        result["state"] = "PROCESS_LOST"
    result["bridge"] = _read_json(paths["bridge"])
    return result


def start(root: Path) -> dict:
    paths = _paths(root)
    with _command_lock(paths):
        if _owned_supervisor(_read_json(paths["state"]), root):
            return status(root)
        for dependency in (paths["prefix"] / "bin" / "python",
                           paths["prefix"] / "lib" / "slam_toolbox" / "async_slam_toolbox_node",
                           paths["solver"] / "lib" / "libr2b4_ceres_solver_plugin.so",
                           paths["params"]):
            if not dependency.is_file():
                raise RuntimeError(f"SLAM dependency/configuration missing: {dependency}")
        if paths["socket"].exists():
            raise RuntimeError(f"SLAM input socket already exists; inspect it before starting: {paths['socket']}")
        paths["bridge"].unlink(missing_ok=True)
        paths["maps"].mkdir(exist_ok=True)
        with (paths["work"] / "supervisor.log").open("a") as log:
            child = subprocess.Popen([sys.executable, "-m", "tools.slam_toolbox", "__supervise", "--root", str(root)],
                                     cwd=root, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            state = _read_json(paths["state"])
            if state.get("pid") == child.pid and state.get("state") == "RUNNING":
                return status(root)
            if child.poll() is not None:
                detail = state.get("error", f"supervisor exit={child.returncode}")
                raise RuntimeError(f"SLAM startup failed: {detail}; logs: {paths['work']}")
            time.sleep(0.1)
        # A partially initialized own session must not survive a failed start.
        state = _read_json(paths["state"])
        if state.get("pid") == child.pid and _owned_supervisor(state, root):
            _stop_owned(state, root)
        else:
            child.terminate()
            child.wait(timeout=5)
        raise RuntimeError("SLAM startup deadline exceeded")


def _stop_owned(state: dict, root: Path) -> None:
    if not _owned_supervisor(state, root):
        return
    pid = state["pid"]
    os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        if not _owned_supervisor(state, root):
            return
        time.sleep(0.1)
    # Identity and dedicated process group are rechecked immediately before kill.
    if _owned_supervisor(state, root):
        os.killpg(pid, signal.SIGKILL)


def stop(root: Path) -> dict:
    paths = _paths(root)
    with _command_lock(paths):
        state = _read_json(paths["state"])
        if _owned_supervisor(state, root):
            _stop_owned(state, root)
        elif state.get("state") in {"STARTING", "RUNNING"}:
            raise RuntimeError("SLAM supervisor identity lost; refusing to signal an unverified process")
        return status(root)


def _terminate_with_parent(expected_parent: int) -> None:
    """The two direct ROS children cannot outlive a killed supervisor."""
    if ctypes.CDLL(None).prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
        raise OSError("Cannot set SLAM child parent-death signal")
    if os.getppid() != expected_parent:
        os.kill(os.getpid(), signal.SIGTERM)


def _supervise(root: Path) -> int:
    paths = _paths(root)
    requested = False

    def request_stop(_signum, _frame):
        nonlocal requested
        requested = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    state = {"pid": os.getpid(), "process_start": _process_start(os.getpid()), "root": str(root),
             "state": "STARTING", "started_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
             "socket": str(paths["socket"]), "children": {}}
    _write_json(paths["state"], state)
    children: list[subprocess.Popen] = []
    logs = []
    try:
        os.nice(10)
        allowed = os.sched_getaffinity(0)
        control = _read_json(root / "conf" / "vezerles.json").get("runtime_affinity", {}).get("control_cpus", [3])
        cpus = (allowed & {1, 2}) - set(control)
        if not cpus:
            cpus = allowed - set(control)
        if not cpus:
            raise RuntimeError("No CPU available outside the configured control CPU")
        # The sidecar has one compute thread and a few I/O threads. Bound its
        # total schedulable CPU capacity as well as its priority.
        # Keep SLAM on the LiDAR compute CPU; Nav2 uses the planner CPU. Both
        # stay outside the control island without contending on one core.
        preferred = set(_read_json(root / "conf" / "vezerles.json")
                        .get("runtime_affinity", {}).get("lidar_matcher_cpus", [2]))
        cpus = {min(cpus & preferred) if cpus & preferred else max(cpus)}
        os.sched_setaffinity(0, cpus)
        state["cpus"] = sorted(cpus)
        env = _ros_environment(root, paths["prefix"])
        from v3.config import ConfigResolver
        resolved = ConfigResolver.for_project(root).resolve()
        maximum_pose_age_ns = resolved.runtime.composition.live_control.control.admission.global_pose_max_age_ns
        commands = {
            "node": [str(paths["prefix"] / "lib" / "slam_toolbox" / "async_slam_toolbox_node"),
                     "--ros-args", "--params-file", str(paths["params"]), "-p", "use_sim_time:=false"],
            "bridge": [str(paths["prefix"] / "bin" / "python"), "-m", "v3.slam_toolbox_bridge",
                       "--socket", str(paths["socket"]), "--status-path", str(paths["bridge"]),
                       "--maximum-pose-age-ns", str(maximum_pose_age_ns),
                       "--output-dir", str(paths["work"])],
        }
        for name, command in commands.items():
            log = (paths["work"] / f"{name}.log").open("a")
            logs.append(log)
            child = subprocess.Popen(command, cwd=root, env=env, stdin=subprocess.DEVNULL,
                                     stdout=log, stderr=subprocess.STDOUT,
                                     preexec_fn=partial(_terminate_with_parent, os.getpid()))
            children.append(child)
            state["children"][name] = child.pid
        deadline = time.monotonic() + 10
        while not requested:
            failed = next((child for child in children if child.poll() is not None), None)
            if failed is not None:
                raise RuntimeError(f"SLAM child exited: pid={failed.pid} exit={failed.returncode}")
            if sum(_resident_bytes(child.pid) for child in children) > 768 * 1024 * 1024:
                raise RuntimeError("SLAM resident memory budget exceeded (768 MiB)")
            if state["state"] == "STARTING":
                if paths["socket"].exists():
                    state["state"] = "RUNNING"
                    _write_json(paths["state"], state)
                elif time.monotonic() >= deadline:
                    raise RuntimeError("SLAM bridge did not create its input socket")
            time.sleep(0.25)
        state["state"] = "STOPPED"
    except Exception as exc:
        state.update(state="FAILED", error=str(exc))
    finally:
        for child in children:
            if child.poll() is None:
                child.terminate()
        deadline = time.monotonic() + 5
        for child in children:
            try:
                child.wait(timeout=max(0.01, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=1)
        for log in logs:
            log.close()
        try:
            if stat.S_ISSOCK(paths["socket"].lstat().st_mode):
                paths["socket"].unlink()
        except FileNotFoundError:
            pass
        state["stopped_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
        _write_json(paths["state"], state)
    return 1 if state["state"] == "FAILED" else 0


def _write_map(grid, output: Path) -> None:
    width, height = int(grid.info.width), int(grid.info.height)
    if width <= 0 or height <= 0 or width * height != len(grid.data):
        raise RuntimeError("SLAM returned an empty or malformed occupancy grid")
    # ROS occupancy starts at the lower left; PGM rows start at the top left.
    rows = bytearray()
    for y in range(height - 1, -1, -1):
        for x in range(width):
            value = grid.data[y * width + x]
            rows.append(205 if value < 0 or 25 < value < 65 else (0 if value >= 65 else 254))
    output.with_suffix(".pgm").write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + rows)
    origin = grid.info.origin
    q = origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    output.with_suffix(".yaml").write_text(
        f"image: {output.name}.pgm\nmode: trinary\nresolution: {grid.info.resolution}\n"
        f"origin: [{origin.position.x}, {origin.position.y}, {yaw}]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n", encoding="utf-8")


def _ros_save(output: Path) -> int:
    # Imports only run in the isolated ROS Python subprocess.
    import rclpy
    from nav_msgs.srv import GetMap
    from slam_toolbox.srv import SerializePoseGraph

    rclpy.init()
    node = rclpy.create_node("r2b4_slam_save")
    try:
        def call(service_type, name, request):
            client = node.create_client(service_type, name)
            if not client.wait_for_service(timeout_sec=5):
                raise RuntimeError(f"SLAM service unavailable: {name}")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=15)
            if not future.done() or future.exception() is not None:
                raise RuntimeError(f"SLAM service failed/deadline exceeded: {name}")
            return future.result()

        grid = call(GetMap, "/slam_toolbox/dynamic_map", GetMap.Request()).map
        if not grid.data:
            raise RuntimeError("SLAM has not produced a map")
        request = SerializePoseGraph.Request()
        request.filename = str(output)
        result = call(SerializePoseGraph, "/slam_toolbox/serialize_map", request)
        if result.result != 0:
            raise RuntimeError(f"SLAM pose graph serialization failed: result={result.result}")
        _write_map(grid, output)
        _write_json(output.with_suffix(".json"), {
            "frame_id": grid.header.frame_id,
            "stamp_sec": grid.header.stamp.sec, "stamp_nanosec": grid.header.stamp.nanosec,
            "width": grid.info.width, "height": grid.info.height,
            "resolution_m": grid.info.resolution, "motor_authority": False})
        print(json.dumps({"state": "SAVED", "output": str(output)}))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def save(root: Path, name: str | None) -> dict:
    paths = _paths(root)
    name = name or dt.datetime.now(dt.timezone.utc).strftime("map_%Y%m%dT%H%M%SZ")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,79}", name):
        raise ValueError("--output must be a map basename containing letters, digits, underscore or hyphen")
    with _command_lock(paths):
        if not status(root)["running"]:
            raise RuntimeError("SLAM is not running")
        output = paths["maps"] / name
        if any(paths["maps"].glob(name + ".*")):
            raise ValueError(f"Map output already exists: {output}")
        env = _ros_environment(root, paths["prefix"])
        result = subprocess.run([str(paths["prefix"] / "bin" / "python"), "-m", "tools.slam_toolbox",
                                 "__save", "--root", str(root), "--output", str(output)],
                                cwd=root, env=env, capture_output=True, text=True, timeout=45)
        if result.returncode:
            raise RuntimeError(f"SLAM save failed: {result.stderr.strip() or result.stdout.strip()}")
        return {"state": "SAVED", "output": str(output),
                "files": sorted(str(path) for path in paths["maps"].glob(name + ".*"))}


def parser(*, private: bool = False) -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(prog="r slam", description=__doc__)
    choices = ("start", "status", "stop", "save")
    result.add_argument("operation", choices=choices + (("__supervise", "__save") if private else ()))
    result.add_argument("--output", help="save map basename under runtime/slam_toolbox/maps")
    result.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1], help=argparse.SUPPRESS)
    return result


def main(argv=None, *, project_root=None) -> int:
    raw = list(sys.argv[1:] if argv is None else argv)
    args = parser(private=bool(raw and raw[0] in {"__supervise", "__save"})).parse_args(raw)
    root = Path(project_root or args.root).resolve()
    try:
        if args.operation == "__supervise":
            return _supervise(root)
        if args.operation == "__save":
            return _ros_save(Path(args.output))
        if args.output is not None and args.operation != "save":
            raise ValueError("--output is only valid with save")
        value = {"start": start, "status": status, "stop": stop}.get(args.operation)
        print(json.dumps(value(root) if value else save(root, args.output), sort_keys=True, indent=2))
        return 0
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
