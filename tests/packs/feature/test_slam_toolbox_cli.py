"""Map export geometry and host-process ownership do not require live hardware."""

import math
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

from tools import slam_toolbox


def test_map_export_keeps_ros_orientation_and_unknown_cells(tmp_path):
    origin = SimpleNamespace(position=SimpleNamespace(x=-1.0, y=2.0),
                             orientation=SimpleNamespace(x=0.0, y=0.0,
                                                         z=math.sin(math.pi / 4), w=math.cos(math.pi / 4)))
    grid = SimpleNamespace(info=SimpleNamespace(width=2, height=2, resolution=0.05, origin=origin),
                           data=[-1, 0, 100, 50])
    target = tmp_path / "room"
    slam_toolbox._write_map(grid, target)
    assert target.with_suffix(".pgm").read_bytes() == b"P5\n2 2\n255\n" + bytes([0, 205, 205, 254])
    yaml = target.with_suffix(".yaml").read_text()
    assert "image: room.pgm" in yaml
    origin_line = next(line for line in yaml.splitlines() if line.startswith("origin: "))
    values = json.loads(origin_line.removeprefix("origin: "))
    assert values[:2] == [-1.0, 2.0]
    assert math.isclose(values[2], math.pi / 2)
    # A map-server reload must keep gray 205 unknown, never reinterpret it free.
    scalars = dict(line.split(": ", 1) for line in yaml.splitlines())
    assert float(scalars["free_thresh"]) < 1 - 205 / 255 < float(scalars["occupied_thresh"])


def test_stop_rejects_reused_or_unrelated_pid(tmp_path):
    child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(10)"], start_new_session=True)
    try:
        state = {"root": str(tmp_path), "pid": child.pid,
                 "process_start": slam_toolbox._process_start(child.pid), "state": "RUNNING"}
        assert not slam_toolbox._owned_supervisor(state, tmp_path)
        slam_toolbox._stop_owned(state, tmp_path)
        assert child.poll() is None
        state["process_start"] = "wrong-generation"
        assert not slam_toolbox._owned_supervisor(state, tmp_path)
    finally:
        child.terminate()
        child.wait(timeout=2)


def test_launcher_slam_route_uses_host_cli_without_robot_commands(monkeypatch, tmp_path):
    from v3 import launcher_cli, runtime_performance

    monkeypatch.setattr(launcher_cli, "project_root", lambda: tmp_path)
    monkeypatch.setattr(runtime_performance, "apply_host_affinity", lambda *args: None)
    calls = []
    monkeypatch.setattr(slam_toolbox, "main", lambda args, *, project_root: calls.append((args, project_root)) or 0)
    assert launcher_cli.main(["slam", "status"]) == 0
    assert calls == [(["status"], Path(tmp_path))]


def test_ros_supervisors_follow_disabled_and_enabled_affinity_policy(monkeypatch, tmp_path):
    from v3 import runtime_performance
    from tools import nav2_planner
    cpus, calls = {0, 1, 2, 3}, []
    monkeypatch.setattr(slam_toolbox.os, "sched_getaffinity", lambda _: cpus)
    policy = SimpleNamespace(enabled=False, strict=True, lidar_matcher_cpus=(1, 2), planner_cpus=(1, 2))
    monkeypatch.setattr(runtime_performance, "load_runtime_affinity_config", lambda _: policy)
    monkeypatch.setattr(runtime_performance, "apply_current_affinity",
        lambda requested, **kwargs: (calls.append((requested, kwargs)), cpus.intersection_update(requested)))
    for helper in (slam_toolbox._apply_ros_affinity, nav2_planner._apply_ros_affinity):
        for role in ("lidar_matcher", "planner"):
            assert helper(tmp_path, role) == [0, 1, 2, 3]
    assert calls == []
    policy.enabled = True
    assert slam_toolbox._apply_ros_affinity(tmp_path, "lidar_matcher") == [1, 2]
    assert nav2_planner._apply_ros_affinity(tmp_path, "planner") == [1, 2]
    assert all(options["strict"] is True for _, options in calls)
