"""External Nav2 lifecycle has one owner and bounded read-only readiness."""
from concurrent.futures import Future
import json
from pathlib import Path
import socket
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

from tools.nav2_planner import _commands, _navigation_environment
from v3.adapters.nav2_lifecycle import Nav2LifecycleMonitor


class Trigger:
    class Request:
        pass


class Client:
    def __init__(self):
        self.ready = True
        self.calls = []
        self.removed = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        assert isinstance(request, Trigger.Request)
        future = Future()
        self.calls.append(future)
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


def monitor():
    client = Client()

    def create_client(service, name):
        assert service is Trigger
        assert name == "/lifecycle_manager_navigation/is_active"
        return client

    return Nav2LifecycleMonitor(SimpleNamespace(create_client=create_client),
                                1_000_000_000, service_type=Trigger), client


def test_initial_inactive_waits_for_manager_then_inactive_fails_closed():
    watch, client = monitor()
    assert not watch.poll(1_000_000_000)
    client.calls[-1].set_result(SimpleNamespace(success=False))
    assert not watch.poll(1_100_000_000)
    assert not watch.poll(1_500_000_000)
    client.calls[-1].set_result(SimpleNamespace(success=True))
    assert watch.poll(1_600_000_000)
    assert watch.poll(2_000_000_000)
    client.calls[-1].set_result(SimpleNamespace(success=False))
    with pytest.raises(RuntimeError, match="LIFECYCLE_INACTIVE"):
        watch.poll(2_100_000_000)


def test_unanswered_query_is_bounded_cancelled_and_old_positive_cannot_renew():
    watch, client = monitor()
    watch.poll(1_000_000_000)
    client.calls[-1].set_result(SimpleNamespace(success=True))
    assert watch.poll(1_100_000_000)
    for now in (1_200_000_000, 1_300_000_000, 1_400_000_000):
        assert watch.poll(now)
    assert len(client.calls) == 1
    assert watch.poll(1_500_000_000)
    old = client.calls[-1]
    # Even a completed answer is discarded after its own request deadline.
    old.set_result(SimpleNamespace(success=True))
    assert watch.poll(2_500_000_000)
    assert client.removed == [old]
    assert len(client.calls) == 3
    with pytest.raises(RuntimeError, match="LIFECYCLE_STATUS_STALE"):
        watch.poll(2_600_000_000)


def test_manager_missing_or_query_exception_never_enables_bridge():
    watch, client = monitor()
    client.ready = False
    assert not watch.poll(1_000_000_000)
    with pytest.raises(RuntimeError, match="LIFECYCLE_STARTUP_TIMEOUT"):
        watch.poll(26_000_000_000)
    assert not client.calls
    watch, client = monitor()
    watch.poll(1_000_000_000)
    client.calls[-1].set_exception(ValueError("dead service"))
    with pytest.raises(RuntimeError, match="LIFECYCLE_QUERY_FAILED"):
        watch.poll(1_100_000_000)


def test_monitor_reports_timeout_and_loop_gap_without_extending_readiness():
    watch, client = monitor()
    watch.poll(1_000_000_000)
    client.calls[-1].set_result(SimpleNamespace(success=True))
    assert watch.poll(1_100_000_000)
    assert watch.poll(1_500_000_000)
    assert watch.poll(2_500_000_000)
    evidence = watch.snapshot(2_500_000_000)
    assert evidence["query_timeouts"] == 1
    assert evidence["max_poll_gap_ns"] == 1_000_000_000
    assert evidence["last_response_checked_ns"] == 1_100_000_000
    assert evidence["last_response_age_ns"] == 1_400_000_000
    assert client.removed[-1].cancelled()
    with pytest.raises(RuntimeError, match="LIFECYCLE_STATUS_STALE"):
        watch.poll(2_600_000_000)


def test_fastdds_service_wait_is_bounded_only_in_navigation_environment(monkeypatch):
    import tools.nav2_planner as host
    import xml.etree.ElementTree as ET
    root = Path(__file__).resolve().parents[3]
    monkeypatch.setattr(host, "_ros_environment", lambda *_: {"UNCHANGED": "kept"})
    env = _navigation_environment({key: root / key for key in ("prefix", "overlay", "frontier")}, root)
    assert env["UNCHANGED"] == "kept"
    profile = Path(env["FASTRTPS_DEFAULT_PROFILES_FILE"])
    ns = {"dds": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    writer = ET.parse(profile).find("dds:data_writer[@profile_name='service']", ns)
    reliability = writer.find("dds:qos/dds:reliability", ns)
    assert reliability.findtext("dds:kind", namespaces=ns) == "RELIABLE"
    blocking = reliability.find("dds:max_blocking_time", ns)
    seconds = int(blocking.findtext("dds:sec", namespaces=ns))
    nanos = int(blocking.findtext("dds:nanosec", namespaces=ns))
    assert 0 < seconds*1_000_000_000+nanos <= Nav2LifecycleMonitor.POLL_NS/10
    assert "RMW_FASTRTPS_USE_QOS_FROM_XML" not in env


@pytest.mark.parametrize("full", [False, True])
def test_supervisor_spawns_one_manager_with_exact_managed_servers(full):
    paths = {key: Path("/isolated") / key for key in
             ("prefix", "params", "geometry", "socket", "status", "bt", "frontier", "frontier_params")}
    geometry = {"body_bounds_m": [-.10, .26, -.18, .18], "lidar_xy_m": [-.005, 0.]}
    commands = _commands(paths, full=full, radius=.3, robot_geometry=geometry)
    managers = [command for name, command in commands if name == "lifecycle_manager"]
    assert len(managers) == 1
    manager = managers[0]
    nodes = [name for name, _ in commands if name not in ("bridge", "lifecycle_manager")]
    assert "node_names:=[" + ",".join(nodes) + "]" in manager
    assert "autostart:=true" in manager
    assert "attempt_respawn_reconnection:=false" in manager
    assert "bond_timeout:=2.0" in manager
    assert commands[-1][0] == "bridge"
    if full:
        assert "behavior_server" in nodes
        for name, command in commands:
            if name in ("controller_server", "behavior_server"):
                assert "cmd_vel:=/r2b4/nav2/cmd_vel" in command
        bridge = commands[-1][1]
        assert json.loads(bridge[bridge.index("--robot-geometry")+1]) == geometry


def test_status_readiness_requires_own_bridge_and_fresh_manager_response(monkeypatch, tmp_path):
    import tools.nav2_planner as host
    state = {"state": "RUNNING", "pid": 10, "bridge_pid": 11, "root": str(tmp_path),
             "children": [11], "child_process_starts": {"11": "bridge-start"}}
    bridge = {"state": "RUNNING", "pid": 11, "observed_monotonic_ns": 9_900_000_000,
              "lifecycle_status": {"active": True, "last_response_checked_ns": 9_000_000_000}}
    paths = {"state": tmp_path/"supervisor.json", "status": tmp_path/"bridge.json"}
    monkeypatch.setattr(host, "_paths", lambda _: paths)
    monkeypatch.setattr(host, "_read_json", lambda path: state if path == paths["state"] else bridge)
    monkeypatch.setattr(host, "_owned", lambda *_: True)
    monkeypatch.setattr(host, "_process_start", lambda pid: "bridge-start" if pid == 11 else None)
    monkeypatch.setattr(host.time, "monotonic_ns", lambda: 10_000_000_000)
    current = host.status(tmp_path)
    assert current["ready"]
    assert current["running_children"] == [11]
    snapshot = current["bridge_snapshot_status"]
    assert snapshot["kind"] == "LIVE" and snapshot["fresh"] and snapshot["identity_verified"]
    assert snapshot["age_ns"] == 100_000_000
    assert snapshot["lifecycle_response_age_ns"] == 1_000_000_000
    assert snapshot["lifecycle_response_fresh"]
    bridge["pid"] = 12
    assert not host.status(tmp_path)["ready"]
    bridge["pid"] = 11
    bridge["lifecycle_status"]["last_response_checked_ns"] = 8_000_000_000
    assert not host.status(tmp_path)["ready"]
    bridge["lifecycle_status"]["last_response_checked_ns"] = 11_000_000_000
    assert not host.status(tmp_path)["ready"]


def test_stopped_status_marks_raw_active_snapshot_historical_and_checks_children(monkeypatch, tmp_path):
    import tools.nav2_planner as host
    state = {"state": "STOPPED", "pid": 10, "bridge_pid": 16, "root": str(tmp_path),
             "children": list(range(11, 17)),
             "child_process_starts": {str(pid): f"start-{pid}" for pid in range(11, 17)}}
    bridge = {"state": "STOPPED", "pid": 16, "observed_monotonic_ns": 9_900_000_000,
              "lifecycle_status": {"active": True, "last_response_checked_ns": 9_500_000_000,
                                   "last_response_age_ns": 100_000_000},
              "nav2_observation": {"sensor_age_ns": {"scan": 1_400_000_000}}}
    original = json.dumps(bridge, sort_keys=True)
    paths = {"state": tmp_path/"supervisor.json", "status": tmp_path/"bridge.json"}
    monkeypatch.setattr(host, "_paths", lambda _: paths)
    monkeypatch.setattr(host, "_read_json", lambda path: state if path == paths["state"] else bridge)
    monkeypatch.setattr(host, "_owned", lambda *_: False)
    monkeypatch.setattr(host, "_process_start", lambda _: None)
    monkeypatch.setattr(host.time, "monotonic_ns", lambda: 100_000_000_000)
    current = host.status(tmp_path)
    assert current["state"] == "STOPPED" and not current["running"] and not current["ready"]
    assert current["children"] == list(range(11, 17))
    assert current["running_children"] == []
    assert all(value["identity"] == "EXITED" for value in current["child_process_status"].values())
    snapshot = current["bridge_snapshot_status"]
    assert snapshot["kind"] == "HISTORICAL" and not snapshot["fresh"]
    assert not snapshot["identity_verified"] and not snapshot["lifecycle_response_fresh"]
    assert snapshot["age_ns"] == 90_100_000_000
    assert snapshot["lifecycle_response_age_ns"] == 90_500_000_000
    assert json.dumps(current["bridge"], sort_keys=True) == original


@pytest.mark.parametrize("observed,identity", [(None, "EXITED"), ("new-start", "PID_REUSED")])
def test_fresh_reply_from_exited_or_reused_bridge_cannot_report_live_ready(
        monkeypatch, tmp_path, observed, identity):
    import tools.nav2_planner as host
    state = {"state": "RUNNING", "pid": 10, "bridge_pid": 11, "root": str(tmp_path),
             "children": [11], "child_process_starts": {"11": "old-start"}}
    bridge = {"state": "RUNNING", "pid": 11, "observed_monotonic_ns": 9_900_000_000,
              "lifecycle_status": {"active": True, "last_response_checked_ns": 9_900_000_000}}
    paths = {"state": tmp_path/"supervisor.json", "status": tmp_path/"bridge.json"}
    monkeypatch.setattr(host, "_paths", lambda _: paths)
    monkeypatch.setattr(host, "_read_json", lambda path: state if path == paths["state"] else bridge)
    monkeypatch.setattr(host, "_owned", lambda *_: True)
    monkeypatch.setattr(host, "_process_start", lambda _: observed)
    monkeypatch.setattr(host.time, "monotonic_ns", lambda: 10_000_000_000)
    current = host.status(tmp_path)
    assert current["child_process_status"]["11"]["identity"] == identity
    assert not current["ready"] and current["running_children"] == []
    assert current["bridge_snapshot_status"]["kind"] == "HISTORICAL"
    assert not current["bridge_snapshot_status"]["identity_verified"]
    assert current["bridge"]["lifecycle_status"]["active"] is True


def test_existing_start_waits_for_readiness_and_does_not_launch_again(monkeypatch, tmp_path):
    import tools.nav2_planner as host
    paths = {"work": tmp_path, "state": tmp_path/"supervisor.json"}
    previous = {"state": "STARTING", "pid": 10, "ros_domain_id": 74}
    states = iter([
        {**previous, "ready": False, "running": True, "bridge": {}},
        {**previous, "state": "RUNNING", "ready": False, "running": True, "bridge": {}},
        {**previous, "state": "RUNNING", "ready": True, "running": True, "bridge": {}},
    ])
    calls = []
    monkeypatch.setattr(host, "_paths", lambda _: paths)
    monkeypatch.setattr(host, "_resolved_control", lambda _: SimpleNamespace(
        navigation=SimpleNamespace(full_nav2_enabled=True)))
    monkeypatch.setattr(host, "_read_json", lambda _: previous)
    monkeypatch.setattr(host, "_owned", lambda *_: True)
    monkeypatch.setattr(host, "status", lambda _: next(states))
    monkeypatch.setattr(host.time, "sleep", lambda value: calls.append(value))
    monkeypatch.setattr(host.subprocess, "Popen", lambda *_a, **_kw: pytest.fail("duplicate supervisor"))
    assert host.start(tmp_path)["ready"]
    assert len(calls) == 2
    with pytest.raises(RuntimeError, match="different ROS domain"):
        host.start(tmp_path, ros_domain_id=75)


def test_existing_failed_or_permanently_stale_session_cannot_pass_start(monkeypatch, tmp_path):
    import tools.nav2_planner as host
    current = {"pid": 10, "state": "FAILED", "error": "NAV2_CHILD_EXITED",
               "running": True, "ready": False, "bridge": {}}
    monkeypatch.setattr(host, "status", lambda _: current)
    with pytest.raises(RuntimeError, match="NAV2_CHILD_EXITED"):
        host._wait_ready(tmp_path, 10)
    current["state"] = "RUNNING"
    clock = [0.]
    monkeypatch.setattr(host.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(host.time, "sleep", lambda value: clock.__setitem__(0, clock[0]+value))
    with pytest.raises(RuntimeError, match="readiness timeout"):
        host._wait_ready(tmp_path, 10)
    assert 30 <= clock[0] < 31


def test_lost_supervisor_cannot_start_over_its_live_children(monkeypatch, tmp_path):
    import tools.nav2_planner as host
    paths = {"work": tmp_path, "state": tmp_path/"supervisor.json"}
    previous = {"state": "RUNNING", "pid": 10, "root": str(tmp_path),
                "child_process_starts": {"11": "still-alive", "12": None}}
    monkeypatch.setattr(host, "_paths", lambda _: paths)
    monkeypatch.setattr(host, "_resolved_control", lambda _: SimpleNamespace(
        navigation=SimpleNamespace(full_nav2_enabled=True)))
    monkeypatch.setattr(host, "_read_json", lambda _: previous)
    monkeypatch.setattr(host, "_owned", lambda *_: False)
    monkeypatch.setattr(host, "_process_start", lambda pid: "still-alive" if pid == 11 else None)
    with pytest.raises(RuntimeError, match="live children"):
        host.start(tmp_path)


def test_stale_socket_cleanup_requires_exact_dead_owned_session(monkeypatch, tmp_path):
    import tools.nav2_planner as host
    path = tmp_path/"nav2.sock"
    endpoint = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    endpoint.bind(str(path))
    endpoint.close()
    paths = {"socket": path}
    state = {"root": str(tmp_path), "pid": 10,
             "child_process_starts": {"11": "old-child"},
             "socket_identity": host._socket_identity(path)}
    starts = {11: "old-child"}
    monkeypatch.setattr(host, "_process_start", lambda pid: starts.get(pid))
    assert not host._remove_owned_socket(paths, state, tmp_path)
    assert path.exists()
    starts.clear()
    identity = state["socket_identity"]
    state["socket_identity"] = [identity[0], identity[1]+1]
    assert not host._remove_owned_socket(paths, state, tmp_path)
    state["socket_identity"] = identity
    state["root"] = str(tmp_path/"other")
    assert not host._remove_owned_socket(paths, state, tmp_path)
    state["root"] = str(tmp_path)
    assert host._remove_owned_socket(paths, state, tmp_path)
    assert not path.exists()


def test_all_hung_children_share_one_shutdown_deadline_and_are_reaped():
    import tools.nav2_planner as host
    script = ("import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); "
              "print('ready',flush=True); time.sleep(30)")
    children = [subprocess.Popen([sys.executable, "-c", script],
                 stdout=subprocess.PIPE, text=True) for _ in range(6)]
    try:
        for child in children:
            assert child.stdout.readline().strip() == "ready"
        started = time.monotonic()
        assert host._stop_children(children) == []
        assert time.monotonic()-started < 4
        assert all(child.returncode is not None for child in children)
    finally:
        for child in children:
            if child.poll() is None:
                child.kill()
            child.wait(timeout=1)
            child.stdout.close()


def test_forced_stop_waits_for_verified_group_and_cleans_its_endpoint(monkeypatch, tmp_path):
    import tools.nav2_planner as host
    paths = {"work": tmp_path, "state": tmp_path/"supervisor.json",
             "status": tmp_path/"bridge.json", "socket": tmp_path/"nav2.sock"}
    script = ("import signal,socket,sys,time; "
              "signal.signal(signal.SIGTERM,signal.SIG_IGN); "
              "s=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM); s.bind(sys.argv[1]); "
              "print('ready',flush=True); time.sleep(30)")
    # Only the command-line identity and isolated process group are emulated;
    # no ROS process, runtime or motor device is started.
    child = subprocess.Popen([sys.executable, "-c", script, str(paths["socket"]),
                              "tools.nav2_planner", "__supervise"],
                             start_new_session=True, stdout=subprocess.PIPE, text=True)
    try:
        assert child.stdout.readline().strip() == "ready"
        started = host._process_start(child.pid)
        state = {"state": "RUNNING", "pid": child.pid, "root": str(tmp_path),
                 "process_start": started, "child_process_starts": {str(child.pid): started},
                 "socket_identity": host._socket_identity(paths["socket"])}
        host._write_json(paths["state"], state)
        monkeypatch.setattr(host, "_paths", lambda _: paths)
        assert host._owned(state, tmp_path)
        before = time.monotonic()
        stopped = host.stop(tmp_path)
        assert time.monotonic()-before < 7
        assert not stopped["running"] and not stopped["ready"]
        assert stopped["state"] == "STOPPED" and stopped["forced_shutdown"] is True
        assert not paths["socket"].exists()
        child.wait(timeout=1)
    finally:
        if child.poll() is None:
            child.kill()
        child.wait(timeout=1)
        child.stdout.close()
