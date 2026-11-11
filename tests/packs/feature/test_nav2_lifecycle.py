"""External Nav2 lifecycle has one owner and bounded read-only readiness."""
from concurrent.futures import Future
import json
from pathlib import Path
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
        bridge = commands[-1][1]
        assert json.loads(bridge[bridge.index("--robot-geometry")+1]) == geometry
