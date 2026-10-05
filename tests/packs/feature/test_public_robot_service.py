"""Real host IPC evidence with temporary state and no hardware/motion requests."""

from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import time
from contextlib import contextmanager
from pathlib import Path

import pytest

from r2b4_orchestration.robot_runtime import MAX_REQUEST_BYTES, PublicRobotClient


PROJECT_ROOT = Path(__file__).resolve().parents[3]


@contextmanager
def owner_service(project: Path, socket_path: Path):
    """Start/terminate only this test's host owner, never an operator process."""

    environment = dict(os.environ)
    environment["PYTHONPATH"] = os.pathsep.join(filter(None, (
        str(PROJECT_ROOT), environment.get("PYTHONPATH"))))
    output_path = project.parent / "public-owner-output.txt"
    with output_path.open("w", encoding="utf-8") as output:
        process = subprocess.Popen(
            [sys.executable, "-m", "r2b4_orchestration.robot_runtime", "--root", str(project),
             "--socket", str(socket_path)],
            cwd=PROJECT_ROOT, env=environment, stdin=subprocess.DEVNULL,
            stdout=output, stderr=subprocess.STDOUT,
        )
        client = PublicRobotClient(project, socket_path=socket_path, timeout_s=2)
        try:
            deadline = time.monotonic() + 8
            while True:
                if process.poll() is not None:
                    pytest.fail("public owner exited during startup:\n" + output_path.read_text())
                ready = client.request("read", resource="behavior.state", launch=False)
                if ready is not None:
                    break
                if time.monotonic() >= deadline:
                    pytest.fail("public owner socket did not become ready:\n" + output_path.read_text())
                time.sleep(0.025)
            yield process, client
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)


def temporary_project(tmp_path: Path) -> tuple[Path, Path]:
    project = tmp_path / "project"
    project.mkdir()
    # ConfigResolver reads these static files; the host service does not change
    # them. Runtime state belongs exclusively to this temporary project.
    (project / "conf").symlink_to(PROJECT_ROOT / "conf", target_is_directory=True)
    return project, tmp_path / "owner.sock"


def matching_fact(snapshot: dict, entity_id: str, attribute: str) -> dict:
    return next(fact for fact in snapshot["facts"]
                if fact["entity_id"] == entity_id and fact["attribute"] == attribute)


def test_real_owner_shares_semantic_world_and_restores_same_boot_without_v3(tmp_path):
    project, path = temporary_project(tmp_path)
    measured = time.monotonic_ns()
    with owner_service(project, path) as (process, first):
        second = PublicRobotClient(project, socket_path=path, timeout_s=2)
        assert first is not second
        assert first.request("read", resource="behavior.state", launch=False)["lifecycle"] == "IDLE"
        state = second.request("read", resource="robot.state", launch=False)
        assert state["active_behavior"]["lifecycle"] == "IDLE"
        assert state["active_mission"]["state"] == "UNKNOWN"
        assert state["health"]["public_runtime"] == "AVAILABLE"

        event = first.request("execute", action="world.observe", launch=False, parameters={
            "entity_id": "object:test-cup", "attribute": "identity", "domain": "object_identity",
            "value": {"label": "cup", "color": "blue"}, "measurement_time_ns": measured,
            "confidence": 0.95, "source": "host-ipc-test", "sequence": 7, "revision": "vision-test",
            "lineage": {"evidence_id": "temporary-semantic-observation"},
        })
        assert event["accepted"]
        shared = second.request("read", resource="world.snapshot", launch=False)
        fact = matching_fact(shared, "object:test-cup", "identity")
        assert fact["value"] == {"label": "cup", "color": "blue"}
        assert fact["state"] == "KNOWN"
        assert fact["measurement_time_ns"] == measured
        assert fact["observation_time_ns"] >= measured
        assert fact["sequence"] == 7
        assert fact["revision"] == "vision-test"
        assert fact["lineage"] == {"evidence_id": "temporary-semantic-observation"}
        assert any(item["event_id"] == event["event_id"]
                   for item in second.request("read", resource="world.history", launch=False))

        saved = json.loads((project / "runtime" / "public_world" / "state.json").read_text())
        assert any(item["observation"]["entity_id"] == "object:test-cup" for item in saved["facts"])
        assert not (project / "runtime" / "v3_command.json").exists()
        assert not (project / "runtime" / ".r2b4_runtime_pid").exists()
        assert not (project / "runtime" / "captures").exists()
        first_pid = process.pid

    # The next process loads only the fixture's durable world evidence. We never
    # ask the production OperatorController to stop or launch any runtime.
    with owner_service(project, path) as (process, restarted):
        assert process.pid != first_pid
        fact = matching_fact(restarted.request("read", resource="world.snapshot", launch=False),
                             "object:test-cup", "identity")
        assert fact["state"] == "KNOWN"
        assert fact["measurement_time_ns"] == measured
        assert fact["sequence"] == 7
        assert fact["clock_epoch"] == event["observation"]["clock_epoch"]
        assert restarted.request("read", resource="behavior.state", launch=False)["lifecycle"] == "IDLE"


def test_real_owner_rejects_unknown_and_oversized_requests_and_remains_available(tmp_path):
    project, path = temporary_project(tmp_path)
    with owner_service(project, path) as (process, client):
        with pytest.raises(RuntimeError, match="unknown public robot operation"):
            client.request("unknown", launch=False)
        with pytest.raises(ValueError, match="exceeded its bound"):
            client.request("unknown", launch=False, payload="x" * MAX_REQUEST_BYTES)

        # Exercise the actual server's ingress bound, independently of the
        # client's preflight. Closing an invalid connection must not kill owner.
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
            conn.settimeout(2)
            conn.connect(str(path))
            conn.sendall(b"x" * (MAX_REQUEST_BYTES + 1) + b"\n")
            try:
                assert conn.recv(1) == b""
            except ConnectionResetError:
                pass
        assert process.poll() is None
        assert client.request("read", resource="behavior.state", launch=False)["lifecycle"] == "IDLE"
        assert not (project / "runtime" / "v3_command.json").exists()
