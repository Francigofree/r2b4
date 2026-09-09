import json

import pytest

from v3 import control_cli
from v3.adapters.resident_command import (
    AtomicResidentCommandGateway,
    ResidentCommandClient,
    ResidentCommandMailboxConfig,
)
from v3.contracts import CommandMode, TickContext


def _status(path, *, ready=True):
    path.write_text(
        json.dumps(
            {
                "schema": control_cli.RESIDENT_PROCESS_STATUS_SCHEMA,
                "state": "RUNNING",
                "ready_for_active": ready,
            }
        ),
        encoding="utf-8",
    )
    path.chmod(0o600)


def test_cli_status_stop_teleop_and_explore_use_the_resident_mailbox(
    tmp_path,
    monkeypatch,
    capsys,
):
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    status_path = runtime / "v3_status.json"
    command_path = runtime / "v3_command.json"
    _status(status_path)
    monkeypatch.setattr(control_cli, "PROJECT_ROOT", tmp_path)

    assert control_cli.main(["status"]) == 0
    assert json.loads(capsys.readouterr().out)["state"] == "RUNNING"

    assert control_cli.main(["stop", "--command-id", "cli-stop"]) == 0
    stop_payload = json.loads(command_path.read_text(encoding="utf-8"))
    gateway = AtomicResidentCommandGateway(
        ResidentCommandMailboxConfig(path=command_path),
        monotonic_ns=lambda: stop_payload["issued_monotonic_ns"],
    )
    assert gateway.snapshot(TickContext(0, 1)).mode is CommandMode.STOP

    revisions = []

    def publish_once(_client, publish, *, command_id, **_kwargs):
        revisions.append(publish(command_id))
        return 0

    monkeypatch.setattr(control_cli, "_run_active", publish_once)
    assert control_cli.main(
        [
            "teleop",
            "--command-id",
            "cli-teleop",
            "--v-mps",
            "0.1",
            "--omega-rad-s",
            "0.2",
        ]
    ) == 0
    teleop = json.loads(command_path.read_text(encoding="utf-8"))
    assert teleop["mode"] == "TELEOP"
    assert teleop["command_id"] == "cli-teleop"

    assert control_cli.main(["explore", "--command-id", "room-cruise"]) == 0
    explore = json.loads(command_path.read_text(encoding="utf-8"))
    assert explore["mode"] == "EXPLORE"
    assert explore["command_id"] == "room-cruise"
    assert explore["max_v_mps"] == 0.3
    assert explore["max_omega_rad_s"] == 0.6
    assert revisions == [2, 3]


def test_active_heartbeat_keeps_identity_and_ctrl_c_publishes_stop(tmp_path):
    path = tmp_path / "command.json"
    config = ResidentCommandMailboxConfig(path=path)
    times = iter((1_000_000_000, 1_100_000_000, 1_200_000_000))
    client = ResidentCommandClient(config, monotonic_ns=times.__next__)
    heartbeats = []

    def publish(command_id):
        return client.publish_explore(
            command_id,
            max_v_mps=0.3,
            max_omega_rad_s=0.6,
            ttl_ns=200_000_000,
        )

    def interrupt_after_two(_seconds):
        heartbeats.append(json.loads(path.read_text(encoding="utf-8")))
        if len(heartbeats) == 2:
            raise KeyboardInterrupt

    assert control_cli._run_active(
        client,
        publish,
        command_id="room-cruise-session",
        ttl_ns=200_000_000,
        heartbeat_ns=100_000_000,
        sleep=interrupt_after_two,
    ) == 130

    stopped = json.loads(path.read_text(encoding="utf-8"))
    assert [item["revision"] for item in heartbeats] == [1, 2]
    assert {item["command_id"] for item in heartbeats} == {"room-cruise-session"}
    assert all(item["mode"] == "EXPLORE" for item in heartbeats)
    assert stopped["revision"] == 3
    assert stopped["mode"] == "STOP"


def test_active_heartbeat_period_includes_publish_latency():
    class Clock:
        now_ns = 0

        def monotonic_ns(self):
            return self.now_ns

        def sleep(self, seconds):
            self.now_ns += round(seconds * 1e9)
            if len(starts) == 2:
                raise KeyboardInterrupt

    class Client:
        stopped = False

        def publish_stop(self, _command_id, *, ttl_ns):
            assert ttl_ns == 200_000_000
            self.stopped = True
            return 3

    clock = Clock()
    client = Client()
    starts = []

    def publish(_command_id):
        starts.append(clock.now_ns)
        clock.now_ns += 80_000_000
        return len(starts)

    assert control_cli._run_active(
        client,
        publish,
        command_id="timed-session",
        ttl_ns=200_000_000,
        heartbeat_ns=100_000_000,
        sleep=clock.sleep,
        monotonic_ns=clock.monotonic_ns,
    ) == 130

    assert starts == [0, 100_000_000]
    assert client.stopped


def test_active_cli_requires_fresh_resident_preflight(tmp_path, monkeypatch, capsys):
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    _status(runtime / "v3_status.json", ready=False)
    monkeypatch.setattr(control_cli, "PROJECT_ROOT", tmp_path)

    assert control_cli.main(["explore"]) == 1
    assert "not ready" in json.loads(capsys.readouterr().err)["error"]
    assert not (runtime / "v3_command.json").exists()
