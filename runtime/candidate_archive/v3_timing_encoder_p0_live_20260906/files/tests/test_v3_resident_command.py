import json
import os
from pathlib import Path

import pytest

from v3.adapters.resident_command import (
    AtomicResidentCommandGateway,
    RESIDENT_COMMAND_SCHEMA,
    ResidentCommandMailboxConfig,
)
from v3.contracts import CommandMode, TickContext


def _gateway(path: Path, *, observed_ns=1_000, **overrides):
    monotonic_ns = observed_ns if callable(observed_ns) else lambda: observed_ns
    return AtomicResidentCommandGateway(
        ResidentCommandMailboxConfig(path=path, **overrides),
        monotonic_ns=monotonic_ns,
    )


def _payload(revision=1, *, issued=1_000, expires=101_000, mode="TELEOP"):
    payload = {
        "schema": RESIDENT_COMMAND_SCHEMA,
        "revision": revision,
        "issued_monotonic_ns": issued,
        "expires_monotonic_ns": expires,
        "mode": mode,
    }
    if mode == "TELEOP":
        payload.update(
            {
                "v_mps": 0.04,
                "omega_rad_s": 0.01,
                "max_v_mps": 0.05,
                "max_omega_rad_s": 0.10,
            }
        )
    return payload


def _write(path: Path, payload, *, mode=0o600):
    path.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
    path.chmod(mode)


def _goal(command):
    return {field.key: field.value for field in command.goal}


def test_missing_mailbox_is_stop_and_valid_revision_is_repeatable_until_expiry(tmp_path):
    path = tmp_path / "command.json"
    gateway = _gateway(
        path,
        observed_ns=iter((2_000, 100_000, 101_001)).__next__,
        maximum_ttl_ns=200_000,
        maximum_future_skew_ns=0,
    )

    missing = gateway.snapshot(TickContext(0, 1_000))
    assert missing.mode is CommandMode.STOP
    assert gateway.last_revision is None

    _write(path, _payload())
    active = gateway.snapshot(TickContext(1, 2_000))
    repeated = gateway.snapshot(TickContext(2, 100_000))
    expired = gateway.snapshot(TickContext(3, 101_001))

    assert active.mode is CommandMode.TELEOP
    assert repeated.mode is CommandMode.TELEOP
    assert _goal(active) == {
        "v_mps": 0.04,
        "omega_rad_s": 0.01,
        "max_v_mps": 0.05,
        "max_omega_rad_s": 0.10,
    }
    assert expired.mode is CommandMode.STOP
    assert expired.command_id == "resident.mailbox.expired.1.3"
    assert gateway.last_revision == 1


def test_mailbox_freshness_uses_ingress_observation_after_blocking_l0_context(
    tmp_path,
):
    path = tmp_path / "command.json"
    _write(
        path,
        _payload(
            issued=1_300_000_000,
            expires=1_500_000_000,
        ),
    )
    gateway = _gateway(path, observed_ns=1_310_000_000)

    command = gateway.snapshot(TickContext(7, 1_000_000_000))

    assert command.mode is CommandMode.TELEOP
    assert command.context == TickContext(7, 1_000_000_000)


def test_mailbox_expiry_uses_ingress_observation_not_stale_tick_context(tmp_path):
    path = tmp_path / "command.json"
    _write(path, _payload(issued=1_000, expires=101_000))
    gateway = _gateway(path, observed_ns=101_001)

    command = gateway.snapshot(TickContext(9, 2_000))

    assert command.mode is CommandMode.STOP
    assert command.command_id == "resident.mailbox.expired.1.9"


def test_stop_revision_and_strict_motion_limits_close_to_command_request(tmp_path):
    path = tmp_path / "command.json"
    gateway = _gateway(path)
    _write(path, _payload(mode="STOP"))

    stopped = gateway.snapshot(TickContext(0, 1_000))
    assert stopped.mode is CommandMode.STOP
    assert stopped.goal == ()

    active_payload = _payload(revision=2)
    _write(path, active_payload)
    active = gateway.snapshot(TickContext(1, 2_000))
    assert active.mode is CommandMode.TELEOP
    assert active.command_id == "resident.mailbox.teleop.2"

    active_payload["revision"] = 3
    active_payload["v_mps"] = 0.051
    _write(path, active_payload)
    with pytest.raises(ValueError, match="declared limit"):
        gateway.snapshot(TickContext(2, 3_000))


def test_default_process_limit_accepts_live_baseline_and_caps_at_half_mps(tmp_path):
    path = tmp_path / "command.json"
    gateway = _gateway(path)

    for revision, speed in ((1, 0.15), (2, 0.50)):
        payload = _payload(revision=revision)
        payload.update(v_mps=speed, max_v_mps=speed)
        _write(path, payload)

        command = gateway.snapshot(TickContext(revision, 1_000 + revision))

        assert command.mode is CommandMode.TELEOP
        assert _goal(command)["v_mps"] == speed
        assert _goal(command)["max_v_mps"] == speed


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda value: value.update(schema="wrong"), "schema"),
        (lambda value: value.update(mode="NAVIGATE"), "STOP or TELEOP"),
        (lambda value: value.update(expires_monotonic_ns=300_000_000), "TTL"),
        (
            lambda value: value.update(
                issued_monotonic_ns=10_000_000,
                expires_monotonic_ns=10_100_000,
            ),
            "future",
        ),
        (lambda value: value.update(max_v_mps=0.500001), "process limit"),
    ],
)
def test_invalid_contracts_fail_closed(tmp_path, mutate, message):
    path = tmp_path / "command.json"
    payload = _payload()
    mutate(payload)
    _write(path, payload)

    with pytest.raises(ValueError, match=message):
        _gateway(path).snapshot(TickContext(0, 1_000))


def test_revision_rewrite_regression_and_reappearance_are_rejected(tmp_path):
    path = tmp_path / "command.json"
    gateway = _gateway(path)
    _write(path, _payload(revision=2))
    assert gateway.snapshot(TickContext(0, 2_000)).mode is CommandMode.TELEOP

    rewritten = _payload(revision=2)
    rewritten["v_mps"] = 0.03
    _write(path, rewritten)
    with pytest.raises(ValueError, match="rewritten"):
        gateway.snapshot(TickContext(1, 3_000))

    _write(path, _payload(revision=1))
    with pytest.raises(ValueError, match="regressed"):
        gateway.snapshot(TickContext(2, 4_000))

    _write(path, _payload(revision=2))
    path.unlink()
    assert gateway.snapshot(TickContext(3, 5_000)).mode is CommandMode.STOP
    _write(path, _payload(revision=2))
    with pytest.raises(ValueError, match="advance after mailbox removal"):
        gateway.snapshot(TickContext(4, 6_000))
    _write(path, _payload(revision=3))
    assert gateway.snapshot(TickContext(5, 7_000)).mode is CommandMode.TELEOP


def test_mailbox_must_be_owned_regular_private_and_bounded(tmp_path):
    path = tmp_path / "command.json"
    gateway = _gateway(path, maximum_file_bytes=512)
    _write(path, _payload(), mode=0o620)
    with pytest.raises(ValueError, match="group/world writable"):
        gateway.snapshot(TickContext(0, 2_000))

    path.unlink()
    path.symlink_to(tmp_path / "missing-target")
    with pytest.raises(ValueError, match="opened safely"):
        gateway.snapshot(TickContext(1, 3_000))

    path.unlink()
    path.write_bytes(b"{" + b"x" * 600 + b"}")
    os.chmod(path, 0o600)
    with pytest.raises(ValueError, match="maximum_file_bytes"):
        gateway.snapshot(TickContext(2, 4_000))
