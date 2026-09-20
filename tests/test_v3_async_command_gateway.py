import threading
import time

import pytest

from v3.adapters.resident_command import (
    AsyncResidentCommandGateway, AtomicResidentCommandGateway, ResidentCommandMailboxConfig,
)
from v3.contracts import CommandMode, TickContext
from test_v3_resident_command import _payload, _write


def _wait(predicate):
    deadline = time.monotonic() + 1.0
    while not predicate():
        assert time.monotonic() < deadline
        time.sleep(.002)


def test_blocked_file_read_cannot_stall_tick_or_extend_ttl(tmp_path, monkeypatch):
    path = tmp_path / "command.json"
    _write(path, _payload())
    entered, release = threading.Event(), threading.Event()
    original = AtomicResidentCommandGateway._read_trusted_bytes
    count = 0

    def blocked(self):
        nonlocal count
        count += 1
        if count > 1:
            entered.set()
            assert release.wait(2.0)
        return original(self)

    monkeypatch.setattr(AtomicResidentCommandGateway, "_read_trusted_bytes", blocked)
    now = [2_000]
    gateway = AsyncResidentCommandGateway(
        ResidentCommandMailboxConfig(path=path), monotonic_ns=lambda: now[0],
    )
    gateway.start()
    try:
        assert entered.wait(1.0)
        started = time.monotonic()
        assert gateway.snapshot(TickContext(1, now[0])).mode is CommandMode.TELEOP
        now[0] = 101_001
        assert gateway.snapshot(TickContext(2, now[0])).mode is CommandMode.STOP
        assert time.monotonic() - started < .05
    finally:
        release.set()
        gateway.close()


def test_async_gateway_keeps_rewrite_and_trust_failures_fail_closed(tmp_path):
    path = tmp_path / "command.json"
    _write(path, _payload())
    gateway = AsyncResidentCommandGateway(
        ResidentCommandMailboxConfig(path=path), monotonic_ns=lambda: 2_000,
    )
    gateway.start()
    try:
        _wait(lambda: gateway._mailbox is not None)
        assert gateway.snapshot(TickContext(1, 2_000)).mode is CommandMode.TELEOP
        old = gateway._mailbox
        _write(path, _payload(command_id="rewritten"))
        _wait(lambda: gateway._mailbox != old)
        with pytest.raises(ValueError, match="rewritten"):
            gateway.snapshot(TickContext(2, 2_000))
        path.chmod(0o666)
        _wait(lambda: gateway._read_error is not None)
        with pytest.raises(ValueError, match="mailbox read failed"):
            gateway.snapshot(TickContext(3, 2_000))
    finally:
        gateway.close()
