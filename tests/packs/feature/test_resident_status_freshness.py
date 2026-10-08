"""Passive status delivery keeps the newest completed tick without renewing it."""
from __future__ import annotations

import json
import multiprocessing
import time
from types import SimpleNamespace

import pytest

from test_spatial_service import Clock, geometry_tick
from v3.process_sidecars import ProcessResidentStatusPublisher, _StatusMailbox
from v3.resident_status import HOST_STATUS_MAX_AGE_NS, _tick_status, status_is_fresh


def _wait_status(path, tick_id):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        received = json.loads(path.read_text())
        if received.get("tick_id") == tick_id:
            return received
        time.sleep(.005)
    pytest.fail(f"status did not deliver tick {tick_id}")


def _blocked_status_sidecar(entered, release, written, *arguments):
    from v3 import process_sidecars
    original = process_sidecars._atomic_private_json

    def write(path, payload, mode):
        if payload["state"] == "RUNNING":
            if not entered.is_set():
                entered.set()
                if not release.wait(5):
                    raise TimeoutError("test status writer was not released")
            written.value += 1
        original(path, payload, mode)

    process_sidecars._atomic_private_json = write
    process_sidecars._status_sidecar_main(*arguments)


def test_slow_process_writer_supersedes_backlog_and_preserves_final_tick(tmp_path):
    context = multiprocessing.get_context("spawn")
    entered, release = context.Event(), context.Event()
    written = context.RawValue("I", 0)
    path = tmp_path / "status.json"
    publisher = ProcessResidentStatusPublisher(SimpleNamespace(path=path, file_mode=0o600))
    publisher._process = context.Process(target=_blocked_status_sidecar, args=(
        entered, release, written, str(path), 0o600, publisher._tick_mailbox,
        publisher._control_queue, publisher._result_queue, publisher._ready_event,
        publisher._failed_event, None, False,
    ))
    publisher.start()
    clock = Clock()
    try:
        publisher.publish_tick(geometry_tick(clock, tick=1))
        assert entered.wait(3)
        started = time.monotonic()
        for tick_id in range(2, 502):
            clock.now += 20_000_000
            latest = geometry_tick(clock, tick=tick_id, sequence=tick_id,
                                   cells=4000 if tick_id == 501 else 1)
            publisher.publish_tick(latest, ready_for_active=True)
        assert time.monotonic() - started < 2
        assert publisher.drop_count == 0 and not publisher.failed
        release.set()
        received = _wait_status(path, latest.trace.context.tick_id)
        assert received == _tick_status(latest, ready_for_active=True)
        assert written.value == 2  # In-flight tick, then the final pending tick.
        assert status_is_fresh(received, observed_ns=clock.now)
        assert not status_is_fresh(received, observed_ns=clock.now + HOST_STATUS_MAX_AGE_NS)
        assert "occupied_cells" not in received["world"]["local_costmap"]
        from v3_process_runtime import AsyncResidentStatusPublisher, ResidentStatusConfig
        direct = AsyncResidentStatusPublisher(ResidentStatusConfig(path=tmp_path / "direct.json"))
        direct.start()
        try:
            direct.publish_tick(latest, ready_for_active=True)
            assert _wait_status(direct._config.path, latest.trace.context.tick_id) == received
        finally:
            direct.finish()
    finally:
        release.set()
        publisher.finish()


def test_status_publication_skips_contended_mailbox_without_waiting(tmp_path):
    publisher = ProcessResidentStatusPublisher(SimpleNamespace(path=tmp_path / "status.json", file_mode=0o600))
    publisher.start()
    clock = Clock()
    try:
        first = geometry_tick(clock, tick=1)
        publisher.publish_tick(first)
        _wait_status(publisher._config.path, 1)
        assert publisher._tick_mailbox._lock.acquire(timeout=1)
        try:
            started = time.monotonic()
            for tick_id in range(2, 102):
                publisher.publish_tick(geometry_tick(clock, tick=tick_id))
            assert time.monotonic() - started < .5
            assert publisher.drop_count == 100
        finally:
            publisher._tick_mailbox._lock.release()
        final = geometry_tick(clock, tick=102)
        publisher.publish_tick(final)
        assert _wait_status(publisher._config.path, 102) == _tick_status(final)
    finally:
        publisher.finish()


def test_status_sidecar_crash_is_observable_and_does_not_renew_last_tick(tmp_path):
    publisher = ProcessResidentStatusPublisher(SimpleNamespace(path=tmp_path / "status.json", file_mode=0o600))
    publisher.start()
    clock = Clock()
    tick = geometry_tick(clock)
    publisher.publish_tick(tick)
    last = _wait_status(publisher._config.path, tick.trace.context.tick_id)
    publisher._process.terminate()
    publisher._process.join(3)
    try:
        assert publisher.failed
        assert "status sidecar exited" in str(publisher.error)
        publisher.publish_tick(geometry_tick(clock, tick=2))
        assert json.loads(publisher._config.path.read_text()) == last
        assert not status_is_fresh(last, observed_ns=clock.now + HOST_STATUS_MAX_AGE_NS)
    finally:
        publisher.finish()


def test_status_file_error_is_reported_by_the_sidecar(tmp_path):
    path = tmp_path / "status.json"
    publisher = ProcessResidentStatusPublisher(SimpleNamespace(path=path, file_mode=0o600))
    publisher.start()
    try:
        path.unlink()
        path.mkdir()
        publisher.publish_tick(geometry_tick(Clock()))
        deadline = time.monotonic() + 3
        while publisher.error is None and time.monotonic() < deadline:
            time.sleep(.005)
        assert publisher.failed
        assert "IsADirectoryError" in str(publisher.error)
    finally:
        publisher.finish()


def test_unexpected_clean_sidecar_exit_is_a_failure_before_first_tick(tmp_path):
    publisher = ProcessResidentStatusPublisher(SimpleNamespace(path=tmp_path / "status.json", file_mode=0o600))
    publisher.start()
    try:
        publisher._control_queue.put(("finish", None, None, None), timeout=1)
        publisher._process.join(3)
        assert publisher._process.exitcode == 0
        assert publisher.failed
        assert "status sidecar exited:0" in str(publisher.error)
    finally:
        publisher.finish()


def test_status_encoding_failure_is_reported_without_throwing_through_completed_tick(tmp_path, monkeypatch):
    from v3 import process_sidecars
    publisher = ProcessResidentStatusPublisher(SimpleNamespace(path=tmp_path / "status.json", file_mode=0o600))
    publisher.start()
    try:
        tick = geometry_tick(Clock())
        actuation = tick.final_actuation
        monkeypatch.setattr(process_sidecars, "_tick_status", lambda *args: {"raw": b"x" * 70_000})
        publisher.publish_tick(tick)
        assert tick.final_actuation is actuation
        assert publisher.failed and publisher.drop_count == 1
        assert "compact status exceeds mailbox bound" in str(publisher.error)
        assert json.loads(publisher._config.path.read_text())["state"] == "BOOTING"
    finally:
        publisher.finish()


def test_status_mailbox_rejects_oversize_or_corrupt_values_without_partial_publication():
    mailbox = _StatusMailbox(multiprocessing.get_context("spawn"))
    assert mailbox.publish({"tick_id": 1, "monotonic_ns": 10})
    with pytest.raises(ValueError, match="mailbox bound"):
        mailbox.publish({"raw": b"x" * 70_000})
    revision, snapshot = mailbox.latest(0)
    assert revision == 1 and snapshot == {"tick_id": 1, "monotonic_ns": 10}
    mailbox._revision.value = 2
    mailbox._size.value = 0
    with pytest.raises(ValueError, match="mailbox size"):
        mailbox.latest(revision)


@pytest.mark.parametrize("stamp, observed, expected", [
    (10, 10, True), (10, 10 + HOST_STATUS_MAX_AGE_NS - 1, True),
    (10, 10 + HOST_STATUS_MAX_AGE_NS, False), (10, 9, False),
    (True, 10, False), (None, 10, False), ("10", 10, False),
])
def test_host_freshness_uses_original_tick_and_completed_observation(stamp, observed, expected):
    assert status_is_fresh({"monotonic_ns": stamp}, observed_ns=observed) is expected
