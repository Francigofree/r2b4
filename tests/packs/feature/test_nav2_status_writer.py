"""The passive SD-card snapshot cannot stall navigation or grow a queue."""
import json
import threading
import time
from pathlib import Path

from v3.nav2_diagnostics import Nav2StatusWriter


def test_blocked_disk_keeps_only_latest_snapshot_and_preserves_encoded_lineage(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = Path.write_text

    def blocked_write(path, encoded, *args, **kwargs):
        entered.set()
        assert release.wait(3.)
        return original(path, encoded, *args, **kwargs)

    monkeypatch.setattr(Path, "write_text", blocked_write)
    writer = Nav2StatusWriter(tmp_path/"bridge_status.json")
    try:
        writer.publish('{"measurement_ns": 1, "sequence": 1}')
        assert entered.wait(1.)
        started = time.monotonic()
        for sequence in range(2, 102):
            writer.publish(json.dumps(dict(measurement_ns=sequence, sequence=sequence)))
        assert time.monotonic()-started < .1
        assert writer.snapshot()["superseded"] == 99
        assert writer.snapshot()["written"] == 0
    finally:
        release.set()
        assert writer.close()
    assert json.loads(writer.path.read_text()) == dict(measurement_ns=101, sequence=101)
    assert writer.snapshot()["written"] == 2


def test_disk_failure_is_visible_and_does_not_acquire_navigation_authority(tmp_path, monkeypatch):
    def fail(*args, **kwargs):
        raise OSError("injected disk failure")

    monkeypatch.setattr(Path, "write_text", fail)
    writer = Nav2StatusWriter(tmp_path/"bridge_status.json")
    writer.publish('{"sequence":1}')
    assert writer.close()
    stats = writer.snapshot()
    assert stats["errors"] == 1 and stats["written"] == 0
    assert "injected disk failure" in stats["last_error"]
    assert not writer.path.exists()


def test_shutdown_has_a_deadline_even_while_disk_write_is_blocked(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = Path.write_text

    def blocked_write(path, encoded, *args, **kwargs):
        entered.set()
        release.wait(3.)
        return original(path, encoded, *args, **kwargs)

    monkeypatch.setattr(Path, "write_text", blocked_write)
    writer = Nav2StatusWriter(tmp_path/"bridge_status.json")
    try:
        writer.publish('{"sequence":1}')
        assert entered.wait(1.)
        started = time.monotonic()
        assert not writer.close()
        assert time.monotonic()-started < 1.5
    finally:
        release.set()
        assert writer.close()
