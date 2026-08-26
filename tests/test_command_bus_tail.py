import json
import os
import threading
import time
from pathlib import Path

from controller import command_bus


def _record(cmd_id: str, state: str) -> str:
    return json.dumps({"cmd_id": cmd_id, "state": state}) + "\n"


def test_latest_command_status_reads_last_matching_state(tmp_path, monkeypatch):
    journal = tmp_path / "command_status.jsonl"
    journal.write_text(
        _record("target", "accepted")
        + _record("other", "effective")
        + _record("target", "applied")
        + _record("target", "effective"),
        encoding="utf-8",
    )
    monkeypatch.setattr(command_bus, "COMMAND_STATUS_PATH", journal)

    result = command_bus.get_latest_command_status("target", max_lines=4)

    assert result is not None
    assert result["state"] == "effective"


def test_latest_command_status_preserves_last_n_line_contract(tmp_path, monkeypatch):
    journal = tmp_path / "command_status.jsonl"
    journal.write_text(
        _record("outside-window", "effective")
        + _record("one", "effective")
        + _record("two", "effective"),
        encoding="utf-8",
    )
    monkeypatch.setattr(command_bus, "COMMAND_STATUS_PATH", journal)

    assert command_bus.get_latest_command_status("outside-window", max_lines=2) is None


def test_tail_reader_does_not_call_full_file_readlines(tmp_path, monkeypatch):
    journal = tmp_path / "command_status.jsonl"
    prefix = "".join(_record(f"old-{idx}", "effective") for idx in range(20_000))
    journal.write_text(prefix + _record("target", "effective"), encoding="utf-8")
    monkeypatch.setattr(command_bus, "COMMAND_STATUS_PATH", journal)

    original_open = Path.open
    bytes_read = 0

    class TrackingStream:
        def __init__(self, wrapped):
            self._wrapped = wrapped

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return self._wrapped.__exit__(*args)

        def seek(self, *args):
            return self._wrapped.seek(*args)

        def tell(self):
            return self._wrapped.tell()

        def read(self, size=-1):
            nonlocal bytes_read
            data = self._wrapped.read(size)
            bytes_read += len(data)
            return data

        def readlines(self, *_args, **_kwargs):
            raise AssertionError("full-file readlines is forbidden")

    def tracking_open(path_obj, *args, **kwargs):
        wrapped = original_open(path_obj, *args, **kwargs)
        if path_obj == journal:
            return TrackingStream(wrapped)
        return wrapped

    monkeypatch.setattr(Path, "open", tracking_open)

    result = command_bus.get_latest_command_status("target", max_lines=20)

    assert result is not None
    assert result["state"] == "effective"
    assert bytes_read <= command_bus.COMMAND_STATUS_TAIL_CHUNK_BYTES
    assert bytes_read < journal.stat().st_size


def test_command_status_async_writer_keeps_latest_per_command(monkeypatch, tmp_path):
    writer = command_bus._LatestCommandStatusWriter(max_pending=4)
    journal = tmp_path / "command_status.jsonl"
    calls = []
    first_write_entered = threading.Event()
    release_first_write = threading.Event()

    def fake_append(path, entry):
        calls.append(dict(entry))
        if entry.get("cmd_id") == "hold":
            first_write_entered.set()
            release_first_write.wait(timeout=1.0)
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with Path(path).open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(entry) + "\n")
        return True

    monkeypatch.setattr(command_bus, "append_jsonl", fake_append)

    assert writer.submit(journal, {"cmd_id": "hold", "state": "accepted"})
    assert first_write_entered.wait(timeout=1.0)
    assert writer.submit(journal, {"cmd_id": "target", "state": "accepted"})
    assert writer.submit(journal, {"cmd_id": "target", "state": "applied"})
    assert writer.submit(journal, {"cmd_id": "target", "state": "effective"})

    release_first_write.set()
    assert writer.flush_for_tests(timeout_s=1.0)

    target_calls = [entry for entry in calls if entry.get("cmd_id") == "target"]
    assert target_calls == [{"cmd_id": "target", "state": "effective"}]
    status = writer.status()
    assert status["latest_per_command"] is True
    assert status["max_pending"] == 4
    assert status["dropped_superseded_or_overflow"] >= 2


def test_append_command_status_is_eventually_queryable(tmp_path, monkeypatch):
    journal = tmp_path / "command_status.jsonl"
    monkeypatch.setattr(command_bus, "COMMAND_STATUS_PATH", journal)

    entry = command_bus.append_command_status("cmd-1", "effective", cmd_type="set_twist")
    assert entry["state"] == "effective"
    assert command_bus.flush_command_status_writer_for_tests(timeout_s=1.0)

    result = command_bus.get_latest_command_status("cmd-1", max_lines=4)
    assert result is not None
    assert result["state"] == "effective"


def test_latest_command_status_falls_back_to_verified_live_runtime_session(tmp_path, monkeypatch):
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir(parents=True)
    (runtime_dir / "status.json").write_text(
        json.dumps({"runtime_process": {"pid": os.getpid()}}),
        encoding="utf-8",
    )
    runtime_journal = tmp_path / "logs" / "session_runtime" / "runtime" / "command_status.jsonl"
    runtime_journal.parent.mkdir(parents=True)
    runtime_journal.write_text(_record("m0-stop", "effective"), encoding="utf-8")
    hub_journal = tmp_path / "logs" / "session_hub" / "runtime" / "command_status.jsonl"

    monkeypatch.setattr(command_bus, "PROJECT_ROOT", tmp_path)
    monkeypatch.setattr(command_bus, "RUNTIME_DIR", runtime_dir)
    monkeypatch.setattr(command_bus, "COMMAND_STATUS_PATH", hub_journal)
    monkeypatch.setattr(
        command_bus,
        "_COMMAND_STATUS_FALLBACK_CACHE",
        {"checked_monotonic": 0.0, "paths": ()},
    )

    result = command_bus.get_latest_command_status("m0-stop", max_lines=4)

    assert result is not None
    assert result["state"] == "effective"


def test_latest_command_status_rejects_fallback_when_runtime_status_is_stale(tmp_path, monkeypatch):
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir(parents=True)
    status_path = runtime_dir / "status.json"
    status_path.write_text(
        json.dumps({"runtime_process": {"pid": os.getpid()}}),
        encoding="utf-8",
    )
    stale_ts = time.time() - command_bus.COMMAND_STATUS_LIVE_STATUS_MAX_AGE_S - 1.0
    os.utime(status_path, (stale_ts, stale_ts))
    runtime_journal = tmp_path / "logs" / "session_runtime" / "runtime" / "command_status.jsonl"
    runtime_journal.parent.mkdir(parents=True)
    runtime_journal.write_text(_record("stale-stop", "effective"), encoding="utf-8")
    hub_journal = tmp_path / "logs" / "session_hub" / "runtime" / "command_status.jsonl"

    monkeypatch.setattr(command_bus, "PROJECT_ROOT", tmp_path)
    monkeypatch.setattr(command_bus, "RUNTIME_DIR", runtime_dir)
    monkeypatch.setattr(command_bus, "COMMAND_STATUS_PATH", hub_journal)
    monkeypatch.setattr(
        command_bus,
        "_COMMAND_STATUS_FALLBACK_CACHE",
        {"checked_monotonic": 0.0, "paths": ()},
    )

    assert command_bus.get_latest_command_status("stale-stop", max_lines=4) is None
