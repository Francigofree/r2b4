import json
from unittest.mock import patch

from log.unified_logger import UnifiedLogger


def _config(base_dir):
    return {
        "level": "INFO",
        "enabled": True,
        "enable_debug": False,
        "buffer_size": 128,
        "flush_interval_ms": 1000,
        "enable_audit_hash_chain": True,
        "channels": {},
        "session": {
            "base_dir": str(base_dir),
            "max_sessions": 2,
            "gzip_on_close": False,
        },
        "system_monitor": {"enabled": False},
        "sensor_diag": {"enabled": False},
        "media_meta": {"enabled": False},
    }


def test_periodic_housekeeping_queues_runtime_stats_without_control_thread_io(tmp_path):
    logger = UnifiedLogger(_config(tmp_path))
    context = logger.start()

    with patch.object(logger, "_write_runtime_stats") as synchronous_write:
        stats = logger.run_housekeeping(force=True)
        synchronous_write.assert_not_called()
        assert stats["write_errors"] == 0
        logger._async_logger._flush_all()

    payload = json.loads((context.session_dir / "runtime" / "runtime_stats.json").read_text(encoding="utf-8"))
    assert payload["session_id"] == context.session_id
    assert payload["stats"]["write_errors"] == 0
    logger.stop()


def test_final_session_stop_still_writes_complete_runtime_stats(tmp_path):
    logger = UnifiedLogger(_config(tmp_path))
    context = logger.start()
    logger.stop()
    payload = json.loads((context.session_dir / "runtime" / "runtime_stats.json").read_text(encoding="utf-8"))
    assert payload["session_id"] == context.session_id
    assert "dropped_messages" in payload["stats"]
    assert "write_errors" in payload["stats"]


def test_priority_channels_enqueue_without_immediate_control_thread_io(tmp_path):
    logger = UnifiedLogger(_config(tmp_path))
    context = logger.start()
    try:
        with patch.object(
            logger._async_logger,
            "write_jsonl_immediate",
            side_effect=AssertionError("priority log must not write synchronously"),
        ) as immediate_write:
            assert logger.emit_audit("COMMAND_RX", "GUI", details={"type": "set_twist"})
            assert logger.log_event("safety", "supervisor", "safety_warn", {"ok": False}, level="WARN")
            immediate_write.assert_not_called()

        stats = logger.run_housekeeping(force=True)
        assert stats["total_immediate_jsonl"] == 0
        logger._async_logger._flush_all()

        audit_records = [
            json.loads(line)
            for line in (context.session_dir / "runtime" / "audit.jsonl").read_text(encoding="utf-8").splitlines()
        ]
        safety_records = [
            json.loads(line)
            for line in (context.session_dir / "runtime" / "safety.jsonl").read_text(encoding="utf-8").splitlines()
        ]
        assert any(record["event"] == "command_rx" for record in audit_records)
        assert any(record["event"] == "safety_warn" for record in safety_records)
    finally:
        logger.stop()
