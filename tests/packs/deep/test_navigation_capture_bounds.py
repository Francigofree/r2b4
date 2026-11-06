"""Long sampled capture remains bounded and reports exhausted evidence quotas."""

from __future__ import annotations

import pytest

from v3.mcap_capture import EVENT_TOPIC, McapCaptureConfig, McapCaptureConsumer
from v3.mcap_reader import McapReader, McapReadError
from v3.observation import ObservationHub


def _worker(tmp_path, **settings):
    hub = ObservationHub()
    worker = McapCaptureConsumer(
        "bounded-navigation", tmp_path / "navigation.mcap",
        subscription=hub.subscribe_reliable("capture", capacity=2, required=True),
        configuration={},
        config=McapCaptureConfig(mode="append_only", tick_sample_hz=5, **settings))
    return hub, worker


def _publish(hub, worker, tick_id, monotonic_ns):
    # Pump accepted frames synchronously to simulate hours of source time without
    # a real-time wait or unrelated producer/consumer scheduling contention.
    hub.publish({
        "record_type": "system_behavior_tick", "tick_id": tick_id,
        "monotonic_ns": monotonic_ns, "expected": {"layers": {}},
    }, topic="v3.capture_record")
    worker._consume_observation(worker._subscription.get_nowait())


def _finish(hub, worker):
    hub.close()
    result = worker.finish()
    assert result is not None
    final = next(row for _, row in McapReader(result.path).iter_json_messages(topics=[EVENT_TOPIC])
                 if row.get("event_type") == "capture_finalized")
    return result, final["integrity"]


def test_two_hour_five_hz_capture_uses_scalar_sampling_accounting(tmp_path):
    hub, worker = _worker(tmp_path)
    for index in range(36_001):
        _publish(hub, worker, index * 10, 1_000_000_000 + index * 200_000_000)
    result, integrity = _finish(hub, worker)
    assert result.complete and not result.replay_complete
    assert result.tick_sequence_gap_count == 0
    assert integrity["sampled_tick_skip_count"] == 36_000 * 9
    assert integrity["captured_tick_sequence_gaps"] == []
    assert integrity["session_records_seen"] == 36_001
    assert integrity["session_records_dropped"] == 0
    assert McapReader(result.path).capture_integrity()["integrity"]["sample_complete"]


def test_anomalous_gap_examples_are_bounded_but_total_is_preserved(tmp_path):
    hub, worker = _worker(tmp_path)
    for index in range(100):
        _publish(hub, worker, 100 - index, 1_000_000_000 + index * 200_000_000)
    result, integrity = _finish(hub, worker)
    assert not result.complete
    assert result.tick_sequence_gap_count == integrity["captured_tick_sequence_gap_count"] == 99
    assert len(integrity["captured_tick_sequence_gaps"]) == 64
    assert integrity["captured_tick_sequence_gap_details_truncated"]
    assert "CAPTURED_TICK_SEQUENCE_NON_MONOTONIC" in integrity["integrity_reasons"]


def test_record_limit_keeps_terminal_transport_loss_and_explicit_drop_count(tmp_path):
    hub, worker = _worker(tmp_path, max_session_records=1)
    _publish(hub, worker, 0, 1_000_000_000)
    _publish(hub, worker, 10, 1_200_000_000)
    hub.publish({"drop_count": 3, "event_type": "capture_core_transport_loss"},
                topic="v3.capture_transport")
    worker._consume_observation(worker._subscription.get_nowait())
    result, integrity = _finish(hub, worker)
    assert not result.complete
    assert {"SESSION_RECORD_LIMIT", "CORE_TRANSPORT_LOSS"}.issubset(integrity["integrity_reasons"])
    assert integrity["session_records_seen"] == 3
    assert integrity["session_records_dropped"] == 2
    assert integrity["first_dropped_monotonic_ns"] <= integrity["last_dropped_monotonic_ns"]


def test_disk_quota_finalizes_incomplete_capture_inside_hard_limit(tmp_path):
    quota = 128 * 1024
    hub, worker = _worker(tmp_path, max_session_bytes=quota)
    for index in range(250):
        _publish(hub, worker, index * 10, 1_000_000_000 + index * 200_000_000)
    result, integrity = _finish(hub, worker)
    assert not result.complete
    assert result.path.stat().st_size <= quota
    assert "SESSION_BYTE_LIMIT" in integrity["integrity_reasons"]
    assert integrity["session_byte_limit"] == quota
    assert 0 < integrity["session_records_dropped"] < 250
    with pytest.raises(McapReadError, match="CAPTURE_INCOMPLETE"):
        McapReader(result.path).capture_integrity()


def test_metadata_cannot_overrun_even_a_too_small_disk_quota(tmp_path):
    hub, worker = _worker(tmp_path, max_session_bytes=64)
    hub.publish({"tick_id": 0, "monotonic_ns": 1_000_000_000}, topic="v3.capture_record")
    hub.close()
    with pytest.raises(RuntimeError, match="MCAP capture failed") as caught:
        worker.finish()
    assert "SESSION_BYTE_LIMIT" in str(caught.value.__cause__)
    assert not (tmp_path / "navigation.mcap").exists()
    partials = tuple(tmp_path.glob("*.partial"))
    assert len(partials) == 1 and partials[0].stat().st_size <= 64
