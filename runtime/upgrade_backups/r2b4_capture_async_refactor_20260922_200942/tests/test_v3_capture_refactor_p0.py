from __future__ import annotations

import pickle
import queue
from pathlib import Path

import pytest

import v3_process_runtime as process
from v3.capture_ipc import CaptureCoreExpander, CaptureCoreFrame, CaptureCoreProjector
from v3.mcap_capture import EncodedRecord, McapCaptureConfig, RAW_LIDAR_TOPIC, TICK_TOPIC
from v3.mcap_reader import McapReadError, McapReader
from v3.process_sidecars import ProcessMcapCaptureSession


def test_capture_process_ipc_uses_projected_frame_not_capture_record(tmp_path):
    from test_v3_mcap_e2e import records

    _, values = records(1)
    session = ProcessMcapCaptureSession(
        "projected",
        tmp_path / "projected.mcap",
        configuration={"x": 1},
        project_root=Path(process.__file__).parent,
    )
    session._data_queue.close()
    session._data_queue = queue.Queue(maxsize=4)
    session._started = True
    session.observe(values[0])
    kind, payload = session._data_queue.get_nowait()
    assert kind == "core"
    assert isinstance(payload, CaptureCoreFrame)
    assert not isinstance(payload, type(values[0]))
    assert payload.estimated_bytes <= 4 * 1024 * 1024
    assert pickle.loads(pickle.dumps(payload)) == payload


def test_capture_projection_roundtrip_restores_canonical_row():
    from test_v3_mcap_e2e import records
    from v3.capture_encoding import encode_capture_record

    _, values = records(2)
    projector = CaptureCoreProjector()
    expander = CaptureCoreExpander()
    for record in values:
        projected = projector.project(record)
        assert expander.expand(projected) == encode_capture_record(record)


def test_total_ring_byte_pressure_evicts_raw_before_core(tmp_path):
    from v3.mcap_capture import McapCaptureConsumer
    from v3.observation import ObservationHub

    hub = ObservationHub()
    sub = hub.subscribe_reliable("capture-test", capacity=8, required=True)
    consumer = McapCaptureConsumer(
        "ring-priority",
        tmp_path / "ring.mcap",
        subscription=sub,
        configuration={"x": 1},
        config=McapCaptureConfig(max_byte_capacity=4096, max_tick_count=8, max_raw_lidar_scans=8),
    )
    core = EncodedRecord(1, "v3.capture_record", TICK_TOPIC, 10, 1, b"x" * 100, tick_id=1)
    raw = EncodedRecord(
        2,
        "v3.raw_lidar",
        RAW_LIDAR_TOPIC,
        11,
        1,
        b"r" * 100,
        raw_lidar_revision=1,
    )
    consumer._append_ring(core)
    consumer._append_ring(raw)
    object.__setattr__(consumer._config, "max_byte_capacity", core.size_bytes + 8)
    consumer._enforce_capacities()
    assert any(item.mcap_topic == TICK_TOPIC for item in consumer._ring)
    assert not any(item.mcap_topic == RAW_LIDAR_TOPIC for item in consumer._ring)
    assert consumer._raw_capacity_eviction_count >= 1
    assert consumer._core_capacity_eviction_count == 0
    hub.close()


def test_raw_loss_is_not_replay_loss(tmp_path):
    from test_v3_mcap_e2e import records, raw
    from v3.mcap_capture import McapCaptureConsumer
    from v3.mcap_replay_bridge import replay_mcap
    from v3.observation import ObservationHub

    config, values = records(4)
    path = tmp_path / "split-integrity.mcap"
    hub = ObservationHub()
    sub = hub.subscribe_reliable(
        "capture",
        capacity=64,
        required=True,
        topics=("v3.capture_record", "v3.raw_lidar", "v3.raw_lidar_transport"),
    )
    consumer = McapCaptureConsumer(
        "split-integrity",
        path,
        subscription=sub,
        configuration={"resolved_control": config},
        config=McapCaptureConfig(
            mode="append_only",
            require_raw_lidar_transport_end=True,
        ),
    )
    consumer.start()
    for record in values:
        hub.publish(record, topic="v3.capture_record")
    for revision in (1, 2, 4):
        hub.publish(
            raw(revision, values[revision - 1].inputs.context.monotonic_ns),
            topic="v3.raw_lidar",
        )
    hub.publish(
        {
            "event_type": "raw_lidar_transport_end",
            "last_revision": 4,
            "produced_count": 4,
            "superseded_count": 1,
        },
        topic="v3.raw_lidar_transport",
    )
    hub.close()
    result = consumer.finish("PASS", terminal=True)
    assert result is not None
    assert result.replay_complete is True
    assert result.raw_evidence_complete is False
    assert result.complete is False

    reader = McapReader(path)
    with pytest.raises(McapReadError, match="required evidence is incomplete"):
        reader.capture_integrity()
    final = reader.capture_integrity(require_raw_evidence=False)
    assert final["integrity"]["replay_complete"] is True
    assert final["integrity"]["raw_evidence_complete"] is False
    assert replay_mcap(path, project_root=Path(process.__file__).parent)["status"] == "MATCH"


def test_production_capture_requires_raw_end_marker():
    source = Path(process.__file__).read_text(encoding="utf-8")
    assert "expect_raw_lidar_end=True" in source
    assert "require_raw_lidar_transport_end=True" in source
