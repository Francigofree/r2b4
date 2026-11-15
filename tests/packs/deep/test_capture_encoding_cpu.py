"""Fused capture encoding preserves stored evidence without a second walk."""
from copy import deepcopy
from dataclasses import dataclass

import pytest

from rig import resolved_config
from v3.capture_behavior import encode_behavior_record, project_behavior_record
from v3.capture_compaction import compact_checkpoint_row, compact_tick_row
from v3.capture_encoding import CaptureEncodingError, encode_capture_record, encode_value
from v3.composition.full_fake import OfflineMotorSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import CommandMode, CommandRequest, DataField, LifecycleState, RawDeviceBatch, TickContext
from v3.engine import TickInputs
from v3.execution import ExecutionRecord, WriterFailureRecord
from v3.mcap_capture import McapCaptureConfig, McapCaptureConsumer, _json_bytes, _referenced_lidar_revisions
from v3.nav2_diagnostics import NAV2_OBSERVATION_TOPIC
from v3.observation import ObservationHub


@dataclass(frozen=True)
class NestedEvidence:
    fields: tuple


def test_fused_values_match_legacy_compaction_without_aliasing_or_input_changes():
    shared = {"__type__": "DataField", "key": "nested", "value": [1., False, None]}
    value = NestedEvidence((DataField("v_mps", .19), shared, shared,
        {**shared, "extra": 1}, {**shared, "key": 17}, frozenset(("b", "a"))))
    before = deepcopy(value)
    expected = compact_checkpoint_row({"state": encode_value(value)})
    actual = {"state": encode_value(value, compact_data_fields=True)}
    assert _json_bytes(actual) == _json_bytes(expected)
    assert value == before
    actual["state"]["fields"][1]["$df"][1].append(99)
    assert actual["state"]["fields"][2]["$df"][1] == [1., False, None]
    assert value == before
    for bad in (float("nan"), float("inf"), object()):
        with pytest.raises(CaptureEncodingError):
            encode_value(NestedEvidence((bad,)), compact_data_fields=True)


def test_behavior_tick_and_writer_fault_store_identical_bytes(tmp_path):
    context = TickContext(0, 1_000_000_000)
    composition = NativeControlComposition(OfflineMotorSink(),
        resolved_config().runtime.composition.live_control.control)
    inputs = composition.close_inputs(TickInputs(context, RawDeviceBatch(context, (), ()),
        CommandRequest(context, "offline-stop", CommandMode.STOP, (), 0), LifecycleState.IDLE))
    normal = ExecutionRecord(inputs, composition.run_tick(inputs))
    fault = WriterFailureRecord(context, LifecycleState.ACTIVE, "WRITE_FAILED", None)
    hub = ObservationHub()
    subscription = hub.subscribe_reliable("capture", capacity=2, required=True)
    consumer = McapCaptureConsumer("fused", tmp_path / "fused.mcap",
        subscription=subscription, configuration={}, config=McapCaptureConfig(tick_sample_hz=5))
    for record in (normal, fault):
        full = encode_capture_record(record)
        compact_full = encode_capture_record(record, compact_data_fields=True)
        assert _json_bytes(compact_tick_row(full)) == _json_bytes(
            compact_tick_row(compact_full, data_fields_compacted=True))
        behavior = project_behavior_record(record)
        legacy = compact_tick_row(encode_behavior_record(behavior))
        encoded = encode_behavior_record(behavior, compact_data_fields=True)
        before = deepcopy(encoded)
        fused = compact_tick_row(encoded, data_fields_compacted=True)
        assert _json_bytes(fused) == _json_bytes(legacy)
        assert encoded == before
        hub.publish(behavior, topic="v3.capture_record")
        observed, = consumer._encode_observation(subscription.get_nowait())
        assert observed.payload == _json_bytes(legacy)
    hub.close()


def test_compact_matcher_fields_keep_raw_scan_integrity_references():
    row = {"inputs": {"raw_devices": {"samples": [{"kind": "lidar_matcher_diagnostics",
        "sequence": 7, "values": [DataField("source_raw_scan_id", 23)]},
        {"kind": "lidar_safety_clearance", "sequence": 19}]}}}
    expected = _referenced_lidar_revisions(encode_value(row))
    assert expected == {23, 19}
    assert _referenced_lidar_revisions(encode_value(row, compact_data_fields=True)) == expected


def test_json_nav2_observation_keeps_canonical_bytes_and_finite_validation(tmp_path):
    hub = ObservationHub()
    subscription = hub.subscribe_reliable("capture", capacity=2, required=True)
    consumer = McapCaptureConsumer("json", tmp_path / "json.mcap",
        subscription=subscription, configuration={}, config=McapCaptureConfig(tick_sample_hz=5))
    payload = dict(measurement_monotonic_ns=123, sample_sequence=7,
        costmap={"cells": [[-1, 254, 255]], "origin": [1., 2.]})
    hub.publish(payload, topic=NAV2_OBSERVATION_TOPIC)
    frame = subscription.get_nowait()
    encoded, = consumer._encode_observation(frame)
    assert encoded.payload == _json_bytes(dict(source_topic=frame.topic,
        hub_sequence=frame.sequence, published_monotonic_ns=frame.published_monotonic_ns,
        monotonic_ns=123, payload=encode_value(payload)))
    hub.publish(dict(payload, invalid=float("nan")), topic=NAV2_OBSERVATION_TOPIC)
    with pytest.raises(CaptureEncodingError):
        consumer._encode_observation(subscription.get_nowait())
    hub.close()
