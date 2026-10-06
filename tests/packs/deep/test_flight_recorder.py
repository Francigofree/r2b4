"""Passive observations retain domain and delivery lineage through recording."""

from __future__ import annotations

from dataclasses import dataclass
import json
import time

import pytest

from tools.mcap_evidence.compiler import compile_evidence
from tools.mcap_evidence.query import query
from tools.mcap_evidence.verify import verify
from v3.mcap_capture import EVENT_TOPIC, McapCaptureConfig, McapCaptureConsumer
from v3.mcap_reader import McapReader, McapReadError
from v3.observation import ObservationHub, ObservationIntegrityError


@dataclass(frozen=True, slots=True)
class FutureObservation:
    measurement_monotonic_ns: int
    sequence: int
    revision: int
    lineage: tuple[str, ...]
    future_field: tuple[object, ...]


def closed_stop_record():
    from rig import resolved_config
    from v3.composition.full_fake import OfflineMotorSink
    from v3.composition.native_control import NativeControlComposition
    from v3.contracts import CommandMode, CommandRequest, LifecycleState, RawDeviceBatch, TickContext
    from v3.engine import TickInputs
    from v3.execution import ExecutionRecord

    composition = NativeControlComposition(
        OfflineMotorSink(), resolved_config().runtime.composition.live_control.control)
    context = TickContext(0, time.monotonic_ns())
    inputs = composition.close_inputs(TickInputs(
        context, RawDeviceBatch(context, (), ()),
        CommandRequest(context, "flight-recorder-offline-stop", CommandMode.STOP, (), 0),
        LifecycleState.IDLE))
    return ExecutionRecord(inputs, composition.run_tick(inputs))


def consumer(hub, output, *, capacity=64):
    subscription = hub.subscribe_reliable("flight-recorder", capacity=capacity, required=True)
    return McapCaptureConsumer(
        "offline-flight-recorder", output, subscription=subscription,
        configuration={}, config=McapCaptureConfig(mode="append_only", tick_sample_hz=10))


def test_slow_latest_is_isolated_and_required_overrun_is_permanent():
    hub = ObservationHub()
    required = hub.subscribe_reliable("recorder", capacity=2, required=True)
    latest = hub.subscribe_latest("gui", capacity=1)
    # No encoding or domain-field inspection is needed to publish this value.
    payload = object()
    receipts = [hub.publish(payload, topic="future.observation") for _ in range(3)]
    assert all(receipt.frame.payload is payload for receipt in receipts)
    assert required.drain() == tuple(receipt.frame for receipt in receipts[:2])
    assert latest.get_nowait() is receipts[-1].frame
    assert latest.snapshot().superseded_count == 2
    assert latest.snapshot().integrity_ok
    assert required.snapshot().lost_count == 1
    assert required.snapshot().first_lost_sequence == receipts[-1].frame.sequence
    hub.close()
    with pytest.raises(ObservationIntegrityError):
        required.assert_integrity()
    assert hub.snapshot().required_failures == ("recorder",)


def test_arbitrary_source_observations_drain_to_mcap_and_compiler_unchanged(tmp_path):
    hub = ObservationHub()
    worker = consumer(hub, tmp_path / "flight.mcap")
    gui = hub.subscribe_latest("gui", capacity=1)
    hub.publish(closed_stop_record(), topic="v3.capture_record")
    payload = FutureObservation(
        measurement_monotonic_ns=123, sequence=913, revision=7,
        lineage=("sensor:7", "world:4"), future_field=(None, 0.9, "completed"))
    topics = (
        "r2b4.hri", "r2b4.brain", "r2b4.agent", "r2b4.public_world", "r2b4.behavior",
        "r2b4.robot_interface", "r2b4.command", "r2b4.safety", "r2b4.health",
        "r2b4.actuator_output", "future.capability",
    )
    frames = [hub.publish(payload, topic=topic).frame for topic in topics]
    # Close before the consumer starts: accepted backlog must survive finalization.
    hub.close()
    result = worker.finish()
    assert result is not None and result.complete and not result.replay_complete
    assert gui.snapshot().superseded_count == len(topics)
    assert gui.snapshot().integrity_ok
    expected_payload = {
        "__type__": "FutureObservation",
        "measurement_monotonic_ns": 123, "sequence": 913, "revision": 7,
        "lineage": ["sensor:7", "world:4"], "future_field": [None, 0.9, "completed"],
    }
    reader = McapReader(result.path)
    reader.capture_integrity()
    recorded = [(message, row) for message, row in reader.iter_json_messages(topics=[EVENT_TOPIC])
                if row.get("source_topic") in topics]
    assert len(recorded) == len(frames)
    for (message, row), frame in zip(recorded, frames, strict=True):
        assert row == {
            "source_topic": frame.topic,
            "monotonic_ns": frame.published_monotonic_ns,
            "hub_sequence": frame.sequence,
            "published_monotonic_ns": frame.published_monotonic_ns,
            "payload": expected_payload,
        }
        assert message.log_time_ns == message.publish_time_ns == frame.published_monotonic_ns
        assert row["payload"]["measurement_monotonic_ns"] != row["published_monotonic_ns"]

    bundle = compile_evidence(result.path, tmp_path / "flight.evidence", workers=1)
    assert bundle["compiler_status"] == "COMPLETE"
    verify(bundle["output"], source=result.path)
    exported = list(query(bundle["output"], topic=EVENT_TOPIC, field="hub_sequence"))
    assert [row["payload"] for row in exported] == [row for _, row in recorded]
    normalized = [json.loads(line)["payload"]
                  for shard in sorted((tmp_path / "flight.evidence" / "normalized").glob("events_*.ndjson"))
                  for line in shard.read_text().splitlines()]
    assert [row for row in normalized if row.get("source_topic") in topics] == [row for _, row in recorded]


def test_required_delivery_loss_persists_as_incomplete_evidence(tmp_path):
    hub = ObservationHub()
    worker = consumer(hub, tmp_path / "loss.mcap", capacity=2)
    hub.publish(closed_stop_record(), topic="v3.capture_record")
    accepted = hub.publish(FutureObservation(123, 9, 4, (), ()), topic="future.capability").frame
    lost = hub.publish(FutureObservation(124, 10, 5, (), ()), topic="future.capability").frame
    hub.close()
    result = worker.finish()
    assert result is not None and not result.complete and result.ingress_drop_count == 1
    reader = McapReader(result.path)
    with pytest.raises(McapReadError, match="CAPTURE_INCOMPLETE"):
        reader.capture_integrity()
    rows = [row for _, row in reader.iter_json_messages(topics=[EVENT_TOPIC])]
    observation = next(row for row in rows if row.get("source_topic") == "future.capability")
    assert observation["hub_sequence"] == accepted.sequence
    final = next(row for row in rows if row.get("event_type") == "capture_finalized")
    integrity = final["integrity"]
    assert "OBSERVATION_RELIABLE_LOSS" in integrity["integrity_reasons"]
    assert integrity["observation_subscription"]["first_lost_sequence"] == lost.sequence
    assert integrity["observation_subscription"]["queued"] == 0


@pytest.mark.parametrize("required_loss", (False, True), ids=("exact-match", "required-loss"))
def test_full_native_flight_recorder_replays_and_rejects_required_loss(tmp_path, required_loss):
    from rig import ROOT, resolved_config
    from v3.composition.full_fake import OfflineMotorSink
    from v3.composition.native_control import NativeControlComposition
    from v3.contracts import CommandMode, CommandRequest, LifecycleState, RawDeviceBatch, TickContext
    from v3.engine import TickInputs
    from v3.execution import ExecutionRecord
    from v3.hri_evidence import HRI_EVENT_SCHEMA, HRI_EVENT_TOPIC
    from v3.mcap_replay_bridge import McapReplayBridgeError
    from v3.replay import replay_capture

    control = resolved_config().runtime.composition.live_control.control
    composition = NativeControlComposition(OfflineMotorSink(), control)
    hub = ObservationHub()
    worker = McapCaptureConsumer(
        "exact-flight-recorder", tmp_path / "exact-flight.mcap",
        subscription=hub.subscribe_reliable("capture", capacity=6, required=True),
        configuration={"production_control": control},
        config=McapCaptureConfig(mode="append_only", tick_sample_hz=50))
    try:
        for tick in range(4):
            context = TickContext(tick, 5_000_000_000 + tick * 20_000_000)
            inputs = composition.close_inputs(TickInputs(
                context, RawDeviceBatch(context, (), ()),
                CommandRequest(context, "exact-offline-stop", CommandMode.STOP, (), tick),
                LifecycleState.IDLE))
            result = composition.run_tick(inputs)
            assert result.final_actuation.left_output == result.final_actuation.right_output == 0.0
            hub.publish(ExecutionRecord(inputs, result), topic="v3.capture_record")
        hri = {
            "schema": HRI_EVENT_SCHEMA, "event_type": "USER_GOAL_ACCEPTED",
            "monotonic_ns": 5_030_000_000, "interaction_id": "interaction:3",
            "command_id": "exact-offline-stop", "world_revision": 8,
        }
        hri_frame = hub.publish(hri, topic=HRI_EVENT_TOPIC).frame
        future_frame = hub.publish(
            FutureObservation(5_025_000_000, 93, 8, ("interaction:3", "world:8"), (None,)),
            topic="future.capability").frame
        if required_loss:
            receipt = hub.publish(object(), topic="future.unaccepted")
            assert receipt.overrun_subscribers == ("capture",)
    finally:
        composition.close()
        hub.close()
        captured = worker.finish()

    assert captured is not None
    reader = McapReader(captured.path)
    rows = {row["source_topic"]: (message, row)
            for message, row in reader.iter_json_messages(topics=[EVENT_TOPIC])
            if "source_topic" in row}
    hri_message, hri_row = rows[HRI_EVENT_TOPIC]
    assert hri_row["payload"] == hri
    assert hri_message.log_time_ns == hri_row["monotonic_ns"] == hri["monotonic_ns"]
    assert hri_message.publish_time_ns == hri_row["published_monotonic_ns"] == hri_frame.published_monotonic_ns
    assert hri_row["hub_sequence"] == hri_frame.sequence
    future_message, future_row = rows["future.capability"]
    assert future_message.publish_time_ns == future_row["published_monotonic_ns"] == future_frame.published_monotonic_ns
    assert future_row["payload"]["measurement_monotonic_ns"] == 5_025_000_000
    assert future_row["payload"]["sequence"] == 93 and future_row["payload"]["revision"] == 8
    assert future_row["payload"]["lineage"] == ["interaction:3", "world:8"]
    if required_loss:
        assert not captured.replay_complete and captured.ingress_drop_count == 1
        with pytest.raises(McapReplayBridgeError, match="CAPTURE_INCOMPLETE"):
            replay_capture(captured.path, project_root=ROOT)
    else:
        assert captured.complete and captured.replay_complete
        replay = replay_capture(captured.path, project_root=ROOT)
        assert replay["status"] == "MATCH", replay["diagnostics"]
        assert replay["determinism"]["repeated_trace_match"]
        assert replay["mcap_bridge"]["authority_capture"] == str(captured.path)
        assert replay["mcap_bridge"]["materialized_tick_count"] == 4
        assert replay["mcap_bridge"]["authority_sha256"] == reader.sha256()
