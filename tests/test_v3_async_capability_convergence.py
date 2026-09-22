from v3.async_capability import (
    CapabilityCounters, CapabilityState, TransportSemantics, WorkerIdentity,
    latest_state_snapshot, request_result_snapshot,
)
from v3.contracts import TickContext


def test_latest_state_fresh_stale_and_failed_are_distinct():
    fresh = latest_state_snapshot(
        name="lidar", observed_monotonic_ns=200, source_sequence=7,
        source_monotonic_ns=150, stale_after_ns=100, running=True,
    )
    stale = latest_state_snapshot(
        name="lidar", observed_monotonic_ns=300, source_sequence=7,
        source_monotonic_ns=150, stale_after_ns=100, running=True,
    )
    failed = latest_state_snapshot(
        name="lidar", observed_monotonic_ns=300, source_sequence=7,
        source_monotonic_ns=150, stale_after_ns=100, running=False,
        error="OWNER_EXITED",
    )
    assert fresh.state is CapabilityState.FRESH
    assert stale.state is CapabilityState.STALE
    assert failed.state is CapabilityState.FAILED
    assert fresh.semantics is TransportSemantics.LATEST_STATE


def test_request_identity_is_generation_request_and_source_context():
    identity = WorkerIdentity(3, 41, TickContext(9, 900))
    assert identity.generation == 3
    assert identity.request_id == 41
    assert identity.source_context == TickContext(9, 900)


def test_pending_request_is_not_failure():
    identity = WorkerIdentity(2, 8, TickContext(5, 500))
    snapshot = request_result_snapshot(
        name="planner", observed_monotonic_ns=650, generation=2,
        pending_identity=identity, last_completed_request_id=7,
        last_completed_source_ns=400, error=None, running=True,
        counters=CapabilityCounters(produced=8, accepted=7),
    )
    assert snapshot.state is CapabilityState.PENDING
    assert snapshot.pending_age_ns == 150
    assert snapshot.error is None
    assert snapshot.semantics is TransportSemantics.REQUEST_RESULT


def test_request_result_ages_from_source_instead_of_completion():
    snapshot = request_result_snapshot(
        name="planner", observed_monotonic_ns=1000, generation=1,
        pending_identity=None, last_completed_request_id=1,
        last_completed_source_ns=500, error=None, running=True,
        counters=CapabilityCounters(), stale_after_ns=100,
    )
    assert snapshot.state is CapabilityState.STALE
    assert snapshot.age_ns == 500


def test_future_latest_measurement_is_failed_not_fresh():
    snapshot = latest_state_snapshot(
        name="imu", observed_monotonic_ns=100, source_sequence=1,
        source_monotonic_ns=101, stale_after_ns=100, running=True,
    )
    assert snapshot.state is CapabilityState.FAILED


def test_encoder_integrity_separates_quadrature_timing_and_semantic_validity():
    from dataclasses import replace
    from types import SimpleNamespace
    from test_v3_counter_encoder_backend import _backend, _snapshot
    from v3.adapters.live_encoder import NativeEncoderConfig, NativeEncoderSource

    backend, _, _ = _backend((_snapshot(100),), (_snapshot(200),))
    context = TickContext(1, 1_000_000_000)
    reading = backend.read(context)
    reading = replace(reading, diagnostics=replace(reading.diagnostics, left_quadrature_rejections=1638))
    source = NativeEncoderSource(SimpleNamespace(read=lambda _: reading), NativeEncoderConfig("enc", .3))
    source.read(context)
    integrity = source.integrity_snapshot()
    assert integrity.quadrature_rejections == 1638
    assert integrity.read_errors == 0
    assert integrity.timing_valid
    assert not integrity.measurement_valid
    assert integrity.measurement_rejection == "BASELINE"
    assert source.capability_snapshot(context.monotonic_ns).state is CapabilityState.DEGRADED
    assert source.capability_snapshot(context.monotonic_ns + 300_000_000).state is CapabilityState.STALE


def test_encoder_latest_state_burst_preserves_newest_and_direct_semantics():
    import queue
    from dataclasses import replace
    from types import SimpleNamespace
    from test_v3_live_encoder_edge import _reading
    from v3.adapters.live_encoder import NativeEncoderConfig, NativeEncoderSource
    from v3.adapters.process_encoder_backend import ProcessEncoderBackend

    proxy = ProcessEncoderBackend.__new__(ProcessEncoderBackend)
    proxy._queue = queue.Queue(2)
    proxy._latest = None
    proxy._fatal_error = ""
    proxy._closed = False
    proxy._process = SimpleNamespace(is_alive=lambda: True)
    newest = replace(_reading(), sequence=14)
    proxy._queue.put(("reading", newest))
    proxy._queue.put(("reading", _reading()))
    config = NativeEncoderConfig("enc", .3)
    context = TickContext(1, 1_000)
    process_source = NativeEncoderSource(proxy, config)
    direct_source = NativeEncoderSource(SimpleNamespace(read=lambda _: newest), config)
    assert process_source.read(context) == direct_source.read(context)
    assert process_source.capability_snapshot(1000) == direct_source.capability_snapshot(1000)
    proxy._queue.put(("error", "OSError", "failed"))
    proxy._queue.put(("reading", replace(newest, sequence=15)))
    import pytest
    with pytest.raises(RuntimeError, match="ENCODER_PROCESS_FAILED"):
        process_source.read(context)
    assert process_source.capability_snapshot(1000).state is CapabilityState.FAILED


def test_sensor_stale_flag_is_rejected_even_when_receipt_age_is_fresh():
    from v3.contracts import AcquisitionFrame, DataField, DeviceHealth, DeviceHealthState, DeviceSample, RejectionReason
    from v3.layers.l2_admission import AdmissionConfig, InputAdmission

    context = TickContext(1, 1_000)
    for kind in ("wheel_velocity", "ekf_heading", "person_detection", "lidar_pose"):
        frame = AcquisitionFrame(context, (DeviceSample("sensor", kind, 1, 990,
            (DataField("measurement_stale", True),)),), (DeviceHealth("sensor", DeviceHealthState.OK),))
        admitted = InputAdmission(AdmissionConfig(max_sample_age_ns=100))(frame)
        assert not admitted.accepted
        assert admitted.rejected[0].reason is RejectionReason.STALE
