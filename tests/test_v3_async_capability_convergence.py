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
