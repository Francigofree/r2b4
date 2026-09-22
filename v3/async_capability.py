"""Small typed vocabulary for V3 async capability edges.

Not a scheduler, event bus, queue manager, or process orchestrator.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from v3.contracts.async_runtime import CompletionTiming, WorkerIdentity, source_is_stale


class TransportSemantics(str, Enum):
    LATEST_STATE = "LATEST_STATE"
    REQUEST_RESULT = "REQUEST_RESULT"
    EVIDENCE_STREAM = "EVIDENCE_STREAM"


class CapabilityState(str, Enum):
    STARTING = "STARTING"
    NO_DATA = "NO_DATA"
    FRESH = "FRESH"
    PENDING = "PENDING"
    STALE = "STALE"
    DEGRADED = "DEGRADED"
    FAILED = "FAILED"
    RESTARTING = "RESTARTING"
    STOPPED = "STOPPED"


@dataclass(frozen=True, slots=True)
class SensorIntegritySnapshot:
    """Passive raw integrity, separate from age and semantic measurement validity.

    Counters are source lifetime totals, never a freshness or safety authority.
    Capability-specific detail (e.g. left/right encoder deltas) stays at its edge.
    """

    source_sequence: int
    source_monotonic_ns: int
    timing_valid: bool
    measurement_valid: bool
    read_errors: int = 0
    invalid_alerts: int = 0
    quadrature_rejections: int = 0
    measurement_rejection: str | None = None

    def __post_init__(self) -> None:
        for value in (self.source_sequence, self.source_monotonic_ns,
                      self.read_errors, self.invalid_alerts, self.quadrature_rejections):
            if type(value) is not int or value < 0:
                raise ValueError("integrity counters/times must be non-negative integers")
        if type(self.timing_valid) is not bool or type(self.measurement_valid) is not bool:
            raise TypeError("integrity validity must be bool")


@dataclass(frozen=True, slots=True)
class CapabilityCounters:
    produced: int = 0
    accepted: int = 0
    superseded: int = 0
    stale: int = 0
    errors: int = 0
    restarts: int = 0
    late_rejected: int = 0
    deadline_missed: int = 0
    completed: int = 0

    def __post_init__(self) -> None:
        for value in (
            self.produced, self.accepted, self.superseded, self.stale,
            self.errors, self.restarts, self.late_rejected, self.deadline_missed, self.completed,
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError("capability counters must be non-negative integers")


@dataclass(frozen=True, slots=True)
class CapabilitySnapshot:
    name: str
    semantics: TransportSemantics
    state: CapabilityState
    observed_monotonic_ns: int
    source_sequence: int | None = None
    source_monotonic_ns: int | None = None
    pending_age_ns: int | None = None
    generation: int | None = None
    request_id: int | None = None
    error: str | None = None
    counters: CapabilityCounters = CapabilityCounters()

    @property
    def age_ns(self) -> int | None:
        if self.source_monotonic_ns is None:
            return None
        return max(0, self.observed_monotonic_ns - self.source_monotonic_ns)

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("capability name must be non-empty")
        if not isinstance(self.semantics, TransportSemantics):
            raise TypeError("semantics must be TransportSemantics")
        if not isinstance(self.state, CapabilityState):
            raise TypeError("state must be CapabilityState")
        if not isinstance(self.observed_monotonic_ns, int) or isinstance(self.observed_monotonic_ns, bool) or self.observed_monotonic_ns < 0:
            raise ValueError("observed_monotonic_ns must be non-negative")
        for value, name in (
            (self.source_sequence, "source_sequence"),
            (self.source_monotonic_ns, "source_monotonic_ns"),
            (self.pending_age_ns, "pending_age_ns"),
            (self.generation, "generation"),
            (self.request_id, "request_id"),
        ):
            if value is not None and (
                not isinstance(value, int) or isinstance(value, bool) or value < 0
            ):
                raise ValueError(f"{name} must be a non-negative integer or None")


def latest_state_snapshot(
    *,
    name: str,
    observed_monotonic_ns: int,
    source_sequence: int | None,
    source_monotonic_ns: int | None,
    stale_after_ns: int,
    running: bool,
    error: str | None = None,
    degraded: bool = False,
    stale: bool = False,
    timing_valid: bool = True,
    counters: CapabilityCounters = CapabilityCounters(),
) -> CapabilitySnapshot:
    if not isinstance(stale_after_ns, int) or isinstance(stale_after_ns, bool) or stale_after_ns <= 0:
        raise ValueError("stale_after_ns must be positive")
    if not timing_valid or (source_monotonic_ns is not None and source_monotonic_ns > observed_monotonic_ns):
        error = error or "SOURCE_TIME_INVALID"
    if error:
        state = CapabilityState.FAILED
    elif not running:
        state = CapabilityState.STOPPED
    elif source_sequence is None or source_monotonic_ns is None:
        state = CapabilityState.NO_DATA
    else:
        if stale or source_is_stale(observed_monotonic_ns, source_monotonic_ns, stale_after_ns):
            state = CapabilityState.STALE
        elif degraded:
            state = CapabilityState.DEGRADED
        else:
            state = CapabilityState.FRESH
    return CapabilitySnapshot(
        name=name,
        semantics=TransportSemantics.LATEST_STATE,
        state=state,
        observed_monotonic_ns=observed_monotonic_ns,
        source_sequence=source_sequence,
        source_monotonic_ns=source_monotonic_ns,
        error=error,
        counters=counters,
    )


def request_result_snapshot(
    *,
    name: str,
    observed_monotonic_ns: int,
    generation: int,
    pending_identity: WorkerIdentity | None,
    last_completed_request_id: int | None,
    last_completed_source_ns: int | None,
    error: str | None,
    running: bool,
    counters: CapabilityCounters,
    stale_after_ns: int = 350_000_000,
    pending_submit_ns: int | None = None,
) -> CapabilitySnapshot:
    if error:
        state = CapabilityState.FAILED
    elif not running:
        state = CapabilityState.STOPPED
    elif pending_identity is not None:
        state = CapabilityState.PENDING
    elif last_completed_source_ns is not None and source_is_stale(
        observed_monotonic_ns, last_completed_source_ns, stale_after_ns
    ):
        state = CapabilityState.STALE
    elif last_completed_request_id is not None:
        state = CapabilityState.FRESH
    else:
        state = CapabilityState.NO_DATA
    pending_age = None
    request_id = last_completed_request_id
    source_ns = last_completed_source_ns
    if pending_identity is not None:
        request_id = pending_identity.request_id
        source_ns = pending_identity.source_context.monotonic_ns
        pending_age = max(0, observed_monotonic_ns - (
            source_ns if pending_submit_ns is None else pending_submit_ns
        ))
    return CapabilitySnapshot(
        name=name,
        semantics=TransportSemantics.REQUEST_RESULT,
        state=state,
        observed_monotonic_ns=observed_monotonic_ns,
        source_monotonic_ns=source_ns,
        pending_age_ns=pending_age,
        generation=generation,
        request_id=request_id,
        error=error,
        counters=counters,
    )


__all__ = [
    "CapabilityCounters",
    "CapabilitySnapshot",
    "CapabilityState",
    "CompletionTiming",
    "TransportSemantics",
    "SensorIntegritySnapshot",
    "WorkerIdentity",
    "latest_state_snapshot",
    "request_result_snapshot",
    "source_is_stale",
]
