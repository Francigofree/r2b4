"""Immutable async identity and monotonic-time semantics; no runtime ownership."""
from __future__ import annotations

from dataclasses import dataclass
from .base import TickContext


@dataclass(frozen=True, slots=True)
class WorkerIdentity:
    generation: int
    request_id: int
    source_context: TickContext

    def __post_init__(self) -> None:
        if not isinstance(self.generation, int) or isinstance(self.generation, bool) or self.generation <= 0:
            raise ValueError("generation must be a positive integer")
        if not isinstance(self.request_id, int) or isinstance(self.request_id, bool) or self.request_id <= 0:
            raise ValueError("request_id must be a positive integer")
        if not isinstance(self.source_context, TickContext):
            raise TypeError("source_context must be TickContext")


@dataclass(frozen=True, slots=True)
class CompletionTiming:
    """Edge timestamps in one monotonic clock domain, independent of visibility."""

    submit_ns: int
    worker_start_ns: int
    worker_completed_ns: int
    collector_received_ns: int

    def __post_init__(self) -> None:
        times = (self.submit_ns, self.worker_start_ns, self.worker_completed_ns,
                 self.collector_received_ns)
        if any(type(value) is not int or value < 0 for value in times):
            raise ValueError("completion times must be non-negative integers")
        if tuple(sorted(times)) != times:
            raise ValueError("completion times must preserve causal order")

    def deadline_missed(self, timeout_ns: int) -> bool:
        if type(timeout_ns) is not int or timeout_ns <= 0:
            raise ValueError("timeout_ns must be positive")
        return self.worker_completed_ns - self.submit_ns > timeout_ns


def source_is_stale(observed_ns: int, source_ns: int, max_age_ns: int) -> bool:
    """Freshness never derives from receipt or input-closure publication time."""
    return observed_ns - source_ns > max_age_ns


