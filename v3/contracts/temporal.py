"""Shared monotonic-time primitives for V3 contracts.

This module owns time semantics only.  It deliberately owns no retry, motion,
safety, navigation, process-lifecycle or fault policy.  Callers keep authority
for the reaction to stale evidence, expired deadlines and control discontinuity.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .base import TickContext


UNKNOWN_AGE_NS = 2**63 - 1


def _non_negative_int(value: int, name: str) -> int:
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _positive_int(value: int, name: str) -> int:
    if type(value) is not int or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def source_age_ns(observed_ns: int, source_ns: int | None) -> int:
    """Return non-negative source age; missing evidence has sentinel max age.

    Future-source validity is intentionally not decided here.  Existing owners
    that care about clock causality already reject future timestamps explicitly.
    Clamping to zero preserves the diagnostic age semantics used by capability
    snapshots and localization quality evidence.
    """

    observed = _non_negative_int(observed_ns, "observed_ns")
    if source_ns is None:
        return UNKNOWN_AGE_NS
    source = _non_negative_int(source_ns, "source_ns")
    return max(0, observed - source)


def source_is_stale(
    observed_ns: int,
    source_ns: int | None,
    max_age_ns: int,
) -> bool:
    """Fresh at the exact age boundary; stale only when age is greater."""

    maximum = _non_negative_int(max_age_ns, "max_age_ns")
    return source_age_ns(observed_ns, source_ns) > maximum


def deadline_reached(now_ns: int, deadline_ns: int) -> bool:
    """A deadline expires at the deadline instant, not one nanosecond later."""

    now = _non_negative_int(now_ns, "now_ns")
    deadline = _non_negative_int(deadline_ns, "deadline_ns")
    return now >= deadline


def bounded_deadline_ns(
    started_ns: int,
    budget_ns: int,
    *,
    extension_until_ns: int | None = None,
) -> int:
    """Return one finite deadline with an optional owner-authorized extension."""

    started = _non_negative_int(started_ns, "started_ns")
    budget = _positive_int(budget_ns, "budget_ns")
    deadline = started + budget
    if extension_until_ns is not None:
        extension = _non_negative_int(extension_until_ns, "extension_until_ns")
        deadline = max(deadline, extension)
    return deadline


class ControlContinuityState(str, Enum):
    FIRST = "FIRST"
    CONTINUOUS = "CONTINUOUS"
    TICK_GAP = "TICK_GAP"
    TIME_GAP = "TIME_GAP"
    NON_MONOTONIC = "NON_MONOTONIC"


@dataclass(frozen=True, slots=True)
class ControlContinuity:
    state: ControlContinuityState
    elapsed_ns: int | None

    @property
    def continuous(self) -> bool:
        return self.state is ControlContinuityState.CONTINUOUS


def classify_control_continuity(
    previous: TickContext | None,
    current: TickContext,
    max_gap_ns: int,
) -> ControlContinuity:
    """Classify tick/time continuity without deciding the layer reaction."""

    if previous is not None and not isinstance(previous, TickContext):
        raise TypeError("previous must be TickContext or None")
    if not isinstance(current, TickContext):
        raise TypeError("current must be TickContext")
    maximum = _positive_int(max_gap_ns, "max_gap_ns")
    if previous is None:
        return ControlContinuity(ControlContinuityState.FIRST, None)

    elapsed = current.monotonic_ns - previous.monotonic_ns
    if current.tick_id <= previous.tick_id or elapsed <= 0:
        return ControlContinuity(ControlContinuityState.NON_MONOTONIC, None)
    if current.tick_id != previous.tick_id + 1:
        return ControlContinuity(ControlContinuityState.TICK_GAP, elapsed)
    if elapsed > maximum:
        return ControlContinuity(ControlContinuityState.TIME_GAP, elapsed)
    return ControlContinuity(ControlContinuityState.CONTINUOUS, elapsed)


__all__ = [
    "UNKNOWN_AGE_NS",
    "ControlContinuity",
    "ControlContinuityState",
    "bounded_deadline_ns",
    "classify_control_continuity",
    "deadline_reached",
    "source_age_ns",
    "source_is_stale",
]
