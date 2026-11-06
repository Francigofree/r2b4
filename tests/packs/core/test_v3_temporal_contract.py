from v3.contracts import TickContext
from v3.contracts.temporal import (
    UNKNOWN_AGE_NS,
    ControlContinuityState,
    bounded_deadline_ns,
    classify_control_continuity,
    deadline_reached,
    source_age_ns,
    source_is_stale,
)


def test_source_freshness_boundary_and_unknown_age():
    assert source_age_ns(100, 90) == 10
    assert source_age_ns(100, 110) == 0
    assert source_age_ns(100, None) == UNKNOWN_AGE_NS
    assert not source_is_stale(100, 90, 10)
    assert source_is_stale(101, 90, 10)
    assert source_is_stale(100, None, 10)


def test_deadline_boundary_and_finite_extension():
    assert not deadline_reached(119, 120)
    assert deadline_reached(120, 120)
    assert bounded_deadline_ns(100, 20) == 120
    assert bounded_deadline_ns(100, 20, extension_until_ns=110) == 120
    assert bounded_deadline_ns(100, 20, extension_until_ns=150) == 150


def test_control_continuity_classification_is_policy_free():
    previous = TickContext(5, 100)
    assert classify_control_continuity(None, previous, 20).state is ControlContinuityState.FIRST

    continuous = classify_control_continuity(previous, TickContext(6, 120), 20)
    assert continuous.state is ControlContinuityState.CONTINUOUS
    assert continuous.elapsed_ns == 20

    tick_gap = classify_control_continuity(previous, TickContext(7, 120), 20)
    assert tick_gap.state is ControlContinuityState.TICK_GAP

    time_gap = classify_control_continuity(previous, TickContext(6, 121), 20)
    assert time_gap.state is ControlContinuityState.TIME_GAP

    assert (
        classify_control_continuity(previous, TickContext(5, 120), 20).state
        is ControlContinuityState.NON_MONOTONIC
    )
    assert (
        classify_control_continuity(previous, TickContext(6, 99), 20).state
        is ControlContinuityState.NON_MONOTONIC
    )
