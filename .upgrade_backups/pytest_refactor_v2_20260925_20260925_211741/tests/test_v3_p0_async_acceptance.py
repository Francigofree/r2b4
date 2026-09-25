from __future__ import annotations

import pytest

from tools.v3_p0_async_acceptance import (
    AcceptanceError,
    _capture_delta,
    _cpu_list,
    _idle_sample_errors,
    _terminal_report,
    _timing_view,
)


def _timing(**overrides):
    value = {
        "target_period_ns": 20_000_000,
        "tick_count": 500,
        "period_count": 499,
        "period_mean_ns": 20_000_000,
        "period_p50_ns": 20_000_000,
        "period_p95_ns": 21_000_000,
        "period_p99_ns": 22_000_000,
        "period_max_ns": 30_000_000,
        "period_over_25ms_count": 1,
        "period_over_40ms_count": 0,
        "lateness_p99_ns": 1_000_000,
        "lateness_max_ns": 2_000_000,
        "lateness_over_2ms_count": 0,
        "control_p99_ns": 10_000_000,
        "control_max_ns": 12_000_000,
        "observer_p99_ns": 500_000,
        "observer_max_ns": 800_000,
        "work_p99_ns": 11_000_000,
        "work_max_ns": 14_000_000,
        "work_over_period_count": 0,
    }
    value.update(overrides)
    return value


def _report(**timing_overrides):
    return {
        "status": "PASS",
        "termination_class": "SHUTDOWN_SAFE_LOW",
        "fault_layer": None,
        "timing": _timing(**timing_overrides),
    }


def test_idle_sample_accepts_only_safe_low_non_faulted_state():
    status = {
        "state": "RUNNING",
        "fault_layer": None,
        "safety_decision": "STOP",
        "enabled": False,
        "left_output": 0,
        "right_output": 0.0,
        "ready_for_active": True,
    }
    assert _idle_sample_errors(status) == ()


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("enabled", True),
        ("left_output", 0.01),
        ("right_output", -0.01),
        ("fault_layer", "L12"),
        ("safety_decision", "FAULT"),
        ("state", "ERROR"),
    ),
)
def test_idle_sample_rejects_motion_fault_or_nonrunning(field, value):
    status = {
        "state": "RUNNING",
        "fault_layer": None,
        "safety_decision": "STOP",
        "enabled": False,
        "left_output": 0,
        "right_output": 0,
    }
    status[field] = value
    assert _idle_sample_errors(status)


def test_terminal_report_requires_pass_safe_low_and_timing():
    payload = {"state": "STOPPED", "report": _report()}
    assert _terminal_report(payload)["termination_class"] == "SHUTDOWN_SAFE_LOW"

    with pytest.raises(AcceptanceError):
        _terminal_report({"state": "STOPPED", "report": _report(period_p99_ns=-1)})


def test_timing_view_rejects_negative_or_noninteger_fields():
    assert _timing_view(_report())["period_p99_ns"] == 22_000_000
    bad = _report()
    bad["timing"]["period_p99_ns"] = -1
    with pytest.raises(AcceptanceError):
        _timing_view(bad)


def test_capture_delta_reports_full_minus_off_without_inventing_a_new_gate():
    off = _report(period_p99_ns=20_000_000, observer_p99_ns=200_000)
    full = _report(period_p99_ns=22_000_000, observer_p99_ns=300_000)
    delta = _capture_delta(off, full)
    assert delta["period_p99_ns"]["delta_ns"] == 2_000_000
    assert delta["period_p99_ns"]["ratio"] == 1.1
    assert delta["observer_p99_ns"]["delta_ns"] == 100_000


def test_cpu_list_parses_linux_affinity_ranges():
    assert _cpu_list("0,2-3") == {0, 2, 3}
    assert _cpu_list("1") == {1}
