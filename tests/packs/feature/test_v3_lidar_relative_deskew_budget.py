"""Delayed relative-scan heading uses the measured scan's freshness budget."""

from dataclasses import replace
import queue
from types import SimpleNamespace

import pytest

from rig import resolved_config
from v3.adapters.native_lidar_port import NativeLidarPort, TimedPoseReference
from v3.adapters.rplidar_c1 import RplidarPoint, RplidarScan


START, MID, END = 2_000_000_000, 2_050_000_000, 2_100_000_000


def _relative_owner(input_budget_ns, result_budget_ns):
    scan = RplidarScan(7, END, START, END, MID, (
        RplidarPoint(0., 1., 15, START),
        RplidarPoint(90., 1., 15, MID),
        RplidarPoint(180., 1., 15, END),
    ))
    state = SimpleNamespace(now=END+10_000_000, heading_ready=False,
                            midpoint_ready=True, heading_reads=0, advance_on_heading=False)
    budget = min(input_budget_ns, result_budget_ns)

    def heading(timestamp):
        state.heading_reads += 1
        if timestamp == MID and not state.midpoint_ready:
            return None
        if timestamp == END:
            if not state.heading_ready:
                return None
            if state.advance_on_heading:
                state.now = MID+budget+1
        return TimedPoseReference(timestamp, 0., 0., (timestamp-MID)/1e9)

    class Driver:
        def start(self): return True
        def stop(self): pass
        def get_latest_scan(self): return scan
        def get_runtime_status(self): return {"connected": True}

    config = replace(resolved_config().lidar, global_backend="slam_toolbox",
                     maximum_input_age_ns=input_budget_ns,
                     maximum_result_age_ns=result_budget_ns)
    owner = NativeLidarPort(config, heading, driver=Driver(),
                            monotonic_ns=lambda: state.now)
    # The exact production pump with finite queues; no process or device starts.
    owner._input_queue, owner._result_queue = queue.Queue(1), queue.Queue(1)
    owner._running = True
    return owner, state, scan, budget


@pytest.mark.parametrize("input_budget,result_budget", (
    (150_000_000, 250_000_000), (250_000_000, 150_000_000),
))
def test_relative_heading_may_arrive_after_fixed_guard_until_inclusive_midpoint_budget(
        input_budget, result_budget):
    owner, state, scan, budget = _relative_owner(input_budget, result_budget)
    state.now = END+60_000_000
    owner.poll_once_for_test()
    assert owner._input_queue.empty()
    assert owner.get_runtime_status()["last_matcher_reason"] == "HEADING_BRACKET_UNAVAILABLE"
    physical = owner.get_raw_scan_snapshot()
    assert physical.health == "OK" and physical.raw_scan is scan.points
    assert physical.summary["minimum_clearance_m"] == 1.

    state.now, state.heading_ready = MID+budget, True
    owner.poll_once_for_test()
    packet = owner._input_queue.get_nowait()
    assert packet["source_scan_revision"] == scan.revision
    assert packet["measurement_monotonic_ns"] == MID
    assert packet["pose_reference_monotonic_ns"] == MID
    assert packet["captured_monotonic_ns"] == END
    assert packet["global_matching_enabled"] is False
    assert len(packet["scan"]) == len(scan.points)
    assert packet["scan"][0]["angle"] != scan.points[0].angle_deg
    assert owner.get_raw_scan_snapshot().raw_scan is scan.points
    owner.poll_once_for_test()
    assert owner._input_queue.empty()


@pytest.mark.parametrize("heading_ready", (False, True))
@pytest.mark.parametrize("input_budget,result_budget", (
    (150_000_000, 250_000_000), (250_000_000, 150_000_000),
))
def test_relative_expired_measurement_cannot_reach_worker_even_with_new_heading(
        heading_ready, input_budget, result_budget):
    owner, state, scan, budget = _relative_owner(input_budget, result_budget)
    owner.poll_once_for_test()
    assert owner._input_queue.empty()

    state.now, state.heading_ready = MID+budget+1, heading_ready
    owner.poll_once_for_test()
    assert owner._input_queue.empty()
    assert owner.get_runtime_status()["last_matcher_reason"] == "HEADING_BRACKET_TIMEOUT"
    assert owner.get_raw_scan_snapshot().raw_scan is scan.points
    assert owner.get_raw_scan_snapshot().health == "OK"

    state.heading_ready = True
    state.now += 1
    owner.poll_once_for_test()
    assert owner._input_queue.empty()
    assert owner.get_relative_result() is None


def test_relative_measurement_expiring_while_heading_is_read_cannot_reach_worker():
    owner, state, _, budget = _relative_owner(150_000_000, 250_000_000)
    state.now, state.heading_ready = MID+budget, True
    state.advance_on_heading = True
    owner.poll_once_for_test()
    assert owner._input_queue.empty()
    assert owner.get_runtime_status()["last_matcher_reason"] == "HEADING_BRACKET_TIMEOUT"


@pytest.mark.parametrize("input_budget,result_budget", (
    (150_000_000, 250_000_000), (250_000_000, 150_000_000),
))
def test_relative_missing_midpoint_reference_expires_without_retrying_consumed_scan(
        input_budget, result_budget):
    owner, state, scan, budget = _relative_owner(input_budget, result_budget)
    state.midpoint_ready = False
    state.now = MID+budget
    owner.poll_once_for_test()
    assert owner._input_queue.empty()
    assert owner.get_runtime_status()["last_matcher_reason"] == "POSE_REFERENCE_UNAVAILABLE"
    assert owner.get_raw_scan_snapshot().raw_scan is scan.points
    assert owner.get_raw_scan_snapshot().health == "OK"

    state.now += 1
    owner.poll_once_for_test()
    assert owner._input_queue.empty()
    assert owner.get_runtime_status()["last_matcher_reason"] == "HEADING_BRACKET_TIMEOUT"
    reads_after_expiry = state.heading_reads
    state.midpoint_ready = state.heading_ready = True
    state.now += 1
    owner.poll_once_for_test()
    assert owner._input_queue.empty()
    assert state.heading_reads == reads_after_expiry
    assert owner.get_raw_scan_snapshot().raw_scan is scan.points
