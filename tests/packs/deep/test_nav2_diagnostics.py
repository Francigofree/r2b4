"""Passive ROS evidence keeps source lineage and distinguishes rates from stops."""
from dataclasses import replace
import json
import math
import time
from types import SimpleNamespace as NS

from v3.capture_behavior import encode_behavior_record, project_behavior_record
from v3.contracts import GLOBAL_FRAME_ID
from v3.mcap_capture import EVENT_TOPIC, TICK_TOPIC, McapCaptureConfig
from v3.mcap_reader import McapReader
from v3.nav2_diagnostics import (
    NAV2_OBSERVATION_SCHEMA, NAV2_OBSERVATION_TOPIC, Nav2ObservationSampler, Nav2Telemetry,
)
from v3.process_sidecars import ProcessMcapCaptureSession
from v3.runtime_performance import RuntimeTimingAccumulator


def command(v=.1, omega=0.):
    return NS(twist=NS(linear=NS(x=v), angular=NS(z=omega)))


def observation(helper, now, *, tf_ns=None, pose=(0., 0., 0.)):
    return helper.snapshot(now, scan_ns=now, odom_ns=now,
        tf_ns=now if tf_ns is None else tf_ns, active=True, robot_pose=pose,
        state={"state": "RUNNING", "mission_id": "test", "selected_goal": [1., 0., None]})


def helper():
    return Nav2Telemetry(10, 0, body_bounds_m=(-.10, .26, -.18, .18), lidar_xy_m=(-.005, 0.))


def test_actual_twenty_hz_stream_distinguishes_receipt_jitter_and_missing_commands():
    telemetry = helper()
    for index in range(41):
        stamp = 1_000_000_000+index*50_000_000
        telemetry.observe_command(command(), stamp, stamp+(10_000_000 if index % 2 else 0))
    row = observation(telemetry, 3_000_000_000)["controller"]
    assert row["source_hz"] == 20.
    assert row["source_max_gap_ns"] == 50_000_000
    assert row["receipt_max_gap_ns"] == 60_000_000
    telemetry.observe_command(command(v=0.), 3_500_000_000, 3_500_000_000)
    row = observation(telemetry, 3_500_000_000)["controller"]
    assert row["source_max_gap_ns"] == 500_000_000 and row["stop_count"] == 1
    row = observation(telemetry, 6_000_000_000)["controller"]
    assert row["source_hz"] is None and row["last_command_age_ns"] == 2_500_000_000


def test_body_clearance_and_coverage_preserve_geometry_measurement_and_stale_state():
    telemetry = helper()
    # Front body edge is x=.26, scanner x=-.005, return x=.36: .10 m clearance.
    scan = NS(ranges=[.365, .28], angle_min=0., angle_increment=math.pi/2,
        range_min=.08, range_max=8.)
    telemetry.observe_scan(scan, 1_000_000_000)
    first = observation(telemetry, 1_000_000_000)
    assert math.isclose(first["wall_clearance"]["front_m"], .1)
    assert first["wall_clearance"]["measurement_monotonic_ns"] == 1_000_000_000
    first_area = first["coverage"]["footprint_raster_area_m2"]
    assert first_area > 0 and first["coverage"]["frame_id"] == GLOBAL_FRAME_ID
    stale = observation(telemetry, 2_000_000_000, tf_ns=1_000_000_000, pose=(2., 2., 0.))
    assert stale["coverage"]["footprint_raster_area_m2"] == first_area
    assert stale["coverage"]["distance_m"] == 0. and not stale["coverage"]["available"]
    duplicate = observation(telemetry, 1_100_000_000, tf_ns=1_000_000_000, pose=(.1, 0., 0.))
    assert duplicate["coverage"]["footprint_raster_area_m2"] == first_area


def test_ros_uint8_diagnostics_and_errors_are_bounded_and_keep_original_time():
    telemetry = helper()
    telemetry.observe_diagnostics(NS(header=NS(stamp=NS(sec=1, nanosec=5)),
        status=[NS(name="controller", level=b"\x02", message="failure", hardware_id="nav2",
            values=[NS(key="detail", value="a"*1000)]*20)]), 1_100_000_000)
    for index in range(20):
        telemetry.record_error(1_100_000_000+index, "NAVIGATION_FAILED", index, "message"*50, index)
    row = observation(telemetry, 1_200_000_000)
    diagnostic = row["diagnostics"][0]
    assert diagnostic["level"] == 2 and diagnostic["measurement_monotonic_ns"] == 1_000_000_005
    assert diagnostic["source_age_ns"] == 199_999_995
    assert len(diagnostic["values"]) == 8 and row["diagnostics_truncated_count"] == 12
    assert row["navigation_error_count"] == 20 and len(row["navigation_errors"]) == 8
    assert len(json.dumps(row)) < 8192


def test_snapshot_reader_reports_stale_and_superseded_without_renewing_source(tmp_path):
    sampler = Nav2ObservationSampler(tmp_path)
    sampler.path.parent.mkdir(parents=True)
    def write(sequence, stamp, generation=10):
        temporary = sampler.path.with_suffix(".tmp")
        temporary.write_text(json.dumps({"nav2_observation": {
            "schema": NAV2_OBSERVATION_SCHEMA, "sample_sequence": sequence,
            "worker_generation": generation, "measurement_monotonic_ns": stamp}}))
        temporary.replace(sampler.path)
    write(1, 1_000_000_000)
    first = sampler.sample(1_100_000_000)
    assert first["measurement_monotonic_ns"] == 1_000_000_000
    assert first["observer_received_monotonic_ns"] == 1_100_000_000
    write(4, 1_200_000_000)
    assert sampler.sample(1_300_000_000)["snapshots_superseded"] == 2
    assert sampler.sample(2_300_000_001)["event_type"] == "nav2_observation_unavailable"
    assert sampler.sample(2_400_000_000) is None
    write(5, 2_500_000_000)
    recovered = sampler.sample(2_600_000_000)
    assert recovered["available"] and recovered["recovered_after_unavailable"]
    assert recovered["measurement_monotonic_ns"] == 2_500_000_000
    assert recovered["sample_sequence"] == 5 and recovered["worker_generation"] == 10
    assert sampler.sample(2_700_000_000) is None
    assert sampler.sample(3_600_000_000)["event_type"] == "nav2_observation_unavailable"
    assert sampler.sample(3_700_000_000) is None
    write(1, 3_800_000_000, generation=20)
    restarted = sampler.sample(3_900_000_000)
    assert restarted["available"] and restarted["recovered_after_unavailable"]
    assert restarted["worker_generation"] == 20 and restarted["sample_sequence"] == 1
    assert restarted["snapshots_superseded"] == 0
    assert sampler.sample(4_000_000_000) is None


def test_five_hz_timing_window_keeps_unsampled_cycle_delay_and_distinct_pipeline_wall():
    from test_flight_recorder import closed_stop_record
    timing = RuntimeTimingAccumulator(20_000_000)
    stamp = 1_000_000_000
    for index in range(10):
        stamp += 60_000_000 if index == 5 else 20_000_000
        timing.observe_tick_start(stamp, stamp-1_000_000)
        timing.observe_control(5_000_000)
        timing.observe_control_phase("PIPELINE_TOTAL", 1_000_000)
    window = timing.take_capture_window()
    assert window.tick_count == 10 and window.period_max_ns == 60_000_000
    assert window.control_wall_max_ns == 5_000_000 and window.pipeline_wall_max_ns == 1_000_000
    record = replace(closed_stop_record(), evidence=(window,))
    encoded = encode_behavior_record(project_behavior_record(record))
    assert encoded["tick_evidence"][0]["period_max_ns"] == 60_000_000
    assert timing.take_capture_window() is None
    assert timing.snapshot().period_max_ns == 60_000_000


def test_nav2_producer_snapshot_reaches_real_mcap_without_control_payload(tmp_path):
    from test_flight_recorder import closed_stop_record
    session = ProcessMcapCaptureSession("nav2-diag", tmp_path/"nav2.mcap", configuration={},
        project_root=tmp_path, config=McapCaptureConfig(mode="append_only", tick_sample_hz=5))
    session.start()
    try:
        stamp = time.monotonic_ns()
        telemetry = observation(helper(), stamp)
        path = tmp_path/"runtime"/"nav2_planner"/"bridge_status.json"
        path.parent.mkdir(parents=True)
        path.write_text(json.dumps({"nav2_observation": telemetry}))
        session.observe(closed_stop_record())
        time.sleep(.08)  # One metadata poll, independent of control queue delivery.
    finally:
        result = session.finalize(NS(status=0))
    reader = McapReader(result)
    assert reader.capture_integrity()["integrity"]["complete"]
    rows = [row for _, row in reader.iter_json_messages(topics=[EVENT_TOPIC])
        if row.get("source_topic") == NAV2_OBSERVATION_TOPIC]
    assert len(rows) == 1 and rows[0]["payload"]["measurement_monotonic_ns"] == stamp
    ticks = [row for _, row in reader.iter_json_messages(topics=[TICK_TOPIC])]
    assert len(ticks) == 1 and NAV2_OBSERVATION_SCHEMA not in json.dumps(ticks)
