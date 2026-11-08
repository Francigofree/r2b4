"""A rotating physical scan stays sharp in both local and SLAM geometries."""
from dataclasses import replace
import math
import queue
import sys
import threading
from types import SimpleNamespace

import pytest

from v3.adapters.native_lidar_port import (
    NativeRawLidarSnapshot, TimedPoseReference, rotation_deskew_points, _sector_summary,
)
from v3.adapters.process_lidar_port import _wire_control_raw, _wire_raw, _unwire_raw
from v3.adapters.rplidar_c1 import RplidarPoint, RplidarScan
from v3.adapters.slam_toolbox_transport import encode_scan, decode_packet
from v3.capture_encoding import encode_raw_lidar_snapshot
from v3.lidar_geometry import lidar_point_xy


START, MID, END = 2_000_000_000, 2_050_000_000, 2_100_000_000
OFFSET = (.08, -.03)


def _heading(timestamp):
    # 1.2 rad/s in-place turn, crossing the +/-pi representation boundary.
    yaw = math.pi + 1.2*(timestamp-MID)/1e9
    return TimedPoseReference(timestamp, 0., 0., math.atan2(math.sin(yaw), math.cos(yaw)))


def _spinning_room_scan(revision=7):
    points = []
    for index in range(72):
        angle = index * 5.
        timestamp = START + index*(END-START)//72
        # Express the static rectangular room in the midpoint robot frame.
        yaw = 1.2*(timestamp-MID)/1e9
        c, s = math.cos(yaw), math.sin(yaw)
        laser_x = c*OFFSET[0]-s*OFFSET[1]
        laser_y = s*OFFSET[0]+c*OFFSET[1]
        direction = yaw-math.radians(angle)
        dx, dy = math.cos(direction), math.sin(direction)
        lengths = [(wall-origin)/velocity
                   for origin, velocity, walls in ((laser_x, dx, (-2., 2.)),
                                                   (laser_y, dy, (-1.5, 1.5)))
                   if abs(velocity) > 1e-12 for wall in walls
                   if (wall-origin)/velocity > 0]
        points.append(RplidarPoint(angle, min(lengths), 15, timestamp))
    physical = RplidarScan(revision, END, START, END, MID, tuple(points))
    return NativeRawLidarSnapshot(revision, END/1e9, START, END, MID, "OK", tuple(points),
                                  _sector_summary(physical), END)


def _deskew(raw, provider=_heading):
    return rotation_deskew_points(raw.raw_scan, scan_start_ns=START, scan_end_ns=END,
        measurement_ns=MID, pose_provider=provider, lidar_x_m=OFFSET[0], lidar_y_m=OFFSET[1])


def _wall_error(points):
    return max(min(abs(abs(x)-2.), abs(abs(y)-1.5)) for x, y in (
        lidar_point_xy(math.radians(point.angle_deg), point.distance_m, *OFFSET)
        for point in points))


def test_rotating_room_wall_error_is_removed_in_local_and_slam_scan_geometry():
    raw = _spinning_room_scan()
    original = raw.raw_scan
    corrected, reason = _deskew(raw)
    assert reason == "APPLIED"
    assert _wall_error(original) > .04
    assert _wall_error(corrected) < 1e-12
    assert raw.raw_scan is original
    geometry = replace(raw, raw_scan=corrected)
    compact = _wire_control_raw(geometry, minimum_range_m=.08, maximum_range_m=6., maximum_points=96)
    local = _unwire_raw(compact)
    assert _wall_error(local.raw_scan) < 1e-12
    assert local.measurement_monotonic_ns == MID
    packet = decode_packet(encode_scan(geometry, session_id=11, source_pid=22,
        minimum_range_m=.08, maximum_range_m=6., lidar_x_m=OFFSET[0], lidar_y_m=OFFSET[1]))
    slam_points = tuple(RplidarPoint((-index*360/len(packet.ranges_m)) % 360, value, 15)
                        for index, value in enumerate(packet.ranges_m) if value)
    # The ROS scan additionally quantizes the bearing into half-degree bins.
    assert _wall_error(slam_points) < .015
    assert packet.measurement_ns == MID


@pytest.mark.parametrize("missing", (START, MID, END-((END-START)//72)))
def test_missing_heading_bracket_keeps_physical_returns_without_extrapolation(missing):
    raw = _spinning_room_scan()
    # Use the actual last return timestamp (integer scan spacing).
    if missing > MID:
        missing = raw.raw_scan[-1].received_monotonic_ns
    points, reason = _deskew(raw, lambda timestamp: None if timestamp == missing else _heading(timestamp))
    assert points is raw.raw_scan
    assert reason == "HEADING_BRACKET_UNAVAILABLE"


def test_legacy_untimed_scan_and_discontinuous_heading_are_explicit_uncompensated():
    raw = _spinning_room_scan()
    untimed = replace(raw, raw_scan=tuple(replace(point, received_monotonic_ns=None) for point in raw.raw_scan))
    assert _deskew(untimed) == (untimed.raw_scan, "POINT_TIMING_UNAVAILABLE")
    points, reason = _deskew(raw, lambda timestamp: TimedPoseReference(
        timestamp, 0., 0., 0. if timestamp == MID else 1.))
    assert points is raw.raw_scan
    assert reason == "HEADING_DISCONTINUITY"


def test_raw_evidence_retains_receipt_times_without_changing_measurement_identity():
    raw = _spinning_room_scan()
    assert _unwire_raw(_wire_raw(raw)) == raw
    encoded = encode_raw_lidar_snapshot(raw, 100)
    assert encoded["point_received_monotonic_ns"] == [point.received_monotonic_ns for point in raw.raw_scan]
    assert encoded["measurement_monotonic_ns"] == MID
    assert encoded["point_encoding"] == "ANGLE_DEG_DISTANCE_M_QUALITY"


@pytest.mark.parametrize("outcome", ("late_ready", "expired_missing", "expired_ready",
                                     "expired_during_lookup", "superseded"))
def test_production_process_publishes_safety_before_bracket_then_completes_or_expires(monkeypatch, outcome):
    import v3.adapters.process_lidar_port as owner
    from rig import resolved_config
    resolved = resolved_config()
    raw = _spinning_room_scan(774)
    if outcome == "superseded":
        shift_ns = 100_000_000
        points = tuple(replace(point, received_monotonic_ns=point.received_monotonic_ns+shift_ns)
                       for point in raw.raw_scan)
        physical = RplidarScan(775, END+shift_ns, START+shift_ns, END+shift_ns, MID+shift_ns, points)
        replacement = NativeRawLidarSnapshot(775, (END+shift_ns)/1e9,
            START+shift_ns, END+shift_ns, MID+shift_ns, "OK", points,
            _sector_summary(physical), END+shift_ns)
    stop = threading.Event()
    class History:
        attempted_at = []
        def __init__(self, *args): pass
        def lookup(self, timestamp):
            self.attempted_at.append(owner.time.monotonic_ns())
            if (outcome == "expired_during_lookup" and port.calls >= 3
                    and timestamp == raw.raw_scan[-1].received_monotonic_ns):
                port.lookup_advanced = True
            return _heading(timestamp) if port.calls >= 3 and outcome != "expired_missing" else None
    class Port:
        calls = 0
        lookup_advanced = False
        def get_raw_scan_snapshot(self):
            self.calls += 1
            if self.calls == 3:
                stop.set()
            return replacement if outcome == "superseded" and self.calls >= 3 else raw
        def get_matcher_result(self): return None
        def get_relative_result(self): return None
        def get_runtime_status(self):
            return dict(running=True, health="OK", driver_connected=True, matcher_process_alive=True)
        def stop(self): pass
    exported = []
    class Sender:
        sent_packets, dropped_packets = 0, 0
        def __init__(self, *args, **kw): pass
        def publish_scan(self, snapshot, **kw):
            exported.append(snapshot)
            return True
        def receive(self): return ()
        def close(self): pass
    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace(Serial=None))
    monkeypatch.setattr(owner, "_SharedPoseHistory", History)
    monkeypatch.setattr(owner, "SlamToolboxSender", Sender)
    port = Port()
    monkeypatch.setattr(owner, "open_native_lidar_port", lambda *a, **kw: port)
    output, evidence = queue.Queue(8), queue.Queue(8)
    config = replace(resolved.lidar, global_backend="slam_toolbox", matcher=replace(
        resolved.lidar.matcher, lidar_x_offset_m=OFFSET[0], lidar_y_offset_m=OFFSET[1]))
    maximum_age_ns = min(config.maximum_input_age_ns, config.maximum_result_age_ns)
    final_ns = (END+80_000_000 if outcome in {"late_ready", "expired_during_lookup"} else END+120_000_000
                if outcome == "superseded" else MID+maximum_age_ns+1)
    monkeypatch.setattr(owner.time, "monotonic_ns", lambda: (
        MID+maximum_age_ns+1 if port.lookup_advanced else END+10_000_000
        if port.calls == 1 else END+60_000_000 if port.calls == 2 else final_ns))
    owner._lidar_owner_process_main(config, None, None, None, None, output, evidence,
        threading.Event(), stop, None, None, False, .08, 6., 96,
        resolved.edges.lidar_process, capture_map_hz=5, slam_session_id=11)
    initial, ready = output.get_nowait(), output.get_nowait()
    # The bracket is still absent on a retry after the former 40 ms deadline.
    # Becoming ready on the *first* attempt after 40 ms would miss that bug.
    assert END+60_000_000 in History.attempted_at
    assert initial[1][0] == raw.raw_scan_id
    assert initial[1][7]["local_geometry_pending"] is True
    assert initial[1][6] == ()
    if outcome.startswith("expired"):
        assert ready[1][0] == raw.raw_scan_id
        assert ready[1][7]["local_geometry_pending"] is True
        assert ready[1][6] == ()
        assert ready[3]["rotation_deskew"] == "HEADING_BRACKET_TIMEOUT"
        assert exported == []
        assert evidence.get_nowait()[0] == "raw_end"
        return
    assert ready[1][7]["local_geometry_pending"] is False
    assert _wall_error(_unwire_raw(ready[1]).raw_scan) < 1e-12
    assert ready[3]["rotation_deskew_applied_scans"] == 1
    assert ready[3]["rotation_deskew_time_basis"] == "HOST_PACKET_RECEIPT"
    assert len(exported) == 1
    assert _wall_error(exported[0].raw_scan) < 1e-12
    observation = evidence.get_nowait()
    assert observation[0] == "navigation_map"
    assert observation[1]["rotation_deskew"] == "APPLIED"
    assert observation[1]["measurement_monotonic_ns"] == (replacement.measurement_monotonic_ns
        if outcome == "superseded" else MID)
    if outcome == "superseded":
        assert ready[1][0] == replacement.raw_scan_id
        assert exported[0].raw_scan_id == replacement.raw_scan_id
        assert output.empty()
        return
    assert ready[1][0] == raw.raw_scan_id

    # Exercise the actual receiving collector cache too: a producer-only
    # assertion misses a pending->ready update discarded under the same scan ID.
    from rig import healthy_localization
    from v3.adapters.latest_lidar import (
        LatestLidarBackendConfig, NativeLatestLidarBackend, SLAM_CONTRACT_ID,
        SLAM_CONFIDENCE_MODEL, SLAM_TRANSPORT,
    )
    from v3.adapters.live_lidar import NativeLidarConfig, NativeLidarSource
    from v3.contracts import (
        AcquisitionFrame, AdmittedFrame, DataField, LOCAL_FRAME_ID, Observation,
        RejectionReason, RobotEstimate, TickContext,
    )
    from v3.layers.l2_admission import AdmissionConfig, InputAdmission
    from v3.layers.l4_world_model import ShadowWorldModel
    collector = owner.ProcessLidarPort.__new__(owner.ProcessLidarPort)
    collector._state_queue = queue.Queue(8)
    collector._process_config = resolved.edges.lidar_process
    collector._process = SimpleNamespace(is_alive=lambda: True)
    collector._stopped, collector._fatal_error = False, None
    collector._raw_snapshot = collector._matcher_result = collector._relative_result = None
    collector._status, collector._raw_queue = {}, None
    collector._slam_sender = SimpleNamespace(sent_packets=0, dropped_packets=0)
    backend = NativeLatestLidarBackend(collector, LatestLidarBackendConfig(
        250_000_000, global_backend="slam_toolbox", matcher_contract_id=SLAM_CONTRACT_ID,
        matcher_confidence_model=SLAM_CONFIDENCE_MODEL, matcher_transport=SLAM_TRANSPORT,
        pose_measurement_variance=(.01, .01, .01)))
    source = NativeLidarSource(backend, NativeLidarConfig(
        "RPLIDAR_C1", .18, 250_000_000, lidar_x_offset_m=OFFSET[0], lidar_y_offset_m=OFFSET[1]))
    admission = InputAdmission(AdmissionConfig(250_000_000, 0))
    model = ShadowWorldModel(resolved.runtime.composition.live_control.control.world_model)
    def update_world(context, accepted):
        estimate = RobotEstimate(context, LOCAL_FRAME_ID, 0., 0., _heading(context.monotonic_ns).yaw_rad,
                                 0., 1.2, (0.,)*25, localization_quality=healthy_localization(generation=3))
        return model(AdmittedFrame(context, accepted, ()), estimate)
    health = Observation("lidar_health", "RPLIDAR_C1", 0, START,
                         (DataField("age_ns", 0), DataField("point_count", len(raw.raw_scan))))
    for timestamp in range(START, END, 20_000_000):
        update_world(TickContext(timestamp, timestamp), (health,) if timestamp == START else ())
    collector._state_queue.put(initial)
    collector._drain_state()
    context = TickContext(100, END+10_000_000)
    pending_sample = source.read(context)
    assert "lidar_safety_clearance" in {sample.kind for sample in pending_sample.samples}
    assert "lidar_local_points" not in {sample.kind for sample in pending_sample.samples}
    admitted = admission(AcquisitionFrame(context, pending_sample.samples, (pending_sample.health,)))
    assert update_world(context, admitted.accepted).local_costmap is None
    collector._state_queue.put(ready)
    collector._drain_state()
    context = TickContext(101, END+80_000_000)
    ready_sample = source.read(context)
    admitted = admission(AcquisitionFrame(context, ready_sample.samples, (ready_sample.health,)))
    local = next(item for item in admitted.accepted if item.kind == "lidar_local_points")
    assert local.source_sequence == raw.raw_scan_id and local.captured_monotonic_ns == MID
    world = update_world(context, admitted.accepted)
    assert world.local_costmap.source_sequence == raw.raw_scan_id
    assert world.local_costmap.freshness_ns == context.monotonic_ns-MID
    assert world.local_costmap.occupied_cells
    context = TickContext(102, END+100_000_000)
    duplicate = source.read(context)
    admitted = admission(AcquisitionFrame(context, duplicate.samples, (duplicate.health,)))
    assert not any(item.kind == "lidar_local_points" for item in admitted.accepted)
    assert admitted.rejected
    assert all(item.reason is RejectionReason.DUPLICATE for item in admitted.rejected)
    unchanged = update_world(context, admitted.accepted)
    assert unchanged.local_costmap.revision == world.local_costmap.revision

    changed_time = list(ready[1])
    changed_time[4] += 1
    collector._state_queue.put((ready[0], tuple(changed_time), *ready[2:]))
    collector._drain_state()
    assert collector._fatal_error == "LIDAR_PHYSICAL_SCAN_IDENTITY_REWRITTEN"
    assert collector.get_raw_scan_snapshot().measurement_monotonic_ns == MID
