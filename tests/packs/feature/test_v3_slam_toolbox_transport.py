"""Real bounded SLAM transport and correction lineage without motor access."""
from collections import deque
from dataclasses import replace
import queue
import json
import socket
import struct
import sys
import threading
from types import SimpleNamespace

import pytest

from rig import resolved_config
from v3.adapters.native_lidar_port import NativeRawLidarSnapshot
from v3.adapters.process_lidar_port import ProcessLidarPort, _slam_matcher_result
from v3.adapters.rplidar_c1 import RplidarPoint
from v3.adapters.slam_toolbox_transport import (
    MAX_PACKET_BYTES, SCAN_BINS, SlamLocalizationReceiver, SlamOdometryPacket,
    SlamPosePacket, SlamScanPacket, SlamToolboxSender, decode_packet,
    encode_odometry, encode_pose, encode_scan,
)
from v3.contracts import (
    DeviceSample, GLOBAL_FRAME_ID, LOCAL_FRAME_ID, LocalizationQuality, Pose2D, QualityState,
    RobotEstimate, TickContext,
)


def _scan(sequence=7, measured=2_000_000_000, *, health="OK", points=None):
    return NativeRawLidarSnapshot(
        sequence, (measured+20_000_000)/1e9,
        measured-20_000_000, measured+20_000_000, measured, health,
        tuple(points or (RplidarPoint(0., 1., 10),)), {}, measured+20_000_000,
    )


def _encode(scan):
    return encode_scan(scan, session_id=11, source_pid=22,
        minimum_range_m=.08, maximum_range_m=12., lidar_x_m=.01, lidar_y_m=-.02)


def _pose(**changes):
    return replace(SlamPosePacket(11, 7, 2_000_000_000, 3,
        1., 2., .3, .01, .02, .03, 99), **changes)


def _accept(receiver, packet=None, **changes):
    arguments = dict(now_ns=2_040_000_000, current_generation=3,
                     odometry_ns=2_040_000_000, odometry_valid=True)
    arguments.update(changes)
    return receiver.accept(encode_pose(packet or _pose()), **arguments)


def test_scan_wire_retains_physical_lineage_ccw_nearest_minimum_and_unknown_bins():
    raw = _scan(points=(RplidarPoint(0., 1., 10), RplidarPoint(90., 2., 10),
        RplidarPoint(90.01, 1.5, 10), RplidarPoint(180., 2., 0),
        RplidarPoint(270., .01, 10), RplidarPoint(1.24, 3., 10)))
    encoded = _encode(raw)
    assert len(encoded) <= MAX_PACKET_BYTES
    packet = decode_packet(encoded)
    assert isinstance(packet, SlamScanPacket)
    assert len(packet.ranges_m) == SCAN_BINS
    assert (packet.session_id, packet.source_pid, packet.source_sequence) == (11, 22, 7)
    assert (packet.scan_start_ns, packet.measurement_ns, packet.scan_end_ns) == (
        raw.scan_start_monotonic_ns, raw.measurement_monotonic_ns, raw.scan_end_monotonic_ns)
    assert packet.ranges_m[0] == 1.
    assert packet.ranges_m[540] == 1.5
    assert packet.ranges_m[718] == 3.
    assert packet.ranges_m[360] == packet.ranges_m[180] == 0.
    assert packet.lidar_x_m == pytest.approx(.01)
    assert packet.lidar_y_m == pytest.approx(-.02)
    with pytest.raises(ValueError):
        _encode(replace(raw, health="ERROR"))


def test_fixed_odometry_and_pose_wire_do_not_contain_raw_payloads():
    odometry = SlamOdometryPacket(11, 22, 8, 2_040_000_000, 3, 1., 2., .3, True, False)
    assert len(encode_odometry(odometry)) <= 128
    assert decode_packet(encode_odometry(odometry)) == odometry
    assert len(encode_pose(_pose())) <= 128
    assert decode_packet(encode_pose(_pose())) == _pose()
    for invalid in (b"", b"x" * (MAX_PACKET_BYTES+1), _encode(_scan())+b"x",
                    b"BAD!"+_encode(_scan())[4:], encode_pose(_pose())[:-1]):
        with pytest.raises(ValueError):
            decode_packet(invalid)
    bad_range = bytearray(_encode(_scan()))
    struct.pack_into("<f", bad_range, len(bad_range)-4, float("nan"))
    with pytest.raises(ValueError):
        decode_packet(bytes(bad_range))


def test_no_consumer_never_encodes_scan_and_probes_are_bounded(tmp_path, monkeypatch):
    import v3.adapters.slam_toolbox_transport as transport
    sender = SlamToolboxSender(tmp_path / "absent.sock")
    monkeypatch.setattr(transport, "encode_scan", lambda *a, **kw: pytest.fail("no scan encoding without demand"))
    original_stat = transport.os.stat
    probes = []
    def checked_stat(path):
        probes.append(path)
        return original_stat(path)
    monkeypatch.setattr(transport.os, "stat", checked_stat)
    for now in range(2_000_000_000, 2_900_000_000, 1_000_000):
        assert not sender.publish_scan(_scan(), now_ns=now, session_id=11, source_pid=22,
            minimum_range_m=.08, maximum_range_m=12., lidar_x_m=0., lidar_y_m=0.,
            maximum_age_ns=250_000_000)
    assert len(probes) == 1
    assert sender._socket is None
    assert sender.sent_packets == sender.dropped_packets == 0


def test_real_socket_nonblocking_scan_throttle_reply_and_dead_sidecar(tmp_path):
    path = tmp_path / "slam.sock"
    server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    server.bind(str(path))
    server.settimeout(.2)
    sender = SlamToolboxSender(path, reply_session_id=11)
    options = dict(session_id=11, source_pid=22, minimum_range_m=.08,
                   maximum_range_m=12., lidar_x_m=0., lidar_y_m=0., maximum_age_ns=250_000_000)
    try:
        assert sender.publish_scan(_scan(), now_ns=2_040_000_000, **options)
        data, address = server.recvfrom(MAX_PACKET_BYTES)
        assert decode_packet(data).measurement_ns == 2_000_000_000
        assert sender._socket.gettimeout() == 0.
        assert address
        assert not sender.publish_scan(_scan(8, 2_100_000_000), now_ns=2_140_000_000, **options)
        server.sendto(encode_pose(_pose()), address)
        assert sender.receive() == (encode_pose(_pose()),)
        # A stale, vanished or full sidecar cannot stall/fault the producer.
        server.close()
        assert not sender.publish_scan(_scan(9, 2_200_000_000), now_ns=2_240_000_000, **options)
        assert sender.dropped_packets == 1
        assert not sender.active
    finally:
        sender.close()
        server.close()


def test_full_socket_drops_without_waiting_and_scan_health_is_independent(tmp_path):
    path = tmp_path / "slam.sock"
    server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    server.bind(str(path))
    sender = SlamToolboxSender(path)
    packet = SlamOdometryPacket(11, 22, 8, 2_040_000_000, 3, 1., 2., .3, True, True)
    try:
        assert sender.available(2_040_000_000)
        sender._socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024)
        for _ in range(100):
            sender.publish_odometry(packet, now_ns=2_040_000_000)
        assert sender.dropped_packets == 1
        assert 0 < sender.sent_packets < 100
        assert not sender.active
    finally:
        sender.close()
        server.close()


def test_scan_throttle_uses_measurement_spacing_across_clock_bucket_boundary(tmp_path):
    path = tmp_path / "slam.sock"
    server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    server.bind(str(path))
    sender = SlamToolboxSender(path)
    options = dict(session_id=11, source_pid=22, minimum_range_m=.08,
                   maximum_range_m=12., lidar_x_m=0., lidar_y_m=0., maximum_age_ns=250_000_000)
    try:
        assert sender.publish_scan(_scan(7, 2_190_000_000), now_ns=2_230_000_000, **options)
        # Adjacent 200 ms clock buckets are only 20 ms apart here. Exporting
        # both caused SLAM Toolbox's 180 ms gate to silently drop the latter.
        assert not sender.publish_scan(_scan(8, 2_210_000_000), now_ns=2_250_000_000, **options)
        assert sender.publish_scan(_scan(9, 2_390_000_000), now_ns=2_430_000_000, **options)
        assert sender.sent_packets == 2
    finally:
        sender.close()
        server.close()


@pytest.mark.parametrize("change,reason", [
    ({"session_id": 12}, "SESSION"), ({"source_sequence": 8}, "UNKNOWN_SCAN"),
    ({"measurement_ns": 2_000_000_001}, "UNKNOWN_SCAN"),
    ({"generation": 4}, "GENERATION"), ({"variance_x_m2": .26}, "COVARIANCE"),
])
def test_correction_rejects_wrong_lineage_or_uncertain_pose(change, reason):
    receiver = SlamLocalizationReceiver(11, 250_000_000)
    receiver.remember_scan(_scan())
    assert _accept(receiver, _pose(**change)) is None
    assert reason in receiver.last_reason
    assert receiver.accepted_packets == 0
    assert receiver.last_pose is None


def test_correction_age_generation_quality_duplicate_and_bridge_restart_fail_closed():
    receiver = SlamLocalizationReceiver(11, 250_000_000)
    receiver.remember_scan(_scan())
    assert _accept(receiver, now_ns=2_251_000_000) is None
    assert _accept(receiver, now_ns=1_999_000_000) is None
    assert _accept(receiver, odometry_valid=False) is None
    assert _accept(receiver, odometry_ns=1_000_000_000) is None
    assert _accept(receiver) == _pose()
    result = _slam_matcher_result(receiver, 2_040_000_000)
    assert result.source_raw_scan_id == 7
    assert result.measurement_monotonic_ns == result.pose_reference_monotonic_ns == 2_000_000_000
    assert result.scan_start_monotonic_ns == 1_980_000_000
    assert result.summary["matcher_contract_id"] == "R2B4_SLAM_TOOLBOX_LATEST_V1"
    assert result.summary["variance_x_m2"] == .01
    assert _accept(receiver) is None
    assert "DUPLICATE" in receiver.last_reason
    receiver.remember_scan(_scan(8, 2_200_000_000))
    restarted = _pose(source_sequence=8, measurement_ns=2_200_000_000, bridge_generation=100)
    assert _accept(receiver, restarted, now_ns=2_240_000_000, odometry_ns=2_240_000_000) is None
    assert receiver.error == "SLAM_BRIDGE_RESTART_REQUIRES_RUNTIME_RESTART"
    assert _slam_matcher_result(receiver, 2_240_000_000) is None
    assert _accept(receiver, replace(restarted, bridge_generation=99),
        now_ns=2_240_000_000, odometry_ns=2_240_000_000) is None


def test_slam_lineage_survives_backend_source_and_canonical_capture_encoding():
    from v3.adapters.latest_lidar import (
        LatestLidarBackendConfig, NativeLatestLidarBackend,
        SLAM_CONTRACT_ID, SLAM_CONFIDENCE_MODEL, SLAM_TRANSPORT,
    )
    from v3.adapters.live_lidar import NativeLidarConfig, NativeLidarSource
    from v3.capture_encoding import encode_value
    from v3.replay import _decode_production_value

    receiver = SlamLocalizationReceiver(11, 250_000_000)
    receiver.remember_scan(_scan())
    assert _accept(receiver) is not None
    result = _slam_matcher_result(receiver, 2_040_000_000)
    raw = replace(_scan(), summary={
        "raw_safety_valid_point_count": 1,
        **{f"{side}_clearance_m": 1. for side in ("front", "rear", "left", "right")},
        **{f"{side}_observation_count": 1 for side in ("front", "rear", "left", "right")},
    })
    class Port:
        def get_matcher_result(self): return result
        def get_raw_scan_snapshot(self): return raw
        def stop(self): pass
        def get_runtime_status(self):
            return dict(matcher_contract_id=SLAM_CONTRACT_ID,
                matcher_confidence_model=SLAM_CONFIDENCE_MODEL,
                matcher_transport=SLAM_TRANSPORT, running=True,
                matcher_process_alive=True, driver_connected=True, health="OK")
    config = LatestLidarBackendConfig(250_000_000, global_backend="slam_toolbox",
        matcher_contract_id=SLAM_CONTRACT_ID, matcher_confidence_model=SLAM_CONFIDENCE_MODEL,
        matcher_transport=SLAM_TRANSPORT, pose_measurement_variance=(.01, .01, .01))
    source = NativeLidarSource(NativeLatestLidarBackend(Port(), config),
        NativeLidarConfig("RPLIDAR_C1", .18, 250_000_000))
    snapshot = source.read(TickContext(8, 2_040_000_000))
    sample = next(item for item in snapshot.samples if item.kind == "lidar_matcher_diagnostics")
    values = {field.key: field.value for field in sample.values}
    expected = dict(matcher_contract_id=SLAM_CONTRACT_ID, slam_session_id=11,
                    slam_generation=3, slam_bridge_generation=99)
    assert {name: values[name] for name in expected} == expected
    # Exercise the actual container-neutral capture and current-source decoder.
    serialized = json.loads(json.dumps(encode_value(sample), allow_nan=False))
    restored = _decode_production_value(serialized, DeviceSample, "sample")
    assert restored == sample
    assert next(item for item in snapshot.samples if item.kind == "lidar_pose").captured_monotonic_ns == 2_000_000_000


def test_control_enqueues_local_pose_scalars_and_quality_not_global_corrections():
    port = ProcessLidarPort.__new__(ProcessLidarPort)
    port._stopped, port._slam_active, port._slam_enabled = False, True, True
    port._pending_slam_odometry = deque(maxlen=1)
    quality = LocalizationQuality(local_translation=QualityState.DEGRADED,
        heading=QualityState.GOOD, local_pose_continuous=True, generation=4)
    estimate = RobotEstimate(TickContext(8, 2_040_000_000), GLOBAL_FRAME_ID,
        100., 200., 1., 0., 0., (0.,)*25,
        local_pose=Pose2D(LOCAL_FRAME_ID, 1., 2., .3),
        global_pose=Pose2D(GLOBAL_FRAME_ID, 100., 200., 1.),
        map_to_odom=Pose2D(GLOBAL_FRAME_ID, 99., 198., .7), localization_quality=quality)
    port.publish_slam_odometry(estimate)
    assert port._pending_slam_odometry[-1] == (8, 2_040_000_000, 4, 1., 2., .3, True, True)
    port.publish_slam_odometry(replace(estimate, localization_quality=replace(quality, pose_discontinuity=True)))
    assert len(port._pending_slam_odometry) == 1
    assert port._pending_slam_odometry[-1][-2:] == (False, False)


def test_real_owner_closure_revokes_old_generation_without_waiting_for_another_reply(monkeypatch):
    import v3.adapters.process_lidar_port as owner
    resolved = resolved_config()
    state = [2, 3, 2_040_000_000, 1]
    stop = threading.Event()
    class Port:
        calls = 0
        def get_raw_scan_snapshot(self):
            self.calls += 1
            if self.calls == 3:
                state[:] = [4, 4, 2_040_000_000, 1]
                stop.set()
            return _scan()
        def get_matcher_result(self): return None
        def get_relative_result(self): return None
        def get_runtime_status(self):
            return dict(running=True, health="OK", driver_connected=True,
                        matcher_process_alive=True, matcher_error="LOCAL_WORKER_ERROR")
        def stop(self): pass
    class Sender:
        sent_packets = 0
        dropped_packets = 0
        received = False
        def __init__(self, *args, **kw): pass
        def publish_scan(self, *args, **kw): return True
        def receive(self):
            if not self.received:
                self.received = True
                return (encode_pose(_pose()),)
            return ()
        def close(self): pass
    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace(Serial=None))
    monkeypatch.setattr(owner, "SlamToolboxSender", Sender)
    monkeypatch.setattr(owner, "open_native_lidar_port", lambda *a, **kw: Port())
    monkeypatch.setattr(owner.time, "monotonic_ns", lambda: 2_040_000_000)
    output = queue.Queue(8)
    owner._lidar_owner_process_main(replace(resolved.lidar, global_backend="slam_toolbox"),
        None, None, None, None, output, None, threading.Event(), stop,
        None, None, False, .08, 2.5, 96, resolved.edges.lidar_process,
        slam_session_id=11, slam_odometry_state=state)
    results = []
    while not output.empty():
        packet = output.get_nowait()
        assert packet[0] == "state", packet
        results.append(packet)
    assert [None if item[2] is None else item[2][0] for item in results] == [None, 1, None]
    assert all(item[3]["matcher_error"] == "LOCAL_WORKER_ERROR" for item in results)
    assert all(item[3]["health"] == "OK" for item in results)


def test_slam_backend_keeps_relative_worker_and_never_builds_native_global_matcher(monkeypatch):
    import v3.lidar_matcher_process as worker
    outgoing = queue.Queue()
    packet = dict(kind="scan", matcher_contract_id=worker.MATCHER_CONTRACT_ID,
        scan_revision=7, source_scan_revision=7, captured_monotonic_ns=2_020_000_000,
        scan_start_monotonic_ns=1_980_000_000, scan_end_monotonic_ns=2_020_000_000,
        measurement_monotonic_ns=2_000_000_000, pose_reference_monotonic_ns=2_000_000_000,
        scan_pose_alignment_delta_ns=0, maximum_input_age_ns=250_000_000,
        matcher_config=resolved_config().lidar.matcher, pose_reference=(0., 0., 0.),
        scan=[], global_matching_enabled=False)
    class Incoming:
        packets = iter((packet, dict(kind="stop")))
        def get(self, timeout): return next(self.packets)
        def get_nowait(self): raise queue.Empty
    monkeypatch.setattr(worker.time, "monotonic_ns", lambda: 2_040_000_000)
    monkeypatch.setattr(worker, "LidarEstimator", lambda **kw: pytest.fail("native global matcher constructed"))
    class Relative:
        def __init__(self, _config): pass
        def process(self, _scan, _ns): return dict(dx_m=.01)
    monkeypatch.setattr(worker, "RelativeLidarOdometry", Relative)
    worker.matcher_process_main(Incoming(), outgoing, threading.Event(), threading.Event())
    assert outgoing.qsize() == 1
    assert outgoing.get_nowait()["kind"] == "relative"
