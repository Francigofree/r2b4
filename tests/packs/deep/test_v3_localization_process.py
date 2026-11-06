"""Real matcher process transport without serial devices or motor access."""
import multiprocessing
import os
from pathlib import Path
import signal
import time
import random

from rig import resolved_config, room_lidar_scan
from v3.contracts.lidar import MATCHER_CONTRACT_ID
from v3.lidar_matcher_process import matcher_process_main
from v3.lidar_relative_odometry import RelativeLidarOdometry


class _IdleLidarDriver:
    def start(self): return True
    def stop(self): pass
    def get_latest_scan(self): return None
    def get_runtime_status(self): return {"running": True, "connected": True}


def _matcher_owner(connection):
    from v3.adapters.native_lidar_port import NativeLidarPort

    port = NativeLidarPort(resolved_config().lidar, lambda _: None, driver=_IdleLidarDriver())
    try:
        assert port.start()
        connection.send(port._process.pid)
        # The test kills this owner without its ordinary stop/finally path.
        connection.recv()
    finally:
        port.stop()
        connection.close()


def _process_running(pid):
    try:
        state = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0]
    except FileNotFoundError:
        return False
    return state != "Z"


def _blocked_global_matcher(incoming, outgoing, stop, ready, entered, release):
    """Run the production worker with a controllable global-compute boundary."""
    import v3.lidar_matcher_process as worker
    original = worker.LidarEstimator
    class BlockedGlobalEstimator(original):
        def process_scan(self, *args, **kwargs):
            entered.set()
            if not release.wait(10):
                raise RuntimeError("test global-compute gate timed out")
            release.clear()
            entered.clear()
            return super().process_scan(*args, **kwargs)
    worker.LidarEstimator = BlockedGlobalEstimator
    worker.matcher_process_main(incoming, outgoing, stop, ready)


def test_matcher_exits_when_its_acquisition_owner_dies_without_stop():
    context = multiprocessing.get_context("spawn")
    parent, child = context.Pipe()
    owner = context.Process(target=_matcher_owner, args=(child,))
    matcher_pid = None
    owner.start()
    child.close()
    try:
        assert parent.poll(10), "the production matcher did not become ready"
        matcher_pid = parent.recv()
        assert _process_running(matcher_pid)
        owner.terminate()
        owner.join(5)
        assert not owner.is_alive()
        deadline = time.monotonic() + 5
        while _process_running(matcher_pid) and time.monotonic() < deadline:
            time.sleep(.02)
        assert not _process_running(matcher_pid), "matcher retained the terminated owner's session"
    finally:
        parent.close()
        if owner.is_alive():
            owner.terminate()
            owner.join(5)
        if matcher_pid is not None and _process_running(matcher_pid):
            os.kill(matcher_pid, signal.SIGTERM)


def test_localization_relative_process_equivalence_and_stale_input():
    cfg = resolved_config().lidar.matcher
    context = multiprocessing.get_context("spawn")
    incoming, outgoing = context.Queue(1), context.Queue(1)
    stop, ready = context.Event(), context.Event()
    process = context.Process(target=matcher_process_main, args=(incoming, outgoing, stop, ready))
    direct = RelativeLidarOdometry(cfg)
    noise = random.Random(77)
    process.start()
    try:
        assert ready.wait(10)
        for revision in (1, 2):
            if revision == 2:
                time.sleep(.1)
            scan = room_lidar_scan(.02*(revision-1), yaw_rad=.01*(revision-1),
                                   count=350+revision, phase=.3*revision)
            for point in scan:
                point['dist'] += noise.gauss(0, 15)
            end_ns = time.monotonic_ns()
            measurement_ns = end_ns-10_000_000
            packet = dict(kind="scan", matcher_contract_id=MATCHER_CONTRACT_ID,
                scan_revision=revision, source_scan_revision=revision, captured_monotonic_ns=end_ns,
                scan_start_monotonic_ns=end_ns-20_000_000, scan_end_monotonic_ns=end_ns,
                measurement_monotonic_ns=measurement_ns, pose_reference_monotonic_ns=measurement_ns,
                scan_pose_alignment_delta_ns=0, maximum_input_age_ns=250_000_000,
                pose_reference=(0., 0., 0.), matcher_config=cfg, scan=scan)
            expected = direct.process(scan, measurement_ns)
            incoming.put(packet, timeout=1)
            result = outgoing.get(timeout=10)
            if result['kind'] == 'relative':
                assert result['relative_motion'] == expected
                assert result['source_scan_revision'] == revision
                assert result['measurement_monotonic_ns'] == measurement_ns
                result = outgoing.get(timeout=10)
            assert result['kind'] == 'result', result
            assert result['source_scan_revision'] == revision
            assert result['measurement_monotonic_ns'] == measurement_ns
            assert result['summary']['relative_motion'] == expected
            if revision == 2:
                assert expected is not None
                assert set(expected) == {'start_ns', 'dx_m', 'dy_m', 'dyaw_rad', 'rmse_m', 'observability'}
        old = time.monotonic_ns()-1_000_000_000
        packet.update(scan_revision=3, source_scan_revision=3, captured_monotonic_ns=old,
            scan_start_monotonic_ns=old-20_000_000, scan_end_monotonic_ns=old,
            measurement_monotonic_ns=old-10_000_000, pose_reference_monotonic_ns=old-10_000_000)
        incoming.put(packet, timeout=1)
        stale = outgoing.get(timeout=10)
        assert stale['kind'] == 'drop' and stale['reason'] == 'stale_input'
    finally:
        stop.set()
        process.join(5)
        if process.is_alive():
            process.terminate()
            process.join(5)
        incoming.close()
        outgoing.close()
    assert process.exitcode == 0


def test_relative_process_publishes_before_global_completion_with_original_lineage():
    cfg = resolved_config().lidar.matcher
    context = multiprocessing.get_context('spawn')
    incoming, outgoing = context.Queue(1), context.Queue(1)
    stop, ready, entered, release = (context.Event() for _ in range(4))
    process = context.Process(target=_blocked_global_matcher,
        args=(incoming, outgoing, stop, ready, entered, release))
    direct = RelativeLidarOdometry(cfg)
    process.start()
    try:
        assert ready.wait(10)
        for revision in (1, 2):
            scan = room_lidar_scan(.02*(revision-1), yaw_rad=.01*(revision-1))
            end_ns = time.monotonic_ns()
            measurement_ns = end_ns-10_000_000
            expected = direct.process(scan, measurement_ns)
            incoming.put(dict(kind='scan', matcher_contract_id=MATCHER_CONTRACT_ID,
                scan_revision=revision, source_scan_revision=revision,
                captured_monotonic_ns=end_ns, scan_start_monotonic_ns=end_ns-20_000_000,
                scan_end_monotonic_ns=end_ns, measurement_monotonic_ns=measurement_ns,
                pose_reference_monotonic_ns=measurement_ns, scan_pose_alignment_delta_ns=0,
                maximum_input_age_ns=250_000_000, pose_reference=(0., 0., 0.),
                matcher_config=cfg, scan=scan), timeout=1)
            assert entered.wait(10)
            if revision == 2:
                early = outgoing.get(timeout=10)
                assert early['kind'] == 'relative'
                assert early['relative_motion'] == expected
                assert early['source_scan_revision'] == revision
                assert early['measurement_monotonic_ns'] == measurement_ns
                assert early['scan_end_monotonic_ns'] == end_ns
                assert early['relative_motion']['start_ns'] < measurement_ns
                assert entered.is_set() and not release.is_set()
                # Only compact registration/identity scalars cross this result
                # edge; raw scan/map/trajectory payloads remain in the worker.
                assert not {'scan', 'raw_scan', 'map_points_xy', 'summary'} & early.keys()
            release.set()
            final = outgoing.get(timeout=10)
            assert final['kind'] == 'result'
            assert final['summary']['relative_motion'] == expected
            assert final['measurement_monotonic_ns'] == measurement_ns
    finally:
        release.set()
        stop.set()
        process.join(5)
        if process.is_alive():
            process.terminate()
            process.join(5)
        incoming.close()
        outgoing.close()
    assert process.exitcode == 0


def _relative_packet(measurement_ns=1_000_000_000, revision=2):
    return dict(kind='relative', matcher_contract_id=MATCHER_CONTRACT_ID,
        scan_revision=revision, source_scan_revision=revision,
        captured_monotonic_ns=measurement_ns+10_000_000,
        scan_start_monotonic_ns=measurement_ns-10_000_000,
        scan_end_monotonic_ns=measurement_ns+10_000_000,
        measurement_monotonic_ns=measurement_ns, pose_reference_monotonic_ns=measurement_ns,
        scan_pose_alignment_delta_ns=0, published_monotonic_ns=measurement_ns+20_000_000,
        relative_motion=dict(start_ns=measurement_ns-100_000_000, dx_m=.02,
                             dy_m=0., dyaw_rad=.01, rmse_m=.002, observability=.9))


def _seed_raw_scan(port, measurement_ns=1_000_000_000, revision=2):
    from v3.adapters.native_lidar_port import NativeRawLidarSnapshot
    port._raw_snapshot = NativeRawLidarSnapshot(revision, (measurement_ns+10_000_000)/1e9,
        measurement_ns-10_000_000, measurement_ns+10_000_000, measurement_ns,
        'OK', (), {})


def test_native_port_relative_is_independent_of_global_result_revision_and_late_fallback():
    from v3.adapters.native_lidar_port import NativeLidarPort
    now = [1_020_000_000]
    port = NativeLidarPort(resolved_config().lidar, lambda _: None, driver=_IdleLidarDriver(),
                           monotonic_ns=lambda: now[0])
    _seed_raw_scan(port)
    early = _relative_packet()
    port._accept_matcher_packet(early)
    relative = port.get_relative_result()
    assert relative is not None and relative.source_sequence == 2
    assert relative.end_ns == 1_000_000_000
    assert port.get_matcher_result() is None
    final = dict(early, kind='result', summary={'relative_motion': early['relative_motion']})
    port._accept_matcher_packet(final)
    assert port.get_matcher_result().matcher_result_id == 1
    assert port.get_relative_result() is relative
    # A duplicate completion cannot mint a new sample sequence or timestamp.
    port._accept_matcher_packet(early)
    assert port.get_relative_result() is relative
    now[0] += port.config.maximum_result_age_ns
    _seed_raw_scan(port, revision=3)
    late = dict(_relative_packet(revision=3), kind='result',
                summary={'relative_motion': early['relative_motion']})
    port._accept_matcher_packet(late)
    assert port.get_relative_result() is relative
    assert port.get_relative_result().end_ns == 1_000_000_000
    assert port.get_matcher_result().matcher_result_id == 1
    assert port._last_matcher_reason == 'STALE_OR_INCONSISTENT_RESULT'


def test_native_port_malformed_relative_revokes_compute_without_stopping_physical_acquisition():
    from v3.adapters.native_lidar_port import NativeLidarPort
    for change in ({'relative_motion': {'dx_m': float('nan')}},
                   {'relative_motion': None}, {'measurement_monotonic_ns': 'invalid'}):
        port = NativeLidarPort(resolved_config().lidar, lambda _: None,
            driver=_IdleLidarDriver(), monotonic_ns=lambda: 1_020_000_000)
        _seed_raw_scan(port)
        port._running = True
        packet = _relative_packet()
        packet.update(change)
        port._accept_matcher_packet(packet)
        assert port._matcher_error
        assert not port._fatal_error and port._running
        assert port._physical_health() == 'OK'
        assert port.get_relative_result() is None
        # Capability error is latched; a later success cannot revive it.
        port._accept_matcher_packet(_relative_packet())
        assert port.get_relative_result() is None


def test_native_port_new_process_lifetime_clears_old_relative_and_global_evidence():
    from v3.adapters.native_lidar_port import NativeLidarPort
    port = NativeLidarPort(resolved_config().lidar, lambda _: None, driver=_IdleLidarDriver())
    try:
        assert port.start()
        measurement_ns = time.monotonic_ns()-20_000_000
        _seed_raw_scan(port, measurement_ns)
        early = _relative_packet(measurement_ns)
        port._accept_matcher_packet(early)
        port._accept_matcher_packet(dict(early, kind='result',
            summary={'relative_motion': early['relative_motion']}))
        assert port.get_relative_result() is not None
        assert port.get_matcher_result() is not None
        port.stop()
        assert port.start()
        assert port.get_relative_result() is None
        assert port.get_matcher_result() is None
        assert port.get_raw_scan_snapshot() is None
    finally:
        port.stop()
