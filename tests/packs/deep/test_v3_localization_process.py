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
