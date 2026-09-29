"""Real matcher process transport without serial devices or motor access."""
import multiprocessing
import time
import random

from rig import resolved_config, room_lidar_scan
from v3.contracts.lidar import MATCHER_CONTRACT_ID
from v3.lidar_matcher_process import matcher_process_main
from v3.lidar_relative_odometry import RelativeLidarOdometry


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
