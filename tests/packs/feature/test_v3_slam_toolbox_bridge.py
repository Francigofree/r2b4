"""Qualified scan-time alignment at the ROS edge, with no ROS/hardware imports."""

from dataclasses import replace
import math

import pytest

from v3.adapters.slam_toolbox_transport import SlamOdometryPacket, SlamScanPacket
from v3.slam_toolbox_bridge import MAX_ODOM_BRACKET_NS, align_scan


MEASURED_NS = 2_000_000_000


def _scan(**changes):
    return replace(SlamScanPacket(11, 22, 7, MEASURED_NS - 20_000_000,
        MEASURED_NS + 20_000_000, MEASURED_NS, .08, 12., .05, -.02, (1., 0., 2., 0.)), **changes)


def _odometry(delta=0, **changes):
    return replace(SlamOdometryPacket(11, 33, 8, MEASURED_NS + delta,
        3, 1., 2., .3, True, True), **changes)


def test_exact_scan_time_preserves_generation_and_pose_without_future_sample():
    expected = _odometry(generation=9, x_m=2., y_m=-3., yaw_rad=1.1)
    result = align_scan(_scan(), [expected])
    assert (result.generation, result.x_m, result.y_m, result.yaw_rad) == (9, 2., -3., 1.1)


def test_interpolation_uses_source_measurement_time_and_same_session_only():
    before = _odometry(-20_000_000, x_m=1., y_m=4., yaw_rad=.1)
    after = _odometry(60_000_000, x_m=5., y_m=0., yaw_rad=.5)
    unrelated = _odometry(0, session_id=99, x_m=100., y_m=200., yaw_rad=2.)
    result = align_scan(_scan(), [before, unrelated, after])
    assert result.generation == 3
    assert (result.x_m, result.y_m, result.yaw_rad) == pytest.approx((2., 3., .2))


@pytest.mark.parametrize("history", (
    [], [_odometry(-1)], [_odometry(1)],
    [_odometry(-20_000_000, session_id=99), _odometry(20_000_000, session_id=99)],
    [_odometry(-20_000_000, generation=3), _odometry(20_000_000, generation=4)],
    [_odometry(-20_000_000, local_translation_valid=False), _odometry(20_000_000)],
    [_odometry(-20_000_000), _odometry(20_000_000, heading_valid=False)],
    [_odometry(local_translation_valid=False)], [_odometry(heading_valid=False)],
    [_odometry(-50_000_000), _odometry(50_000_001)],
))
def test_missing_extrapolated_reset_invalid_and_excessive_gap_are_unavailable(history):
    assert align_scan(_scan(), history) is None


def test_maximum_gap_is_inclusive_and_invalid_intermediate_evidence_is_not_skipped():
    before = _odometry(-MAX_ODOM_BRACKET_NS // 2)
    after = _odometry(MAX_ODOM_BRACKET_NS // 2)
    assert align_scan(_scan(), [before, after]) is not None
    invalid = _odometry(-1, heading_valid=False)
    assert align_scan(_scan(), [before, invalid, after]) is None


def test_yaw_interpolation_takes_short_arc_across_pi():
    before = _odometry(-20_000_000, yaw_rad=math.radians(179))
    after = _odometry(20_000_000, yaw_rad=math.radians(-179))
    result = align_scan(_scan(), [before, after])
    assert abs(result.yaw_rad) == pytest.approx(math.pi)


def test_large_integer_measurement_clock_keeps_exact_bracket_selection():
    # This source clock cannot be round-tripped through float seconds exactly.
    measured = 2**60 + 3
    scan = _scan(scan_start_ns=measured-20, scan_end_ns=measured+20, measurement_ns=measured)
    before = _odometry(monotonic_ns=measured-1, x_m=0.)
    after = _odometry(monotonic_ns=measured+3, x_m=4.)
    assert align_scan(scan, [before, after]).x_m == pytest.approx(1.)
