"""Physical clearance is measured from the offset chassis, with fail-closed units."""
import pytest

from rig import resolved_config
from v3.adapters.fake_edges import FakeHal
from v3.adapters.native_lidar_port import _sector_summary
from v3.adapters.rplidar_c1 import RplidarPoint, RplidarScan
from v3.contracts import (ActuatorRequest, DataField, DeviceSample, LifecycleState,
                         SafetyDecision, TickContext)
from v3.layers.l12_safety_final import FinalSafetyGate
from v3.lidar_geometry import body_clearance_m


def _scan(points):
    return RplidarScan(7, 200, 100, 200, 150, tuple(points))


def test_body_geometry_measured_from_drive_axle_and_preserves_scan_lineage():
    cfg = resolved_config().lidar
    assert cfg.safety_body_bounds_m == pytest.approx((-.10, .26, -.18, .18))
    # All four returns are 10 cm outside different physical edges. Raw front
    # and rear ranges differ because the lidar is close to the drive axle.
    points = [RplidarPoint(0., .365, 15), RplidarPoint(180., .195, 15),
              RplidarPoint(270., .28, 15), RplidarPoint(90., .28, 15)]
    summary = _sector_summary(_scan(points), cfg)
    for sector in ('front', 'rear', 'left', 'right'):
        assert summary[sector+'_clearance_m'] == pytest.approx(.1)
        assert summary[sector+'_observation_count'] == 1
    assert summary['clearance_basis'] == 'BODY_OUTLINE'
    assert summary['measurement_monotonic_ns'] == 150
    assert summary['raw_scan_id'] == 7
    assert summary['minimum_range_m'] == pytest.approx(.195)


def test_corner_clearance_is_euclidean_not_radial_subtraction():
    bounds = resolved_config().lidar.safety_body_bounds_m
    assert body_clearance_m(.32, .26, bounds) == pytest.approx(.1)
    assert body_clearance_m(.1, .1, bounds) == 0.
    assert body_clearance_m(-.16, -.26, bounds) == pytest.approx(.1)


def test_front_corner_is_observed_by_both_exposed_faces():
    import math
    cfg = resolved_config().lidar
    x, y = .27, .279  # Laser bearing falls outside the old front cone.
    laser_x = x-cfg.matcher.lidar_x_offset_m
    point = RplidarPoint(math.degrees(math.atan2(-y, laser_x)) % 360,
                         math.hypot(laser_x, y), 15)
    summary = _sector_summary(_scan([point]), cfg)
    assert summary['front_observation_count'] == summary['left_observation_count'] == 1
    assert summary['front_clearance_m'] < .1
    assert summary['front_clearance_m'] == summary['left_clearance_m']


@pytest.mark.parametrize('distance,expected', [(.099, SafetyDecision.STOP),
                                              (.101, SafetyDecision.ALLOW)])
def test_body_clearance_reaches_l12_without_changing_stop_authority(distance, expected):
    resolved = resolved_config()
    cfg = resolved.runtime.composition.live_control.control.lidar_safety
    raw = .26 - resolved.lidar.matcher.lidar_x_offset_m + distance
    summary = _sector_summary(_scan([RplidarPoint(0., raw, 15)]), resolved.lidar)
    values = tuple(DataField(key, summary[key]) for key in
        ('clearance_basis', 'front_clearance_m', 'front_observation_count')) + (DataField('age_ns', 10),)
    sample = DeviceSample(cfg.device_id, 'lidar_safety_clearance', 7, 150, values)
    writer = FakeHal()
    context = TickContext(1, 160)
    output = FinalSafetyGate(writer, cfg).finalize(context,
        ActuatorRequest(context, .3, .3), (), LifecycleState.ACTIVE, None, (sample,))
    assert output.safety_decision is expected
    assert writer.writes == (output,)
    if expected is SafetyDecision.STOP:
        assert output.left_output == output.right_output == 0.


@pytest.mark.parametrize('basis', [None, 'LIDAR_ORIGIN'])
def test_l12_rejects_missing_or_laser_origin_clearance_for_body_config(basis):
    cfg = resolved_config().runtime.composition.live_control.control.lidar_safety
    values = (DataField('age_ns', 0), DataField('front_clearance_m', 3.),
              DataField('front_observation_count', 1))
    if basis is not None:
        values += (DataField('clearance_basis', basis),)
    sample = DeviceSample(cfg.device_id, 'lidar_safety_clearance', 1, 100, values)
    context = TickContext(0, 100)
    result = FinalSafetyGate(FakeHal(), cfg).finalize(context,
        ActuatorRequest(context, .2, .2), (), LifecycleState.ACTIVE, None, (sample,))
    assert result.safety_decision is SafetyDecision.FAULT
    assert result.reason == 'LIDAR_SAFETY_INVALID'


@pytest.mark.parametrize('left,right', [(-.2, .2), (.2, -.2), (.2, .3), (.3, .2)])
@pytest.mark.parametrize('near_face', ['front', 'rear', 'left', 'right'])
def test_turning_offset_body_checks_opposite_corner_and_all_exposed_faces(left, right, near_face):
    from v3.contracts import WheelVelocitySetpoint
    cfg = resolved_config().runtime.composition.live_control.control.lidar_safety
    context = TickContext(0, 100)
    values = (DataField('age_ns', 0), DataField('clearance_basis', 'BODY_OUTLINE'))
    for side in ('front', 'rear', 'left', 'right'):
        values += (DataField(side+'_observation_count', 1),
                   DataField(side+'_clearance_m', .015 if side == near_face else 1.))
    sample = DeviceSample(cfg.device_id, 'lidar_safety_clearance', 1, 100, values)
    result = FinalSafetyGate(FakeHal(), cfg).finalize(context,
        ActuatorRequest(context, left, right), (), LifecycleState.ACTIVE, None, (sample,))
    assert result.safety_decision is SafetyDecision.STOP
    assert result.reason == 'LIDAR_CLEARANCE_LOW'
    assert result.left_output == result.right_output == 0.


def test_generated_nav2_polygon_uses_same_body_and_safety_padding(tmp_path):
    import json
    from tools.nav2_planner import _write_geometry
    control = resolved_config().runtime.composition.live_control.control
    path = tmp_path/'geometry.yaml'
    _write_geometry({'geometry': path, 'bt': tmp_path/'tree.xml'}, control)
    generated = path.read_text()
    for name in ('global_costmap', 'local_costmap'):
        block = generated.split(name+':\n  '+name+':\n')[1].split('\ncontroller_server:')[0]
        footprint = next(line.split(': ', 1)[1] for line in block.splitlines()
                         if line.strip().startswith('footprint:'))
        points = json.loads(json.loads(footprint))
        assert [p[0] for p in points] == pytest.approx([-.1, .26, .26, -.1])
        assert [p[1] for p in points] == pytest.approx([-.18, -.18, .18, .18])
        padding = float(next(line.split(': ', 1)[1] for line in block.splitlines()
                             if line.strip().startswith('footprint_padding:')))
        assert padding >= control.lidar_safety.minimum_clearance_m
        assert 'robot_radius' not in block
