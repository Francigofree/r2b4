import asyncio
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import terkepesz_r2b4 as m

class FakeRobot:
    root = Path('/not-real')
    def __init__(self, *, mode='full', hz=50):
        self.mode, self.hz = mode, hz
        self.active = False
        self.calls = []
        self.seq = 0
    async def read(self, name):
        if name == 'operator.status':
            return {'runtime_running': True, 'runtime_pid': 10,
                    'capture_mode': self.mode, 'capture_hz': self.hz}
        if name == 'capture.status':
            return {'path': '/tmp/sample.mcap'}
        if name == 'v3.status':
            self.seq += 1
            return {'state': 'RUNNING', 'enabled': self.active,
                    'safety_decision': 'ALLOW' if self.active else 'STOP',
                    'safety_reason': None if self.active else 'NOT_ACTIVE',
                    'fault_layer': None,
                    'source_health': [{'device_id': d, 'state': 'OK'} for d in
                                      ('WHEEL_ENCODERS', 'BNO055_IMU', 'RPLIDAR_C1')],
                    'estimate': {'v_mps': 0.0, 'omega_rad_s': 0.0,
                                 'localization_quality': {'heading': 'GOOD',
                                                          'local_translation': 'GOOD'}},
                    'world': {'local_costmap': {'source_sequence': self.seq}}}
        if name == 'behavior.state':
            return {'lifecycle': 'ACTIVE' if self.active else 'CANCELLED',
                    'behavior_id': 'behavior-1'}
        raise AssertionError(name)
    async def call(self, name, **parameters):
        self.calls.append((name, parameters))
        if name == 'behavior.room_cruise':
            self.active = True
            return {'lifecycle': 'STARTING', 'behavior_id': 'behavior-1'}
        if name == 'behavior.cancel':
            self.active = False
            return {'lifecycle': 'CANCELLED'}
        raise AssertionError(name)

def test_wrong_capture_mode_no_movement():
    robot = FakeRobot(mode='alap', hz=10)
    result = asyncio.run(m.run(robot, mode='survey'))
    assert result['status'] == 'FAILED'
    assert result['reason'] == 'REQUIRES_FULL_50_CAPTURE'
    assert not robot.calls

def test_canonical_behavior_cancel_preserves_invocation():
    robot = FakeRobot()
    result = asyncio.run(m._moving_window(robot, 10, 0.05,
                                         max_v_mps=0.18, max_omega_rad_s=0.5))
    assert result['behavior_id'] == 'behavior-1'
    assert [a for a, _ in robot.calls] == ['behavior.room_cruise', 'behavior.cancel']
    assert not robot.active

def test_stationary_evidence_is_not_automatic_keyframe():
    robot = FakeRobot()
    result = asyncio.run(m._stationary_window(robot, 10, 0.1, interval_s=0.02))
    assert result['scan_sequence_advanced']
    assert result['stationary_samples'] >= 2
    assert not result['keyframe_candidate']

def test_build_needs_real_finalized_mcap():
    robot = FakeRobot()
    result = asyncio.run(m.run(robot, mode='build', capture_path='/missing/f.mcap'))
    assert result['status'] == 'FAILED'
    assert result['reason'] in {'CANONICAL_MAP_TOOL_MISSING', 'FINALIZED_MCAP_REQUIRED'}

def test_survey_numeric_bounds():
    robot = FakeRobot()
    try:
        asyncio.run(m.run(robot, mode='survey', duration_s=float('nan')))
    except ValueError as exc:
        assert 'duration_s' in str(exc)
    else:
        raise AssertionError('must reject nan')

def test_build_success_and_atlas_import(tmp_path):
    root = tmp_path / 'r2b4'
    tool = root / 'tools' / 'mcap50-to-map.py'
    tool.parent.mkdir(parents=True)
    tool.write_text('''import json, sys\nfrom pathlib import Path\nout=Path(sys.argv[sys.argv.index("--output")+1]);out.mkdir(parents=True)\n(out/"report.json").write_text(json.dumps({"schema":"R2B4_GLOBAL_MAP_BUILD_V2","evidence_gate":{"strict_zero_loss":True},"stationary":{"usable_keyframes":3},"pose_graph":{"edge_count":2},"map":{"observed_area_m2":4.25}}))\n(out/"atlas_reference.json").write_text(json.dumps({"schema":"R2B4_ATLAS_REFERENCE_V1","map_id":"atlas:test"}))\n''')
    capture = root / 'runtime' / 'captures' / 'test.mcap'
    capture.parent.mkdir(parents=True)
    capture.write_bytes(b'dummy')
    class ImportRobot(FakeRobot):
        root = root if False else None
        async def call(self, name, **parameters):
            self.calls.append((name, parameters))
            if name == 'spatial.load_atlas':
                return {'status': 'LOADED'}
            if name == 'hri.report':
                return {'status': 'RECORDED'}
            raise AssertionError(name)
    robot = ImportRobot()
    robot.root = root
    result = asyncio.run(m.run(robot, mode='build', capture_path=str(capture)))
    assert result['status'] == 'MAP_BUILT'
    assert result['atlas_registered']
    assert result['keyframes'] == 3
    assert result['observed_area_m2'] == 4.25
    assert result['live_navigation_map'] is False
    assert (capture.with_name('test_global_map') / 'report.json').exists()
