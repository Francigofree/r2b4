"""Bounded Nav2 group recovery without ROS or physical device access."""
import signal
from types import SimpleNamespace
import pytest
from rig import resolved_config
from tools import nav2_planner as host


@pytest.mark.parametrize('exhaust', [False, True])
def test_group_restart_reaps_old_children_and_requires_fresh_bridge(monkeypatch, tmp_path, exhaust):
    paths = dict(work=tmp_path, state=tmp_path/'state.json', status=tmp_path/'status.json',
                 socket=tmp_path/'nav.sock')
    signals, children, published = {}, [], []
    now = [10.]
    def write_json(path, state):
        if path == paths['state']:
            published.append(dict(state))
    def read_json(path):
        if path == paths['status'] and children:
            return dict(pid=children[-1].pid, state='RUNNING', lifecycle_status=dict(
                active=True, last_response_checked_ns=int(now[0]*1e9)))
        return {}
    class Child:
        def __init__(self):
            assert all(not child.alive for child in children)
            self.pid = 100 + len(children)
            self.alive = not (exhaust or len(children) == 0)
            self.returncode = None if self.alive else 1
            children.append(self)
        def poll(self): return None if self.alive else self.returncode
        def terminate(self): self.alive, self.returncode = False, -15
        def kill(self): self.alive, self.returncode = False, -9
        def wait(self, timeout): return self.returncode
    def sleep(seconds):
        now[0] += seconds
        if not exhaust and any(s['state'] == 'RUNNING' for s in published):
            signals[signal.SIGTERM](signal.SIGTERM, None)
    monkeypatch.setattr(host, '_paths', lambda _: paths)
    monkeypatch.setattr(host, '_resolved_config', lambda _: resolved_config())
    monkeypatch.setattr(host, '_write_geometry', lambda *a: .3)
    monkeypatch.setattr(host, '_navigation_environment', lambda *a: {})
    monkeypatch.setattr(host, '_commands', lambda *a, **kw: [('bridge', ['fake-bridge'])])
    monkeypatch.setattr(host, '_apply_ros_affinity', lambda *a: [])
    monkeypatch.setattr(host, '_process_start', lambda pid: str(pid))
    monkeypatch.setattr(host, '_write_json', write_json)
    monkeypatch.setattr(host, '_read_json', read_json)
    monkeypatch.setattr(host.os, 'nice', lambda *a: None)
    monkeypatch.setattr(host.signal, 'signal', lambda signum, handler: signals.__setitem__(signum, handler))
    monkeypatch.setattr(host.subprocess, 'Popen', lambda *a, **kw: Child())
    monkeypatch.setattr(host.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(host.time, 'monotonic_ns', lambda: int(now[0]*1e9))
    monkeypatch.setattr(host.time, 'sleep', sleep)
    if exhaust:
        with pytest.raises(RuntimeError, match='NAV2_RECOVERY_EXHAUSTED'):
            host._supervise(tmp_path, 74)
        assert len(children) == 4
        assert published[-1]['state'] == 'FAILED'
    else:
        assert host._supervise(tmp_path, 74) == 0
        assert len(children) == 2
        assert any(s['state'] == 'RUNNING' and s['bridge_pid'] == children[-1].pid for s in published)
        assert published[-1]['state'] == 'STOPPED'
    assert any(s['state'] == 'RESTARTING' for s in published)
    assert all(not child.alive for child in children)
