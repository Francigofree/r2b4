import asyncio
import pytest
from lce_coop_probe_261011 import run

class FakeRobot:
    def __init__(self, values):
        self.values = iter(values)
        self.calls = []
    async def read(self, resource):
        assert resource == 'operator.status'
        self.calls.append(('read', resource))
        value = next(self.values)
        if isinstance(value, Exception):
            raise value
        return {'runtime_running': value}
    async def call(self, name, **parameters):
        self.calls.append(('call', name, parameters))

def test_samples_and_report(monkeypatch):
    sleeps = []
    async def sleep(delay):
        sleeps.append(delay)
    monkeypatch.setattr(asyncio, 'sleep', sleep)
    robot = FakeRobot([True, False, RuntimeError('probe failure'), None])
    result = asyncio.run(run(robot, count=4))
    assert sleeps == [0.5] * 4
    assert result['successful_count'] == 2
    assert result['runtime_running_true_count'] == 1
    assert len(result['measurements']) == 4
    assert [m['success'] for m in result['measurements']] == [True, True, False, False]
    assert robot.calls[-1] == ('call', 'hri.report', {'text': 'LCE probe completed'})
    assert len(robot.calls) == 5

def test_default_count(monkeypatch):
    async def sleep(delay):
        assert delay == 0.5
    monkeypatch.setattr(asyncio, 'sleep', sleep)
    result = asyncio.run(run(FakeRobot([False] * 12)))
    assert result['requested_count'] == 12
    assert result['successful_count'] == 12

@pytest.mark.parametrize('count', [-1, True, 1.5])
def test_invalid_count(count):
    robot = FakeRobot([])
    with pytest.raises(ValueError):
        asyncio.run(run(robot, count=count))
    assert robot.calls == []
