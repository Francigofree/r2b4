from concurrent.futures import ThreadPoolExecutor
from queue import Empty
import threading

import pytest

from v3.observation import ObservationClosed, ObservationHub, ObservationIntegrityError
from v3.import_guard import validate_v3_imports


def test_reliable_overflow_is_permanent_and_latest_is_isolated():
    hub = ObservationHub()
    reliable = hub.subscribe_reliable('capture', capacity=2, required=True)
    latest = hub.subscribe_latest('gui')
    frames = [hub.publish(object(), topic='tick').frame for _ in range(4)]
    assert reliable.drain() == tuple(frames[:2])
    assert latest.get_nowait() is frames[-1]
    assert latest.snapshot().superseded_count == 3
    assert latest.snapshot().integrity_ok
    assert reliable.snapshot().lost_count == 2
    assert reliable.snapshot().high_water_mark == 2
    hub.close()
    with pytest.raises(ObservationIntegrityError):
        reliable.assert_integrity()
    with pytest.raises(ObservationIntegrityError):
        hub.assert_required_integrity()
    assert not hub.snapshot().required_integrity_ok


def test_payload_identity_and_filter_do_not_require_global_continuity():
    class Opaque:
        def __getattribute__(self, name):
            raise AssertionError('hub inspected payload')
    payload = Opaque()
    hub = ObservationHub()
    all_frames = hub.subscribe_reliable('all', capacity=3)
    filtered = hub.subscribe_reliable('capture', capacity=2, topics=['tick'], required=True)
    first = hub.publish(payload, topic='tick').frame
    hub.publish(payload, topic='unrelated')
    last = hub.publish(payload, topic='tick').frame
    assert filtered.drain() == (first, last)
    assert first.payload is payload and last.payload is payload
    assert all_frames.get_nowait() is first
    filtered.assert_integrity()


def test_close_drains_and_wakes_blocked_consumers():
    hub = ObservationHub()
    sub = hub.subscribe_reliable('capture', capacity=2)
    frame = hub.publish(object(), topic='tick').frame
    hub.close()
    hub.close()
    assert sub.get() is frame
    with pytest.raises(ObservationClosed):
        sub.get()
    with pytest.raises(ObservationClosed):
        hub.publish(object(), topic='tick')
    with pytest.raises(ObservationClosed):
        hub.subscribe_latest('late')
    hub2 = ObservationHub()
    waiting = hub2.subscribe_latest('wait')
    with ThreadPoolExecutor() as pool:
        future = pool.submit(waiting.get)
        waiting.close()
        with pytest.raises(ObservationClosed):
            future.result(timeout=2)


def test_concurrent_publishers_and_consumer_preserve_identity_order_and_drain():
    hub = ObservationHub()
    a = hub.subscribe_reliable('a', capacity=2000, required=True)
    b = hub.subscribe_reliable('b', capacity=2000, required=True)
    barrier = threading.Barrier(5)
    def publish():
        barrier.wait()
        return [hub.publish(object(), topic='tick').frame for _ in range(400)]
    def consume():
        frames = []
        while True:
            try:
                frames.append(a.get())
            except ObservationClosed:
                return frames
    with ThreadPoolExecutor(max_workers=5) as pool:
        consumer = pool.submit(consume)
        publishers = [pool.submit(publish) for _ in range(4)]
        barrier.wait()
        for future in publishers:
            future.result(timeout=5)
        hub.close()
        frames = consumer.result(timeout=5)
    assert [f.sequence for f in frames] == list(range(1600))
    assert all(x is y for x, y in zip(frames, b.drain(), strict=True))
    hub.assert_required_integrity()


@pytest.mark.parametrize('source', [
    'import v3.engine', 'from . import capture', 'from .mcap_writer import StdlibMcapWriter',
    'import serial', 'import RPi.GPIO', 'import pathlib', 'import os', 'import io',
    'from builtins import open', "open('/tmp/no')", 'import numpy',
])
def test_observation_guard_rejects_engine_capture_hardware_and_io(tmp_path, source):
    (tmp_path / 'v3').mkdir()
    (tmp_path / 'v3' / 'observation.py').write_text(source)
    assert any(v.code.startswith('OBSERVATION_') for v in validate_v3_imports(tmp_path))
