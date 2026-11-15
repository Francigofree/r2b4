"""Startup failures retain their cause without serial access or robot motion."""
from dataclasses import replace
import queue
import threading

import pytest

from rig import resolved_config
from v3.adapters import native_lidar_port as owner


class _MatcherProcess:
    def __init__(self, exitcode):
        self.exitcode = exitcode
        self.terminated = False

    def start(self): pass
    def is_alive(self): return self.exitcode is None
    def join(self, timeout): pass

    def terminate(self):
        self.terminated = True
        self.exitcode = -15


class _Context:
    def __init__(self, *, ready=True, exitcode=None):
        self.ready = ready
        self.process = _MatcherProcess(exitcode)
        self.events = []

    def Queue(self, maxsize): return queue.Queue(maxsize)
    def Process(self, **kwargs): return self.process

    def Event(self):
        event = threading.Event()
        self.events.append(event)
        if len(self.events) == 2 and self.ready:
            event.set()
        return event


def _config():
    config = resolved_config().lidar
    return replace(config, driver=replace(config.driver, port="/dev/test-lidar"),
                   process_ready_timeout_s=.001)


@pytest.mark.parametrize("ready,exitcode,reason", [
    (False, None, "matcher readiness timeout"),
    (False, 17, "exitcode=17"),
    (True, 23, "matcher exited before driver start (exitcode=23)"),
])
def test_factory_reports_matcher_failure_before_cleanup(monkeypatch, ready, exitcode, reason):
    context = _Context(ready=ready, exitcode=exitcode)
    monkeypatch.setattr(owner.multiprocessing, "get_context", lambda _: context)
    serial_calls = []

    def serial_factory(*args, **kwargs):
        serial_calls.append(args)
        raise AssertionError("matcher failure must not open serial")

    with pytest.raises(RuntimeError) as caught:
        owner.open_native_lidar_port(_config(), lambda _: None, serial_factory)
    assert reason in str(caught.value)
    assert "exitcode=-15" not in str(caught.value)
    assert not serial_calls
    assert not context.process.is_alive()
    assert context.events[0].is_set()


def test_factory_preserves_serial_open_error_and_stops_matcher(monkeypatch):
    context = _Context()
    monkeypatch.setattr(owner.multiprocessing, "get_context", lambda _: context)

    def denied_serial(*args, **kwargs):
        raise PermissionError("test serial access denied")

    with pytest.raises(RuntimeError) as caught:
        owner.open_native_lidar_port(_config(), lambda _: None, denied_serial)
    assert "driver start rejected: PermissionError:test serial access denied" in str(caught.value)
    assert context.process.terminated
    assert context.events[0].is_set()


def test_driver_status_failure_does_not_hide_start_rejection(monkeypatch):
    context = _Context()
    monkeypatch.setattr(owner.multiprocessing, "get_context", lambda _: context)

    class Driver:
        def start(self): return False
        def stop(self): pass
        def get_latest_scan(self): return None

        def get_runtime_status(self):
            raise ValueError("diagnostic unavailable")

    monkeypatch.setattr(owner, "NativeRplidarC1", lambda *args, **kwargs: Driver())
    with pytest.raises(RuntimeError) as caught:
        owner.open_native_lidar_port(_config(), lambda _: None, lambda: None)
    assert "driver start rejected: status unavailable: ValueError:diagnostic unavailable" in str(caught.value)
    assert context.process.terminated
