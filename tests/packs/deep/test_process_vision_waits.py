"""The passive vision consumer wakes for demand/data/shutdown, without polling."""
import socket
import threading

import pytest

from v3.adapters.picamera2_camera import Picamera2CameraConfig
from v3.adapters.process_vision_port import ProcessVisionPort


def test_idle_and_failed_demand_wait_for_state_change_without_reconnecting(tmp_path, monkeypatch):
    from v3.adapters.vision_media_socket import VisionClient
    attempts = []
    def fail(_client):
        attempts.append(1)
        raise ConnectionError("offline-test")
    monkeypatch.setattr(VisionClient, "connect", fail)
    port = ProcessVisionPort(Picamera2CameraConfig(), None, root=tmp_path)
    try:
        assert attempts == []
        port.set_person_detection_demand(True)
        with port._condition:
            assert port._condition.wait_for(lambda: port._error is not None, timeout=1.)
            assert "offline-test" in port._error
        assert len(attempts) == 1
        # A new explicit demand revision is the only way to reconnect after
        # failure. A rapid release/reacquire must not lose its notification.
        port.set_person_detection_demand(False)
        port.set_person_detection_demand(True)
        with port._condition:
            assert port._condition.wait_for(lambda: port._error is not None, timeout=1.)
        assert len(attempts) == 2
    finally:
        port.stop()
    assert not port._collector.is_alive()


@pytest.mark.parametrize("release", [False, True])
def test_silent_stream_is_interrupted_by_release_or_stop(tmp_path, monkeypatch, release):
    from v3.adapters.vision_media_socket import VisionClient
    connected = threading.Event()
    client, server = socket.socketpair()
    def connect(_client):
        connected.set()
        return client
    monkeypatch.setattr(VisionClient, "connect", connect)
    port = ProcessVisionPort(Picamera2CameraConfig(), None, root=tmp_path)
    try:
        port.set_person_detection_demand(True)
        assert connected.wait(1.)
        server.settimeout(1.)
        assert server.recv(128) == b"R2B4VISION1 PERSON\n"
        if release:
            port.set_person_detection_demand(False)
            assert server.recv(128) == b""
        port.stop()
        assert not port._collector.is_alive()
        assert port.get_edge_snapshot().frame is None
    finally:
        port.stop()
        server.close()
