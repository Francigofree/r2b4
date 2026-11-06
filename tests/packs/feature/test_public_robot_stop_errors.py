"""Responsive host errors preserve the owner; dead IPC keeps the stop fallback."""
from contextlib import contextmanager
import json
import os
import socket
import threading

import pytest

from r2b4_orchestration.robot_runtime import (
    MAX_REQUEST_BYTES, SCHEMA, PublicRobotClient, PublicRobotProtocolError,
    PublicRobotRemoteError, _recv,
)


@contextmanager
def _reply_owner(tmp_path, response):
    path = tmp_path / "owner.sock"
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(str(path))
    listener.listen(1)
    listener.settimeout(2.)
    release = threading.Event()
    requests, errors = [], []

    def serve():
        try:
            connection, _ = listener.accept()
            with connection:
                connection.settimeout(2.)
                requests.append(_recv(connection, MAX_REQUEST_BYTES))
                if response is None:
                    release.wait(2.)
                else:
                    connection.sendall(response)
        except Exception as exc:
            errors.append(exc)

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    try:
        yield PublicRobotClient(tmp_path, socket_path=path, timeout_s=.1), requests
    finally:
        release.set()
        thread.join(2.)
        listener.close()
        assert not thread.is_alive() and errors == []


@pytest.mark.parametrize("operation", ["preempt", "revoke"])
def test_responsive_stop_error_does_not_terminate_the_public_owner(tmp_path, monkeypatch, operation):
    response = json.dumps({"schema": SCHEMA, "error": "OperatorError:final STOP failed"}).encode() + b"\n"
    with _reply_owner(tmp_path, response) as (client, requests):
        terminated = []
        monkeypatch.setattr(client, "_terminate_unresponsive_owner", lambda: terminated.append(True))
        with pytest.raises(PublicRobotRemoteError, match="final STOP failed"):
            getattr(client, operation)("STOP")
        assert requests[0]["operation"] == operation
        assert client._peer_pid == os.getpid()
        assert terminated == []


@pytest.mark.parametrize("operation", ["preempt", "revoke"])
@pytest.mark.parametrize("response", [b'{"schema":"wrong"}\n', b"invalid json\n", b"incomplete"])
def test_invalid_stop_reply_keeps_the_verified_owner_fallback(tmp_path, monkeypatch, operation, response):
    with _reply_owner(tmp_path, response) as (client, _requests):
        terminated = []
        monkeypatch.setattr(client, "_terminate_unresponsive_owner", lambda: terminated.append(client._peer_pid))
        with pytest.raises(PublicRobotProtocolError):
            getattr(client, operation)("STOP")
        assert terminated == [os.getpid()]


@pytest.mark.parametrize("operation", ["preempt", "revoke"])
def test_unresponsive_stop_peer_keeps_the_verified_owner_fallback(tmp_path, monkeypatch, operation):
    with _reply_owner(tmp_path, None) as (client, _requests):
        terminated = []
        monkeypatch.setattr(client, "_terminate_unresponsive_owner", lambda: terminated.append(client._peer_pid))
        with pytest.raises(TimeoutError):
            getattr(client, operation)("STOP")
        assert terminated == [os.getpid()]


@pytest.mark.parametrize("operation", ["preempt", "revoke"])
def test_invalid_local_stop_input_cannot_kill_a_previously_verified_peer(tmp_path, monkeypatch, operation):
    client = PublicRobotClient(tmp_path)
    client._peer_pid = 123
    terminated = []
    monkeypatch.setattr(client, "_terminate_unresponsive_owner", lambda: terminated.append(True))
    with pytest.raises(ValueError):
        getattr(client, operation)(float("nan"))
    assert client._peer_pid is None and terminated == []
