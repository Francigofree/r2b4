from __future__ import annotations

import asyncio
import fcntl
import json
import socket
import threading
import time
from types import SimpleNamespace

import pytest


def _check_revoked_conversation_cannot_dispatch_late_default_robot_tool(tmp_path, monkeypatch, revocation):
    from r2b4_orchestration import agent_core, agent_tools
    from r2b4_orchestration.agent_contracts import AgentModelReply, AgentToolRequest
    from r2b4_voice import conversation_service

    entered, release, finished, dispatched = (threading.Event() for _ in range(4))
    now = [100.0]
    clock = SimpleNamespace(monotonic=lambda: now[0], monotonic_ns=lambda: int(now[0] * 1e9))
    monkeypatch.setattr(agent_core, "time", clock)
    monkeypatch.setattr(conversation_service, "time", clock)
    monkeypatch.setattr(agent_tools, "_er2_delegate", lambda *a, **kw: dispatched.set())

    class Model:
        model = "host-test"
        def complete(self, *args):
            raise AssertionError("AgentCore must own this turn")
        def complete_agent_step(self, *args, **kwargs):
            entered.set()
            assert release.wait(1)
            return AgentModelReply(self.model, tool_request=AgentToolRequest(
                "er2.delegate", {"task": "move", "reason": "multi_step_physical"}))

    class Agent(agent_core.AgentCore):
        def run(self, *args, **kwargs):
            try:
                return super().run(*args, **kwargs)
            finally:
                finished.set()

    model = Model()
    service = conversation_service.ConversationService(
        llm=model, agent=Agent(model, agent_core.AgentToolBroker(agent_tools.build_default_agent_tools(tmp_path))),
        robot_context=SimpleNamespace(build=lambda: SimpleNamespace(available_actions=(), to_jsonable=lambda: {})),
        prompt_assembler=SimpleNamespace(build_messages=lambda *a, **kw: [{"role": "user", "content": "test"}]),
        journal=SimpleNamespace(session_id="test", path=tmp_path / "unused", append=lambda *a, **kw: None),
    )
    try:
        turn = service.submit_text("move", source="test")
        assert entered.wait(1)
        if revocation == "timeout":
            now[0] += 1
            # The waiter's elapsed time expires while the model is held.
            original = clock.monotonic
            counter = iter((101.0, 102.0))
            clock.monotonic = lambda: next(counter, 102.0)
            assert service.wait_for_turn(turn, timeout_s=0.01) is None
            clock.monotonic = original
        elif revocation == "close":
            service.close(timeout_s=0.01)
        elif revocation == "stop":
            service.cancel_pending_turns()
        else:
            now[0] += 91
        release.set()
        assert finished.wait(1)
        assert not dispatched.is_set()
    finally:
        release.set()
        service.close()


def _check_host_readiness_and_active_preflight_reject_stale_status(tmp_path, monkeypatch):
    from v3 import control_cli, operator_controller, resident_status
    clock = [10.0]
    fake_time = SimpleNamespace(monotonic=lambda: clock[0], monotonic_ns=lambda: int(clock[0] * 1e9),
                                sleep=lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    monkeypatch.setattr(operator_controller, "time", fake_time)
    monkeypatch.setattr(resident_status, "time", fake_time)
    controller = operator_controller.OperatorController(tmp_path)
    status = {"state": "RUNNING", "ready_for_active": True, "monotonic_ns": 1,
              "tick_id": 2, "safety_decision": "ALLOW", "enabled": True,
              "left_output": 0.1, "right_output": 0.1}
    monkeypatch.setattr(controller, "_runtime_pid", lambda: 123)
    monkeypatch.setattr(controller, "_pid_matches", lambda *a: True)
    monkeypatch.setattr(controller, "_read_status_optional", lambda: status)
    monkeypatch.setattr(controller, "_failure_report", lambda *a: None)
    with pytest.raises(operator_controller.OperatorError, match="not ready"):
        controller._wait_ready(timeout=0.1)
    assert not controller._wait_fresh_ready(123, None, timeout=0.1)
    assert not controller._wait_allow(456, 1, "test")
    path = tmp_path / "status.json"
    path.write_text(json.dumps({"schema": control_cli.RESIDENT_PROCESS_STATUS_SCHEMA, **status}))
    with pytest.raises(ValueError, match="not ready"):
        control_cli._active_preflight(path)
    status["monotonic_ns"] = fake_time.monotonic_ns()
    assert controller.live_runtime_status() == status
    controller._wait_ready(timeout=0.1)


def _check_host_shutdown_waits_for_allowed_capture_finalization(tmp_path, monkeypatch):
    from v3 import operator_controller, process_sidecars
    clock = [0.0]
    fake_time = SimpleNamespace(monotonic=lambda: clock[0], sleep=lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    monkeypatch.setattr(operator_controller, "time", fake_time)
    monkeypatch.setattr(operator_controller.os, "kill", lambda *args: None)
    controller = operator_controller.OperatorController(tmp_path)
    finish_at = process_sidecars._SIDECAR_FINISH_TIMEOUT_S * 0.75
    monkeypatch.setattr(controller, "stop", lambda **kw: None)
    monkeypatch.setattr(controller, "_runtime_pid", lambda: 123)
    monkeypatch.setattr(controller, "_pid_matches", lambda *a: clock[0] < finish_at)
    monkeypatch.setattr(controller, "current_capture_mode", lambda: "full")
    monkeypatch.setattr(controller, "current_capture_path", lambda: None)
    controller.runtime_stop()
    assert clock[0] >= finish_at


def _check_finite_navigation_preparation_obeys_total_budget(tmp_path, boundary):
    from v3.finite_navigation import FiniteNavigationExecutor
    from v3.operator_controller import OperatorController
    entered, cancelled, done = (threading.Event() for _ in range(3))
    stops, results = [], []
    class Controller(OperatorController):
        def ensure_runtime(self, *a, **kw):
            entered.set()
            self._transition_sleep(5)
            raise AssertionError("expired preparation reached runtime admission")
        def stop(self, **kw):
            stops.append("STOP")
        def navigate(self, **kw):
            raise AssertionError("no command may be admitted after expiry")
    executor = FiniteNavigationExecutor(Controller(tmp_path))
    def run():
        try:
            results.append(executor.execute("v3.command.move_relative", {"forward_m": 0.1},
                                           cancel_event=cancelled, finite_timeout_s=0.05 if boundary == "deadline" else 1))
        finally:
            done.set()
    worker = threading.Thread(target=run)
    worker.start()
    try:
        assert entered.wait(1)
        if boundary == "cancel":
            cancelled.set()
        assert done.wait(0.5)
        assert results[0]["reason"] == ("TIMEOUT" if boundary == "deadline" else "CANCELLED")
        assert results[0]["command_id"] is None
        assert stops == ["STOP"]
    finally:
        cancelled.set()
        worker.join(1)


def _check_er2_duration_covers_initial_blocking_phases(monkeypatch, phase):
    from r2b4_er2.streaming import Er2StreamingClient
    entered, exited = [], []
    async def blocked():
        entered.append(phase)
        try:
            await asyncio.Event().wait()
        finally:
            exited.append(phase)
    class Session:
        async def send_client_content(self, **kw):
            if phase == "send":
                await blocked()
        async def receive(self):
            await asyncio.Event().wait()
            if False:
                yield None
    class Connect:
        async def __aenter__(self):
            if phase == "connect":
                await blocked()
            return Session()
        async def __aexit__(self, *a):
            pass
    async def observe(**kw):
        if phase == "media":
            await blocked()
        return None
    sdk = SimpleNamespace(aio=SimpleNamespace(live=SimpleNamespace(connect=lambda **kw: Connect())))
    types = SimpleNamespace(**{name: (lambda **kw: SimpleNamespace(**kw)) for name in
                             ("Content", "Part", "LiveConnectConfig", "ContextWindowCompressionConfig",
                              "SlidingWindow", "SessionResumptionConfig")})
    client = Er2StreamingClient(None, SimpleNamespace(observe=observe))
    monkeypatch.setattr(client, "_sdk", lambda: (sdk, types))
    result = asyncio.run(asyncio.wait_for(client.run_async("test", duration_s=0.04), timeout=0.5))
    assert result.stopped_cleanly
    assert entered == exited == [phase]


def _check_preview_timeout_discards_late_robot_tool_response():
    from r2b4_er2.preview import Er2PreviewClient, Er2PreviewError
    entered, release, returned = (threading.Event() for _ in range(3))
    actions = []
    def create(**kwargs):
        entered.set()
        assert release.wait(1)
        returned.set()
        return SimpleNamespace(id="late", steps=[SimpleNamespace(type="function_call", name="robot_move_relative",
                                                               id="call", arguments={"forward_m": 0.1})])
    tools = SimpleNamespace(motion_attempted=False, interaction_tools=lambda: [],
                            execute=lambda *a, **kw: actions.append(a))
    client = Er2PreviewClient(client=SimpleNamespace(interactions=SimpleNamespace(create=create)))
    try:
        with pytest.raises(Er2PreviewError, match="deadline"):
            client.run("move", tools=tools, timeout_s=0.04)
        assert entered.is_set()
    finally:
        release.set()
        assert returned.wait(1)
    assert not actions


def _check_provider_turn_deadline_blocks_late_failover_and_bounds_requests():
    from r2b4_voice.llm_failover import FailoverLLMClient, LLMProviderCandidate
    entered, release, returned = (threading.Event() for _ in range(3))
    calls = []
    def blocked(*a, **kw):
        calls.append("primary")
        entered.set()
        assert release.wait(1)
        returned.set()
        raise TimeoutError("network timeout")
    primary = SimpleNamespace(model="primary", complete_agent_step=blocked)
    fallback = SimpleNamespace(model="fallback", complete_agent_step=lambda *a, **kw: calls.append("fallback"))
    client = FailoverLLMClient([LLMProviderCandidate("primary", primary), LLMProviderCandidate("fallback", fallback)])
    try:
        with pytest.raises(TimeoutError, match="turn deadline"):
            client.complete_agent_step([], [], [], deadline=time.monotonic() + 0.04)
        assert entered.is_set()
        # The previous request still owns the bounded slot: no new request is
        # admitted while its late I/O is held, even from a subsequent turn.
        with pytest.raises(TimeoutError, match="turn deadline"):
            client.complete_agent_step([], [], [], deadline=time.monotonic() + 0.04)
        assert calls == ["primary"]
    finally:
        release.set()
        assert returned.wait(1)
    assert calls == ["primary"]


def _check_tts_startup_budget_includes_ping_and_file_lock(tmp_path, phase):
    from r2b4_voice.piper_tts import PiperTtsConfig
    from r2b4_voice.resident_tts import ResidentPiperTtsClient
    model = tmp_path / "voice.onnx"
    model.write_bytes(b"test")
    (tmp_path / "voice.onnx.json").write_text("{}")
    client = ResidentPiperTtsClient(PiperTtsConfig(model_path=model, voice="test"), project_root=tmp_path,
                                  dependency_dir=tmp_path, startup_timeout_s=0.04, request_timeout_s=1)
    client._socket_path = tmp_path / "tts.sock"
    lock = server = worker = None
    release = threading.Event()
    if phase == "lock":
        lock = client.socket_path.with_suffix(".sock.lock").open("a+b")
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
    else:
        server = socket.socket(socket.AF_UNIX)
        server.bind(str(client.socket_path))
        server.listen(1)
        server.settimeout(1)
        def serve():
            with server.accept()[0] as conn:
                conn.recv(4096)
                release.wait(1)
        worker = threading.Thread(target=serve)
        worker.start()
    try:
        started = time.monotonic()
        assert client.warmup() is False
        assert time.monotonic() - started < 0.4
    finally:
        release.set()
        if lock is not None:
            lock.close()
        if worker is not None:
            worker.join(1)
        if server is not None:
            server.close()


def test_turn_revocation_and_fresh_host_admission(tmp_path, monkeypatch):
    for revocation in ("timeout", "close", "stop", "deadline"):
        with monkeypatch.context() as patch:
            _check_revoked_conversation_cannot_dispatch_late_default_robot_tool(tmp_path, patch, revocation)
    with monkeypatch.context() as patch:
        _check_host_readiness_and_active_preflight_reject_stale_status(tmp_path, patch)
    for boundary in ("deadline", "cancel"):
        _check_finite_navigation_preparation_obeys_total_budget(tmp_path, boundary)
    _check_provider_turn_deadline_blocks_late_failover_and_bounds_requests()


def test_host_operation_budgets_cover_connect_io_lock_and_finish(tmp_path, monkeypatch):
    with monkeypatch.context() as patch:
        _check_host_shutdown_waits_for_allowed_capture_finalization(tmp_path, patch)
    for phase in ("connect", "media", "send"):
        with monkeypatch.context() as patch:
            _check_er2_duration_covers_initial_blocking_phases(patch, phase)
    _check_preview_timeout_discards_late_robot_tool_response()
    for phase in ("ping", "lock"):
        target = tmp_path / phase
        target.mkdir()
        _check_tts_startup_budget_includes_ping_and_file_lock(target, phase)
