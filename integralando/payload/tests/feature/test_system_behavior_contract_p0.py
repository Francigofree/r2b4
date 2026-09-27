from __future__ import annotations

from pathlib import Path

import pytest

from r2b4_voice.conversation_contracts import LLMDecision, RobotAction, RobotContextSnapshot
from r2b4_voice.execution_mode import ExecutionMode, ExecutionModeSelector
from r2b4_voice.llm_decision import parse_llm_decision
from r2b4_voice.voice_service import VoiceConversationService, VoiceServiceConfig
from v3.adapters.camera import CameraInterfaceAdapter


def _context(*, observations=(), actions=()):
    return RobotContextSnapshot(
        schema="TEST",
        runtime={"state": "STOPPED", "fault_layer": None},
        pose=None,
        safety=None,
        health=(),
        available_actions=tuple(actions),
        host={"runtime_running": False},
        available_observations=tuple(observations),
    )


def test_p0_defaults_are_contract_defaults():
    config = VoiceServiceConfig()
    assert config.keyword == "robot"
    assert config.ready_text == "figyelek"
    assert config.session_silence_timeout_s == 10.0


def test_execution_selector_keeps_conversation_and_observation_separate_from_robot_action():
    selector = ExecutionModeSelector()
    assert selector.select(LLMDecision("szia", None, "fake"), _context()).mode is ExecutionMode.CONVERSATION

    obs = {
        "name": "camera.photo",
        "kind": "action",
        "capability_class": "observation",
        "available": True,
        "ready": True,
    }
    plan = selector.select(
        LLMDecision(None, None, "fake", observation_name="camera.photo"),
        _context(observations=(obs,)),
    )
    assert plan.mode is ExecutionMode.OBSERVATION
    assert plan.observation_name == "camera.photo"

    action = RobotAction("v3.command.forward")
    action_cap = {
        "name": "v3.command.forward",
        "available": True,
        "ready": True,
    }
    assert (
        selector.select(
            LLMDecision(None, action, "fake"),
            _context(actions=(action_cap,)),
        ).mode
        is ExecutionMode.ROBOT_ACTION
    )


def test_llm_observation_must_come_from_current_observation_catalog():
    raw = {
        "spoken_text": None,
        "observation_name": "camera.photo",
        "action_name": None,
        "action_parameters": {},
    }
    obs = [{
        "name": "camera.photo",
        "kind": "action",
        "capability_class": "observation",
        "available": True,
        "ready": True,
    }]
    decision = parse_llm_decision(
        raw,
        model="fake",
        action_catalog=(),
        observation_catalog=obs,
    )
    assert decision.observation_name == "camera.photo"
    with pytest.raises(ValueError):
        parse_llm_decision(
            raw,
            model="fake",
            action_catalog=(),
            observation_catalog=(),
        )


class _Controller:
    def __init__(self, running: bool):
        self.running = running
        self.root = Path("/tmp")
    def status(self):
        return {"runtime_running": self.running}
    class _Transition:
        def __enter__(self): return None
        def __exit__(self, *_): return None
    def operator_transition(self):
        return self._Transition()


def test_camera_photo_is_observation_only_when_v3_is_off():
    off = CameraInterfaceAdapter(_Controller(False)).capabilities()["camera.photo"]
    assert off["available"] is True
    assert off["ready"] is True
    assert off["capability_class"] == "observation"

    on = CameraInterfaceAdapter(_Controller(True)).capabilities()["camera.photo"]
    assert on["available"] is False
    assert on["ready"] is False
    assert on["reason"] == "OWNED_BY_V3_RUNTIME"


class _Port:
    def read_after(self, *_args, **_kwargs):
        return None


class _Mic:
    port = _Port()
    def start(self): return True
    def stop(self, timeout_s=2.0): return None
    def health(self):
        class H:
            state = type("S", (), {"value": "CAPTURING"})()
            sequence = 0
            last_frame_age_ms = 0.0
            last_error = None
        return H()


class _Transcriber:
    def transcribe(self, _utterance): return "robot"


class _Coordinator:
    def __init__(self):
        self.activate_calls = 0
    def robot_running(self): return False
    def activate(self):
        self.activate_calls += 1
        raise AssertionError("wake must not start V3")


class _Iface:
    def execute(self, *_args, **_kwargs): return {"status": "OK"}
    def read(self, *_args, **_kwargs): return {}


class _Conversation:
    session_id = "test-session"
    def wait_for_turn(self, *_args, **_kwargs): return None


class _Tts:
    model = "fake"
    voice = "fake"
    def synthesize(self, text): return text


class _Playback:
    def available_player(self): return "fake"
    def play(self, _speech): return "fake"


def _service(clock):
    coordinator = _Coordinator()
    service = VoiceConversationService(
        _Mic(),
        _Transcriber(),
        coordinator,
        _Iface(),
        _Conversation(),
        _Tts(),
        _Playback(),
        config=VoiceServiceConfig(),
        monotonic_ns=lambda: clock[0],
        sleep=lambda _s: None,
    )
    return service, coordinator


def test_wake_opens_voice_session_without_runtime_activation():
    clock = [1_000_000_000]
    service, coordinator = _service(clock)
    service._read_utterance = lambda: object()
    service._transcribe = lambda _utterance: "robot"
    service._speak_ready_feedback = lambda: None
    service._wake_cycle()
    assert coordinator.activate_calls == 0
    assert service._session_active is True
    assert service._state.value == "CONVERSATION_LISTENING"
    service._behavior_observer.close()


def test_voice_session_timeout_is_own_clock_not_runtime_state():
    clock = [5_000_000_000]
    service, coordinator = _service(clock)
    service._open_session()
    assert coordinator.activate_calls == 0
    clock[0] += 9_999_000_000
    assert service._session_expired() is False
    clock[0] += 2_000_000
    assert service._session_expired() is True
    service._behavior_observer.close()
