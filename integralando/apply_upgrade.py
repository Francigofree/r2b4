#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import os
import shutil
import subprocess
import sys
from pathlib import Path

BASE_COMMIT = "9f2df99f3a772e47ba5f84804889e9eb9e5ee86a"
VOICE_SERVICE_BLOB = "401912c790ddf03436ae3bd3de0ecbc557aa8e86"
TARGET = Path("r2b4_voice/voice_service.py")
TEST_TARGET = Path("tests/feature/test_voice_orchestration_p0.py")

TEST_SOURCE = r'''from __future__ import annotations

from dataclasses import dataclass

import pytest

from r2b4_voice.voice_service import (
    VoiceConversationService,
    VoiceServiceConfig,
    VoiceServiceState,
)
from v3.adapters.microphone import MicrophoneHealth, MicrophoneState


class FakePort:
    def read_after(self, sequence: int, timeout_s: float = 1.0):
        return None


class FakeMicrophone:
    def __init__(self) -> None:
        self.port = FakePort()
        self.sequence = 7
        self.started = True

    def start(self) -> bool:
        self.started = True
        return True

    def stop(self, timeout_s: float = 2.0) -> None:
        self.started = False

    def health(self) -> MicrophoneHealth:
        return MicrophoneHealth(
            state=MicrophoneState.CAPTURING if self.started else MicrophoneState.STOPPED,
            device_present=True,
            sequence=self.sequence,
            last_frame_monotonic_ns=1,
            last_frame_age_ms=0.0,
            ring_overwrite_count=0,
            last_error=None,
        )


class FakeTranscriber:
    def __init__(self, text: str = "robot") -> None:
        self.text = text

    def transcribe(self, utterance) -> str:
        return self.text


class FakeRuntimeStatus:
    """Deliberately has no activate() method: wake must not require one."""

    def __init__(self, running: bool) -> None:
        self.running = running

    def robot_running(self) -> bool:
        return self.running


class FakeInterface:
    def __init__(self) -> None:
        self.executed: list[tuple[str, dict[str, object]]] = []

    def execute(self, action: str, **parameters: object):
        self.executed.append((action, dict(parameters)))
        if action == "conversation.submit_text":
            return {"turn_id": "turn-1"}
        return {"status": "OK"}

    def read(self, resource: str):
        if resource == "v3.status":
            return {"state": "RUNNING"}
        raise KeyError(resource)


class FakeConversation:
    session_id = "test-session"

    def wait_for_turn(self, turn_id: str, timeout_s: float = 20.0):
        return {
            "action_status": "NONE",
            "proposed_action": None,
            "spoken_text": "rendben",
            "error": None,
        }


class FakeTts:
    model = "fake-local"
    voice = "test"

    def __init__(self) -> None:
        self.texts: list[str] = []

    def synthesize(self, text: str):
        self.texts.append(text)
        return text.encode("utf-8")


class FakePlayback:
    def __init__(self) -> None:
        self.items: list[object] = []

    def available_player(self) -> str | None:
        return "fake"

    def play(self, speech) -> str:
        self.items.append(speech)
        return "fake"


@dataclass
class Clock:
    now_ns: int = 1_000_000_000

    def monotonic_ns(self) -> int:
        return self.now_ns


def build_service(*, running: bool, clock: Clock):
    interface = FakeInterface()
    tts = FakeTts()
    playback = FakePlayback()
    service = VoiceConversationService(
        FakeMicrophone(),
        FakeTranscriber(),
        FakeRuntimeStatus(running),
        interface,
        FakeConversation(),
        tts,
        playback,
        config=VoiceServiceConfig(
            session_silence_s=10.0,
            speaker_settle_s=0.01,
        ),
        monotonic_ns=clock.monotonic_ns,
    )
    return service, interface, tts, playback


def test_contract_defaults_are_robot_figyelek_and_ten_seconds() -> None:
    config = VoiceServiceConfig()
    assert config.keyword == "robot"
    assert config.ready_text == "figyelek"
    assert config.session_silence_s == 10.0


@pytest.mark.parametrize("runtime_running", [False, True])
def test_wake_opens_voice_session_without_runtime_activation(
    monkeypatch: pytest.MonkeyPatch, runtime_running: bool
) -> None:
    clock = Clock()
    service, interface, tts, playback = build_service(running=runtime_running, clock=clock)
    monkeypatch.setattr(service, "_read_utterance", lambda: object())
    monkeypatch.setattr(service, "_settle_and_discard", lambda: None)

    service._wake_cycle()

    snap = service.snapshot()
    assert snap.session_open is True
    assert snap.state is VoiceServiceState.CONVERSATION_LISTENING
    assert tts.texts == ["figyelek"]
    assert len(playback.items) == 1
    assert interface.executed == []
    assert snap.runtime_running is runtime_running


def test_session_timeout_returns_to_wake_without_stopping_runtime(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = Clock()
    service, interface, _tts, _playback = build_service(running=True, clock=clock)
    monkeypatch.setattr(service, "_discard_audio_history", lambda: None)

    service._open_session()
    service._resume_session_timeout()
    clock.now_ns += 10_000_000_001

    service._conversation_cycle()

    snap = service.snapshot()
    assert snap.session_open is False
    assert snap.state is VoiceServiceState.WAKE_LISTENING
    assert interface.executed == []
    assert snap.runtime_running is True
'''


def run(cmd: list[str], cwd: Path, *, capture: bool = False) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=cwd,
        check=False,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.PIPE if capture else None,
    )


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one source match, found {count}")
    return text.replace(old, new, 1)


def transform_voice_service(text: str) -> str:
    text = replace_once(
        text,
        'from .runtime_control import WakeRuntimeCoordinator, WakeRuntimeOutcome\n',
        '',
        'remove wake runtime coordinator import',
    )
    text = replace_once(
        text,
        'from .speaker import ReadyWaveSpeaker\n',
        '',
        'remove ready-wave runtime acknowledgement import',
    )

    text = replace_once(
        text,
        '''class CoordinatorPort(Protocol):\n    def robot_running(self) -> bool: ...\n    def activate(self): ...\n\n\n''',
        '''class CoordinatorPort(Protocol):\n    def robot_running(self) -> bool: ...\n\n\nclass RuntimeStatusObserver:\n    \"\"\"Read-only host observer; never starts/stops V3 and owns no robot authority.\"\"\"\n\n    __slots__ = (\"_controller\",)\n\n    def __init__(self, controller: object) -> None:\n        status = getattr(controller, \"status\", None)\n        if not callable(status):\n            raise TypeError(\"controller must provide status()\")\n        self._controller = controller\n\n    def robot_running(self) -> bool:\n        status = self._controller.status()\n        return isinstance(status, Mapping) and status.get(\"runtime_running\") is True\n\n\n''',
        'runtime status observer',
    )

    text = replace_once(
        text,
        '''@dataclass(frozen=True, slots=True)\nclass VoiceServiceConfig:\n    keyword: str = \"alba\"\n    capture_mode: str = \"alap\"\n    microphone_retry_s: float = 2.0\n    failure_cooldown_s: float = 1.5\n    frame_wait_s: float = 1.0\n    llm_timeout_s: float = 25.0\n    speaker_settle_s: float = 0.45\n\n    def __post_init__(self) -> None:\n        if not self.keyword.strip():\n            raise ValueError(\"keyword must be non-empty\")\n        if self.capture_mode not in {\"alap\", \"full\", \"nincs\"}:\n            raise ValueError(\"capture_mode must be alap, full or nincs\")\n        for value, name in (\n            (self.microphone_retry_s, \"microphone_retry_s\"),\n            (self.failure_cooldown_s, \"failure_cooldown_s\"),\n            (self.frame_wait_s, \"frame_wait_s\"),\n            (self.llm_timeout_s, \"llm_timeout_s\"),\n            (self.speaker_settle_s, \"speaker_settle_s\"),\n        ):\n            if not isinstance(value, (int, float)) or isinstance(value, bool) or float(value) <= 0:\n                raise ValueError(f\"{name} must be positive\")\n''',
        '''@dataclass(frozen=True, slots=True)\nclass VoiceServiceConfig:\n    keyword: str = \"robot\"\n    ready_text: str = \"figyelek\"\n    session_silence_s: float = 10.0\n    microphone_retry_s: float = 2.0\n    frame_wait_s: float = 1.0\n    llm_timeout_s: float = 25.0\n    speaker_settle_s: float = 0.45\n\n    def __post_init__(self) -> None:\n        if not self.keyword.strip():\n            raise ValueError(\"keyword must be non-empty\")\n        if not self.ready_text.strip():\n            raise ValueError(\"ready_text must be non-empty\")\n        for value, name in (\n            (self.session_silence_s, \"session_silence_s\"),\n            (self.microphone_retry_s, \"microphone_retry_s\"),\n            (self.frame_wait_s, \"frame_wait_s\"),\n            (self.llm_timeout_s, \"llm_timeout_s\"),\n            (self.speaker_settle_s, \"speaker_settle_s\"),\n        ):\n            if not isinstance(value, (int, float)) or isinstance(value, bool) or float(value) <= 0:\n                raise ValueError(f\"{name} must be positive\")\n''',
        'voice service config defaults',
    )

    text = replace_once(
        text,
        '''    runtime_running: bool\n    conversation_session_id: str\n''',
        '''    runtime_running: bool\n    session_open: bool\n    session_silence_remaining_ms: float | None\n    conversation_session_id: str\n''',
        'session fields in status snapshot',
    )

    text = replace_once(
        text,
        '''        if not callable(getattr(coordinator, \"robot_running\", None)) or not callable(\n            getattr(coordinator, \"activate\", None)\n        ):\n            raise TypeError(\"coordinator must provide robot_running and activate\")\n''',
        '''        if not callable(getattr(coordinator, \"robot_running\", None)):\n            raise TypeError(\"coordinator must provide robot_running\")\n''',
        'coordinator validation',
    )

    text = replace_once(
        text,
        '''        self._last_action_status: str | None = None\n        self._last_error: str | None = None\n''',
        '''        self._last_action_status: str | None = None\n        self._last_error: str | None = None\n        self._session_open = False\n        self._session_deadline_ns: int | None = None\n''',
        'session state initialization',
    )

    text = replace_once(
        text,
        '''                if self._robot_running():\n                    self._conversation_cycle()\n                else:\n                    self._wake_cycle()\n''',
        '''                if self._session_open:\n                    self._conversation_cycle()\n                else:\n                    self._wake_cycle()\n''',
        'main orchestration loop',
    )

    text = replace_once(
        text,
        '''    def snapshot(self) -> VoiceServiceSnapshot:\n        health = self._microphone.health()\n        return VoiceServiceSnapshot(\n            state=self._state,\n            pid=os.getpid(),\n            microphone_state=health.state.value,\n            frame_sequence=health.sequence,\n            frame_age_ms=health.last_frame_age_ms,\n            sequence_gap_count=self._builder.sequence_gap_count,\n            runtime_running=self._robot_running(quiet=True),\n            conversation_session_id=self._conversation.session_id,\n            last_transcript=self._last_transcript,\n            last_spoken_text=self._last_spoken_text,\n            last_action_status=self._last_action_status,\n            last_error=self._last_error,\n            monotonic_ns=self._monotonic_ns(),\n        )\n''',
        '''    def snapshot(self) -> VoiceServiceSnapshot:\n        health = self._microphone.health()\n        now_ns = self._monotonic_ns()\n        remaining_ms: float | None = None\n        if self._session_open and self._session_deadline_ns is not None:\n            remaining_ms = max(0.0, (self._session_deadline_ns - now_ns) / 1_000_000.0)\n        return VoiceServiceSnapshot(\n            state=self._state,\n            pid=os.getpid(),\n            microphone_state=health.state.value,\n            frame_sequence=health.sequence,\n            frame_age_ms=health.last_frame_age_ms,\n            sequence_gap_count=self._builder.sequence_gap_count,\n            runtime_running=self._robot_running(quiet=True),\n            session_open=self._session_open,\n            session_silence_remaining_ms=remaining_ms,\n            conversation_session_id=self._conversation.session_id,\n            last_transcript=self._last_transcript,\n            last_spoken_text=self._last_spoken_text,\n            last_action_status=self._last_action_status,\n            last_error=self._last_error,\n            monotonic_ns=now_ns,\n        )\n''',
        'snapshot session status',
    )

    old_wake = '''    def _wake_cycle(self) -> None:\n        self._set_state(VoiceServiceState.WAKE_LISTENING)\n        utterance = self._read_utterance()\n        if utterance is None:\n            return\n        self._set_state(VoiceServiceState.WAKE_TRANSCRIBING)\n        transcript = self._transcribe(utterance)\n        if transcript is None:\n            return\n        self._last_transcript = transcript\n        self._publish_status()\n        if not self._matcher.matches(transcript):\n            return\n\n        print(f\"wake: keyword={self._matcher.keyword} transcript={transcript!r}\", flush=True)\n        self._set_state(VoiceServiceState.STARTING_ROBOT)\n        result = self._coordinator.activate()\n        if result.outcome is WakeRuntimeOutcome.START_FAILED:\n            self._last_error = result.error\n            print(f\"wake: runtime start failed: {result.error}\", file=sys.stderr, flush=True)\n            self._interruptible_sleep(self._config.failure_cooldown_s)\n            self._discard_audio_history()\n            return\n        if result.error:\n            self._last_error = result.error\n            print(f\"wake: {result.error}\", file=sys.stderr, flush=True)\n        else:\n            self._last_error = None\n        if result.outcome is WakeRuntimeOutcome.STARTED_READY:\n            print(\n                \"wake: robot READY/IDLE\"\n                + (f\"; ack={result.acknowledgement_backend}\" if result.acknowledged else \"; ack=FAILED\"),\n                flush=True,\n            )\n        elif result.outcome is WakeRuntimeOutcome.ALREADY_RUNNING:\n            print(\"wake: runtime already running\", flush=True)\n\n        # The coordinator may just have played \"Kész vagyok.\".  Keep capture\n        # ownership but ignore the speaker tail before conversational listening.\n        self._settle_and_discard()\n        self._set_state(VoiceServiceState.CONVERSATION_LISTENING)\n'''
    new_wake = '''    def _wake_cycle(self) -> None:\n        self._set_state(VoiceServiceState.WAKE_LISTENING)\n        utterance = self._read_utterance()\n        if utterance is None:\n            return\n        self._set_state(VoiceServiceState.WAKE_TRANSCRIBING)\n        transcript = self._transcribe(utterance)\n        if transcript is None:\n            return\n        self._last_transcript = transcript\n        self._publish_status()\n        if not self._matcher.matches(transcript):\n            return\n\n        print(f\"wake: keyword={self._matcher.keyword} transcript={transcript!r}\", flush=True)\n        self._open_session()\n        self._last_spoken_text = self._config.ready_text.strip()\n        self._hri_event(\n            \"WAKE_SESSION_OPENED\",\n            text=self._last_spoken_text,\n            runtime_running=self._robot_running(quiet=True),\n        )\n        self._set_state(VoiceServiceState.SPEAKING)\n        self._interrupt_enabled.set()\n        try:\n            speech = self._tts.synthesize(self._last_spoken_text)\n            backend = self._playback.play(speech)\n            print(\n                f\"wake: ready={self._last_spoken_text!r}; tts={self._tts.model}/{self._tts.voice}; player={backend}\",\n                flush=True,\n            )\n            self._last_error = None\n        except Exception as exc:\n            # Ready audio is capability-local; a failed speaker must not start or fault V3.\n            self._last_error = f\"ready speech {type(exc).__name__}: {exc}\"\n            print(f\"wake: {self._last_error}\", file=sys.stderr, flush=True)\n        finally:\n            self._interrupt_enabled.clear()\n            self._settle_and_discard()\n            if self._session_open:\n                self._resume_session_timeout()\n        self._set_state(VoiceServiceState.CONVERSATION_LISTENING)\n'''
    text = replace_once(text, old_wake, new_wake, 'wake no longer starts V3')

    text = replace_once(
        text,
        '''    def _conversation_cycle(self) -> None:\n        self._set_state(VoiceServiceState.CONVERSATION_LISTENING)\n        utterance = self._read_utterance()\n        if utterance is None:\n            return\n        self._set_state(VoiceServiceState.TRANSCRIBING)\n        transcript = self._transcribe(utterance)\n        if transcript is None or not transcript.strip():\n            self._discard_audio_history()\n            return\n''',
        '''    def _conversation_cycle(self) -> None:\n        if self._session_expired():\n            self._close_session(\"silence-timeout\")\n            return\n        self._set_state(VoiceServiceState.CONVERSATION_LISTENING)\n        utterance = self._read_utterance()\n        if utterance is None:\n            if self._session_expired():\n                self._close_session(\"silence-timeout\")\n            return\n        # Silence timeout applies only while actively listening. Network/STT/TTS\n        # latency must not consume the user's next 10 second listen window.\n        self._pause_session_timeout()\n        self._set_state(VoiceServiceState.TRANSCRIBING)\n        transcript = self._transcribe(utterance)\n        if transcript is None or not transcript.strip():\n            self._discard_audio_history()\n            self._resume_session_timeout()\n            return\n''',
        'conversation session timeout entry',
    )

    text = replace_once(
        text,
        '''            self._settle_and_discard()\n            return\n\n        self._set_state(VoiceServiceState.THINKING)\n''',
        '''            self._settle_and_discard()\n            self._resume_session_timeout()\n            return\n\n        self._set_state(VoiceServiceState.THINKING)\n''',
        'resume session after stop fast path',
    )

    text = replace_once(
        text,
        '''            print(f\"voice: {self._last_error}\", file=sys.stderr, flush=True)\n            self._settle_and_discard()\n            return\n\n        self._last_action_status = str(result.get(\"action_status\")) if result.get(\"action_status\") is not None else None\n''',
        '''            print(f\"voice: {self._last_error}\", file=sys.stderr, flush=True)\n            self._settle_and_discard()\n            self._resume_session_timeout()\n            return\n\n        self._last_action_status = str(result.get(\"action_status\")) if result.get(\"action_status\") is not None else None\n''',
        'resume session after conversation error',
    )

    text = replace_once(
        text,
        '''            print(f\"voice: LLM error: {error}\", file=sys.stderr, flush=True)\n            self._settle_and_discard()\n            return\n''',
        '''            print(f\"voice: LLM error: {error}\", file=sys.stderr, flush=True)\n            self._settle_and_discard()\n            self._resume_session_timeout()\n            return\n''',
        'resume session after LLM error',
    )

    text = replace_once(
        text,
        '''            self._last_spoken_text = None\n            self._last_error = None\n            self._settle_and_discard()\n            return\n''',
        '''            self._last_spoken_text = None\n            self._last_error = None\n            self._settle_and_discard()\n            self._resume_session_timeout()\n            return\n''',
        'resume session after empty response',
    )

    text = replace_once(
        text,
        '''            # the configured acoustic tail is discarded before we listen again.\n            self._settle_and_discard()\n\n\n    def _queue_behavior_feedback''',
        '''            # the configured acoustic tail is discarded before we listen again.\n            self._settle_and_discard()\n            self._resume_session_timeout()\n\n\n    def _queue_behavior_feedback''',
        'resume session after spoken response',
    )

    text = replace_once(
        text,
        '''        self._last_spoken_text = text\n        self._hri_event(\"BEHAVIOR_FEEDBACK_SPEAKING\", text=text, **fields)\n''',
        '''        if self._session_open:\n            self._pause_session_timeout()\n        self._last_spoken_text = text\n        self._hri_event(\"BEHAVIOR_FEEDBACK_SPEAKING\", text=text, **fields)\n''',
        'pause timeout for behavior feedback speech',
    )

    text = replace_once(
        text,
        '''        finally:\n            self._interrupt_enabled.clear()\n            self._settle_and_discard()\n        return True\n''',
        '''        finally:\n            self._interrupt_enabled.clear()\n            self._settle_and_discard()\n            if self._session_open:\n                self._resume_session_timeout()\n        return True\n''',
        'resume timeout after behavior feedback',
    )

    text = replace_once(
        text,
        '''    def _ensure_microphone(self) -> bool:\n        health = self._microphone.health()\n        if health.state is MicrophoneState.CAPTURING:\n            return True\n        self._set_state(VoiceServiceState.MIC_RETRY)\n        if self._microphone.start():\n            self._last_error = None\n            self._discard_audio_history()\n            return True\n        health = self._microphone.health()\n''',
        '''    def _ensure_microphone(self) -> bool:\n        health = self._microphone.health()\n        if health.state is MicrophoneState.CAPTURING:\n            return True\n        if self._session_open:\n            self._pause_session_timeout()\n        self._set_state(VoiceServiceState.MIC_RETRY)\n        if self._microphone.start():\n            self._last_error = None\n            self._discard_audio_history()\n            if self._session_open:\n                self._resume_session_timeout()\n            return True\n        health = self._microphone.health()\n''',
        'pause silence timeout while microphone is unavailable',
    )

    text = replace_once(
        text,
        '''    def _robot_running(self, *, quiet: bool = False) -> bool:\n''',
        '''    def _open_session(self) -> None:\n        self._session_open = True\n        self._session_deadline_ns = None\n        self._publish_status()\n\n    def _close_session(self, reason: str) -> None:\n        was_open = self._session_open\n        self._session_open = False\n        self._session_deadline_ns = None\n        if was_open:\n            self._hri_event(\"WAKE_SESSION_CLOSED\", reason=reason)\n        self._set_state(VoiceServiceState.WAKE_LISTENING)\n        self._discard_audio_history()\n\n    def _pause_session_timeout(self) -> None:\n        if self._session_open:\n            self._session_deadline_ns = None\n            self._publish_status()\n\n    def _resume_session_timeout(self) -> None:\n        if not self._session_open:\n            return\n        self._session_deadline_ns = self._monotonic_ns() + int(\n            self._config.session_silence_s * 1_000_000_000\n        )\n        self._publish_status()\n\n    def _session_expired(self) -> bool:\n        return (\n            self._session_open\n            and self._session_deadline_ns is not None\n            and self._monotonic_ns() >= self._session_deadline_ns\n        )\n\n    def _robot_running(self, *, quiet: bool = False) -> bool:\n''',
        'voice session helpers',
    )

    text = replace_once(
        text,
        '''    parser.add_argument(\"--keyword\", default=\"alba\")\n    parser.add_argument(\"--capture-mode\", choices=(\"alap\", \"full\", \"nincs\"), default=\"alap\")\n    parser.add_argument(\"--action-mode\", choices=(\"shadow\", \"execute\"), default=None)\n''',
        '''    parser.add_argument(\"--keyword\", default=\"robot\")\n    parser.add_argument(\"--ready-text\", default=\"figyelek\")\n    parser.add_argument(\"--session-silence-s\", type=float, default=10.0)\n    parser.add_argument(\"--action-mode\", choices=(\"shadow\", \"execute\"), default=None)\n''',
        'voice CLI orchestration defaults',
    )

    text = replace_once(
        text,
        '''        controller = OperatorController(project_root=root)\n        coordinator = WakeRuntimeCoordinator(\n            controller,\n            ReadyWaveSpeaker(),\n            capture_mode=args.capture_mode,\n            idle_timeout_s=3.0,\n        )\n''',
        '''        controller = OperatorController(project_root=root)\n        coordinator = RuntimeStatusObserver(controller)\n''',
        'production read-only runtime observer',
    )

    text = replace_once(
        text,
        '''            config=VoiceServiceConfig(keyword=args.keyword, capture_mode=args.capture_mode),\n''',
        '''            config=VoiceServiceConfig(\n                keyword=args.keyword,\n                ready_text=args.ready_text,\n                session_silence_s=args.session_silence_s,\n            ),\n''',
        'production voice session config',
    )

    return text


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Apply R2B4 voice orchestration P0 upgrade")
    parser.add_argument("repo", nargs="?", default=".", help="R2B4 repository root")
    parser.add_argument("--no-head-check", action="store_true", help="allow a different HEAD if the target file blob is still exact")
    parser.add_argument("--run-test", action="store_true", help="run the new targeted pytest after applying")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = Path(args.repo).expanduser().resolve()
    if not (root / ".git").exists() or not (root / TARGET).is_file():
        print(f"ERROR: not an R2B4 git checkout: {root}", file=sys.stderr)
        return 2

    head = run(["git", "rev-parse", "HEAD"], root, capture=True)
    if head.returncode != 0:
        print("ERROR: cannot resolve git HEAD", file=sys.stderr)
        return 2
    head_sha = head.stdout.strip()
    if not args.no_head_check and head_sha != BASE_COMMIT:
        print(f"ERROR: expected HEAD {BASE_COMMIT}, got {head_sha}", file=sys.stderr)
        print("Re-run source-first against the newer repo, or use --no-head-check only if the target blob is unchanged.", file=sys.stderr)
        return 3

    blob = run(["git", "hash-object", str(TARGET)], root, capture=True)
    if blob.returncode != 0 or blob.stdout.strip() != VOICE_SERVICE_BLOB:
        print(
            f"ERROR: {TARGET} is not the expected source blob {VOICE_SERVICE_BLOB}; refusing to patch drifted/local-edited source.",
            file=sys.stderr,
        )
        return 4
    if (root / TEST_TARGET).exists():
        print(f"ERROR: test target already exists: {TEST_TARGET}", file=sys.stderr)
        return 5

    original = (root / TARGET).read_text(encoding="utf-8")
    try:
        updated = transform_voice_service(original)
    except Exception as exc:
        print(f"ERROR: source transform failed before write: {exc}", file=sys.stderr)
        return 6

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_root = root / ".upgrade_backups" / f"voice_orchestration_p0_v1_{stamp}"
    (backup_root / TARGET.parent).mkdir(parents=True, exist_ok=True)
    shutil.copy2(root / TARGET, backup_root / TARGET)

    (root / TARGET).write_text(updated, encoding="utf-8")
    (root / TEST_TARGET).write_text(TEST_SOURCE, encoding="utf-8")

    compile_result = run(
        [sys.executable, "-m", "py_compile", str(TARGET), str(TEST_TARGET)],
        root,
        capture=True,
    )
    if compile_result.returncode != 0:
        shutil.copy2(backup_root / TARGET, root / TARGET)
        try:
            (root / TEST_TARGET).unlink()
        except FileNotFoundError:
            pass
        print("ERROR: py_compile failed; source restored.", file=sys.stderr)
        print((compile_result.stderr or compile_result.stdout).strip(), file=sys.stderr)
        return 7

    print(f"APPLIED: {TARGET}")
    print(f"ADDED:   {TEST_TARGET}")
    print(f"BACKUP:  {backup_root}")
    print("VALIDATION: py_compile PASS")

    if args.run_test:
        test_result = run(
            [sys.executable, "-m", "pytest", "-q", str(TEST_TARGET)],
            root,
            capture=False,
        )
        if test_result.returncode != 0:
            print("ERROR: targeted pytest failed; files are left applied for inspection.", file=sys.stderr)
            return 8
        print("VALIDATION: targeted pytest PASS")
    else:
        print(f"NEXT: ./{'r'} pytest -q {TEST_TARGET}")
        print("NEXT: ./r test")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
