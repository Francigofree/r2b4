"""Always-on R2B4 wake + half-duplex conversation supervisor.

Data path:
    NativeUsbMicrophone -> local energy utterance gate -> Groq STT
    -> RobotInterface conversation.submit_text -> Gemini/Groq LLM
    -> LLMDecision proposal -> fresh VoiceActionExecutor gate -> canonical RobotInterface
    -> Gemini TTS -> Linux/PipeWire speaker.

The service is host-side orchestration only.  It never writes motor/GPIO state; optional
LLM proposals can execute only through the fresh-state canonical RobotInterface gate.  One process owns the microphone
for wake and conversation. Normal dialogue remains half-duplex; during
THINKING/SPEAKING only a separate exact-STOP interrupt lane may read the same
bounded microphone frame port. The acoustic tail is discarded before normal listening.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import queue
import signal
import stat
import sys
import tempfile
import threading
import time
from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, Mapping, Protocol

from v3.adapters.microphone import MicrophoneHealth, MicrophoneState, NativeUsbMicrophone
from v3.hri_evidence import HriBehaviorObserver, HriEventJournal

from .action_executor import VoiceActionExecutor
from .conversation_interface import VoiceInterfaceBundle, build_voice_interface
from .gemini_tts import GeminiTtsClient, GeminiTtsConfig
from .groq_stt import GroqWakeTranscriber, WakeTranscriptionError
from .llm_provider import default_model_for, resolve_llm_provider
from .runtime_control import WakeRuntimeCoordinator, WakeRuntimeOutcome
from .safety_intents import is_stop_intent
from .speaker import ReadyWaveSpeaker
from .voice_output import PcmWavePlayer
from .wake_core import EnergyUtteranceBuilder, WakePhraseMatcher, WakeVoiceActivityConfig

# R2B4_HRI_P0_V1


class VoiceServiceState(str, Enum):
    STARTING = "STARTING"
    MIC_RETRY = "MIC_RETRY"
    WAKE_LISTENING = "WAKE_LISTENING"
    WAKE_TRANSCRIBING = "WAKE_TRANSCRIBING"
    STARTING_ROBOT = "STARTING_ROBOT"
    CONVERSATION_LISTENING = "CONVERSATION_LISTENING"
    TRANSCRIBING = "TRANSCRIBING"
    THINKING = "THINKING"
    SPEAKING = "SPEAKING"
    STOPPED = "STOPPED"
    FAILED = "FAILED"


class MicrophonePort(Protocol):
    def read_after(self, sequence: int, timeout_s: float = 1.0): ...


class MicrophoneOwnerPort(Protocol):
    @property
    def port(self) -> MicrophonePort: ...
    def start(self) -> bool: ...
    def stop(self, timeout_s: float = 2.0) -> None: ...
    def health(self) -> MicrophoneHealth: ...


class TranscriberPort(Protocol):
    def transcribe(self, utterance) -> str: ...


class CoordinatorPort(Protocol):
    def robot_running(self) -> bool: ...
    def activate(self): ...


class ConversationInterfacePort(Protocol):
    def execute(self, action: str, **parameters: object) -> object: ...


class ConversationWaitPort(Protocol):
    @property
    def session_id(self) -> str: ...
    def wait_for_turn(self, turn_id: str, timeout_s: float = 20.0) -> dict[str, object] | None: ...


class TtsPort(Protocol):
    @property
    def model(self) -> str: ...
    @property
    def voice(self) -> str: ...
    def synthesize(self, text: str): ...


class PlaybackPort(Protocol):
    def available_player(self) -> str | None: ...
    def play(self, speech) -> str: ...


@dataclass(frozen=True, slots=True)
class VoiceServiceConfig:
    keyword: str = "alba"
    capture_mode: str = "alap"
    microphone_retry_s: float = 2.0
    failure_cooldown_s: float = 1.5
    frame_wait_s: float = 1.0
    llm_timeout_s: float = 25.0
    speaker_settle_s: float = 0.45

    def __post_init__(self) -> None:
        if not self.keyword.strip():
            raise ValueError("keyword must be non-empty")
        if self.capture_mode not in {"alap", "full", "nincs"}:
            raise ValueError("capture_mode must be alap, full or nincs")
        for value, name in (
            (self.microphone_retry_s, "microphone_retry_s"),
            (self.failure_cooldown_s, "failure_cooldown_s"),
            (self.frame_wait_s, "frame_wait_s"),
            (self.llm_timeout_s, "llm_timeout_s"),
            (self.speaker_settle_s, "speaker_settle_s"),
        ):
            if not isinstance(value, (int, float)) or isinstance(value, bool) or float(value) <= 0:
                raise ValueError(f"{name} must be positive")


@dataclass(frozen=True, slots=True)
class VoiceServiceSnapshot:
    state: VoiceServiceState
    pid: int
    microphone_state: str
    frame_sequence: int
    frame_age_ms: float | None
    sequence_gap_count: int
    runtime_running: bool
    conversation_session_id: str
    last_transcript: str | None
    last_spoken_text: str | None
    last_action_status: str | None
    last_error: str | None
    monotonic_ns: int


class VoiceConversationService:
    """One half-duplex wake/conversation loop with a single microphone owner."""

    def __init__(
        self,
        microphone: MicrophoneOwnerPort,
        transcriber: TranscriberPort,
        coordinator: CoordinatorPort,
        conversation_interface: ConversationInterfacePort,
        conversation: ConversationWaitPort,
        tts: TtsPort,
        playback: PlaybackPort,
        *,
        action_executor: VoiceActionExecutor | None = None,
        config: VoiceServiceConfig = VoiceServiceConfig(),
        activity_config: WakeVoiceActivityConfig = WakeVoiceActivityConfig(),
        hri_journal: HriEventJournal | None = None,
        status_file: Path | str | None = None,
        stop_event: threading.Event | None = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        for method in ("start", "stop", "health"):
            if not callable(getattr(microphone, method, None)):
                raise TypeError(f"microphone must provide {method}")
        if not callable(getattr(getattr(microphone, "port", None), "read_after", None)):
            raise TypeError("microphone.port must provide read_after")
        if not callable(getattr(transcriber, "transcribe", None)):
            raise TypeError("transcriber must provide transcribe")
        if not callable(getattr(coordinator, "robot_running", None)) or not callable(
            getattr(coordinator, "activate", None)
        ):
            raise TypeError("coordinator must provide robot_running and activate")
        if not callable(getattr(conversation_interface, "execute", None)):
            raise TypeError("conversation_interface must provide execute")
        if not callable(getattr(conversation, "wait_for_turn", None)):
            raise TypeError("conversation must provide wait_for_turn")
        if not callable(getattr(tts, "synthesize", None)):
            raise TypeError("tts must provide synthesize")
        if not callable(getattr(playback, "play", None)):
            raise TypeError("playback must provide play")
        if not isinstance(config, VoiceServiceConfig):
            raise TypeError("config must be VoiceServiceConfig")

        self._microphone = microphone
        self._transcriber = transcriber
        self._coordinator = coordinator
        self._conversation_interface = conversation_interface
        self._conversation = conversation
        self._tts = tts
        self._playback = playback
        self._action_executor = action_executor
        self._config = config
        self._builder = EnergyUtteranceBuilder(activity_config)
        self._activity_config = activity_config
        self._hri_journal = hri_journal
        self._behavior_feedback_queue = queue.Queue(maxsize=4)
        self._behavior_observer = HriBehaviorObserver(
            conversation_interface, hri_journal, feedback_sink=self._queue_behavior_feedback,
        )
        self._interaction_counter = 0
        self._interrupt_enabled = threading.Event()
        self._stop_interrupt_latched = threading.Event()
        self._interrupt_thread = threading.Thread(
            target=self._interrupt_stop_loop,
            name="r2b4-voice-stop-interrupt",
            daemon=True,
        )
        self._matcher = WakePhraseMatcher(config.keyword)
        self._status_file = Path(status_file) if status_file is not None else None
        self._stop_event = stop_event or threading.Event()
        self._monotonic_ns = monotonic_ns
        self._sleep = sleep
        self._state = VoiceServiceState.STARTING
        self._last_sequence = 0
        self._last_transcript: str | None = None
        self._last_spoken_text: str | None = None
        self._last_action_status: str | None = None
        self._last_error: str | None = None

    def request_stop(self) -> None:
        self._stop_event.set()

    def run_forever(self) -> int:
        self._publish_status()
        if not self._interrupt_thread.is_alive():
            self._interrupt_thread.start()
        try:
            while not self._stop_event.is_set():
                if self._deliver_behavior_feedback():
                    continue
                if not self._ensure_microphone():
                    continue
                if self._robot_running():
                    self._conversation_cycle()
                else:
                    self._wake_cycle()
            return 0
        except Exception as exc:
            self._last_error = f"{type(exc).__name__}: {exc}"
            self._set_state(VoiceServiceState.FAILED)
            return 1
        finally:
            self._interrupt_enabled.clear()
            self._behavior_observer.close()
            self._stop_event.set()
            if self._interrupt_thread.is_alive():
                self._interrupt_thread.join(timeout=1.0)
            try:
                self._microphone.stop()
            except Exception:
                pass
            self._set_state(VoiceServiceState.STOPPED)

    def snapshot(self) -> VoiceServiceSnapshot:
        health = self._microphone.health()
        return VoiceServiceSnapshot(
            state=self._state,
            pid=os.getpid(),
            microphone_state=health.state.value,
            frame_sequence=health.sequence,
            frame_age_ms=health.last_frame_age_ms,
            sequence_gap_count=self._builder.sequence_gap_count,
            runtime_running=self._robot_running(quiet=True),
            conversation_session_id=self._conversation.session_id,
            last_transcript=self._last_transcript,
            last_spoken_text=self._last_spoken_text,
            last_action_status=self._last_action_status,
            last_error=self._last_error,
            monotonic_ns=self._monotonic_ns(),
        )

    def _wake_cycle(self) -> None:
        self._set_state(VoiceServiceState.WAKE_LISTENING)
        utterance = self._read_utterance()
        if utterance is None:
            return
        self._set_state(VoiceServiceState.WAKE_TRANSCRIBING)
        transcript = self._transcribe(utterance)
        if transcript is None:
            return
        self._last_transcript = transcript
        self._publish_status()
        if not self._matcher.matches(transcript):
            return

        print(f"wake: keyword={self._matcher.keyword} transcript={transcript!r}", flush=True)
        self._set_state(VoiceServiceState.STARTING_ROBOT)
        result = self._coordinator.activate()
        if result.outcome is WakeRuntimeOutcome.START_FAILED:
            self._last_error = result.error
            print(f"wake: runtime start failed: {result.error}", file=sys.stderr, flush=True)
            self._interruptible_sleep(self._config.failure_cooldown_s)
            self._discard_audio_history()
            return
        if result.error:
            self._last_error = result.error
            print(f"wake: {result.error}", file=sys.stderr, flush=True)
        else:
            self._last_error = None
        if result.outcome is WakeRuntimeOutcome.STARTED_READY:
            print(
                "wake: robot READY/IDLE"
                + (f"; ack={result.acknowledgement_backend}" if result.acknowledged else "; ack=FAILED"),
                flush=True,
            )
        elif result.outcome is WakeRuntimeOutcome.ALREADY_RUNNING:
            print("wake: runtime already running", flush=True)

        # The coordinator may just have played "Kész vagyok.".  Keep capture
        # ownership but ignore the speaker tail before conversational listening.
        self._settle_and_discard()
        self._set_state(VoiceServiceState.CONVERSATION_LISTENING)

    def _conversation_cycle(self) -> None:
        self._set_state(VoiceServiceState.CONVERSATION_LISTENING)
        utterance = self._read_utterance()
        if utterance is None:
            return
        self._set_state(VoiceServiceState.TRANSCRIBING)
        transcript = self._transcribe(utterance)
        if transcript is None or not transcript.strip():
            self._discard_audio_history()
            return
        self._last_transcript = transcript.strip()
        self._last_error = None
        self._publish_status()
        print(f"voice: user={self._last_transcript!r}", flush=True)
        interaction_id = self._next_interaction_id()
        self._stop_interrupt_latched.clear()
        self._hri_event(
            "STT_RESULT", interaction_id=interaction_id, text=self._last_transcript,
            phase=self._state.value,
        )

        # Deterministic STOP bypasses the LLM/action-proposal path after STT.
        # It still enters only through the canonical RobotInterface command path.
        if is_stop_intent(self._last_transcript):
            self._execute_voice_stop(
                interaction_id=interaction_id, source="FAST_PATH", transcript=self._last_transcript
            )
            self._settle_and_discard()
            return

        self._set_state(VoiceServiceState.THINKING)
        try:
            accepted = self._conversation_interface.execute(
                "conversation.submit_text",
                text=self._last_transcript,
                source="stt",
            )
            if not isinstance(accepted, Mapping) or not isinstance(accepted.get("turn_id"), str):
                raise RuntimeError("conversation.submit_text did not return a turn_id")
            turn_id = accepted["turn_id"]
            self._hri_event(
                "TURN_ACCEPTED", interaction_id=interaction_id, turn_id=turn_id,
                text=self._last_transcript, phase=self._state.value,
            )
            self._interrupt_enabled.set()
            try:
                result = self._conversation.wait_for_turn(
                    turn_id, timeout_s=self._config.llm_timeout_s,
                )
            finally:
                self._interrupt_enabled.clear()
            if result is None:
                raise TimeoutError("conversation turn timed out")
        except Exception as exc:
            self._last_error = f"conversation {type(exc).__name__}: {exc}"
            print(f"voice: {self._last_error}", file=sys.stderr, flush=True)
            self._settle_and_discard()
            return

        self._last_action_status = str(result.get("action_status")) if result.get("action_status") is not None else None
        defer_action_feedback = False
        proposed = result.get("proposed_action")
        if proposed is not None:
            self._hri_event(
                "INTENT_PROPOSED", interaction_id=interaction_id, turn_id=turn_id,
                proposal=proposed, phase=self._state.value,
            )
            if self._stop_interrupt_latched.is_set():
                self._last_action_status = "REJECTED:INTERRUPTED_BY_STOP"
                self._hri_event(
                    "ACTION_REJECTED", interaction_id=interaction_id, turn_id=turn_id,
                    action_status=self._last_action_status, reason="INTERRUPTED_BY_STOP",
                )
                proposed = None
        if proposed is not None:
            mode = self._action_executor.mode.upper() if self._action_executor is not None else "PROPOSAL_ONLY"
            print(
                f"voice: intent[{mode}]=" + json.dumps(proposed, ensure_ascii=False, sort_keys=True),
                flush=True,
            )
            if self._action_executor is not None:
                try:
                    execution = self._action_executor.execute_proposal(proposed)
                    self._last_action_status = execution.status
                    self._hri_event(
                        "ACTION_EXECUTED" if execution.executed else "ACTION_REJECTED",
                        interaction_id=interaction_id, turn_id=turn_id,
                        action_name=execution.action_name, action_status=execution.status,
                        command_id=execution.command_id, mission_id=execution.mission_id,
                    )
                    if (
                        execution.executed and execution.action_name != "v3.command.stop"
                        and execution.command_id and execution.mission_id
                    ):
                        defer_action_feedback = True
                        self._behavior_observer.observe(
                            interaction_id=interaction_id, turn_id=turn_id,
                            session_id=self._conversation.session_id,
                            action_name=execution.action_name, command_id=execution.command_id,
                            mission_id=execution.mission_id,
                        )
                    print(
                        f"voice: action={execution.action_name} status={execution.status} command_id={execution.command_id} mission_id={execution.mission_id}",
                        flush=True,
                    )
                except Exception as exc:
                    # Fail closed: executor failure never falls back to a direct action path.
                    self._last_action_status = "REJECTED:EXECUTOR_ERROR"
                    self._last_error = f"voice action {type(exc).__name__}: {exc}"
                    self._hri_event(
                        "ACTION_REJECTED", interaction_id=interaction_id, turn_id=turn_id,
                        action_status=self._last_action_status, reason=self._last_error,
                    )
                    print(f"voice: {self._last_error}", file=sys.stderr, flush=True)

        error = result.get("error")
        if error:
            self._last_error = str(error)
            print(f"voice: LLM error: {error}", file=sys.stderr, flush=True)
            self._settle_and_discard()
            return

        if self._stop_interrupt_latched.is_set():
            spoken = "Megálltam."
        elif defer_action_feedback:
            spoken = "Rendben."
        else:
            spoken = result.get("spoken_text")
        if not isinstance(spoken, str) or not spoken.strip():
            self._last_spoken_text = None
            self._last_error = None
            self._settle_and_discard()
            return

        self._last_spoken_text = spoken.strip()
        self._hri_event(
            "HRI_RESPONSE", interaction_id=interaction_id, turn_id=turn_id,
            text=self._last_spoken_text, action_status=self._last_action_status,
        )
        self._set_state(VoiceServiceState.SPEAKING)
        self._interrupt_enabled.set()
        try:
            speech = self._tts.synthesize(self._last_spoken_text)
            backend = self._playback.play(speech)
            print(
                f"voice: alba={self._last_spoken_text!r}; tts={self._tts.model}/{self._tts.voice}; player={backend}",
                flush=True,
            )
            self._last_error = None
        except Exception as exc:
            self._last_error = f"speech {type(exc).__name__}: {exc}"
            print(f"voice: {self._last_error}", file=sys.stderr, flush=True)
        finally:
            self._interrupt_enabled.clear()
            # Half-duplex remains the normal dialogue path; the separate exact STOP
            # lane is the only microphone consumer allowed during THINKING/SPEAKING.
            # the configured acoustic tail is discarded before we listen again.
            self._settle_and_discard()


    def _queue_behavior_feedback(self, text: str, fields: Mapping[str, object]) -> None:
        item = (str(text), dict(fields))
        try:
            self._behavior_feedback_queue.put_nowait(item)
        except queue.Full:
            try:
                self._behavior_feedback_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._behavior_feedback_queue.put_nowait(item)
            except queue.Full:
                pass

    def _deliver_behavior_feedback(self) -> bool:
        try:
            text, fields = self._behavior_feedback_queue.get_nowait()
        except queue.Empty:
            return False
        self._last_spoken_text = text
        self._hri_event("BEHAVIOR_FEEDBACK_SPEAKING", text=text, **fields)
        self._set_state(VoiceServiceState.SPEAKING)
        self._interrupt_enabled.set()
        try:
            speech = self._tts.synthesize(text)
            backend = self._playback.play(speech)
            print(f"voice: behavior_feedback={text!r}; player={backend}", flush=True)
            self._last_error = None
        except Exception as exc:
            self._last_error = f"behavior feedback {type(exc).__name__}: {exc}"
            self._hri_event("BEHAVIOR_FEEDBACK_FAILED", text=text, reason=self._last_error, **fields)
            print(f"voice: {self._last_error}", file=sys.stderr, flush=True)
        finally:
            self._interrupt_enabled.clear()
            self._settle_and_discard()
        return True

    def _next_interaction_id(self) -> str:
        self._interaction_counter += 1
        return f"voice-{os.getpid()}-{self._monotonic_ns()}-{self._interaction_counter}"

    def _hri_event(self, event_type: str, **fields: object) -> None:
        if self._hri_journal is None:
            return
        try:
            self._hri_journal.append(
                event_type,
                session_id=self._conversation.session_id,
                **fields,
            )
        except Exception:
            # HRI evidence is passive; it must never perturb dialogue or control.
            return

    def _execute_voice_stop(
        self, *, interaction_id: str, source: str, transcript: str | None = None
    ) -> bool:
        if source == "INTERRUPT":
            self._stop_interrupt_latched.set()
        self._hri_event(
            "STOP_REQUESTED",
            interaction_id=interaction_id,
            source=source,
            transcript=transcript,
            phase=self._state.value,
        )
        try:
            self._conversation_interface.execute("v3.command.stop")
            status = (
                "EXECUTED:VOICE_STOP_INTERRUPT"
                if source == "INTERRUPT"
                else "EXECUTED:VOICE_STOP_FAST_PATH"
            )
            self._last_action_status = status
            self._last_error = None
            self._hri_event(
                "INTERRUPT_STOP_EXECUTED" if source == "INTERRUPT" else "STOP_EXECUTED",
                interaction_id=interaction_id,
                source=source,
                action_name="v3.command.stop",
                action_status=status,
                phase=self._state.value,
            )
            print(f"voice: STOP {source.lower()} executed via RobotInterface", flush=True)
            return True
        except Exception as exc:
            self._last_action_status = "STOP_FAILED"
            self._last_error = f"voice STOP {type(exc).__name__}: {exc}"
            self._hri_event(
                "INTERRUPT_STOP_FAILED" if source == "INTERRUPT" else "STOP_FAILED",
                interaction_id=interaction_id,
                source=source,
                action_name="v3.command.stop",
                action_status=self._last_action_status,
                reason=self._last_error,
                phase=self._state.value,
            )
            print(f"voice: {self._last_error}", file=sys.stderr, flush=True)
            return False

    def _handle_interrupt_transcript(self, transcript: str, *, phase: str) -> bool:
        normalized = transcript.strip()
        if not normalized or not is_stop_intent(normalized):
            return False
        interaction_id = self._next_interaction_id()
        self._hri_event(
            "INTERRUPT_STOP_DETECTED",
            interaction_id=interaction_id,
            transcript=normalized,
            phase=phase,
        )
        return self._execute_voice_stop(
            interaction_id=interaction_id,
            source="INTERRUPT",
            transcript=normalized,
        )

    def _interrupt_stop_loop(self) -> None:
        builder = EnergyUtteranceBuilder(self._activity_config)
        sequence = 0
        lane_active = False
        while not self._stop_event.is_set():
            if not self._interrupt_enabled.is_set():
                lane_active = False
                self._stop_event.wait(0.05)
                continue
            health = self._microphone.health()
            if health.state is not MicrophoneState.CAPTURING:
                self._stop_event.wait(0.05)
                continue
            if not lane_active:
                sequence = health.sequence
                builder.reset(reset_sequence=True)
                lane_active = True
                # Let transient THINKING/SPEAKING phases collapse without touching
                # the audio ring; real speech remains buffered behind this cursor.
                if self._stop_event.wait(0.02):
                    continue
                if not self._interrupt_enabled.is_set():
                    continue
            frame = self._microphone.port.read_after(sequence, timeout_s=0.10)
            if frame is None:
                continue
            sequence = frame.sequence
            try:
                utterance = builder.feed(frame)
            except Exception:
                builder.reset(reset_sequence=True)
                sequence = self._microphone.health().sequence
                continue
            if utterance is None:
                continue
            try:
                transcript = self._transcriber.transcribe(utterance).strip()
            except Exception as exc:
                self._hri_event(
                    "INTERRUPT_STT_ERROR",
                    phase=self._state.value,
                    reason=f"{type(exc).__name__}: {exc}",
                )
                continue
            if self._handle_interrupt_transcript(transcript, phase=self._state.value):
                builder.reset(reset_sequence=True)
                sequence = self._microphone.health().sequence

    def _read_utterance(self):
        frame = self._microphone.port.read_after(
            self._last_sequence,
            timeout_s=self._config.frame_wait_s,
        )
        if frame is None:
            health = self._microphone.health()
            if health.state is not MicrophoneState.CAPTURING:
                self._restart_microphone()
            else:
                self._publish_status()
            return None
        self._last_sequence = frame.sequence
        return self._builder.feed(frame)

    def _transcribe(self, utterance) -> str | None:
        try:
            transcript = self._transcriber.transcribe(utterance)
        except WakeTranscriptionError as exc:
            self._last_error = str(exc)
            print(f"voice: STT error: {exc}", file=sys.stderr, flush=True)
            return None
        except Exception as exc:
            self._last_error = f"STT {type(exc).__name__}: {exc}"
            print(f"voice: {self._last_error}", file=sys.stderr, flush=True)
            return None
        self._last_error = None
        return transcript.strip()

    def _ensure_microphone(self) -> bool:
        health = self._microphone.health()
        if health.state is MicrophoneState.CAPTURING:
            return True
        self._set_state(VoiceServiceState.MIC_RETRY)
        if self._microphone.start():
            self._last_error = None
            self._discard_audio_history()
            return True
        health = self._microphone.health()
        self._last_error = health.last_error or f"microphone state {health.state.value}"
        self._publish_status()
        self._interruptible_sleep(self._config.microphone_retry_s)
        return False

    def _restart_microphone(self) -> None:
        try:
            self._microphone.stop()
        except Exception:
            pass
        self._interruptible_sleep(self._config.microphone_retry_s)

    def _robot_running(self, *, quiet: bool = False) -> bool:
        try:
            return bool(self._coordinator.robot_running())
        except Exception as exc:
            if not quiet:
                self._last_error = f"runtime status {type(exc).__name__}: {exc}"
            return False

    def _settle_and_discard(self) -> None:
        self._interruptible_sleep(self._config.speaker_settle_s)
        self._discard_audio_history()

    def _discard_audio_history(self) -> None:
        health = self._microphone.health()
        self._last_sequence = health.sequence
        self._builder.reset(reset_sequence=True)
        self._publish_status()

    def _set_state(self, state: VoiceServiceState) -> None:
        self._state = state
        self._publish_status()

    def _interruptible_sleep(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while not self._stop_event.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            self._sleep(min(0.1, remaining))

    def _publish_status(self) -> None:
        if self._status_file is None:
            return
        try:
            snapshot = self.snapshot()
            payload = asdict(snapshot)
            payload["state"] = snapshot.state.value
            self._status_file.parent.mkdir(parents=True, exist_ok=True)
            fd, temp_name = tempfile.mkstemp(
                prefix=self._status_file.name + ".",
                dir=self._status_file.parent,
                text=True,
            )
            try:
                os.fchmod(fd, 0o600)
                with os.fdopen(fd, "w", encoding="utf-8") as handle:
                    json.dump(payload, handle, ensure_ascii=False, sort_keys=True)
                    handle.write("\n")
                    handle.flush()
                    os.fsync(handle.fileno())
                os.replace(temp_name, self._status_file)
            finally:
                try:
                    os.unlink(temp_name)
                except FileNotFoundError:
                    pass
        except Exception:
            pass


def _project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _load_project_env(root: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    path = root / "conf" / ".wake.env"
    if not path.is_file():
        return values
    mode = stat.S_IMODE(path.stat().st_mode)
    if mode & 0o077:
        raise RuntimeError(f"secret file permissions are too open: {oct(mode)}; expected 0o600")
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key and value:
            values[key] = value
    return values


def _setting(project_env: Mapping[str, str], name: str) -> str | None:
    value = os.environ.get(name)
    if isinstance(value, str) and value.strip():
        return value.strip()
    value = project_env.get(name)
    return value.strip() if isinstance(value, str) and value.strip() else None


def _resolved_llm(project_env: Mapping[str, str]) -> tuple[str, str, str | None]:
    provider = resolve_llm_provider(_setting(project_env, "R2B4_LLM_PROVIDER"))
    model = _setting(project_env, "R2B4_LLM_MODEL") or default_model_for(provider)
    key_name = "GEMINI_API_KEY" if provider == "gemini" else "GROQ_API_KEY"
    return provider, model, _setting(project_env, key_name)


def _diagnostic_check(root: Path) -> int:
    from v3.adapters.microphone import resolve_usb_microphone

    try:
        project_env = _load_project_env(root)
        provider, model, llm_key = _resolved_llm(project_env)
    except Exception as exc:
        print(json.dumps({"project_root": str(root), "config": "FAIL", "error": str(exc)}, ensure_ascii=False, indent=2))
        return 1

    groq_key = _setting(project_env, "GROQ_API_KEY")
    gemini_key = _setting(project_env, "GEMINI_API_KEY")
    tts_model = _setting(project_env, "R2B4_TTS_MODEL") or "gemini-3.1-flash-tts-preview"
    tts_voice = _setting(project_env, "R2B4_TTS_VOICE") or "Kore"
    result: dict[str, object] = {
        "project_root": str(root),
        "secret_file": str(root / "conf" / ".wake.env"),
        "groq_stt_key": "PASS" if groq_key else "FAIL",
        "llm_provider": provider,
        "llm_model": model,
        "llm_api_key": "PASS" if llm_key else "FAIL",
        "tts_provider": "gemini",
        "tts_model": tts_model,
        "tts_voice": tts_voice,
        "tts_api_key": "PASS" if gemini_key else "FAIL",
        "action_mode": (_setting(project_env, "R2B4_VOICE_ACTION_MODE") or "shadow").lower(),
        "motor_action_execution": (_setting(project_env, "R2B4_VOICE_ACTION_MODE") or "shadow").lower() == "execute",
        "half_duplex_self_hearing_guard": True,
    }
    try:
        identity = resolve_usb_microphone()
        result["microphone"] = {
            "status": "PASS",
            "usb_path": identity.usb_path,
            "alsa_pcm": identity.alsa_pcm_name,
        }
    except Exception as exc:
        result["microphone"] = {"status": "FAIL", "error": f"{type(exc).__name__}: {exc}"}
    player = PcmWavePlayer().available_player()
    result["speaker"] = {"status": "PASS" if player else "FAIL", "player": player}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if all(
        (
            groq_key,
            llm_key,
            gemini_key,
            isinstance(result["microphone"], dict) and result["microphone"].get("status") == "PASS",
            isinstance(result["speaker"], dict) and result["speaker"].get("status") == "PASS",
        )
    ) else 1


def _acquire_instance_lock(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError("another R2B4 wake/voice service instance is already running")
    handle.seek(0)
    handle.truncate()
    handle.write(str(os.getpid()) + "\n")
    handle.flush()
    os.fchmod(handle.fileno(), 0o600)
    return handle


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="r2b4-voice")
    parser.add_argument("--check", action="store_true", help="check mic/keys/LLM/TTS/speaker without network calls")
    parser.add_argument("--keyword", default="alba")
    parser.add_argument("--capture-mode", choices=("alap", "full", "nincs"), default="alap")
    parser.add_argument("--action-mode", choices=("shadow", "execute"), default=None)
    parser.add_argument("--action-watchdog-s", type=float, default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    root = _project_root()
    if args.check:
        return _diagnostic_check(root)

    try:
        project_env = _load_project_env(root)
        groq_key = _setting(project_env, "GROQ_API_KEY")
        gemini_key = _setting(project_env, "GEMINI_API_KEY")
        provider, model, llm_key = _resolved_llm(project_env)
        action_mode = (args.action_mode or _setting(project_env, "R2B4_VOICE_ACTION_MODE") or "shadow").strip().lower()
        if action_mode not in {"shadow", "execute"}:
            raise RuntimeError("R2B4_VOICE_ACTION_MODE must be shadow or execute")
        watchdog_raw = args.action_watchdog_s if args.action_watchdog_s is not None else (_setting(project_env, "R2B4_VOICE_ACTION_WATCHDOG_S") or "30")
        action_watchdog_s = float(watchdog_raw)
        if not 1.0 <= action_watchdog_s <= 600.0:
            raise RuntimeError("voice action watchdog must be within [1, 600] seconds")
        if not groq_key:
            raise RuntimeError("GROQ_API_KEY is required for STT")
        if not llm_key:
            raise RuntimeError(f"missing API key for LLM provider {provider}")
        if not gemini_key:
            raise RuntimeError("GEMINI_API_KEY is required for Gemini TTS")
        transcriber = GroqWakeTranscriber(api_key=groq_key)
        tts = GeminiTtsClient(
            api_key=gemini_key,
            config=GeminiTtsConfig(
                model=_setting(project_env, "R2B4_TTS_MODEL") or "gemini-3.1-flash-tts-preview",
                voice=_setting(project_env, "R2B4_TTS_VOICE") or "Kore",
            ),
        )
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    from v3.operator_controller import OperatorController

    runtime = root / "runtime"
    lock = None
    bundle: VoiceInterfaceBundle | None = None
    service: VoiceConversationService | None = None
    stop_event = threading.Event()
    old_handlers: dict[int, object] = {}

    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()
        if service is not None:
            service.request_stop()

    try:
        lock = _acquire_instance_lock(runtime / ".r2b4_wake.lock")
        controller = OperatorController(project_root=root)
        coordinator = WakeRuntimeCoordinator(
            controller,
            ReadyWaveSpeaker(),
            capture_mode=args.capture_mode,
            idle_timeout_s=3.0,
        )
        bundle = build_voice_interface(
            root,
            api_key=llm_key,
            provider=provider,
            model=model,
        )
        service = VoiceConversationService(
            NativeUsbMicrophone(),
            transcriber,
            coordinator,
            bundle.interface,
            bundle.conversation,
            tts,
            PcmWavePlayer(),
            hri_journal=HriEventJournal((runtime / "hri_events.ndjson").resolve()),
            action_executor=VoiceActionExecutor(
                bundle.interface,
                mode=action_mode,
                session_owner_pid=os.getpid(),
                session_watchdog_s=action_watchdog_s,
            ),
            config=VoiceServiceConfig(keyword=args.keyword, capture_mode=args.capture_mode),
            status_file=runtime / "wake_status.json",
            stop_event=stop_event,
        )
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, request_stop)
        return service.run_forever()
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)  # type: ignore[arg-type]
        if bundle is not None:
            try:
                bundle.close()
            except Exception:
                pass
        if lock is not None:
            try:
                fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
            finally:
                lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
