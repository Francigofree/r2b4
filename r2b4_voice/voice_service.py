"""Always-on R2B4 wake + half-duplex conversation supervisor.

Data path:
    NativeUsbMicrophone -> local energy utterance gate -> Groq STT
    -> RobotInterface conversation.submit_text -> ChatGPT OAuth/OpenAI API key/Gemini/Groq LLM
    -> LLMDecision proposal -> fresh VoiceActionExecutor gate -> canonical RobotInterface
    -> local Piper TTS (Gemini optional) -> Linux/PipeWire speaker.

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

from r2b4_orchestration.execution_mode import (
    ExecutionMode,
    ExecutionModeSelector,
    ExecutionPlan,
    RouteEvidenceJournal,
)
from v3.adapters.microphone import MicrophoneHealth, MicrophoneState, NativeUsbMicrophone
from v3.hri_evidence import HriBehaviorObserver, HriEventJournal

from .action_executor import VoiceActionExecutor
from .conversation_interface import VoiceInterfaceBundle, build_voice_interface
from .tts_provider import build_tts_client, diagnose_tts
from .groq_stt import GroqWakeTranscriber, WakeTranscriptionError
from .llm_provider import api_key_env_for, default_model_for, llm_auth_summary, resolve_llm_provider
from .safety_intents import is_stop_intent
from .voice_output import PcmWavePlayer
from .wake_core import EnergyUtteranceBuilder, WakePhraseMatcher, WakeVoiceActivityConfig

# R2B4_HRI_P0_V1

DEFAULT_ACTION_MODE = "execute"


def _action_receipt_text(*, status: str, executed: bool, action_name: str | None) -> str:
    """Return speech derived only from the fresh executor receipt, never from LLM claims."""
    if executed:
        return "Megálltam." if action_name == "v3.command.stop" else "Rendben."
    if status.startswith("FAILED:"):
        return "A kért robotművelet nem fejeződött be."
    if status == "SHADOW_ACCEPTED":
        return "Értettem, de a végrehajtás teszt módban van."
    return "A parancsot most nem tudom végrehajtani."


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


class RuntimeStatusObserver:
    """Read-only host observer; never starts/stops V3 and owns no robot authority."""

    __slots__ = ("_controller",)

    def __init__(self, controller: object) -> None:
        status = getattr(controller, "status", None)
        if not callable(status):
            raise TypeError("controller must provide status()")
        self._controller = controller

    def robot_running(self) -> bool:
        status = self._controller.status()
        return isinstance(status, Mapping) and status.get("runtime_running") is True


class ConversationInterfacePort(Protocol):
    def read(self, resource: str) -> object: ...
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
    keyword: str = "robot"
    ready_text: str = "figyelek"
    session_silence_s: float = 10.0
    microphone_retry_s: float = 2.0
    frame_wait_s: float = 1.0
    llm_timeout_s: float = 90.0
    speaker_settle_s: float = 0.45

    def __post_init__(self) -> None:
        if not self.keyword.strip():
            raise ValueError("keyword must be non-empty")
        if not self.ready_text.strip():
            raise ValueError("ready_text must be non-empty")
        for value, name in (
            (self.session_silence_s, "session_silence_s"),
            (self.microphone_retry_s, "microphone_retry_s"),
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
    session_open: bool
    session_silence_remaining_ms: float | None
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
        if not callable(getattr(coordinator, "robot_running", None)):
            raise TypeError("coordinator must provide robot_running")
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
        self._mode_selector = ExecutionModeSelector()
        self._route_evidence = RouteEvidenceJournal(Path(__file__).resolve().parents[1])
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
        self._session_open = False
        self._session_deadline_ns: int | None = None

    def request_stop(self) -> None:
        self._stop_event.set()
        self._stop_interrupt_latched.set()
        cancel = getattr(self._conversation, "cancel_pending_turns", None)
        if callable(cancel):
            cancel()

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
                if self._session_open:
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
        now_ns = self._monotonic_ns()
        remaining_ms: float | None = None
        if self._session_open and self._session_deadline_ns is not None:
            remaining_ms = max(0.0, (self._session_deadline_ns - now_ns) / 1_000_000.0)
        return VoiceServiceSnapshot(
            state=self._state,
            pid=os.getpid(),
            microphone_state=health.state.value,
            frame_sequence=health.sequence,
            frame_age_ms=health.last_frame_age_ms,
            sequence_gap_count=self._builder.sequence_gap_count,
            runtime_running=self._robot_running(quiet=True),
            session_open=self._session_open,
            session_silence_remaining_ms=remaining_ms,
            conversation_session_id=self._conversation.session_id,
            last_transcript=self._last_transcript,
            last_spoken_text=self._last_spoken_text,
            last_action_status=self._last_action_status,
            last_error=self._last_error,
            monotonic_ns=now_ns,
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
        self._open_session()
        self._last_spoken_text = self._config.ready_text.strip()
        self._hri_event(
            "WAKE_SESSION_OPENED",
            text=self._last_spoken_text,
            runtime_running=self._robot_running(quiet=True),
        )
        self._set_state(VoiceServiceState.SPEAKING)
        self._interrupt_enabled.set()
        try:
            speech = self._tts.synthesize(self._last_spoken_text)
            backend = self._playback.play(speech)
            print(
                f"wake: ready={self._last_spoken_text!r}; tts={self._tts.model}/{self._tts.voice}; player={backend}",
                flush=True,
            )
            self._last_error = None
        except Exception as exc:
            # Ready audio is capability-local; a failed speaker must not start or fault V3.
            self._last_error = f"ready speech {type(exc).__name__}: {exc}"
            print(f"wake: {self._last_error}", file=sys.stderr, flush=True)
        finally:
            self._interrupt_enabled.clear()
            self._settle_and_discard()
            if self._session_open:
                self._resume_session_timeout()
        self._set_state(VoiceServiceState.CONVERSATION_LISTENING)

    def _conversation_cycle(self) -> None:
        self._set_state(VoiceServiceState.CONVERSATION_LISTENING)
        utterance = self._read_utterance()
        if self._session_expired():
            self._close_session("silence-timeout")
            return
        if utterance is None:
            if self._session_expired():
                self._close_session("silence-timeout")
            return
        # Silence timeout applies only while actively listening. Network/STT/TTS
        # latency must not consume the user's next 10 second listen window.
        self._pause_session_timeout()
        self._set_state(VoiceServiceState.TRANSCRIBING)
        transcript = self._transcribe(utterance)
        if transcript is None or not transcript.strip():
            self._discard_audio_history()
            self._resume_session_timeout()
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

        plan = self._mode_selector.select(self._last_transcript, source="voice")
        self._route_evidence.emit("ROUTE_SELECTED", plan, interaction_id=interaction_id)
        self._hri_event(
            "EXECUTION_MODE_SELECTED",
            interaction_id=interaction_id,
            route_id=plan.route_id,
            execution_mode=plan.mode.value,
            route_reason=plan.reason,
            requires_v3=plan.requires_v3,
            capability=plan.capability,
        )

        # Deterministic STOP bypasses the LLM/action-proposal path after STT.
        # It still enters only through the canonical RobotInterface command path.
        if is_stop_intent(self._last_transcript):
            executed = self._execute_voice_stop(
                interaction_id=interaction_id, source="FAST_PATH", transcript=self._last_transcript
            )
            self._route_evidence.emit(
                "ROUTE_EXECUTION_COMPLETE", plan, interaction_id=interaction_id,
                executed=executed, action_status=self._last_action_status,
            )
            self._settle_and_discard()
            self._resume_session_timeout()
            return

        if plan.mode is not ExecutionMode.GEMINI_CHAT:
            self._execute_selected_plan(plan, interaction_id=interaction_id)
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
            self._resume_session_timeout()
            return

        self._last_action_status = str(result.get("action_status")) if result.get("action_status") is not None else None
        action_spoken: str | None = None
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
                    action_spoken = _action_receipt_text(
                        status=execution.status,
                        executed=execution.executed,
                        action_name=execution.action_name,
                    )
                    self._hri_event(
                        "ACTION_EXECUTED" if execution.executed else "ACTION_REJECTED",
                        interaction_id=interaction_id, turn_id=turn_id,
                        action_name=execution.action_name, action_status=execution.status,
                        command_id=execution.command_id, mission_id=execution.mission_id,
                    )
                    if (
                        execution.executed and execution.status != "COMPLETED"
                        and execution.action_name != "v3.command.stop"
                        and execution.command_id and execution.mission_id
                    ):
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
                    action_spoken = _action_receipt_text(
                        status=self._last_action_status, executed=False, action_name=None
                    )
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
            self._resume_session_timeout()
            return

        if self._stop_interrupt_latched.is_set():
            spoken = "Megálltam."
        elif action_spoken is not None:
            spoken = action_spoken
        else:
            spoken = result.get("spoken_text")
        if not isinstance(spoken, str) or not spoken.strip():
            self._last_spoken_text = None
            self._last_error = None
            self._settle_and_discard()
            self._resume_session_timeout()
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
            self._resume_session_timeout()

    def _execute_selected_plan(self, plan: ExecutionPlan, *, interaction_id: str) -> None:
        """Execute a non-chat selector result without falling back to Gemini."""
        self._route_evidence.emit("ROUTE_EXECUTION_START", plan, interaction_id=interaction_id)
        try:
            if plan.mode is ExecutionMode.DIRECT_V3:
                if self._action_executor is None:
                    self._last_action_status = "REJECTED:NO_ACTION_EXECUTOR"
                    self._speak_routed_response(
                        "A parancsot most nem tudom végrehajtani.",
                        interaction_id=interaction_id,
                        plan=plan,
                    )
                    return
                execution = self._action_executor.execute_proposal({
                    "name": plan.action_name,
                    "parameters": dict(plan.action_parameters),
                })
                self._last_action_status = execution.status
                self._hri_event(
                    "ACTION_EXECUTED" if execution.executed else "ACTION_REJECTED",
                    interaction_id=interaction_id,
                    route_id=plan.route_id,
                    action_name=execution.action_name,
                    action_status=execution.status,
                    command_id=execution.command_id,
                    mission_id=execution.mission_id,
                )
                if (
                    execution.executed and execution.status != "COMPLETED"
                    and execution.action_name != "v3.command.stop"
                    and execution.command_id and execution.mission_id
                ):
                    self._behavior_observer.observe(
                        interaction_id=interaction_id,
                        turn_id=plan.route_id,
                        session_id=self._conversation.session_id,
                        action_name=execution.action_name,
                        command_id=execution.command_id,
                        mission_id=execution.mission_id,
                    )
                self._route_evidence.emit(
                    "ROUTE_EXECUTION_COMPLETE",
                    plan,
                    interaction_id=interaction_id,
                    action_status=execution.status,
                    executed=execution.executed,
                    command_id=execution.command_id,
                    mission_id=execution.mission_id,
                )
                self._speak_routed_response(
                    _action_receipt_text(
                        status=execution.status,
                        executed=execution.executed,
                        action_name=execution.action_name,
                    ),
                    interaction_id=interaction_id,
                    plan=plan,
                )
                return

            if plan.mode is ExecutionMode.HOST_READ:
                reader = getattr(self._conversation_interface, "read", None)
                if not callable(reader):
                    raise RuntimeError("conversation interface has no read()")
                payload = reader(plan.capability or "operator.status")
                from r2b4_orchestration.executor import status_text
                text = status_text(payload)
                self._route_evidence.emit(
                    "ROUTE_EXECUTION_COMPLETE", plan, interaction_id=interaction_id,
                )
                self._speak_routed_response(text, interaction_id=interaction_id, plan=plan)
                return

            if plan.mode in {
                ExecutionMode.OBSERVATION,
                ExecutionMode.ER2_PREVIEW,
                ExecutionMode.ER2_STREAM,
            }:
                if (
                    plan.mode is ExecutionMode.ER2_STREAM
                    and (self._action_executor is None or self._action_executor.mode != "execute")
                ):
                    self._last_action_status = "SHADOW_ACCEPTED"
                    self._route_evidence.emit(
                        "ROUTE_EXECUTION_COMPLETE", plan, interaction_id=interaction_id,
                        action_status=self._last_action_status, executed=False,
                    )
                    self._speak_routed_response(
                        "Értettem, de a végrehajtás teszt módban van.",
                        interaction_id=interaction_id, plan=plan,
                    )
                    return
                from r2b4_er2.executor import run_er2_task

                self._set_state(VoiceServiceState.THINKING)
                self._interrupt_enabled.set()
                try:
                    result = run_er2_task(
                        plan.text,
                        project_root=Path(__file__).resolve().parents[1],
                        mode=(
                            "preview"
                            if plan.mode in {ExecutionMode.OBSERVATION, ExecutionMode.ER2_PREVIEW}
                            else "stream"
                        ),
                        camera=plan.camera,
                        tools_enabled=plan.tools,
                        cancel_event=self._stop_interrupt_latched,
                        duration_s=self._config.llm_timeout_s,
                    )
                finally:
                    self._interrupt_enabled.clear()
                if not result.text:
                    raise RuntimeError("ER2 returned no final text")
                self._route_evidence.emit(
                    "ROUTE_EXECUTION_COMPLETE",
                    plan,
                    interaction_id=interaction_id,
                    er2_mode=result.mode,
                    reconnect_count=result.reconnect_count,
                    stopped_cleanly=result.stopped_cleanly,
                )
                self._speak_routed_response(result.text, interaction_id=interaction_id, plan=plan)
                return

            raise RuntimeError(f"unsupported selected execution mode: {plan.mode.value}")
        except Exception as exc:
            self._last_action_status = "REJECTED:ROUTE_EXECUTOR_ERROR"
            self._last_error = f"execution route {type(exc).__name__}: {exc}"
            self._route_evidence.emit(
                "ROUTE_EXECUTION_ERROR",
                plan,
                interaction_id=interaction_id,
                error_type=type(exc).__name__,
                error=str(exc)[:500],
            )
            self._hri_event(
                "EXECUTION_MODE_FAILED",
                interaction_id=interaction_id,
                route_id=plan.route_id,
                execution_mode=plan.mode.value,
                reason=self._last_error,
            )
            print(f"voice: {self._last_error}", file=sys.stderr, flush=True)
            self._speak_routed_response(
                "A kérést most nem tudom végrehajtani.",
                interaction_id=interaction_id,
                plan=plan,
            )

    def _speak_routed_response(
        self, text: str, *, interaction_id: str, plan: ExecutionPlan
    ) -> None:
        spoken = text.strip() if isinstance(text, str) else ""
        if not spoken:
            self._settle_and_discard()
            self._resume_session_timeout()
            return
        self._last_spoken_text = spoken
        self._hri_event(
            "HRI_RESPONSE",
            interaction_id=interaction_id,
            route_id=plan.route_id,
            text=spoken,
            action_status=self._last_action_status,
        )
        self._set_state(VoiceServiceState.SPEAKING)
        self._interrupt_enabled.set()
        try:
            speech = self._tts.synthesize(spoken)
            backend = self._playback.play(speech)
            print(
                f"voice: alba={spoken!r}; route={plan.mode.value}; "
                f"tts={self._tts.model}/{self._tts.voice}; player={backend}",
                flush=True,
            )
            self._last_error = None
        except Exception as exc:
            self._last_error = f"speech {type(exc).__name__}: {exc}"
            print(f"voice: {self._last_error}", file=sys.stderr, flush=True)
        finally:
            self._interrupt_enabled.clear()
            self._settle_and_discard()
            self._resume_session_timeout()


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
        if self._session_open:
            self._pause_session_timeout()
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
            if self._session_open:
                self._resume_session_timeout()
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
        self._stop_interrupt_latched.set()
        cancel = getattr(self._conversation, "cancel_pending_turns", None)
        if callable(cancel):
            cancel()
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
        wait_s = self._config.frame_wait_s
        if self._session_open and self._session_deadline_ns is not None:
            wait_s = min(wait_s, max(0.0, (self._session_deadline_ns - self._monotonic_ns()) / 1_000_000_000))
        frame = self._microphone.port.read_after(
            self._last_sequence,
            timeout_s=wait_s,
        )
        if frame is None:
            health = self._microphone.health()
            if health.state is not MicrophoneState.CAPTURING:
                self._restart_microphone()
            else:
                self._publish_status()
            return None
        self._last_sequence = frame.sequence
        utterance = self._builder.feed(frame)
        if (self._session_open and self._builder.last_frame_voiced
                and self._session_deadline_ns is not None
                and frame.read_monotonic_ns <= self._session_deadline_ns):
            # Continuous silence starts at physical PCM activity, rather than
            # completion of an utterance or late consumption of buffered audio.
            self._session_deadline_ns = min(frame.read_monotonic_ns, self._monotonic_ns()) + int(
                self._config.session_silence_s * 1_000_000_000
            )
            self._publish_status()
        return utterance

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
        if self._session_open:
            self._pause_session_timeout()
        self._set_state(VoiceServiceState.MIC_RETRY)
        if self._microphone.start():
            self._last_error = None
            self._discard_audio_history()
            if self._session_open:
                self._resume_session_timeout()
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

    def _open_session(self) -> None:
        self._session_open = True
        self._session_deadline_ns = None
        self._publish_status()

    def _close_session(self, reason: str) -> None:
        was_open = self._session_open
        self._session_open = False
        self._session_deadline_ns = None
        self._stop_interrupt_latched.set()
        cancel = getattr(self._conversation, "cancel_pending_turns", None)
        if callable(cancel):
            cancel()
        if was_open:
            self._hri_event("WAKE_SESSION_CLOSED", reason=reason)
        self._set_state(VoiceServiceState.WAKE_LISTENING)
        self._discard_audio_history()

    def _pause_session_timeout(self) -> None:
        if self._session_open:
            self._session_deadline_ns = None
            self._publish_status()

    def _resume_session_timeout(self) -> None:
        if not self._session_open:
            return
        self._session_deadline_ns = self._monotonic_ns() + int(
            self._config.session_silence_s * 1_000_000_000
        )
        self._publish_status()

    def _session_expired(self) -> bool:
        return (
            self._session_open
            and self._session_deadline_ns is not None
            and self._monotonic_ns() >= self._session_deadline_ns
        )

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
    key_name = api_key_env_for(provider)
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
    auth = llm_auth_summary(root, project_env=project_env)
    try:
        tts_diagnostic = diagnose_tts(root, project_env, gemini_api_key=gemini_key)
    except Exception as exc:
        tts_diagnostic = {
            "tts_provider": _setting(project_env, "R2B4_TTS_PROVIDER") or "piper",
            "tts_status": "FAIL",
            "tts_error": f"{type(exc).__name__}: {exc}",
        }
    result: dict[str, object] = {
        "project_root": str(root),
        "secret_file": str(root / "conf" / ".wake.env"),
        "groq_stt_key": "PASS" if groq_key else "FAIL",
        "llm_provider": provider,
        "llm_model": model,
        "llm_api_key": "OPTIONAL:PRESENT" if llm_key else "OPTIONAL:MISSING",
        "llm_auth": auth,
        **tts_diagnostic,
        "action_mode": (_setting(project_env, "R2B4_VOICE_ACTION_MODE") or DEFAULT_ACTION_MODE).lower(),
        "motor_action_execution": (_setting(project_env, "R2B4_VOICE_ACTION_MODE") or DEFAULT_ACTION_MODE).lower() == "execute",
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
            auth.get("llm_ready") is True,
            tts_diagnostic.get("tts_status") == "PASS",
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
    parser.add_argument("--keyword", default="robot")
    parser.add_argument("--ready-text", default="figyelek")
    parser.add_argument("--session-silence-s", type=float, default=10.0)
    parser.add_argument("--action-mode", choices=("shadow", "execute"), default=None)
    parser.add_argument("--action-watchdog-s", type=float, default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    root = _project_root()
    from v3.runtime_performance import apply_host_affinity
    apply_host_affinity(root, "voice")
    if args.check:
        return _diagnostic_check(root)

    try:
        project_env = _load_project_env(root)
        groq_key = _setting(project_env, "GROQ_API_KEY")
        gemini_key = _setting(project_env, "GEMINI_API_KEY")
        provider, model, llm_key = _resolved_llm(project_env)
        auth = llm_auth_summary(root, project_env=project_env)
        action_mode = (args.action_mode or _setting(project_env, "R2B4_VOICE_ACTION_MODE") or DEFAULT_ACTION_MODE).strip().lower()
        if action_mode not in {"shadow", "execute"}:
            raise RuntimeError("R2B4_VOICE_ACTION_MODE must be shadow or execute")
        watchdog_raw = args.action_watchdog_s if args.action_watchdog_s is not None else (_setting(project_env, "R2B4_VOICE_ACTION_WATCHDOG_S") or "30")
        action_watchdog_s = float(watchdog_raw)
        if not 1.0 <= action_watchdog_s <= 600.0:
            raise RuntimeError("voice action watchdog must be within [1, 600] seconds")
        if not groq_key:
            raise RuntimeError("GROQ_API_KEY is required for STT")
        if auth.get("llm_ready") is not True:
            raise RuntimeError("no usable LLM authentication; run ./r chatgpt login or configure a fallback API key")
        transcriber = GroqWakeTranscriber(api_key=groq_key)
        tts = build_tts_client(
            root,
            project_env,
            gemini_api_key=gemini_key,
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
        coordinator = RuntimeStatusObserver(controller)
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
            config=VoiceServiceConfig(
                keyword=args.keyword,
                ready_text=args.ready_text,
                session_silence_s=args.session_silence_s,
            ),
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
