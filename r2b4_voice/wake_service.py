"""Always-on Linux wake service for the R2B4 robot.

The service is host-side orchestration.  It is the single microphone owner while
running, listens for the spoken wake token "Alba", and requests robot startup
through the public OperatorController lifecycle API.  It never writes commands,
motors or V3 layer state directly.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import signal
import sys
import tempfile
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable, Protocol

from v3.adapters.microphone import AudioFramePort, MicrophoneHealth, MicrophoneState, NativeUsbMicrophone

from .groq_stt import GroqWakeTranscriber, WakeTranscriptionError
from .runtime_control import WakeRuntimeCoordinator, WakeRuntimeOutcome
from .speaker import ReadyWaveSpeaker
from .wake_core import (
    EnergyUtteranceBuilder,
    WakePhraseMatcher,
    WakeServiceState,
    WakeVoiceActivityConfig,
)


class MicrophoneOwnerPort(Protocol):
    @property
    def port(self) -> AudioFramePort: ...
    def start(self) -> bool: ...
    def stop(self, timeout_s: float = 2.0) -> None: ...
    def health(self) -> MicrophoneHealth: ...


class WakeTranscriberPort(Protocol):
    def transcribe(self, utterance) -> str: ...


class WakeCoordinatorPort(Protocol):
    def robot_running(self) -> bool: ...
    def activate(self): ...


@dataclass(frozen=True, slots=True)
class WakeServiceConfig:
    keyword: str = "alba"
    capture_mode: str = "alap"
    microphone_retry_s: float = 2.0
    runtime_poll_s: float = 0.5
    failure_cooldown_s: float = 1.5
    frame_wait_s: float = 1.0
    runtime_idle_timeout_s: float = 3.0

    def __post_init__(self) -> None:
        if not self.keyword.strip():
            raise ValueError("keyword must be non-empty")
        if self.capture_mode not in {"alap", "full", "nincs"}:
            raise ValueError("capture_mode must be alap, full or nincs")
        for value, name in (
            (self.microphone_retry_s, "microphone_retry_s"),
            (self.runtime_poll_s, "runtime_poll_s"),
            (self.failure_cooldown_s, "failure_cooldown_s"),
            (self.frame_wait_s, "frame_wait_s"),
            (self.runtime_idle_timeout_s, "runtime_idle_timeout_s"),
        ):
            if value <= 0:
                raise ValueError(f"{name} must be positive")


@dataclass(frozen=True, slots=True)
class WakeServiceSnapshot:
    state: WakeServiceState
    pid: int
    microphone_state: str
    frame_sequence: int
    frame_age_ms: float | None
    sequence_gap_count: int
    runtime_running: bool
    last_transcript: str | None
    last_error: str | None
    monotonic_ns: int


class WakeService:
    __slots__ = (
        "_builder",
        "_config",
        "_coordinator",
        "_last_error",
        "_last_sequence",
        "_last_transcript",
        "_matcher",
        "_microphone",
        "_monotonic_ns",
        "_sleep",
        "_state",
        "_status_file",
        "_stop_event",
        "_transcriber",
    )

    def __init__(
        self,
        microphone: MicrophoneOwnerPort,
        transcriber: WakeTranscriberPort,
        coordinator: WakeCoordinatorPort,
        *,
        config: WakeServiceConfig = WakeServiceConfig(),
        activity_config: WakeVoiceActivityConfig = WakeVoiceActivityConfig(),
        status_file: Path | str | None = None,
        stop_event: threading.Event | None = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        for method in ("start", "stop", "health"):
            if not callable(getattr(microphone, method, None)):
                raise TypeError(f"microphone must provide {method}")
        if not isinstance(getattr(microphone, "port", None), AudioFramePort):
            raise TypeError("microphone.port must be AudioFramePort")
        if not callable(getattr(transcriber, "transcribe", None)):
            raise TypeError("transcriber must provide transcribe")
        if not callable(getattr(coordinator, "robot_running", None)) or not callable(
            getattr(coordinator, "activate", None)
        ):
            raise TypeError("coordinator must provide robot_running and activate")
        if not isinstance(config, WakeServiceConfig):
            raise TypeError("config must be WakeServiceConfig")
        self._microphone = microphone
        self._transcriber = transcriber
        self._coordinator = coordinator
        self._config = config
        self._builder = EnergyUtteranceBuilder(activity_config)
        self._matcher = WakePhraseMatcher(config.keyword)
        self._status_file = Path(status_file) if status_file is not None else None
        self._stop_event = stop_event or threading.Event()
        self._monotonic_ns = monotonic_ns
        self._sleep = sleep
        self._state = WakeServiceState.STARTING
        self._last_sequence = 0
        self._last_transcript: str | None = None
        self._last_error: str | None = None

    def request_stop(self) -> None:
        self._stop_event.set()

    def run_forever(self) -> int:
        self._publish_status()
        try:
            while not self._stop_event.is_set():
                if not self._ensure_microphone():
                    continue
                if self._coordinator.robot_running():
                    self._set_state(WakeServiceState.ROBOT_READY)
                    self._wait_until_robot_stops()
                    self._discard_audio_history()
                    continue
                self._set_state(WakeServiceState.LISTENING)
                if not self._listen_until_transition():
                    self._restart_microphone()
            return 0
        except Exception as exc:
            self._last_error = f"{type(exc).__name__}: {exc}"
            self._set_state(WakeServiceState.FAILED)
            return 1
        finally:
            try:
                self._microphone.stop()
            except Exception:
                pass
            self._set_state(WakeServiceState.STOPPED)

    def snapshot(self) -> WakeServiceSnapshot:
        health = self._microphone.health()
        try:
            runtime_running = self._coordinator.robot_running()
        except Exception:
            runtime_running = False
        return WakeServiceSnapshot(
            state=self._state,
            pid=os.getpid(),
            microphone_state=health.state.value,
            frame_sequence=health.sequence,
            frame_age_ms=health.last_frame_age_ms,
            sequence_gap_count=self._builder.sequence_gap_count,
            runtime_running=runtime_running,
            last_transcript=self._last_transcript,
            last_error=self._last_error,
            monotonic_ns=self._monotonic_ns(),
        )

    def _ensure_microphone(self) -> bool:
        health = self._microphone.health()
        if health.state is MicrophoneState.CAPTURING:
            return True
        self._set_state(WakeServiceState.MIC_RETRY)
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

    def _listen_until_transition(self) -> bool:
        frame = self._microphone.port.read_after(
            self._last_sequence,
            timeout_s=self._config.frame_wait_s,
        )
        if frame is None:
            health = self._microphone.health()
            self._publish_status()
            return health.state is MicrophoneState.CAPTURING
        self._last_sequence = frame.sequence
        utterance = self._builder.feed(frame)
        if utterance is None:
            return True

        self._set_state(WakeServiceState.TRANSCRIBING)
        try:
            transcript = self._transcriber.transcribe(utterance)
        except WakeTranscriptionError as exc:
            self._last_error = str(exc)
            self._set_state(WakeServiceState.LISTENING)
            return True
        self._last_transcript = transcript
        self._last_error = None
        self._publish_status()
        if not self._matcher.matches(transcript):
            self._set_state(WakeServiceState.LISTENING)
            return True

        print(f"wake: keyword={self._matcher.keyword} transcript={transcript!r}", flush=True)
        self._set_state(WakeServiceState.STARTING_ROBOT)
        result = self._coordinator.activate()
        if result.outcome is WakeRuntimeOutcome.START_FAILED:
            self._last_error = result.error
            print(f"wake: runtime start failed: {result.error}", file=sys.stderr, flush=True)
            self._interruptible_sleep(self._config.failure_cooldown_s)
            self._discard_audio_history()
            self._set_state(WakeServiceState.LISTENING)
            return True

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
        self._set_state(WakeServiceState.ROBOT_READY)
        return True

    def _wait_until_robot_stops(self) -> None:
        while not self._stop_event.is_set():
            try:
                if not self._coordinator.robot_running():
                    return
            except Exception as exc:
                self._last_error = f"runtime status {type(exc).__name__}: {exc}"
            self._publish_status()
            self._interruptible_sleep(self._config.runtime_poll_s)

    def _discard_audio_history(self) -> None:
        health = self._microphone.health()
        self._last_sequence = health.sequence
        self._builder.reset(reset_sequence=True)
        self._publish_status()

    def _set_state(self, state: WakeServiceState) -> None:
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
            # Status publication is diagnostic-only and cannot own wake/runtime
            # lifecycle authority.
            pass


def _project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _diagnostic_check(root: Path) -> int:
    from v3.adapters.microphone import resolve_usb_microphone

    result: dict[str, object] = {"project_root": str(root)}
    try:
        identity = resolve_usb_microphone()
        result["microphone"] = {
            "status": "PASS",
            "usb_path": identity.usb_path,
            "alsa_pcm": identity.alsa_pcm_name,
        }
    except Exception as exc:
        result["microphone"] = {"status": "FAIL", "error": f"{type(exc).__name__}: {exc}"}
    result["groq_api_key"] = "PASS" if os.environ.get("GROQ_API_KEY", "").strip() else "FAIL"
    speaker = ReadyWaveSpeaker()
    result["speaker"] = {
        "status": "PASS" if speaker.available_player() else "FAIL",
        "player": speaker.available_player(),
        "asset": str(speaker.asset),
        "asset_exists": speaker.asset.is_file(),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if all(
        (
            isinstance(result["microphone"], dict) and result["microphone"].get("status") == "PASS",
            result["groq_api_key"] == "PASS",
            isinstance(result["speaker"], dict)
            and result["speaker"].get("status") == "PASS"
            and result["speaker"].get("asset_exists") is True,
        )
    ) else 1


def _acquire_instance_lock(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError("another R2B4 wake service instance is already running")
    handle.seek(0)
    handle.truncate()
    handle.write(str(os.getpid()) + "\n")
    handle.flush()
    os.fchmod(handle.fileno(), 0o600)
    return handle


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="r2b4-wake")
    parser.add_argument("--check", action="store_true", help="check mic/API-key/speaker without starting the service")
    parser.add_argument("--keyword", default="alba")
    parser.add_argument("--capture-mode", choices=("alap", "full", "nincs"), default="alap")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    root = _project_root()
    if args.check:
        return _diagnostic_check(root)

    try:
        transcriber = GroqWakeTranscriber()
    except WakeTranscriptionError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    # Import only at the host composition edge.  The wake package itself does
    # not become a V3 production dependency.
    from v3.operator_controller import OperatorController

    runtime = root / "runtime"
    lock = None
    service: WakeService | None = None
    stop_event = threading.Event()
    old_handlers: dict[int, object] = {}

    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()
        if service is not None:
            service.request_stop()

    try:
        lock = _acquire_instance_lock(runtime / ".r2b4_wake.lock")
        controller = OperatorController(project_root=root)
        speaker = ReadyWaveSpeaker()
        coordinator = WakeRuntimeCoordinator(
            controller,
            speaker,
            capture_mode=args.capture_mode,
            idle_timeout_s=3.0,
        )
        service = WakeService(
            NativeUsbMicrophone(),
            transcriber,
            coordinator,
            config=WakeServiceConfig(keyword=args.keyword, capture_mode=args.capture_mode),
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
        if lock is not None:
            try:
                fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
            finally:
                lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
