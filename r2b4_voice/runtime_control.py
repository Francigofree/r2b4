"""Canonical host-side bridge from a wake event to R2B4 runtime readiness."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Mapping, Protocol


class RuntimeControllerPort(Protocol):
    def status(self) -> dict[str, object]: ...
    def runtime_start(self, capture_mode: str = "alap") -> int: ...
    def wait_idle(self, timeout: float = 3.0) -> None: ...


class ReadySpeakerPort(Protocol):
    def play_ready(self) -> str: ...


class WakeRuntimeOutcome(str, Enum):
    STARTED_READY = "STARTED_READY"
    ALREADY_RUNNING = "ALREADY_RUNNING"
    START_FAILED = "START_FAILED"


@dataclass(frozen=True, slots=True)
class WakeRuntimeResult:
    outcome: WakeRuntimeOutcome
    runtime_pid: int | None
    ready_confirmed: bool
    acknowledged: bool
    acknowledgement_backend: str | None
    error: str | None = None


class WakeRuntimeCoordinator:
    """Use only the public operator lifecycle API; never bypass V3 startup."""

    __slots__ = ("_capture_mode", "_controller", "_idle_timeout_s", "_speaker")

    def __init__(
        self,
        controller: RuntimeControllerPort,
        speaker: ReadySpeakerPort,
        *,
        capture_mode: str = "alap",
        idle_timeout_s: float = 3.0,
    ) -> None:
        if not callable(getattr(controller, "status", None)):
            raise TypeError("controller must provide status")
        if not callable(getattr(controller, "runtime_start", None)):
            raise TypeError("controller must provide runtime_start")
        if not callable(getattr(controller, "wait_idle", None)):
            raise TypeError("controller must provide wait_idle")
        if not callable(getattr(speaker, "play_ready", None)):
            raise TypeError("speaker must provide play_ready")
        if capture_mode not in {"alap", "full", "nincs"}:
            raise ValueError("capture_mode must be alap, full or nincs")
        if idle_timeout_s <= 0:
            raise ValueError("idle_timeout_s must be positive")
        self._controller = controller
        self._speaker = speaker
        self._capture_mode = capture_mode
        self._idle_timeout_s = float(idle_timeout_s)

    def robot_running(self) -> bool:
        status = self._controller.status()
        return status.get("runtime_running") is True

    def activate(self) -> WakeRuntimeResult:
        try:
            current = self._controller.status()
            if current.get("runtime_running") is True:
                return WakeRuntimeResult(
                    WakeRuntimeOutcome.ALREADY_RUNNING,
                    self._pid(current),
                    False,
                    False,
                    None,
                )
            pid = self._controller.runtime_start(self._capture_mode)
            self._controller.wait_idle(timeout=self._idle_timeout_s)
        except Exception as exc:
            return WakeRuntimeResult(
                WakeRuntimeOutcome.START_FAILED,
                None,
                False,
                False,
                None,
                f"{type(exc).__name__}: {exc}",
            )

        backend: str | None = None
        acknowledged = False
        error: str | None = None
        try:
            backend = self._speaker.play_ready()
            acknowledged = True
        except Exception as exc:
            # Audio acknowledgement is capability-local.  A ready robot must not
            # be shut down merely because the speaker path is unavailable.
            error = f"speaker {type(exc).__name__}: {exc}"
        return WakeRuntimeResult(
            WakeRuntimeOutcome.STARTED_READY,
            pid,
            True,
            acknowledged,
            backend,
            error,
        )

    @staticmethod
    def _pid(status: Mapping[str, object]) -> int | None:
        value = status.get("runtime_pid")
        return value if isinstance(value, int) and not isinstance(value, bool) else None


__all__ = [
    "ReadySpeakerPort",
    "RuntimeControllerPort",
    "WakeRuntimeCoordinator",
    "WakeRuntimeOutcome",
    "WakeRuntimeResult",
]
