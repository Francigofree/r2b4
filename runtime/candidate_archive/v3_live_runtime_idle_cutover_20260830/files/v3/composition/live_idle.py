"""Explicit physical V3 composition limited to BOOTING, IDLE and zero output."""

from __future__ import annotations

from dataclasses import dataclass

from v3.adapters.live_idle import (
    GpioBackend,
    GpioZeroMotorWriter,
    GpioZeroWriterConfig,
    LiveIdleDeviceReader,
    LockedStopCommandGateway,
)
from v3.contracts import LifecycleState
from v3.engine import TickResult

from .stop_only import StopOnlyComposition


@dataclass(frozen=True, slots=True)
class LiveIdleConfig:
    """Immutable configuration closed before physical GPIO is claimed."""

    motors: GpioZeroWriterConfig
    frame_id: str = "R2B4_BOOT_ROBOT_MAP"

    def __post_init__(self) -> None:
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("frame_id must be a non-empty string")


class LiveIdleComposition:
    """Own the zero-only physical edge and the complete V3 L1-L12 chain.

    Construction claims the configured GPIOs and immediately applies zero.
    The physical writer is passed only to ``StopOnlyComposition``/L12 and is
    never exposed by this API.  There is deliberately no ACTIVE transition.
    """

    __slots__ = ("_closed", "_runtime", "_writer")

    def __init__(self, backend: GpioBackend, config: LiveIdleConfig) -> None:
        writer = GpioZeroMotorWriter(backend, config.motors)
        try:
            runtime = StopOnlyComposition(
                LiveIdleDeviceReader(),
                LockedStopCommandGateway(),
                writer,
                frame_id=config.frame_id,
            )
        except Exception:
            writer.close()
            raise
        self._writer = writer
        self._runtime = runtime
        self._closed = False

    @property
    def lifecycle(self) -> LifecycleState:
        return self._runtime.lifecycle

    @property
    def closed(self) -> bool:
        return self._closed

    def enter_idle(self) -> None:
        if self._closed:
            raise RuntimeError("the live IDLE composition is closed")
        self._runtime.enter_idle()

    def tick(self, monotonic_ns: int) -> TickResult:
        if self._closed:
            raise RuntimeError("the live IDLE composition is closed")
        return self._runtime.tick(monotonic_ns)

    def close(self, monotonic_ns: int) -> TickResult | None:
        """Commit one SHUTDOWN zero when safe, then close the GPIO handle."""

        if self._closed:
            return None
        result: TickResult | None = None
        try:
            if self._runtime.lifecycle is not LifecycleState.FAULT:
                self._runtime.shutdown()
                result = self._runtime.tick(monotonic_ns)
        finally:
            try:
                self._writer.close()
            finally:
                self._closed = True
        return result


__all__ = ["LiveIdleComposition", "LiveIdleConfig"]
