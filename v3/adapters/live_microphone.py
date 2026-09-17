"""V3 L0 microphone health/metadata projection over the native audio edge.

Raw PCM remains on ``AudioFramePort`` for voice/audio consumers and never enters
``DeviceSample``.  This source contributes only bounded timing/stream metadata
and non-critical capability health to the V3 acquisition path.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from v3.adapters.live_inputs import LiveDeviceSnapshot
from v3.adapters.microphone import (
    MicrophoneHealth,
    MicrophoneState,
    MicrophoneStreamConfig,
)
from v3.contracts import (
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    TickContext,
)


class MicrophoneHealthPort(Protocol):
    @property
    def stream(self) -> MicrophoneStreamConfig: ...

    def health(self) -> MicrophoneHealth: ...


@dataclass(frozen=True, slots=True)
class NativeMicrophoneConfig:
    device_id: str = "MICROPHONE_FRONT"
    maximum_frame_age_ns: int = 250_000_000

    def __post_init__(self) -> None:
        if not isinstance(self.device_id, str) or not self.device_id.strip():
            raise ValueError("device_id must be a non-empty string")
        if (
            not isinstance(self.maximum_frame_age_ns, int)
            or isinstance(self.maximum_frame_age_ns, bool)
            or self.maximum_frame_age_ns <= 0
        ):
            raise ValueError("maximum_frame_age_ns must be a positive integer")


class NativeMicrophoneSource:
    """Project one microphone owner's bounded health into the V3 L0 edge."""

    __slots__ = ("_config", "_port")

    def __init__(
        self,
        port: MicrophoneHealthPort,
        config: NativeMicrophoneConfig,
    ) -> None:
        if not callable(getattr(port, "health", None)):
            raise TypeError("port must provide health")
        if not isinstance(getattr(port, "stream", None), MicrophoneStreamConfig):
            raise TypeError("port.stream must be MicrophoneStreamConfig")
        if not isinstance(config, NativeMicrophoneConfig):
            raise TypeError("config must be NativeMicrophoneConfig")
        self._port = port
        self._config = config

    @property
    def device_id(self) -> str:
        return self._config.device_id

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        try:
            health = self._port.health()
            stream = self._port.stream
        except Exception:
            return self._failed(context, "MICROPHONE_PORT_ERROR")

        if not isinstance(health, MicrophoneHealth) or not isinstance(
            stream, MicrophoneStreamConfig
        ):
            return self._failed(context, "MICROPHONE_PORT_INVALID")

        if health.state is MicrophoneState.DISCONNECTED:
            return self._failed(context, "MICROPHONE_DISCONNECTED")
        if health.state is MicrophoneState.FAILED:
            return self._failed(context, "MICROPHONE_RUNTIME_ERROR")
        if health.state is MicrophoneState.STOPPED:
            return self._failed(context, "MICROPHONE_NOT_RUNNING")
        if health.state is not MicrophoneState.CAPTURING:
            return self._failed(context, "MICROPHONE_STATE_INVALID")
        if not health.device_present:
            return self._failed(context, "MICROPHONE_DEVICE_MISSING")

        captured_ns = health.last_frame_monotonic_ns
        if health.sequence == 0 or captured_ns is None:
            return LiveDeviceSnapshot(
                context,
                DeviceHealth(
                    self.device_id,
                    DeviceHealthState.UNKNOWN,
                    "MICROPHONE_NO_FRAME",
                ),
            )
        if captured_ns > context.monotonic_ns:
            return self._failed(context, "MICROPHONE_TIME_INVALID")

        age_ns = context.monotonic_ns - captured_ns
        stale = age_ns > self._config.maximum_frame_age_ns
        state = DeviceHealthState.DEGRADED if stale else DeviceHealthState.OK
        reason = "MICROPHONE_FRAME_STALE" if stale else None

        sample = DeviceSample(
            device_id=self.device_id,
            kind="microphone_frame_health",
            sequence=health.sequence,
            captured_monotonic_ns=captured_ns,
            values=(
                DataField("age_ns", age_ns),
                DataField("measurement_stale", stale),
                DataField("sample_rate_hz", stream.sample_rate_hz),
                DataField("channels", stream.channels),
                DataField("sample_format", stream.sample_format),
                DataField("frame_duration_ms", stream.frame_duration_ms),
                DataField("frame_samples", stream.frame_samples),
                DataField("frame_bytes", stream.frame_bytes),
                DataField("ring_capacity_frames", stream.queue_capacity_frames),
                DataField("ring_overwrite_count", health.ring_overwrite_count),
            ),
        )
        return LiveDeviceSnapshot(
            context,
            DeviceHealth(self.device_id, state, reason),
            (sample,),
        )

    def _failed(self, context: TickContext, reason: str) -> LiveDeviceSnapshot:
        return LiveDeviceSnapshot(
            context,
            DeviceHealth(self.device_id, DeviceHealthState.FAILED, reason),
        )


__all__ = [
    "MicrophoneHealthPort",
    "NativeMicrophoneConfig",
    "NativeMicrophoneSource",
]
