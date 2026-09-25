from __future__ import annotations

from dataclasses import dataclass

from v3.adapters.live_microphone import NativeMicrophoneConfig, NativeMicrophoneSource
from v3.adapters.microphone import (
    MicrophoneHealth,
    MicrophoneState,
    MicrophoneStreamConfig,
)
from v3.contracts import DeviceHealthState, TickContext


@dataclass
class _Port:
    status: MicrophoneHealth
    stream: MicrophoneStreamConfig = MicrophoneStreamConfig()

    def health(self) -> MicrophoneHealth:
        return self.status


def _health(
    *,
    state: MicrophoneState = MicrophoneState.CAPTURING,
    sequence: int = 7,
    captured_ns: int | None = 900,
    present: bool = True,
    overwrites: int = 12,
    error: str | None = None,
) -> MicrophoneHealth:
    age_ms = None if captured_ns is None else 0.1
    return MicrophoneHealth(
        state=state,
        device_present=present,
        sequence=sequence,
        last_frame_monotonic_ns=captured_ns,
        last_frame_age_ms=age_ms,
        ring_overwrite_count=overwrites,
        last_error=error,
    )


def test_live_microphone_exposes_metadata_but_never_raw_pcm():
    source = NativeMicrophoneSource(
        _Port(_health()),
        NativeMicrophoneConfig(maximum_frame_age_ns=250),
    )
    snapshot = source.read(TickContext(1, 1_000))
    assert snapshot.health.state is DeviceHealthState.OK
    assert len(snapshot.samples) == 1
    sample = snapshot.samples[0]
    assert sample.kind == "microphone_frame_health"
    assert sample.captured_monotonic_ns == 900
    values = {field.key: field.value for field in sample.values}
    assert values["age_ns"] == 100
    assert values["sample_rate_hz"] == 48_000
    assert values["channels"] == 1
    assert values["sample_format"] == "S16_LE"
    assert values["frame_samples"] == 960
    assert values["frame_bytes"] == 1920
    assert values["ring_overwrite_count"] == 12
    assert "pcm" not in values


def test_live_microphone_marks_stale_frame_degraded():
    source = NativeMicrophoneSource(
        _Port(_health(captured_ns=500)),
        NativeMicrophoneConfig(maximum_frame_age_ns=250),
    )
    snapshot = source.read(TickContext(1, 1_000))
    assert snapshot.health.state is DeviceHealthState.DEGRADED
    assert snapshot.health.reason == "MICROPHONE_FRAME_STALE"
    assert len(snapshot.samples) == 1


def test_live_microphone_reports_no_frame_without_fabricating_sample():
    source = NativeMicrophoneSource(
        _Port(_health(sequence=0, captured_ns=None)),
        NativeMicrophoneConfig(),
    )
    snapshot = source.read(TickContext(1, 1_000))
    assert snapshot.health.state is DeviceHealthState.UNKNOWN
    assert snapshot.health.reason == "MICROPHONE_NO_FRAME"
    assert snapshot.samples == ()


def test_live_microphone_disconnect_is_capability_local_failure():
    source = NativeMicrophoneSource(
        _Port(
            _health(
                state=MicrophoneState.DISCONNECTED,
                sequence=0,
                captured_ns=None,
                present=False,
                error="not connected",
            )
        ),
        NativeMicrophoneConfig(),
    )
    snapshot = source.read(TickContext(1, 1_000))
    assert snapshot.health.state is DeviceHealthState.FAILED
    assert snapshot.health.reason == "MICROPHONE_DISCONNECTED"
    assert snapshot.samples == ()


def test_live_microphone_rejects_future_timestamp():
    source = NativeMicrophoneSource(
        _Port(_health(captured_ns=1_001)),
        NativeMicrophoneConfig(),
    )
    snapshot = source.read(TickContext(1, 1_000))
    assert snapshot.health.state is DeviceHealthState.FAILED
    assert snapshot.health.reason == "MICROPHONE_TIME_INVALID"
    assert snapshot.samples == ()


def test_live_microphone_port_exception_isolated():
    class BrokenPort:
        stream = MicrophoneStreamConfig()

        def health(self):
            raise OSError("capture status unavailable")

    source = NativeMicrophoneSource(BrokenPort(), NativeMicrophoneConfig())
    snapshot = source.read(TickContext(1, 1_000))
    assert snapshot.health.state is DeviceHealthState.FAILED
    assert snapshot.health.reason == "MICROPHONE_PORT_ERROR"
    assert snapshot.samples == ()
