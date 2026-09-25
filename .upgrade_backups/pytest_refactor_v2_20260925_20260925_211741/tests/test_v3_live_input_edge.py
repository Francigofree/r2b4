from dataclasses import FrozenInstanceError

import pytest

from v3.adapters.live_inputs import LiveDeviceSnapshot, NativeLiveInputReader
from v3.contracts import (
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    RawDeviceBatch,
    TickContext,
)


class _FakeSource:
    def __init__(
        self,
        device_id: str,
        snapshot: LiveDeviceSnapshot | Exception,
        call_order: list[str],
    ) -> None:
        self.device_id = device_id
        self._snapshot = snapshot
        self._call_order = call_order
        self.calls: list[TickContext] = []

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        self.calls.append(context)
        self._call_order.append(self.device_id)
        if isinstance(self._snapshot, Exception):
            raise self._snapshot
        return self._snapshot


def _snapshot(
    context: TickContext,
    device_id: str,
    kind: str,
    sequence: int,
) -> LiveDeviceSnapshot:
    return LiveDeviceSnapshot(
        context=context,
        health=DeviceHealth(device_id, DeviceHealthState.OK),
        samples=(
            DeviceSample(
                device_id=device_id,
                kind=kind,
                sequence=sequence,
                captured_monotonic_ns=context.monotonic_ns - 10,
                values=(DataField("value", 0.5),),
            ),
        ),
    )


def test_native_live_reader_polls_each_owned_source_once_in_fixed_order():
    context = TickContext(4, 1_000)
    call_order: list[str] = []
    encoder_snapshot = _snapshot(context, "encoder", "wheel_velocity", 8)
    imu_snapshot = _snapshot(context, "imu", "imu_heading", 11)
    encoder = _FakeSource(
        "encoder",
        encoder_snapshot,
        call_order,
    )
    imu = _FakeSource(
        "imu",
        imu_snapshot,
        call_order,
    )
    reader = NativeLiveInputReader((encoder, imu))

    batch = reader.read(context)

    assert batch == RawDeviceBatch(
        context=context,
        samples=encoder_snapshot.samples + imu_snapshot.samples,
        device_health=(encoder_snapshot.health, imu_snapshot.health),
    )
    assert call_order == ["encoder", "imu"]
    assert encoder.calls == [context]
    assert imu.calls == [context]


def test_native_live_reader_rejects_duplicate_source_authority_before_polling():
    context = TickContext(4, 1_000)
    call_order: list[str] = []
    first = _FakeSource("encoder", _snapshot(context, "encoder", "a", 1), call_order)
    second = _FakeSource("encoder", _snapshot(context, "encoder", "b", 2), call_order)

    with pytest.raises(ValueError, match="source IDs must be unique"):
        NativeLiveInputReader((first, second))

    assert call_order == []


def test_native_live_reader_rejects_wrong_tick_context_fail_closed():
    requested = TickContext(4, 1_000)
    stale = TickContext(3, 900)
    source = _FakeSource(
        "encoder",
        _snapshot(stale, "encoder", "wheel_velocity", 8),
        [],
    )

    with pytest.raises(ValueError, match="context must match"):
        NativeLiveInputReader((source,)).read(requested)

    assert source.calls == [requested]


def test_native_live_reader_rejects_snapshot_from_the_wrong_configured_device():
    context = TickContext(4, 1_000)
    source = _FakeSource(
        "encoder",
        _snapshot(context, "imu", "imu_heading", 11),
        [],
    )

    with pytest.raises(ValueError, match="configured source"):
        NativeLiveInputReader((source,)).read(context)

    assert source.calls == [context]


def test_native_live_reader_propagates_failure_without_retry_or_partial_batch():
    context = TickContext(4, 1_000)
    call_order: list[str] = []
    failed = _FakeSource("encoder", OSError("injected read failure"), call_order)
    later = _FakeSource(
        "imu",
        _snapshot(context, "imu", "imu_heading", 11),
        call_order,
    )
    reader = NativeLiveInputReader((failed, later))

    with pytest.raises(OSError, match="read failure"):
        reader.read(context)

    assert failed.calls == [context]
    assert later.calls == []
    assert call_order == ["encoder"]


def test_live_device_snapshot_is_immutable_and_device_scoped():
    context = TickContext(4, 1_000)
    snapshot = _snapshot(context, "encoder", "wheel_velocity", 8)

    with pytest.raises(FrozenInstanceError):
        snapshot.context = TickContext(5, 2_000)
    with pytest.raises(ValueError, match="device IDs must match"):
        LiveDeviceSnapshot(
            context=context,
            health=DeviceHealth("encoder", DeviceHealthState.OK),
            samples=(
                DeviceSample(
                    "imu",
                    "imu_heading",
                    1,
                    1_000,
                    (DataField("value", 0.0),),
                ),
            ),
        )


def test_live_device_snapshot_rejects_duplicate_sample_identity():
    context = TickContext(4, 1_000)
    duplicate = DeviceSample(
        "encoder",
        "wheel_velocity",
        8,
        990,
        (DataField("value", 0.5),),
    )

    with pytest.raises(ValueError, match="duplicate sample identity"):
        LiveDeviceSnapshot(
            context=context,
            health=DeviceHealth("encoder", DeviceHealthState.OK),
            samples=(duplicate, duplicate),
        )
