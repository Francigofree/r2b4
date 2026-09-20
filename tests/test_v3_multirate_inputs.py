import time

import pytest

from v3.adapters.live_inputs import LiveDeviceSnapshot
from v3.adapters.multirate_inputs import MultiRateInputConfig, MultiRateLiveInputReader
from v3.contracts import (
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    TickContext,
)


class Clock:
    def __init__(self, start: int = 1_000, step: int = 10) -> None:
        self.value = start - step
        self.step = step

    def __call__(self) -> int:
        self.value += self.step
        return self.value


class Source:
    def __init__(self, device_id: str, *, fail: bool = False) -> None:
        self._device_id = device_id
        self.fail = fail
        self.calls = 0

    @property
    def device_id(self) -> str:
        return self._device_id

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        self.calls += 1
        if self.fail:
            raise OSError("injected read failure")
        sample = DeviceSample(
            device_id=self.device_id,
            kind="test_sample",
            sequence=self.calls,
            captured_monotonic_ns=context.monotonic_ns,
            values=(DataField("value", self.calls),),
        )
        return LiveDeviceSnapshot(
            context,
            DeviceHealth(self.device_id, DeviceHealthState.OK),
            (sample,),
        )


def _config() -> MultiRateInputConfig:
    return MultiRateInputConfig(
        critical_default_period_ns=1_000,
        auxiliary_default_period_ns=2_000,
        history_size=4,
    )


def test_control_read_only_closes_buffer_and_never_polls_source():
    source = Source("ENC")
    reader = MultiRateLiveInputReader(
        (source,),
        critical_device_ids=frozenset({"ENC"}),
        config=_config(),
        monotonic_ns=Clock(),
        start_workers=False,
    )
    try:
        assert source.calls == 1
        batch = reader.read(TickContext(99, 2_000))
        assert source.calls == 1
        assert batch.context == TickContext(99, 2_000)
        assert len(batch.samples) == 1
        assert batch.samples[0].device_id == "ENC"
        assert batch.device_health[0].state is DeviceHealthState.OK
    finally:
        reader.close()


def test_snapshot_is_not_visible_before_its_publication_time():
    source = Source("ENC")
    reader = MultiRateLiveInputReader(
        (source,),
        critical_device_ids=frozenset({"ENC"}),
        config=_config(),
        monotonic_ns=Clock(start=1_000, step=10),
        start_workers=False,
    )
    try:
        # Prime starts at 1000 and completes at 1010, publishes at 1020.
        early = reader.read(TickContext(1, 1_005))
        assert early.samples == ()
        assert early.device_health[0].state is DeviceHealthState.UNKNOWN
        assert early.device_health[0].reason == "L0_NO_VISIBLE_SNAPSHOT"

        visible = reader.read(TickContext(2, 1_020))
        assert len(visible.samples) == 1
        assert visible.device_health[0].state is DeviceHealthState.OK
    finally:
        reader.close()


def test_auxiliary_read_failure_is_health_not_global_reader_exception():
    critical = Source("ENC")
    auxiliary = Source("CAM", fail=True)
    reader = MultiRateLiveInputReader(
        (critical, auxiliary),
        critical_device_ids=frozenset({"ENC"}),
        config=_config(),
        monotonic_ns=Clock(),
        start_workers=False,
    )
    try:
        batch = reader.read(TickContext(1, 3_000))
        health = {item.device_id: item for item in batch.device_health}
        assert health["ENC"].state is DeviceHealthState.OK
        assert health["CAM"].state is DeviceHealthState.FAILED
        assert health["CAM"].reason == "L0_SOURCE_READ_ERROR"
    finally:
        reader.close()


def test_missing_required_critical_source_is_rejected_before_runtime_start():
    source = Source("ENC")
    with pytest.raises(ValueError, match="IMU"):
        MultiRateLiveInputReader(
            (source,),
            critical_device_ids=frozenset({"ENC", "IMU"}),
            config=_config(),
            monotonic_ns=Clock(),
            start_workers=False,
        )


def test_worker_initializer_runs_for_per_source_critical_and_auxiliary_lanes():
    roles: list[str] = []
    config = MultiRateInputConfig(
        critical_default_period_ns=1_000_000_000,
        auxiliary_default_period_ns=1_000_000_000,
        history_size=2,
    )
    reader = MultiRateLiveInputReader(
        (Source("ENC"), Source("CAM")),
        critical_device_ids=frozenset({"ENC"}),
        config=config,
        monotonic_ns=time.monotonic_ns,
        worker_initializer=roles.append,
    )
    try:
        assert sorted(roles) == ["l0-aux", "l0-critical-ENC"]
    finally:
        reader.close()


def test_worker_initializer_failure_aborts_before_runtime_use():
    def fail(_role: str) -> None:
        raise RuntimeError("affinity failed")

    with pytest.raises(RuntimeError, match="worker startup failed"):
        MultiRateLiveInputReader(
            (Source("ENC"),),
            critical_device_ids=frozenset({"ENC"}),
            config=MultiRateInputConfig(
                critical_default_period_ns=1_000_000_000,
                auxiliary_default_period_ns=1_000_000_000,
            ),
            monotonic_ns=time.monotonic_ns,
            worker_initializer=fail,
        )
