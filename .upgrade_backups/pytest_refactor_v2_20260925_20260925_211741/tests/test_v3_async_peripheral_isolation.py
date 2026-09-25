from __future__ import annotations

import threading
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


class FastSource:
    def __init__(self, device_id: str) -> None:
        self._device_id = device_id
        self.calls = 0
        self._lock = threading.Lock()

    @property
    def device_id(self) -> str:
        return self._device_id

    def call_count(self) -> int:
        with self._lock:
            return self.calls

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        with self._lock:
            self.calls += 1
            sequence = self.calls
        return LiveDeviceSnapshot(
            context,
            DeviceHealth(self.device_id, DeviceHealthState.OK),
            (
                DeviceSample(
                    device_id=self.device_id,
                    kind="test_sample",
                    sequence=sequence,
                    captured_monotonic_ns=context.monotonic_ns,
                    values=(DataField("value", sequence),),
                ),
            ),
        )


class BlockingAfterPrimeSource(FastSource):
    def __init__(self, device_id: str) -> None:
        super().__init__(device_id)
        self.block_entered = threading.Event()
        self.release = threading.Event()

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        with self._lock:
            self.calls += 1
            sequence = self.calls
        if sequence >= 2:
            self.block_entered.set()
            self.release.wait(timeout=2.0)
        return LiveDeviceSnapshot(
            context,
            DeviceHealth(self.device_id, DeviceHealthState.OK),
            (
                DeviceSample(
                    device_id=self.device_id,
                    kind="test_sample",
                    sequence=sequence,
                    captured_monotonic_ns=context.monotonic_ns,
                    values=(DataField("value", sequence),),
                ),
            ),
        )


class BlockingFirstReadSource(FastSource):
    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        time.sleep(0.20)
        return super().read(context)


def _wait_until(predicate, timeout_s: float = 1.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.002)
    raise AssertionError("condition did not become true before timeout")


def test_blocked_critical_source_does_not_stall_other_critical_streams():
    fast = FastSource("FAST")
    slow = BlockingAfterPrimeSource("SLOW")
    reader = MultiRateLiveInputReader(
        (fast, slow),
        critical_device_ids=frozenset({"FAST", "SLOW"}),
        config=MultiRateInputConfig(
            critical_default_period_ns=5_000_000,
            auxiliary_default_period_ns=20_000_000,
            history_size=8,
            max_snapshot_age_ns=30_000_000,
            worker_join_timeout_s=1.0,
            source_periods=(),
        ),
        monotonic_ns=time.monotonic_ns,
    )
    try:
        assert slow.block_entered.wait(timeout=1.0)
        fast_before = fast.call_count()
        _wait_until(lambda: fast.call_count() >= fast_before + 4, timeout_s=0.5)

        now_ns = time.monotonic_ns()
        live = {item.device_id: item for item in reader.liveness_snapshot(now_ns)}
        assert live["SLOW"].read_in_flight is True
        assert live["SLOW"].read_elapsed_ns is not None
        assert live["FAST"].publication_count > live["SLOW"].publication_count

        # A blocked SLOW read ages only SLOW. The control closure itself remains
        # bounded and still receives current FAST observations.
        time.sleep(0.040)
        context = reader.begin_tick(100)
        batch = reader.read(context)
        health = {item.device_id: item for item in batch.device_health}
        assert health["FAST"].state is DeviceHealthState.OK
        assert health["SLOW"].state is DeviceHealthState.UNKNOWN
        assert health["SLOW"].reason == "L0_STREAM_EXPIRED"
    finally:
        slow.release.set()
        reader.close()


def test_critical_first_read_has_bounded_startup_wait():
    started = time.monotonic()
    with pytest.raises(RuntimeError, match="critical source prime timed out"):
        MultiRateLiveInputReader(
            (BlockingFirstReadSource("SLOW"),),
            critical_device_ids=frozenset({"SLOW"}),
            config=MultiRateInputConfig(
                critical_default_period_ns=5_000_000,
                auxiliary_default_period_ns=20_000_000,
                history_size=4,
                worker_join_timeout_s=0.05,
                source_periods=(),
            ),
            monotonic_ns=time.monotonic_ns,
        )
    assert time.monotonic() - started < 0.5


def test_liveness_snapshot_reports_measurement_publication_and_overruns():
    class Clock:
        def __init__(self) -> None:
            self.value = 980

        def __call__(self) -> int:
            self.value += 20
            return self.value

    source = FastSource("ENC")
    reader = MultiRateLiveInputReader(
        (source,),
        critical_device_ids=frozenset({"ENC"}),
        config=MultiRateInputConfig(
            critical_default_period_ns=10,
            auxiliary_default_period_ns=20,
            history_size=4,
            source_periods=(),
        ),
        monotonic_ns=Clock(),
        start_workers=False,
    )
    try:
        # Prime: start=1000, complete=1020, publish=1040. With a 10 ns period,
        # slots at 1010 and 1020 were skipped before next_due advances to 1030.
        item = reader.liveness_snapshot(1100)[0]
        assert item.device_id == "ENC"
        assert item.publication_count == 1
        assert item.last_measurement_ns == 1000
        assert item.last_read_duration_ns == 20
        assert item.last_publication_latency_ns == 20
        assert item.measurement_age_ns == 100
        assert item.publication_age_ns == 60
        assert item.missed_period_count == 2
        assert item.deadline_overrun_count == 1
        assert item.read_in_flight is False
    finally:
        reader.close()
