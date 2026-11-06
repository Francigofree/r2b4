import time
import pytest
from v3.adapters.live_inputs import LiveDeviceSnapshot
from v3.adapters.multirate_inputs import MultiRateInputConfig, MultiRateLiveInputReader
from v3.contracts import DataField, DeviceHealth, DeviceHealthState, DeviceSample, TickContext

class Clock:

    def __init__(self, start: int=1000, step: int=10) -> None:
        self.value = start - step
        self.step = step

    def __call__(self) -> int:
        self.value += self.step
        return self.value

class Source:

    def __init__(self, device_id: str, *, fail: bool=False) -> None:
        self._device_id = device_id
        self.fail = fail
        self.calls = 0

    @property
    def device_id(self) -> str:
        return self._device_id

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        self.calls += 1
        if self.fail:
            raise OSError('injected read failure')
        sample = DeviceSample(device_id=self.device_id, kind='test_sample', sequence=self.calls, captured_monotonic_ns=context.monotonic_ns, values=(DataField('value', self.calls),))
        return LiveDeviceSnapshot(context, DeviceHealth(self.device_id, DeviceHealthState.OK), (sample,))

def _config() -> MultiRateInputConfig:
    return MultiRateInputConfig(critical_default_period_ns=1000, auxiliary_default_period_ns=2000, history_size=4)

def test_worker_initializer_failure_aborts_before_runtime_use():

    def fail(_role: str) -> None:
        raise RuntimeError('affinity failed')
    with pytest.raises(RuntimeError, match='worker startup failed'):
        MultiRateLiveInputReader((Source('ENC'),), critical_device_ids=frozenset({'ENC'}), config=MultiRateInputConfig(critical_default_period_ns=1000000000, auxiliary_default_period_ns=1000000000), monotonic_ns=time.monotonic_ns, worker_initializer=fail)
