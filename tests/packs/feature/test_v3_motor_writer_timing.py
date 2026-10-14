"""Passive motor-write timing cannot change the sole physical write result."""
import pytest

from v3.adapters import motor_writer
from v3.adapters.motor_pwm import MotorChannelPhysicalConfig, plan_final_actuation
from v3.adapters.motor_writer import NativeMotorWriter
from v3.contracts import FinalActuation, SafetyDecision, TickContext
from v3.runtime_performance import RuntimeTimingAccumulator


class _Sink:
    def __init__(self, error=None):
        self.frames = []
        self.error = error

    def write(self, frame):
        self.frames.append(frame)
        if self.error is not None:
            raise self.error


def _writer(sink):
    return NativeMotorWriter(MotorChannelPhysicalConfig(12, 13),
                             MotorChannelPhysicalConfig(18, 19), sink)


def _command():
    return FinalActuation(TickContext(7, 1_000_000_000), .2, -.2,
                          True, SafetyDecision.ALLOW, "CLEAR", None)


def test_motor_writer_wall_and_thread_cpu_reach_runtime_timing_without_an_extra_write(monkeypatch):
    sink = _Sink()
    writer = _writer(sink)
    timing = RuntimeTimingAccumulator(20_000_000)
    writer.set_timing_observer(timing.observe_control_phase)
    wall = iter((1_000_000, 40_470_000))
    cpu = iter((1_000_000, 1_354_000))
    monkeypatch.setattr(motor_writer.time, "perf_counter_ns", lambda: next(wall))
    monkeypatch.setattr(motor_writer.time, "thread_time_ns", lambda: next(cpu))

    command = _command()
    writer.write(command)
    assert sink.frames == [plan_final_actuation(
        MotorChannelPhysicalConfig(12, 13), MotorChannelPhysicalConfig(18, 19), command)]
    evidence = timing.snapshot().as_dict()["control_phase_timing"]
    assert evidence["motor_write_overlaps_l12"] and evidence["causal_claim"] is False
    phases = evidence["phases"]
    assert phases["MOTOR_WRITE"]["count"] == phases["MOTOR_WRITE_CPU"]["count"] == 1
    assert phases["MOTOR_WRITE"]["max_ns"] == 39_470_000
    assert phases["MOTOR_WRITE_CPU"]["max_ns"] == 354_000


@pytest.mark.parametrize("physical_failure", (False, True))
def test_motor_writer_observer_failure_preserves_physical_result_and_never_retries(physical_failure):
    error = OSError("physical write failure") if physical_failure else None
    sink = _Sink(error)
    writer = _writer(sink)
    observations = []

    def broken_observer(name, duration):
        observations.append((name, duration))
        raise RuntimeError("diagnostic failure")

    writer.set_timing_observer(broken_observer)
    if error is None:
        writer.write(_command())
    else:
        with pytest.raises(OSError) as caught:
            writer.write(_command())
        assert caught.value is error
    assert len(sink.frames) == 1
    assert len(observations) == 1 and observations[0][0] == "MOTOR_WRITE"
    # A failed passive consumer is disconnected; an explicitly subsequent
    # successful write still uses the same one-call boundary.
    sink.error = None
    writer.write(_command())
    assert len(sink.frames) == 2 and len(observations) == 1
