import pytest

from v3.adapters.motor_pwm import (
    Drv8871MotorFrame,
    Drv8871PwmPlan,
    MotorChannelPhysicalConfig,
    PwmDecayMode,
    plan_final_actuation,
)
from v3.adapters.motor_writer import NativeMotorWriter
from v3.contracts import FinalActuation, SafetyDecision, TickContext


class RecordingFrameSink:
    def __init__(self, error: Exception | None = None) -> None:
        self.calls = 0
        self.frames: list[Drv8871MotorFrame] = []
        self.error = error

    def write(self, frame: Drv8871MotorFrame) -> None:
        self.calls += 1
        self.frames.append(frame)
        if self.error is not None:
            raise self.error


def _configs() -> tuple[MotorChannelPhysicalConfig, MotorChannelPhysicalConfig]:
    return (
        MotorChannelPhysicalConfig(1, 2),
        MotorChannelPhysicalConfig(
            3,
            4,
            invert=True,
            pwm_decay_mode=PwmDecayMode.BRAKE,
        ),
    )


def _command(
    decision: SafetyDecision,
    *,
    left_output: float = 0.0,
    right_output: float = 0.0,
    enabled: bool = False,
) -> FinalActuation:
    return FinalActuation(
        context=TickContext(tick_id=17, monotonic_ns=4_000_000),
        left_output=left_output,
        right_output=right_output,
        enabled=enabled,
        safety_decision=decision,
        latch_state="CLEAR",
        reason=None if decision is SafetyDecision.ALLOW else "STOPPED",
    )


def test_allow_decision_becomes_one_validated_frame_and_one_sink_call():
    left_config, right_config = _configs()
    sink = RecordingFrameSink()
    writer = NativeMotorWriter(left_config, right_config, sink)
    command = _command(
        SafetyDecision.ALLOW,
        left_output=0.25,
        right_output=-0.5,
        enabled=True,
    )

    result = writer.write(command)

    assert result is None
    assert sink.calls == 1
    assert sink.frames == [
        plan_final_actuation(left_config, right_config, command)
    ]
    assert sink.frames[0].context is command.context
    assert sink.frames[0].left == Drv8871PwmPlan(25.0, 0.0)
    assert sink.frames[0].right == Drv8871PwmPlan(100.0, 50.0)


@pytest.mark.parametrize("decision", [SafetyDecision.STOP, SafetyDecision.FAULT])
def test_stop_and_fault_each_become_one_all_zero_sink_call(
    decision: SafetyDecision,
):
    left_config, right_config = _configs()
    sink = RecordingFrameSink()
    writer = NativeMotorWriter(left_config, right_config, sink)

    writer.write(_command(decision))

    assert sink.calls == 1
    assert len(sink.frames) == 1
    assert sink.frames[0].safety_decision is decision
    assert sink.frames[0].left == Drv8871PwmPlan(0.0, 0.0)
    assert sink.frames[0].right == Drv8871PwmPlan(0.0, 0.0)


def test_sink_exception_propagates_without_retry_or_fallback_write():
    left_config, right_config = _configs()
    error = RuntimeError("physical frame write failed")
    sink = RecordingFrameSink(error)
    writer = NativeMotorWriter(left_config, right_config, sink)

    with pytest.raises(RuntimeError) as raised:
        writer.write(
            _command(
                SafetyDecision.ALLOW,
                left_output=0.25,
                right_output=0.25,
                enabled=True,
            )
        )

    assert raised.value is error
    assert sink.calls == 1
    assert len(sink.frames) == 1


def test_invalid_command_is_rejected_before_the_sink_boundary():
    left_config, right_config = _configs()
    sink = RecordingFrameSink()
    writer = NativeMotorWriter(left_config, right_config, sink)

    with pytest.raises(TypeError, match="command must be FinalActuation"):
        writer.write(object())  # type: ignore[arg-type]

    assert sink.calls == 0


def test_duplicate_physical_pins_are_rejected_during_construction():
    sink = RecordingFrameSink()

    with pytest.raises(ValueError, match="GPIO pins must be unique"):
        NativeMotorWriter(
            MotorChannelPhysicalConfig(1, 2),
            MotorChannelPhysicalConfig(2, 3),
            sink,
        )

    assert sink.calls == 0


def test_sink_contract_is_checked_before_any_write_is_possible():
    left_config, right_config = _configs()

    with pytest.raises(TypeError, match="callable write"):
        NativeMotorWriter(left_config, right_config, object())  # type: ignore[arg-type]
