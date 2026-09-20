from dataclasses import replace

from v3.composition.native_control import NativeControlComposition
from v3.contracts import CommandMode, CommandRequest, DataField, DeviceSample, TickContext
from v3.layers.l6_navigation import InlineTrajectoryRolloutBackend
from v3_validation_helpers import RecordingMotorSink, control_config, tick_inputs


class DelayedBackend:
    def __init__(self, config, *, delay_calls: int) -> None:
        self.inner = InlineTrajectoryRolloutBackend(config)
        self.delay_calls = delay_calls
        self.calls = 0

    def submit(self, request):
        self.calls = 0
        return self.inner.submit(request)

    def take(self, request_id):
        self.calls += 1
        if self.calls <= self.delay_calls:
            return None
        return self.inner.take(request_id)

    def abandon(self, request_id):
        self.inner.abandon(request_id)

    def close(self):
        self.inner.close()


def _retime(item, monotonic_ns: int):
    context = TickContext(item.context.tick_id, monotonic_ns)
    raw = replace(
        item.raw_devices,
        context=context,
        samples=tuple(
            replace(sample, captured_monotonic_ns=monotonic_ns)
            for sample in item.raw_devices.samples
        ),
    )
    command = replace(item.command, context=context)
    return replace(item, context=context, raw_devices=raw, command=command)


def _explore_values(count: int = 6):
    values = []
    for item in tick_inputs(count):
        context = item.context
        active = 1 <= context.tick_id < count - 1
        command = CommandRequest(
            context,
            "boundary-fix",
            CommandMode.EXPLORE if active else CommandMode.STOP,
            (
                DataField("max_v_mps", 0.15),
                DataField("max_omega_rad_s", 0.3),
            ) if active else (),
            context.tick_id,
        )
        local = DeviceSample(
            "RPLIDAR_C1",
            "lidar_local_points",
            context.tick_id,
            context.monotonic_ns,
            (DataField("frame_id", "ROBOT_BASE"), DataField("point_count", 0)),
        )
        values.append(
            replace(
                item,
                command=command,
                raw_devices=replace(
                    item.raw_devices,
                    samples=item.raw_devices.samples + (local,),
                ),
            )
        )
    return values


def test_pre_submit_scheduler_delay_does_not_consume_worker_timeout_budget():
    config = control_config()
    backend = DelayedBackend(config.navigation, delay_calls=1)
    production = NativeControlComposition(
        RecordingMotorSink(),
        config,
        trajectory_rollout_backend=backend,
    )
    values = _explore_values()

    production.run_tick(production.close_inputs(values[0]))
    first_active = production.close_inputs(values[1])
    result = production.run_tick(first_active)
    assert result.trace.fault_layer is None

    # The request was created at 1.02 s, but cannot physically be submitted
    # until this next close_inputs call. Simulate >300 ms scheduler delay before
    # that boundary. The old L6 request-source timeout faulted immediately here.
    delayed_submit = _retime(values[2], 1_331_000_000)
    closed = production.close_inputs(delayed_submit)
    assert closed.planner_input is not None
    assert closed.planner_input.error is None
    result = production.run_tick(closed)
    assert result.trace.fault_layer is None

    ready = _retime(values[3], 1_340_000_000)
    result = production.run_tick(production.close_inputs(ready))
    assert result.trace.fault_layer is None
    assert result.final_actuation.safety_decision.value != "FAULT"
    production.close()


def test_worker_timeout_is_closed_as_typed_planner_input_after_actual_submit():
    config = control_config()
    backend = DelayedBackend(config.navigation, delay_calls=10_000)
    production = NativeControlComposition(
        RecordingMotorSink(),
        config,
        trajectory_rollout_backend=backend,
    )
    values = _explore_values()

    production.run_tick(production.close_inputs(values[0]))
    production.run_tick(production.close_inputs(values[1]))

    submitted = _retime(values[2], 1_331_000_000)
    closed = production.close_inputs(submitted)
    assert closed.planner_input is not None
    assert closed.planner_input.error is None
    assert production.run_tick(closed).trace.fault_layer is None

    expired = _retime(values[3], 1_631_000_001)
    closed = production.close_inputs(expired)
    assert closed.planner_input is not None
    assert closed.planner_input.request_context is not None
    assert closed.planner_input.error == "ASYNC_L6_DEADLINE_MISSED"
    result = production.run_tick(closed)
    assert result.trace.fault_layer == "L6"
    assert result.final_actuation.left_output == 0
    assert result.final_actuation.right_output == 0
    production.close()
