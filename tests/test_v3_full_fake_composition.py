from dataclasses import replace

import pytest

from v3.composition.full_fake import FullFakeComposition, LayerFault
from v3.contracts import (
    CommandMode,
    CommandRequest,
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    LifecycleState,
    RawDeviceBatch,
    SafetyDecision,
    TickContext,
)
from v3.engine import TickExecutionError, TickInputs
from v3.replay import first_divergence


_ALL_LAYERS = tuple(f"L{number}" for number in range(1, 13))


def _inputs(count: int, *, target_x_m: float = 20.0) -> tuple[TickInputs, ...]:
    ticks: list[TickInputs] = []
    for tick_id in range(count):
        monotonic_ns = 1_000_000_000 + tick_id * 100_000_000
        context = TickContext(tick_id, monotonic_ns)
        samples = (
            DeviceSample(
                device_id="fake-wheel-odometry",
                kind="wheel_velocity",
                sequence=tick_id,
                captured_monotonic_ns=monotonic_ns,
                values=(
                    DataField("left_mps", 0.10),
                    DataField("right_mps", 0.10),
                    DataField("trust", 1.0),
                ),
            ),
            DeviceSample(
                device_id="fake-ekf-heading",
                kind="ekf_heading",
                sequence=tick_id,
                captured_monotonic_ns=monotonic_ns,
                values=(
                    DataField("yaw_rad", 0.0),
                    DataField("omega_rad_s", 0.0),
                    DataField("confidence", 1.0),
                ),
            ),
            DeviceSample(
                device_id="fake-lidar",
                kind="lidar_health",
                sequence=tick_id,
                captured_monotonic_ns=monotonic_ns,
                values=(
                    DataField("age_ns", 0),
                    DataField("confidence", 1.0),
                ),
            ),
        )
        health = tuple(
            DeviceHealth(device_id, DeviceHealthState.OK)
            for device_id in (
                "fake-wheel-odometry",
                "fake-ekf-heading",
                "fake-lidar",
                "fake-motor-driver",
            )
        )
        ticks.append(
            TickInputs(
                context=context,
                raw_devices=RawDeviceBatch(context, samples, health),
                command=CommandRequest(
                    context=context,
                    command_id="long-navigation",
                    mode=CommandMode.NAVIGATE,
                    goal=(DataField("x_m", target_x_m), DataField("y_m", 0.0)),
                    expiry_tick=count + 10,
                ),
                lifecycle=LifecycleState.ACTIVE,
            )
        )
    return tuple(ticks)


def test_long_full_stack_replay_is_deterministic_and_commits_once_per_tick():
    inputs = _inputs(256)
    first = FullFakeComposition()
    second = FullFakeComposition()

    first_trace = first.run_replay(inputs)
    second_trace = second.run_replay(inputs)

    assert first_trace == second_trace
    assert first_divergence(first_trace, second_trace) is None
    assert first.writes == second.writes
    assert len(first.writes) == len(second.writes) == len(inputs)
    assert all(tuple(record.layer for record in trace.layers) == _ALL_LAYERS for trace in first_trace)
    assert all(write.safety_decision is SafetyDecision.ALLOW for write in first.writes)
    assert any(write.left_output != 0.0 or write.right_output != 0.0 for write in first.writes)


@pytest.mark.parametrize("layer", tuple(f"L{number}" for number in range(1, 12)))
def test_each_upstream_layer_fault_is_fail_closed_and_next_tick_reanchors(layer: str):
    composition = FullFakeComposition(faults=(LayerFault(8, layer),))

    traces = composition.run_replay(_inputs(11))

    fault_trace = traces[8]
    fault_output = fault_trace.layers[-1].output
    assert fault_trace.fault_layer == layer
    assert tuple(record.layer for record in fault_trace.layers) == (
        *tuple(f"L{number}" for number in range(1, int(layer[1:]))),
        "L12",
    )
    assert fault_output.safety_decision is SafetyDecision.FAULT
    assert fault_output.left_output == fault_output.right_output == 0.0
    assert fault_output.reason == f"{layer}_ERROR"
    assert tuple(record.layer for record in traces[9].layers) == _ALL_LAYERS
    assert traces[9].layers[-1].output.reason == "FAULT_LATCHED"
    assert len(composition.writes) == 11


def test_faulted_long_replay_is_direct_value_deterministic():
    inputs = _inputs(80)
    faults = (LayerFault(17, "L4"), LayerFault(43, "L10"))
    first = FullFakeComposition(faults=faults)
    second = FullFakeComposition(faults=faults)

    first_trace = first.run_replay(inputs)
    second_trace = second.run_replay(inputs)

    assert first_trace == second_trace
    assert first_divergence(first_trace, second_trace) is None
    assert first.writes == second.writes
    assert first_trace[17].fault_layer == "L4"
    assert first_trace[43].fault_layer == "L10"


def test_l12_writer_fault_has_one_attempt_and_no_retry():
    composition = FullFakeComposition(faults=(LayerFault(3, "L12"),))
    inputs = _inputs(5)
    for item in inputs[:3]:
        composition.run_tick(item)

    with pytest.raises(TickExecutionError, match="L12"):
        composition.run_tick(inputs[3])

    assert tuple(write.context.tick_id for write in composition.writes) == (0, 1, 2, 3)


def test_non_active_full_stack_tick_still_runs_typed_chain_but_commits_zero():
    item = replace(_inputs(1)[0], lifecycle=LifecycleState.IDLE)
    composition = FullFakeComposition()

    result = composition.run_tick(item)

    assert tuple(record.layer for record in result.trace.layers) == _ALL_LAYERS
    assert result.final_actuation.safety_decision is SafetyDecision.STOP
    assert result.final_actuation.left_output == result.final_actuation.right_output == 0.0
    assert composition.writes == (result.final_actuation,)
