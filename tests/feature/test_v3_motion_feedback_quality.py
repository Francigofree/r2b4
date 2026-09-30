"""Finite low-speed transitions preserve counter evidence and fail closed."""
from dataclasses import replace

import pytest

from rig import resolved_config
from v3.contracts import AdmittedFrame, DataField, Observation, TickContext, WheelVelocitySetpoint
from v3.layers.l11_actuator_control import WheelActuatorController


def _feedback(tick, speed, **extra):
    context = TickContext(tick, 1_000_000_000 + tick*20_000_000)
    values = dict(left_mps=speed, right_mps=speed, trust=1.0,
                  measurement_timing_valid=True, measurement_stale=False,
                  left_estimation_timebase="GPIO_EDGE_HISTORY",
                  right_estimation_timebase="GPIO_EDGE_HISTORY", **extra)
    return AdmittedFrame(context, (Observation("wheel_velocity", "ENCODER", tick,
        context.monotonic_ns, tuple(DataField(k, v) for k, v in values.items())),), ())


def test_motion_low_speed_feedback_blends_recovers_and_preserves_watchdogs():
    config = resolved_config().runtime.composition.live_control.control
    controller = WheelActuatorController(config.speed_map, config.wheel_pi)
    restored = None
    for tick, speed in enumerate((.0, .08, .12, .13, .14, .15, .19, .14, .131, .129, -.12, -.15, -.19)):
        frame = _feedback(tick, speed)
        reference = -.19 if speed < 0 else .19
        wheels = WheelVelocitySetpoint(frame.context, reference, reference,
                                       velocity_transition_until_ns=1_800_000_000)
        output = controller(wheels, frame)
        if restored is not None:
            assert restored(wheels, frame) == output
        if tick < 5 or tick in (9, 10):
            assert output.left_normalized == pytest.approx(config.speed_map.lookup("left", reference)[0])
            assert output.right_normalized == pytest.approx(config.speed_map.lookup("right", reference)[0])
        if tick == 6:
            restored = WheelActuatorController(config.speed_map, config.wheel_pi)
            restored.restore(controller.checkpoint())
    # Healthy pulse timing does not authorize indefinite operation below the
    # reliable velocity band. Alternating wheel demands cannot renew the budget.
    for missing in (False, True):
        controller = WheelActuatorController(config.speed_map, config.wheel_pi)
        deadline = 1_800_000_000
        fault_tick = None
        for tick in range(50):
            frame = _feedback(tick, .08)
            if missing:
                obs = frame.accepted[0]
                frame = replace(frame, accepted=(replace(obs, values=tuple(
                    DataField(v.key, "TICK_SNAPSHOT") if v.key.endswith("estimation_timebase") else v
                    for v in obs.values)),))
            sign = -1 if tick % 2 else 1
            wheels = WheelVelocitySetpoint(frame.context, sign*.19, .19,
                velocity_transition_until_ns=deadline if frame.context.monotonic_ns <= deadline else None)
            try:
                controller(wheels, frame)
            except ValueError as exc:
                assert "feedback remained uncertain too long" in str(exc)
                fault_tick = tick
                break
        assert fault_tick is not None
        elapsed = fault_tick * 20_000_000
        assert elapsed < 800_000_000 if missing else elapsed == 800_000_000

    # A weak left fit must not discard valid right-wheel correction. In the
    # capture the right wheel was already overspeeding when the left lost PI.
    controller = WheelActuatorController(config.speed_map, config.wheel_pi)
    healthy = WheelActuatorController(config.speed_map, config.wheel_pi)
    restored = None
    for tick in range(7):
        good = _feedback(tick, .27)
        wheel = good.accepted[0]
        frame = replace(good, accepted=(replace(wheel, values=tuple(
            DataField(v.key, .08) if v.key == "left_mps" and tick else v
            for v in wheel.values)),))
        wheels = WheelVelocitySetpoint(frame.context, .19, .19)
        expected = healthy(wheels, good)
        output = controller(wheels, frame)
        assert output.right_normalized == expected.right_normalized
        if tick:
            assert output.left_normalized == pytest.approx(config.speed_map.lookup("left", .19)[0])
            assert output.right_normalized < config.speed_map.lookup("right", .19)[0]
        if restored is not None:
            assert restored(wheels, frame) == output
        if tick == 3:
            restored = WheelActuatorController(config.speed_map, config.wheel_pi)
            restored.restore(controller.checkpoint())

    # Learned overspeed correction must fade with confidence before the fit
    # becomes unusable. Otherwise crossing 0.13 erases a full-strength integral
    # and steps the wheel back to the (higher) feed-forward output.
    for sign in (1, -1):
        controller = WheelActuatorController(config.speed_map, config.wheel_pi)
        for tick in range(20):
            frame = _feedback(tick, sign * .23)
            wheels = WheelVelocitySetpoint(frame.context, sign * .15, sign * .15)
            output = controller(wheels, frame)
        feedforward = config.speed_map.lookup("left", sign * .15)[0]
        strong_correction = abs(output.left_normalized - feedforward)
        assert strong_correction > .01
        for tick, speed in enumerate((.14, .131, .129), 20):
            frame = _feedback(tick, sign * speed)
            wheels = replace(wheels, context=frame.context)
            output = controller(wheels, frame)
            correction = abs(output.left_normalized - feedforward)
            if speed == .131:
                assert correction < strong_correction * .1
                restored = WheelActuatorController(config.speed_map, config.wheel_pi)
                restored.restore(controller.checkpoint())
            if speed == .129:
                assert output == restored(wheels, frame)
                assert correction == pytest.approx(0.0)
