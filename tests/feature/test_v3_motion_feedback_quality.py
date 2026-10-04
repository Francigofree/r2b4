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
    low = config.wheel_pi.velocity_unreliable_below_mps
    reliable = config.wheel_pi.minimum_reliable_speed_mps
    band = reliable - low
    middle, near_low, below = low + band * .5, low + band * .01, low * .5
    controller = WheelActuatorController(config.speed_map, config.wheel_pi)
    restored = None
    for tick, speed in enumerate((0., below, low*.9, low, middle, reliable, .19,
                                  middle, near_low, below, -below, -reliable, -.19)):
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
            frame = _feedback(tick, below)
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
            DataField(v.key, below) if v.key == "left_mps" and tick else v
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
    # becomes unusable. Otherwise crossing the lower confidence bound erases an integral
    # and steps the wheel back to the (higher) feed-forward output. Sample the
    # configured confidence band rather than freezing a historical tuning limit.
    for sign in (1, -1):
        controller = WheelActuatorController(config.speed_map, config.wheel_pi)
        for tick in range(20):
            frame = _feedback(tick, sign * .23)
            wheels = WheelVelocitySetpoint(frame.context, sign * .15, sign * .15)
            output = controller(wheels, frame)
        feedforward = config.speed_map.lookup("left", sign * .15)[0]
        strong_correction = abs(output.left_normalized - feedforward)
        assert strong_correction > .01
        for tick, speed in enumerate((middle, near_low, below), 20):
            frame = _feedback(tick, sign * speed)
            wheels = replace(wheels, context=frame.context)
            output = controller(wheels, frame)
            correction = abs(output.left_normalized - feedforward)
            if speed == near_low:
                assert correction < strong_correction * .1
                restored = WheelActuatorController(config.speed_map, config.wheel_pi)
                restored.restore(controller.checkpoint())
            if speed == below:
                assert output == restored(wheels, frame)
                assert correction == pytest.approx(0.0)

    # Clean initial standstill can acquire motion within the ordinary budget.
    # Losing already observed edges holds BOTH wheels, even when acquisitions
    # remain fresh and alternating sides would otherwise renew the watchdog.
    controller = WheelActuatorController(config.speed_map, config.wheel_pi)
    restored = None
    fault_tick = None
    for tick in range(24):
        f = _feedback(tick, reliable)
        values = {v.key: v.value for v in f.accepted[0].values}
        if tick < 2:
            sides = ("left", "right")
        elif tick == 2:
            sides = ()
        else:
            sides = ("left",) if tick % 2 else ("right",)
        for side in sides:
            values[f"{side}_mps"] = 0.0
            values[f"{side}_estimation_timebase"] = "TICK_SNAPSHOT"
        if tick == 1:
            # First raw pulses can have a GPIO timebase without a qualified
            # fit. This is startup fill, not loss of previously trusted motion.
            values.update(trust=0.0, left_measurement_trust=0.0,
                          right_measurement_trust=0.0,
                          left_estimation_timebase=None,
                          right_estimation_timebase="GPIO_EDGE_HISTORY")
        f = replace(f, accepted=(replace(f.accepted[0], values=tuple(
            DataField(k, v) for k, v in values.items())),))
        wheels = WheelVelocitySetpoint(f.context, .19, .19,
                                       velocity_transition_until_ns=1_800_000_000)
        try:
            output = controller(wheels, f)
        except ValueError as exc:
            assert "MISSING_EDGE:" in str(exc)
            with pytest.raises(ValueError, match="MISSING_EDGE:"):
                restored(wheels, f)
            fault_tick = tick
            break
        if restored is not None:
            assert restored(wheels, f) == output
        if tick < 2:
            assert output.left_normalized > 0 and output.right_normalized > 0
        elif tick > 2:
            assert output.left_normalized == output.right_normalized == 0.0
        if tick == 5:
            restored = WheelActuatorController(config.speed_map, config.wheel_pi)
            restored.restore(controller.checkpoint())
    assert fault_tick is not None
    assert (fault_tick - 3) * 20_000_000 < 800_000_000

    # The live failure changed from weak fit to edge loss after spending the
    # ordinary budget. Do not forgive that time or label the healthy wheel lost.
    controller = WheelActuatorController(config.speed_map, config.wheel_pi)
    for tick in range(19):
        f = _feedback(tick, below)
        values = {v.key: v.value for v in f.accepted[0].values}
        values['right_mps'] = reliable
        if tick == 18:
            values.update(left_mps=0.0, left_estimation_timebase="TICK_SNAPSHOT")
        f = replace(f, accepted=(replace(f.accepted[0], values=tuple(
            DataField(k, v) for k, v in values.items())),))
        wheels = WheelVelocitySetpoint(f.context, .19, .19,
                                       velocity_transition_until_ns=1_800_000_000)
        if tick == 18:
            with pytest.raises(ValueError, match="MISSING_EDGE:left$"):
                controller(wheels, f)
        else:
            controller(wheels, f)
    assert controller(replace(wheels, left_mps=0.0, right_mps=0.0), f).left_normalized == 0.0
