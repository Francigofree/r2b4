"""Finite low-speed transitions preserve counter evidence and fail closed."""
from dataclasses import replace

import pytest

from rig import resolved_config
from v3.adapters.counter_encoder import (
    NativeCounterEncoderBackend, SignedPulseCounterSnapshot, SignedPulseEdge,
)
from v3.adapters.live_encoder import NativeEncoderSource
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


class _Counter:
    running = True

    def __init__(self):
        self.value = SignedPulseCounterSnapshot(0)

    def snapshot(self):
        return self.value


class _NativeFeedback:
    """Physical signed histories through the resolved production encoder path."""

    def __init__(self, config):
        self.counters = {side: _Counter() for side in ("left", "right")}
        self.config = config.runtime.sensor_inputs.inputs.encoder_backend
        self.source = NativeEncoderSource(NativeCounterEncoderBackend(
            self.counters["left"], self.counters["right"], self.config,
        ), config.runtime.sensor_inputs.inputs.encoder_source)
        self.previous_ns = 1_000_000_000

    def frame(self, tick, left_mps, right_mps, *, now_ns=None):
        now_ns = 1_000_000_000 + tick * 20_000_000 if now_ns is None else now_ns
        for side, speed in (("left", left_mps), ("right", right_mps)):
            counter = self.counters[side]
            old = counter.value
            edges = list(old.edge_history)
            count, direction = old.pulse_count, old.confirmed_direction
            if speed:
                direction = 1 if speed > 0 else -1
                interval = round(getattr(self.config, f"{side}_step_distance_m") / abs(speed) * 1e9)
                for timestamp in range(self.previous_ns + interval, now_ns + 1, interval):
                    count += direction
                    edges.append(SignedPulseEdge(timestamp, count))
            counter.value = replace(old, pulse_count=count, edge_history=tuple(edges[-128:]),
                                    confirmed_direction=direction,
                                    # Rejected/pending A callbacks and receipt
                                    # can advance without an accepted signed edge.
                                    last_a_timestamp_ns=now_ns,
                                    last_callback_received_ns=now_ns)
        self.previous_ns = now_ns
        context = TickContext(tick, now_ns)
        snapshot = self.source.read(context)
        sample = snapshot.samples[0]
        wheel = Observation(sample.kind, sample.device_id, sample.sequence,
                            sample.captured_monotonic_ns, sample.values)
        degraded = () if snapshot.health.state.value == "OK" else (snapshot.health.device_id,)
        return AdmittedFrame(context, (wheel,), (), degraded_sources=degraded)


def _warm_native_feedback(resolved, deadline, *, side=None, sign=1):
    config = resolved.runtime.composition.live_control.control
    native = _NativeFeedback(resolved)
    controller = WheelActuatorController(config.speed_map, config.wheel_pi)
    low = config.wheel_pi.velocity_unreliable_below_mps * .6
    for tick in range(30):
        measured = {s: -sign * low if side is None or s == side else .25
                    for s in ("left", "right")}
        frame = native.frame(tick, measured["left"], measured["right"])
        wheels = WheelVelocitySetpoint(frame.context,
            -sign * low if side is None or side == "left" else .19,
            -sign * low if side is None or side == "right" else .19,
            velocity_transition_until_ns=deadline)
        controller(wheels, frame)
    return native, controller


def test_motion_native_reversal_pause_continues_and_recovers_per_wheel_pi():
    resolved = resolved_config()
    config = resolved.runtime.composition.live_control.control
    for side in ("left", "right"):
        for sign in (1, -1):
            native, controller = _warm_native_feedback(resolved, 2_400_000_000,
                                                       side=side, sign=sign)
            restored = None
            snapshots = 0
            for tick in range(30, 46):
                paused = tick < 37
                speed = 0.0 if paused else sign * .25
                frame = native.frame(tick, speed if side == "left" else .25,
                                     speed if side == "right" else .25)
                reference = sign * (min(.012 * (tick - 29), .12) if paused else .19)
                wheels = WheelVelocitySetpoint(frame.context,
                    reference if side == "left" else .19,
                    reference if side == "right" else .19,
                    velocity_transition_until_ns=3_000_000_000 + tick * 20_000_000)
                output = controller(wheels, frame)
                if restored is not None:
                    assert restored(wheels, frame) == output
                values = {v.key: v.value for v in frame.accepted[0].values}
                if values[f"{side}_estimation_timebase"] == "TICK_SNAPSHOT":
                    snapshots += 1
                    assert getattr(output, f"{side}_normalized") == pytest.approx(
                        config.speed_map.lookup(side, reference)[0])
                    other = "right" if side == "left" else "left"
                    assert 0 < getattr(output, f"{other}_normalized") < config.speed_map.lookup(other, .19)[0]
                    if restored is None:
                        restored = WheelActuatorController(config.speed_map, config.wheel_pi)
                        restored.restore(controller.checkpoint())
                assert sign * getattr(output, f"{side}_normalized") > 0
            assert snapshots > 0
            assert abs(getattr(output, f"{side}_normalized")) < abs(config.speed_map.lookup(side, reference)[0])


def test_motion_native_reversal_pause_keeps_original_deadlines_and_fails_closed():
    from v3.adapters.fake_edges import FakeHal
    from v3.contracts import LifecycleState, SafetyDecision
    from v3.layers.l12_safety_final import FinalSafetyGate

    resolved = resolved_config()
    config = resolved.runtime.composition.live_control.control
    for original_deadline in (1_740_000_000, 2_400_000_000):
        native, controller = _warm_native_feedback(resolved, original_deadline)
        restored = WheelActuatorController(config.speed_map, config.wheel_pi)
        restored.restore(controller.checkpoint())
        last_edge = min(c.value.edge_history[-1].timestamp_ns for c in native.counters.values())
        deadline = min(original_deadline, last_edge + config.wheel_pi.max_feedback_uncertainty_ns)
        # Planner deadlines, magnitudes, active sides and fresh acquisitions
        # all change while the original uncertainty episode remains continuous.
        for tick in range(30, 44):
            now = min(1_000_000_000 + tick * 20_000_000, deadline - 1)
            frame = native.frame(tick, 0., 0., now_ns=now)
            target = .02 if tick % 2 else .04
            wheels = WheelVelocitySetpoint(frame.context,
                target if tick % 2 else 0., 0. if tick % 2 else target,
                velocity_transition_until_ns=now + 2_000_000_000)
            output = controller(wheels, frame)
            assert restored(wheels, frame) == output
            assert output.left_normalized > 0 or output.right_normalized > 0
            if now == deadline - 1:
                break
        frame = native.frame(tick + 1, 0., 0., now_ns=deadline)
        wheels = replace(wheels, context=frame.context, left_mps=.02, right_mps=.02)
        for candidate in (controller, restored):
            with pytest.raises(ValueError, match="MISSING_EDGE:"):
                candidate(wheels, frame)
        writer = FakeHal()
        final = FinalSafetyGate(writer).finalize(frame.context, None, (), LifecycleState.ACTIVE, "L11_ERROR")
        assert final.safety_decision is SafetyDecision.FAULT
        assert not final.enabled and final.left_output == final.right_output == 0
        assert writer.writes == (final,)

    # Native stale/timing/counter errors and ordinary same-direction edge
    # loss still reach the existing HOLD/FAULT path after a spent watchdog.
    for failure in ("same_direction", "speed_boundary", "no_original_deadline",
                    "stale_acquisition", "stale_edges", "read_error", "stopped_counter", "invalid_timing"):
        native, controller = _warm_native_feedback(resolved, 2_400_000_000)
        for tick in range(30, 36):
            frame = native.frame(tick, 0., 0.)
        context = frame.context
        target = .02
        if failure == "same_direction":
            target = -target
        elif failure == "speed_boundary":
            target = config.wheel_pi.velocity_unreliable_below_mps
        elif failure == "no_original_deadline":
            checkpoint = controller.checkpoint()
            controller.restore(replace(checkpoint, feedback_transition_until_ns=None))
        elif failure == "stale_acquisition":
            context = TickContext(36, context.monotonic_ns + config.wheel_pi.max_feedback_age_ns + 1)
            frame = replace(frame, context=context)
        else:
            if failure == "read_error":
                counter = native.counters["left"]
                counter.value = replace(counter.value, read_errors=1)
            elif failure == "stopped_counter":
                native.counters["left"].running = False
            elif failure == "invalid_timing":
                counter = native.counters["left"]
                edges = counter.value.edge_history
                counter.value = replace(counter.value, edge_history=edges[:-1] + (
                    replace(edges[-1], timestamp_ns=2_000_000_000),))
            elif failure == "stale_edges":
                counter = native.counters["left"]
                old = counter.value
                edge = SignedPulseEdge(old.edge_history[-1].timestamp_ns + 1,
                                       old.pulse_count - 1)
                counter.value = replace(old, pulse_count=edge.pulse_count,
                                        edge_history=old.edge_history + (edge,))
            frame = native.frame(36, 0., 0.)
            context = frame.context
        wheels = WheelVelocitySetpoint(context, target, target,
                                       velocity_transition_until_ns=3_000_000_000)
        with pytest.raises(ValueError):
            controller(wheels, frame)
        writer = FakeHal()
        final = FinalSafetyGate(writer).finalize(context, None, (), LifecycleState.ACTIVE, "L11_ERROR")
        assert not final.enabled and final.left_output == final.right_output == 0
