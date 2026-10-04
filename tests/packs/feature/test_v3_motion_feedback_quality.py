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


def test_motion_low_speed_feedback_uses_feedforward_recovers_and_preserves_watchdogs():
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
        if abs(speed) < reliable:
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

    # A previously learned correction cannot survive in an uncertain fit band.
    # Signed-edge liveness is independent; the intermediate fit gives no PI.
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
            assert correction == pytest.approx(0.0)
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
        self.pulse_fraction = {side: 0.0 for side in self.counters}

    def frame(self, tick, left_mps, right_mps, *, now_ns=None):
        now_ns = 1_000_000_000 + tick * 20_000_000 if now_ns is None else now_ns
        for side, speed in (("left", left_mps), ("right", right_mps)):
            counter = self.counters[side]
            old = counter.value
            edges = list(old.edge_history)
            count, direction = old.pulse_count, old.confirmed_direction
            if speed:
                next_direction = 1 if speed > 0 else -1
                if next_direction != direction:
                    self.pulse_fraction[side] = 0.0
                direction = next_direction
                interval = getattr(self.config, f"{side}_step_distance_m") / abs(speed) * 1e9
                fraction = self.pulse_fraction[side]
                total = fraction + (now_ns - self.previous_ns) / interval
                for index in range(int(total)):
                    timestamp = round(self.previous_ns + (index + 1 - fraction) * interval)
                    count += direction
                    edges.append(SignedPulseEdge(timestamp, count))
                self.pulse_fraction[side] = total - int(total)
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


def test_native_above_floor_reversal_without_ramp_deadline_has_one_edge_budget():
    resolved = resolved_config()
    config = resolved.runtime.composition.live_control.control
    for flips in (False, True):
        native, controller = _warm_native_feedback(resolved, None, side="left", sign=1)
        restored = None
        original_start = None
        fault = None
        for tick in range(30, 80):
            # No new left physical edge; right feedback and callback receipt
            # keep advancing. Flip-back and active zeros cannot revive old edges.
            f = native.frame(tick, 0., .25)
            target = (0. if tick % 3 == 1 else -.19 if tick % 3 == 2 else .19) if flips else .19
            wheels = WheelVelocitySetpoint(f.context, target, .19)
            try:
                output = controller(wheels, f)
            except ValueError as exc:
                assert "feedback remained uncertain too long" in str(exc)
                if restored is not None:
                    with pytest.raises(ValueError):
                        restored(wheels, f)
                fault = f.context.monotonic_ns
                break
            if restored is not None:
                assert restored(wheels, f) == output
            state = controller.checkpoint()
            if original_start is None:
                original_start = state.feedback_uncertain_since_ns
            assert state.feedback_uncertain_since_ns == original_start
            assert state.feedback_transition_until_ns is None
            if target:
                assert f.context.monotonic_ns < original_start + config.wheel_pi.max_feedback_uncertainty_ns
            if tick == 30:
                assert output.left_normalized == pytest.approx(config.speed_map.lookup("left", .19)[0])
                restored = WheelActuatorController(config.speed_map, config.wheel_pi)
                restored.restore(state)
        assert fault is not None
        # A zero left demand needs no left liveness check; the first subsequent
        # active demand must fault at the same original deadline.
        assert fault <= original_start + config.wheel_pi.max_feedback_uncertainty_ns + 40_000_000

    # A full active zero likewise preserves the episode; explicit revocation
    # is the only zero that resets it. Post-epoch matching physical edges recover.
    native, controller = _warm_native_feedback(resolved, None, side="left", sign=1)
    f = native.frame(30, 0., .25)
    controller(WheelVelocitySetpoint(f.context, .19, .19), f)
    start = controller.checkpoint().feedback_uncertain_since_ns
    f = native.frame(31, 0., 0.)
    controller(WheelVelocitySetpoint(f.context, 0., 0.), f)
    assert controller.checkpoint().feedback_uncertain_since_ns == start
    f = native.frame(32, .25, .25)
    controller(WheelVelocitySetpoint(f.context, .19, .19), f)
    f = native.frame(33, .25, .25)
    controller(WheelVelocitySetpoint(f.context, .19, .19), f)
    assert controller.checkpoint().feedback_uncertain_since_ns is None
    f = native.frame(34, 0., 0.)
    controller(WheelVelocitySetpoint(f.context, 0., 0., motion_revoked=True), f)
    assert controller.checkpoint().left_direction_epoch_ns is None


def test_motion_native_degraded_start_hold_resume_and_edge_watchdogs():
    from rig import healthy_localization
    from v3.adapters.fake_edges import FakeHal
    from v3.contracts import (LifecycleState, LocalizationRequirement, MotionIntent,
                              QualityState, RobotEstimate, SafetyDecision)
    from v3.layers.l9_operational_constraints import OperationalConstraintLayer
    from v3.layers.l10_chassis_control import DifferentialDriveKinematics
    from v3.layers.l12_safety_final import FinalSafetyGate

    resolved = resolved_config()
    config = resolved.runtime.composition.live_control.control
    limiter = OperationalConstraintLayer(config.operational_constraints)
    chassis = DifferentialDriveKinematics(config.chassis_control)
    controller = WheelActuatorController(config.speed_map, config.wheel_pi)
    native = _NativeFeedback(resolved)
    restored = None
    subfloor_after_watchdog = 0
    saw_first_edge_without_fit = False
    previous = None
    # Both startup and HOLD recovery have a real DEGRADED L9 pivot ramp.
    # Simulate static friction for 320 ms, then slow accepted signed progress:
    # the fit remains below control-grade speed while physical liveness is good.
    for tick in range(500):
        hold = 80 <= tick < 85
        starting = tick < 18 or 85 <= tick < 103
        measured_left = 0.0 if starting or hold else -.02
        measured_right = 0.0 if starting or hold else .02
        frame = native.frame(tick, measured_left, measured_right)
        context = frame.context
        quality = healthy_localization(local_translation=QualityState.DEGRADED,
                                       heading=QualityState.DEGRADED,
                                       global_position=QualityState.LOST)
        estimate = RobotEstimate(context, 'odom', 0., 0., 0., 0., 0., (0.,) * 25,
                                 localization_quality=quality)
        motion = MotionIntent(context, 0., 0. if hold else config.operational_constraints.wheel_limits.target_center_spin_rad_s,
            config.motion_realization.horizon_ns, config.mission.default_constraints,
            stop_reason='LOCALIZATION_HOLD' if hold else None,
            transition_allowed=not hold,
            localization_requirement=LocalizationRequirement(False, True, False))
        wheels = chassis(limiter.evaluate(motion, estimate))
        output = controller(wheels, frame)
        if restored is not None:
            assert restored(wheels, frame) == output
        if hold or tick == 0:
            assert output.left_normalized == output.right_normalized == 0.
        else:
            assert output.left_normalized < 0 < output.right_normalized
            assert output.left_normalized == pytest.approx(config.speed_map.lookup('left', wheels.left_mps)[0])
            assert output.right_normalized == pytest.approx(config.speed_map.lookup('right', wheels.right_mps)[0])
            if previous is not None:
                budget = min(config.operational_constraints.max_acceleration_mps2,
                    config.operational_constraints.max_angular_acceleration_rad_s2 *
                    config.chassis_control.track_width_m / 2) * config.operational_constraints.degraded_acceleration_scale * .02
                assert abs(wheels.left_mps - previous.left_mps) <= budget + 1e-12
                assert abs(wheels.right_mps - previous.right_mps) <= budget + 1e-12
            if (tick > 13 and tick < 80 or tick > 98) and abs(wheels.left_mps) < config.speed_map.minimum_continuous_speed_mps:
                subfloor_after_watchdog += 1
            values = {v.key: v.value for v in frame.accepted[0].values}
            if values['left_last_accepted_edge_timestamp_ns'] is not None and values['left_estimation_timebase'] is None:
                saw_first_edge_without_fit = True
        if tick == 105:
            restored = WheelActuatorController(config.speed_map, config.wheel_pi)
            restored.restore(controller.checkpoint())
        previous = None if hold else wheels
    assert subfloor_after_watchdog > 40
    assert saw_first_edge_without_fit

    # A stopped left wheel must fail at its own last accepted edge deadline.
    # The right wheel, fresh snapshots, rejected callbacks and moving L9
    # deadlines cannot renew it; the real L12 error path writes zero FAULT.
    last_edge = native.counters['left'].value.edge_history[-1].timestamp_ns
    deadline = last_edge + config.wheel_pi.max_feedback_uncertainty_ns
    for tick in range(500, 520):
        now = min(1_000_000_000 + tick * 20_000_000, deadline)
        frame = native.frame(tick, 0., .02, now_ns=now)
        wheels = WheelVelocitySetpoint(frame.context, -.19, .19,
                                       velocity_transition_until_ns=now + 5_000_000_000)
        if now < deadline:
            assert controller(wheels, frame).left_normalized < 0
            continue
        with pytest.raises(ValueError, match='MISSING_EDGE:left$'):
            controller(wheels, frame)
        writer = FakeHal()
        final = FinalSafetyGate(writer).finalize(frame.context, None, (), LifecycleState.ACTIVE, 'L11_ERROR')
        assert final.safety_decision is SafetyDecision.FAULT
        assert not final.enabled and final.left_output == final.right_output == 0
        assert writer.writes == (final,)
        break
    else:
        pytest.fail('silent left encoder did not spend its own accepted-edge watchdog')

    # No first accepted edge has only the original L9 startup budget. Direction
    # changes and publications cannot prolong it; a fresh wrong-direction fit
    # cannot substitute for requested-direction signed progress either.
    for failure in ('no_edges', 'wrong_direction', 'direction_counter_mismatch'):
        native = _NativeFeedback(resolved)
        controller = WheelActuatorController(config.speed_map, config.wheel_pi)
        initial_deadline = 1_800_000_000
        for tick in range(41):
            speed = -.25 if failure == 'wrong_direction' else .25 if failure == 'direction_counter_mismatch' else 0.
            frame = native.frame(tick, speed, speed)
            if failure == 'direction_counter_mismatch':
                observation = frame.accepted[0]
                frame = replace(frame, accepted=(replace(observation, values=tuple(
                    DataField(v.key, -1) if v.key.endswith('confirmed_direction') else v
                    for v in observation.values)),))
            target = -.02 if failure == 'no_edges' and tick % 2 else .02
            wheels = WheelVelocitySetpoint(frame.context, target, .02,
                velocity_transition_until_ns=initial_deadline + tick * 20_000_000)
            try:
                controller(wheels, frame)
            except ValueError:
                assert frame.context.monotonic_ns <= initial_deadline
                break
        else:
            pytest.fail(f'{failure} renewed the original startup uncertainty deadline')

    # Keep independent source-failure coverage on the native L11 -> L12 path.
    # The startup allowance never covers stale, corrupt or stopped encoders.
    for failure in ('stale_acquisition', 'stale_edges', 'read_error',
                    'stopped_counter', 'invalid_timing'):
        native, controller = _warm_native_feedback(resolved, 2_400_000_000)
        counter = native.counters['left']
        old = counter.value
        if failure == 'read_error':
            counter.value = replace(old, read_errors=1)
        elif failure == 'stopped_counter':
            counter.running = False
        elif failure == 'invalid_timing':
            counter.value = replace(old, edge_history=old.edge_history[:-1] + (
                replace(old.edge_history[-1], timestamp_ns=2_000_000_000),))
        elif failure == 'stale_edges':
            edge = SignedPulseEdge(old.edge_history[-1].timestamp_ns + 1,
                                   old.pulse_count - 1)
            counter.value = replace(old, pulse_count=edge.pulse_count,
                                    edge_history=old.edge_history + (edge,))
        now = 1_900_000_000 if failure == 'stale_edges' else 1_600_000_000
        frame = native.frame(30, 0., 0., now_ns=now)
        if failure == 'stale_acquisition':
            context = TickContext(31, now + config.wheel_pi.max_feedback_age_ns + 1)
            frame = replace(frame, context=context)
            wheels = WheelVelocitySetpoint(context, -.02, -.02,
                                           velocity_transition_until_ns=3_000_000_000)
            held = controller(wheels, frame)
            assert held.left_normalized == held.right_normalized == 0.
            context = TickContext(32, context.monotonic_ns + config.wheel_pi.max_feedback_uncertainty_ns)
            frame = replace(frame, context=context)
        wheels = WheelVelocitySetpoint(frame.context, -.02, -.02,
                                       velocity_transition_until_ns=3_000_000_000)
        with pytest.raises(ValueError):
            controller(wheels, frame)
        writer = FakeHal()
        final = FinalSafetyGate(writer).finalize(frame.context, None, (), LifecycleState.ACTIVE, 'L11_ERROR')
        assert final.safety_decision is SafetyDecision.FAULT
        assert not final.enabled and final.left_output == final.right_output == 0.
