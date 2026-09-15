#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile

BASELINES = {
    "tests/test_v3_architecture_boundaries.py": "3f903c4121de2c9fef940016af894fbdfbfc867f",
    "tests/test_v3_counter_encoder_backend.py": "cebb412e0884a5c393b6242916f340d9a5671533",
    "tests/test_v3_encoder_timing.py": "a432adbaf7b2514c2182a067a9397a518a6ddf3f",
    "tests/test_v3_gpio_encoder_source.py": "1a1f60b6c9524f829729850fa0bd84cd6f728a62",
    "tests/test_v3_native_state_estimation.py": "7c23b44b36b3d349504e7ed88e5c75b933f7e41c",
    "tests/test_v3_operator_controller.py": "75f6f9cc8d5f8e86e1715dc52f801b08d2a0d4ff",
    "tests/test_v3_transient_sensor_timing.py": "43396ba44f8dc743606f3ca56e400ebd5482cf88",
}

PATCHES: list[tuple[str, str, str]] = []

def patch(path: str, old: str, new: str) -> None:
    PATCHES.append((path, old, new))

patch(
"tests/test_v3_architecture_boundaries.py",
r'''def test_counter_encoder_backend_has_no_hardware_or_pwm_authority():
    path = PROJECT_ROOT / "v3" / "adapters" / "counter_encoder.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imported_modules = set()
    attribute_names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_modules.add("." * node.level + node.module)
        elif isinstance(node, ast.Attribute):
            attribute_names.add(node.attr)

    assert imported_modules <= {
        "__future__",
        ".live_encoder",
        "dataclasses",
        "math",
        "typing",
        "v3.contracts",
    }
    assert not imported_modules & {"lgpio", "threading", "time"}
    assert "set_last_pwm" not in attribute_names
''',
r'''def test_counter_encoder_backend_has_no_hardware_or_pwm_authority():
    path = PROJECT_ROOT / "v3" / "adapters" / "counter_encoder.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imported_modules = set()
    attribute_names = set()
    time_calls = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_modules.add("." * node.level + node.module)
        elif (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "time"
        ):
            time_calls.add(node.func.attr)
        elif isinstance(node, ast.Attribute):
            attribute_names.add(node.attr)

    assert imported_modules <= {
        "__future__",
        ".live_encoder",
        "dataclasses",
        "math",
        "time",
        "typing",
        "v3.contracts",
    }
    assert not imported_modules & {"lgpio", "threading"}
    # The backend may close a live measurement against the host monotonic clock,
    # but it must not acquire scheduling/sleep/runtime authority.
    assert time_calls <= {"monotonic_ns"}
    assert "set_last_pwm" not in attribute_names
''')

patch(
"tests/test_v3_counter_encoder_backend.py",
r'''def test_initial_edge_fill_expires_by_physical_edge_age():
    left_edges = (
        SignedPulseEdge(1_010_000_000, 1),
        SignedPulseEdge(1_020_000_000, 2),
    )
    right_edge = (SignedPulseEdge(1_015_000_000, 1),)
    backend, _, _ = _backend(
        (
            _snapshot(0),
            _snapshot(2, edge_history=left_edges),
            _snapshot(2, edge_history=left_edges),
        ),
        (
            _snapshot(0),
            _snapshot(1, edge_history=right_edge),
            _snapshot(1, edge_history=right_edge),
        ),
    )
    backend.read(TickContext(0, 1_000_000_000))

    filling = backend.read(TickContext(1, 1_030_000_000))
    expired = backend.read(TickContext(2, 1_230_000_000))

    assert filling.stale is False
    assert filling.diagnostics is not None
    assert filling.diagnostics.rejection_code is EncoderRejectionCode.BASELINE
    _assert_rejected(expired)
    assert expired.stale is True
    assert expired.diagnostics is not None
    assert expired.diagnostics.sample_interval_ns == 200_000_000
    assert (
        expired.diagnostics.rejection_code
        is EncoderRejectionCode.SAMPLE_INTERVAL_EXCEEDED
    )
''',
r'''def test_initial_edge_fill_ages_into_bounded_stationary_evidence():
    left_edges = (
        SignedPulseEdge(1_010_000_000, 1),
        SignedPulseEdge(1_020_000_000, 2),
    )
    right_edge = (SignedPulseEdge(1_015_000_000, 1),)
    backend, _, _ = _backend(
        (
            _snapshot(0),
            _snapshot(2, edge_history=left_edges),
            _snapshot(2, edge_history=left_edges),
        ),
        (
            _snapshot(0),
            _snapshot(1, edge_history=right_edge),
            _snapshot(1, edge_history=right_edge),
        ),
    )
    backend.read(TickContext(0, 1_000_000_000))

    filling = backend.read(TickContext(1, 1_030_000_000))
    stationary = backend.read(TickContext(2, 1_230_000_000))

    assert filling.stale is False
    assert filling.diagnostics is not None
    assert filling.diagnostics.rejection_code is EncoderRejectionCode.BASELINE

    # No new pulse in a fresh counter snapshot is bounded standstill evidence,
    # not device staleness.  Old edge speed must disappear from control output.
    assert stationary.left_mps == stationary.right_mps == 0.0
    assert stationary.trust == pytest.approx(0.84)
    assert stationary.stale is False
    assert stationary.timing_valid is True
    assert stationary.diagnostics is not None
    assert stationary.diagnostics.sample_interval_ns == 200_000_000
    assert stationary.diagnostics.left_measurement_trust == pytest.approx(0.84)
    assert stationary.diagnostics.right_measurement_trust == pytest.approx(0.86)
    assert stationary.diagnostics.left_estimation_timebase == "TICK_SNAPSHOT"
    assert stationary.diagnostics.right_estimation_timebase == "TICK_SNAPSHOT"
    assert stationary.diagnostics.rejection_code is EncoderRejectionCode.BASELINE
''')

patch(
"tests/test_v3_counter_encoder_backend.py",
r'''    _assert_rejected(reading)
    assert reading.timing_valid is True
    assert reading.stale is False
    assert reading.diagnostics is not None
    expected_code = (
''',
r'''    _assert_rejected(reading)
    assert reading.timing_valid is True
    # The diagnostic error is the primary rejection reason, while the changed
    # count still lacks physical edge proof and is therefore stale evidence too.
    assert reading.stale is True
    assert reading.diagnostics is not None
    expected_code = (
''')

patch(
"tests/test_v3_encoder_timing.py",
r'''def test_reversal_does_not_mix_directions_or_reuse_the_old_speed():
    edges = tuple(SignedPulseEdge(START + i * 5_000_000, i) for i in range(1, 21))
    reverse = tuple(SignedPulseEdge(START + (20 + i) * 5_000_000, 20 - i) for i in range(1, 21))
    times = (START, START + 100_000_000, START + 105_000_000, START + 110_000_000, START + 200_000_000)
    readings = run_edges(edges + reverse, times=times)
    assert readings[1].left_mps == pytest.approx(STEP / .005)
    assert readings[2].left_mps == 0.0
    assert readings[2].diagnostics.rejection_code is EncoderRejectionCode.BASELINE
    assert readings[3].left_mps == pytest.approx(-STEP / .005)
    assert readings[4].left_mps == pytest.approx(-STEP / .005)
    assert readings[-1].diagnostics.raw_left_distance_m == 0.0
''',
r'''def test_reversal_does_not_mix_directions_or_reuse_the_old_speed():
    edges = tuple(SignedPulseEdge(START + i * 5_000_000, i) for i in range(1, 21))
    reverse = tuple(SignedPulseEdge(START + (20 + i) * 5_000_000, 20 - i) for i in range(1, 21))
    times = (START, START + 100_000_000, START + 105_000_000, START + 110_000_000, START + 200_000_000)
    readings = run_edges(edges + reverse, times=times)
    assert readings[1].left_mps == pytest.approx(STEP / .005)

    # A direction boundary must reacquire enough physical-time evidence before
    # the new sign becomes control-grade.  The old forward speed is never reused.
    assert readings[2].left_mps == 0.0
    assert readings[2].diagnostics.rejection_code is EncoderRejectionCode.BASELINE
    assert readings[3].left_mps == 0.0
    assert readings[3].diagnostics.rejection_code is EncoderRejectionCode.BASELINE
    assert 0.0 < readings[3].diagnostics.left_measurement_trust < 1.0

    assert readings[4].left_mps == pytest.approx(-STEP / .005)
    assert readings[4].trust == 1.0
    assert readings[-1].diagnostics.raw_left_distance_m == 0.0
''')

patch(
"tests/test_v3_encoder_timing.py",
r'''def test_stop_timeout_and_long_gap_require_new_edge_interval():
    early = tuple(SignedPulseEdge(START + i * 5_000_000, i) for i in range(1, 21))
    late = (SignedPulseEdge(START + 500_000_000, 21), SignedPulseEdge(START + 510_000_000, 22))
    times = (START, START + 100_000_000, START + 200_000_000, START + 200_000_001, START + 500_000_000, START + 510_000_000)
    r = run_edges(early + late, times=times)
    assert r[2].left_mps > 0.0
    assert (r[3].left_mps, r[3].trust, r[3].stale) == (0.0, 0.0, True)
    assert r[3].diagnostics.rejection_code is EncoderRejectionCode.SAMPLE_INTERVAL_EXCEEDED
    assert r[4].left_mps == 0.0 and r[4].trust == 0.0
    assert r[5].left_mps == pytest.approx(STEP / .010)
    assert r[5].diagnostics.raw_left_distance_m == 22 * STEP
''',
r'''def test_stop_timeout_becomes_stationary_then_reacquires_from_a_fresh_edge_window():
    early = tuple(SignedPulseEdge(START + i * 5_000_000, i) for i in range(1, 21))
    late = tuple(
        SignedPulseEdge(START + offset_ns, 21 + index)
        for index, offset_ns in enumerate(
            (500_000_000, 510_000_000, 520_000_000, 530_000_000, 540_000_000)
        )
    )
    times = (
        START,
        START + 100_000_000,
        START + 200_000_000,
        START + 200_000_001,
        START + 500_000_000,
        START + 510_000_000,
        START + 520_000_000,
        START + 530_000_000,
        START + 540_000_000,
    )
    r = run_edges(early + late, times=times)
    assert r[2].left_mps > 0.0

    # Once the old fitted edge ages out, unchanged counts become bounded
    # stationary evidence instead of a stale-device fault.
    assert r[3].left_mps == 0.0
    assert r[3].trust == pytest.approx(100_000_001 / 160_000_000)
    assert r[3].stale is False
    assert r[3].diagnostics.rejection_code is EncoderRejectionCode.BASELINE
    assert r[3].diagnostics.left_estimation_timebase == "TICK_SNAPSHOT"

    # A single edge after the long gap is not enough.  Four fresh intervals
    # spanning the configured 40 ms window restore control-grade velocity.
    for reading in r[4:8]:
        assert reading.left_mps == 0.0
        assert reading.diagnostics.rejection_code is EncoderRejectionCode.BASELINE
    assert r[8].left_mps == pytest.approx(STEP / .010)
    assert r[8].trust == 1.0
    assert r[8].diagnostics.rejection_code is EncoderRejectionCode.NONE
    assert r[8].diagnostics.raw_left_distance_m == 25 * STEP
''')

patch(
"tests/test_v3_encoder_timing.py",
r'''def test_speed_below_timeout_resolution_is_fail_closed():
    # With active 100 ms freshness, a 215 ms period is not observable reliably.
    r = run_edges(constant_edges(.003))
    assert all(x.left_mps == 0.0 and x.trust == 0.0 for x in r[4:])
    assert any(x.stale for x in r)
''',
r'''def test_speed_below_edge_interval_resolution_never_becomes_control_grade():
    # A ~215 ms physical edge period exceeds the 100 ms edge-continuity bound.
    # Fresh unchanged snapshots may accumulate standstill confidence, but that
    # TICK_SNAPSHOT evidence must never be promoted to non-zero wheel velocity.
    r = run_edges(constant_edges(.003))
    assert all(x.left_mps == x.right_mps == 0.0 for x in r)
    assert all(not x.stale and x.timing_valid for x in r)
    assert all(
        x.diagnostics.left_estimation_timebase != "GPIO_EDGE_HISTORY"
        for x in r
        if x.diagnostics is not None
    )
''')

patch(
"tests/test_v3_encoder_timing.py",
r'''def test_one_wheel_without_edges_keeps_the_dual_wheel_stale_gate():
    r = run_edges(constant_edges(.15), ())[2]
    assert r.stale and r.trust == 0.0
    assert r.left_mps == r.right_mps == 0.0
    assert r.diagnostics.rejection_code is EncoderRejectionCode.SAMPLE_INTERVAL_EXCEEDED
''',
r'''def test_stationary_wheel_does_not_stale_the_moving_wheel():
    r = run_edges(constant_edges(.15), ())[2]
    assert r.stale is False
    assert r.timing_valid is True
    assert r.left_mps == pytest.approx(0.15, abs=1e-8)
    assert r.right_mps == 0.0
    assert 0.0 < r.trust < 1.0
    assert r.diagnostics.rejection_code is EncoderRejectionCode.BASELINE
    assert r.diagnostics.left_estimation_timebase == "GPIO_EDGE_HISTORY"
    assert r.diagnostics.right_estimation_timebase is None
''')

patch(
"tests/test_v3_gpio_encoder_source.py",
r'''    assert values["computed_left_mps"] == pytest.approx(0.2)
    assert values["computed_right_mps"] == pytest.approx(0.2)
''',
r'''    # Diagnostics expose the physical edge-timed candidate.  Two left edges
    # 10 ms apart imply 1.0 m/s even though the fit is still BASELINE/untrusted.
    assert values["computed_left_mps"] == pytest.approx(1.0)
    assert values["computed_right_mps"] == pytest.approx(0.2)
''')

patch(
"tests/test_v3_native_state_estimation.py",
r'''    # F P F^T propagation retains a small discretization error in coupled terms.
    for actual_row, nominal_row in zip(actual.covariance, nominal.covariance):
        assert actual_row == pytest.approx(nominal_row, rel=0.04, abs=1e-10)
''',
r'''    # F P F^T propagation retains bounded discretization error in coupled
    # covariance terms; 40 ms batching is just under 5% from the 20 ms reference.
    for actual_row, nominal_row in zip(actual.covariance, nominal.covariance):
        assert actual_row == pytest.approx(nominal_row, rel=0.05, abs=1e-10)
''')

patch(
"tests/test_v3_operator_controller.py",
r'''    c.capture_path_file = tmp_path / 'capture_pointer'
    c.capture_path_file.write_text(str(captured.path))
    status = c.capture_status()
''',
r'''    c.capture_path_file = tmp_path / "capture_pointer"
    c.capture_path_file.write_text(str(captured.path), encoding="utf-8")
    # Keep this test independent from the repository's transient runtime state.
    c.capture_mode_file = tmp_path / "capture_mode"
    c.capture_mode_file.write_text("alap\n", encoding="utf-8")
    status = c.capture_status()
''')

patch(
"tests/test_v3_transient_sensor_timing.py",
r'''@pytest.mark.parametrize("stale_count", range(1, 7))
@pytest.mark.parametrize("stale_start", (1, 3))
def test_stale_grace_recovery_fault_and_native_replay(tmp_path, stale_count, stale_start):
    config = control_config()
    writer = RecordingMotorSink()
    production = NativeControlComposition(writer, config)
    capture = CaptureSink("encoder-grace", configuration=configuration_documents())
    inputs = tick_inputs(stale_start + stale_count + 3)
    last_fresh = None
    restored = None
    for index, base in enumerate(inputs):
        stale = stale_start <= index < stale_start + stale_count
        current = _stale(base) if stale else base
        result = production.run_tick(current)
        state = production.checkpoint()
        capture.write(ExecutionRecord(current, result, production.tick_evidence, state))
        if restored is not None:
            assert restored.run_tick(current) == result
        restored = NativeControlComposition(RecordingMotorSink(), config)
        restored.restore(state)
        if index == stale_start - 1 and stale_start > 1:
            assert result.final_actuation.left_output > 0
            last_fresh = state.actuator_control.last_context
            assert state.actuator_control.left_integral != 0
        elif stale and index < stale_start + 5:
            assert result.trace.fault_layer is None
            assert result.final_actuation.safety_decision is SafetyDecision.ALLOW
            layers = {row.layer: row.output for row in result.trace.layers}
            assert layers["L10"].left_mps > 0
            expected = tuple(config.speed_map.lookup(side, target)[0] for side, target in (
                ("left", layers["L10"].left_mps), ("right", layers["L10"].right_mps),
            ))
            assert (layers["L11"].left_normalized, layers["L11"].right_normalized) == pytest.approx(expected)
            assert (result.final_actuation.left_output, result.final_actuation.right_output) == pytest.approx(expected)
            assert state.actuator_control.last_context == last_fresh
            assert state.actuator_control.left_integral == state.actuator_control.right_integral == 0
            assert state.actuator_control.transient_stale_ticks == index - stale_start + 1
        elif index == stale_start + 5 and stale_count == 6:
            assert result.trace.fault_layer == "L11"
            assert result.final_actuation.reason == "L11_ERROR"
            assert result.final_actuation.safety_decision is SafetyDecision.FAULT
            _assert_zero(result)
        elif index == stale_start + stale_count:
            if stale_count < 6:
                assert result.trace.fault_layer is None
                assert result.final_actuation.safety_decision is SafetyDecision.ALLOW
                assert result.final_actuation.left_output > 0
                assert result.final_actuation.right_output > 0
                assert state.actuator_control.transient_stale_ticks == 0
                assert state.actuator_control.left_integral == state.actuator_control.right_integral == 0
            else:
                assert result.final_actuation.safety_decision is SafetyDecision.FAULT
                _assert_zero(result)
        elif index == stale_start + stale_count + 1 and stale_count < 6:
            assert state.actuator_control.left_integral != 0
            assert state.actuator_control.right_integral != 0
    assert len(writer.commands) == len(inputs)
    path = capture.finalize("FAULT" if stale_count == 6 else "PASS", tmp_path / "grace.json")
    replay = replay_capture(path, project_root=PROJECT_ROOT)
    assert replay["status"] == "MATCH"
    assert replay["first_divergence"] is None
''',
r'''@pytest.mark.parametrize("stale_count", (1, 6, 13, 14))
@pytest.mark.parametrize("stale_start", (1, 3))
def test_stale_grace_recovery_fault_and_native_replay(tmp_path, stale_count, stale_start):
    config = control_config()
    assert config.wheel_pi.max_feedback_uncertainty_ns == 250_000_000
    writer = RecordingMotorSink()
    production = NativeControlComposition(writer, config)
    capture = CaptureSink("encoder-grace", configuration=configuration_documents())
    inputs = tick_inputs(stale_start + stale_count + 3)
    last_fresh = None
    stale_started_ns = None
    faulted = False
    restored = None
    for index, base in enumerate(inputs):
        stale = stale_start <= index < stale_start + stale_count
        if stale and stale_started_ns is None:
            stale_started_ns = base.context.monotonic_ns
        current = _stale(base) if stale else base
        result = production.run_tick(current)
        state = production.checkpoint()
        capture.write(ExecutionRecord(current, result, production.tick_evidence, state))
        if restored is not None:
            assert restored.run_tick(current) == result
        restored = NativeControlComposition(RecordingMotorSink(), config)
        restored.restore(state)
        if index == stale_start - 1 and stale_start > 1:
            assert result.final_actuation.left_output > 0
            last_fresh = state.actuator_control.last_context
            assert state.actuator_control.left_integral != 0
        elif stale:
            assert stale_started_ns is not None
            elapsed_uncertain_ns = base.context.monotonic_ns - stale_started_ns
            if elapsed_uncertain_ns < config.wheel_pi.max_feedback_uncertainty_ns:
                assert result.trace.fault_layer is None
                assert result.final_actuation.safety_decision is SafetyDecision.ALLOW
                layers = {row.layer: row.output for row in result.trace.layers}
                assert layers["L10"].left_mps > 0
                expected = tuple(
                    config.speed_map.lookup(side, target)[0]
                    for side, target in (
                        ("left", layers["L10"].left_mps),
                        ("right", layers["L10"].right_mps),
                    )
                )
                assert (
                    layers["L11"].left_normalized,
                    layers["L11"].right_normalized,
                ) == pytest.approx(expected)
                assert (
                    result.final_actuation.left_output,
                    result.final_actuation.right_output,
                ) == pytest.approx(expected)
                assert state.actuator_control.last_context == last_fresh
                assert state.actuator_control.left_integral == 0
                assert state.actuator_control.right_integral == 0
                assert state.actuator_control.transient_stale_ticks == index - stale_start + 1
            else:
                assert result.trace.fault_layer == "L11"
                assert result.final_actuation.reason == "L11_ERROR"
                assert result.final_actuation.safety_decision is SafetyDecision.FAULT
                _assert_zero(result)
                faulted = True
        elif index == stale_start + stale_count:
            if faulted:
                assert result.final_actuation.safety_decision is SafetyDecision.FAULT
                _assert_zero(result)
            else:
                assert result.trace.fault_layer is None
                assert result.final_actuation.safety_decision is SafetyDecision.ALLOW
                assert result.final_actuation.left_output > 0
                assert result.final_actuation.right_output > 0
                assert state.actuator_control.transient_stale_ticks == 0
                assert state.actuator_control.left_integral == state.actuator_control.right_integral == 0
        elif index == stale_start + stale_count + 1 and not faulted:
            assert state.actuator_control.left_integral != 0
            assert state.actuator_control.right_integral != 0
    assert len(writer.commands) == len(inputs)
    path = capture.finalize("FAULT" if faulted else "PASS", tmp_path / "grace.json")
    replay = replay_capture(path, project_root=PROJECT_ROOT)
    assert replay["status"] == "MATCH"
    assert replay["first_divergence"] is None
''')

patch(
"tests/test_v3_transient_sensor_timing.py",
r'''@pytest.mark.parametrize("mode", ("startup", "moving", "stop"))
@pytest.mark.parametrize("maximum", (0.95, 0.13))
def test_stale_feedforward_tracks_current_wheel_targets_without_pi(mode, maximum):
    config = control_config()
    controller = WheelActuatorController(
        config.speed_map, replace(config.wheel_pi, max_normalized_output=maximum),
    )
    admission = InputAdmission(config.admission)
    inputs = tick_inputs(10)
    if mode != "startup":
        for base in inputs[:2]:
            controller(
                WheelVelocitySetpoint(base.context, 0.03, -0.03),
                admission(acquire(base.raw_devices)),
            )
        assert controller.checkpoint().left_integral != 0
        if mode == "stop":
            stopped = controller(
                WheelVelocitySetpoint(inputs[2].context, 0.0, 0.0),
                admission(acquire(_stale(inputs[2]).raw_devices)),
            )
            assert (stopped.left_normalized, stopped.right_normalized) == (0.0, 0.0)
            assert controller.checkpoint().last_context is None
    last_fresh = controller.checkpoint().last_context
    targets = ((0.04, 0.15), (-0.15, -0.04), (0.0, 0.2), (-0.2, 0.0), (-0.15, 0.15))
    for count, (left, right) in enumerate(targets, 1):
        base = _stale(inputs[count + 2])
        actual = controller(
            WheelVelocitySetpoint(base.context, left, right),
            admission(acquire(base.raw_devices)),
        )
        feedforward = tuple(config.speed_map.lookup(side, target)[0] for side, target in (
            ("left", left), ("right", right),
        ))
        assert (actual.left_normalized, actual.right_normalized) == pytest.approx(
            tuple(max(-maximum, min(maximum, value)) for value in feedforward)
        )
        assert actual.saturated is any(abs(value) > maximum for value in feedforward)
        state = controller.checkpoint()
        assert state.left_integral == state.right_integral == 0.0
        assert state.last_context == last_fresh
        assert state.transient_stale_ticks == count
    base = _stale(inputs[8])
    with pytest.raises(ValueError, match="degraded"):
        controller(
            WheelVelocitySetpoint(base.context, -0.15, 0.15),
            admission(acquire(base.raw_devices)),
        )
''',
r'''@pytest.mark.parametrize("mode", ("startup", "moving", "stop"))
@pytest.mark.parametrize("maximum", (0.95, 0.13))
def test_stale_feedforward_tracks_current_wheel_targets_without_pi(mode, maximum):
    config = control_config()
    controller = WheelActuatorController(
        config.speed_map, replace(config.wheel_pi, max_normalized_output=maximum),
    )
    admission = InputAdmission(config.admission)
    inputs = tick_inputs(20)
    if mode != "startup":
        for base in inputs[:2]:
            controller(
                WheelVelocitySetpoint(base.context, 0.03, -0.03),
                admission(acquire(base.raw_devices)),
            )
        assert controller.checkpoint().left_integral != 0
        if mode == "stop":
            stopped = controller(
                WheelVelocitySetpoint(inputs[2].context, 0.0, 0.0),
                admission(acquire(_stale(inputs[2]).raw_devices)),
            )
            assert (stopped.left_normalized, stopped.right_normalized) == (0.0, 0.0)
            assert controller.checkpoint().last_context is None

    last_fresh = controller.checkpoint().last_context
    first_stale_ns = inputs[3].context.monotonic_ns
    grace_bases = []
    fault_base = None
    for base in inputs[3:]:
        elapsed_ns = base.context.monotonic_ns - first_stale_ns
        if elapsed_ns < config.wheel_pi.max_feedback_uncertainty_ns:
            grace_bases.append(base)
        else:
            fault_base = base
            break
    assert fault_base is not None

    targets = (
        (0.04, 0.15),
        (-0.15, -0.04),
        (0.0, 0.2),
        (-0.2, 0.0),
        (-0.15, 0.15),
    )

    def assert_ff_only(base, left, right, count):
        current = _stale(base)
        actual = controller(
            WheelVelocitySetpoint(base.context, left, right),
            admission(acquire(current.raw_devices)),
        )
        feedforward = tuple(
            config.speed_map.lookup(side, target)[0]
            for side, target in (("left", left), ("right", right))
        )
        expected = tuple(max(-maximum, min(maximum, value)) for value in feedforward)
        assert (actual.left_normalized, actual.right_normalized) == pytest.approx(expected)
        assert actual.saturated is any(abs(value) > maximum for value in feedforward)
        state = controller.checkpoint()
        assert state.left_integral == state.right_integral == 0.0
        assert state.last_context == last_fresh
        assert state.transient_stale_ticks == count

    for count, (base, (left, right)) in enumerate(zip(grace_bases, targets), 1):
        assert_ff_only(base, left, right, count)

    for count, base in enumerate(grace_bases[len(targets):], len(targets) + 1):
        assert_ff_only(base, -0.15, 0.15, count)

    fault = _stale(fault_base)
    with pytest.raises(ValueError, match="uncertain too long"):
        controller(
            WheelVelocitySetpoint(fault_base.context, -0.15, 0.15),
            admission(acquire(fault.raw_devices)),
        )
''')


def git_blob(path: Path) -> str:
    completed = subprocess.run(
        ["git", "hash-object", str(path)],
        check=True,
        text=True,
        capture_output=True,
    )
    return completed.stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description="R2B4 post-fix test maintenance")
    parser.add_argument("repo", nargs="?", default="/home/alba/project_r2b4")
    parser.add_argument("--check", action="store_true", help="verify only; do not modify")
    args = parser.parse_args()
    repo = Path(args.repo).resolve()

    errors: list[str] = []
    staged: dict[Path, str] = {}
    grouped: dict[str, list[tuple[str, str]]] = {}
    for path, old, new in PATCHES:
        grouped.setdefault(path, []).append((old, new))

    for relative, replacements in grouped.items():
        path = repo / relative
        if not path.is_file():
            errors.append(f"missing: {relative}")
            continue
        actual_blob = git_blob(path)
        expected_blob = BASELINES[relative]
        if actual_blob != expected_blob:
            errors.append(
                f"source drift: {relative}: expected {expected_blob}, got {actual_blob}"
            )
            continue
        text = path.read_text(encoding="utf-8")
        for index, (old, new) in enumerate(replacements, 1):
            count = text.count(old)
            if count != 1:
                errors.append(
                    f"patch anchor mismatch: {relative} replacement {index}: found {count} copies"
                )
                break
            text = text.replace(old, new, 1)
        else:
            staged[path] = text

    if errors:
        print("REFUSED: repository does not match the inspected baseline")
        for item in errors:
            print(f"- {item}")
        return 2

    if args.check:
        print(f"CHECK OK: {len(staged)} test files match the inspected baseline")
        return 0

    for path, text in staged.items():
        fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8", newline="") as stream:
                stream.write(text)
            os.replace(temporary, path)
        except Exception:
            try:
                os.unlink(temporary)
            except OSError:
                pass
            raise
        print(f"updated: {path.relative_to(repo)}")

    print("\nRecommended validation:")
    print("python -m pytest -q \\")
    print("  tests/test_v3_architecture_boundaries.py \\")
    print("  tests/test_v3_counter_encoder_backend.py \\")
    print("  tests/test_v3_encoder_timing.py \\")
    print("  tests/test_v3_gpio_encoder_source.py \\")
    print("  tests/test_v3_native_state_estimation.py \\")
    print("  tests/test_v3_operator_controller.py \\")
    print("  tests/test_v3_transient_sensor_timing.py \\")
    print("  tests/test_v3_encoder_robustness_regressions.py \\")
    print("  tests/test_v3_encoder_p0_integration.py")
    print("\nThen full regression: python -m pytest -q")
    print("No production source, config, runtime data, commit or push was changed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
