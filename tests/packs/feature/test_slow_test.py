"""Offline live-skill supervision evidence; these tests never use hardware."""

import asyncio
import json
from types import SimpleNamespace

import pytest

from robot_skills import slow_test


class FakeClock:
    def __init__(self):
        self.now_ns = 1_000_000_000

    def monotonic_ns(self):
        return self.now_ns

    async def sleep(self, delay):
        assert 0 < delay <= slow_test.POLL_S
        self.now_ns += round(delay * 1e9)


class FakeRobot:
    """Model SDK lifecycle, fresh status and command-specific mission lineage."""

    def __init__(self, clock, root, *, existing_runtime=False, fault=None):
        self.clock = clock
        self.root = root
        self.runtime_running = existing_runtime
        self.pid = 101
        self.tick = 0
        self.speed = 0.0
        self.command_id = None
        self.calls = []
        self.reads = []
        self.fault = fault
        self.fault_seen = False
        self.capture_complete = True
        self.capture_state = "FINALIZED"
        self.capture_identity_changed = False
        self.capture_mode = "full"
        self.ready = True
        self.shutdown_failure = None
        self.shutdown_unconfirmed = False
        self.emergency_stops = 0

    async def read(self, resource):
        self.reads.append(resource)
        if resource == "operator.status":
            result = {"runtime_running": self.runtime_running,
                      "runtime_pid": self.pid if self.runtime_running else None}
            if self.command_id and self.fault == "pid_changed" and not self.fault_seen:
                self.fault_seen = True
                result["runtime_pid"] += 1
            return result
        if resource == "capture.status":
            return {"mode": self.capture_mode, "hz": 50,
                    "path": str(self.root / ("changed.mcap" if not self.runtime_running
                                             and self.capture_identity_changed else "test.mcap")),
                    "state": "RECORDING" if self.runtime_running else self.capture_state,
                    "complete": False if self.runtime_running else self.capture_complete}
        assert resource == "v3.status"
        self.tick += 1
        result = {
            "monotonic_ns": self.clock.now_ns, "tick_id": self.tick,
            "state": "RUNNING", "fault_layer": None,
            "ready_for_active": self.ready,
            "source_health": [{"device_id": name, "state": "OK"} for name in
                              ("WHEEL_ENCODERS", "BNO055_IMU", "RPLIDAR_C1")]
                             + [{"device_id": "PERSON_DETECTION", "state": "UNKNOWN"}],
            "mission": {"mission_id": "mission-" + self.command_id,
                        "lifecycle": "ACTIVE"} if self.command_id else None,
            "safety_decision": "ALLOW" if self.command_id else "STOP",
            "safety_reason": None,
            "enabled": self.command_id is not None,
            "estimate": {"x_m": 0.0, "y_m": 0.0,
                         "v_mps": self.speed, "omega_rad_s": 0.0},
        }
        if self.command_id and self.fault and not self.fault_seen:
            self.fault_seen = True
            if self.fault == "safety":
                result.update(safety_decision="STOP", safety_reason="CLEARANCE")
            elif self.fault == "source":
                result["source_health"][0]["state"] = "FAULT"
            elif self.fault == "missing_source":
                result["source_health"] = result["source_health"][1:]
            elif self.fault == "stale":
                result["monotonic_ns"] -= slow_test.STATUS_MAX_AGE_NS
            elif self.fault == "runtime":
                result.update(state="FAULT", fault_layer="L11")
            elif self.fault == "wrong_mission":
                result["mission"]["mission_id"] = "mission-unrelated"
            elif self.fault == "read_error":
                raise RuntimeError("observation unavailable")
            elif self.fault == "cancelled":
                raise asyncio.CancelledError
            else:
                raise AssertionError(f"Unimplemented fake failure: {self.fault}")
        return result

    async def call(self, action, **parameters):
        self.calls.append((action, parameters))
        if action == "operator.runtime.start":
            assert self.runtime_running is False
            self.runtime_running = True
            return {"pid": self.pid}
        if action == "v3.command.teleop":
            assert self.runtime_running is True
            self.speed = parameters["v_mps"]
            self.command_id = f"command-{len(self.teleops)}"
            return {"command_id": self.command_id, "pid": 202, "capture_mode": "full"}
        assert action == "operator.runtime.stop"
        if self.shutdown_failure:
            raise self.shutdown_failure
        if not self.shutdown_unconfirmed:
            self.runtime_running = False
        return {"status": "STOPPED"}

    async def stop(self):
        self.emergency_stops += 1

    @property
    def teleops(self):
        return [parameters for action, parameters in self.calls
                if action == "v3.command.teleop"]


@pytest.fixture
def rig(monkeypatch, tmp_path):
    clock = FakeClock()
    monkeypatch.setattr(slow_test.time, "monotonic_ns", clock.monotonic_ns)
    monkeypatch.setattr(slow_test.asyncio, "sleep", clock.sleep)
    analyzed = []

    def analyze(capture, phases, root):
        analyzed.append((capture, phases, root))
        assert capture["state"] == "FINALIZED" and capture["complete"] is True
        return {"status": "PASS", "reason": "fake complete encoder evidence"}

    monkeypatch.setattr(slow_test, "_analyze_capture", analyze, raising=False)
    return FakeRobot(clock, tmp_path), analyzed


def test_three_feasible_plateaus_use_full_capture_and_owned_shutdown(rig):
    robot, analyzed = rig
    result = asyncio.run(slow_test.run(robot, surface="hard"))

    assert result["status"] == "PASS"
    assert result["failure"] is None
    assert result["surface"] == "hard"
    assert result["elapsed_s"] == pytest.approx(10.0)
    assert result["runtime_stopped"] is True
    assert result["independent_physical_speed_verified"] is False
    assert robot.calls[0] == ("operator.runtime.start", {"capture_mode": "full", "capture_hz": 50})
    assert [call["v_mps"] for call in robot.teleops] == [0.15, 0.20, 0.15]
    for command, phase in zip(robot.teleops, result["phases"]):
        assert 0.15 <= command["v_mps"] <= command["max_v_mps"] <= 0.20
        assert command["omega_rad_s"] == 0.0
        assert command["capture"] is False
        assert command["capture_mode"] == "full" and command["capture_hz"] == 50
        assert command["session_watchdog_s"] > phase["hold_s"]
        assert phase["observations"] >= 2
        assert phase["first_tick"] < phase["last_tick"]
        assert phase["end_ns"] - phase["admitted_ns"] == round(phase["hold_s"] * 1e9)
    assert robot.calls[-1] == ("operator.runtime.stop", {})
    assert robot.emergency_stops == 0
    assert len(analyzed) == 1
    assert analyzed[0][2] == robot.root


def test_existing_runtime_rejected_before_mutating_it(rig):
    robot, analyzed = rig
    robot.runtime_running = True
    with pytest.raises(RuntimeError, match="requires a stopped runtime"):
        asyncio.run(slow_test.run(robot))
    assert robot.reads == ["operator.status"]
    assert robot.calls == [] and robot.emergency_stops == 0
    assert analyzed == []


@pytest.mark.parametrize("parameters", [{"surface": "unknown"}, {"duration_s": 3.0}])
def test_invalid_parameters_do_not_start_hardware(rig, parameters):
    robot, analyzed = rig
    with pytest.raises(ValueError):
        asyncio.run(slow_test.run(robot, **parameters))
    assert robot.reads == [] and robot.calls == [] and analyzed == []


@pytest.mark.parametrize("fault", ["safety", "source", "missing_source", "stale", "pid_changed",
                                  "runtime", "wrong_mission", "read_error"])
def test_first_motion_failure_stops_once_without_next_admission(rig, fault):
    robot, _ = rig
    robot.fault = fault
    result = asyncio.run(slow_test.run(robot))
    assert robot.fault_seen is True
    assert result["status"] == "FAILED" and result["failure"]
    assert len(robot.teleops) == 1
    assert [action for action, _ in robot.calls].count("operator.runtime.stop") == 1
    assert robot.runtime_running is False
    assert robot.emergency_stops == 0
    assert result["phases"][0]["end_ns"] >= result["phases"][0]["start_ns"]


@pytest.mark.parametrize("failure", ["capture_mode", "not_ready"])
def test_preflight_failure_shuts_down_without_motion(rig, failure):
    robot, _ = rig
    if failure == "capture_mode":
        robot.capture_mode = "alap"
    else:
        robot.ready = False
    result = asyncio.run(slow_test.run(robot))
    assert result["status"] == "FAILED" and result["failure"]
    assert robot.teleops == []
    assert [action for action, _ in robot.calls].count("operator.runtime.stop") == 1
    assert robot.runtime_running is False
    assert robot.emergency_stops == 0


@pytest.mark.parametrize("incomplete", ["state", "complete"])
def test_incomplete_capture_never_passes_or_invokes_analyzer(rig, incomplete):
    robot, analyzed = rig
    if incomplete == "state":
        robot.capture_state = "INCOMPLETE"
    else:
        robot.capture_complete = False
    result = asyncio.run(slow_test.run(robot))
    assert result["status"] == "INCONCLUSIVE"
    assert result["evidence"]["status"] == "INCONCLUSIVE"
    assert result["runtime_stopped"] is True
    assert analyzed == [] and robot.emergency_stops == 0


def test_offline_analyzer_error_never_passes(rig, monkeypatch):
    robot, _ = rig

    def fail(*_):
        raise RuntimeError("Replay input missing")

    monkeypatch.setattr(slow_test, "_analyze_capture", fail)
    result = asyncio.run(slow_test.run(robot))
    assert result["status"] == "INCONCLUSIVE"
    assert "Replay input missing" in result["evidence"]["reason"]
    assert result["runtime_stopped"] is True
    assert robot.emergency_stops == 0


def test_complete_capture_can_still_be_inconclusive(rig, monkeypatch):
    robot, _ = rig
    monkeypatch.setattr(slow_test, "_analyze_capture", lambda *_:
                        {"status": "INCONCLUSIVE", "reason": "No low-confidence wheel coverage"})
    result = asyncio.run(slow_test.run(robot))
    assert result["status"] == "INCONCLUSIVE"
    assert result["failure"] is None


@pytest.mark.parametrize("failure", ["exception", "unconfirmed"])
def test_shutdown_problem_uses_canonical_emergency_preemption(rig, failure):
    robot, _ = rig
    if failure == "exception":
        robot.shutdown_failure = RuntimeError("Shutdown unavailable")
    elif failure == "unconfirmed":
        robot.shutdown_unconfirmed = True
    result = asyncio.run(slow_test.run(robot))
    assert result["status"] == "FAILED"
    assert "Shutdown/capture error" in result["failure"]
    assert [action for action, _ in robot.calls].count("operator.runtime.stop") == 1
    assert robot.emergency_stops == 1


def test_capture_identity_problem_after_confirmed_shutdown_does_not_preempt_worker(rig):
    robot, analyzed = rig
    robot.capture_identity_changed = True
    result = asyncio.run(slow_test.run(robot))
    assert result["status"] == "FAILED"
    assert "Capture identity changed" in result["failure"]
    assert result["runtime_stopped"] is True
    assert robot.emergency_stops == 0
    assert analyzed == []


def test_cancellation_still_shuts_down_without_readmitting_motion(rig):
    robot, _ = rig
    robot.fault = "cancelled"
    with pytest.raises(asyncio.CancelledError):
        asyncio.run(slow_test.run(robot))
    assert len(robot.teleops) == 1
    assert [action for action, _ in robot.calls].count("operator.runtime.stop") == 1
    assert robot.runtime_running is False
    assert robot.emergency_stops == 0


class FakeSpeedMap:
    minimum_continuous_speed_mps = 0.15

    def lookup(self, side, target):
        return target * (2.0 if side == "left" else 2.1), 0.05


@pytest.fixture
def encoder_configuration(monkeypatch):
    """Isolate evidence arithmetic from production configuration resolution."""
    import v3.replay

    pi = SimpleNamespace(max_feedback_age_ns=250_000_000,
                         max_feedback_uncertainty_ns=250_000_000,
                         minimum_reliable_speed_mps=0.15,
                         max_normalized_output=1.0)
    control = SimpleNamespace(wheel_pi=pi, speed_map=FakeSpeedMap())
    backend = {"left_step_distance_m": 0.001, "right_step_distance_m": 0.001,
               "minimum_estimation_window_ns": 40_000_000}
    configuration = {"resolved_robot": {"runtime": {"tick_period_ns": 20_000_000,
        "sensor_inputs": {"inputs": {
        "encoder_backend": backend}}}}}

    def resolve(runtime, _inputs):
        assert runtime["configuration"] is configuration
        return control, None

    monkeypatch.setattr(v3.replay, "_production_control_config", resolve)
    return configuration, control


def recorded_profile(control, *, weak_sides=("left", "right"), partial_trust=False):
    """Three held missions; each 20 ms record contains fresh accepted edges.

    A 40 ms physical interval with four millimetre steps estimates 0.10 m/s.
    Partial trust uses a 20 ms interval and two steps: the candidate is still
    0.10 m/s, while published velocity must be zero and FF remains usable.
    """
    phases, ticks = [], []
    cursor_ns = 2_000_000_000
    for speed, duration in slow_test.PROFILE:
        phase = {"v_mps": speed, "hold_s": duration,
                 "mission_id": f"mission-{len(phases)}",
                 "start_ns": cursor_ns - 200_000_000, "admitted_ns": cursor_ns,
                 "end_ns": cursor_ns + round(duration * 1e9)}
        phases.append(phase)
        for offset_ns in range(0, round(duration * 1e9) + 1, 20_000_000):
            now_ns = cursor_ns + offset_ns
            tick = len(ticks) + 1
            values = {"measurement_stale": False, "measurement_timing_valid": True,
                      "rejection_code": "NONE"}
            layers = {"L2": {"accepted_sample_indices": [0]},
                      "L5": {"mission_id": phase["mission_id"], "lifecycle": "ACTIVE",
                             "velocity_target": {"v_mps": speed}},
                      "L8": {"requested_v_mps": speed},
                      "L10": {"left_mps": speed, "right_mps": speed,
                              "motion_revoked": False},
                      "L11": {}, "L12": {"safety_decision": "ALLOW", "enabled": True}}
            for side in ("left", "right"):
                weak = side in weak_sides
                window_ns = 20_000_000 if partial_trust and weak else 40_000_000
                pulse_delta = 2 if window_ns == 20_000_000 else 4 if weak else 8
                candidate = pulse_delta * 0.001 * 1e9 / window_ns
                trust = min(1.0, window_ns / 40_000_000)
                edge_ns = now_ns - 1_000_000
                raw_count = 100 + tick * (2 if weak else 4)
                values.update({
                    f"raw_{side}_pulse_count": raw_count,
                    f"raw_{side}_distance_m": raw_count * 0.001,
                    f"{side}_pulse_delta": 2 if weak else 4,
                    f"{side}_distance_delta_m": 0.002 if weak else 0.004,
                    f"{side}_estimation_timebase": "GPIO_EDGE_HISTORY",
                    f"{side}_estimation_window_ns": window_ns,
                    f"{side}_estimation_start_edge_timestamp_ns": edge_ns - window_ns,
                    f"{side}_estimation_end_edge_timestamp_ns": edge_ns,
                    f"{side}_estimation_pulse_delta": pulse_delta,
                    f"computed_{side}_mps": candidate,
                    f"{side}_measurement_trust": trust,
                    f"{side}_mps": candidate if trust == 1.0 else 0.0,
                    f"{side}_counter_running": True,
                    f"{side}_read_error_delta": 0,
                    f"{side}_invalid_alert_delta": 0,
                    f"{side}_last_accepted_edge_timestamp_ns": edge_ns,
                    f"{side}_last_accepted_edge_direction": 1,
                    f"{side}_confirmed_direction": 1,
                })
                output, _ = control.speed_map.lookup(side, speed)
                layers["L11"][f"{side}_normalized"] = output
                layers["L12"][f"{side}_output"] = output
            sample = {"device_id": "WHEEL_ENCODERS", "kind": "wheel_velocity",
                      "sequence": tick, "captured_monotonic_ns": now_ns,
                      "values": [{"key": key, "value": value} for key, value in values.items()]}
            ticks.append({"tick_id": tick, "monotonic_ns": now_ns,
                          "inputs": {"raw_devices": {"samples": [sample]}},
                          "expected": {"fault_layer": None, "layers": layers}})
        cursor_ns = phase["end_ns"] + 200_000_000
    return phases, ticks


def replace_encoder_value(row, key, value):
    values = row["inputs"]["raw_devices"]["samples"][0]["values"]
    next(item for item in values if item["key"] == key)["value"] = value


@pytest.mark.parametrize("partial_trust", [False, True])
def test_recorded_both_wheel_sustained_weak_fit_with_exact_ff_passes(
        encoder_configuration, partial_trust):
    configuration, control = encoder_configuration
    phases, ticks = recorded_profile(control, partial_trust=partial_trust)
    result = slow_test._evaluate_ticks(configuration, ticks, phases)
    assert result["status"] == "PASS" and result["first_violation"] is None
    assert result["reasons"] == []
    for side in ("left", "right"):
        wheel = result["wheels"][side]
        assert wheel["edge_fits"] == len(ticks)
        assert wheel["accepted_edge_advances"] > 1
        assert wheel["longest_fallback_s"] >= result["required_fallback_s"]


@pytest.mark.parametrize(("key", "actual", "field"), [
    ("computed_left_mps", 0.25, "computed_mps"),
    ("left_measurement_trust", 0.5, "measurement_trust"),
    ("raw_left_distance_m", 100.0, "raw_distance_m"),
    ("left_distance_delta_m", 0.02, "distance_delta_m"),
])
def test_encoder_interval_and_unchanged_count_evidence_reject_corruption(
        encoder_configuration, key, actual, field):
    configuration, control = encoder_configuration
    phases, ticks = recorded_profile(control)
    # 0.25 m/s represents an inflated burst-weighted estimate, while these
    # four physical steps across 40 ms can only prove 0.10 m/s.
    replace_encoder_value(ticks[0], key, actual)
    result = slow_test._evaluate_ticks(configuration, ticks, phases)
    violation = result["first_violation"]
    assert result["status"] == "FAILED"
    assert violation["tick_id"] == ticks[0]["tick_id"]
    assert violation["side"] == "left" and violation["field"] == field
    assert violation["actual"] == actual
    if field == "computed_mps":
        assert violation["expected"] == pytest.approx(0.10)


def test_short_window_cannot_publish_untrusted_candidate(encoder_configuration):
    configuration, control = encoder_configuration
    phases, ticks = recorded_profile(control, partial_trust=True)
    replace_encoder_value(ticks[0], "left_mps", 0.1)
    result = slow_test._evaluate_ticks(configuration, ticks, phases)
    assert result["status"] == "FAILED"
    assert result["first_violation"]["field"] == "published_mps"
    assert result["first_violation"]["expected"] == 0.0


def test_heading_corrected_unequal_wheels_preserve_realized_body_reference(
        encoder_configuration):
    configuration, control = encoder_configuration
    phases, ticks = recorded_profile(control)
    corrected = [row for row in ticks if row["expected"]["layers"]["L5"]["velocity_target"]["v_mps"] == 0.20]
    for row in corrected:
        layers = row["expected"]["layers"]
        # Canonical heading correction changes body v by cos(error) and adds
        # opposite wheel deltas; neither wheel equals the 0.20 reference.
        realized_body_v = 0.19999
        layers["L8"]["requested_v_mps"] = realized_body_v
        for side, target in (("left", realized_body_v + 0.003),
                             ("right", realized_body_v - 0.003)):
            layers["L10"][f"{side}_mps"] = target
            output, _ = control.speed_map.lookup(side, target)
            layers["L11"][f"{side}_normalized"] = output
            layers["L12"][f"{side}_output"] = output
    result = slow_test._evaluate_ticks(configuration, ticks, phases)
    assert result["status"] == "PASS" and result["first_violation"] is None
    assert result["phases"][phases[1]["mission_id"]]["settled_target_ticks"] == len(corrected)


@pytest.mark.parametrize("failure", ["zero_wheels", "zero_body", "revoked"])
def test_positive_mission_without_realized_reference_plateau_is_inconclusive(
        encoder_configuration, failure):
    configuration, control = encoder_configuration
    phases, ticks = recorded_profile(control)
    for row in ticks:
        layers = row["expected"]["layers"]
        if layers["L5"]["mission_id"] != phases[1]["mission_id"]:
            continue
        if failure == "zero_wheels":
            for side in ("left", "right"):
                layers["L10"][f"{side}_mps"] = 0.0
                layers["L11"][f"{side}_normalized"] = 0.0
                layers["L12"][f"{side}_output"] = 0.0
        elif failure == "zero_body":
            layers["L8"]["requested_v_mps"] = 0.0
        else:
            layers["L10"]["motion_revoked"] = True
    result = slow_test._evaluate_ticks(configuration, ticks, phases)
    assert result["status"] == "INCONCLUSIVE"
    assert result["first_violation"] is None
    assert result["phases"][phases[1]["mission_id"]]["settled_target_ticks"] == 0
    assert any(phases[1]["mission_id"] in reason and "0.5 s" in reason
               for reason in result["reasons"])


def test_wrong_l5_mission_reference_is_rejected_even_with_realized_motion(
        encoder_configuration):
    configuration, control = encoder_configuration
    phases, ticks = recorded_profile(control)
    changed = next(row for row in ticks if row["expected"]["layers"]["L5"]["mission_id"]
                   == phases[1]["mission_id"])
    changed["expected"]["layers"]["L5"]["velocity_target"]["v_mps"] = 0.15
    result = slow_test._evaluate_ticks(configuration, ticks, phases)
    assert result["status"] == "FAILED"
    assert result["first_violation"]["tick_id"] == changed["tick_id"]
    assert result["first_violation"]["field"] == "L5.requested_v_mps"
    assert result["first_violation"]["expected"] == 0.20


@pytest.mark.parametrize("weak_sides", [(), ("left",), ("right",)])
def test_unexercised_or_one_wheel_only_fallback_is_inconclusive(
        encoder_configuration, weak_sides):
    configuration, control = encoder_configuration
    phases, ticks = recorded_profile(control, weak_sides=weak_sides)
    result = slow_test._evaluate_ticks(configuration, ticks, phases)
    assert result["status"] == "INCONCLUSIVE"
    assert result["first_violation"] is None
    for side in {"left", "right"} - set(weak_sides):
        assert f"Sustained weak-fit fallback not exercised: {side}" in result["reasons"]


def test_first_faulty_motor_tick_is_reported(encoder_configuration):
    configuration, control = encoder_configuration
    phases, ticks = recorded_profile(control)
    bad = ticks[7]
    bad["expected"]["layers"]["L11"]["right_normalized"] += 0.02
    ticks[12]["expected"]["layers"]["L12"]["left_output"] += 0.03
    result = slow_test._evaluate_ticks(configuration, ticks, phases)
    assert result["status"] == "FAILED"
    assert result["first_violation"]["tick_id"] == bad["tick_id"]
    assert result["first_violation"]["side"] == "right"
    assert result["first_violation"]["field"] == "L11.feedforward"


def test_capture_catches_stop_between_live_poll_observations(encoder_configuration):
    configuration, control = encoder_configuration
    phases, ticks = recorded_profile(control)
    bad = ticks[1]  # 20 ms, between the skill's 100 ms observations.
    bad["expected"]["layers"]["L12"].update(safety_decision="STOP", enabled=False,
                                            reason="TRANSIENT_CLEARANCE")
    result = slow_test._evaluate_ticks(configuration, ticks, phases)
    assert result["status"] == "FAILED"
    assert result["first_violation"]["tick_id"] == bad["tick_id"]
    assert result["first_violation"]["field"] == "motion"
    assert result["first_violation"]["actual"] == "TRANSIENT_CLEARANCE"


@pytest.fixture
def offline_tools(monkeypatch, tmp_path):
    import tempfile
    import tools.mcap_evidence.compiler as compiler
    import tools.mcap_evidence.query as query_module
    import tools.mcap_evidence.verify as verify_module
    import v3.capture_compaction as compaction
    import v3.mcap_replay_bridge as bridge
    import v3.replay as replay_module

    output = tmp_path / "offline-output"
    output.mkdir()
    monkeypatch.setattr(tempfile, "mkdtemp", lambda **_: str(output))
    state = SimpleNamespace(calls=[], compiler_status="COMPLETE", verify_status="PASS",
                            replay_status="MATCH", row_count=2, output=output)
    configuration = {"captured": "configuration"}

    def compile_capture(source, bundle, *, workers):
        state.calls.append(("compile", source, bundle, workers))
        return {"compiler_status": state.compiler_status}

    def verify_capture(bundle, *, source):
        state.calls.append(("verify", bundle, source))
        return {"status": state.verify_status}

    def query_capture(bundle, **parameters):
        state.calls.append(("query", bundle, parameters))
        if parameters["topic"] == "/r2b4/runtime":
            return iter([{"payload": {"configuration": configuration}}])
        assert parameters["topic"] == "/r2b4/tick"
        return iter({"payload": {"compact_tick": index}} for index in range(state.row_count))

    def expand(payload):
        state.calls.append(("expand", payload))
        return {"expanded_tick": payload["compact_tick"]}

    def evaluate(config, ticks, phases):
        state.calls.append(("evaluate", config, ticks, phases))
        assert config is configuration
        return {"status": "PASS", "first_violation": None, "reasons": [], "wheels": {}}

    def replay_capture(source, *, project_root, window):
        state.calls.append(("replay", source, project_root, window))
        return {"status": state.replay_status}

    def write_replay(result, path):
        state.calls.append(("write_replay", result, path))
        path.write_text(json.dumps(result))

    monkeypatch.setattr(compiler, "compile_evidence", compile_capture)
    monkeypatch.setattr(verify_module, "verify", verify_capture)
    monkeypatch.setattr(query_module, "query", query_capture)
    monkeypatch.setattr(compaction, "expand_tick_row", expand)
    monkeypatch.setattr(slow_test, "_evaluate_ticks", evaluate)
    monkeypatch.setattr(bridge, "replay_mcap", replay_capture)
    monkeypatch.setattr(replay_module, "write_replay_result", write_replay)
    state.capture = {"path": str(tmp_path / "recording.mcap"),
                     "raw_evidence_requested": True, "raw_evidence_complete": True,
                     "sample_complete": True, "replay_core_complete": True}
    state.phases = [{"start_ns": 1_000_000_000, "end_ns": 5_000_000_000}]
    return state


@pytest.mark.parametrize("replay_status", ["MATCH", "MISMATCH"])
def test_offline_compiler_query_expansion_and_native_replay_are_required(
        offline_tools, replay_status, tmp_path):
    state = offline_tools
    state.replay_status = replay_status
    result = slow_test._analyze_capture(state.capture, state.phases, tmp_path)
    assert [call[0] for call in state.calls] == ["compile", "verify", "query", "query",
                                               "expand", "expand", "evaluate", "replay",
                                               "write_replay"]
    compile_call = state.calls[0]
    assert compile_call[1] == state.capture["path"] and compile_call[3] == 1
    ticks_query = state.calls[3][2]
    assert ticks_query["start_ns"] == state.phases[0]["start_ns"]
    assert ticks_query["end_ns"] == state.phases[-1]["end_ns"]
    assert state.calls[6][2] == [{"expanded_tick": 0}, {"expanded_tick": 1}]
    replay_call = state.calls[7]
    assert replay_call[2] == tmp_path
    assert replay_call[3].requested_start_ns == state.phases[0]["start_ns"]
    assert replay_call[3].requested_end_ns == state.phases[-1]["end_ns"]
    assert result["status"] == ("PASS" if replay_status == "MATCH" else "FAILED")
    assert result["replay_status"] == replay_status and result["analyzed_ticks"] == 2
    assert json.loads((state.output / "result.json").read_text()) == result
    assert json.loads((state.output / "replay.json").read_text())["status"] == replay_status


@pytest.mark.parametrize("missing_flag", ["raw_evidence_requested", "raw_evidence_complete",
                                        "sample_complete", "replay_core_complete"])
def test_missing_raw_or_replay_capture_evidence_does_not_compile_or_pass(
        offline_tools, missing_flag, tmp_path):
    state = offline_tools
    state.capture[missing_flag] = False
    result = slow_test._analyze_capture(state.capture, state.phases, tmp_path)
    assert result["status"] == "INCONCLUSIVE" and state.calls == []


@pytest.mark.parametrize("failure", ["compiler", "verification", "no_ticks", "too_many_ticks"])
def test_incomplete_or_unbounded_offline_evidence_never_replays_or_passes(
        offline_tools, failure, tmp_path):
    state = offline_tools
    if failure == "compiler":
        state.compiler_status = "INCOMPLETE"
    elif failure == "verification":
        state.verify_status = "FAILED"
    elif failure == "no_ticks":
        state.row_count = 0
    else:
        state.row_count = 2049
    result = slow_test._analyze_capture(state.capture, state.phases, tmp_path)
    assert result["status"] == "INCONCLUSIVE"
    assert not any(call[0] in {"evaluate", "replay", "write_replay"} for call in state.calls)
