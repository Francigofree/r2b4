"""Explicit live encoder acceptance test; importing this file never moves.

Motion, supervision and capture lifecycle use the robot SDK. Only after the
runtime has stopped do we use the canonical offline evidence/replay tools.
"""

import asyncio
import math
import time


PROFILE = ((0.15, 4.0), (0.20, 2.0), (0.15, 4.0))
POLL_S = 0.1
STATUS_MAX_AGE_NS = 500_000_000


async def _read(robot, resource, timeout_s=0.5):
    # A stuck observation must not postpone shutdown for the SDK's 35 s default.
    return await asyncio.wait_for(robot.read(resource), timeout=timeout_s)


def _check_status(status, now_ns, previous=None, mission_id=None):
    from v3.device_health_policy import PRODUCTION_CRITICAL_DEVICE_IDS

    stamp, tick = status.get("monotonic_ns"), status.get("tick_id")
    if (type(stamp) is not int or type(tick) is not int
            or not 0 <= now_ns - stamp < STATUS_MAX_AGE_NS):
        raise RuntimeError("Missing, stale or future runtime status")
    if previous is not None and (tick < previous[0] or stamp < previous[1]):
        raise RuntimeError("Runtime tick/time regressed")
    if status.get("state") != "RUNNING" or status.get("fault_layer") is not None:
        raise RuntimeError(f"Runtime fault at tick {tick}: {status.get('fault_layer')}")
    health = {item.get("device_id"): item.get("state") for item in status.get("source_health", [])}
    if any(health.get(device) != "OK" for device in PRODUCTION_CRITICAL_DEVICE_IDS):
        raise RuntimeError(f"Source health failure at tick {tick}: {health}")
    if mission_id is not None:
        mission = status.get("mission") or {}
        if (mission.get("mission_id") != mission_id
                or mission.get("lifecycle") != "ACTIVE"
                or status.get("safety_decision") != "ALLOW"
                or status.get("enabled") is not True):
            raise RuntimeError(
                f"Motion interrupted at tick {tick}: {status.get('safety_decision')} "
                f"{status.get('safety_reason')}; mission={mission.get('mission_id')}"
            )
        estimate = status.get("estimate") or {}
        values = [estimate.get(key) for key in ("x_m", "y_m", "v_mps", "omega_rad_s")]
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
            raise RuntimeError(f"Invalid motion estimate at tick {tick}")
    return tick, stamp


async def run(robot, *, surface="unspecified", **parameters):
    """Run three forward plateaus once, then stop and evaluate their capture."""
    if parameters or surface not in ("unspecified", "hard", "soft"):
        raise ValueError("Only surface=hard|soft|unspecified is accepted; the motion profile is fixed")
    if (await _read(robot, "operator.status")).get("runtime_running") is not False:
        raise RuntimeError("slow_test requires a stopped runtime; finish the existing session first")

    phases = []
    failure = None
    capture = {}
    runtime_attempted = False
    runtime_stopped = False
    started_ns = time.monotonic_ns()
    try:
        runtime_attempted = True
        runtime = await robot.call("operator.runtime.start", capture_mode="full", capture_hz=50)
        runtime_pid = runtime["pid"]
        capture = await _read(robot, "capture.status", timeout_s=35.0)
        if capture.get("mode") != "full" or capture.get("hz") != 50 or not capture.get("path"):
            raise RuntimeError("FULL 50 Hz capture was not armed")
        initial = await _read(robot, "v3.status")
        previous = _check_status(initial, time.monotonic_ns())
        if initial.get("ready_for_active") is not True:
            raise RuntimeError("Runtime is not ready for motion")

        for speed, duration in PROFILE:
            phase = {"v_mps": speed, "hold_s": duration, "start_ns": time.monotonic_ns(),
                     "observations": 0, "first_tick": None, "last_tick": None,
                     "estimated_v_min_mps": None, "estimated_v_max_mps": None}
            phases.append(phase)
            # TELEOP admits a mission. Its producer owns heartbeat/TTL; it is
            # deliberately called once per phase, never as a velocity setter.
            handle = await robot.call(
                "v3.command.teleop", v_mps=speed, omega_rad_s=0.0, max_v_mps=0.20,
                capture=False, capture_mode="full", capture_hz=50,
                session_watchdog_s=duration + 10.0,
            )
            phase["mission_id"] = "mission-" + handle["command_id"]
            phase["admitted_ns"] = time.monotonic_ns()
            deadline_ns = phase["admitted_ns"] + round(duration * 1e9)
            while True:
                operator = await _read(robot, "operator.status")
                if operator.get("runtime_running") is not True or operator.get("runtime_pid") != runtime_pid:
                    raise RuntimeError("Runtime process/session changed; no restart")
                status = await _read(robot, "v3.status")
                observed_ns = time.monotonic_ns()
                current = _check_status(status, observed_ns, previous, phase["mission_id"])
                if current != previous:
                    phase["observations"] += 1
                    phase["first_tick"] = phase["first_tick"] if phase["first_tick"] is not None else current[0]
                    phase["last_tick"] = current[0]
                    measured = status["estimate"]["v_mps"]
                    for name, reduce in (("min", min), ("max", max)):
                        key = f"estimated_v_{name}_mps"
                        phase[key] = measured if phase[key] is None else reduce(phase[key], measured)
                previous = current
                if observed_ns >= deadline_ns:
                    break
                await asyncio.sleep(min(POLL_S, (deadline_ns - observed_ns) / 1e9))
            phase["end_ns"] = time.monotonic_ns()
            if phase["observations"] < 2:
                raise RuntimeError("Insufficient distinct runtime observations")
    except Exception as exc:
        failure = f"{type(exc).__name__}: {exc}"
    finally:
        if phases and "end_ns" not in phases[-1]:
            phases[-1]["end_ns"] = time.monotonic_ns()
        if runtime_attempted:
            try:
                # This canonical action stops motors AND finalizes FULL capture
                # without revoking the calling skill, unlike robot.stop().
                await robot.call("operator.runtime.stop")
                runtime_stopped = (await _read(robot, "operator.status")).get("runtime_running") is False
                if not runtime_stopped:
                    raise RuntimeError("Runtime shutdown not confirmed")
                final_capture = await _read(robot, "capture.status", timeout_s=35.0)
                if capture.get("path") and final_capture.get("path") != capture["path"]:
                    raise RuntimeError("Capture identity changed")
                capture = final_capture
            except Exception as exc:
                failure = (failure + "; " if failure else "") + f"Shutdown/capture error: {exc}"
                # Last resort canonical preemption; this may revoke the worker.
                if not runtime_stopped:
                    await robot.stop()

    evidence = {"status": "INCONCLUSIVE", "reason": "No finalized complete capture"}
    if runtime_stopped and capture.get("state") == "FINALIZED" and capture.get("complete") is True:
        try:
            evidence = _analyze_capture(capture, phases, robot.root)
        except Exception as exc:
            evidence = {"status": "INCONCLUSIVE", "reason": f"Offline evidence error: {type(exc).__name__}: {exc}"}
    return {
        "status": "FAILED" if failure else evidence["status"], "surface": surface,
        "failure": failure, "runtime_stopped": runtime_stopped,
        "elapsed_s": (time.monotonic_ns() - started_ns) / 1e9,
        "phases": phases, "capture": capture, "evidence": evidence,
        "independent_physical_speed_verified": False,
    }


def _evaluate_ticks(configuration, ticks, phases):
    """Check recorded estimator invariants and sustained per-wheel fallback.

    This consumes evidence only. It never creates control outputs or hardware.
    Replay separately checks the full production decisions against the capture.
    """
    from v3.replay import _production_control_config

    control, _ = _production_control_config({"configuration": configuration}, ())
    runtime = configuration["resolved_robot"]["runtime"]
    backend = runtime["sensor_inputs"]["inputs"]["encoder_backend"]
    pi = control.wheel_pi
    edge_budget_ns = min(pi.max_feedback_age_ns, pi.max_feedback_uncertainty_ns)
    required_span_ns = max(1_000_000_000, 2 * pi.max_feedback_uncertainty_ns)
    metrics = {side: {"edge_fits": 0, "fallback_ticks": 0, "longest_fallback_s": 0.0,
                      "accepted_edge_advances": 0} for side in ("left", "right")}
    streak = {side: None for side in metrics}
    previous_edge = {side: None for side in metrics}
    seen_samples = set()
    coverage = {phase.get("mission_id"): [] for phase in phases if phase.get("mission_id")}
    phase_metrics = {mission: {"settled_target_ticks": 0, "first_counts": None, "last_counts": None,
                              "center_target_min_mps": None, "center_target_max_mps": None}
                     for mission in coverage}
    feedback = None
    first_violation = None

    def fail(tick, side, field, actual, expected):
        nonlocal first_violation
        if first_violation is None:
            first_violation = {"tick_id": tick, "side": side, "field": field,
                               "actual": actual, "expected": expected}

    def equal(tick, side, field, actual, expected):
        if (type(actual) not in (int, float) or not math.isfinite(actual)
                or not math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-10)):
            fail(tick, side, field, actual, expected)

    for row in ticks:
        tick, now_ns = row["tick_id"], row["monotonic_ns"]
        layers = row["expected"]["layers"]
        samples = row["inputs"]["raw_devices"]["samples"]
        for sample in samples:
            if sample["kind"] != "wheel_velocity":
                continue
            identity = (sample["device_id"], sample["sequence"])
            if identity in seen_samples:
                continue
            seen_samples.add(identity)
            values = {item["key"]: item["value"] for item in sample["values"]}
            for side in metrics:
                step = backend[f"{side}_step_distance_m"]
                equal(tick, side, "raw_distance_m", values.get(f"raw_{side}_distance_m"),
                      values[f"raw_{side}_pulse_count"] * step)
                delta = values.get(f"{side}_pulse_delta")
                if delta is not None:
                    equal(tick, side, "distance_delta_m", values.get(f"{side}_distance_delta_m"), delta * step)
                if (values.get(f"{side}_estimation_timebase") != "GPIO_EDGE_HISTORY"
                        or values.get("rejection_code") not in ("NONE", "BASELINE")):
                    continue
                window = values[f"{side}_estimation_window_ns"]
                start = values[f"{side}_estimation_start_edge_timestamp_ns"]
                end = values[f"{side}_estimation_end_edge_timestamp_ns"]
                if type(window) is not int or window <= 0:
                    fail(tick, side, "estimation_window_ns", window, "positive physical interval")
                    continue
                equal(tick, side, "estimation_window_ns", window, end - start)
                candidate = values[f"{side}_estimation_pulse_delta"] * step * 1e9 / window
                equal(tick, side, "computed_mps", values.get(f"computed_{side}_mps"), candidate)
                trust = min(1.0, window / backend["minimum_estimation_window_ns"])
                equal(tick, side, "measurement_trust", values.get(f"{side}_measurement_trust"), trust)
                equal(tick, side, "published_mps", values.get(f"{side}_mps"),
                      candidate if trust >= 1.0 - 1e-12 else 0.0)
                metrics[side]["edge_fits"] += 1

        # L11 can reuse admitted feedback on duplicate-input ticks. Follow the
        # capture's L2 admission indices rather than treating any raw as valid.
        for index in layers.get("L2", {}).get("accepted_sample_indices", []):
            if samples[index]["kind"] == "wheel_velocity":
                feedback = samples[index]
        phase = next((item for item in phases if item.get("admitted_ns", math.inf) <= now_ns
                      <= item["end_ns"]), None)
        if phase is None:
            for side in streak:
                streak[side] = None
            continue
        mission = layers.get("L5", {})
        final = layers.get("L12", {})
        if (row["expected"].get("fault_layer") is not None
                or mission.get("mission_id") != phase["mission_id"]
                or mission.get("lifecycle") != "ACTIVE"
                or final.get("safety_decision") != "ALLOW" or final.get("enabled") is not True):
            fail(tick, None, "motion", final.get("reason"), "same active mission, ALLOW, no fault")
        equal(tick, None, "L5.requested_v_mps", (mission.get("velocity_target") or {}).get("v_mps"),
              phase["v_mps"])
        coverage[phase["mission_id"]].append(now_ns)
        phase_evidence = phase_metrics[phase["mission_id"]]
        targets = [layers.get("L10", {}).get(f"{side}_mps", 0.0) for side in metrics]
        center = sum(targets) / 2.0
        for name, reduce in (("min", min), ("max", max)):
            key = f"center_target_{name}_mps"
            phase_evidence[key] = center if phase_evidence[key] is None else reduce(phase_evidence[key], center)
        reference = layers.get("L8", {}).get("requested_v_mps", 0.0)
        # Canonical heading tracking may change both omega and v*cos(error).
        # Settling means realizing that reference, not equal wheel speeds.
        if (reference > 0.0 and math.isclose(center, reference, rel_tol=0.0, abs_tol=1e-6)
                and min(targets) >= control.speed_map.minimum_continuous_speed_mps - 1e-6
                and layers.get("L10", {}).get("motion_revoked") is False):
            phase_evidence["settled_target_ticks"] += 1
        values = {} if feedback is None else {item["key"]: item["value"] for item in feedback["values"]}
        clean = (values.get("measurement_stale") is False
                 and values.get("measurement_timing_valid") is True
                 and values.get("rejection_code") in ("NONE", "BASELINE")
                 and all(values.get(f"{side}_counter_running") is True
                         and values.get(f"{side}_read_error_delta") in (0, None)
                         and values.get(f"{side}_invalid_alert_delta") in (0, None) for side in metrics))
        if clean:
            counts = {side: values[f"raw_{side}_pulse_count"] for side in metrics}
            if phase_evidence["first_counts"] is None:
                phase_evidence["first_counts"] = counts
            phase_evidence["last_counts"] = counts
        for side in metrics:
            target = layers.get("L10", {}).get(f"{side}_mps", 0.0)
            edge_ns = values.get(f"{side}_last_accepted_edge_timestamp_ns")
            direction = values.get(f"{side}_last_accepted_edge_direction", 0)
            measured = values.get(f"{side}_mps", 0.0)
            weak = (values.get(f"{side}_measurement_trust", 0.0) < 1.0 - 1e-12
                    or values.get(f"{side}_estimation_timebase") != "GPIO_EDGE_HISTORY"
                    or abs(measured) < pi.minimum_reliable_speed_mps)
            live = (clean and feedback is not None
                    and 0 <= now_ns - feedback["captured_monotonic_ns"] <= pi.max_feedback_age_ns
                    and type(edge_ns) is int and phase["start_ns"] <= edge_ns <= now_ns
                    and now_ns - edge_ns < edge_budget_ns
                    and type(direction) is int and direction * target > 0
                    and direction == values.get(f"{side}_confirmed_direction"))
            if (not weak or not live or abs(target) < 1e-9
                    or final.get("safety_decision") != "ALLOW"):
                streak[side] = None
                continue
            expected, _ = control.speed_map.lookup(side, target)
            expected = max(-pi.max_normalized_output, min(pi.max_normalized_output, expected))
            output = layers.get("L11", {}).get(f"{side}_normalized")
            equal(tick, side, "L11.feedforward", output, expected)
            equal(tick, side, "L12.output", final.get(f"{side}_output"), expected)
            metrics[side]["fallback_ticks"] += 1
            if previous_edge[side] is not None and edge_ns < previous_edge[side]:
                fail(tick, side, "accepted_edge_timestamp_ns", edge_ns, ">= previous accepted edge")
            if previous_edge[side] is None or edge_ns > previous_edge[side]:
                metrics[side]["accepted_edge_advances"] += 1
                previous_edge[side] = edge_ns
            if streak[side] is None:
                streak[side] = now_ns
            span_s = (now_ns - streak[side]) / 1e9
            metrics[side]["longest_fallback_s"] = max(metrics[side]["longest_fallback_s"], span_s)

    reasons = []
    if len(phases) != len(PROFILE):
        reasons.append("Motion profile did not finish")
    for phase in phases:
        stamps = coverage.get(phase.get("mission_id"), [])
        if not stamps or (stamps[-1] - stamps[0]) / 1e9 < phase["hold_s"] - 0.15:
            reasons.append(f"Insufficient held-motion capture coverage: {phase.get('mission_id')}")
        phase_evidence = phase_metrics.get(phase.get("mission_id"))
        if phase_evidence is None:
            continue
        if phase_evidence["settled_target_ticks"] * runtime["tick_period_ns"] < 500_000_000:
            reasons.append(f"Canonical motion reference not realized for 0.5 s: {phase['mission_id']}")
        first, last = phase_evidence.pop("first_counts"), phase_evidence.pop("last_counts")
        phase_evidence["signed_distance_m"] = {
            side: None if first is None or last is None else
            (last[side] - first[side]) * backend[f"{side}_step_distance_m"] for side in metrics}
        if any(distance is None or distance <= 0 for distance in phase_evidence["signed_distance_m"].values()):
            reasons.append(f"Positive per-wheel displacement not demonstrated: {phase['mission_id']}")
    for side, evidence in metrics.items():
        if not evidence["edge_fits"]:
            reasons.append(f"No GPIO interval estimate: {side}")
        if (evidence["longest_fallback_s"] * 1e9 < required_span_ns
                or evidence["accepted_edge_advances"] < 2):
            reasons.append(f"Sustained weak-fit fallback not exercised: {side}")
    return {"status": "FAILED" if first_violation else "INCONCLUSIVE" if reasons else "PASS",
            "first_violation": first_violation, "reasons": reasons, "wheels": metrics,
            "required_fallback_s": required_span_ns / 1e9, "phases": phase_metrics}


def _analyze_capture(capture, phases, project_root):
    """Explicit offline operation, invoked only after confirmed shutdown."""
    import json
    from pathlib import Path
    import tempfile

    from tools.mcap_evidence.compiler import compile_evidence
    from tools.mcap_evidence.query import query
    from tools.mcap_evidence.verify import verify
    from v3.capture_compaction import expand_tick_row
    from v3.mcap_replay_bridge import ReplayWindow, replay_mcap
    from v3.replay import write_replay_result

    required = ("raw_evidence_requested", "raw_evidence_complete", "sample_complete", "replay_core_complete")
    if any(capture.get(key) is not True for key in required) or not phases:
        return {"status": "INCONCLUSIVE", "reason": "Full raw/replay evidence or motion phases missing"}
    output = Path(tempfile.mkdtemp(prefix="r2b4-slow-test-", dir="/tmp"))
    bundle = output / "capture.evidence"
    compiled = compile_evidence(capture["path"], bundle, workers=1)
    verified = verify(bundle, source=capture["path"])
    if compiled["compiler_status"] != "COMPLETE" or verified["status"] != "PASS":
        return {"status": "INCONCLUSIVE", "reason": "Evidence compilation/verification incomplete",
                "artifact_dir": str(output)}
    runtime = next(query(bundle, topic="/r2b4/runtime", limit=1))["payload"]
    start_ns, end_ns = phases[0]["start_ns"], phases[-1]["end_ns"]
    rows = list(query(bundle, topic="/r2b4/tick", start_ns=start_ns, end_ns=end_ns, limit=2049))
    if not rows or len(rows) > 2048:
        return {"status": "INCONCLUSIVE", "reason": "Missing ticks or bounded analysis limit exceeded",
                "artifact_dir": str(output)}
    ticks = [expand_tick_row(row["payload"]) for row in rows]
    evidence = _evaluate_ticks(runtime["configuration"], ticks, phases)
    replay = replay_mcap(capture["path"], project_root=project_root,
                         window=ReplayWindow(requested_start_ns=start_ns, requested_end_ns=end_ns))
    write_replay_result(replay, output / "replay.json")
    evidence.update(artifact_dir=str(output), replay_status=replay["status"],
                    capture_path=capture["path"], analyzed_ticks=len(ticks))
    if replay["status"] != "MATCH":
        evidence["status"] = "FAILED"
        evidence["reasons"].append("Canonical replay did not MATCH")
    (output / "result.json").write_text(json.dumps(evidence, indent=2, allow_nan=False) + "\n")
    return evidence
