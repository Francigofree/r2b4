"""Measured Speed Map generation from canonical, complete 50 Hz MCAP.

No PWM command or hardware access exists here. A sweep uses only the public
finite-motion launcher; fitting consumes admitted signed encoder displacement
and the final L12 output over the same physical measurement interval.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics
import subprocess
import time

from v3.config import ConfigResolver
from v3.layers.l11_actuator_control import WHEEL_CURVE_NAMES, WheelSpeedMap

ROOT = Path(__file__).resolve().parents[1]


def physical_binding(path):
    from v3.mcap_reader import McapReader, RUNTIME_TOPIC
    runtime = McapReader(path).first_json(RUNTIME_TOPIC)
    if not runtime or runtime[1].get("metadata", {}).get("runtime") != "v3_process_runtime":
        raise ValueError("physical calibration requires the canonical live hardware runtime")
    resolved = runtime[1]["configuration"]["resolved_robot"]["runtime"]
    inputs = resolved["sensor_inputs"]["inputs"]
    return {"motor_output": resolved["composition"]["motor_output"],
            "encoder_geometry": {name: inputs["encoder_backend"][name] for name in (
                "left_step_distance_m", "right_step_distance_m")},
            "encoder_counter": inputs["encoder_counter"]}


def capture_rows(path):
    from v3.mcap_inspect import inspect_mcap
    from v3.mcap_reader import McapReader, TICK_TOPIC, RUNTIME_TOPIC
    inspection = inspect_mcap(path, deep=True)
    reader = McapReader(path)
    final = reader.latest_metadata("r2b4.capture.final") or {}
    if (inspection["status"] != "PASS" or final.get("complete") != "true"
            or final.get("tick_sample_hz") != "50" or final.get("replay_complete") != "true"):
        raise ValueError("calibration requires complete, replay-eligible 50 Hz MCAP")
    runtime = reader.first_json(RUNTIME_TOPIC)
    if not runtime or runtime[1].get("metadata", {}).get("runtime") != "v3_process_runtime":
        raise ValueError("simulation/offline captures cannot establish a physical Speed Map")
    last_encoder = None
    for _, row in reader.iter_json_messages(topics=(TICK_TOPIC,)):
        layers = row.get("expected", {}).get("layers", {})
        final_output = layers.get("L12", {})
        wheels = layers.get("L10", {})
        admitted = layers.get("L2", {}).get("accepted_sample_indices", ())
        samples = row["inputs"]["raw_devices"]["samples"]
        encoder = next((samples[i] for i in admitted if samples[i]["kind"] == "wheel_velocity"), None)
        if encoder is not None:
            last_encoder = encoder
        elif last_encoder is not None and last_encoder in samples:
            # Independent acquisition/control clocks can repeat an already
            # admitted immutable sample. Preserve its ORIGINAL measurement
            # time/sequence; plateau fitting deduplicates it below. A missing
            # or changed unadmitted sample never receives this treatment.
            encoder = last_encoder
        valid = (encoder is not None and final_output.get("safety_decision") == "ALLOW"
                 and final_output.get("enabled") and not row["expected"].get("fault_layer")
                 and not layers.get("L11", {}).get("saturated")
                 and wheels.get("velocity_transition_until_ns") is None
                 and not wheels.get("motion_revoked"))
        values = {field["key"]: field["value"] for field in encoder["values"]} if encoder else {}
        measured = encoder["captured_monotonic_ns"] if encoder else row["monotonic_ns"]
        valid = (valid and values.get("measurement_timing_valid") is True
                 and values.get("measurement_stale") is False
                 and values.get("rejection_code") == "NONE"
                 and 0 <= row["monotonic_ns"] - measured <= 50_000_000)
        for side in ("left", "right"):
            yield {"side": side, "time_ns": row["monotonic_ns"], "measured_ns": measured,
                   "sequence": encoder["sequence"] if encoder else None,
                   "target": wheels.get(side + "_mps", 0.),
                   "output": final_output.get(side + "_output", 0.),
                   "distance": values.get("raw_" + side + "_distance_m"),
                   "valid": bool(valid and values.get(side + "_counter_running") is True),
                   "edge_ns": values.get(side + "_last_accepted_edge_timestamp_ns"),
                   "edge_direction": values.get(side + "_last_accepted_edge_direction")}


def _integrated_output(rows, start, end):
    total = 0.
    for first, second in zip(rows, rows[1:]):
        span = max(0, min(second["time_ns"], end) - max(first["time_ns"], start))
        total += abs(first["output"]) * span
    return total / (end - start)


def plateau_point(rows, *, settle_s=.5, minimum_s=1.):
    start = rows[0]["time_ns"] + int(settle_s * 1e9)
    measurements = []
    last_sequence = None
    for row in rows:
        if row["measured_ns"] < start or row["sequence"] == last_sequence:
            continue
        last_sequence = row["sequence"]
        measurements.append(row)
    if len(measurements) < 40 or (measurements[-1]["measured_ns"] - measurements[0]["measured_ns"]) < minimum_s * 1e9:
        return None, "SHORT_PLATEAU"
    # A fixed 0.5 s delay is only a lower bound. Discard the PI settling tail
    # in 0.25 s steps, retaining at least the SAME duration/sample evidence.
    original_start = measurements[0]["measured_ns"]
    while True:
        if len(measurements) < 40 or measurements[-1]["measured_ns"] - measurements[0]["measured_ns"] < minimum_s * 1e9:
            return None, "NOT_SETTLED"
        first, last = measurements[0], measurements[-1]
        span = (last["measured_ns"] - first["measured_ns"]) / 1e9
        velocity = (last["distance"] - first["distance"]) / span
        middle = measurements[len(measurements) // 2]
        speeds = [(b["distance"] - a["distance"]) / ((b["measured_ns"] - a["measured_ns"]) / 1e9)
                  for a, b in ((first, middle), (middle, last))]
        if abs(speeds[0] - speeds[1]) <= max(.005, .05 * abs(velocity)):
            break
        cutoff = first["measured_ns"] + 250_000_000
        measurements = [row for row in measurements if row["measured_ns"] >= cutoff]
    if abs(last["distance"] - first["distance"]) < .01:
        return None, "INSUFFICIENT_DISPLACEMENT"
    target = statistics.mean(row["target"] for row in measurements)
    if velocity * target <= 0 or abs(velocity) > 1.5:
        return None, "DIRECTION_OR_RANGE"
    # Feed-forward is measured at the actual speed. Tracking error is retained
    # as evidence, rather than relabelling that speed as the requested target.
    output = _integrated_output(rows, first["measured_ns"], last["measured_ns"])
    if not 0 < output <= 1:
        return None, "INVALID_DUTY"
    return {"speed_mps": abs(velocity), "pwm": output, "duration_s": span,
            "tracking_error_mps": velocity - target, "half_window_speeds_mps": speeds,
            "extra_settle_s": (first["measured_ns"] - original_start) / 1e9,
            "speed_spread_mps": abs(speeds[0] - speeds[1]),
            "first_measurement_ns": first["measured_ns"], "last_measurement_ns": last["measured_ns"]}, None


def fit_rows(rows, *, baseline: WheelSpeedMap, required_maximum_mps=.45):
    minimum_mps = baseline.minimum_continuous_speed_mps
    segments = {side: [] for side in ("left", "right")}
    points = {name: [] for name in WHEEL_CURVE_NAMES}
    rejected = Counter()

    def finish(side):
        segment = segments[side]
        if segment:
            point, reason = plateau_point(segment)
            if point:
                name = side + ("_forward" if segment[0]["target"] > 0 else "_reverse")
                points[name].append(point)
            else:
                rejected[reason] += 1
            segment.clear()

    for row in rows:
        side = row["side"]
        segment = segments[side]
        valid = (row["valid"] and row["distance"] is not None
                 and all(math.isfinite(row[key]) for key in ("target", "output", "distance"))
                 and abs(row["target"]) > 1e-9 and row["target"] * row["output"] > 0
                 and row["edge_direction"] == (1 if row["target"] > 0 else -1)
                 and row["edge_ns"] is not None
                 and 0 <= row["measured_ns"] - row["edge_ns"] <= 100_000_000)
        if segment and (not valid or row["target"] * segment[0]["target"] <= 0
                        or abs(row["target"] - segment[0]["target"]) > .01
                        or not 0 < row["time_ns"] - segment[-1]["time_ns"] <= 50_000_000):
            finish(side)
        if valid:
            segments[side].append(row)
    for side in segments:
        finish(side)
    curves = {}
    problems = []
    for name, evidence in points.items():
        ordered = sorted(evidence, key=lambda point: point["speed_mps"])
        # Repeated plateaus at the same physical speed form one robust point.
        groups = []
        for point in ordered:
            if groups and abs(point["speed_mps"] - statistics.mean(p["speed_mps"] for p in groups[-1])) <= .005:
                groups[-1].append(point)
            else:
                groups.append([point])
        fitted = [{"speed_mps": statistics.median(p["speed_mps"] for p in group),
                   "pwm": statistics.median(p["pwm"] for p in group)} for group in groups]
        if (len(fitted) < 2 or fitted[0]["speed_mps"] > minimum_mps * 1.05
                or fitted[-1]["speed_mps"] < required_maximum_mps):
            problems.append(name + ": INSUFFICIENT_RANGE")
            continue
        if any(b["pwm"] < a["pwm"] for a, b in zip(fitted, fitted[1:])):
            problems.append(name + ": NON_MONOTONIC_MEASUREMENTS")
            continue
        # A measured steady-speed point is NOT a measured stiction threshold.
        # Preserve the separately owned thresholds; setting them to the first
        # point would turn every subfloor ramp into an immediate full-floor FF.
        original = next(curve for curve in baseline.curves if curve.name == name)
        curves[name] = {"points": fitted, "maintenance_pwm": original.maintenance_output,
                        "startup_pwm": original.startup_output}
    report = {"schema": "R2B4_SPEED_CALIBRATION_V1", "status": "INSUFFICIENT_EVIDENCE" if problems else "PASS",
              "plateaus": points, "rejected_segments": dict(rejected), "problems": problems,
              "required_maximum_mps": required_maximum_mps,
              "limitations": ["Encoder displacement is not independent ground truth or slip validation.",
                              "No extrapolation; no automatic activation; PI gains require dynamic plant evidence."]}
    speed_map = None
    if not problems:
        speed_map = {"schema": "R2B4_WHEEL_SPEED_MAP_V2", "map_state": "ACTIVE", "curves": curves,
                     "minimum_continuous_speed_mps": minimum_mps}
        WheelSpeedMap.from_mapping(speed_map)
        report["common_maximum_mps"] = min(curve["points"][-1]["speed_mps"] for curve in curves.values())
    return speed_map, report


def sweep_plan(speeds, duration_s, track_length_m, resolved, direction="both"):
    control = resolved.runtime.composition.live_control.control
    limits = control.operational_constraints
    maximum = min(limits.max_v_mps, limits.wheel_limits.maximum_mps,
                  resolved.edges.command_ingress.maximum_linear_speed_mps, .5)
    if not 1.6 <= duration_s <= 20 or not math.isfinite(track_length_m) or track_length_m <= 0:
        raise ValueError("sweep needs finite duration 1.6..20 s and a measured positive free track length")
    if not speeds or any(not math.isfinite(v) or not limits.wheel_limits.minimum_mps <= v <= maximum for v in speeds):
        raise ValueError(f"sweep speeds must be inside the currently executable calibrated range {limits.wheel_limits.minimum_mps}..{maximum} m/s")
    commands = []
    if direction not in {"both", "forward", "backward"}:
        raise ValueError("invalid sweep direction")
    cumulative_distance = 0.
    for speed in sorted(set(speeds)):
        # Include stopping distance and a fixed body/placement margin. This
        # does not replace the operator's clearance check or final L12 safety.
        cumulative_distance += speed * duration_s
        distance = speed * duration_s if direction == "both" else cumulative_distance
        required = distance + speed * speed / (2 * limits.max_acceleration_mps2) + .4
        if required > track_length_m:
            raise ValueError(f"{speed} m/s requires at least {required:.2f} m free track for this sweep")
        for selected in (("forward", "backward") if direction == "both" else (direction,)):
            commands.append([str(ROOT / "r"), selected, str(duration_s), str(speed), "c", "50", "--json"])
    return commands


def _run_step(command, duration_s, interface, log):
    """Host-side supervision may revoke motion, never bypass final safety."""
    process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
    deadline = time.monotonic() + duration_s + 30
    try:
        while process.poll() is None:
            status = interface.read("operator.status")
            state = status.get("status") or {}
            mission = state.get("mission") or {}
            if (not status.get("runtime_running") or state.get("fault_layer")
                    or state.get("safety_decision") == "FAULT"
                    or (mission.get("mode") == "TELEOP" and state.get("safety_decision") == "STOP")):
                raise RuntimeError(f"sweep revoked at tick {state.get('tick_id')}: {state.get('safety_reason')}; no retry")
            if time.monotonic() > deadline:
                raise RuntimeError("sweep command deadline exceeded; no retry")
            time.sleep(.05)
        if process.returncode:
            raise subprocess.CalledProcessError(process.returncode, command)
    finally:
        if process.poll() is None:
            # A FAULT cannot confirm ready IDLE. Close the sweep-owned runtime
            # through its canonical STOP/shutdown, preserving the first error.
            try:
                interface.execute("operator.runtime.stop")
            finally:
                process.terminate()
                process.wait(timeout=5)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    fit = sub.add_parser("generate")
    fit.add_argument("captures", type=Path, nargs="+")
    fit.add_argument("--required-maximum", type=float, default=.45)
    for mode in ("plan", "run"):
        sweep = sub.add_parser(mode)
        sweep.add_argument("--speeds", type=float, nargs="+", required=True)
        sweep.add_argument("--duration-s", type=float, default=3.)
        sweep.add_argument("--track-length-m", type=float, required=True)
        sweep.add_argument("--direction", choices=("both", "forward", "backward"), default="both",
                           help="both requires this free length in EACH direction from the starting pose")
    for command in sub.choices.values():
        command.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=False)
    if args.mode == "generate":
        if not math.isfinite(args.required_maximum) or not 0 < args.required_maximum <= 1.5:
            raise ValueError("required maximum must be 0..1.5 m/s")
        resolved = ConfigResolver.for_project(ROOT).resolve()
        baseline = resolved.runtime.composition.live_control.control.speed_map
        all_points = []
        bindings = [physical_binding(path) for path in args.captures]
        if any(binding != bindings[0] for binding in bindings[1:]):
            raise ValueError("captures have different PWM/wiring/encoder geometry; calibrate one physical setup at a time")
        # Close each capture's segments independently; process/clock epochs
        # and encoder origins must never be joined across capture boundaries.
        for path in args.captures:
            all_points.extend(capture_rows(path))
            for side in ("left", "right"):
                all_points.append({"side": side, "valid": False, "distance": None,
                                   "time_ns": 0, "target": 0})
        speed_map, report = fit_rows(all_points, baseline=baseline, required_maximum_mps=args.required_maximum)
        report["captures"] = [str(path.resolve()) for path in args.captures]
        report["physical_binding"] = bindings[0]
        if speed_map:
            (args.output / "speed_map.json").write_text(json.dumps(speed_map, indent=2) + "\n")
    else:
        resolved = ConfigResolver.for_project(ROOT).resolve()
        commands = sweep_plan(args.speeds, args.duration_s, args.track_length_m, resolved, args.direction)
        report = {"status": "PLANNED", "commands": commands, "physical_motion": args.mode == "run"}
        if args.mode == "run":
            from v3.robot_interface import RobotInterface
            interface = RobotInterface()
            if interface.read("operator.status").get("runtime_running"):
                raise ValueError("sweep requires a stopped runtime so its capture and shutdown remain owned")
            try:
                with (args.output / "sweep.log").open("w") as log:
                    subprocess.run([str(ROOT / "r"), "runtime", "start", "c", "50", "--json"],
                                   cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=90)
                    status = interface.read("operator.status")
                    report["capture"] = interface.read("capture.status").get("path")
                    for command in commands:
                        _run_step(command, args.duration_s, interface, log)
                        status = interface.read("operator.status")
                        state = status.get("status") or {}
                        if not status.get("runtime_running") or state.get("fault_layer") or state.get("safety_decision") == "FAULT":
                            raise RuntimeError("sweep stopped at the first runtime/safety fault; no retry")
                report["status"] = "CAPTURED"
            except BaseException as exc:
                report.update(status="FAILED", reason=str(exc))
                (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
                raise
            finally:
                shutdown = subprocess.run([str(ROOT / "r"), "shutdown", "--json"], cwd=ROOT, timeout=30)
                if shutdown.returncode and report["status"] != "FAILED":
                    raise RuntimeError("sweep runtime shutdown failed")
            speed_map, fit_report = fit_rows(capture_rows(report["capture"]),
                baseline=resolved.runtime.composition.live_control.control.speed_map,
                required_maximum_mps=max(args.speeds))
            (args.output / "fit_report.json").write_text(json.dumps(fit_report, indent=2) + "\n")
            if speed_map:
                (args.output / "speed_map.json").write_text(json.dumps(speed_map, indent=2) + "\n")
            report["status"] = fit_report["status"]
    (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "output": str(args.output)}))
    return 0 if report["status"] in {"PASS", "PLANNED", "CAPTURED"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
