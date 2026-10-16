"""Paired motor-free experiments using the existing native V3/PWM plant.

This is an offline host tool, not an alternative controller or a physical
calibration. Navigation, motion, constraints and safety use production source.
"""
from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace

from r2b4_orchestration.motion_simulation import DifferentialDriveExperiment, motion_scenario

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "R2B4_MOTION_EXPERIMENT_V1"
OVERRIDE_SCHEMA = "R2B4_OFFLINE_CONFIG_OVERRIDE_V1"


def scenarios():
    """Fixed geometry/seed and independent truth, shared by every config variant."""
    base = {"kind": "differential_drive", "seed": 7, "duration_s": 12.,
            "goal": {"x_m": 1.5, "y_m": 0., "yaw_rad": 0.},
            "room": {"x_min_m": -2.5, "x_max_m": 4., "y_min_m": -2., "y_max_m": 2.}}
    return {
        "straight": {**base, "behavior": "motion"},
        "straight_asymmetric": {**base, "behavior": "motion", "plant": {"right_gain": 1.08}},
        "cruise_open": {**base, "behavior": "roomcruise", "duration_s": 20.},
        "cruise_clutter": {**base, "behavior": "roomcruise", "duration_s": 20., "obstacles": [
            {"x_m": 1.2, "y_m": .15, "radius_m": .3},
            {"x_m": 2.6, "y_m": -.85, "radius_m": .3},
            {"x_m": -.8, "y_m": .9, "radius_m": .18}]},
        "follow_moving": {**base, "behavior": "followperson"},
        "follow_occluded": {**base, "behavior": "followperson", "occlusion_s": [3., 5.]},
    }


def _merge(document, patch, path="vezerles"):
    if not isinstance(patch, dict):
        raise ValueError("config override must be a JSON object")
    result = deepcopy(document)
    for key, value in patch.items():
        if key not in result:
            # Follow Person profile can be introduced into an older config.
            if path == "vezerles.behavior" and key == "followperson":
                result[key] = {}
            elif path == "vezerles.behavior.followperson" and key in {"max_v_mps", "max_omega_rad_s"}:
                result[key] = None
            else:
                raise ValueError(f"unknown config override: {path}.{key}")
        if isinstance(value, dict):
            if not isinstance(result[key], dict):
                raise ValueError(f"config override changes object type: {path}.{key}")
            result[key] = _merge(result[key], value, path + "." + key)
        else:
            result[key] = deepcopy(value)
    return result


def resolve_config(root, patch=None, config_path=None):
    """Resolve the same four authorities as live; never persist an overlay."""
    from v3.config import ConfigResolver
    conf = Path(root) / "conf"
    docs = [json.loads((conf / name).read_text()) for name in ("hardver.json", "fizika.json", "speed_map.json")]
    control = json.loads((Path(config_path) if config_path else conf / "vezerles.json").read_text())
    return ConfigResolver.from_documents(*docs, _merge(control, patch or {}))


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


class _BehaviorPlant(DifferentialDriveExperiment):
    """Supply camera and moving LiDAR geometry before the canonical L1 edge."""

    def __init__(self, *args, behavior, occlusion_s=(), **kwargs):
        self.behavior, self.occlusion_s = behavior, occlusion_s
        super().__init__(*args, **kwargs)

    def person_position(self):
        elapsed = (self.now_ns - self.start_ns) / 1e9
        return 1.6 + .08 * elapsed, .25 * math.sin(elapsed / 3.)

    def physical_clearance(self):
        room = self.scenario["room"]
        walls = min(self.x-room["x_min_m"], room["x_max_m"]-self.x,
                    self.y-room["y_min_m"], room["y_max_m"]-self.y)
        circles = [math.hypot(self.x-row["x_m"], self.y-row["y_m"])-row["radius_m"]
                   for row in self._obstacles()]
        return min([walls, *circles])-self.radius

    def _obstacles(self):
        rows = super()._obstacles()
        if self.behavior != "followperson":
            return rows
        x, y = self.person_position()
        return [*rows, {"x_m": x, "y_m": y, "radius_m": .18}]

    def _additional_samples(self, sample):
        if self.behavior != "followperson":
            return ()
        # A camera observation shares the scan's actual measurement timestamp;
        # intervening control ticks cannot renew an old detection's freshness.
        if self.tick % 5 == 0:
            x, y = self.person_position()
            bearing = math.atan2(math.sin(math.atan2(y-self.y, x-self.x)-self.yaw),
                                 math.cos(math.atan2(y-self.y, x-self.x)-self.yaw))
            elapsed = (self.now_ns-self.start_ns)/1e9
            hidden = self.occlusion_s and self.occlusion_s[0] <= elapsed < self.occlusion_s[1]
            fov = self.config.world_model.person_camera_horizontal_fov_rad
            visible = not hidden and fov is not None and abs(bearing) < fov/2
            fields = {"person_count": int(visible), "person_detected": bool(visible),
                      "source_frame_sequence": self.tick//5+1, "measurement_timing_valid": True,
                      "measurement_stale": False, "inference_duration_ns": 0}
            if visible:
                half = math.asin(min(1., .18 / max(.18, math.hypot(x-self.x, y-self.y))))
                fields.update(primary_confidence=.95, primary_xmin=.3, primary_xmax=.7,
                              geometry_projection_state="VALID", primary_bearing_left_rad=bearing+half,
                              primary_bearing_right_rad=bearing-half)
            self._person_samples = (sample("PERSON_DETECTOR_FRONT", "person_detection", **fields),)
        return self._person_samples


def _metrics(rows, config, scenario):
    active = [row for row in rows if row["active"] and row["mission_lifecycle"] == "ACTIVE"
              and row["mission_mode"] != "STOP" and row["navigation_status"] != "COMPLETE"]
    dt = config.runtime.tick_period_ns / 1e9
    stopped = longest = current = reversals = 0
    previous_sign = 0
    reasons, follow_states = Counter(), Counter()
    coverage = set()
    path = progress_hold = 0.
    window = max(1, math.ceil(2./dt))
    for index, row in enumerate(active):
        stationary = abs(row["v_mps"]) < .02 and abs(row["omega_rad_s"]) < .1
        current = current+1 if stationary else 0
        stopped += stationary
        longest = max(longest, current)
        sign = 1 if row["omega_rad_s"] > .05 else -1 if row["omega_rad_s"] < -.05 else 0
        if sign and previous_sign and sign != previous_sign:
            reversals += 1
        if sign:
            previous_sign = sign
        if index:
            previous = active[index-1]
            path += math.hypot(row["x_m"]-previous["x_m"], row["y_m"]-previous["y_m"])
        if index >= window:
            previous = active[index-window]
            if math.hypot(row["x_m"]-previous["x_m"], row["y_m"]-previous["y_m"]) < .05:
                progress_hold += dt
        size = config.runtime.composition.live_control.control.navigation.coverage_cell_size_m
        coverage.add((math.floor(row["x_m"]/size), math.floor(row["y_m"]/size)))
        reasons[str(row["navigation_reason"])] += 1
        if row["follow_state"]:
            follow_states[row["follow_state"]] += 1
    count = len(active)
    result = {"active_duration_s": count*dt, "path_length_m": path,
              "unique_coverage_cells": len(coverage), "stopped_duration_s": stopped*dt,
              "longest_stop_s": longest*dt, "no_translation_progress_2s_duration_s": progress_hold,
              "moving_ratio": 1.-stopped/count if count else None,
              "active_mission_sample_count": count,
              "angular_sign_reversals": reversals,
              "angular_sign_reversals_per_m": reversals/path if path > .01 else None,
              "navigation_reason_duration_s": {key: value*dt for key, value in reasons.items()},
              "follow_state_duration_s": {key: value*dt for key, value in follow_states.items()},
              "minimum_footprint_clearance_m": min((row["physical_clearance_m"] for row in active), default=None),
              "max_abs_jerk_mps3": max((row["max_abs_jerk_mps3"] for row in active), default=0.)}
    if scenario["behavior"] == "motion":
        result.update(straight_cross_track_rms_m=math.sqrt(sum(row["y_m"]**2 for row in active)/count) if count else None,
                      straight_max_abs_cross_track_m=max((abs(row["y_m"]) for row in active), default=None),
                      straight_heading_rms_rad=math.sqrt(sum(row["yaw_rad"]**2 for row in active)/count) if count else None)
    if scenario["behavior"] == "followperson":
        distances = [row["person_distance_m"] for row in active]
        stand_off = config.runtime.composition.live_control.control.navigation.follow_person_stand_off_m
        result.update(person_distance_min_m=min(distances, default=None),
                      person_standoff_error_rms_m=math.sqrt(sum((distance-stand_off)**2 for distance in distances)/count) if count else None)
    return result


def run_experiment(resolved, scenario_name, directory, *, scenario=None, replay=False):
    """Finite closed loop with an in-memory motor sink and canonical command gateway."""
    from v3.capture_encoding import encode_value
    from v3.composition.full_fake import OfflineMotorSink
    from v3.composition.native_control import NativeControlComposition
    from v3.mcap_capture import McapCaptureConfig
    from v3_process_runtime import McapCaptureSession

    scenario = deepcopy(scenario if scenario is not None else scenarios()[scenario_name])
    behavior, occlusion = scenario["behavior"], scenario.get("occlusion_s", ())
    plant_scenario = motion_scenario({key: value for key, value in scenario.items()
                                     if key not in {"behavior", "occlusion_s"}})
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    config = resolved.runtime.composition.live_control.control
    if behavior == "followperson" and not config.world_model.person_tracking_enabled:
        raise ValueError("Follow Person experiment requires production person_tracking_enabled")
    writer, rows = OfflineMotorSink(), []
    composition = NativeControlComposition(writer, config)
    session = McapCaptureSession("motion-"+scenario_name, directory/"capture.mcap",
        configuration={"production_control": config},
        metadata={"source": "simulation", "scenario": scenario, "physical_hardware": False},
        config=McapCaptureConfig(mode="append_only", tick_sample_hz=50), capacity=2048)
    timeline = (directory/"timeline.ndjson").open("w")
    plant = None

    def observe(record, scan):
        if scan is not None:
            session.hub.publish(scan, topic="v3.raw_lidar")
        session.observe(record)
        layers = {row.layer: row.output for row in record.result.trace.layers}
        follow = next((item for item in record.evidence if type(item).__name__ == "FollowPersonEvidence"), None)
        nav, mission = layers.get("L6"), layers.get("L5")
        person = plant.person_position() if behavior == "followperson" else None
        row = {"tick_id": record.inputs.context.tick_id, "measurement_monotonic_ns": plant.now_ns,
               "active": record.inputs.lifecycle.value == "ACTIVE", "x_m": plant.x, "y_m": plant.y,
               "mission_mode": getattr(getattr(mission, "mode", None), "value", None),
               "mission_lifecycle": getattr(getattr(mission, "lifecycle", None), "value", None),
               "mission_id": getattr(mission, "mission_id", None),
               "yaw_rad": plant.yaw, "v_mps": plant.previous_v,
               "physical_clearance_m": plant.physical_clearance(),
               "omega_rad_s": (plant.right-plant.left)*plant_scenario["plant"]["traction"]/config.chassis_control.track_width_m,
               "max_abs_jerk_mps3": plant.max_jerk, "navigation_reason": getattr(nav, "reason", None),
               "navigation_status": getattr(getattr(nav, "status", None), "value", None),
               "follow_state": getattr(follow, "state", None),
               "person_distance_m": math.hypot(person[0]-plant.x, person[1]-plant.y) if person else None,
               "allowed_v_mps": getattr(layers.get("L9"), "allowed_v_mps", None),
               "allowed_omega_rad_s": getattr(layers.get("L9"), "allowed_omega_rad_s", None),
               "safety_decision": record.result.final_actuation.safety_decision.value,
               "safety_reason": record.result.final_actuation.reason, "fault_layer": record.result.trace.fault_layer,
               "fault_evidence": [encode_value(item) for item in record.evidence
                                  if type(item).__name__ == "LayerFaultEvidence"]}
        rows.append(row)
        timeline.write(json.dumps(row, allow_nan=False)+"\n")
        if session.failed:
            raise RuntimeError("SIMULATION_CAPTURE_INTEGRITY_FAILURE")

    ingress = resolved.edges.command_ingress
    plant = _BehaviorPlant(plant_scenario, config, composition, writer, directory, 1_000_000_000,
        observe, lambda *args: None, behavior=behavior, occlusion_s=occlusion,
        tick_period_ns=resolved.runtime.tick_period_ns,
        maximum_linear_speed_mps=ingress.maximum_linear_speed_mps,
        maximum_angular_speed_rad_s=ingress.maximum_angular_speed_rad_s,
        lidar_ray_count=min(96, config.world_model.local_costmap_max_points_per_scan))
    error = None
    termination = "DURATION_REACHED"
    session.start()
    try:
        plant.step()
        ticks = math.ceil(plant_scenario["duration_s"]*1e9/plant.dt_ns)
        for _ in range(ticks):
            if plant.fault_layer or plant.collision_count:
                termination = "NATIVE_FAULT" if plant.fault_layer else "FOOTPRINT_COLLISION"
                break
            if behavior == "roomcruise":
                profile = resolved.roomcruise
                plant.client.publish_explore("simulation.cruise", max_v_mps=profile.max_v_mps,
                    max_omega_rad_s=profile.max_omega_rad_s, preferences=profile.preferences, ttl_ns=200_000_000)
            elif behavior == "followperson":
                profile = resolved.followperson
                plant.client.publish_follow_person("simulation.follow", max_v_mps=profile.max_v_mps,
                    max_omega_rad_s=profile.max_omega_rad_s, ttl_ns=200_000_000)
            else:
                plant.client.publish_navigate("simulation.straight", **plant_scenario["goal"],
                    frame_id="R2B4_ODOM_LOCAL", max_v_mps=.25,
                    max_omega_rad_s=min(1.4, ingress.maximum_angular_speed_rad_s), ttl_ns=200_000_000)
            result = plant.step(active=True)
            if plant.fault_layer or plant.collision_count:
                termination = "NATIVE_FAULT" if plant.fault_layer else "FOOTPRINT_COLLISION"
                break
            if behavior == "motion" and plant.status["navigation"]["status"] == "COMPLETE":
                termination = "GOAL_COMPLETE"
                break
        plant.settle()
    except BaseException as exc:
        error = exc
        raise
    finally:
        composition.close()
        timeline.close()
        capture = session.finalize(SimpleNamespace(status=0 if error is None else 1), error=error)
    metrics = _metrics(rows, resolved, scenario)
    performance = plant.performance()
    metrics.update(collision_count=plant.collision_count, safety_stop_count=plant.safety_stop_count,
                   tracking_error_rms_mps=performance["metrics"]["tracking_error_rms_mps"])
    if behavior == "motion":
        metrics["goal_position_error_m"] = performance["metrics"]["goal_position_error_m"]
    replay_result = None
    if replay:
        from v3.replay import replay_capture
        replay_result = replay_capture(capture, project_root=PROJECT_ROOT)
    result = {"scenario": scenario_name, "scenario_input": scenario, "scenario_sha256": _digest(scenario),
              "config_snapshot_id": resolved.snapshot_id,
              "control_config_sha256": _digest(encode_value(config)), "metrics": metrics,
              "fault_layer": plant.fault_layer, "first_boundary": plant.first_boundary,
              "termination_reason": termination, "requested_duration_s": plant_scenario["duration_s"],
              "observed_runtime_active_s": sum(row["active"] for row in rows)*plant.dt_ns/1e9,
              "early_terminal": termination in {"NATIVE_FAULT", "FOOTPRINT_COLLISION", "GOAL_COMPLETE"},
              "final_stop": plant.output == (0., 0.), "physical_output": False,
              "capture": str(capture), "timeline": str(directory/"timeline.ndjson"),
              "replay": replay_result}
    (directory/"result.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    return result


def compare_results(baseline, variant):
    if baseline["scenario_sha256"] != variant["scenario_sha256"]:
        raise ValueError("paired scenarios differ")
    deltas = {key: value-baseline["metrics"][key] for key, value in variant["metrics"].items()
              if type(value) in {float, int} and type(baseline["metrics"].get(key)) in {float, int}}
    return {"scenario": baseline["scenario"], "metric_deltas_variant_minus_baseline": deltas,
            "baseline_fault": baseline["fault_layer"], "variant_fault": variant["fault_layer"],
            "baseline_termination": baseline["termination_reason"],
            "variant_termination": variant["termination_reason"],
            "automatic_recommendation": False}


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--project-root", type=Path, default=PROJECT_ROOT)
    result.add_argument("--config", type=Path, help="alternate complete vezerles.json; production file is never written")
    result.add_argument("--override", type=Path, action="append", default=[], help="R2B4_OFFLINE_CONFIG_OVERRIDE_V1 JSON with vezerles overlay; each file is a separate paired variant")
    result.add_argument("--scenarios", default="straight,straight_asymmetric,cruise_open,cruise_clutter,follow_moving,follow_occluded")
    result.add_argument("--duration-s", type=float, help="same simulated duration for each selected scenario (0.1..30)")
    result.add_argument("--output", type=Path, help="new artifact directory; default unique /tmp/r2b4-motion-*")
    result.add_argument("--replay", action="store_true", help="explicitly run canonical replay on every generated capture")
    result.add_argument("--list-scenarios", action="store_true")
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    available = scenarios()
    if args.list_scenarios:
        print("\n".join(available))
        return 0
    names = [name.strip() for name in args.scenarios.split(",") if name.strip()]
    if not names or set(names)-available.keys():
        raise SystemExit("select known nonempty scenarios with --list-scenarios")
    if args.duration_s is not None and (not math.isfinite(args.duration_s) or not .1 <= args.duration_s <= 30.):
        raise SystemExit("duration-s must be finite within [0.1, 30]")
    variants = [("baseline", {}, resolve_config(args.project_root, config_path=args.config))]
    for index, path in enumerate(args.override):
        overlay = json.loads(path.read_text())
        if (not isinstance(overlay, dict) or set(overlay) != {"schema", "vezerles"}
                or overlay["schema"] != OVERRIDE_SCHEMA or not isinstance(overlay["vezerles"], dict)):
            raise SystemExit(f"{path}: expected {OVERRIDE_SCHEMA} with vezerles object")
        patch = overlay["vezerles"]
        variants.append((f"variant-{index+1}", patch, resolve_config(args.project_root, patch, args.config)))
    if args.output:
        output = args.output.resolve()
        output.mkdir(parents=True, exist_ok=False)
    else:
        output = Path(tempfile.mkdtemp(prefix="r2b4-motion-"))
    results = []
    for label, patch, config in variants:
        trials = []
        for name in names:
            scenario = deepcopy(available[name])
            if args.duration_s is not None:
                scenario["duration_s"] = args.duration_s
            trials.append(run_experiment(config, name, output/label/name, scenario=scenario, replay=args.replay))
        results.append({"name": label, "override": patch, "trials": trials})
    comparisons = [{"variant": variant["name"], **compare_results(base, changed)} for variant in results[1:]
                   for base, changed in zip(results[0]["trials"], variant["trials"])]
    payload = {"schema": SCHEMA, "physical_output": False, "results": results, "comparisons": comparisons,
               "limitations": [
                   "Synthetic wheel/PWM/traction and ideal IMU/relative-LiDAR evidence; no physical calibration or live acceptance.",
                   "Native asynchronous input closure uses deterministic inline rollout, not process/GIL/IPC/scheduler timing.",
                   "Room Cruise coverage and continuity are proxies, not proof of globally optimal route choice.",
                   "No-progress includes pivots and intentional Follow Person stand-off/occlusion holds; inspect reasons/states.",
                   "Synthetic camera boxes and LiDAR person geometry do not validate detector accuracy or camera calibration."]}
    (output/"comparison.json").write_text(json.dumps(payload, indent=2, allow_nan=False)+"\n")
    failed = any(trial["fault_layer"] or trial["metrics"]["collision_count"] or not trial["final_stop"]
                 or (args.replay and trial["replay"]["status"] != "MATCH")
                 for result in results for trial in result["trials"])
    print(json.dumps({"schema": SCHEMA, "output": str(output/"comparison.json"),
                      "status": "FAILED" if failed else "COMPLETED", "physical_output": False}))
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
