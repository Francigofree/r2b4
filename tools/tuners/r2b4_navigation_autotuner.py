"""Explicit motor-free navigation search anchored to finalized recorded runs.

Recorded behavior logs are diagnostic context, not counterfactual sensor input.
Live recordings therefore require an explicit surrogate scenario suite. Search
uses the native V3/PWM plant; a shortlist requires independent holdout scenes,
paired safety checks and MATCH replay of newly recorded validation trials.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import tempfile
import time

from . import r2b4_motion_tuner as motion

PROJECT_ROOT = motion.PROJECT_ROOT
SCHEMA = "R2B4_NAVIGATION_AUTOTUNE_V1"
SUITE_SCHEMA = "R2B4_NAVIGATION_TUNING_SCENARIOS_V1"
DOCUMENTS = ("hardver.json", "fizika.json", "speed_map.json", "vezerles.json")
# Explicit behavior/realization knobs only. Safety, freshness, footprint,
# localization quality and final motor limits cannot be searched by this tool.
AXES = (
    "layers.navigation.progress_weight", "layers.navigation.clearance_weight",
    "layers.navigation.smoothness_weight", "layers.navigation.novelty_weight",
    "layers.navigation.localization_observability_weight",
    "behavior.roomcruise.preferences.local_goal_forward_weight",
    "behavior.roomcruise.preferences.local_goal_novelty_weight",
    "behavior.roomcruise.preferences.local_goal_clearance_weight",
    "behavior.roomcruise.max_v_mps",
    "layers.motion_realization.heading_gain", "layers.motion_realization.cross_track_gain",
    "layers.motion_realization.angular_velocity_gain",
)


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def read_documents(root, config_path=None):
    paths = [Path(root) / "conf" / name for name in DOCUMENTS]
    if config_path is not None:
        paths[-1] = Path(config_path)
    # One immutable document snapshot throughout the entire search.
    documents = [json.loads(path.read_bytes()) for path in paths]
    lineage = [{"path": str(path.resolve()), "document_sha256": motion._digest(doc)}
               for path, doc in zip(paths, documents)]
    return documents, lineage


def resolve_documents(documents, patch=None):
    from v3.config import ConfigResolver
    return ConfigResolver.from_documents(*documents[:3], motion._merge(documents[3], patch or {}))


def generate_overrides(control, count=200, seed=7):
    """Sample unique bounded combinations, including coupled ranking/gain knobs."""
    if type(count) is not int or not 1 <= count <= 512:
        raise ValueError("variants must be an integer within [1, 512]")
    if type(seed) is not int or not 0 <= seed <= 2**31 - 1:
        raise ValueError("seed must be an integer within [0, 2**31-1]")
    axes = []
    for path in AXES:
        value = control
        for key in path.split("."):
            value = value[key]
        if type(value) not in {int, float} or not math.isfinite(value) or value < 0:
            raise ValueError("search axis must be finite nonnegative: " + path)
        values = ((value, value * .9, value * .8) if path.endswith("max_v_mps") else
                  (value, value * .8, value * 1.2) if value else (0., .05, .1))
        axes.append((path.split("."), tuple(dict.fromkeys(values))))
    total = math.prod(len(values) for _, values in axes)
    if count >= total:
        raise ValueError("requested variants exceed nonbaseline search space")
    patches = []
    for number in random.Random(seed).sample(range(1, total), count):
        patch = {}
        for path, values in axes:
            number, index = divmod(number, len(values))
            target = patch
            for key in path[:-1]:
                target = target.setdefault(key, {})
            target[path[-1]] = values[index]
        patches.append(patch)
    return patches


def read_recording(path, root, evidence=None):
    """Verify original authority and retain recorded DIAG independently of scores."""
    from v3.mcap_inspect import inspect_mcap
    from v3.mcap_reader import McapReader, RUNTIME_TOPIC
    from v3.replay import replay_capture, _production_control_config, V3ReplayError
    from v3.capture_encoding import encode_value

    path = Path(path)
    inspection = inspect_mcap(path, deep=True)
    if inspection["status"] != "PASS" or inspection["final_metadata"].get("complete") != "true":
        raise ValueError("EVIDENCE_BLOCKED: finalized complete capture scope required")
    digest = _sha256(path)
    reader = McapReader(path)
    first = next(reader.iter_messages(topics=(RUNTIME_TOPIC,)), None)
    runtime = first.json() if first else {}
    metadata = runtime.get("metadata", {})
    try:
        captured_control, _ = _production_control_config(runtime, ())
        control_digest = motion._digest(encode_value(captured_control))
    except V3ReplayError:
        control_digest = None
    replay = ({"status": "NOT_ELIGIBLE_BEHAVIOR_LOG", "reason": "Sampled log has no closed control inputs."}
              if inspection["final_metadata"].get("replay_complete") != "true"
              else replay_capture(path, project_root=root))
    if replay["status"] not in {"MATCH", "NOT_ELIGIBLE_BEHAVIOR_LOG"}:
        raise ValueError("EVIDENCE_BLOCKED: canonical source replay did not MATCH")
    recording = {"capture": str(path.resolve()), "capture_sha256": digest,
                 "capture_metadata": inspection["capture_metadata"],
                 "capture_integrity": inspection["final_metadata"], "replay": replay,
                 "configuration_sha256": motion._digest(runtime.get("configuration", {})),
                 "captured_control_config_sha256": control_digest,
                 "metadata": metadata, "observed_diagnostics": None}
    if evidence is not None:
        from tools.diag.context import open_context
        from tools.diag.registry import build_default_registry
        from tools.mcap_evidence.verify import verify_artifacts
        context = open_context(Path(root), evidence)
        bundle = context.bundle
        if bundle.manifest["source"].get("sha256") != digest:
            raise ValueError("EVIDENCE_BLOCKED: EVI and capture SHA256 differ")
        verified = verify_artifacts(bundle.root, bundle.manifest, source=path)
        if verified["status"] != "PASS":
            raise ValueError("EVIDENCE_BLOCKED: EVI verification failed")
        registry = build_default_registry()
        recording["observed_diagnostics"] = {
            "evidence": str(bundle.root), "manifest_sha256": _sha256(bundle.root / "manifest.json"),
            "verification": verified,
            "reports": [registry.run(name, context).as_dict() for name in
                        ("navigation", "navigation_root_cause", "motion_performance")],
            "comparison_scope": "Recorded estimator/decision measurements; separate from synthetic plant truth scores."}
    return recording


def _scenario(value):
    from r2b4_orchestration.motion_simulation import motion_scenario
    if not isinstance(value, dict) or value.get("behavior") not in {"motion", "roomcruise", "followperson"}:
        raise ValueError("each scenario requires behavior: motion, roomcruise or followperson")
    plant = motion_scenario({key: item for key, item in value.items() if key not in {"behavior", "occlusion_s"}})
    if plant["v3_overrides"]:
        raise ValueError("scenario v3_overrides would hide the selected configuration")
    result = {**plant, "behavior": value["behavior"]}
    if "occlusion_s" in value:
        span = value["occlusion_s"]
        if (value["behavior"] != "followperson" or not isinstance(span, list) or len(span) != 2
                or any(type(x) not in {int, float} or not math.isfinite(x) for x in span)
                or not 0 <= span[0] < span[1] <= plant["duration_s"]):
            raise ValueError("occlusion_s requires an ordered Follow Person interval inside duration")
        result["occlusion_s"] = span
    return result


def read_suite(path, recording):
    """Live evidence needs explicit independently described train/holdout scenes."""
    if path is None:
        metadata = recording["metadata"]
        if metadata.get("source") != "simulation" or metadata.get("physical_hardware") is not False:
            raise ValueError("EVIDENCE_BLOCKED: live behavior capture requires --scenarios surrogate suite")
        _scenario(metadata.get("scenario"))
        # A different seed alone does not hold out deterministic room geometry.
        raise ValueError("EVIDENCE_BLOCKED: supply --scenarios with distinct train/holdout geometry; recorded scenario available in metadata")
    path = Path(path)
    if path.stat().st_size > 65_536:
        raise ValueError("scenario suite exceeded 65536 bytes")
    value = json.loads(path.read_bytes())
    if not isinstance(value, dict) or set(value) != {"schema", "source_capture_sha256", "train", "holdout"} or value["schema"] != SUITE_SCHEMA:
        raise ValueError("expected " + SUITE_SCHEMA + " with source_capture_sha256, train, holdout")
    if value["source_capture_sha256"] != recording["capture_sha256"]:
        raise ValueError("scenario suite is not bound to source capture SHA256")
    suite = {}
    for group in ("train", "holdout"):
        rows = value[group]
        if not isinstance(rows, list) or not 1 <= len(rows) <= 4:
            raise ValueError(group + " requires 1..4 explicit scenarios")
        suite[group] = [_scenario(row) for row in rows]
    geometry = lambda row: motion._digest({key: item for key, item in row.items() if key not in {"seed", "environment_id", "duration_s"}})
    if {geometry(row) for row in suite["train"]} & {geometry(row) for row in suite["holdout"]}:
        raise ValueError("holdout requires distinct geometry, behavior or plant; renaming/reseeding is insufficient")
    return suite, {"path": str(path.resolve()), "sha256": _sha256(path),
                   "scope": "Explicit reconstructed surrogate; no physical counterfactual or exact replay claim."}


def _score(trial):
    m = trial["metrics"]
    elapsed = max(m["active_duration_s"], .02)
    behavior = trial["scenario_input"]["behavior"]
    if behavior == "motion":
        return -(m["goal_position_error_m"] + m["straight_cross_track_rms_m"] + m["tracking_error_rms_mps"]
                 + .01 * trial["observed_runtime_active_s"])
    if behavior == "followperson":
        return -m["person_standoff_error_rms_m"]
    return (m["unique_coverage_cells"] / elapsed - .5 * m["stopped_duration_s"] / elapsed
            - .02 * m["angular_sign_reversals"] / elapsed)


def paired_assessment(baseline, variant):
    comparison = motion.compare_results(baseline, variant)
    blockers = list(comparison["quality_comparison_blockers"])
    if comparison["completion_time_comparable"]:
        blockers = [reason for reason in blockers if reason != "DIFFERENT_OBSERVED_EXPOSURE"]
    for key in ("safety_stop_count", "max_abs_jerk_mps3"):
        if variant["metrics"][key] > baseline["metrics"][key] + 1e-9:
            blockers.append("REGRESSION:" + key)
    clearance = variant["metrics"]["minimum_footprint_clearance_m"]
    floor = baseline["footprint_safety_margin_m"]
    if clearance is None or clearance < floor:
        blockers.append("SAFETY:minimum_footprint_clearance_m")
    score = None if blockers else _score(variant)
    original = None if blockers else _score(baseline)
    return {**comparison, "eligible": not blockers, "rejection_reasons": blockers,
            "baseline_score": original, "variant_score": score,
            "normalized_score_gain": (score - original) / max(1., abs(original)) if not blockers else None}


def _group_summary(pairs, minimum):
    eligible = all(row["eligible"] for row in pairs)
    gains = [row["normalized_score_gain"] for row in pairs] if eligible else []
    mean = sum(gains) / len(gains) if gains else None
    return {"eligible": eligible, "mean_normalized_score_gain": mean,
            "improved": eligible and mean > minimum and min(gains) >= -1e-9, "pairs": pairs}


def run_search(documents, suite, output, *, count=200, seed=7, finalists=3,
               minimum_improvement=.01, budget_s=3600.):
    """Bounded paired train search followed by held-out canonical validation."""
    if type(finalists) is not int or not 1 <= finalists <= 10:
        raise ValueError("finalists must be within [1, 10]")
    if not math.isfinite(minimum_improvement) or minimum_improvement < 0:
        raise ValueError("minimum improvement must be finite nonnegative")
    if not math.isfinite(budget_s) or not 1 <= budget_s <= 86400:
        raise ValueError("budget-s must be finite within [1, 86400]")
    output = Path(output)
    started = time.monotonic()
    baseline_config = resolve_documents(documents)
    patches = generate_overrides(documents[-1], count, seed)
    baseline, rows, shortlist = {}, [], []

    def trials(config, group, directory, *, capture):
        measured = []
        for i, scenario in enumerate(suite[group]):
            if time.monotonic() - started >= budget_s:
                break
            measured.append(motion.run_experiment(config, f"{group}-{i}", directory / f"{group}-{i}",
                scenario=scenario, record_capture=capture, replay=capture))
        return measured

    for group in ("train", "holdout"):
        baseline[group] = trials(baseline_config, group, output / "baseline", capture=True)
        if len(baseline[group]) != len(suite[group]):
            return {"status": "BUDGET_EXHAUSTED", "baseline": baseline, "variants": [], "shortlist": [],
                    "planned_variants": count, "tested_variants": 0, "search_complete": False}
    baseline_valid = all(trial["replay"]["status"] == "MATCH" and paired_assessment(trial, trial)["eligible"]
                         for trials in baseline.values() for trial in trials)
    if not baseline_valid:
        return {"status": "BASELINE_BLOCKED", "baseline": baseline, "variants": [], "shortlist": [],
                "planned_variants": count, "tested_variants": 0, "search_complete": False}
    for index, patch in enumerate(patches):
        if time.monotonic() - started >= budget_s:
            break
        row = {"name": f"variant-{index + 1:04d}", "override": patch}
        try:
            config = resolve_documents(documents, patch)
        except ValueError as exc:
            row.update(status="CONFIG_REJECTED", reason=str(exc))
            rows.append(row)
            continue
        measured = trials(config, "train", output / row["name"], capture=False)
        if len(measured) != len(suite["train"]):
            row.update(status="TRAIN_INCOMPLETE", train_trials=measured)
            rows.append(row)
            break
        row.update(status="TRAIN_EVALUATED", config_snapshot_id=config.snapshot_id, train_trials=measured,
                   train=_group_summary([paired_assessment(a, b) for a, b in zip(baseline["train"], measured)], minimum_improvement))
        rows.append(row)
    ranked = sorted((row for row in rows if row.get("train", {}).get("improved")),
                    key=lambda row: (-row["train"]["mean_normalized_score_gain"], row["name"]))[:finalists]
    for row in ranked:
        if time.monotonic() - started >= budget_s:
            break
        config = resolve_documents(documents, row["override"])
        measured = trials(config, "holdout", output / row["name"], capture=True)
        row["holdout_trials"] = measured
        if len(measured) != len(suite["holdout"]):
            row["status"] = "HOLDOUT_INCOMPLETE"
            break
        row["holdout"] = _group_summary([paired_assessment(a, b) for a, b in zip(baseline["holdout"], measured)], minimum_improvement)
        if not row["holdout"]["improved"] or any(trial["replay"]["status"] != "MATCH" for trial in measured):
            row["status"] = "HOLDOUT_REJECTED"
            continue
        validation = trials(config, "train", output / row["name"] / "validation", capture=True)
        row["validation_train_trials"] = validation
        if len(validation) != len(suite["train"]):
            row["status"] = "VALIDATION_INCOMPLETE"
            break
        if any(trial["replay"]["status"] != "MATCH" for trial in validation) or any(
                a["metrics"] != b["metrics"] for a, b in zip(row["train_trials"], validation)):
            row["status"] = "VALIDATION_REJECTED"
            continue
        overlay = output / row["name"] / "override.json"
        _write(overlay, {"schema": motion.OVERRIDE_SCHEMA, "vezerles": row["override"]})
        row["status"] = "PROMISING_IN_SIMULATION"
        shortlist.append({"name": row["name"], "override": str(overlay), "override_sha256": _sha256(overlay),
                          "train_gain": row["train"]["mean_normalized_score_gain"],
                          "holdout_gain": row["holdout"]["mean_normalized_score_gain"],
                          "live_validation": "NOT_PERFORMED", "automatic_application": False})
    complete = (len(rows) == count and all(row["status"] != "TRAIN_INCOMPLETE" for row in rows)
                and all(row["status"] in {"HOLDOUT_REJECTED", "VALIDATION_REJECTED", "PROMISING_IN_SIMULATION"} for row in ranked))
    return {"status": "COMPLETED" if complete else "BUDGET_EXHAUSTED", "baseline": baseline,
            "variants": rows, "shortlist": shortlist, "planned_variants": count,
            "tested_variants": sum("train" in row for row in rows),
            "search_complete": complete,
            "elapsed_s": time.monotonic() - started}


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--project-root", type=Path, default=PROJECT_ROOT)
    result.add_argument("--capture", type=Path, required=True, help="original finalized recorded MCAP authority")
    result.add_argument("--evidence", type=Path, help="matching sealed EVI bundle; reads recorded DIAG metrics and lineage")
    result.add_argument("--scenarios", type=Path, required=True, help=SUITE_SCHEMA + " JSON; source hash, train and holdout arrays")
    result.add_argument("--config", type=Path, help="explicit complete alternate vezerles.json; never written")
    result.add_argument("--variants", type=int, default=200, help="bounded search size [1,512]")
    result.add_argument("--seed", type=int, default=7)
    result.add_argument("--finalists", type=int, default=3, help="maximum held-out variants [1,10]")
    result.add_argument("--minimum-improvement", type=float, default=.01)
    result.add_argument("--budget-s", type=float, default=3600., help="stop between finite trials; at most eight 30s simulated scenes per variant")
    result.add_argument("--output", type=Path, help="new artifact directory; default unique /tmp/r2b4-autotune-*")
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        documents, config_lineage = read_documents(args.project_root, args.config)
        baseline_config = resolve_documents(documents)
        generate_overrides(documents[-1], args.variants, args.seed)
        recording = read_recording(args.capture, args.project_root, args.evidence)
        from v3.capture_encoding import encode_value
        digest = motion._digest(encode_value(baseline_config.runtime.composition.live_control.control))
        recording["search_baseline_control_config_sha256"] = digest
        recording["search_baseline_relation"] = (
            "SAME_CAPTURED_CONTROL_CONFIG" if digest == recording["captured_control_config_sha256"]
            else "EXPLICIT_CURRENT_CONFIG; original recorded behavior is not the paired simulation baseline")
        suite, suite_lineage = read_suite(args.scenarios, recording)
        output = args.output.resolve() if args.output else Path(tempfile.mkdtemp(prefix="r2b4-autotune-"))
        if args.output:
            output.mkdir(parents=True, exist_ok=False)
        result = run_search(documents, suite, output, count=args.variants, seed=args.seed,
                            finalists=args.finalists, minimum_improvement=args.minimum_improvement, budget_s=args.budget_s)
        payload = {"schema": SCHEMA, "physical_output": False, "production_config_written": False,
                   "source_recording": recording, "configuration_documents": config_lineage,
                   "scenario_suite": suite, "scenario_lineage": suite_lineage,
                   "search": result, "live_validation": "NOT_PERFORMED",
                   "score_definition": {"roomcruise": "coverage_cells/active_s - 0.5*stopped_s/active_s - 0.02*angular_reversals/active_s",
                       "motion": "-(goal_error_m + cross_track_rms_m + tracking_rms_mps + 0.01*active_s)",
                       "followperson": "-person_standoff_error_rms_m",
                       "aggregation": "mean paired gain/max(1,abs(baseline_score)); no scene may regress"},
                   "limitations": ["Recorded DIAG metrics are prior observed evidence; synthetic scores do not measure improvement on the original physical run.",
                       "Explicit surrogate geometry, plant and synthetic sensors are assumptions; no physical calibration or exact counterfactual claim.",
                       "Training ranks variants; distinct held-out scenes gate the shortlist, without proving generalization beyond this suite.",
                       "Native deterministic inline completion does not measure process/GIL/IPC or live timing.",
                       "Shortlist needs user-authorized physical validation through canonical runtime; no automatic apply or motion."]}
        _write(output / "autotune.json", payload)
        print(json.dumps({"schema": SCHEMA, "status": result["status"], "result": str(output / "autotune.json"),
                          "tested_variants": result["tested_variants"], "shortlisted": len(result["shortlist"]), "physical_output": False}))
        return 0 if result["status"] == "COMPLETED" else 2
    except (ValueError, OSError, RuntimeError) as exc:
        print(json.dumps({"schema": SCHEMA, "status": "BLOCKED", "reason": str(exc), "physical_output": False}))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
