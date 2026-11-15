"""Serial paired comparisons, explicit overlays and repeated finite sessions."""
from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import time

from .evidence import digest, write
from .scenario import builtin, validate

SUITE = "R2B4_LOCAL_NAVIGATION_SUITE_V1"
OVERRIDE = "R2B4_LOCAL_NAVIGATION_OVERRIDE_V1"


def read_overlay(path):
    if path is None:
        return {}
    value = read_json(path)
    if not isinstance(value, dict) or value.get("schema") != OVERRIDE or set(value)-{"schema","vezerles","nav2","slam"}:
        raise ValueError("expected " + OVERRIDE)
    return {k:v for k,v in value.items() if k!="schema"}


def read_json(path):
    path = Path(path)
    if path.stat().st_size>1_048_576:
        raise ValueError("JSON input exceeds 1 MiB")
    return json.loads(path.read_bytes())


def read_suite(path):
    if path is None:
        return {"train":[builtin("room")],"holdout":[builtin("corridor")]}
    value=read_json(path)
    if not isinstance(value,dict) or value.get("schema")!=SUITE or set(value)!={"schema","train","holdout"}:
        raise ValueError("expected "+SUITE+" with train and holdout")
    result={}
    for name in ("train","holdout"):
        rows=value[name]
        if not isinstance(rows,list) or not 1<=len(rows)<=4:
            raise ValueError(name+" requires 1..4 scenarios")
        result[name]=[validate(row) for row in rows]
    def geometry(row):
        return digest({key:row[key] for key in ("room","obstacles","walls","moving_obstacles","plant","behavior","faults")})
    if {geometry(row) for row in result["train"]}&{geometry(row) for row in result["holdout"]}:
        raise ValueError("holdout requires distinct geometry, plant, behavior or fault schedule")
    return result


def variants(root,count=2,kind="nav2",seed=7):
    """Reuse the existing V3 search; ROS search varies real MPPI critic weights."""
    if type(count) is not int or not 1<=count<=16:
        raise ValueError("variants must be 1..16 on Pi")
    if kind=="v3":
        from tools.tuners.r2b4_navigation_autotuner import generate_overrides
        control=json.loads((root/"conf/vezerles.json").read_bytes())
        return [{"vezerles":patch} for patch in generate_overrides(control,count,seed)]
    from .runtime import read_parameters
    params=read_parameters(root,root/"conf/nav2_planner.yaml")
    follow=params["controller_server"]["ros__parameters"]["FollowPath"]
    # Keep CPU budgets, safety limits, geometry and freshness unchanged.
    axes=[(critic,factor) for critic in ("PathFollowCritic","GoalCritic","PathAlignCritic","GoalAngleCritic")
          for factor in (.8,1.2,.9,1.1)]
    return [{"nav2":{"controller_server":{"ros__parameters":{"FollowPath":{
        critic:{"cost_weight":follow[critic]["cost_weight"]*factor}}}}}} for critic,factor in axes[:count]]


def compare(before,after):
    blockers=[]
    if before.get("schema")!="R2B4_LOCAL_NAVIGATION_RESULT_V1" or after.get("schema")!="R2B4_LOCAL_NAVIGATION_RESULT_V1":
        raise ValueError("unsupported navigation result schema")
    for key in ("schema","backend","scenario_sha256"):
        if before.get(key)!=after.get(key):
            blockers.append("UNPAIRED:"+key)
    for label,result in (("baseline",before),("current",after)):
        if result.get("status")!="PASS" or result.get("final_stop") is not True or result.get("physical_output") is not False:
            blockers.append(label+":RUN_NOT_PASS")
        if result.get("replay",{}).get("status")!="MATCH":
            blockers.append(label+":NATIVE_REPLAY_NOT_MATCH")
    first,second=before.get("metrics",{}),after.get("metrics",{})
    # ROS scheduling is nondeterministic. These are reporting tolerances, never
    # production safety thresholds. Compare efficiency only after task success.
    tolerances={"goal_error_m":.02,"longest_stationary_s":.5,"minimum_clearance_m":.02,
                "max_localization_error_m":.02,"collision_count":0.,"active_duration_s":.5,
                "max_tick_wall_ms":2.,"path_length_m":.05}
    deltas={key:second[key]-first[key] for key in tolerances
            if type(first.get(key)) in {int,float} and type(second.get(key)) in {int,float}}
    regressed=[]
    improved=[]
    for key,delta in deltas.items():
        # RC path length measures exploration; goal navigation path measures
        # efficiency. Never reward a short failed route.
        higher_better=key=="minimum_clearance_m" or key=="path_length_m" and after["scenario"]["behavior"]=="roomcruise"
        if abs(delta)>tolerances[key]:
            (improved if (delta>0)==higher_better else regressed).append(key)
    if before.get("status")=="PASS" and after.get("status")!="PASS":
        status="REGRESSED"
    elif blockers:
        status="INCONCLUSIVE"
    elif regressed and improved:
        status="TRADEOFF"
    elif regressed:
        status="REGRESSED"
    elif improved:
        status="IMPROVED"
    else:
        status="UNCHANGED"
    return {"status":status,"blockers":blockers,"deltas_current_minus_baseline":deltas,
            "tolerances":tolerances,"regressed_metrics":regressed,"improved_metrics":improved,
            "first_failure":after.get("first_failure"),"source_changed":before.get("source_sha256")!=after.get("source_sha256"),
            "scope":"paired synthetic scene; ROS execution can vary; V3 closed-input replay is checked separately"}


def safe_evidence(result):
    import math
    behavior_failures = {"TRUTH_GOAL_NOT_REACHED", "EXCESSIVE_STATIONARY_INTERVAL", "INSUFFICIENT_PROGRESS"}
    metrics = result.get("metrics", {})
    duration = metrics.get("active_duration_s", 0.)
    complete = (result.get("termination") == "GOAL_COMPLETE" and metrics.get("goal_reached") is True
                or result.get("termination") == "DURATION_REACHED"
                and duration >= result.get("scenario", {}).get("duration_s", math.inf) - .021)
    return (result.get("physical_output") is False and result.get("final_stop") is True
            and result.get("virtual_body_stopped") is True and result.get("replay", {}).get("status") == "MATCH"
            and metrics.get("active_samples", 0) >= 1 and type(duration) in {int, float}
            and math.isfinite(duration) and duration > 0 and complete
            and all(type(v) not in {int, float} or math.isfinite(v) for v in metrics.values())
            and metrics.get("collision_count") == 0
            and set(result.get("failures", [])) <= behavior_failures)


def score(result):
    if not safe_evidence(result) or result["status"] != "PASS":
        return None
    metrics = result["metrics"]
    if result["scenario"]["behavior"] == "navigate":
        return -(metrics["goal_error_m"] + metrics["active_duration_s"] * .01 + metrics["path_length_m"] * .01)
    # Independent truth coverage, not distance driven around the same loop.
    return ((metrics["unique_coverage_cells"] - 1) / metrics["active_duration_s"]
            - .5 * metrics["longest_stationary_s"] / metrics["active_duration_s"])


def _assessment(baseline, measured, count):
    pairs = [compare(a, b) for a, b in zip(baseline, measured)]
    eligible = (len(measured) == count and all(safe_evidence(row) and row["status"] == "PASS" for row in measured)
                and all(not pair["regressed_metrics"] for base, pair in zip(baseline, pairs) if base["status"] == "PASS"))
    gains = [(score(trial) - score(base)) / max(1., abs(score(base)))
             for base, trial in zip(baseline, measured)
             if score(base) is not None and score(trial) is not None]
    fixed = sum(base["status"] != "PASS" and trial["status"] == "PASS" for base, trial in zip(baseline, measured))
    return {"eligible": eligible, "trials": measured, "pairs": pairs, "fixed_scenarios": fixed,
            "mean_score_gain": sum(gains) / len(gains) if gains else 0.,
            "numeric_gain_scope": "common PASS scenarios only; FAIL-to-PASS is a separate behavior result"}


def search(root, output, suite, patches, *, domain=75, budget_s=600., minimum_gain=.01):
    from .runtime import run
    started = time.monotonic()
    source_id = None
    baseline = {}
    tested = []
    reason = None
    best = None
    frozen_index = None
    validation = None

    def trials(scenes, target, patch=None):
        nonlocal source_id, reason
        rows = []
        for index, scene in enumerate(scenes):
            if time.monotonic() - started >= budget_s:
                reason = "BUDGET_EXHAUSTED"
                break
            row = run(root, target / str(index), scene, domain=domain, overlay=patch)
            rows.append(row)
            if source_id is None:
                source_id = row["source_sha256"]
            if source_id != row["source_sha256"]:
                reason = "SOURCE_CHANGED_DURING_SEARCH"
                break
            if not safe_evidence(row):
                reason = "EVIDENCE_OR_SAFETY_FAILURE"
                break
        return rows

    baseline["train"] = trials(suite["train"], output / "baseline" / "train")
    if reason is None:
        for index, patch in enumerate(patches):
            measured = trials(suite["train"], output / f"variant_{index}" / "train", patch)
            row = {"index": index, "overlay": patch,
                   **_assessment(baseline["train"], measured, len(suite["train"]))}
            tested.append(row)
            if reason is not None:
                break
    # Freeze one winner on training evidence. Holdout is evaluated once and
    # cannot select a replacement if that winner fails validation.
    if reason is None and len(tested) == len(patches):
        ranked = sorted((row for row in tested if row["eligible"]
                         and (row["fixed_scenarios"] > 0 or row["mean_score_gain"] > minimum_gain)),
                        key=lambda row: (-row["fixed_scenarios"], -row["mean_score_gain"], row["index"]))
        if ranked:
            chosen = ranked[0]
            frozen_index = chosen["index"]
            baseline["holdout"] = trials(suite["holdout"], output / "baseline" / "holdout")
            if reason is None:
                measured = trials(suite["holdout"], output / f"variant_{frozen_index}" / "holdout", chosen["overlay"])
                validation = _assessment(baseline["holdout"], measured, len(suite["holdout"]))
                if reason is None and validation["eligible"]:
                    best = chosen
    if best:
        write(output / "best_override.json", {"schema": OVERRIDE, **best["overlay"]})
    result = {"schema": "R2B4_LOCAL_NAVIGATION_AUTOTUNE_V1",
              "status": "COMPLETE" if reason is None else "INCOMPLETE", "reason": reason,
              "baseline": baseline, "variants": tested,
              "frozen_train_winner": frozen_index, "holdout_validation": validation,
              "best_index": None if best is None else best["index"],
              "automatic_application": False, "production_config_written": False, "physical_output": False,
              "elapsed_s": time.monotonic() - started,
              "limitations": ["One training winner is checked once on supplied independent holdout scenes.",
                              "Synthetic model and real ROS scheduling need independent repetition before physical conclusions."]}
    write(output / "autotune.json", result)
    return result
