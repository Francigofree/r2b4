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


def score(result):
    if result["status"]!="PASS" or result["replay"]["status"]!="MATCH" or not result["metrics"]["active_samples"]:
        return None
    m=result["metrics"]
    if result["scenario"]["behavior"]=="navigate":
        return -(m["goal_error_m"]+m["active_duration_s"]*.01+m["path_length_m"]*.01)
    return m["path_length_m"]/max(.02,m["active_duration_s"])-m["longest_stationary_s"]*.01


def search(root,output,suite,patches,*,domain=75,budget_s=600.,minimum_gain=.01):
    from .runtime import run
    started=time.monotonic()
    baseline={}
    tested=[]
    best=None
    # Baseline has the same finite conditions as every overlay; no recording
    # is treated as counterfactual sensor data.
    for group,scenes in suite.items():
        baseline[group]=[run(root,output/"baseline"/f"{group}_{i}",scene,domain=domain) for i,scene in enumerate(scenes)]
    source_ids={trial["source_sha256"] for rows in baseline.values() for trial in rows}
    if len(source_ids)!=1 or any(score(trial) is None for rows in baseline.values() for trial in rows):
        result={"status":"BLOCKED","reason":"BASELINE_NOT_COMPLETE_SAFE_PASS","baseline":baseline,"variants":[]}
        write(output/"autotune.json",result)
        return result
    for index,patch in enumerate(patches):
        if time.monotonic()-started>=budget_s:
            break
        row={"index":index,"overlay":patch,"groups":{},"eligible":True}
        for group,scenes in suite.items():
            results=[]
            for i,scene in enumerate(scenes):
                measured=run(root,output/f"variant_{index}"/f"{group}_{i}",scene,domain=domain,overlay=patch)
                results.append(measured)
                if measured["source_sha256"] not in source_ids:
                    row["eligible"]=False
            pairs=[compare(a,b) for a,b in zip(baseline[group],results)]
            row["groups"][group]={"trials":results,"pairs":pairs}
            if any(pair["blockers"] or pair["regressed_metrics"] for pair in pairs):
                row["eligible"]=False
                break
        gains=[score(trial)-score(base) for group,data in row["groups"].items()
               for base,trial in zip(baseline[group],data["trials"]) if score(trial) is not None]
        row["mean_score_gain"]=sum(gains)/len(gains) if row["eligible"] and gains else None
        tested.append(row)
        if row["eligible"] and row["mean_score_gain"]>minimum_gain and (best is None or row["mean_score_gain"]>best["mean_score_gain"]):
            best=row
        # A fault/process death is evidence to inspect, not a trigger for an
        # automatic retry or further tuning on an unhealthy stack.
        if any(trial["status"]!="PASS" for data in row["groups"].values() for trial in data["trials"]):
            break
    if best:
        write(output/"best_override.json",{"schema":OVERRIDE,**best["overlay"]})
    result={"schema":"R2B4_LOCAL_NAVIGATION_AUTOTUNE_V1","status":"COMPLETE" if len(tested)==len(patches) else "INCOMPLETE",
            "baseline":baseline,"variants":tested,"best_index":None if best is None else best["index"],
            "automatic_application":False,"production_config_written":False,"physical_output":False,
            "elapsed_s":time.monotonic()-started,"limitations":["Train and holdout are supplied synthetic environments, not physical calibration.",
                "Real ROS scheduling needs repeated measurements before treating small score gains as stable."]}
    write(output/"autotune.json",result)
    return result
