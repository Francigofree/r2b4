"""Headless motorless navigation experiments using current production source."""
from __future__ import annotations

import argparse
import inspect
import json
from pathlib import Path
import tempfile
import subprocess

from .evidence import analyze, replay, source_snapshot, write
from .scenario import BUILTINS, builtin, validate
from .experiments import read_json,read_overlay,read_suite,variants,compare,search

ROOT=Path(__file__).resolve().parents[2]


def parser():
    result=argparse.ArgumentParser(prog="r sim",description=__doc__)
    commands=result.add_subparsers(dest="mode",required=True)
    doctor=commands.add_parser("doctor",help="check current source interfaces and installed dependencies without starting processes")
    doctor.add_argument("--output",type=Path)
    for name in ("analyze","replay","scenario"):
        child=commands.add_parser(name)
        child.add_argument("capture",type=Path)
        child.add_argument("--output",type=Path)
        if name=="analyze":
            child.add_argument("--summary-only",action="store_true",help="stream native timeline without compiling DIAG reports")
    for name in ("simulate","regression","stress"):
        child=commands.add_parser(name,aliases=["fault"] if name=="simulate" else [])
        child.add_argument("--scenario",help="built-in name or scenario JSON; regression defaults to recorded baseline scene")
        child.add_argument("--duration-s",type=float)
        child.add_argument("--override",type=Path)
        child.add_argument("--ros-domain-id",type=int,default=75)
        child.add_argument("--output",type=Path)
        if name=="simulate":
            child.add_argument("--no-replay",action="store_true")
        if name=="regression":
            child.add_argument("--baseline",type=Path,required=True,help="previous result.json, same scenario; code may differ")
        if name=="stress":
            child.add_argument("--repeat",type=int,default=10,help="finite serial sessions, stop on the first failure (1..1000)")
    child=commands.add_parser("autotune")
    child.add_argument("--suite",type=Path,help="explicit train/holdout scenes; default room/corridor")
    child.add_argument("--variants",type=int,default=2)
    child.add_argument("--search",choices=("nav2","v3"),default="nav2")
    child.add_argument("--seed",type=int,default=7)
    child.add_argument("--overlays",type=Path,help="JSON array of explicit override documents")
    child.add_argument("--budget-s",type=float,default=600.)
    child.add_argument("--ros-domain-id",type=int,default=75)
    child.add_argument("--output",type=Path)
    return result


def doctor(root):
    from v3.composition.native_control import NativeControlComposition
    from v3.adapters.slam_toolbox_transport import SlamToolboxSender,SlamLocalizationReceiver
    from tools.nav2_planner import _resolved_config,_paths
    resolved=_resolved_config(root)
    paths=_paths(root)
    required=[paths["prefix"]/"bin/python",paths["overlay"]/"lib/libr2b4_nav2_wheel_adapter.so",
              paths["frontier"]/"lib/explore_lite/explore",root/"runtime/deps/slam-toolbox/bin/python",
              root/"runtime/deps/slam-toolbox/lib/slam_toolbox/async_slam_toolbox_node"]
    required.extend(paths["prefix"]/"lib"/package/executable for package,executable in (
        ("nav2_planner","planner_server"),("nav2_controller","controller_server"),
        ("nav2_behaviors","behavior_server"),("nav2_bt_navigator","bt_navigator"),
        ("nav2_lifecycle_manager","lifecycle_manager")))
    missing=[str(path) for path in required if not path.is_file()]
    interfaces={"navigation_backend" in inspect.signature(NativeControlComposition).parameters,
                callable(getattr(SlamToolboxSender,"publish_scan",None)),callable(getattr(SlamLocalizationReceiver,"accept",None))}
    full=resolved.runtime.composition.live_control.control.navigation.full_nav2_enabled
    return {"schema":"R2B4_LOCAL_NAVIGATION_DOCTOR_V1","status":"PASS" if not missing and interfaces=={True} and full else "BLOCKED",
            "physical_output":False,"full_nav2_enabled":full,"missing_dependencies":missing,"interfaces_supported":interfaces=={True},
            "source":source_snapshot(root),"builtins":list(BUILTINS),"modes":["analyze","replay","scenario","simulate","autotune","regression","stress"],
            "scope":"installation and current interface check; no ROS, robot runtime or motor process started"}


def main(argv=None,*,project_root=None):
    args=parser().parse_args(argv)
    root=Path(project_root or ROOT).resolve()
    output=None
    try:
        if args.mode=="doctor":
            result=doctor(root)
            if args.output:
                write(args.output,result)
            print(json.dumps(result,sort_keys=True))
            return 0 if result["status"]=="PASS" else 2
        (root/"tmp").mkdir(exist_ok=True)
        output=args.output.resolve() if args.output else Path(tempfile.mkdtemp(prefix="nav_sim_",dir=root/"tmp"))
        if args.output:
            output.mkdir(parents=True,exist_ok=False)
        if args.mode=="analyze":
            result=analyze(args.capture,root,output,compile_reports=not args.summary_only)
        elif args.mode=="replay":
            result=replay(args.capture,root,output/"replay.json")
        elif args.mode=="scenario":
            from .evidence import scenario_from_mcap
            scenario=scenario_from_mcap(args.capture)
            write(output/"scenario.json",scenario)
            result={"status":"PASS","scenario":str(output/"scenario.json"),"physical_output":False}
        elif args.mode=="autotune":
            import math
            if not math.isfinite(args.budget_s) or not 1<=args.budget_s<=86400:
                raise ValueError("budget-s must be 1..86400")
            if args.overlays:
                documents=read_json(args.overlays)
                from .experiments import OVERRIDE
                if not isinstance(documents,list) or not 1<=len(documents)<=16:
                    raise ValueError("overlays requires 1..16 explicit overrides")
                for document in documents:
                    if not isinstance(document,dict) or document.get("schema")!=OVERRIDE or set(document)-{"schema","vezerles","nav2","slam"}:
                        raise ValueError("invalid override document")
                patches=[{k:v for k,v in document.items() if k!="schema"} for document in documents]
            else:
                patches=variants(root,args.variants,args.search,args.seed)
            result=search(root,output,read_suite(args.suite),patches,domain=args.ros_domain_id,budget_s=args.budget_s)
        else:
            from .runtime import run
            baseline=read_json(args.baseline) if args.mode=="regression" else None
            selected=args.scenario or (None if baseline else "room")
            scenario=(validate(baseline["scenario"]) if selected is None else
                      builtin(selected) if selected in BUILTINS else validate(read_json(selected)))
            if args.duration_s is not None:
                scenario=validate({**scenario,"duration_s":args.duration_s})
            overlay=read_overlay(args.override)
            if args.mode=="stress":
                if not 1<=args.repeat<=1000:
                    raise ValueError("repeat must be 1..1000")
                results=[]
                for index in range(args.repeat):
                    trial=run(root,output/f"run_{index:04d}",scenario,domain=args.ros_domain_id,overlay=overlay)
                    results.append({"index":index,"status":trial["status"],"metrics":trial["metrics"],"failures":trial["failures"]})
                    write(output/"stress.json",{"status":"RUNNING","runs":results,"physical_output":False})
                    if trial["status"]!="PASS":
                        break
                result={"status":"PASS" if len(results)==args.repeat and all(row["status"]=="PASS" for row in results) else "FAIL",
                        "requested":args.repeat,"completed":len(results),"runs":results,"physical_output":False}
                write(output/"stress.json",result)
            else:
                result=run(root,output/"run",scenario,domain=args.ros_domain_id,overlay=overlay,
                           replay=not getattr(args,"no_replay",False))
                if args.mode=="regression":
                    comparison=compare(baseline,result)
                    result={"status":"PASS" if comparison["status"] in {"IMPROVED","UNCHANGED"} else "FAIL",**{"comparison":comparison},
                            "run":str(output/"run/result.json"),"physical_output":False}
                    write(output/"regression.json",result)
        print(json.dumps({"status":result["status"],"output":str(output),"physical_output":False,
                          "failures":result.get("failures",[]),"reason":result.get("reason")},sort_keys=True))
        return 0 if result["status"] in {"PASS","MATCH","COMPLETE"} else 2
    except (OSError,ValueError,RuntimeError,TypeError,AttributeError,subprocess.SubprocessError) as exc:
        result={"status":"BLOCKED","reason":str(exc),"physical_output":False}
        if output:
            write(output/"error.json",result)
        print(json.dumps(result,sort_keys=True))
        return 2


if __name__=="__main__":
    raise SystemExit(main())
