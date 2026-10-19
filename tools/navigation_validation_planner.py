"""Recompute a few captured pure planner requests without robot or worker I/O.

Native replay consumes the recorded completion. This separate offline check
compares that completion to the current pure kernel on the exact request kept
by a checkpoint. Missing requests never become reconstructed synthetic input.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import time

from tools.diag.profiling import evidence_ref
from tools.mcap_evidence.reader import EvidenceBundle
from v3.capture_encoding import encode_value
from v3.contracts.planner import PlannerInput, TrajectoryRolloutRequest
from v3.layers.l6_navigation import TrajectoryRolloutComputer
from v3.replay import V3ReplayError, _decode_production_value, _production_control_config


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def evaluate_captured_planner_requests(evidence, root, maximum_requests=4, phases=()):
    """Compare at most four checkpoint requests with later closed completions.

    The caller owns source-MCAP verification and live run binding. ``phases``
    uses the acceptance tool's (command_id, start_ns, end_ns) intervals. Without
    phases the result applies to the supplied bundle, including synthetic EVI.
    """
    if type(maximum_requests) is not int or not 1 <= maximum_requests <= 4:
        raise ValueError("maximum_requests must be an integer within 1..4")
    bundle = EvidenceBundle.open(evidence) if isinstance(evidence, (str, Path)) else evidence
    report = {"status": "NOT_EXERCISED",
        "claim": "Captured pure planner requests produce the recorded worker result with unchanged measurement lineage.",
        "metrics": {"maximum_requests": maximum_requests, "samples": [], "matched_requests": 0,
                    "mismatches": 0, "missing_completions": 0,
                    "capture_sha256": bundle.manifest["source"]["sha256"]},
        "evidence_refs": [], "limits": [
            "Only exact checkpoint requests with later source-matched completions are compared.",
            "Current offline call wall/thread CPU time does not reconstruct original worker transport, queueing, GIL or scheduler timing.",
            "This does not prove physical accuracy or substitute for canonical native replay.",
            "The caller separately verifies source MCAP integrity and live skill-run binding."]}
    metrics = report["metrics"]
    def failure(error, row, pointer):
        report["status"] = "FAIL"
        metrics["error"] = f"{type(error).__name__}:{error}"[:500]
        report["evidence_refs"].append(evidence_ref(row, pointer).as_dict())
        return report

    if not all(bundle.has_view(name) for name in ("runtime", "checkpoints", "ticks")):
        report["limits"].append("Required runtime, checkpoint or closed tick evidence is absent.")
        return report
    runtime = list(bundle.iter_view("runtime", limit=2))
    if len(runtime) != 1:
        report["limits"].append("Exactly one captured runtime configuration is required.")
        return report
    try:
        config, config_scope = _production_control_config(runtime[0].payload, ())
    except (V3ReplayError, TypeError, ValueError) as exc:
        return failure(exc, runtime[0], "/configuration")
    metrics.update(config_scope=config_scope, control_config_sha256=_digest(encode_value(config)))
    metrics["clock_epoch"] = runtime[0].payload.get("clock_epoch")
    metrics["monotonic_boot_id"] = runtime[0].payload.get("metadata", {}).get("monotonic_boot_id")
    project_root = Path(root).resolve()
    source_files = (Path(__file__).resolve(), project_root / "v3/layers/l6_navigation.py",
                    project_root / "v3/contracts/planner.py", project_root / "v3/replay.py")
    metrics["current_source_sha256"] = {str(path.relative_to(project_root)) if path.is_relative_to(project_root)
        else str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in source_files}
    pending = {}
    for row in bundle.iter_view("checkpoints"):
        state = row.payload.get("state", {})
        encoded = state.get("navigation", {}).get("pending_rollout_request")
        if not isinstance(encoded, dict):
            continue
        try:
            request = _decode_production_value(encoded, TrajectoryRolloutRequest,
                "checkpoint.state.navigation.pending_rollout_request")
        except (V3ReplayError, TypeError, ValueError) as exc:
            return failure(exc, row, "/state/navigation/pending_rollout_request")
        context = request.context
        if phases and not any(start <= context.monotonic_ns <= end for _, start, end in phases):
            continue
        identity = (context.tick_id, context.monotonic_ns)
        if identity in pending:
            continue
        pending[identity] = (request, row)
        if len(pending) >= maximum_requests:
            break
    metrics["checkpoint_requests"] = len(pending)
    if not pending:
        report["limits"].append("No eligible immutable pending request was captured; no request was invented.")
        return report
    computer = TrajectoryRolloutComputer(config.navigation)
    for row in bundle.iter_view("ticks"):
        payload = row.payload
        planner = payload.get("inputs", {}).get("planner_input")
        if not isinstance(planner, dict) or not isinstance(planner.get("result"), dict):
            continue
        source = planner.get("request_context") or {}
        identity = (source.get("tick_id"), source.get("monotonic_ns"))
        if identity not in pending:
            continue
        request, checkpoint = pending[identity]
        command = payload.get("inputs", {}).get("command", {}).get("command_id")
        stamp = payload.get("monotonic_ns")
        if phases and not any(command == name and start <= request.context.monotonic_ns
                and type(stamp) is int and request.context.monotonic_ns < stamp <= end
                for name, start, end in phases):
            continue
        try:
            completion = _decode_production_value(planner, PlannerInput, "tick.inputs.planner_input")
            if (completion.context.tick_id != payload.get("tick_id")
                    or completion.context.monotonic_ns != stamp
                    or type(checkpoint.tick_id) is not int
                    or completion.context.tick_id <= checkpoint.tick_id):
                raise ValueError("Captured planner completion is not later than its checkpoint request")
            started = time.perf_counter_ns()
            cpu_started = time.thread_time_ns()
            actual = computer.compute(request)
        except (V3ReplayError, TypeError, ValueError, RuntimeError) as exc:
            return failure(exc, row, "/inputs/planner_input")
        compute_ns = time.perf_counter_ns() - started
        compute_cpu_ns = time.thread_time_ns() - cpu_started
        same = actual == completion.result
        sample = {"request_tick_id": request.context.tick_id,
            "request_measurement_monotonic_ns": request.context.monotonic_ns,
            "checkpoint_tick_id": checkpoint.tick_id,
            "completion_tick_id": completion.context.tick_id,
            "completion_measurement_monotonic_ns": completion.context.monotonic_ns,
            "command_id": command, "select_local_goal": request.select_local_goal,
            "trajectory_candidate_count": len(actual.trajectory_candidates),
            "proposed_goal_count": len(actual.goal_options),
            "world_frame_id": request.world.frame_id,
            "localization_generation": request.estimate.localization_quality.generation,
            "world_geometry_revision": request.world.local_costmap.revision if request.world.local_costmap else None,
            "geometry_source_sequence": request.world.local_costmap.source_sequence if request.world.local_costmap else None,
            "request_sha256": _digest(encode_value(request)),
            "recorded_result_sha256": _digest(encode_value(completion.result)),
            "recomputed_result_sha256": _digest(encode_value(actual)),
            "measurement_lineage_matches": actual.source_context == completion.request_context == request.context,
            "exact_result_matches": same, "pure_compute_elapsed_ns": compute_ns,
            "pure_compute_thread_cpu_ns": compute_cpu_ns}
        metrics["samples"].append(sample)
        metrics["matched_requests"] += 1
        metrics["mismatches"] += not (same and sample["measurement_lineage_matches"])
        report["evidence_refs"].extend((
            evidence_ref(checkpoint, "/state/navigation/pending_rollout_request").as_dict(),
            evidence_ref(row, "/inputs/planner_input/result").as_dict()))
        del pending[identity]
        if not pending:
            break
    metrics["missing_completions"] = len(pending)
    metrics["pure_compute_elapsed_ns_total"] = sum(sample["pure_compute_elapsed_ns"] for sample in metrics["samples"])
    metrics["pure_compute_thread_cpu_ns_total"] = sum(sample["pure_compute_thread_cpu_ns"] for sample in metrics["samples"])
    metrics["goal_search_requests_compared"] = sum(sample["select_local_goal"] for sample in metrics["samples"])
    report["status"] = ("FAIL" if metrics["mismatches"] else "PASS" if metrics["matched_requests"]
                        and not pending else "NOT_EXERCISED")
    if pending:
        report["limits"].append("Some captured pending requests have no later phase-bound completion in the supplied evidence.")
    return report
