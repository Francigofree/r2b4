"""Explicit offline acceptance measurements for the navigation validation skill.

The sealed EVI reader, strict capture reader and canonical replay verifier own
their formats. This consumer never decodes MCAP messages itself and never starts
the robot. PASS applies only to the named, observed relationship; missing branch
coverage remains NOT_EXERCISED and physical ground truth remains unmeasured.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from tools.diag.profiling import evidence_ref
from tools.diag.tick_evidence import command_ingress_summary
from tools.mcap_evidence.reader import EvidenceBundle
from tools.mcap_evidence.verify import verify
from v3.mcap_reader import McapReader, McapReadError
from v3.replay import verify_replay_result


SCHEMA = "R2B4_NAVIGATION_LIVE_ACCEPTANCE_V1"
_MAX_EXAMPLES = 8


def _dict(value):
    return value if isinstance(value, dict) else {}


def _digest(value):
    # The public runtime uses these exact JSON settings for result_sha256.
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def _feature(status, claim, *, metrics=None, refs=(), limits=()):
    return {"status": status, "claim": claim, "metrics": metrics or {},
            "evidence_refs": list(refs)[:_MAX_EXAMPLES], "limits": list(limits)}


def _skill(value):
    """Accept the public skill.status response or its complete result object."""
    outer = _dict(value)
    result = _dict(outer.get("result")) if "run_id" in outer and isinstance(outer.get("result"), dict) else outer
    identity = {key: outer.get(key, result.get(key)) for key in ("run_id", "source_hash", "started_ns", "finished_ns")}
    identity["started_ns"] = identity["started_ns"] or result.get("skill_started_ns")
    return outer, result, identity


def _phases(skill):
    phases = skill.get("phases", [])
    if not isinstance(phases, list) or len(phases) > 32:
        raise ValueError("skill phases must contain at most 32 intervals")
    result = []
    for phase in phases:
        phase = _dict(phase)
        command, start, end = (phase.get(key) for key in ("command_id", "start_monotonic_ns", "end_monotonic_ns"))
        if (isinstance(command, str) and command and type(start) is int and type(end) is int and 0 <= start < end):
            result.append((command, start, end))
    return tuple(result)


def _in_phase(payload, phases):
    stamp = payload.get("monotonic_ns")
    if type(stamp) is not int:
        return False
    command = _dict(_dict(payload.get("inputs")).get("command")).get("command_id")
    return any(command == identity and start <= stamp <= end for identity, start, end in phases)


class _PhaseTicks:
    """Small adapter that keeps the canonical ingress measurement run scoped."""
    def __init__(self, bundle, phases):
        self.bundle, self.phases = bundle, phases

    def has_view(self, name):
        return self.bundle.has_view(name)

    def iter_view(self, name):
        for row in self.bundle.iter_view(name):
            if _in_phase(_dict(row.payload), self.phases):
                yield row


def _binding(bundle, supplied):
    outer, skill, expected = _skill(supplied)
    epoch = skill.get("clock_epoch")
    runtime = list(bundle.iter_view("runtime", limit=2)) if bundle.has_view("runtime") else []
    metadata = _dict(_dict(runtime[0].payload).get("metadata")) if len(runtime) == 1 else {}
    physical = (metadata.get("runtime") == "v3_process_runtime"
                and metadata.get("command_gateway") == "AtomicResidentCommandGateway"
                and metadata.get("physical_hardware") is not False
                and _dict(runtime[0].payload).get("clock_epoch") == "linux_monotonic_boot") if runtime else False
    clock_ok = isinstance(epoch, str) and epoch and epoch == metadata.get("monotonic_boot_id")
    terminal, conflicts = [], []
    for row in bundle.iter_view("events") if bundle.has_view("events") else ():
        envelope = _dict(row.payload)
        event = _dict(envelope.get("payload"))
        value = _dict(event.get("value"))
        if (envelope.get("source_topic") != "v3.public_runtime_event" or event.get("kind") != "skill"
                or not expected.get("run_id") or value.get("run_id") != expected["run_id"]
                or value.get("event_type") != "skill_terminal"):
            continue
        checks = (value.get("source_hash") == expected.get("source_hash"),
                  value.get("result_sha256") == _digest(skill), event.get("clock_epoch") == epoch,
                  all(expected.get(key) is None or value.get(key) == expected[key]
                      for key in ("started_ns", "finished_ns")))
        ref = evidence_ref(row, "/payload/value").as_dict()
        if all(checks):
            terminal.append({"state": value.get("state"), "cleanup_status": value.get("cleanup_status"),
                             "started_ns": value.get("started_ns"), "finished_ns": value.get("finished_ns"),
                             "evidence_ref": ref})
        else:
            conflicts.append(ref)
    status = "PASS" if terminal and clock_ok and physical and not conflicts else "FAIL" if conflicts else "NOT_EXERCISED"
    if terminal and any(item["cleanup_status"] == "FAILED" for item in terminal):
        status = "FAIL"
    return _feature(status, "Live runtime clock, terminal run/source identity and complete skill-result hash agree.",
                    metrics={"expected": expected, "clock_epoch": epoch, "runtime_boot_id": metadata.get("monotonic_boot_id"),
                             "physical_runtime": physical, "clock_matches": bool(clock_ok),
                             "terminal_events": terminal, "conflicting_terminal_count": len(conflicts)},
                    refs=[item["evidence_ref"] for item in terminal] + conflicts,
                    limits=("Program completion does not prove the physical goal.",))


def _publication(bundle, phases):
    scoped = _PhaseTicks(bundle, phases)
    metrics = command_ingress_summary(scoped)
    counts = metrics["counts"]
    ttl_values = set()
    for row in scoped.iter_view("ticks"):
        for item in _dict(row.payload).get("tick_evidence", ()):
            if (isinstance(item, dict) and item.get("__type__") == "CommandIngressTiming"
                    and type(item.get("expires_monotonic_ns")) is int and type(item.get("issued_monotonic_ns")) is int):
                ttl_values.add(item["expires_monotonic_ns"] - item["issued_monotonic_ns"])
    metrics["observed_ttl_ns"] = sorted(ttl_values)
    runtime = next(bundle.iter_view("runtime"), None) if bundle.has_view("runtime") else None
    resolved = _dict(_dict(_dict(runtime.payload).get("configuration")).get("resolved_robot")) if runtime else {}
    maximum_ttl = _dict(_dict(resolved.get("edges")).get("command_ingress")).get("maximum_ttl_ns")
    metrics["configured_maximum_ttl_ns"] = maximum_ttl
    metrics["ttl_matches_captured_configuration"] = ttl_values == {maximum_ttl} if type(maximum_ttl) is int else None
    failed = any(counts.get(key, 0) for key in ("continuing_motion_command_issue_ttl_gaps", "expired_at_reader_samples",
                                               "expired_at_observation_samples", "invalid", "revision_or_issue_order_breaks"))
    failed |= bool(ttl_values) and type(maximum_ttl) is int and ttl_values != {maximum_ttl}
    exercised = counts.get("adjacent_revision_pairs", 0) > 0 and counts.get("previous_publication_bounds", 0) > 0
    status = "FAIL" if failed else "PASS" if exercised and counts.get("unobserved_revisions", 0) == 0 else "NOT_EXERCISED"
    refs = list(metrics.get("first_evidence_refs", {}).values())
    return _feature(status, "Observed consecutive publications of phase command IDs cover the previous TTL and retain post-replace timing bounds.",
                    metrics=metrics, refs=refs, limits=("No producer scheduler/GIL cause is inferred.",
                    "The final publication has no next-revision return timestamp.",
                    "Command revisions missing from the capture cannot be reconstructed."))


def _tick_measurements(bundle, phases):
    counts, refs, statuses, hold_reasons = Counter(), {}, Counter(), Counter()
    first_failure = None
    previous = None
    checkpoints = iter(bundle.iter_view("checkpoints")) if bundle.has_view("checkpoints") else iter(())
    checkpoint = next(checkpoints, None)
    # Decision rows use source time. Advance a single cursor alongside ticks.
    def decisions():
        for row in bundle.iter_view("events") if bundle.has_view("events") else ():
            envelope = _dict(row.payload)
            payload = _dict(envelope.get("payload"))
            if envelope.get("source_topic") == "v3.navigation_decision":
                counts["navigation_decision_events"] += 1
                yield row, payload
    events = iter(decisions())
    event = next(events, None)
    for row in bundle.iter_view("ticks") if bundle.has_view("ticks") else ():
        payload = _dict(row.payload)
        stamp = payload.get("monotonic_ns")
        if type(stamp) is not int:
            continue
        while event is not None and event[1].get("monotonic_ns", -1) < stamp:
            event = next(events, None)
        while checkpoint is not None and checkpoint.log_time_ns < stamp:
            checkpoint = next(checkpoints, None)
        if not _in_phase(payload, phases):
            previous = None
            continue
        counts["phase_ticks"] += 1
        layers = _dict(_dict(payload.get("expected")).get("layers"))
        plan = _dict(layers.get("L6"))
        l7, l8, l12 = (_dict(layers.get(name)) for name in ("L7", "L8", "L12"))
        statuses[str(plan.get("status"))] += 1
        context = _dict(_dict(payload.get("inputs")).get("context"))
        if context.get("tick_id") != row.tick_id or context.get("monotonic_ns") != stamp:
            counts["invalid_tick_context"] += 1
            refs.setdefault("invalid_tick_context", evidence_ref(row, "/inputs/context").as_dict())
            previous = None
            continue
        planner = _dict(_dict(payload.get("inputs")).get("planner_input"))
        completion = _dict(planner.get("result"))
        if completion:
            counts["planner_completions"] += 1
            options = completion.get("goal_options")
            if isinstance(options, list) and options:
                counts["worker_goal_search_results"] += 1
                valid = (len(options) <= 4 and completion.get("selected_goal") in options
                         and completion.get("source_context") == planner.get("request_context"))
                counts["invalid_worker_goal_search_results"] += not valid
                if not valid and first_failure is None:
                    first_failure = {"tick_id": row.tick_id, "layer": "L6", "field": "inputs.planner_input.result",
                                     "value": {"goal_options_count": len(options), "selected_goal": completion.get("selected_goal"),
                                               "request_context": planner.get("request_context"), "source_context": completion.get("source_context")},
                                     "evidence_ref": evidence_ref(row, "/inputs/planner_input/result").as_dict()}
                accepted = valid and plan.get("local_goal") == completion.get("selected_goal") and plan.get("status") == "ACTIVE"
                counts["accepted_worker_goal_search_results"] += accepted
                refs.setdefault("accepted_search" if accepted else "search_result", evidence_ref(row, "/inputs/planner_input/result").as_dict())
        witnesses = [(plan.get("rejection_witness"), "/expected/layers/L6/rejection_witness")]
        candidates = plan.get("trajectory_candidates", [])
        if isinstance(candidates, list):
            witnesses.extend((_dict(candidate).get("rejection_witness"),
                              f"/expected/layers/L6/trajectory_candidates/{index}/rejection_witness")
                             for index, candidate in enumerate(candidates))
        for witness, pointer in witnesses:
            if not isinstance(witness, dict):
                continue
            counts["geometric_rejection_witnesses"] += 1
            measured = _dict(witness.get("source_context")).get("monotonic_ns")
            geometry_time = witness.get("geometry_captured_ns")
            valid = (isinstance(witness.get("candidate_id"), str) and isinstance(witness.get("reason"), str)
                     and type(measured) is int and type(geometry_time) is int and 0 <= geometry_time <= measured <= stamp
                     and all(type(witness.get(key)) is int and witness[key] >= 0
                             for key in ("geometry_revision", "map_revision", "time_offset_ns"))
                     and all(type(witness.get(key)) in (int, float) and math.isfinite(witness[key]) and witness[key] >= 0
                             for key in ("clearance_m", "required_clearance_m", "uncertainty_m")))
            counts["invalid_geometric_rejection_witnesses"] += not valid
            refs.setdefault("geometric_rejection", evidence_ref(row, pointer).as_dict())
        if checkpoint is not None and checkpoint.log_time_ns == stamp:
            state = _dict(_dict(checkpoint.payload).get("state"))
            navigation = _dict(state.get("navigation"))
            pending = _dict(navigation.get("pending_rollout_request"))
            if pending.get("select_local_goal") is True:
                counts["pending_search_checkpoints"] += 1
                refs.setdefault("pending_search", evidence_ref(checkpoint, "/state/navigation/pending_rollout_request").as_dict())
            committed = navigation.get("local_goal")
            if committed is not None and pending and pending.get("goal") != committed:
                counts["different_pending_and_committed_goal_checkpoints"] += 1
                # Compare explicit committed state and published goal, never the pending proposal.
                retained = plan.get("local_goal") == committed
                counts["committed_goal_retained_under_pending"] += retained
                counts["committed_goal_replaced_by_pending"] += (not retained and plan.get("local_goal") == pending.get("goal"))
                refs.setdefault("pending_overlap", evidence_ref(checkpoint, "/state/navigation").as_dict())
        reason = plan.get("reason")
        if isinstance(reason, str) and reason.startswith(("NO_PATH_", "NO_PROGRESS_")) and reason.endswith("HOLD"):
            hold_reasons[reason] += 1
            counts["bounded_hold_ticks"] += 1
            zero = (l8.get("requested_v_mps") == 0 and l8.get("requested_omega_rad_s") == 0
                    and l12.get("left_output") == 0 and l12.get("right_output") == 0)
            counts["bounded_hold_nonzero_output_ticks"] += not zero
            if not zero and first_failure is None:
                first_failure = {"tick_id": row.tick_id, "layer": "L8/L12", "field": "HOLD motion outputs",
                                 "value": {"reason": reason, "requested_v_mps": l8.get("requested_v_mps"),
                                           "requested_omega_rad_s": l8.get("requested_omega_rad_s"),
                                           "left_output": l12.get("left_output"), "right_output": l12.get("right_output")},
                                 "evidence_ref": evidence_ref(row, "/expected/layers").as_dict()}
            refs.setdefault(reason, evidence_ref(row, "/expected/layers/L6/reason").as_dict())
        proof = _dict(plan.get("motion_validity")).get("source_context")
        if (previous is not None and previous[0] == (plan.get("status"), reason)
                and plan.get("status") == "ACTIVE" and reason == "LOCAL_GUIDANCE_REVALIDATED"
                and proof is not None and proof != previous[1]):
            counts["local_revalidation_source_refresh_pairs"] += 1
            refs.setdefault("proof_refresh_pair", {"previous": previous[2],
                "next": evidence_ref(row, "/expected/layers/L6/motion_validity/source_context").as_dict()})
            if event is not None and event[1].get("monotonic_ns") == stamp and "plan" in event[1].get("changes", []):
                counts["source_refresh_plan_events"] += 1
                refs.setdefault("source_refresh_plan_event", evidence_ref(event[0], "/payload/changes").as_dict())
        previous = ((plan.get("status"), reason), proof,
                    evidence_ref(row, "/expected/layers/L6/motion_validity/source_context").as_dict())
        if l12.get("safety_decision") == "FAULT" or _dict(payload.get("expected")).get("fault_layer"):
            counts["fault_ticks"] += 1
            refs.setdefault("fault", evidence_ref(row, "/expected").as_dict())
            if first_failure is None:
                first_failure = {"tick_id": row.tick_id, "layer": _dict(payload.get("expected")).get("fault_layer") or "L12",
                                 "field": "safety_decision", "value": l12.get("safety_decision"),
                                 "evidence_ref": refs["fault"]}
        trajectory = _dict(l7.get("trajectory"))
        if trajectory and l7.get("selected_source") == "navigation.trajectory":
            counts["selected_trajectory_ticks"] += 1
    common = {"counts": dict(counts), "navigation_status_counts": dict(statuses), "bounded_hold_reason_counts": dict(hold_reasons),
              "first_recorded_failure": first_failure}
    def status(exercised, failure=0):
        return "FAIL" if failure else "PASS" if exercised else "NOT_EXERCISED"
    return {
        "worker_goal_search": _feature(status(counts["accepted_worker_goal_search_results"], counts["invalid_worker_goal_search_results"]),
            "Captured closed worker goal options are bounded, source-linked and accepted by L6.", metrics=common,
            refs=[value for name, value in refs.items() if name in {"pending_search", "accepted_search", "search_result"}],
            limits=("Completion evidence does not by itself measure worker CPU or scheduling latency.",)),
        "committed_goal_during_pending": _feature(status(counts["committed_goal_retained_under_pending"], counts["committed_goal_replaced_by_pending"]),
            "At observed checkpoints, a different pending proposal does not replace the published committed goal.", metrics=common,
            refs=[refs["pending_overlap"]] if "pending_overlap" in refs else (),
            limits=("Checkpoints without a different committed/pending target do not exercise this case.",)),
        "bounded_no_path_no_progress": _feature(status(counts["bounded_hold_ticks"], counts["bounded_hold_nonzero_output_ticks"]),
            "Observed NO_PATH/NO_PROGRESS HOLD decisions propagate to zero requested motion and motor output.", metrics=common,
            refs=[value for name, value in refs.items() if name.startswith(("NO_PATH_", "NO_PROGRESS_"))],
            limits=("This checks observed HOLD decisions; retry exhaustion and every watchdog branch need their own exercised inputs.",)),
        "geometric_rejection_evidence": _feature(status(counts["geometric_rejection_witnesses"], counts["invalid_geometric_rejection_witnesses"]),
            "Observed geometric rejection witnesses retain candidate, threshold, uncertainty and source geometry time/revision.", metrics=common,
            refs=[refs["geometric_rejection"]] if "geometric_rejection" in refs else (),
            limits=("An explicit recorded rejection does not prove a physical obstacle or the accuracy of its measured geometry.",)),
        "recorder_semantic_transitions": _feature(status(counts["local_revalidation_source_refresh_pairs"]
                and counts["navigation_decision_events"], counts["source_refresh_plan_events"]),
            "Changing only the source proof time of an unchanged local-revalidation status/reason produces no plan-change event.", metrics=common,
            refs=[value for name, value in refs.items() if name in {"source_refresh_plan_event", "proof_refresh_pair"}],
            limits=("Goal, selection, safety and other legitimate change events remain allowed.",)),
        "observed_faults": _feature("FAIL" if counts["fault_ticks"] or counts["invalid_tick_context"]
            else "PASS" if counts["phase_ticks"] else "NOT_EXERCISED",
            "No captured phase tick contains an explicit runtime fault.", metrics={"phase_ticks": counts["phase_ticks"], "fault_ticks": counts["fault_ticks"]},
            refs=[refs["fault"]] if "fault" in refs else (), limits=("Absence of FAULT is not a navigation-success verdict.",)),
    }


def _spatial(skill, binding_ok):
    before, after = (_dict(skill.get(name)) for name in ("spatial_before", "spatial_after"))
    coverage = after.get("coverage", [])
    frontiers = after.get("frontier_indexes", [])
    coverage = coverage if isinstance(coverage, list) else []
    frontiers = frontiers if isinstance(frontiers, list) else []
    unsafe = any(_dict(row).get("motion_authority") is not False for row in coverage + frontiers)
    stats = {"before_coverage_count": len(before.get("coverage", [])), "after_coverage_count": len(coverage),
             "frontier_index_count": len(frontiers), "coverage_truncated": after.get("coverage_truncated"),
             "frontiers_truncated": after.get("frontiers_truncated"),
             "source": "TERMINAL_SKILL_RESULT_HASH" if binding_ok else "UNBOUND_SUPPLIED_SKILL_RESULT"}
    indexes_valid = all(_dict(row).get("current_alignment") is None and len(_dict(row).get("candidates", [])) <= 64
                        and isinstance(_dict(row).get("occupancy_sha256"), str) for row in frontiers)
    return {
        "spatial_history_observation": _feature("FAIL" if binding_ok and unsafe else "PASS" if binding_ok and coverage else "NOT_EXERCISED",
            "The source-bound skill observed historical coverage without positive motion authority.", metrics=stats,
            limits=("A before/after read in one host session does not prove persistence across a host restart.",)),
        "atlas_frontier_observation": _feature("FAIL" if binding_ok and frontiers and (unsafe or not indexes_valid)
            else "PASS" if binding_ok and frontiers else "NOT_EXERCISED",
            "Observed bounded atlas frontier indexes retain asset identity and no current alignment/motion authority.", metrics=stats,
            limits=("Historical frontiers do not establish robot-to-atlas alignment or validate physical frontier locations.",)),
        "spatial_restart_persistence": _feature("NOT_EXERCISED", "Persistence requires a separately recorded host restart and restored source identities."),
        "strategic_explore": _feature("PASS" if binding_ok and any(_dict(p).get("name") == "strategic_explore"
            and _dict(p).get("lifecycle") == "COMPLETED" for p in skill.get("phases", [])) else "NOT_EXERCISED",
            "A source-bound strategic_explore phase completed through the canonical behavior lifecycle.",
            limits=("No current qualified place coordinates means no strategy motion test; historical atlas coordinates cannot substitute.",)),
    }


def _diagnostics(supplied, source_hash):
    raw = supplied or {}
    candidates = raw.get("results", raw.get("analyzers", raw)) if isinstance(raw, dict) else raw
    if isinstance(candidates, dict):
        candidates = list(candidates.values())
    if isinstance(raw, dict) and "analyzer_id" in raw:
        candidates = [raw]
    if not isinstance(candidates, list):
        candidates = []
    reports = {}
    for report in candidates:
        report = _dict(report)
        facts = _dict(report.get("evidence")) or _dict(_dict(raw).get("evidence"))
        if facts.get("source_sha256") == source_hash:
            reports[report.get("analyzer_id")] = report
    navigation = _dict(_dict(reports.get("navigation")).get("metrics"))
    decisions = _dict(navigation.get("decision_events"))
    timing = _dict(decisions.get("runtime_timing"))
    selected = {name: {"summary": report.get("summary"), "admission": report.get("admission"),
                      "metrics": {key: value for key, value in _dict(report.get("metrics")).items()
                                  if key in {"performance", "motion_quality", "tracking", "motion_continuity", "sampled_no_progress",
                                             "temporal_coverage", "goal_progress", "dense_navigation", "sampled_translation",
                                             "longest_episode", "cause_counts", "evidence_blockers", "navigation_health_verdict"}}}
                for name, report in reports.items() if name in {"navigation_root_cause", "motion_performance"}}
    return _feature("PASS" if timing else "NOT_EXERCISED",
        "A source-bound canonical DIAG report preserves the producer control-loop timing aggregate.",
        metrics={"source_bound_analyzers": sorted(name for name in reports if isinstance(name, str)), "runtime_timing": timing,
                 "canonical_navigation_measurements": selected},
        limits=("One run without a comparable baseline cannot prove timing improvement or non-regression.",
                "Wall minus thread CPU does not distinguish device I/O, GIL waits and scheduler contention."))


def _replay_coverage(bundle, resolved, skill, phases):
    complete_layers = resolved.get("layers") == [f"L{i}" for i in range(1, 13)]
    if complete_layers and resolved.get("tick_count") == bundle.view_count("ticks"):
        return {"covered": True, "scope": "CAPTURE_WIDE", "required_tick_count": bundle.view_count("ticks")}
    finish = skill.get("observed_end_ns")
    if not phases or type(finish) is not int:
        return {"covered": False, "scope": "NO_BOUND_SKILL_INTERVAL"}
    start = min(phase[1] for phase in phases)
    required = []
    count = 0
    for row in bundle.iter_view("ticks", start_ns=start, end_ns=finish):
        payload = _dict(row.payload)
        stamp, tick = payload.get("monotonic_ns"), payload.get("tick_id")
        if type(stamp) is int and type(tick) is int and start <= stamp <= finish:
            count += 1
            if not required:
                required.append((tick, stamp))
            elif len(required) == 1:
                required.append((tick, stamp))
            else:
                required[1] = (tick, stamp)
    first, last = required[0] if required else (None, None), required[-1] if required else (None, None)
    lower, upper = resolved.get("first_tick_id"), resolved.get("last_tick_id")
    covered = (complete_layers and count > 0 and type(lower) is int and type(upper) is int
               and lower <= first[0] <= last[0] <= upper)
    return {"covered": bool(covered), "scope": "SUPPLIED_SKILL_INTERVAL", "required_interval_ns": [start, finish],
            "required_tick_count": count, "required_first_tick_id": first[0], "required_last_tick_id": last[0],
            "required_first_source_time_ns": first[1], "required_last_source_time_ns": last[1]}


def evaluate_navigation_live(*, evidence, capture, replay_result=None, diag_results=None,
                             skill_result=None, project_root=None):
    """Read explicit finalized artifacts under the canonical offline guard."""
    from v3.host_cli import hardware_guard

    root = Path(project_root or Path(__file__).resolve().parents[1]).resolve()
    with hardware_guard(root):
        verification = verify(evidence, source=capture)
        bundle = EvidenceBundle.open(evidence, verification="manifest")
        source_hash = bundle.manifest["source"]["sha256"]
        _, skill, _ = _skill(skill_result)
        phases = _phases(skill)
        try:
            reader = McapReader(capture)
            structure = reader.inspect(verify_chunks=True)
            if not structure.valid:
                raise McapReadError("Strict native CRC/index preflight failed: " + "; ".join(structure.errors))
            capture_fact = reader.capture_integrity()
            integrity = _dict(capture_fact.get("integrity"))
            capture_ok = all(integrity.get(name) is True for name in ("complete", "sample_complete", "replay_complete",
                                                                     "raw_evidence_requested", "raw_evidence_complete"))
            capture_metrics = {"status": capture_fact.get("status"), "native_crc_valid": structure.valid, "integrity": integrity}
        except (OSError, ValueError, McapReadError) as exc:
            capture_ok, capture_metrics = False, {"error": str(exc)}
        features = {"capture_integrity": _feature("PASS" if capture_ok else "FAIL",
            "Strict native capture integrity and full EVI verification bind complete requested raw/replay evidence to the source MCAP.",
            metrics={"verification": verification, **capture_metrics})}
        if replay_result is None:
            features["native_replay"] = _feature("NOT_EXERCISED", "Canonical replay was not supplied.")
        else:
            replay_verification = verify_replay_result(replay_result)
            replay = json.loads(Path(replay_result).read_text())
            resolved = _dict(_dict(replay.get("scope")).get("resolved"))
            scope_coverage = _replay_coverage(bundle, resolved, skill, phases)
            match = replay_verification["status"] == "PASS" and _dict(replay.get("capture")).get("sha256") == source_hash
            features["native_replay"] = _feature("PASS" if capture_ok and match and scope_coverage["covered"] else "FAIL",
                "Verified canonical replay MATCH covers L1-L12 and every captured tick of the declared live observation/stop interval from the same source hash.",
                metrics={"verification": replay_verification, "source_matches": _dict(replay.get("capture")).get("sha256") == source_hash,
                         "coverage": scope_coverage, "scope": replay.get("scope")},
                limits=("Replay proves recorded decision reproduction, not physical path accuracy.",))
        features["live_run_binding"] = _binding(bundle, skill_result)
        binding_ok = features["live_run_binding"]["status"] == "PASS"
        features["command_publication"] = _publication(bundle, phases)
        features.update(_tick_measurements(bundle, phases))
        from tools.navigation_validation_planner import evaluate_captured_planner_requests
        features["captured_planner_direct_equivalence"] = evaluate_captured_planner_requests(
            bundle, root, maximum_requests=4, phases=phases) if phases else _feature("NOT_EXERCISED", "No valid skill phase interval was supplied.")
        features.update(_spatial(skill, binding_ok))
        features["runtime_timing_measurement"] = _diagnostics(diag_results, source_hash)
        outer, _, _ = _skill(skill_result)
        failed = outer.get("state") in {"FAILED", "CANCELLED"} or skill.get("status") == "FAILED"
        planned_windows = [p for p in skill.get("phases", []) if _dict(p).get("name") == "room_cruise"]
        window_complete = bool(planned_windows) and all(_dict(p).get("outcome") == "TIMED_OBSERVATION_COMPLETE"
            and _dict(p).get("stop_confirmed") is True for p in planned_windows)
        features["skill_observation_window"] = _feature("FAIL" if failed else "PASS" if skill.get("status") == "COMPLETED" and phases and window_complete else "NOT_EXERCISED",
            "The bounded SDK observation program completed and confirmed its canonical stop.",
            metrics={"program_state": outer.get("state"), "status": skill.get("status"), "reason": skill.get("reason"),
                     "measurement_outcome": skill.get("measurement_outcome"),
                     "phases": [{key: _dict(p).get(key) for key in ("name", "command_id", "first_tick_id", "last_tick_id",
                          "lifecycle", "reason", "outcome", "stop_confirmed", "estimated_path_m", "max_status_age_ns",
                          "requested_observation_s", "observation_started_ns", "end_monotonic_ns")}
                                for p in skill.get("phases", [])]},
            limits=("Program completion is distinct from navigation quality and physical goal satisfaction.",
                    "BOUND_REACHED ends observation before the requested duration and does not prove that full window."))
        # Preserve measured failures; never let unbound snapshots create a PASS.
        for name, result in features.items():
            if name not in {"capture_integrity", "native_replay", "live_run_binding", "runtime_timing_measurement"} and result["status"] == "PASS" and (not capture_ok or not binding_ok):
                result["status"] = "NOT_EXERCISED"
                result["limits"].append("Positive live acceptance blocked by capture integrity or skill-run binding.")
        statuses = Counter(item["status"] for item in features.values())
        return {"schema": SCHEMA, "status": "FAIL" if statuses["FAIL"] else "PARTIAL" if statuses["NOT_EXERCISED"] else "PASS",
                "source": {"capture": str(Path(capture).resolve()), "sha256": source_hash,
                           "evidence": str(bundle.root), "bundle_manifest_sha256": bundle.manifest["self_sha256"]},
                "feature_counts": dict(statuses), "features": features,
                "unproven": ["Independent physical path/obstacle accuracy", "Timing improvement without a comparable baseline",
                             "Unexercised stale/crash/no-path/no-progress/strategy branches"],
                "physical_ground_truth": "NOT_INDEPENDENTLY_MEASURED"}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", required=True)
    parser.add_argument("--capture", required=True)
    parser.add_argument("--replay-result")
    parser.add_argument("--diag-results")
    parser.add_argument("--skill-result", required=True)
    parser.add_argument("--project-root", default=str(Path(__file__).resolve().parents[1]))
    parser.add_argument("--output", required=True, help="New report JSON; existing artifacts are never overwritten.")
    args = parser.parse_args(argv)
    def read(path):
        return json.loads(Path(path).read_text()) if path else None
    result = evaluate_navigation_live(evidence=args.evidence, capture=args.capture, replay_result=args.replay_result,
        diag_results=read(args.diag_results), skill_result=read(args.skill_result), project_root=args.project_root)
    with Path(args.output).open("x") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"status": result["status"], "feature_counts": result["feature_counts"], "output": args.output}))
    return 1 if result["status"] == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
