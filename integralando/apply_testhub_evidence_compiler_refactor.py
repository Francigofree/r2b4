#!/usr/bin/env python3
"""
R2B4 TestHub evidence-compiler refactor.

Source authority:
  Francigofree/r2b4
  expected HEAD: 3f2dfbb4c9bbcfd07d114f9386847139f9c88f9b

Goal:
  TestHub compiles, validates and prioritizes evidence, but does not infer root
  cause. Causal/root-cause synthesis is owned by the analyzer LLM.

Transactional behavior:
  - refuses a mismatching HEAD unless --allow-head-mismatch is explicit
  - refuses dirty target files
  - verifies source anchors before writing
  - creates a backup
  - runs compile + focused pytest
  - restores originals on validation failure
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import textwrap
import time
from pathlib import Path

EXPECTED_HEAD = "3f2dfbb4c9bbcfd07d114f9386847139f9c88f9b"

TARGETS = (
    "v3/test_hub_analysis.py",
    "v3/test_hub_v2.py",
    "v3/replay.py",
    "v3/test_hub.py",
    "v3/test_hub_next.py",
    "v3/diagnostic_contracts.py",
    "v3/mcap_capture.py",
    "v3/test_hub_motion_quality.py",
    "v3/test_hub_localization_quality.py",
)
NEW_TEST = "tests/deep/test_v3_test_hub_evidence_compiler_contract.py"


def run(*args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        args,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if check and result.returncode != 0:
        print(
            f"COMMAND FAILED ({result.returncode}): {' '.join(args)}",
            file=sys.stderr,
        )
        if result.stdout:
            print("--- stdout ---", file=sys.stderr)
            print(result.stdout, file=sys.stderr, end="" if result.stdout.endswith("\n") else "\n")
        if result.stderr:
            print("--- stderr ---", file=sys.stderr)
            print(result.stderr, file=sys.stderr, end="" if result.stderr.endswith("\n") else "\n")
        raise subprocess.CalledProcessError(
            result.returncode,
            args,
            output=result.stdout,
            stderr=result.stderr,
        )
    return result


def git(*args: str, check: bool = True) -> str:
    return run("git", *args, check=check).stdout.strip()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one anchor, found {count}")
    return text.replace(old, new, 1)


def replace_function(
    text: str,
    start_name: str,
    next_name: str,
    new_body: str,
    label: str,
) -> str:
    pattern = re.compile(
        rf"\ndef {re.escape(start_name)}\([\s\S]*?(?=\n\ndef {re.escape(next_name)}\()"
    )
    matches = list(pattern.finditer(text))
    if len(matches) != 1:
        raise RuntimeError(f"{label}: expected one function block, found {len(matches)}")
    replacement = "\n" + textwrap.dedent(new_body).rstrip() + "\n"
    return text[: matches[0].start()] + replacement + text[matches[0].end() :]


def patch_test_hub_analysis(text: str) -> str:
    text = replace_once(
        text,
        r"""    def as_dict(self) -> dict[str, object]:
        return {
            "id": self.incident_id,
            "severity": self.severity,
            "category": self.category,
            "tick_id": self.tick_id,
            "monotonic_ns": self.monotonic_ns,
            "layer": self.layer,
            "reason": self.reason,
            "evidence": dict(self.evidence),
        }
""",
        r"""    def as_dict(self) -> dict[str, object]:
        if self.category in {"TIMING", "LOCALIZATION", "NAVIGATION"}:
            claim_class = "HEURISTIC_FINDING"
        elif self.category in {
            "ADMISSION",
            "PRODUCTION_FAULT",
            "SAFETY",
            "MOTION_BLOCKED",
        }:
            claim_class = "POLICY_VERDICT"
        else:
            claim_class = "FACT"
        return {
            "id": self.incident_id,
            "severity": self.severity,
            "category": self.category,
            "claim_class": claim_class,
            "causal_claim": False,
            "tick_id": self.tick_id,
            "monotonic_ns": self.monotonic_ns,
            "layer": self.layer,
            "reason": self.reason,
            "evidence": dict(self.evidence),
        }
""",
        "analysis:incident-claim-class",
    )
    text = replace_once(
        text,
        "    l2_rejection_reasons: dict[str, int] = {}\n",
        "    l2_rejection_reasons: dict[str, int] = {}\n"
        "    previous_l2_actionable_signature: tuple[tuple[str, str], ...] = ()\n",
        "analysis:l2-signature-state",
    )

    old = """                if actionable:
                    ordered_reasons = tuple(sorted(actionable_reasons))
                    severity = (
                        "HIGH"
                        if any(reason in _HIGH_L2_REASONS for reason in ordered_reasons)
                        else "MEDIUM"
                    )
                    reason_text = "OBSERVATION_REJECTED:" + ",".join(ordered_reasons)
                    append_incident(
                        incidents,
                        Incident(
                            f"l2-reject-{tick_id}",
                            severity,
                            "ADMISSION",
                            tick_id,
                            monotonic_ns,
                            "L2",
                            reason_text,
                            {"rejected": actionable[:8]},
                        ),
                        max_incidents,
                    )
"""
    new = """                if actionable:
                    ordered_reasons = tuple(sorted(actionable_reasons))
                    severity = (
                        "HIGH"
                        if any(reason in _HIGH_L2_REASONS for reason in ordered_reasons)
                        else "MEDIUM"
                    )
                    reason_text = "OBSERVATION_REJECTED:" + ",".join(ordered_reasons)
                    actionable_signature = tuple(
                        sorted(
                            (
                                str(item.get("source_device_id") or ""),
                                str(item.get("reason") or "UNKNOWN"),
                            )
                            for item in actionable
                        )
                    )
                    # Repeated identical admission states are one evidence episode,
                    # not one actionable incident per 50 Hz control tick. Device
                    # identity is part of the signature so independent sources do
                    # not get conflated into the same episode.
                    if actionable_signature != previous_l2_actionable_signature:
                        append_incident(
                            incidents,
                            Incident(
                                f"l2-reject-{tick_id}",
                                severity,
                                "ADMISSION",
                                tick_id,
                                monotonic_ns,
                                "L2",
                                reason_text,
                                {
                                    "rejected": actionable[:8],
                                    "episode_edge": "START",
                                },
                            ),
                            max_incidents,
                        )
                    previous_l2_actionable_signature = actionable_signature
                else:
                    previous_l2_actionable_signature = ()
"""
    text = replace_once(text, old, new, "analysis:l2-episode-collapse")

    old = """    incidents.sort(key=_incident_sort_key)
    root = _root_cause_candidate([item for item in incidents if item.severity != "WARNING"])
    if behavioral:
        root["scope"] = "CAPTURED_SAMPLES_ONLY"
"""
    new = """    incidents.sort(key=_incident_sort_key)
    priority = _priority_evidence_candidate(
        [item for item in incidents if item.severity != "WARNING"]
    )
    if behavioral:
        priority["scope"] = "CAPTURED_SAMPLES_ONLY"
"""
    text = replace_once(text, old, new, "analysis:priority-selection")

    text = replace_once(
        text,
        '        "root_cause_candidate": root,\n',
        """        "priority_evidence_candidate": priority,
        "analysis_handoff": {
            "root_cause_inferred": False,
            "causal_analysis_owner": "ANALYZER_LLM",
            "test_hub_role": "EVIDENCE_COMPILER",
        },
""",
        "analysis:triage-contract",
    )

    new_func = """
def _priority_evidence_candidate(
    incidents: Sequence[Incident],
) -> dict[str, object]:
    \"\"\"Select the most useful evidence slice without inferring causality.\"\"\"

    if not incidents:
        return {
            "claim_class": "FACT",
            "causal_claim": False,
            "evidence_strength": "NONE",
            "selection_basis": "NO_NON_NOMINAL_EVIDENCE",
            "reason": "NO_NON_NOMINAL_EVIDENCE_FOUND",
            "tick_id": None,
            "layer": None,
            "kind": None,
            "evidence_ids": [],
        }

    blocked = next(
        (item for item in incidents if item.category == "MOTION_BLOCKED"),
        None,
    )

    direct_categories = {
        "CAPTURE_INTEGRITY",
        "TICK_SEQUENCE",
        "EDGE_FAULT",
        "PRODUCTION_FAULT",
        "SAFETY",
    }
    direct = next(
        (
            item
            for item in incidents
            if item.category in direct_categories
            and not (
                item.category in {"SAFETY", "PRODUCTION_FAULT"}
                and blocked is not None
                and blocked.tick_id is not None
                and item.tick_id is not None
                and blocked.tick_id < item.tick_id
            )
        ),
        None,
    )
    if direct is not None:
        return {
            "claim_class": (
                "POLICY_VERDICT"
                if direct.category in {"PRODUCTION_FAULT", "SAFETY"}
                else "FACT"
            ),
            "causal_claim": False,
            "evidence_strength": "DIRECT",
            "selection_basis": "DIRECT_CAPTURE_OR_PRODUCTION_EVIDENCE",
            "reason": direct.reason,
            "tick_id": direct.tick_id,
            "layer": direct.layer,
            "kind": direct.category,
            "evidence_ids": [direct.incident_id],
        }

    if blocked is not None:
        same_tick = [
            item
            for item in incidents
            if item.tick_id == blocked.tick_id
            and item.category != "ADMISSION"
        ]
        return {
            "claim_class": "POLICY_VERDICT",
            "causal_claim": False,
            "evidence_strength": "DIRECT",
            "selection_basis": "DIRECT_MOTION_BLOCK_OBSERVATION",
            "reason": blocked.reason,
            "tick_id": blocked.tick_id,
            "layer": blocked.layer,
            "kind": "MOTION_BLOCKED",
            "evidence_ids": [
                item.incident_id for item in same_tick[:6]
            ]
            or [blocked.incident_id],
        }

    first = incidents[0]
    return {
        "claim_class": "HEURISTIC_FINDING",
        "causal_claim": False,
        "evidence_strength": "HEURISTIC",
        "selection_basis": "SEVERITY_TIME_ORDER",
        "reason": first.reason,
        "tick_id": first.tick_id,
        "layer": first.layer,
        "kind": first.category,
        "evidence_ids": [first.incident_id],
    }
"""
    return replace_function(
        text,
        "_root_cause_candidate",
        "_incident_sort_key",
        new_func,
        "analysis:root-to-priority-function",
    )


def patch_replay(text: str) -> str:
    old = """    first_live_incident = _first_live_incident(selected_ticks)
    physical_root_cause = _physical_root_cause(
        general_payload,
        first_live_incident,
        divergence,
    )
"""
    new = """    first_live_observation = _first_live_observation(selected_ticks)
    physical_evidence = _physical_evidence(
        general_payload,
        first_live_observation,
        divergence,
    )
"""
    text = replace_once(text, old, new, "replay:observation-build")

    old = """        "diagnostics": {
            "layers": layer_rows,
            "first_divergence": divergence,
            "first_live_incident": first_live_incident,
            "physical_root_cause": physical_root_cause,
        },
        "source_first": source_first,
        "first_divergence": divergence,
        "first_live_incident": first_live_incident,
        "physical_root_cause": physical_root_cause,
"""
    new = """        "diagnostics": {
            "layers": layer_rows,
            "first_divergence": divergence,
            "first_live_observation": first_live_observation,
            "physical_evidence": physical_evidence,
        },
        "source_first": source_first,
        "first_divergence": divergence,
        "first_live_observation": first_live_observation,
        "physical_evidence": physical_evidence,
        "analysis_handoff": {
            "root_cause_inferred": False,
            "causal_analysis_owner": "ANALYZER_LLM",
            "replay_role": "DETERMINISM_AND_DIRECT_EVIDENCE_ONLY",
        },
"""
    text = replace_once(text, old, new, "replay:result-contract")

    new_first = """
def _first_live_observation(
    ticks: Sequence[Mapping[str, object]],
) -> dict[str, object] | None:
    \"\"\"Return the first direct production-relevant observation in scope.\"\"\"

    for tick in ticks:
        expected = _mapping(tick.get("expected"), "tick.expected")
        edge = tick.get("edge_fault")
        if isinstance(edge, Mapping):
            return {
                "tick_id": tick.get("tick_id"),
                "layer": edge.get("fault_layer"),
                "reason": edge.get("reason"),
                "evidence": _tick_evidence(tick),
            }
        writer_failure = expected.get("writer_failure")
        if isinstance(writer_failure, Mapping):
            return {
                "tick_id": tick.get("tick_id"),
                "layer": "L12",
                "reason": writer_failure.get("reason"),
                "evidence": _tick_evidence(tick),
            }
        raw = tick.get("inputs")
        if isinstance(raw, Mapping):
            devices = raw.get("raw_devices")
            health = devices.get("device_health") if isinstance(devices, Mapping) else None
            if isinstance(health, Sequence) and not isinstance(health, (str, bytes)):
                degraded = next(
                    (
                        row
                        for row in health
                        if isinstance(row, Mapping)
                        and row.get("state") != "OK"
                        and str(row.get("device_id") or "")
                        in PRODUCTION_CRITICAL_DEVICE_IDS
                    ),
                    None,
                )
                if degraded is not None:
                    return {
                        "tick_id": tick.get("tick_id"),
                        "layer": "L0",
                        "reason": degraded.get("reason") or degraded.get("state"),
                        "device_id": degraded.get("device_id"),
                        "evidence": _tick_evidence(tick),
                    }
        fault_layer = expected.get("fault_layer")
        layers = expected.get("layers")
        l12 = layers.get("L12") if isinstance(layers, Mapping) else None
        decision = l12.get("safety_decision") if isinstance(l12, Mapping) else None
        if fault_layer is not None or decision == "FAULT":
            return {
                "tick_id": tick.get("tick_id"),
                "layer": fault_layer or "L12",
                "reason": l12.get("reason") if isinstance(l12, Mapping) else None,
                "evidence": _tick_evidence(tick),
            }
    return None
"""
    text = replace_function(
        text,
        "_first_live_incident",
        "_physical_root_cause",
        new_first,
        "replay:first-live-observation",
    )

    new_physical = """
def _physical_evidence(
    payload: Mapping[str, object],
    observation: Mapping[str, object] | None,
    divergence: Mapping[str, object] | None,
) -> dict[str, object]:
    \"\"\"Describe physical-edge evidence without turning it into a causal verdict.\"\"\"

    reason = (
        str(observation.get("reason") or "")
        if observation is not None
        else ""
    )
    if reason == "MOTOR_WRITER_FAILURE":
        return {
            "claim_class": "POLICY_VERDICT",
            "causal_claim": False,
            "evidence_strength": "DIRECT",
            "observation": "MOTOR_WRITER_FAILURE",
            "reason": "THE_CANONICAL_L12_WRITE_RAISED",
            "evidence": observation.get("evidence"),
        }

    raw = payload.get("raw_lidar_evidence")
    missing = raw.get("missing_revisions") if isinstance(raw, Mapping) else ()
    if (
        isinstance(missing, Sequence)
        and not isinstance(missing, (str, bytes))
        and missing
    ):
        return {
            "claim_class": "FACT",
            "causal_claim": False,
            "evidence_strength": "DIRECT",
            "observation": "RAW_LIDAR_EVIDENCE_INCOMPLETE",
            "reason": "REFERENCED_RAW_LIDAR_MISSING",
            "evidence": {"missing_raw_lidar_revisions": list(missing)},
        }

    if observation is None:
        return {
            "claim_class": "FACT",
            "causal_claim": False,
            "evidence_strength": "NONE",
            "observation": None,
            "reason": "NO_PRODUCTION_RELEVANT_LIVE_OBSERVATION_IN_SCOPE",
            "evidence": None,
        }

    evidence = observation.get("evidence")
    device_health = (
        evidence.get("device_health")
        if isinstance(evidence, Mapping)
        else None
    )
    critical_degraded = (
        [
            dict(row)
            for row in device_health
            if isinstance(row, Mapping)
            and row.get("state") != "OK"
            and str(row.get("device_id") or "")
            in PRODUCTION_CRITICAL_DEVICE_IDS
        ]
        if isinstance(device_health, Sequence)
        and not isinstance(device_health, (str, bytes))
        else []
    )

    if critical_degraded:
        return {
            "claim_class": "HEURISTIC_FINDING",
            "causal_claim": False,
            "evidence_strength": "INDICATOR",
            "observation": reason or "CRITICAL_DEVICE_HEALTH_DEGRADATION",
            "reason": "CRITICAL_DEVICE_HEALTH_COOBSERVED",
            "evidence": {
                "critical_device_health": critical_degraded,
                "tick_evidence": evidence,
            },
        }

    return {
        "claim_class": "FACT",
        "causal_claim": False,
        "evidence_strength": "INSUFFICIENT",
        "observation": None,
        "reason": (
            "REPLAY_DIVERGENCE_IS_NOT_PHYSICAL_CAUSAL_PROOF"
            if divergence is not None
            else "NO_DIRECT_PHYSICAL_EDGE_EXPLANATION"
        ),
        "evidence": evidence,
    }
"""
    return replace_function(
        text,
        "_physical_root_cause",
        "_general_execution_summary",
        new_physical,
        "replay:physical-evidence",
    )


def patch_test_hub_v2(text: str) -> str:
    text = replace_once(
        text,
        """    root = diagnosis.get("root_cause")
    root_tick = (
        root.get("tick_id") if isinstance(root, Mapping) else None
    )
    root_layer = (
        root.get("layer") if isinstance(root, Mapping) else None
    )
""",
        """    priority = diagnosis.get("priority_evidence")
    priority_tick = (
        priority.get("tick_id") if isinstance(priority, Mapping) else None
    )
    priority_layer = (
        priority.get("layer") if isinstance(priority, Mapping) else None
    )
""",
        "v2:diagnose-priority",
    )
    text = replace_once(
        text,
        """        around_tick=root_tick if isinstance(root_tick, int) else None,
        root_layer=(
            str(root_layer) if isinstance(root_layer, str) else None
        ),
""",
        """        around_tick=priority_tick if isinstance(priority_tick, int) else None,
        root_layer=(
            str(priority_layer) if isinstance(priority_layer, str) else None
        ),
""",
        "v2:slice-priority",
    )
    text = replace_once(
        text,
        """        "replay_status": diagnosis.get("replay_status"),
        "root_cause": diagnosis.get("root_cause"),
        "incident_count": triage.get("incident_count"),
""",
        """        "replay_status": diagnosis.get("replay_status"),
        "priority_evidence": diagnosis.get("priority_evidence"),
        "analysis_handoff": diagnosis.get("analysis_handoff"),
        "incident_count": triage.get("incident_count"),
""",
        "v2:return-priority",
    )
    text = replace_once(
        text,
        '                        "root_tick": around_tick,\n',
        '                        "priority_tick": around_tick,\n',
        "v2:slice-note",
    )

    new_diag = """
def _build_diagnosis(
    inspect: Mapping[str, object],
    triage: Mapping[str, object],
    replay: Mapping[str, object] | None,
    replay_error: str | None,
    *,
    replay_requested: bool = True,
) -> dict[str, object]:
    \"\"\"Compile validated evidence without making an automatic causal judgment.\"\"\"

    profile = inspect.get("analysis_profile") or triage.get("analysis_profile")
    behavioral = isinstance(profile, Mapping) and profile.get("name") == BEHAVIORAL
    priority = (
        dict(triage.get("priority_evidence_candidate") or {})
        if isinstance(triage.get("priority_evidence_candidate"), Mapping)
        else {}
    )
    if not priority and isinstance(triage.get("root_cause_candidate"), Mapping):
        legacy = dict(triage["root_cause_candidate"])
        priority = {
            "claim_class": "HEURISTIC_FINDING",
            "causal_claim": False,
            "evidence_strength": str(legacy.get("confidence") or "LEGACY"),
            "selection_basis": "LEGACY_TRIAGE_PRIORITY",
            "reason": legacy.get("reason"),
            "tick_id": legacy.get("tick_id"),
            "layer": legacy.get("layer"),
            "kind": legacy.get("kind"),
            "evidence_ids": legacy.get("evidence_ids", []),
        }

    replay_status = replay.get("status") if isinstance(replay, Mapping) else None
    first_divergence = (
        replay.get("first_divergence")
        if isinstance(replay, Mapping)
        else None
    )
    first_live_observation = (
        replay.get("first_live_observation")
        if isinstance(replay, Mapping)
        else None
    )
    if (
        first_live_observation is None
        and isinstance(replay, Mapping)
        and isinstance(replay.get("first_live_incident"), Mapping)
    ):
        first_live_observation = replay.get("first_live_incident")

    physical_evidence = (
        replay.get("physical_evidence")
        if isinstance(replay, Mapping)
        else None
    )
    if (
        physical_evidence is None
        and isinstance(replay, Mapping)
        and isinstance(replay.get("physical_root_cause"), Mapping)
    ):
        legacy_physical = replay["physical_root_cause"]
        physical_evidence = {
            "claim_class": "HEURISTIC_FINDING",
            "causal_claim": False,
            "evidence_strength": str(legacy_physical.get("status") or "LEGACY"),
            "observation": legacy_physical.get("cause"),
            "reason": legacy_physical.get("reason"),
            "evidence": _prune_large(legacy_physical.get("evidence")),
            "source": "LEGACY_REPLAY_COMPATIBILITY",
        }

    if (
        isinstance(first_divergence, Mapping)
        and first_divergence.get("reason") != "CAPTURE_INCOMPLETE"
        and replay_status == "MISMATCH"
        and not inspect.get("integrity_error")
    ):
        priority = {
            "claim_class": "POLICY_VERDICT",
            "causal_claim": False,
            "evidence_strength": "DIRECT",
            "selection_basis": "REPLAY_DIVERGENCE",
            "reason": first_divergence.get("reason") or "REPLAY_DIVERGENCE",
            "tick_id": first_divergence.get("tick_id"),
            "layer": first_divergence.get("layer"),
            "kind": "SOFTWARE_REPLAY_DIVERGENCE",
            "evidence": _prune_large(first_divergence),
        }

    final_event = inspect.get("final_event")
    integrity = (
        final_event.get("integrity")
        if isinstance(final_event, Mapping)
        else None
    )
    structure = inspect.get("structure")
    structure_ok = bool(
        isinstance(structure, Mapping)
        and structure.get("valid") is True
    )
    capture_complete = bool(
        isinstance(integrity, Mapping)
        and integrity.get("complete") is True
    )
    capture_complete = capture_complete and not inspect.get("integrity_error")

    if (
        not capture_complete
        or (
            isinstance(first_divergence, Mapping)
            and first_divergence.get("reason") == "CAPTURE_INCOMPLETE"
        )
    ):
        priority = {
            "claim_class": "FACT",
            "causal_claim": False,
            "evidence_strength": "DIRECT",
            "selection_basis": "EVIDENCE_VALIDITY_GATE",
            "kind": "CAPTURE_INCOMPLETE",
            "reason": inspect.get("integrity_error") or "CAPTURE_INCOMPLETE",
            "tick_id": None,
            "layer": "Capture",
            "evidence_ids": ["capture-integrity"],
        }

    replay_gate_ok = (
        (not replay_requested and replay_error is None)
        or replay_status == "MATCH"
    )
    if replay_requested and replay_status is None:
        replay_status = "ERROR" if replay_error else "NOT_RUN"

    evidence_status = (
        "PASS"
        if structure_ok and capture_complete and replay_gate_ok
        else "FAIL"
    )
    behavior_status = _triage_behavior_status(triage)
    actionable = _has_actionable_finding(triage)

    if evidence_status != "PASS":
        diagnosis_status = "FAIL"
    elif actionable:
        diagnosis_status = "FINDING"
    elif behavioral and triage.get("warning_count", 0):
        diagnosis_status = "WARNING"
    else:
        diagnosis_status = "PASS"

    return {
        "schema": "R2B4_TEST_HUB_DIAGNOSIS_V2",
        **({"analysis_profile": profile} if profile is not None else {}),
        "status": diagnosis_status,
        "diagnosis_status": diagnosis_status,
        "evidence_status": evidence_status,
        "behavior_status": behavior_status,
        "capture_status": (
            final_event.get("status")
            if isinstance(final_event, Mapping)
            else None
        ),
        "capture_integrity": integrity,
        "structure_valid": structure_ok,
        "replay_status": (
            "NOT_APPLICABLE"
            if behavioral and not replay_requested
            else replay_status
        ),
        **(
            {
                "replay_skip_reason": "SAMPLED_TICK_STREAM",
                "warning_count": triage.get("warning_count", 0),
            }
            if behavioral
            else {}
        ),
        "replay_error": replay_error,
        "replay_requested": replay_requested,
        "evidence_compiler_policy": "NO_AUTOMATIC_ROOT_CAUSE_V1",
        "priority_evidence": priority,
        "first_divergence": first_divergence,
        "first_live_observation": first_live_observation,
        "physical_evidence": physical_evidence,
        "analysis_handoff": {
            "root_cause_inferred": False,
            "causal_analysis_owner": "ANALYZER_LLM",
            "test_hub_role": "EVIDENCE_COMPILER",
            "allowed_claim_classes": [
                "FACT",
                "POLICY_VERDICT",
                "HEURISTIC_FINDING",
            ],
        },
        "metrics": {
            "ticks": triage.get("ticks"),
            "timing": triage.get("timing"),
            "motion": triage.get("motion"),
            "safety": triage.get("safety"),
            "sensors": triage.get("sensors"),
            "localization": triage.get("localization"),
            "navigation": triage.get("navigation"),
            **(
                {"behavioral_trends": triage.get("behavioral_trends")}
                if behavioral
                else {}
            ),
        },
        "incident_count": triage.get("incident_count"),
    }
"""
    text = replace_function(
        text,
        "_build_diagnosis",
        "_triage_behavior_status",
        new_diag,
        "v2:build-diagnosis",
    )

    new_window = """
def _incident_replay_window(
    triage: Mapping[str, object],
) -> ReplayWindow:
    priority = triage.get("priority_evidence_candidate")
    if not isinstance(priority, Mapping):
        priority = triage.get("root_cause_candidate")
    tick_id = (
        priority.get("tick_id")
        if isinstance(priority, Mapping)
        else None
    )
    first_tick = _nested_int(triage, "ticks", "first_tick_id")
    last_tick = _nested_int(triage, "ticks", "last_tick_id")

    if not isinstance(tick_id, int):
        return ReplayWindow(
            requested_start_tick_id=first_tick,
            requested_end_tick_id=(
                min(last_tick, first_tick + 10)
                if first_tick is not None and last_tick is not None
                else None
            ),
        )

    layer = (
        priority.get("layer")
        if isinstance(priority, Mapping)
        else None
    )
    kind = (
        priority.get("kind")
        if isinstance(priority, Mapping)
        else None
    )
    reason = (
        str(priority.get("reason") or "")
        if isinstance(priority, Mapping)
        else ""
    )

    if (
        kind == "MOTION_BLOCKED"
        and (
            layer == "L9"
            or "LOCALIZATION" in reason.upper()
        )
    ):
        start = max(0, tick_id - 50)
        if first_tick is not None:
            start = max(first_tick, start)
        end = tick_id + 10
        if last_tick is not None:
            end = min(last_tick, end)
        return ReplayWindow(
            requested_start_tick_id=start,
            requested_end_tick_id=end,
            start_layer="L1",
            end_layer="L12",
        )

    return ReplayWindow(
        requested_start_tick_id=max(0, tick_id - 3),
        requested_end_tick_id=tick_id + 3,
        start_layer="L1",
        end_layer="L12",
    )
"""
    text = replace_function(
        text,
        "_incident_replay_window",
        "_build_agent_brief",
        new_window,
        "v2:incident-window",
    )

    new_brief = """
def _build_agent_brief(
    capture_path: str | Path,
    inspect: Mapping[str, object],
    triage: Mapping[str, object],
    replay: Mapping[str, object] | None,
    diagnosis: Mapping[str, object],
    slice_info: Mapping[str, object],
    *,
    max_bytes: int,
) -> dict[str, object]:
    priority = (
        diagnosis.get("priority_evidence")
        if isinstance(diagnosis.get("priority_evidence"), Mapping)
        else {}
    )
    incidents = (
        triage.get("incidents")
        if isinstance(triage.get("incidents"), list)
        else []
    )
    priority_layer = (
        priority.get("layer")
        if isinstance(priority, Mapping)
        else None
    )
    source_files = _source_hints(
        str(priority_layer)
        if isinstance(priority_layer, str)
        else None,
        replay,
        reason=(
            str(priority.get("reason") or "")
            if isinstance(priority, Mapping)
            else ""
        ),
    )
    brief: dict[str, object] = {
        "schema": AGENT_BRIEF_SCHEMA,
        "status": diagnosis.get("status"),
        "analysis_profile": diagnosis.get("analysis_profile"),
        "diagnosis_status": diagnosis.get("diagnosis_status"),
        "evidence_status": diagnosis.get("evidence_status"),
        "behavior_status": diagnosis.get("behavior_status"),
        "capture": {
            "path": str(Path(capture_path).resolve()),
            "capture_status": diagnosis.get("capture_status"),
            "integrity_complete": (
                diagnosis.get("capture_integrity", {}).get("complete")
                if isinstance(diagnosis.get("capture_integrity"), Mapping)
                else None
            ),
        },
        "evidence_compiler_policy": diagnosis.get("evidence_compiler_policy"),
        "priority_evidence": priority,
        "physical_evidence": _prune_large(
            diagnosis.get("physical_evidence")
        ),
        "analysis_handoff": diagnosis.get("analysis_handoff"),
        "replay": {
            "status": diagnosis.get("replay_status"),
            "first_divergence": _prune_large(
                diagnosis.get("first_divergence")
            ),
            "first_live_observation": _prune_large(
                diagnosis.get("first_live_observation")
            ),
            "bridge": (
                replay.get("mcap_bridge")
                if isinstance(replay, Mapping)
                else None
            ),
        },
        "top_incidents": [
            _prune_large(item) for item in incidents[:8]
        ],
        "metrics": diagnosis.get("metrics"),
        "source_files": source_files,
        "evidence_slice": dict(slice_info),
        "next_queries": _suggest_queries(priority),
        "contract": {
            "claim_policy": (
                "Test Hub emits FACT/POLICY_VERDICT/HEURISTIC_FINDING "
                "evidence only; causal/root-cause synthesis belongs to the analyzer LLM"
            ),
            "status_semantics": (
                "PASS=no actionable finding; "
                "FINDING=evidence valid but behaviour/problem found; "
                "WARNING=sampled low-level observation only; "
                "FAIL=evidence/replay gate failed"
            ),
            "mcap_is_authority": True,
            "agent_output_is_derived": True,
        },
    }
    return _fit_json_budget(brief, max_bytes)
"""
    text = replace_function(
        text,
        "_build_agent_brief",
        "_build_gui_manifest",
        new_brief,
        "v2:agent-brief",
    )

    new_gui = """
def _build_gui_manifest(
    inspect: Mapping[str, object],
    triage: Mapping[str, object],
    diagnosis: Mapping[str, object],
    timeline_path: Path,
    slice_path: Path,
) -> dict[str, object]:
    return {
        "schema": GUI_MANIFEST_SCHEMA,
        "status": diagnosis.get("status"),
        "analysis_profile": diagnosis.get("analysis_profile"),
        "diagnosis_status": diagnosis.get("diagnosis_status"),
        "evidence_status": diagnosis.get("evidence_status"),
        "behavior_status": diagnosis.get("behavior_status"),
        "capture": {
            "status": diagnosis.get("capture_status"),
            "integrity": diagnosis.get("capture_integrity"),
            "topics": inspect.get("topics"),
        },
        "evidence_compiler_policy": diagnosis.get("evidence_compiler_policy"),
        "priority_evidence": diagnosis.get("priority_evidence"),
        "analysis_handoff": diagnosis.get("analysis_handoff"),
        "incidents": triage.get("incidents"),
        "tracks": [
            "pose.x_m",
            "pose.y_m",
            "pose.yaw_rad",
            "pose.covariance_trace",
            "navigation_progress",
            "requested_motion.v_mps",
            "constrained_motion.v_mps",
            "safety_decision",
            "l9_constraints",
        ],
        "artifacts": {
            "timeline_ndjson": timeline_path.name,
            "interesting_slice_ndjson": slice_path.name,
            "diagnosis_json": "diagnosis.json",
            "replay_json": (
                "replay_result.json"
                if diagnosis.get("replay_status") != "NOT_APPLICABLE"
                else None
            ),
        },
    }
"""
    text = replace_function(
        text,
        "_build_gui_manifest",
        "_build_evidence_index",
        new_gui,
        "v2:gui-manifest",
    )

    new_index = """
def _build_evidence_index(
    reader: McapReader,
    destination: Path,
    artifact_paths: Sequence[Path],
    diagnosis: Mapping[str, object],
) -> dict[str, object]:
    artifacts = {
        path.name: {
            "sha256": _sha256_file(path),
            "size_bytes": path.stat().st_size,
        }
        for path in artifact_paths
    }
    payload: dict[str, object] = {
        "schema": EVIDENCE_INDEX_SCHEMA,
        "run_id": destination.name,
        "created_at_utc": time.strftime(
            "%Y-%m-%dT%H:%M:%SZ", time.gmtime()
        ),
        "status": diagnosis.get("status"),
        "analysis_profile": diagnosis.get("analysis_profile"),
        "diagnosis_status": diagnosis.get("diagnosis_status"),
        "evidence_status": diagnosis.get("evidence_status"),
        "behavior_status": diagnosis.get("behavior_status"),
        "authority_capture": {
            "path": str(reader.path.resolve()),
            "sha256": reader.sha256(),
            "container": "MCAP",
        },
        "evidence_compiler_policy": diagnosis.get("evidence_compiler_policy"),
        "priority_evidence": diagnosis.get("priority_evidence"),
        "analysis_handoff": diagnosis.get("analysis_handoff"),
        "artifacts": artifacts,
    }
    payload["evidence_sha256"] = _payload_sha256(
        payload, "evidence_sha256"
    )
    return payload
"""
    return replace_function(
        text,
        "_build_evidence_index",
        "_source_hints",
        new_index,
        "v2:evidence-index",
    )


def patch_legacy_test_hub(text: str) -> str:
    text = text.replace(
        '"first_live_incident": replay.get("first_live_incident"),',
        '"first_live_observation": replay.get("first_live_observation"),',
    )
    text = text.replace(
        '"physical_root_cause": replay.get("physical_root_cause"),',
        '"physical_evidence": replay.get("physical_evidence"),',
    )
    return text


def patch_test_hub_next(text: str) -> str:
    text = replace_once(
        text,
        """        "behavior_status": base.get("behavior_status"),
        "behavior": behavior,
""",
        """        "behavior_status": base.get("behavior_status"),
        "evidence_compiler_policy": base.get("evidence_compiler_policy"),
        "priority_evidence": base.get("priority_evidence"),
        "analysis_handoff": base.get("analysis_handoff"),
        "behavior": behavior,
""",
        "next:agent-contract",
    )
    text = replace_once(
        text,
        """        "diagnostic_coverage_status": diagnostic_coverage.get("status"),
        "diagnostic_schema_drift_detected": diagnostic_coverage.get("schema_drift_detected"),
""",
        """        "diagnostic_coverage_status": diagnostic_coverage.get("status"),
        "diagnostic_schema_drift_detected": diagnostic_coverage.get("schema_drift_detected"),
        "priority_evidence": base.get("priority_evidence"),
        "root_cause_inferred": False,
""",
        "next:return-contract",
    )
    return text


def patch_diagnostic_contracts(text: str) -> str:
    old = """    elif any(token in low for token in ("trust", "confidence", "observability", "coverage", "ratio", "margin")):
        role = "QUALITY"
        unit = "ratio"
        checks = checks + ("FINITE_RANGE", "UNIT_INTERVAL_CANDIDATE")
    elif any(token in low for token in ("count", "errors", "rejections", "revision", "generation", "candidate_id", "sequence")):
        role = "COUNTER"
        checks = checks + ("NONNEGATIVE", "MONOTONICITY_OBSERVATION")
"""
    new = """    elif any(token in low for token in ("count", "errors", "rejections", "revision", "generation", "candidate_id", "sequence")):
        # Counter semantics must win before substring-based quality inference:
        # "generation" contains "ratio" but is a generation counter.
        role = "COUNTER"
        checks = checks + ("NONNEGATIVE", "MONOTONICITY_OBSERVATION")
    elif any(token in low for token in ("trust", "confidence", "observability", "coverage", "ratio", "margin")):
        role = "QUALITY"
        unit = "ratio"
        checks = checks + ("FINITE_RANGE", "UNIT_INTERVAL_CANDIDATE")
"""
    return replace_once(text, old, new, "contracts:generation-counter")


def patch_mcap_capture(text: str) -> str:
    old = """        raw_lidar_loss_within_tolerance = bool(
            raw_lidar_missing_count
            and raw_lidar_loss_fraction <= float(self._config.max_raw_lidar_missing_fraction)
            and raw_lidar_max_consecutive_missing <= self._config.max_consecutive_raw_lidar_missing
        )
"""
    new = """        raw_lidar_loss_within_tolerance = bool(
            not raw_lidar_missing_count
            or (
                raw_lidar_loss_fraction <= float(self._config.max_raw_lidar_missing_fraction)
                and raw_lidar_max_consecutive_missing
                <= self._config.max_consecutive_raw_lidar_missing
            )
        )
"""
    return replace_once(text, old, new, "capture:zero-loss-tolerance")


TEST_CONTENT = r"""from __future__ import annotations

from v3.diagnostic_contracts import infer_field_contract
from v3.replay import _physical_evidence
from v3.test_hub_analysis import Incident, _priority_evidence_candidate
from v3.test_hub_v2 import _build_diagnosis
from v3.test_hub_motion_quality import analyze_motion_quality_ticks
from v3.test_hub_localization_quality import analyze_localization_quality_ticks


def test_priority_evidence_is_explicitly_noncausal() -> None:
    result = _priority_evidence_candidate(
        (
            Incident(
                "timing-1",
                "HIGH",
                "TIMING",
                10,
                123,
                "Runtime",
                "TICK_INTERVAL_OUTLIER",
                {"delta_ms": 120.0},
            ),
        )
    )
    assert result["claim_class"] == "HEURISTIC_FINDING"
    assert result["causal_claim"] is False
    assert result["evidence_strength"] == "HEURISTIC"
    assert "confidence" not in result


def test_direct_production_fault_is_policy_verdict_not_root_cause() -> None:
    result = _priority_evidence_candidate(
        (
            Incident(
                "fault-1",
                "CRITICAL",
                "PRODUCTION_FAULT",
                11,
                124,
                "L11",
                "L11_ERROR",
                {},
            ),
        )
    )
    assert result["claim_class"] == "POLICY_VERDICT"
    assert result["causal_claim"] is False
    assert result["evidence_strength"] == "DIRECT"


def test_optional_device_health_is_not_physical_explanation() -> None:
    observation = {
        "reason": "PERSON_GEOMETRY_DEGRADED",
        "evidence": {
            "device_health": [
                {
                    "device_id": "PERSON_DETECTOR_FRONT",
                    "state": "DEGRADED",
                    "reason": "PERSON_GEOMETRY_DEGRADED",
                }
            ]
        },
    }
    result = _physical_evidence({}, observation, None)
    assert result["causal_claim"] is False
    assert result["evidence_strength"] == "INSUFFICIENT"
    assert result["observation"] is None
    assert "cause" not in result


def test_critical_device_health_stays_an_indicator_only() -> None:
    observation = {
        "reason": "ENCODER_DEGRADED",
        "evidence": {
            "device_health": [
                {
                    "device_id": "WHEEL_ENCODERS",
                    "state": "DEGRADED",
                    "reason": "ENCODER_DEGRADED",
                }
            ]
        },
    }
    result = _physical_evidence({}, observation, None)
    assert result["claim_class"] == "HEURISTIC_FINDING"
    assert result["causal_claim"] is False
    assert result["evidence_strength"] == "INDICATOR"
    assert "cause" not in result


def test_generation_is_a_counter_not_a_ratio() -> None:
    field = infer_field_contract("generation")
    assert field.semantic_role == "COUNTER"
    assert field.unit is None
    assert "MONOTONICITY_OBSERVATION" in field.generic_checks


def test_quality_findings_are_declared_noncausal() -> None:
    # Empty input intentionally exercises INSUFFICIENT_DATA. Claim semantics
    # are part of the evidence contract even when no threshold can be evaluated.
    motion, _segments = analyze_motion_quality_ticks([])
    localization, _events = analyze_localization_quality_ticks([])
    for payload in (motion, localization):
        assert payload["claim_policy"]["finding_class"] == "HEURISTIC_FINDING"
        assert payload["claim_policy"]["causal_claim"] is False
        assert payload["claim_policy"]["root_cause_inferred"] is False
        assert payload["claim_policy"]["causal_analysis_owner"] == "ANALYZER_LLM"


def test_diagnosis_contract_hands_causality_to_analyzer_llm() -> None:
    inspect = {
        "analysis_profile": {"name": "FORENSIC_LOW_LEVEL"},
        "structure": {"valid": True},
        "final_event": {
            "status": "PASS",
            "integrity": {"complete": True},
        },
    }
    triage = {
        "behavior_status": "NOMINAL",
        "incidents": [],
        "priority_evidence_candidate": {
            "claim_class": "HEURISTIC_FINDING",
            "causal_claim": False,
            "evidence_strength": "HEURISTIC",
            "selection_basis": "SEVERITY_TIME_ORDER",
            "reason": "TICK_INTERVAL_OUTLIER",
            "tick_id": None,
            "layer": "Runtime",
            "kind": "TIMING",
            "evidence_ids": ["timing-1"],
        },
    }
    replay = {
        "status": "MATCH",
        "first_divergence": None,
        "first_live_observation": None,
        "physical_evidence": {
            "claim_class": "FACT",
            "causal_claim": False,
            "evidence_strength": "NONE",
            "observation": None,
            "reason": "NO_PRODUCTION_RELEVANT_LIVE_OBSERVATION_IN_SCOPE",
            "evidence": None,
        },
    }
    result = _build_diagnosis(
        inspect,
        triage,
        replay,
        None,
        replay_requested=True,
    )
    assert "root_cause" not in result
    assert "physical_root_cause" not in result
    assert result["analysis_handoff"]["root_cause_inferred"] is False
    assert result["analysis_handoff"]["causal_analysis_owner"] == "ANALYZER_LLM"
    assert result["priority_evidence"]["causal_claim"] is False
"""



def patch_motion_quality(text: str) -> str:
    text = replace_once(
        text,
        r'''        findings.append({"code": code, "severity": severity, "evidence": dict(evidence)})
''',
        r'''        findings.append({
            "code": code,
            "severity": severity,
            "claim_class": "HEURISTIC_FINDING",
            "causal_claim": False,
            "selection_basis": "DECLARED_THRESHOLD_RULE",
            "evidence": dict(evidence),
        })
''',
        "motion-quality:finding-class",
    )
    text = replace_once(
        text,
        r'''            "schema": MOTION_QUALITY_SCHEMA,
            "status": "INSUFFICIENT_DATA",
''',
        r'''            "schema": MOTION_QUALITY_SCHEMA,
            "status": "INSUFFICIENT_DATA",
            "claim_policy": {
                "finding_class": "HEURISTIC_FINDING",
                "causal_claim": False,
                "root_cause_inferred": False,
                "causal_analysis_owner": "ANALYZER_LLM",
            },
''',
        "motion-quality:insufficient-data-claim-policy",
    )
    return replace_once(
        text,
        r'''        "schema": MOTION_QUALITY_SCHEMA,
        "status": status,
''',
        r'''        "schema": MOTION_QUALITY_SCHEMA,
        "status": status,
        "claim_policy": {
            "finding_class": "HEURISTIC_FINDING",
            "causal_claim": False,
            "root_cause_inferred": False,
            "causal_analysis_owner": "ANALYZER_LLM",
        },
''',
        "motion-quality:claim-policy",
    )

def patch_localization_quality(text: str) -> str:
    text = replace_once(
        text,
        r'''        findings.append({"code": code, "severity": severity, "evidence": dict(evidence)})
''',
        r'''        findings.append({
            "code": code,
            "severity": severity,
            "claim_class": "HEURISTIC_FINDING",
            "causal_claim": False,
            "selection_basis": "DECLARED_THRESHOLD_RULE",
            "evidence": dict(evidence),
        })
''',
        "localization-quality:finding-class",
    )
    text = replace_once(
        text,
        r'''        return ({"schema": LOCALIZATION_QUALITY_SCHEMA, "status": "INSUFFICIENT_DATA", "tick_count": len(rows), "findings": []}, [])
''',
        r'''        return ({
            "schema": LOCALIZATION_QUALITY_SCHEMA,
            "status": "INSUFFICIENT_DATA",
            "claim_policy": {
                "finding_class": "HEURISTIC_FINDING",
                "causal_claim": False,
                "root_cause_inferred": False,
                "causal_analysis_owner": "ANALYZER_LLM",
            },
            "tick_count": len(rows),
            "findings": [],
        }, [])
''',
        "localization-quality:insufficient-data-claim-policy",
    )
    return replace_once(
        text,
        r'''        "schema": LOCALIZATION_QUALITY_SCHEMA,
        "status": status,
''',
        r'''        "schema": LOCALIZATION_QUALITY_SCHEMA,
        "status": status,
        "claim_policy": {
            "finding_class": "HEURISTIC_FINDING",
            "causal_claim": False,
            "root_cause_inferred": False,
            "causal_analysis_owner": "ANALYZER_LLM",
        },
''',
        "localization-quality:claim-policy",
    )



PATCHERS = {
    "v3/test_hub_analysis.py": patch_test_hub_analysis,
    "v3/replay.py": patch_replay,
    "v3/test_hub_v2.py": patch_test_hub_v2,
    "v3/test_hub.py": patch_legacy_test_hub,
    "v3/test_hub_next.py": patch_test_hub_next,
    "v3/diagnostic_contracts.py": patch_diagnostic_contracts,
    "v3/mcap_capture.py": patch_mcap_capture,
    "v3/test_hub_motion_quality.py": patch_motion_quality,
    "v3/test_hub_localization_quality.py": patch_localization_quality,
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--allow-head-mismatch", action="store_true")
    parser.add_argument("--no-tests", action="store_true")
    args = parser.parse_args()

    root = Path.cwd()
    if not (root / ".git").exists():
        raise SystemExit("Run from the r2b4 repository root.")

    head = git("rev-parse", "HEAD")
    if head != EXPECTED_HEAD and not args.allow_head_mismatch:
        raise SystemExit(
            f"HEAD mismatch: expected {EXPECTED_HEAD}, got {head}. "
            "Refusing source-blind application."
        )

    missing = [path for path in TARGETS if not (root / path).is_file()]
    if missing:
        raise SystemExit(f"Missing target files: {missing}")

    dirty = git("status", "--porcelain", "--", *TARGETS, NEW_TEST)
    if dirty:
        raise SystemExit(
            "Target files have local changes; refusing to overwrite:\n" + dirty
        )

    originals: dict[str, bytes | None] = {}
    patched: dict[str, str] = {}
    for rel, patcher in PATCHERS.items():
        path = root / rel
        originals[rel] = path.read_bytes()
        source = path.read_text(encoding="utf-8")
        patched[rel] = patcher(source)

    test_path = root / NEW_TEST
    originals[NEW_TEST] = test_path.read_bytes() if test_path.exists() else None

    assert '"root_cause_candidate": root' not in patched["v3/test_hub_analysis.py"]
    assert '"root_cause": diagnosis.get("root_cause")' not in patched["v3/test_hub_v2.py"]
    assert '"physical_root_cause": physical_root_cause' not in patched["v3/replay.py"]
    assert '"first_live_incident": first_live_incident' not in patched["v3/replay.py"]
    assert (
        "raw_lidar_missing_count\n"
        "            and raw_lidar_loss_fraction"
        not in patched["v3/mcap_capture.py"]
    )

    if args.check_only:
        print("CHECK=PASS")
        print(f"HEAD={head}")
        print("Targets:")
        for rel in PATCHERS:
            print(f"  {rel}")
        print(f"  {NEW_TEST} (new)")
        return 0

    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup = root / ".upgrade_backups" / f"testhub_evidence_compiler_{stamp}"
    backup.mkdir(parents=True, exist_ok=False)
    for rel, raw in originals.items():
        if raw is None:
            continue
        dest = backup / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(raw)

    try:
        for rel, content in patched.items():
            (root / rel).write_text(content, encoding="utf-8")
        test_path.parent.mkdir(parents=True, exist_ok=True)
        test_path.write_text(TEST_CONTENT, encoding="utf-8")

        if not args.no_tests:
            run(sys.executable, "-m", "compileall", "-q", "v3")
            run(sys.executable, "-m", "pytest", "-q", NEW_TEST)
            replay_test = root / "tests/deep/test_v3_lidar_world_replay.py"
            if replay_test.is_file():
                run(
                    sys.executable,
                    "-m",
                    "pytest",
                    "-q",
                    str(replay_test.relative_to(root)),
                )

        print("APPLY=PASS")
        print(f"BASE_HEAD={head}")
        print(f"BACKUP={backup}")
        print("Changed files:")
        for rel in PATCHERS:
            print(f"  {rel}")
        print(f"  {NEW_TEST}")
        print()
        print("Generate a fresh capture/evidence bundle next.")
        return 0

    except Exception:
        for rel, raw in originals.items():
            path = root / rel
            if raw is None:
                if path.exists():
                    path.unlink()
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(raw)
        print("APPLY=FAILED; originals restored.", file=sys.stderr)
        raise


if __name__ == "__main__":
    raise SystemExit(main())
