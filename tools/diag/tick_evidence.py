"""Bounded summaries of existing, passive typed evidence in sampled tick rows.

These are observed samples, not a reconstructed sensor stream. In particular,
missing revisions or EKF updates between sampled ticks remain unobserved.
"""
from __future__ import annotations

from collections import Counter
import math

from .profiling import evidence_ref


def _number(value):
    return float(value) if type(value) in (int, float) and math.isfinite(value) else None


class _Numbers:
    def __init__(self):
        self.count = 0
        self.total = 0.0
        self.minimum = self.maximum = None

    def add(self, value):
        number = _number(value)
        if number is not None:
            self.count += 1
            self.total += number
            self.minimum = number if self.minimum is None else min(self.minimum, number)
            self.maximum = number if self.maximum is None else max(self.maximum, number)

    def as_dict(self):
        return {"count": self.count, "min": self.minimum, "max": self.maximum,
                "mean": self.total / self.count if self.count else None}


def _items(bundle, kind):
    if not bundle.has_view("ticks"):
        return
    for row in bundle.iter_view("ticks"):
        payload = row.payload if isinstance(row.payload, dict) else {}
        items = payload.get("tick_evidence")
        if not isinstance(items, list):
            continue
        for index, item in enumerate(items):
            if isinstance(item, dict) and item.get("__type__") == kind:
                yield row, item, evidence_ref(row, f"/tick_evidence/{index}").as_dict()


def wheel_feedback_summary(bundle):
    counts, admission, admission_reasons, rejection, readiness = Counter(), Counter(), Counter(), Counter(), Counter()
    trust = {side: _Numbers() for side in ("left", "right")}
    examples = {}
    for row, item, ref in _items(bundle, "WheelFeedbackEvidence"):
        counts["observed"] += 1
        payload = row.payload
        inputs = payload.get("inputs")
        source_context = inputs.get("context") if isinstance(inputs, dict) else None
        if (item.get("context") != source_context or not isinstance(source_context, dict)
                or source_context.get("tick_id") != row.tick_id
                or source_context.get("monotonic_ns") != payload.get("monotonic_ns")):
            counts["invalid_context"] += 1
            continue
        counts["valid"] += 1
        admission[str(item.get("admission_state"))] += 1
        admission_reasons[str(item.get("admission_reason"))] += 1
        rejection[str(item.get("rejection_code"))] += 1
        readiness[str((item.get("left_velocity_feedback_ready"), item.get("right_velocity_feedback_ready")))] += 1
        for field in ("measurement_stale", "measurement_timing_valid", "left_motion_live", "right_motion_live"):
            if type(item.get(field)) is bool:
                counts[field + ("_true" if item[field] else "_false")] += 1
        for side in trust:
            trust[side].add(item.get(side + "_measurement_trust"))
            target = _number(item.get(side + "_target_mps"))
            if target is not None and abs(target) > 1e-9:
                counts[side + "_nonzero_target"] += 1
                if item.get(side + "_velocity_feedback_ready") is False:
                    counts[side + "_nonzero_target_not_ready"] += 1
                    examples.setdefault(side + "_nonzero_target_not_ready", ref)
        if item.get("measurement_stale") is True:
            examples.setdefault("measurement_stale", ref)
        if item.get("error_type"):
            counts["error"] += 1
            examples.setdefault("error", ref)
    return {"counts": dict(counts), "admission_states": dict(admission), "admission_reasons": dict(admission_reasons),
            "rejection_codes": dict(rejection),
            "readiness_pairs_left_right": dict(readiness),
            "measurement_trust": {side: values.as_dict() for side, values in trust.items()},
            "first_evidence_refs": examples,
            "physical_wheel_tracking_proven": False,
            "sampling_semantics": "RECORDED_TICK_EVIDENCE_SAMPLES_ONLY"}


def ekf_updates_summary(bundle):
    groups = {}
    count = 0
    for _, item, ref in _items(bundle, "EkfUpdateEvidence"):
        count += 1
        kind = item.get("update_type")
        if not isinstance(kind, str):
            kind = "UNKNOWN"
        group = groups.setdefault(kind, {"counts": Counter(), "nis": _Numbers(), "threshold": _Numbers(),
                                        "nis_over_threshold": _Numbers(), "innovation_components": {},
                                        "first_rejected_ref": None})
        group["counts"]["observed"] += 1
        accepted = item.get("accepted")
        group["counts"]["accepted" if accepted is True else "rejected" if accepted is False else "unknown_acceptance"] += 1
        if accepted is False and group["first_rejected_ref"] is None:
            group["first_rejected_ref"] = ref
        for field in ("nis", "threshold"):
            group[field].add(item.get(field))
        nis, threshold = _number(item.get("nis")), _number(item.get("threshold"))
        if nis is not None and threshold is not None and threshold > 0:
            group["nis_over_threshold"].add(nis / threshold)
            group["counts"]["above_threshold"] += nis > threshold
        innovation = item.get("innovation")
        if isinstance(innovation, list):
            for index, value in enumerate(innovation):
                group["innovation_components"].setdefault(str(index), _Numbers()).add(value)
    return {"update_count": count, "by_update_type": {
        kind: {"counts": dict(group["counts"]),
               **{field: group[field].as_dict() for field in ("nis", "threshold", "nis_over_threshold")},
               "innovation_components": {index: values.as_dict() for index, values in group["innovation_components"].items()},
               "first_rejected_ref": group["first_rejected_ref"]}
        for kind, group in groups.items()},
        "sampling_semantics": "ONLY_UPDATES_PRESENT_ON_RECORDED_TICKS_NOT_ALL_RUNTIME_UPDATES",
        "physical_localization_accuracy_proven": False}


def command_ingress_summary(bundle):
    counts = Counter()
    timing = {field: _Numbers() for field in (
        "issue_to_reader_ns", "reader_to_observed_ns", "issue_to_observed_ns", "ttl_remaining_ns",
        "adjacent_revision_issue_gap_ns", "adjacent_revision_uncovered_issue_ns",
        "continuing_motion_command_issue_gap_ns",
        "previous_preparation_to_issue_ns", "previous_issue_to_publication_return_upper_bound_ns",
        "previous_publication_to_reader_lower_bound_ns")}
    previous = last_publication_revision = None
    examples, gaps = {}, []
    for row, item, ref in _items(bundle, "CommandIngressTiming"):
        counts["observed"] += 1
        fields = ("revision", "issued_monotonic_ns", "reader_received_ns", "observed_monotonic_ns", "expires_monotonic_ns")
        if any(type(item.get(field)) is not int or item[field] < 0 for field in fields):
            counts["invalid"] += 1
            continue
        revision, issued, received, observed, expires = (item[field] for field in fields)
        inputs = row.payload.get("inputs")
        command = inputs.get("command") if isinstance(inputs, dict) else None
        command = command if isinstance(command, dict) else {}
        command_id, mode = command.get("command_id"), command.get("mode")
        counts["valid"] += 1
        publication_fields = ("previous_publication_revision", "previous_preparation_started_ns",
                              "previous_issued_monotonic_ns", "previous_publication_returned_ns")
        if all(type(item.get(field)) is int and item[field] >= 0 for field in publication_fields):
            publication_revision, prepared, publication_issued, returned = (item[field] for field in publication_fields)
            if (prepared <= publication_issued <= returned and publication_revision < revision
                    and publication_revision != last_publication_revision):
                last_publication_revision = publication_revision
                counts["previous_publication_bounds"] += 1
                timing["previous_preparation_to_issue_ns"].add(publication_issued - prepared)
                timing["previous_issue_to_publication_return_upper_bound_ns"].add(returned - publication_issued)
                if (previous is not None and previous["revision"] == publication_revision
                        and previous["issued_monotonic_ns"] == publication_issued):
                    counts["previous_publication_reader_pairs"] += 1
                    timing["previous_publication_to_reader_lower_bound_ns"].add(max(0, previous["reader_received_ns"] - returned))
                else:
                    counts["previous_publication_reader_not_sampled"] += 1
        for field, value in (("issue_to_reader_ns", received - issued), ("reader_to_observed_ns", observed - received),
                             ("issue_to_observed_ns", observed - issued), ("ttl_remaining_ns", expires - observed)):
            timing[field].add(value)
        if received > expires:
            counts["expired_at_reader_samples"] += 1
            examples.setdefault("expired_at_reader", ref)
        if observed > expires:
            counts["expired_at_observation_samples"] += 1
            examples.setdefault("expired_at_observation", ref)
        if previous is not None and revision == previous["revision"]:
            counts["repeated_revision_samples"] += 1
            continue
        counts["observed_revision_runs"] += 1
        if previous is not None:
            delta = revision - previous["revision"]
            if delta == 1 and issued >= previous["issued_monotonic_ns"]:
                counts["adjacent_revision_pairs"] += 1
                timing["adjacent_revision_issue_gap_ns"].add(issued - previous["issued_monotonic_ns"])
                uncovered = max(0, issued - previous["expires_monotonic_ns"])
                timing["adjacent_revision_uncovered_issue_ns"].add(uncovered)
                if uncovered:
                    counts["adjacent_revision_issue_ttl_gaps"] += 1
                    examples.setdefault("adjacent_revision_issue_ttl_gap", {"previous": previous["ref"], "next": ref})
                    same_motion = (isinstance(command_id, str) and command_id == previous["command_id"]
                                   and mode == previous["mode"] and mode in (
                                       "TELEOP", "NAVIGATE", "EXPLORE", "FACE_PERSON", "FOLLOW_PERSON"))
                    if same_motion:
                        counts["continuing_motion_command_issue_ttl_gaps"] += 1
                        timing["continuing_motion_command_issue_gap_ns"].add(issued - previous["issued_monotonic_ns"])
                        examples.setdefault("continuing_motion_command_issue_ttl_gap", {"previous": previous["ref"], "next": ref})
                    if len(gaps) < 16:
                        gaps.append({"previous_revision": previous["revision"], "next_revision": revision,
                                     "issue_gap_ns": issued - previous["issued_monotonic_ns"], "uncovered_issue_ns": uncovered,
                                     "same_recorded_motion_command": same_motion,
                                     "previous_recorded_mode": previous["mode"], "next_recorded_mode": mode,
                                     "previous_ref": previous["ref"], "next_ref": ref})
            elif delta > 1:
                counts["unobserved_revisions"] += delta - 1
            else:
                counts["revision_or_issue_order_breaks"] += 1
        previous = {**{field: item[field] for field in fields}, "ref": ref, "command_id": command_id, "mode": mode}
    return {"counts": dict(counts), "timing": {field: values.as_dict() for field, values in timing.items()},
            "first_evidence_refs": examples,
            "adjacent_revision_issue_ttl_gap_examples": gaps,
            "issue_ttl_gap_examples_truncated": counts["adjacent_revision_issue_ttl_gaps"] > len(gaps),
            "sampling_semantics": "ADJACENT_REVISION_GAPS_ONLY_SKIPPED_REVISIONS_NOT_RECONSTRUCTED",
            "command_scope": "ALL_RECORDED_INTENTS_WITH_CONTINUING_MOTION_COMMAND_SUBSET_EXPLICIT_IDLE_STOP_GAPS_ARE_INCLUDED",
            "publication_timing": "PREVIOUS_REVISION_POST_REPLACE_RETURN_BOUND" if counts["previous_publication_bounds"] else "NOT_RECORDED",
            "publication_timing_semantics": "RETURN_TIMESTAMP_UPPER_BOUNDS_ATOMIC_REPLACEMENT_SCHEDULING_AFTER_REPLACE_IS_NOT_SEPARABLE",
            "producer_delay_root_cause_proven": False}
