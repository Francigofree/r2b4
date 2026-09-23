"""Behavioral policy and elapsed-time trends for sampled MCAP evidence.

Only recorded outputs are observed. No production logic or missing ticks are
reconstructed. The shared triage reader supplies the same captured facts as the
forensic profile, with a different scope and severity policy here.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import replace
from typing import Mapping

from .test_hub_analysis import Incident, _append_incident, _finite, _mapping

LOW_LEVEL_CATEGORIES = frozenset({
    "TICK_SEQUENCE", "TIMING", "DEVICE_HEALTH", "ADMISSION",
    "EDGE_FAULT", "PRODUCTION_FAULT",
})
TREND_WINDOW_NS = 5_000_000_000


def append_sampled_incident(items: list[Incident], item: Incident, limit: int) -> None:
    # Cap before ranking so early low-level noise cannot evict behavior findings.
    if item.category in LOW_LEVEL_CATEGORIES:
        item = replace(item, severity="WARNING")
    _append_incident(items, item, limit)


class BehavioralTrends:
    def __init__(self, hz: int):
        self.max_gap_ns = 3_000_000_000 / hz
        self.previous_ns: int | None = None
        self.previous_identity: tuple[object, ...] | None = None
        self.commands: set[str] = set()
        self.missions: set[str] = set()
        self.lifecycle_counts: Counter[str] = Counter()
        self.navigation_counts: Counter[str] = Counter()
        self.command_changes = 0
        self.lifecycle_changes = 0
        self.previous_command: str | None = None
        self.previous_lifecycle: str | None = None
        self.progress_anchor: tuple[int, float] | None = None
        self.stagnation: dict[str, object] | None = None
        self.blocked_since: int | None = None
        self.longest_blocked_span_s = 0.0
        self.blocked_samples = 0
        self.unsafe_output_samples = 0
        self.localization_samples = 0

    def observe(self, tick: Mapping[str, object], row: dict[str, object]) -> list[Incident]:
        tick_id, ns = int(row["tick_id"]), int(row["monotonic_ns"])
        layers = _mapping(_mapping(tick.get("expected")).get("layers"))
        l5, l6, l12 = (_mapping(layers.get(layer)) for layer in ("L5", "L6", "L12"))
        command = _mapping(_mapping(tick.get("inputs")).get("command"))
        command_id, mission_id = command.get("command_id"), l5.get("mission_id")
        lifecycle, mode, navigation = l5.get("lifecycle"), l5.get("mode"), row.get("navigation_status")
        identity = (mission_id, command_id, mode, lifecycle)
        gap = self.previous_ns is not None and not 0 < ns - self.previous_ns <= self.max_gap_ns
        if gap or identity != self.previous_identity:
            self.progress_anchor = None
            self.blocked_since = None
        if isinstance(command_id, str):
            self.commands.add(command_id)
        if isinstance(mission_id, str):
            self.missions.add(mission_id)
        if isinstance(lifecycle, str):
            self.lifecycle_counts[lifecycle] += 1
        if isinstance(navigation, str):
            self.navigation_counts[navigation] += 1
        self.command_changes += int(self.previous_ns is not None and command_id != self.previous_command)
        self.lifecycle_changes += int(self.previous_ns is not None and lifecycle != self.previous_lifecycle)
        self.previous_command, self.previous_lifecycle = command_id, lifecycle
        self.previous_identity, self.previous_ns = identity, ns
        row["command"] = {"command_id": command_id, "mode": command.get("mode"), "expiry_tick": command.get("expiry_tick")}
        row["mission"] = {**_mapping(row.get("mission")), "mission_id": mission_id}

        findings: list[Incident] = []

        def add(category: str, layer: str, reason: str, evidence: Mapping[str, object]) -> None:
            findings.append(Incident(
                f"behavior-{reason.lower()}-{tick_id}", "HIGH", category,
                tick_id, ns, layer, reason, dict(evidence),
            ))

        if lifecycle == "FAULT":
            add("MISSION", "L5", "MISSION_FAULT_OUTCOME", {"mission_id": mission_id, "stop_reason": l5.get("stop_reason")})
        if lifecycle == "ACTIVE" and navigation in {"NO_PATH", "INVALIDATED"}:
            add("NAVIGATION", "L6", "NAVIGATION_UNAVAILABLE", {"status": navigation, "mission_id": mission_id})
        plan_id = l6.get("mission_id")
        if lifecycle == "ACTIVE" and mission_id and plan_id and plan_id != mission_id:
            add("MISSION", "L6", "MISSION_PLAN_ID_MISMATCH", {"mission_id": mission_id, "navigation_mission_id": plan_id})

        blocked = bool(row.get("motion_requested")) and (
            bool(row.get("constrained_to_zero")) or row.get("safety_decision") in {"STOP", "FAULT"}
        )
        self.blocked_samples += int(blocked)
        if blocked:
            if self.blocked_since is None:
                self.blocked_since = ns
            self.longest_blocked_span_s = max(self.longest_blocked_span_s, (ns - self.blocked_since) / 1e9)
        else:
            self.blocked_since = None

        # Navigation progress is meaningful only within one active autonomous
        # mission. A command change or a missing time interval resets the window.
        progress = _finite(row.get("navigation_progress"))
        if lifecycle == "ACTIVE" and mode not in {None, "TELEOP", "STOP"} and navigation == "ACTIVE" and progress is not None:
            if self.progress_anchor is None or abs(progress - self.progress_anchor[1]) > 1e-4:
                self.progress_anchor = (ns, progress)
            elif ns - self.progress_anchor[0] >= TREND_WINDOW_NS:
                self.stagnation = {
                    "tick_id": tick_id, "monotonic_ns": ns,
                    "observed_span_s": (ns - self.progress_anchor[0]) / 1e9,
                    "last_progress": progress, "mission_id": mission_id,
                }
                add("NAVIGATION", "L6", "NAVIGATION_PROGRESS_STAGNATION", self.stagnation)
                self.progress_anchor = (ns, progress)
        else:
            self.progress_anchor = None

        if row.get("safety_decision") in {"STOP", "FAULT"} and (
            l12.get("enabled") is True
            or any(abs(_finite(l12.get(key)) or 0.0) > 1e-9 for key in ("left_output", "right_output"))
        ):
            self.unsafe_output_samples += 1
            add("SAFETY", "L12", "STOP_FAULT_WITH_ACTIVE_OUTPUT", {
                key: l12.get(key) for key in ("enabled", "left_output", "right_output", "safety_decision")
            })
        if _finite(_mapping(row.get("pose")).get("covariance_trace")) is not None:
            self.localization_samples += 1
        return findings

    def summary(self) -> dict[str, object]:
        return {
            "commands": {"observed_unique_count": len(self.commands), "observed_change_count": self.command_changes},
            "missions": {
                "observed_unique_count": len(self.missions),
                "lifecycle_sample_counts": dict(self.lifecycle_counts),
                "observed_lifecycle_change_count": self.lifecycle_changes,
            },
            "navigation": {"status_sample_counts": dict(self.navigation_counts), "stagnation": self.stagnation},
            "motion": {"blocked_sample_count": self.blocked_samples, "longest_observed_blocked_span_s": self.longest_blocked_span_s},
            "safety": {"unsafe_output_sample_count": self.unsafe_output_samples},
            "localization": {"sample_count": self.localization_samples},
            "trend_window_s": TREND_WINDOW_NS / 1e9,
            "maximum_contiguous_sample_gap_s": self.max_gap_ns / 1e9,
        }


def sampled_quality(triage: Mapping[str, object], kind: str) -> tuple[dict[str, object], list[object]]:
    """Publish only metrics supported by sparse samples in the quality slots."""
    categories = {"LOCALIZATION"} if kind == "localization" else {"MOTION_BLOCKED", "NAVIGATION", "MISSION"}
    findings = [dict(item) for item in triage.get("incidents", ()) if item.get("category") in categories]
    trends = _mapping(triage.get("behavioral_trends"))
    count = _mapping(trends.get("localization")).get("sample_count", 0) if kind == "localization" else _mapping(triage.get("ticks")).get("count", 0)
    summary = {
        "schema": f"R2B4_TEST_HUB_SAMPLED_{kind.upper()}_V1",
        "analysis_profile": triage.get("analysis_profile"),
        "status": "INSUFFICIENT_DATA" if count < 2 else "FINDING" if findings else "OK",
        "sample_count": count,
        "metrics": triage.get(kind),
        "trends": trends.get(kind),
        "findings": findings,
    }
    return summary, findings


def compare_sampled_quality(before: Mapping[str, object], after: Mapping[str, object]) -> dict[str, object]:
    same_profile = before.get("schema") == after.get("schema")
    return {
        "status": "OK" if same_profile else "NOT_COMPARABLE",
        "before_profile": before.get("analysis_profile"),
        "after_profile": after.get("analysis_profile"),
        "before_status": before.get("status"),
        "after_status": after.get("status"),
        "before_metrics": before.get("metrics"),
        "after_metrics": after.get("metrics"),
        "before_trends": before.get("trends"),
        "after_trends": after.get("trends"),
        "verdict_policy": "Descriptive sampled trends only; rates and observed durations may differ.",
    }
