"""Generic offline behavior evidence derived from finished R2B4 MCAP captures.

The analyzer is deliberately read-only and owns no production, replay, hardware,
lifecycle or safety authority.  It correlates captured command, L5 mission, L6
navigation, requested/constrained motion, safety and existing Test Hub incidents
into logical ACTIVE mission episodes.
"""

from __future__ import annotations

import json
import math
from collections import Counter
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

from .mcap_reader import McapReader, TICK_TOPIC
from .test_hub_analysis import _summarize_tick

BEHAVIOR_SCHEMA = "R2B4_TEST_HUB_BEHAVIOR_V1"
BEHAVIOR_SUMMARY_NAME = "behavior_summary.json"
BEHAVIOR_EPISODES_NAME = "behavior_episodes.ndjson"
BEHAVIOR_TIMELINE_NAME = "behavior_timeline.ndjson"

_SAFETY_STATES = ("ALLOW", "STOP", "FAULT")


def _mapping(value: object) -> Mapping[str, object]:
    return value if isinstance(value, Mapping) else {}


def _sequence(value: object) -> Sequence[object]:
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return value
    return ()


def _finite(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _text(value: object) -> str | None:
    return value if isinstance(value, str) and value else None


def _json_value(value: object) -> object:
    """Copy already-captured JSON values without interpreting their semantics."""
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return [_json_value(item) for item in value]
    return str(value)


def _layers(tick: Mapping[str, object]) -> Mapping[str, object]:
    expected = _mapping(tick.get("expected"))
    return _mapping(expected.get("layers"))


def _command_snapshot(tick: Mapping[str, object]) -> dict[str, object]:
    inputs = _mapping(tick.get("inputs"))
    command = _mapping(inputs.get("command"))
    command_id = _text(command.get("command_id"))
    mode = _text(command.get("mode"))
    expiry_tick = command.get("expiry_tick")
    return {
        "id": command_id,
        "mode": mode,
        "goal": _json_value(command.get("goal") if "goal" in command else []),
        "expiry_tick": expiry_tick if isinstance(expiry_tick, int) else None,
    }


def _triage_incidents(triage: Mapping[str, object] | None) -> tuple[Mapping[str, object], ...]:
    if not isinstance(triage, Mapping):
        return ()
    return tuple(item for item in _sequence(triage.get("incidents")) if isinstance(item, Mapping))


def _incident_ids_for_range(
    incidents: Sequence[Mapping[str, object]], start_tick: int, end_tick: int
) -> tuple[str, ...]:
    result: list[str] = []
    for incident in incidents:
        tick_id = incident.get("tick_id")
        if not isinstance(tick_id, int) or not start_tick <= tick_id <= end_tick:
            continue
        incident_id = incident.get("id")
        if not isinstance(incident_id, str) or not incident_id:
            incident_id = incident.get("incident_id")
        if isinstance(incident_id, str) and incident_id and incident_id not in result:
            result.append(incident_id)
    return tuple(result)


def _timeline_event(
    event_type: str,
    *,
    tick_id: int,
    monotonic_ns: int,
    command_id: str | None,
    mission_id: str | None,
    mode: str | None,
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    row: dict[str, object] = {
        "row_type": "event",
        "event_type": event_type,
        "tick_id": tick_id,
        "monotonic_ns": monotonic_ns,
        "command_id": command_id,
        "mission_id": mission_id,
        "mode": mode,
    }
    if extra:
        row.update(extra)
    return row


@dataclass(frozen=True, slots=True)
class BehaviorEpisode:
    """One continuous ACTIVE mission segment in a finished capture."""

    episode_id: str
    command_id: str | None
    mission_id: str
    mode: str | None
    command: Mapping[str, object]
    start_tick: int
    end_tick: int
    start_monotonic_ns: int
    end_monotonic_ns: int
    duration_s: float
    end_reason: str
    navigation: Mapping[str, object]
    motion: Mapping[str, object]
    safety: Mapping[str, int]
    path_length_m: float
    incident_ids: tuple[str, ...]
    findings: tuple[Mapping[str, object], ...]
    correlation: Mapping[str, object]

    def as_dict(self) -> dict[str, object]:
        return {
            "episode_id": self.episode_id,
            "command_id": self.command_id,
            "mission_id": self.mission_id,
            "mode": self.mode,
            "command": dict(self.command),
            "start_tick": self.start_tick,
            "end_tick": self.end_tick,
            "start_monotonic_ns": self.start_monotonic_ns,
            "end_monotonic_ns": self.end_monotonic_ns,
            "duration_s": self.duration_s,
            "end_reason": self.end_reason,
            "navigation": dict(self.navigation),
            "motion": dict(self.motion),
            "safety": dict(self.safety),
            "path_length_m": self.path_length_m,
            "incident_ids": list(self.incident_ids),
            "findings": [dict(item) for item in self.findings],
            "correlation": dict(self.correlation),
        }


@dataclass(slots=True)
class _EpisodeBuilder:
    episode_id: str
    mission_id: str
    mode: str | None
    command: dict[str, object]
    start_tick: int
    start_monotonic_ns: int
    end_tick: int
    end_monotonic_ns: int
    initial_progress: float | None = None
    final_progress: float | None = None
    navigation_status_counts: Counter[str] = field(default_factory=Counter)
    requested_tick_count: int = 0
    constrained_zero_tick_count: int = 0
    safety_counts: Counter[str] = field(default_factory=Counter)
    path_length_m: float = 0.0
    previous_pose: tuple[float, float] | None = None
    navigation_mission_ids: set[str] = field(default_factory=set)
    mismatch_first_tick: int | None = None
    mismatch_last_tick: int | None = None
    mismatch_count: int = 0

    @property
    def command_id(self) -> str | None:
        return _text(self.command.get("id"))

    def observe(
        self,
        *,
        tick_id: int,
        monotonic_ns: int,
        row: Mapping[str, object],
        l6_mission_id: str | None,
    ) -> None:
        self.end_tick = tick_id
        self.end_monotonic_ns = monotonic_ns

        progress = _finite(row.get("navigation_progress"))
        if progress is not None:
            if self.initial_progress is None:
                self.initial_progress = progress
            self.final_progress = progress

        navigation_status = _text(row.get("navigation_status"))
        if navigation_status is not None:
            self.navigation_status_counts[navigation_status] += 1

        if bool(row.get("motion_requested")):
            self.requested_tick_count += 1
        if bool(row.get("motion_requested")) and bool(row.get("constrained_to_zero")):
            self.constrained_zero_tick_count += 1

        safety = _text(row.get("safety_decision"))
        if safety in _SAFETY_STATES:
            self.safety_counts[safety] += 1

        pose = row.get("pose")
        if isinstance(pose, Mapping):
            x_m = _finite(pose.get("x_m"))
            y_m = _finite(pose.get("y_m"))
            if x_m is not None and y_m is not None:
                if self.previous_pose is not None:
                    self.path_length_m += math.hypot(
                        x_m - self.previous_pose[0], y_m - self.previous_pose[1]
                    )
                self.previous_pose = (x_m, y_m)

        if l6_mission_id is not None:
            self.navigation_mission_ids.add(l6_mission_id)
            if l6_mission_id != self.mission_id:
                self.mismatch_count += 1
                if self.mismatch_first_tick is None:
                    self.mismatch_first_tick = tick_id
                self.mismatch_last_tick = tick_id

    def finish(
        self,
        *,
        end_reason: str,
        incidents: Sequence[Mapping[str, object]],
    ) -> BehaviorEpisode:
        progress_delta = None
        if self.initial_progress is not None and self.final_progress is not None:
            progress_delta = self.final_progress - self.initial_progress

        findings: list[dict[str, object]] = []
        if self.mismatch_count:
            findings.append({
                "code": "MISSION_PLAN_ID_MISMATCH",
                "first_tick": self.mismatch_first_tick,
                "last_tick": self.mismatch_last_tick,
                "count": self.mismatch_count,
                "mission_id": self.mission_id,
                "navigation_mission_ids": sorted(self.navigation_mission_ids),
            })

        nav_ids = sorted(self.navigation_mission_ids)
        plan_match: bool | None
        if not nav_ids:
            plan_match = None
        else:
            plan_match = self.mismatch_count == 0
        expected_mission_id = (
            f"mission-{self.command_id}" if self.command_id is not None else None
        )
        command_mission_match = (
            self.mission_id == expected_mission_id
            if expected_mission_id is not None
            else None
        )

        return BehaviorEpisode(
            episode_id=self.episode_id,
            command_id=self.command_id,
            mission_id=self.mission_id,
            mode=self.mode,
            command=dict(self.command),
            start_tick=self.start_tick,
            end_tick=self.end_tick,
            start_monotonic_ns=self.start_monotonic_ns,
            end_monotonic_ns=self.end_monotonic_ns,
            duration_s=max(0.0, (self.end_monotonic_ns - self.start_monotonic_ns) / 1_000_000_000.0),
            end_reason=end_reason,
            navigation={
                "initial_progress": self.initial_progress,
                "final_progress": self.final_progress,
                "progress_delta": progress_delta,
                "status_counts": dict(sorted(self.navigation_status_counts.items())),
            },
            motion={
                "requested_tick_count": self.requested_tick_count,
                "constrained_zero_tick_count": self.constrained_zero_tick_count,
            },
            safety={state: int(self.safety_counts.get(state, 0)) for state in _SAFETY_STATES},
            path_length_m=self.path_length_m,
            incident_ids=_incident_ids_for_range(incidents, self.start_tick, self.end_tick),
            findings=tuple(findings),
            correlation={
                "command_id": self.command_id,
                "expected_mission_id": expected_mission_id,
                "mission_id": self.mission_id,
                "command_mission_match": command_mission_match,
                "navigation_mission_ids": nav_ids,
                "mission_plan_match": plan_match,
            },
        )


def _write_json(path: Path, payload: Mapping[str, object]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as handle:
        handle.write(
            json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
            + "\n"
        )
    return path


def _write_ndjson(path: Path, rows: Sequence[Mapping[str, object]]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as handle:
        for row in rows:
            handle.write(
                json.dumps(row, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
                + "\n"
            )
    return path


def build_behavior_evidence(
    reader: McapReader,
    destination: str | Path,
    *,
    triage: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """Build generic mission/behavior artifacts from captured ticks only."""

    output_dir = Path(destination)
    output_dir.mkdir(parents=True, exist_ok=True)
    incidents = _triage_incidents(triage)

    episodes: list[BehaviorEpisode] = []
    timeline: list[dict[str, object]] = []
    active: _EpisodeBuilder | None = None
    episode_number = 0

    previous_lifecycle: str | None = None
    previous_navigation_status: str | None = None
    previous_motion_requested = False
    previous_motion_blocked = False
    previous_safety: str | None = None

    def close_active(end_reason: str) -> None:
        nonlocal active
        if active is None:
            return
        episode = active.finish(end_reason=end_reason, incidents=incidents)
        episodes.append(episode)
        timeline.append(
            _timeline_event(
                "BEHAVIOR_END",
                tick_id=episode.end_tick,
                monotonic_ns=episode.end_monotonic_ns,
                command_id=episode.command_id,
                mission_id=episode.mission_id,
                mode=episode.mode,
                extra={"episode_id": episode.episode_id, "end_reason": end_reason},
            )
        )
        active = None

    for message, payload in reader.iter_json_messages(topics=(TICK_TOPIC,)):
        if not isinstance(payload, Mapping):
            continue
        row = _summarize_tick(payload, message.log_time_ns)
        tick_id = row.get("tick_id")
        if not isinstance(tick_id, int):
            tick_id = message.sequence
        monotonic_ns = row.get("monotonic_ns")
        if not isinstance(monotonic_ns, int):
            monotonic_ns = message.log_time_ns

        layers = _layers(payload)
        l5 = _mapping(layers.get("L5"))
        l6 = _mapping(layers.get("L6"))
        command = _command_snapshot(payload)
        command_id = _text(command.get("id"))
        mission_id = _text(l5.get("mission_id"))
        mode = _text(l5.get("mode")) or _text(command.get("mode"))
        lifecycle = _text(l5.get("lifecycle"))
        l6_mission_id = _text(l6.get("mission_id"))
        is_active = lifecycle == "ACTIVE" and mission_id is not None

        if previous_lifecycle is not None and lifecycle != previous_lifecycle:
            timeline.append(
                _timeline_event(
                    "MISSION_LIFECYCLE_CHANGE",
                    tick_id=tick_id,
                    monotonic_ns=monotonic_ns,
                    command_id=command_id,
                    mission_id=mission_id,
                    mode=mode,
                    extra={"from": previous_lifecycle, "to": lifecycle},
                )
            )

        if active is not None and (not is_active or mission_id != active.mission_id):
            if not is_active and previous_motion_requested:
                timeline.append(
                    _timeline_event(
                        "MOTION_STOPPED",
                        tick_id=tick_id,
                        monotonic_ns=monotonic_ns,
                        command_id=active.command_id,
                        mission_id=active.mission_id,
                        mode=active.mode,
                    )
                )
            if is_active and mission_id != active.mission_id:
                old_mission_id = active.mission_id
                close_active("MISSION_CHANGED")
                timeline.append(
                    _timeline_event(
                        "MISSION_CHANGE",
                        tick_id=tick_id,
                        monotonic_ns=monotonic_ns,
                        command_id=command_id,
                        mission_id=mission_id,
                        mode=mode,
                        extra={"from_mission_id": old_mission_id, "to_mission_id": mission_id},
                    )
                )
            else:
                close_active("MISSION_LIFECYCLE_CHANGED")

            previous_navigation_status = None
            previous_motion_requested = False
            previous_motion_blocked = False
            previous_safety = None

        if is_active and active is None:
            episode_number += 1
            active = _EpisodeBuilder(
                episode_id=f"behavior-{episode_number}",
                mission_id=mission_id,
                mode=mode,
                command=command,
                start_tick=tick_id,
                start_monotonic_ns=monotonic_ns,
                end_tick=tick_id,
                end_monotonic_ns=monotonic_ns,
            )
            timeline.append(
                _timeline_event(
                    "BEHAVIOR_START",
                    tick_id=tick_id,
                    monotonic_ns=monotonic_ns,
                    command_id=active.command_id,
                    mission_id=active.mission_id,
                    mode=active.mode,
                    extra={"episode_id": active.episode_id},
                )
            )

        if active is not None and is_active and mission_id == active.mission_id:
            active.observe(
                tick_id=tick_id,
                monotonic_ns=monotonic_ns,
                row=row,
                l6_mission_id=l6_mission_id,
            )

            navigation_status = _text(row.get("navigation_status"))
            if (
                previous_navigation_status is not None
                and navigation_status != previous_navigation_status
            ):
                timeline.append(
                    _timeline_event(
                        "NAVIGATION_STATUS_CHANGE",
                        tick_id=tick_id,
                        monotonic_ns=monotonic_ns,
                        command_id=active.command_id,
                        mission_id=active.mission_id,
                        mode=active.mode,
                        extra={"from": previous_navigation_status, "to": navigation_status},
                    )
                )
            previous_navigation_status = navigation_status

            motion_requested = bool(row.get("motion_requested"))
            if motion_requested != previous_motion_requested:
                timeline.append(
                    _timeline_event(
                        "MOTION_STARTED" if motion_requested else "MOTION_STOPPED",
                        tick_id=tick_id,
                        monotonic_ns=monotonic_ns,
                        command_id=active.command_id,
                        mission_id=active.mission_id,
                        mode=active.mode,
                    )
                )
            previous_motion_requested = motion_requested

            safety = _text(row.get("safety_decision"))
            blocked = motion_requested and (
                bool(row.get("constrained_to_zero")) or safety in {"STOP", "FAULT"}
            )
            if blocked and not previous_motion_blocked:
                blocked_by = "L9" if bool(row.get("constrained_to_zero")) else "L12"
                timeline.append(
                    _timeline_event(
                        "MOTION_BLOCKED",
                        tick_id=tick_id,
                        monotonic_ns=monotonic_ns,
                        command_id=active.command_id,
                        mission_id=active.mission_id,
                        mode=active.mode,
                        extra={"blocked_by": blocked_by},
                    )
                )
            previous_motion_blocked = blocked

            if previous_safety is not None and safety != previous_safety:
                timeline.append(
                    _timeline_event(
                        "SAFETY_CHANGE",
                        tick_id=tick_id,
                        monotonic_ns=monotonic_ns,
                        command_id=active.command_id,
                        mission_id=active.mission_id,
                        mode=active.mode,
                        extra={"from": previous_safety, "to": safety},
                    )
                )
            previous_safety = safety

        previous_lifecycle = lifecycle

    close_active("CAPTURE_END")

    episode_rows = [episode.as_dict() for episode in episodes]
    modes = Counter(
        episode.mode for episode in episodes if isinstance(episode.mode, str) and episode.mode
    )
    command_ids = {
        episode.command_id for episode in episodes if isinstance(episode.command_id, str)
    }
    mission_ids = {episode.mission_id for episode in episodes}
    safety_totals = {
        state: sum(int(episode.safety.get(state, 0)) for episode in episodes)
        for state in _SAFETY_STATES
    }
    finding_counts: Counter[str] = Counter()
    for episode in episodes:
        for finding in episode.findings:
            code = _text(finding.get("code"))
            if code is not None:
                finding_counts[code] += 1

    summary = {
        "schema": BEHAVIOR_SCHEMA,
        **({"analysis_profile": triage["analysis_profile"]} if triage and "analysis_profile" in triage else {}),
        "episode_count": len(episodes),
        "active_duration_s": sum(episode.duration_s for episode in episodes),
        "modes": dict(sorted(modes.items())),
        "unique_command_count": len(command_ids),
        "unique_mission_count": len(mission_ids),
        "episodes_with_incidents": sum(bool(episode.incident_ids) for episode in episodes),
        "total_requested_motion_ticks": sum(
            int(episode.motion.get("requested_tick_count", 0)) for episode in episodes
        ),
        "total_constrained_zero_ticks": sum(
            int(episode.motion.get("constrained_zero_tick_count", 0)) for episode in episodes
        ),
        "safety": safety_totals,
        "finding_counts": dict(sorted(finding_counts.items())),
        "correlation": {
            "primary": "command_id",
            "mission": "mission_id",
            "navigation": "NavigationPlan.mission_id",
        },
    }

    # Lifecycle transition rows can be observed one tick after the final ACTIVE
    # tick. Keep the exported event stream chronological without changing same-tick
    # insertion order.
    timeline.sort(key=lambda row: (int(row["tick_id"]), int(row["monotonic_ns"])))

    _write_json(output_dir / BEHAVIOR_SUMMARY_NAME, summary)
    _write_ndjson(output_dir / BEHAVIOR_EPISODES_NAME, episode_rows)
    _write_ndjson(output_dir / BEHAVIOR_TIMELINE_NAME, timeline)

    return {
        "schema": BEHAVIOR_SCHEMA,
        "summary": BEHAVIOR_SUMMARY_NAME,
        "episodes": BEHAVIOR_EPISODES_NAME,
        "timeline": BEHAVIOR_TIMELINE_NAME,
        "episode_count": len(episodes),
        "correlation_keys": ["command_id", "mission_id"],
    }


__all__ = [
    "BEHAVIOR_EPISODES_NAME",
    "BEHAVIOR_SCHEMA",
    "BEHAVIOR_SUMMARY_NAME",
    "BEHAVIOR_TIMELINE_NAME",
    "BehaviorEpisode",
    "build_behavior_evidence",
]
