"""Host-side human/robot interaction evidence for R2B4.

This module has no command, mission, safety, runtime or motor authority. Voice
writes a tiny append-only journal; the passive capture sidecar imports a bounded
recent slice as ``v3.hri_event`` observations. MCAP remains the evidence
authority used by Test Hub.
"""

from __future__ import annotations

import fcntl
import json
import os
import threading
import time
from collections import Counter
from collections.abc import Mapping, Sequence
from pathlib import Path

HRI_EVENT_SCHEMA = "R2B4_HRI_EVENT_V1"
HRI_TEST_HUB_SCHEMA = "R2B4_TEST_HUB_HRI_V1"
HRI_EVENT_TOPIC = "v3.hri_event"
HRI_JOURNAL_NAME = "hri_events.ndjson"
HRI_TIMELINE_NAME = "hri_timeline.ndjson"
HRI_SUMMARY_NAME = "hri_summary.json"
DEFAULT_LOOKBACK_NS = 60_000_000_000
DEFAULT_MAX_IMPORT_EVENTS = 512
DEFAULT_MAX_JOURNAL_BYTES = 8 * 1024 * 1024


def _json_value(value: object) -> object:
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return [_json_value(item) for item in value]
    return str(value)


def _text(value: object) -> str | None:
    return value if isinstance(value, str) and value else None


def _event_time(row: Mapping[str, object]) -> int | None:
    value = row.get("monotonic_ns")
    return value if isinstance(value, int) and not isinstance(value, bool) and value >= 0 else None


class HriEventJournal:
    """Small best-effort host journal; failures never grant or alter robot authority."""

    def __init__(self, path: str | Path, *, max_bytes: int = DEFAULT_MAX_JOURNAL_BYTES) -> None:
        target = Path(path)
        if not target.is_absolute():
            raise ValueError("HRI journal path must be absolute")
        if not isinstance(max_bytes, int) or isinstance(max_bytes, bool) or max_bytes < 64 * 1024:
            raise ValueError("max_bytes must be an integer >= 65536")
        self.path = target
        self.max_bytes = max_bytes

    def append(self, event_type: str, **fields: object) -> dict[str, object]:
        normalized = str(event_type or "").strip().upper()
        if not normalized:
            raise ValueError("event_type must be non-empty")
        stamp = fields.pop("monotonic_ns", None)
        monotonic_ns = stamp if isinstance(stamp, int) and stamp >= 0 else time.monotonic_ns()
        row: dict[str, object] = {
            "schema": HRI_EVENT_SCHEMA,
            "event_type": normalized,
            "monotonic_ns": monotonic_ns,
        }
        for key, value in fields.items():
            if not isinstance(key, str) or not key:
                raise ValueError("HRI field names must be non-empty strings")
            if key in row:
                continue
            row[key] = _json_value(value)
        encoded = (json.dumps(row, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(self.path, os.O_CREAT | os.O_APPEND | os.O_WRONLY, 0o600)
        try:
            os.fchmod(fd, 0o600)
            fcntl.flock(fd, fcntl.LOCK_EX)
            os.write(fd, encoded)
            try:
                size = os.fstat(fd).st_size
            except OSError:
                size = 0
            if size > self.max_bytes:
                self._compact_locked(fd)
        finally:
            try:
                fcntl.flock(fd, fcntl.LOCK_UN)
            finally:
                os.close(fd)
        return row

    def _compact_locked(self, fd: int) -> None:
        """Keep the newest half of the bounded journal while holding the file lock."""
        try:
            with self.path.open("rb") as source:
                size = source.seek(0, os.SEEK_END)
                start = max(0, size - self.max_bytes // 2)
                source.seek(start)
                data = source.read()
            if start:
                newline = data.find(b"\n")
                data = data[newline + 1 :] if newline >= 0 else b""
            os.ftruncate(fd, 0)
            os.lseek(fd, 0, os.SEEK_SET)
            if data:
                os.write(fd, data)
            os.lseek(fd, 0, os.SEEK_END)
        except OSError:
            # Evidence compaction is best-effort; never break voice/control for it.
            return



class HriBehaviorObserver:
    """Read-only live observer for one accepted high-level robot action.

    It polls the canonical ``v3.status`` projection only. It owns no command,
    mission, safety or motor state and never calls execute(). A newer receipt
    supersedes an older watch because V3 has a single active mission path.
    """

    def __init__(
        self,
        interface: object,
        journal: HriEventJournal | None,
        *,
        feedback_sink=None,
        poll_s: float = 0.10,
        max_watch_s: float = 45.0,
    ) -> None:
        if not isinstance(poll_s, (int, float)) or isinstance(poll_s, bool) or poll_s <= 0:
            raise ValueError("poll_s must be positive")
        if not isinstance(max_watch_s, (int, float)) or isinstance(max_watch_s, bool) or max_watch_s <= 0:
            raise ValueError("max_watch_s must be positive")
        self._interface = interface
        self._read = getattr(interface, "read", None)
        self._journal = journal
        self._feedback_sink = feedback_sink
        self._poll_s = float(poll_s)
        self._max_watch_s = float(max_watch_s)
        self._lock = threading.Lock()
        self._generation = 0
        self._closed = threading.Event()

    def observe(
        self,
        *,
        interaction_id: str,
        turn_id: str | None,
        session_id: str | None,
        action_name: str,
        command_id: str,
        mission_id: str,
    ) -> None:
        if not all(isinstance(value, str) and value for value in (interaction_id, action_name, command_id, mission_id)):
            raise ValueError("interaction/action/command/mission ids must be non-empty strings")
        if not callable(self._read):
            self._emit(
                "BEHAVIOR_OBSERVER_UNAVAILABLE",
                interaction_id=interaction_id, turn_id=turn_id, session_id=session_id,
                action_name=action_name, command_id=command_id, mission_id=mission_id,
                reason="V3_STATUS_READ_UNAVAILABLE",
            )
            return
        with self._lock:
            self._generation += 1
            generation = self._generation
        threading.Thread(
            target=self._watch,
            kwargs={
                "generation": generation,
                "interaction_id": interaction_id,
                "turn_id": turn_id,
                "session_id": session_id,
                "action_name": action_name,
                "command_id": command_id,
                "mission_id": mission_id,
            },
            name="r2b4-hri-behavior-observer",
            daemon=True,
        ).start()

    def close(self) -> None:
        self._closed.set()
        with self._lock:
            self._generation += 1

    def _current(self, generation: int) -> bool:
        with self._lock:
            return not self._closed.is_set() and generation == self._generation

    def _emit(self, event_type: str, **fields: object) -> None:
        if self._journal is None:
            return
        try:
            self._journal.append(event_type, **fields)
        except Exception:
            return

    def _feedback(self, text: str, **fields: object) -> None:
        sink = self._feedback_sink
        if not callable(sink):
            return
        try:
            sink(text, fields)
        except Exception:
            return

    @staticmethod
    def _mapping(value: object) -> Mapping[str, object]:
        return value if isinstance(value, Mapping) else {}

    def _watch(
        self,
        *,
        generation: int,
        interaction_id: str,
        turn_id: str | None,
        session_id: str | None,
        action_name: str,
        command_id: str,
        mission_id: str,
    ) -> None:
        deadline = time.monotonic() + self._max_watch_s
        matched = False
        previous_lifecycle: str | None = None
        previous_navigation: str | None = None
        previous_safety: str | None = None
        previous_enabled: bool | None = None
        navigation_was_active = False
        active_feedback_sent = False
        common = {
            "interaction_id": interaction_id,
            "turn_id": turn_id,
            "session_id": session_id,
            "action_name": action_name,
            "command_id": command_id,
            "mission_id": mission_id,
        }
        while self._current(generation) and time.monotonic() < deadline:
            try:
                status = self._read("v3.status")
            except Exception:
                time.sleep(self._poll_s)
                continue
            if not isinstance(status, Mapping):
                time.sleep(self._poll_s)
                continue
            mission = self._mapping(status.get("mission"))
            current_mission = _text(mission.get("mission_id"))
            if current_mission != mission_id:
                if matched:
                    self._emit(
                        "BEHAVIOR_ENDED", **common,
                        observed_mission_id=current_mission,
                        reason="MISSION_REPLACED_OR_ENDED",
                    )
                    return
                time.sleep(self._poll_s)
                continue
            matched = True
            lifecycle = _text(mission.get("lifecycle"))
            if lifecycle != previous_lifecycle:
                self._emit(
                    "BEHAVIOR_MISSION_STATE", **common, lifecycle=lifecycle,
                    stop_reason=mission.get("stop_reason"), tick_id=status.get("tick_id"),
                )
                previous_lifecycle = lifecycle

            navigation = self._mapping(status.get("navigation"))
            navigation_status = _text(navigation.get("status"))
            if navigation_status != previous_navigation:
                self._emit(
                    "BEHAVIOR_NAVIGATION_STATE", **common, status=navigation_status,
                    reason=navigation.get("reason"), tick_id=status.get("tick_id"),
                )
                if navigation_status == "ACTIVE":
                    navigation_was_active = True
                    if action_name == "v3.command.follow_person" and not active_feedback_sent:
                        self._feedback("Követlek.", **common, feedback_reason="NAVIGATION_ACTIVE")
                        active_feedback_sent = True
                elif navigation_status == "INVALIDATED" and navigation_was_active:
                    self._feedback(
                        "Megálltam, jelenleg nem tudlak biztonságosan követni.",
                        **common, feedback_reason="NAVIGATION_INVALIDATED",
                    )
                previous_navigation = navigation_status

            safety = _text(status.get("safety_decision"))
            if safety != previous_safety:
                self._emit(
                    "BEHAVIOR_SAFETY_STATE", **common, safety_decision=safety,
                    safety_reason=status.get("safety_reason"), tick_id=status.get("tick_id"),
                )
                previous_safety = safety

            enabled = status.get("enabled")
            if isinstance(enabled, bool) and enabled != previous_enabled:
                self._emit(
                    "BEHAVIOR_ACTUATION_STATE", **common, enabled=enabled,
                    tick_id=status.get("tick_id"),
                )
                if action_name == "v3.command.face_person" and enabled and not active_feedback_sent:
                    self._feedback("Feléd fordulok.", **common, feedback_reason="ACTUATION_ENABLED")
                    active_feedback_sent = True
                previous_enabled = enabled

            if lifecycle in {"IDLE", "FAILED"}:
                self._emit(
                    "BEHAVIOR_ENDED", **common, lifecycle=lifecycle,
                    reason=mission.get("stop_reason"), tick_id=status.get("tick_id"),
                )
                return
            time.sleep(self._poll_s)

        if self._current(generation):
            self._emit("BEHAVIOR_OBSERVER_TIMEOUT", **common, matched=matched)


def default_hri_journal(project_root: str | Path) -> HriEventJournal:
    root = Path(project_root).resolve()
    return HriEventJournal(root / "runtime" / HRI_JOURNAL_NAME)


def load_hri_events_for_capture(
    project_root: str | Path,
    capture_started_ns: int,
    capture_finished_ns: int,
    *,
    lookback_ns: int = DEFAULT_LOOKBACK_NS,
    max_events: int = DEFAULT_MAX_IMPORT_EVENTS,
) -> tuple[dict[str, object], ...]:
    """Read a bounded time slice for passive MCAP import.

    A lookback is intentional: speech/STT/intent can precede creation of the
    movement capture. Test Hub later correlates those rows by interaction/turn/
    command identity. Invalid/truncated rows are ignored rather than affecting
    capture integrity.
    """
    if not isinstance(capture_started_ns, int) or not isinstance(capture_finished_ns, int):
        return ()
    if capture_finished_ns < capture_started_ns:
        return ()
    path = Path(project_root).resolve() / "runtime" / HRI_JOURNAL_NAME
    if not path.is_file() or path.is_symlink():
        return ()
    lower = max(0, capture_started_ns - max(0, int(lookback_ns)))
    upper = capture_finished_ns + 1_000_000_000
    rows: list[dict[str, object]] = []
    try:
        with path.open("r", encoding="utf-8") as handle:
            fcntl.flock(handle.fileno(), fcntl.LOCK_SH)
            for line in handle:
                try:
                    value = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not isinstance(value, Mapping) or value.get("schema") != HRI_EVENT_SCHEMA:
                    continue
                stamp = _event_time(value)
                if stamp is None or not lower <= stamp <= upper:
                    continue
                rows.append(dict(value))
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
    except (OSError, UnicodeError):
        return ()
    if len(rows) > max_events:
        rows = rows[-max_events:]
    return tuple(rows)


def _load_behavior_timeline(path: Path | None) -> tuple[dict[str, object], ...]:
    if path is None or not path.is_file():
        return ()
    rows: list[dict[str, object]] = []
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            value = json.loads(line)
            if isinstance(value, Mapping):
                rows.append(dict(value))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return ()
    return tuple(rows)


def _write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _write_ndjson(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n")


def build_hri_evidence(
    reader: object,
    destination: str | Path,
    *,
    behavior_timeline_path: str | Path | None = None,
) -> dict[str, object]:
    """Derive HRI timeline/correlation from MCAP EVENT_TOPIC only."""
    from .mcap_reader import EVENT_TOPIC

    output_dir = Path(destination)
    output_dir.mkdir(parents=True, exist_ok=True)
    behavior_rows = _load_behavior_timeline(
        Path(behavior_timeline_path) if behavior_timeline_path is not None else None
    )

    raw_events: list[dict[str, object]] = []
    iterator = getattr(reader, "iter_json_messages", None)
    if callable(iterator):
        for _message, wrapper in iterator(topics=(EVENT_TOPIC,)):
            if not isinstance(wrapper, Mapping) or wrapper.get("source_topic") != HRI_EVENT_TOPIC:
                continue
            payload = wrapper.get("payload")
            if not isinstance(payload, Mapping) or payload.get("schema") != HRI_EVENT_SCHEMA:
                continue
            raw_events.append(dict(payload))

    raw_events.sort(key=lambda item: (_event_time(item) or 0, str(item.get("event_type") or "")))

    interaction_to_command: dict[str, str] = {}
    turn_to_command: dict[str, str] = {}
    for row in raw_events:
        command_id = _text(row.get("command_id"))
        if command_id is None:
            continue
        interaction_id = _text(row.get("interaction_id"))
        turn_id = _text(row.get("turn_id"))
        if interaction_id is not None:
            interaction_to_command[interaction_id] = command_id
        if turn_id is not None:
            turn_to_command[turn_id] = command_id

    behavior_by_command: dict[str, dict[str, object]] = {}
    for row in behavior_rows:
        command_id = _text(row.get("command_id"))
        if command_id is None:
            continue
        current = behavior_by_command.setdefault(command_id, {})
        mission_id = _text(row.get("mission_id"))
        mode = _text(row.get("mode"))
        if mission_id is not None:
            current["mission_id"] = mission_id
        if mode is not None:
            current["mode"] = mode
        event_type = _text(row.get("event_type"))
        stamp = row.get("monotonic_ns")
        if isinstance(stamp, int) and event_type in {"BEHAVIOR_START", "MOTION_STARTED", "BEHAVIOR_END"}:
            current.setdefault(event_type.lower() + "_monotonic_ns", stamp)

    timeline: list[dict[str, object]] = []
    for source in raw_events:
        row = dict(source)
        command_id = _text(row.get("command_id"))
        if command_id is None:
            interaction_id = _text(row.get("interaction_id"))
            turn_id = _text(row.get("turn_id"))
            if interaction_id is not None:
                command_id = interaction_to_command.get(interaction_id)
            if command_id is None and turn_id is not None:
                command_id = turn_to_command.get(turn_id)
            if command_id is not None:
                row["command_id"] = command_id
        if command_id is not None:
            behavior = behavior_by_command.get(command_id, {})
            if row.get("mission_id") in {None, ""} and behavior.get("mission_id") is not None:
                row["mission_id"] = behavior["mission_id"]
            if row.get("mode") in {None, ""} and behavior.get("mode") is not None:
                row["mode"] = behavior["mode"]
            expected = f"mission-{command_id}"
            actual = _text(row.get("mission_id"))
            row["correlation"] = {
                "command_id": command_id,
                "expected_mission_id": expected,
                "mission_id": actual,
                "command_mission_match": None if actual is None else actual == expected,
            }
        timeline.append(row)

    counts = Counter(str(row.get("event_type") or "UNKNOWN") for row in timeline)
    correlated = sum(1 for row in timeline if _text(row.get("command_id")) is not None)
    interactions = {_text(row.get("interaction_id")) for row in timeline}
    interactions.discard(None)
    turns = {_text(row.get("turn_id")) for row in timeline}
    turns.discard(None)
    commands = {_text(row.get("command_id")) for row in timeline}
    commands.discard(None)

    latency_rows: list[dict[str, object]] = []
    for command_id in sorted(commands):
        action_rows = [row for row in timeline if row.get("command_id") == command_id]
        action_time = next(
            (_event_time(row) for row in action_rows if row.get("event_type") == "ACTION_EXECUTED"),
            None,
        )
        behavior = behavior_by_command.get(command_id, {})
        behavior_start = behavior.get("behavior_start_monotonic_ns")
        motion_start = behavior.get("motion_started_monotonic_ns")
        item: dict[str, object] = {"command_id": command_id}
        if isinstance(action_time, int) and isinstance(behavior_start, int):
            item["action_to_behavior_start_ms"] = (behavior_start - action_time) / 1_000_000.0
        if isinstance(action_time, int) and isinstance(motion_start, int):
            item["action_to_motion_start_ms"] = (motion_start - action_time) / 1_000_000.0
        if len(item) > 1:
            latency_rows.append(item)

    summary = {
        "schema": HRI_TEST_HUB_SCHEMA,
        "availability": "PRESENT" if timeline else "NONE",
        "event_count": len(timeline),
        "correlated_event_count": correlated,
        "interaction_count": len(interactions),
        "turn_count": len(turns),
        "command_count": len(commands),
        "event_type_counts": dict(sorted(counts.items())),
        "stop_interrupt_count": counts.get("INTERRUPT_STOP_EXECUTED", 0),
        "latencies": latency_rows,
        "timeline": HRI_TIMELINE_NAME,
        "authority": "MCAP_EVENT_TOPIC",
        "policy": "DERIVED_READ_ONLY_NO_ROBOT_AUTHORITY",
    }
    _write_ndjson(output_dir / HRI_TIMELINE_NAME, timeline)
    _write_json(output_dir / HRI_SUMMARY_NAME, summary)
    return summary


__all__ = [
    "DEFAULT_LOOKBACK_NS",
    "HRI_EVENT_SCHEMA",
    "HRI_EVENT_TOPIC",
    "HRI_JOURNAL_NAME",
    "HRI_SUMMARY_NAME",
    "HRI_TEST_HUB_SCHEMA",
    "HRI_TIMELINE_NAME",
    "HriBehaviorObserver",
    "HriEventJournal",
    "build_hri_evidence",
    "default_hri_journal",
    "load_hri_events_for_capture",
]
