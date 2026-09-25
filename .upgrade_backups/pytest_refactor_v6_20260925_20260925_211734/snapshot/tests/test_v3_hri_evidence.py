import json
from dataclasses import dataclass

from v3.hri_evidence import (
    HRI_EVENT_SCHEMA,
    HRI_EVENT_TOPIC,
    HriBehaviorObserver,
    HriEventJournal,
    build_hri_evidence,
    load_hri_events_for_capture,
)
from v3.mcap_reader import EVENT_TOPIC


@dataclass
class FakeMessage:
    log_time_ns: int
    sequence: int = 1


class FakeReader:
    def __init__(self, rows):
        self.rows = rows

    def iter_json_messages(self, topics=()):
        assert topics == (EVENT_TOPIC,)
        for index, row in enumerate(self.rows, 1):
            yield FakeMessage(log_time_ns=10_000 + index, sequence=index), row


def _wrapper(payload):
    return {
        "source_topic": HRI_EVENT_TOPIC,
        "monotonic_ns": payload["monotonic_ns"] + 100,
        "payload": payload,
    }


def test_journal_import_is_bounded_by_capture_window(tmp_path):
    root = tmp_path.resolve()
    journal = HriEventJournal(root / "runtime" / "hri_events.ndjson")
    journal.append("OLD", monotonic_ns=1)
    journal.append("STT_RESULT", monotonic_ns=1000, interaction_id="i1")
    journal.append("ACTION_EXECUTED", monotonic_ns=1100, interaction_id="i1", command_id="c1")
    rows = load_hri_events_for_capture(root, 1050, 1200, lookback_ns=100)
    assert [row["event_type"] for row in rows] == ["STT_RESULT", "ACTION_EXECUTED"]


def test_testhub_correlates_turn_to_captured_behavior(tmp_path):
    events = [
        {
            "schema": HRI_EVENT_SCHEMA,
            "event_type": "STT_RESULT",
            "monotonic_ns": 1000,
            "interaction_id": "i1",
            "turn_id": "t1",
            "text": "kövess",
        },
        {
            "schema": HRI_EVENT_SCHEMA,
            "event_type": "INTENT_PROPOSED",
            "monotonic_ns": 1100,
            "interaction_id": "i1",
            "turn_id": "t1",
        },
        {
            "schema": HRI_EVENT_SCHEMA,
            "event_type": "ACTION_EXECUTED",
            "monotonic_ns": 1200,
            "interaction_id": "i1",
            "turn_id": "t1",
            "command_id": "cmd-7",
            "mission_id": "mission-cmd-7",
        },
    ]
    behavior = tmp_path / "behavior_timeline.ndjson"
    behavior.write_text(
        "\n".join(
            json.dumps(row)
            for row in (
                {
                    "event_type": "BEHAVIOR_START",
                    "monotonic_ns": 1300,
                    "command_id": "cmd-7",
                    "mission_id": "mission-cmd-7",
                    "mode": "FOLLOW_PERSON",
                },
                {
                    "event_type": "MOTION_STARTED",
                    "monotonic_ns": 1500,
                    "command_id": "cmd-7",
                    "mission_id": "mission-cmd-7",
                    "mode": "FOLLOW_PERSON",
                },
            )
        )
        + "\n",
        encoding="utf-8",
    )
    summary = build_hri_evidence(
        FakeReader([_wrapper(row) for row in events]),
        tmp_path,
        behavior_timeline_path=behavior,
    )
    assert summary["event_count"] == 3
    assert summary["correlated_event_count"] == 3
    assert summary["command_count"] == 1
    assert summary["latencies"] == [
        {
            "command_id": "cmd-7",
            "action_to_behavior_start_ms": 0.0001,
            "action_to_motion_start_ms": 0.0003,
        }
    ]
    timeline = [
        json.loads(line)
        for line in (tmp_path / "hri_timeline.ndjson").read_text(encoding="utf-8").splitlines()
    ]
    assert timeline[0]["command_id"] == "cmd-7"
    assert timeline[0]["mission_id"] == "mission-cmd-7"
    assert timeline[0]["correlation"]["command_mission_match"] is True


class StatusSequenceInterface:
    def __init__(self):
        self.rows = [
            {
                "tick_id": 10,
                "mission": {"mission_id": "mission-cmd-1", "lifecycle": "ACTIVE", "stop_reason": None},
                "navigation": {"mission_id": "mission-cmd-1", "status": "IDLE", "reason": "TARGET_WAIT"},
                "safety_decision": "ALLOW",
                "safety_reason": "OK",
                "enabled": False,
            },
            {
                "tick_id": 11,
                "mission": {"mission_id": "mission-cmd-1", "lifecycle": "ACTIVE", "stop_reason": None},
                "navigation": {"mission_id": "mission-cmd-1", "status": "ACTIVE", "reason": "FOLLOW_TARGET"},
                "safety_decision": "ALLOW",
                "safety_reason": "OK",
                "enabled": True,
            },
            {
                "tick_id": 12,
                "mission": {"mission_id": "mission-cmd-1", "lifecycle": "ACTIVE", "stop_reason": None},
                "navigation": {"mission_id": "mission-cmd-1", "status": "INVALIDATED", "reason": "TARGET_UNAVAILABLE"},
                "safety_decision": "STOP",
                "safety_reason": "TARGET_UNAVAILABLE",
                "enabled": False,
            },
            {
                "tick_id": 13,
                "mission": {"mission_id": "mission-cmd-1", "lifecycle": "IDLE", "stop_reason": "DONE"},
                "navigation": {"mission_id": "mission-cmd-1", "status": "IDLE", "reason": "DONE"},
                "safety_decision": "STOP",
                "safety_reason": "DONE",
                "enabled": False,
            },
        ]
        self.index = 0

    def read(self, resource):
        assert resource == "v3.status"
        row = self.rows[min(self.index, len(self.rows) - 1)]
        self.index += 1
        return row


def test_live_behavior_observer_is_read_only_and_emits_bounded_feedback(tmp_path):
    import time

    interface = StatusSequenceInterface()
    journal = HriEventJournal((tmp_path / "hri.ndjson").resolve())
    feedback = []
    observer = HriBehaviorObserver(
        interface, journal, feedback_sink=lambda text, fields: feedback.append((text, dict(fields))),
        poll_s=0.001, max_watch_s=0.2,
    )
    observer.observe(
        interaction_id="i1", turn_id="t1", session_id="s1",
        action_name="v3.command.follow_person", command_id="cmd-1", mission_id="mission-cmd-1",
    )
    deadline = time.monotonic() + 0.5
    rows = []
    while time.monotonic() < deadline:
        if journal.path.is_file():
            rows = [json.loads(line) for line in journal.path.read_text(encoding="utf-8").splitlines()]
            if any(row.get("event_type") == "BEHAVIOR_ENDED" for row in rows):
                break
        time.sleep(0.005)
    observer.close()
    kinds = [row["event_type"] for row in rows]
    assert "BEHAVIOR_MISSION_STATE" in kinds
    assert "BEHAVIOR_NAVIGATION_STATE" in kinds
    assert "BEHAVIOR_SAFETY_STATE" in kinds
    assert "BEHAVIOR_ACTUATION_STATE" in kinds
    assert "BEHAVIOR_ENDED" in kinds
    assert [text for text, _fields in feedback] == [
        "Követlek.",
        "Megálltam, jelenleg nem tudlak biztonságosan követni.",
    ]
