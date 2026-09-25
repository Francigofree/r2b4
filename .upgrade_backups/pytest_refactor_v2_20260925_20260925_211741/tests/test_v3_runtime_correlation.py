from __future__ import annotations

from dataclasses import dataclass

from v3.test_hub_runtime_correlation import slow_tick_correlation


@dataclass
class Message:
    sequence: int
    log_time_ns: int
    publish_time_ns: int


class FakeReader:
    def __init__(self, rows):
        self.rows = rows

    def iter_json_messages(self, **kwargs):
        topics = tuple(kwargs.get("topics") or self.rows)
        for topic in topics:
            yield from self.rows.get(topic, ())


def sample(device, sequence, captured):
    return {
        "device_id": device,
        "kind": "sample",
        "sequence": sequence,
        "captured_monotonic_ns": captured,
        "values": [],
    }


def tick(tick_id, start, camera_seq, map_rev, candidate):
    payload = {
        "tick_id": tick_id,
        "monotonic_ns": start,
        "inputs": {
            "raw_devices": {
                "samples": [
                    sample("WHEEL_ENCODERS", tick_id + 1, start),
                    sample("BNO055_IMU", tick_id + 1, start),
                    sample("RPLIDAR_C1", tick_id + 1, start),
                    sample("CAMERA_FRONT", camera_seq, start),
                    sample("PERSON_DETECTOR_FRONT", tick_id + 1, start),
                ]
            }
        },
        "expected": {
            "layers": {
                "L4": {"map_revision": map_rev, "local_costmap": {"revision": map_rev}},
                "L6": {
                    "trajectory_candidates": [
                        {
                            "candidate_id": candidate,
                            "v_mps": 0.12,
                            "omega_rad_s": 0.15,
                            "collision": False,
                            "total_score": 0.7,
                            "min_clearance_m": 0.4,
                        }
                    ]
                },
                "L7": {
                    "trajectory": {
                        "candidate_id": candidate,
                        "v_mps": 0.12,
                        "omega_rad_s": 0.15,
                    }
                },
            }
        },
    }
    return Message(tick_id, start, start), payload


def test_slow_tick_correlation_is_association_not_causation():
    rows = {
        "/r2b4/tick": [
            tick(0, 0, 1, 1, "a"),
            tick(1, 20_000_000, 2, 2, "b"),
            tick(2, 55_000_000, 2, 2, "b"),
            tick(3, 75_000_000, 2, 2, "b"),
        ],
        "/r2b4/runtime": [
            (Message(0, 0, 0), {"configuration": {"tick_period_ns": 20_000_000}})
        ],
        "/r2b4/raw_lidar": [
            (Message(7, 30_000_000, 30_000_000), {"revision": 7, "scan_end_monotonic_ns": 30_000_000})
        ],
    }
    result = slow_tick_correlation(FakeReader(rows))

    assert result["status"] == "PASS"
    assert result["claim_policy"]["root_cause_inferred"] is False
    assert result["time_semantics"]["per_layer_duration_available"] is False
    assert result["captured_runtime_target"]["status"] == "PROVEN"
    assert result["captured_runtime_target"]["period_ns"] == 20_000_000
    assert result["period_summary"]["over_reference_50hz_count"] == 1

    top = result["top_slow_intervals"][0]
    assert top["preceding_tick_id"] == 1
    assert top["following_tick_id"] == 2
    assert top["period_ms"] == 35.0
    assert "camera_sequence_changed" in top["observed_features"]
    assert "l4_map_revision_changed" in top["observed_features"]
    assert "raw_lidar_scan_end_during_interval" in top["observed_features"]

    assoc = next(row for row in result["associations"] if row["feature"] == "camera_sequence_changed")
    assert assoc["causal_claim"] is False
    assert assoc["with_feature"]["period_mean_ms"] == 35.0


def test_tick_gap_is_not_interpreted_as_long_runtime_period():
    result = slow_tick_correlation(
        FakeReader(
            {
                "/r2b4/tick": [
                    tick(10, 100, 1, 1, "a"),
                    tick(12, 30_000_100, 2, 2, "b"),
                ]
            }
        )
    )
    assert result["status"] == "UNAVAILABLE"
    assert result["evidence_scope"]["interval_count"] == 0
    assert result["evidence_scope"]["skipped_tick_gap_interval_count"] == 1


def test_50hz_reference_is_not_fake_captured_target():
    result = slow_tick_correlation(
        FakeReader(
            {
                "/r2b4/tick": [
                    tick(0, 0, 1, 1, "a"),
                    tick(1, 25_000_000, 1, 1, "a"),
                ]
            }
        )
    )
    assert result["reference_50hz"]["period_ns"] == 20_000_000
    assert result["captured_runtime_target"]["status"] == "NOT_CAPTURED"
    assert result["captured_runtime_target"]["period_ns"] is None
