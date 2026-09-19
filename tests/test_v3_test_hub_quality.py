from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from v3 import test_hub_next
from v3 import test_hub_portable as portable
from v3.test_hub_next import run_default

from v3.test_hub_motion_quality import (
    MOTION_QUALITY_SCHEMA,
    analyze_motion_quality_ticks,
    compare_motion_quality_sources,
)
from v3.test_hub_localization_quality import (
    LOCALIZATION_QUALITY_SCHEMA,
    analyze_localization_quality_ticks,
    compare_localization_quality_sources,
)


def _fields(**values):
    return [
        {"__type__": "DataField", "key": key, "value": value}
        for key, value in values.items()
    ]


def _sample(kind: str, tick: int, **values):
    return {
        "__type__": "DeviceSample",
        "device_id": "test-device",
        "kind": kind,
        "sequence": tick,
        "captured_monotonic_ns": 1_000_000_000 + tick * 20_000_000,
        "values": _fields(**values),
    }


def _tick(
    tick: int,
    *,
    requested_v: float = 0.20,
    requested_omega: float = 0.0,
    allowed_v: float | None = None,
    allowed_omega: float | None = None,
    actual_v: float = 0.20,
    actual_omega: float = 0.0,
    left_target: float = 0.20,
    right_target: float = 0.20,
    left_measured: float = 0.20,
    right_measured: float = 0.20,
    left_delta: float = 0.004,
    right_delta: float = 0.004,
    imu_omega: float = 0.0,
    imu_yaw: float = 0.0,
    x: float | None = None,
    y: float = 0.0,
    yaw: float = 0.0,
    saturated: bool = False,
    matcher_degenerate: bool = False,
    matcher_timed_out: bool = False,
    ekf_evidence=(),
    candidates=(),
    selected=None,
):
    allowed_v = requested_v if allowed_v is None else allowed_v
    allowed_omega = requested_omega if allowed_omega is None else allowed_omega
    x = tick * actual_v * 0.02 if x is None else x
    if selected is None and candidates:
        selected = candidates[0]
    layers = {
        "L3": {
            "x_m": x,
            "y_m": y,
            "yaw_rad": yaw,
            "v_mps": actual_v,
            "omega_rad_s": actual_omega,
            "covariance_5x5": [
                0.01,0,0,0,0,
                0,0.01,0,0,0,
                0,0,0.01,0,0,
                0,0,0,0.01,0,
                0,0,0,0,0.001,
            ],
        },
        "L6": {"trajectory_candidates": list(candidates)},
        "L7": {"trajectory": selected},
        "L8": {"requested_v_mps": requested_v, "requested_omega_rad_s": requested_omega},
        "L9": {"allowed_v_mps": allowed_v, "allowed_omega_rad_s": allowed_omega},
        "L10": {"left_mps": left_target, "right_mps": right_target},
        "L11": {"left_normalized": 0.3, "right_normalized": 0.3, "saturated": saturated},
        "L12": {"enabled": True, "safety_decision": "ALLOW"},
    }
    samples = [
        _sample(
            "wheel_velocity", tick,
            left_mps=left_measured,
            right_mps=right_measured,
            trust=1.0,
            left_measurement_trust=1.0,
            right_measurement_trust=1.0,
            left_distance_delta_m=left_delta,
            right_distance_delta_m=right_delta,
            raw_left_distance_m=left_delta * (tick + 1),
            raw_right_distance_m=right_delta * (tick + 1),
            rejection_code="NONE",
            left_read_error_delta=0,
            right_read_error_delta=0,
            left_invalid_alert_delta=0,
            right_invalid_alert_delta=0,
            left_quadrature_rejection_delta=0,
            right_quadrature_rejection_delta=0,
        ),
        _sample(
            "ekf_heading", tick,
            yaw_rad=imu_yaw,
            omega_rad_s=imu_omega,
            confidence=1.0,
            calibration=3,
            omega_confidence=1.0,
            omega_calibration=3,
        ),
        _sample(
            "lidar_matcher_diagnostics", tick,
            tracking_ready=True,
            matcher_timed_out=matcher_timed_out,
            matcher_degenerate=matcher_degenerate,
            matcher_runtime_ms=25.0,
            matcher_queue_delay_ms=2.0,
            matcher_confidence=0.8,
            inlier_ratio=0.8,
            robust_rmse_m=0.05,
            sector_coverage=0.8,
            observability_score=0.8,
            ambiguity_margin=0.8,
        ),
    ]
    return {
        "tick_id": tick,
        "monotonic_ns": 1_000_000_000 + tick * 20_000_000,
        "inputs": {"sensor_samples": samples},
        "expected": {"layers": layers},
        "tick_evidence": list(ekf_evidence),
    }


def _candidate(candidate_id: str, v: float, omega: float, total: float):
    return {
        "candidate_id": candidate_id,
        "v_mps": v,
        "omega_rad_s": omega,
        "collision": False,
        "min_clearance_m": 0.5,
        "progress_score": 0.8,
        "smoothness_score": 0.9,
        "novelty_score": 0.7,
        "total_score": total,
    }


def test_motion_quality_detects_persistent_right_wheel_error_and_tags_behavior():
    ticks = []
    for tick in range(80):
        ticks.append(_tick(tick, right_measured=0.14, actual_v=0.17))
    behavior = ({"episode_id": "ep-1", "mission_id": "mission-1", "mode": "EXPLORE", "start_tick": 0, "end_tick": 79},)
    summary, segments = analyze_motion_quality_ticks(ticks, behavior_episodes=behavior)
    assert summary["schema"] == MOTION_QUALITY_SCHEMA
    assert summary["status"] == "WARN"
    assert summary["wheel_tracking"]["right"]["mae"] == pytest.approx(0.06)
    assert any(item["code"] == "RIGHT_WHEEL_TRACKING_ERROR_HIGH" for item in summary["findings"])
    assert segments
    assert segments[0]["episode_id"] == "ep-1"
    assert segments[0]["mission_id"] == "mission-1"


def test_motion_quality_reports_planner_churn_and_candidate_scores():
    a = _candidate("trajectory-a", 0.20, 0.0, 0.9)
    b = _candidate("trajectory-b", 0.18, 0.2, 0.8)
    ticks = []
    for tick in range(30):
        selected = a if tick % 2 == 0 else b
        ticks.append(_tick(tick, candidates=(a, b), selected=selected))
    summary, _ = analyze_motion_quality_ticks(ticks)
    planner = summary["planner"]
    assert planner["selected_candidate_changes"] >= 20
    assert planner["selected_min_clearance_m"]["count"] == 30
    assert planner["best_vs_second_score_margin"]["mean"] == pytest.approx(0.1)


def test_localization_quality_detects_encoder_distance_mismatch_and_imu_divergence():
    ticks = []
    for tick in range(100):
        # Velocity integrates to 0.004 m/tick, while raw distance claims 0.003 m.
        ticks.append(_tick(
            tick,
            left_measured=0.20,
            right_measured=0.20,
            left_delta=0.003,
            right_delta=0.003,
            imu_omega=0.25,
        ))
    summary, _ = analyze_localization_quality_ticks(
        ticks,
        runtime_configuration={"physics": {"nyomtav_szelesseg_m": 0.3557}},
    )
    assert summary["schema"] == LOCALIZATION_QUALITY_SCHEMA
    codes = {item["code"] for item in summary["findings"]}
    assert "ENCODER_LEFT_DISTANCE_VELOCITY_MISMATCH" in codes
    assert "ENCODER_RIGHT_DISTANCE_VELOCITY_MISMATCH" in codes
    assert "ENCODER_IMU_OMEGA_DIVERGENCE" in codes
    assert summary["configuration"]["track_width_m"] == pytest.approx(0.3557)


def test_localization_quality_uses_ekf_gate_evidence_and_pose_jump():
    ticks = []
    for tick in range(20):
        evidence = ({
            "__type__": "EkfUpdateEvidence",
            "update_type": "YAW",
            "innovation": [0.2],
            "nis": 34.0,
            "threshold": 35.0,
            "accepted": tick % 3 != 0,
        },)
        x = tick * 0.004
        if tick == 12:
            x += 0.5
        ticks.append(_tick(tick, x=x, ekf_evidence=evidence))
    summary, events = analyze_localization_quality_ticks(
        ticks,
        runtime_configuration={"track_width_m": 0.3557},
    )
    codes = {item["code"] for item in summary["findings"]}
    assert "EKF_REJECTION_BURST" in codes
    assert "EKF_NIS_NEAR_GATE" in codes
    assert "LOCALIZATION_POSE_JUMP" in codes
    assert summary["status"] == "ALERT"
    assert events


def test_quality_compare_reads_portable_evidence(tmp_path: Path):
    before = tmp_path / "before"; after = tmp_path / "after"
    before.mkdir(); after.mkdir()
    motion_before = {
        "status": "WARN",
        "tracking": {"allowed_to_actual_linear": {"mae": 0.05}, "allowed_to_actual_angular": {"mae": 0.2}},
        "wheel_tracking": {"left": {"mae": 0.04}, "right": {"mae": 0.06}},
        "smoothness": {"linear_jerk_mps3": {"p95_abs": 2.0}, "angular_jerk_rad_s3": {"p95_abs": 3.0}},
        "planner": {"selected_candidate_changes": 10, "replan_count": 20},
    }
    motion_after = json.loads(json.dumps(motion_before))
    motion_after["tracking"]["allowed_to_actual_linear"]["mae"] = 0.02
    motion_after["wheel_tracking"]["right"]["mae"] = 0.02
    (before / "motion_quality.json").write_text(json.dumps(motion_before), encoding="utf-8")
    (after / "motion_quality.json").write_text(json.dumps(motion_after), encoding="utf-8")

    loc_before = {
        "status": "WARN",
        "cross_sensor": {"encoder_minus_imu_omega_rad_s": {"mae": 0.2}},
        "lidar_matcher": {"timeout_ratio": 0.1, "degenerate_ratio": 0.2},
        "ekf": {"covariance": {"trace_growth_ratio": 4.0}},
        "anomaly_events": {"count": 3},
    }
    loc_after = json.loads(json.dumps(loc_before))
    loc_after["cross_sensor"]["encoder_minus_imu_omega_rad_s"]["mae"] = 0.05
    loc_after["anomaly_events"]["count"] = 0
    (before / "localization_quality.json").write_text(json.dumps(loc_before), encoding="utf-8")
    (after / "localization_quality.json").write_text(json.dumps(loc_after), encoding="utf-8")

    motion = compare_motion_quality_sources(before, after)
    localization = compare_localization_quality_sources(before, after)
    assert motion["delta"]["linear_tracking_mae_mps"] == pytest.approx(-0.03)
    assert motion["delta"]["right_wheel_mae_mps"] == pytest.approx(-0.04)
    assert localization["delta"]["encoder_imu_omega_mae_rad_s"] == pytest.approx(-0.15)
    assert localization["delta"]["anomaly_event_count"] == pytest.approx(-3.0)


def test_no_motion_or_sensor_evidence_is_insufficient_data():
    tick = {
        "tick_id": 0,
        "monotonic_ns": 1_000_000_000,
        "inputs": {},
        "expected": {"layers": {}},
        "tick_evidence": [],
    }
    motion, _ = analyze_motion_quality_ticks([tick])
    localization, _ = analyze_localization_quality_ticks([tick])
    assert motion["status"] == "INSUFFICIENT_DATA"
    assert localization["status"] == "INSUFFICIENT_DATA"


def test_quality_is_emitted_by_behavior_upgraded_test_hub(monkeypatch, tmp_path):
    ticks = [
        _tick(tick, right_measured=0.14, actual_v=0.17)
        for tick in range(80)
    ]

    class FakeReader:
        def iter_json_messages(self, *, topics):
            del topics
            for payload in ticks:
                tick_id = payload["tick_id"]
                monotonic_ns = payload["monotonic_ns"]
                yield SimpleNamespace(
                    sequence=tick_id,
                    log_time_ns=monotonic_ns,
                ), payload

        def first_json(self, topic):
            del topic
            return None

        def sha256(self):
            return "0" * 64

    reader = FakeReader()
    capture_path = tmp_path / "quality-test.mcap"
    capture_path.write_bytes(b"offline-quality-fixture")
    destination = tmp_path / "quality.evidence"

    def fake_diagnose_once(_capture, output_dir, _replay_mode):
        output_dir.mkdir(parents=True, exist_ok=False)
        return {
            "status": "PASS",
            "diagnosis_status": "PASS",
            "evidence_status": "PASS",
            "behavior_status": "PASS",
            "replay_status": "MATCH",
        }

    def fake_behavior(_reader, output_dir, *, triage):
        del triage
        (output_dir / "behavior_summary.json").write_text(
            json.dumps({"status": "PASS"}) + "\n",
            encoding="utf-8",
        )
        (output_dir / "behavior_episodes.ndjson").write_text(
            json.dumps(
                {
                    "episode_id": "episode-1",
                    "mission_id": "mission-1",
                    "mode": "EXPLORE",
                    "start_tick": 0,
                    "end_tick": 79,
                }
            )
            + "\n",
            encoding="utf-8",
        )
        (output_dir / "behavior_timeline.ndjson").write_text("", encoding="utf-8")
        return {
            "schema": "R2B4_TEST_HUB_BEHAVIOR_V1",
            "summary": "behavior_summary.json",
            "episodes": "behavior_episodes.ndjson",
            "timeline": "behavior_timeline.ndjson",
            "episode_count": 1,
            "correlation_keys": ["command_id", "mission_id"],
        }

    def fake_build_run_view(_capture, *, hz, output_path, triage):
        del hz, triage
        output_path.write_text(
            json.dumps({"row_type": "header", "effective_incidents": []}) + "\n",
            encoding="utf-8",
        )
        return {
            "effective_incidents": [],
            "data_coverage": {},
            "phases": [],
            "suppressed_agent_noise": [],
        }

    def fake_write_lidar_summary(_capture, output_path, *, reader):
        del reader
        output_path.write_text("", encoding="utf-8")
        return {"path": str(output_path), "scan_count": 0}, {}

    def fake_write_manifest(output_dir, _capture, **_kwargs):
        artifacts = sorted(
            path.name for path in output_dir.iterdir() if path.is_file()
        )
        path = output_dir / "portable_manifest.json"
        path.write_text(
            json.dumps({"artifacts": artifacts}, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return path

    monkeypatch.setattr(test_hub_next, "McapReader", lambda _capture: reader)
    monkeypatch.setattr(test_hub_next, "_diagnose_once", fake_diagnose_once)
    monkeypatch.setattr(test_hub_next, "analyze_capture", lambda _reader: {})
    monkeypatch.setattr(test_hub_next, "build_behavior_evidence", fake_behavior)
    monkeypatch.setattr(test_hub_next, "build_run_view", fake_build_run_view)
    monkeypatch.setattr(
        test_hub_next,
        "write_incident_slices",
        lambda *_args, **_kwargs: [],
    )
    monkeypatch.setattr(
        test_hub_next,
        "write_lidar_summary",
        fake_write_lidar_summary,
    )
    monkeypatch.setattr(
        test_hub_next,
        "write_raw_lidar_incident_slices",
        lambda *_args, **_kwargs: [],
    )
    monkeypatch.setattr(
        test_hub_next,
        "write_portable_manifest",
        fake_write_manifest,
    )

    result = run_default(
        capture_path,
        output_dir=destination,
        replay_mode="incident",
        replay_sweep_enabled=False,
    )

    for name in (
        "behavior_summary.json",
        "behavior_episodes.ndjson",
        "behavior_timeline.ndjson",
        "motion_quality.json",
        "motion_quality_segments.ndjson",
        "localization_quality.json",
        "localization_events.ndjson",
        "agent_view.json",
        "portable_manifest.json",
    ):
        assert (destination / name).is_file()

    assert result["motion_quality_status"] != "ERROR"
    assert result["localization_quality_status"] != "ERROR"

    agent = json.loads(
        (destination / "agent_view.json").read_text(encoding="utf-8")
    )
    assert agent["quality"]["motion"]["summary"] == "motion_quality.json"
    assert agent["quality"]["motion"]["details"] == "motion_quality_segments.ndjson"
    assert agent["quality"]["localization"]["summary"] == "localization_quality.json"
    assert agent["quality"]["localization"]["events"] == "localization_events.ndjson"
    assert agent["quality"]["motion"]["status"] == result["motion_quality_status"]
    assert (
        agent["quality"]["localization"]["status"]
        == result["localization_quality_status"]
    )


def test_testhub_pytest_scope_executes_quality_tests(monkeypatch, tmp_path):
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout="pass", stderr="")

    monkeypatch.setattr(portable.subprocess, "run", fake_run)

    result = portable.run_pytest(tmp_path, scope="testhub")

    assert result["status"] == "PASS"
    assert "tests/test_v3_test_hub_quality.py" in result["command"]
    assert calls
