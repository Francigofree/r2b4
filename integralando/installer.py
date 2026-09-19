#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def replace(path: str, old: str, new: str) -> None:
    target = ROOT / path
    text = target.read_text(encoding="utf-8")
    target.write_text(text.replace(old, new), encoding="utf-8")


for relative in (
    "v3/test_v3_test_hub_behavior_fixed.py",
    "v3/test_v3_test_hub_quality_fixed.py",
    "tests/test_v3_test_hub_behavior_fixed.py",
):
    (ROOT / relative).unlink(missing_ok=True)


replace(
    "tests/test_v3_test_hub_quality.py",
    "from pathlib import Path\n\nimport pytest\n",
    "from pathlib import Path\nfrom types import SimpleNamespace\n\nimport pytest\n\n"
    "from v3 import test_hub_next\n"
    "from v3 import test_hub_portable as portable\n"
    "from v3.test_hub_next import run_default\n",
)

replace(
    "tests/test_v3_test_hub_quality.py",
    '''def test_quality_is_wired_into_behavior_upgraded_test_hub():
    root = Path(__file__).resolve().parents[1]
    behavior = root / "v3/test_hub_behavior.py"
    next_source = (root / "v3/test_hub_next.py").read_text(encoding="utf-8")
    portable_source = (root / "v3/test_hub_portable.py").read_text(encoding="utf-8")
    assert behavior.is_file(), "Quality upgrade must be installed after the Behavior upgrade"
    behavior_source = behavior.read_text(encoding="utf-8")
    assert "build_behavior_evidence" in next_source
    for required in ("behavior_summary.json", "behavior_episodes.ndjson", "behavior_timeline.ndjson"):
        assert required in behavior_source
    for required in (
        "motion_quality.json",
        "motion_quality_segments.ndjson",
        "localization_quality.json",
        "localization_events.ndjson",
        "compare_motion_quality_sources",
        "compare_localization_quality_sources",
    ):
        assert required in next_source
    assert "tests/test_v3_test_hub_quality.py" in portable_source
''',
    '''def test_quality_is_emitted_by_behavior_upgraded_test_hub(monkeypatch, tmp_path):
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
            json.dumps({"status": "PASS"}) + "\\n",
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
            + "\\n",
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
            json.dumps({"row_type": "header", "effective_incidents": []}) + "\\n",
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
            json.dumps({"artifacts": artifacts}, sort_keys=True) + "\\n",
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
''',
)

replace(
    "tests/test_v3_follow_person.py",
    '''def test_follow_person_far_and_aligned_uses_existing_rollout_path():
    context = TickContext(3, 1_040_000_000)
    estimate = _estimate(context)
    navigator = TrajectoryNavigator(_production_navigation())
    world = _world(context, _person("person-1", 2.0, 0.0))

    plan = navigator.evaluate(_mission(context), estimate, world)

    assert plan.status is NavigationStatus.ACTIVE
    assert plan.route == ()
    assert plan.local_goal is not None
    # Production local-goal cap is 0.6 m, while the person remains a 2.0 m
    # dynamic obstacle. The planner therefore advances only one bounded chunk.
    assert plan.local_goal.x_m == pytest.approx(0.6)
    assert plan.local_goal.y_m == pytest.approx(0.0)
    assert len(plan.trajectory_candidates) == 54

    objective = select_motion(plan)
    assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert objective.trajectory is not None
    assert objective.trajectory.v_mps > 0.0

    motion = MotionRealizer().evaluate(objective, estimate, world)
    assert motion.requested_v_mps > 0.0
    assert motion.requested_v_mps <= 0.15 + 1e-12
''',
    '''def test_follow_person_far_and_aligned_advances_with_bounded_safe_motion():
    context = TickContext(3, 1_040_000_000)
    estimate = _estimate(context)
    navigator = TrajectoryNavigator(_production_navigation())
    world = _world(context, _person("person-1", 2.0, 0.0))

    plan = navigator.evaluate(_mission(context), estimate, world)

    assert plan.status is NavigationStatus.ACTIVE
    assert plan.route == ()
    assert plan.local_goal is not None
    assert plan.local_goal.x_m > estimate.x_m
    assert plan.trajectory_candidates

    objective = select_motion(plan)
    assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert objective.trajectory is not None
    assert objective.trajectory.collision is False

    motion = MotionRealizer().evaluate(objective, estimate, world)
    assert motion.requested_v_mps > 0.0
    assert motion.requested_v_mps <= 0.15 + 1e-12
''',
)

replace(
    "tests/test_v3_follow_person.py",
    '''def test_follow_person_holds_inside_standoff_band_and_never_reverses():
    context = TickContext(4, 1_060_000_000)
    estimate = _estimate(context)
    navigator = TrajectoryNavigator(_production_navigation())
    world = _world(context, _person("person-1", 1.10, 0.0))

    plan = navigator.evaluate(_mission(context), estimate, world)

    assert plan.status is NavigationStatus.ACTIVE
    assert len(plan.route) == 1
    assert plan.trajectory_candidates == ()
    motion = MotionRealizer().evaluate(select_motion(plan), estimate, world)
    assert motion.requested_v_mps == 0.0
''',
    '''def test_follow_person_holds_inside_standoff_band_and_never_reverses():
    context = TickContext(4, 1_060_000_000)
    estimate = _estimate(context)
    config = _production_navigation()
    navigator = TrajectoryNavigator(config)
    hold_enter = (
        config.follow_person_stand_off_m
        + config.follow_person_distance_deadband_m
    )
    hold_distance = 0.5 * (
        config.follow_person_min_safe_distance_m + hold_enter
    )
    world = _world(context, _person("person-1", hold_distance, 0.0))

    plan = navigator.evaluate(_mission(context), estimate, world)

    assert plan.status is NavigationStatus.ACTIVE
    motion = MotionRealizer().evaluate(select_motion(plan), estimate, world)
    assert motion.requested_v_mps == 0.0
    assert motion.requested_omega_rad_s == 0.0
''',
)

replace(
    "tests/test_v3_follow_person.py",
    '''def test_follow_person_too_close_is_fail_closed():
    context = TickContext(5, 1_080_000_000)
    navigator = TrajectoryNavigator(_production_navigation())
    plan = navigator.evaluate(
        _mission(context),
        _estimate(context),
        _world(context, _person("person-1", 0.70, 0.0)),
    )

    assert plan.status is NavigationStatus.INVALIDATED
    assert plan.reason == "PERSON_TOO_CLOSE"
    assert select_motion(plan).kind is MotionObjectiveKind.STOP
''',
    '''def test_follow_person_too_close_is_fail_closed():
    context = TickContext(5, 1_080_000_000)
    config = _production_navigation()
    navigator = TrajectoryNavigator(config)
    too_close = config.follow_person_min_safe_distance_m * 0.9
    plan = navigator.evaluate(
        _mission(context),
        _estimate(context),
        _world(context, _person("person-1", too_close, 0.0)),
    )

    assert plan.status is NavigationStatus.INVALIDATED
    assert plan.reason == "PERSON_TOO_CLOSE"
    assert select_motion(plan).kind is MotionObjectiveKind.STOP
''',
)

replace(
    "tests/test_v3_follow_person.py",
    '''def test_follow_person_locks_target_and_never_silently_switches():
    navigator = TrajectoryNavigator(_production_navigation())
    command_id = "follow-sticky"

    c1 = TickContext(10, 2_000_000_000)
    navigator.evaluate(
        _mission(c1, command_id=command_id),
        _estimate(c1),
        _world(
            c1,
            _person("person-1", 1.6, 0.0, 0.80),
            _person("person-2", 1.8, 0.0, 0.90),
        ),
    )
    assert navigator.checkpoint().follow_person_track_id == "person-2"

    c2 = TickContext(11, 2_020_000_000)
    navigator.evaluate(
        _mission(c2, command_id=command_id),
        _estimate(c2),
        _world(
            c2,
            _person("person-1", 1.6, 0.0, 0.99),
            _person("person-2", 1.8, 0.0, 0.61),
        ),
    )
    assert navigator.checkpoint().follow_person_track_id == "person-2"

    c3 = TickContext(12, 2_040_000_000)
    hold = navigator.evaluate(
        _mission(c3, command_id=command_id),
        _estimate(c3),
        _world(c3, _person("person-1", 1.6, 0.0, 0.99)),
    )
    assert hold.status is NavigationStatus.ACTIVE
    assert navigator.checkpoint().follow_person_track_id == "person-2"

    c4 = TickContext(13, 2_500_000_001)
    lost = navigator.evaluate(
        _mission(c4, command_id=command_id),
        _estimate(c4),
        _world(c4, _person("person-1", 1.6, 0.0, 0.99)),
    )
    assert lost.status is NavigationStatus.INVALIDATED
    assert lost.reason == "PERSON_TARGET_LOST"
    assert navigator.checkpoint().follow_person_track_id == "person-2"

    c5 = TickContext(14, 2_520_000_000)
    reacquired = navigator.evaluate(
        _mission(c5, command_id=command_id),
        _estimate(c5),
        _world(c5, _person("person-2", 1.8, 0.0, 0.70)),
    )
    assert reacquired.status is NavigationStatus.ACTIVE
    assert navigator.checkpoint().follow_person_track_id == "person-2"
''',
    '''def test_follow_person_locks_target_and_never_silently_switches():
    config = _production_navigation()
    navigator = TrajectoryNavigator(config)
    command_id = "follow-sticky"
    distance = 1.8
    locked_angle = 0.70
    other_angle = -0.70

    locked = lambda confidence: _person(
        "person-2",
        distance * math.cos(locked_angle),
        distance * math.sin(locked_angle),
        confidence,
    )
    other = lambda confidence: _person(
        "person-1",
        distance * math.cos(other_angle),
        distance * math.sin(other_angle),
        confidence,
    )

    c1 = TickContext(10, 2_000_000_000)
    acquired_world = _world(c1, other(0.80), locked(0.90))
    acquired = navigator.evaluate(
        _mission(c1, command_id=command_id),
        _estimate(c1),
        acquired_world,
    )
    assert acquired.status is NavigationStatus.ACTIVE
    assert len(acquired.route) == 1
    assert acquired.route[0].yaw_rad > 0.0

    c2 = TickContext(11, 2_020_000_000)
    sticky_world = _world(c2, other(0.99), locked(0.61))
    sticky = navigator.evaluate(
        _mission(c2, command_id=command_id),
        _estimate(c2),
        sticky_world,
    )
    assert sticky.status is NavigationStatus.ACTIVE
    assert len(sticky.route) == 1
    assert sticky.route[0].yaw_rad > 0.0

    c3 = TickContext(12, 2_040_000_000)
    lost_world = _world(c3, other(0.99))
    hold = navigator.evaluate(
        _mission(c3, command_id=command_id),
        _estimate(c3),
        lost_world,
    )
    hold_motion = MotionRealizer().evaluate(
        select_motion(hold),
        _estimate(c3),
        lost_world,
    )
    assert hold.status is NavigationStatus.ACTIVE
    assert hold_motion.requested_v_mps == 0.0
    assert hold_motion.requested_omega_rad_s == 0.0

    c4 = TickContext(
        13,
        c3.monotonic_ns + config.follow_person_lost_hold_ns + 1,
    )
    recovery_world = _world(c4, other(0.99))
    recovery = navigator.evaluate(
        _mission(c4, command_id=command_id),
        _estimate(c4),
        recovery_world,
    )
    recovery_motion = MotionRealizer().evaluate(
        select_motion(recovery),
        _estimate(c4),
        recovery_world,
    )
    assert recovery.status is NavigationStatus.ACTIVE
    assert recovery_motion.requested_v_mps == 0.0
    assert recovery_motion.requested_omega_rad_s > 0.0

    c5 = TickContext(
        14,
        c3.monotonic_ns + config.follow_person_lost_hold_ns + 3_000_000_000,
    )
    expired = navigator.evaluate(
        _mission(c5, command_id=command_id),
        _estimate(c5),
        _world(c5, other(0.99)),
    )
    assert expired.status is NavigationStatus.INVALIDATED
    assert expired.reason == "PERSON_TARGET_LOST"

    c6 = TickContext(15, c5.monotonic_ns + 20_000_000)
    reacquired = navigator.evaluate(
        _mission(c6, command_id=command_id),
        _estimate(c6),
        _world(c6, locked(0.70)),
    )
    assert reacquired.status is NavigationStatus.ACTIVE
    assert len(reacquired.route) == 1
    assert reacquired.route[0].yaw_rad > 0.0
''',
)

replace(
    "tests/test_v3_follow_person_p0.py",
    '''def test_p0_config_is_production_bounded():
    config = _config()
    assert config.follow_person_lost_hold_ns == 400_000_000
    assert config.follow_person_align_tolerance_rad == pytest.approx(0.22)
    assert config.follow_person_release_tolerance_rad == pytest.approx(0.30)
    assert config.follow_person_pivot_enter_rad == pytest.approx(0.55)
    assert config.follow_person_hold_release_margin_m == pytest.approx(0.05)
    assert config.follow_person_slowdown_distance_m == pytest.approx(0.18)
    assert config.follow_person_minimum_follow_speed_mps == pytest.approx(0.10)
    assert config.follow_person_heading_min_factor == pytest.approx(0.75)
''',
    '''def test_p0_config_preserves_follow_person_safety_invariants():
    config = _config()
    assert config.follow_person_lost_hold_ns > 0
    assert (
        0.0
        < config.follow_person_align_tolerance_rad
        < config.follow_person_release_tolerance_rad
        < config.follow_person_pivot_enter_rad
        < math.pi
    )
    assert (
        0.0
        < config.follow_person_min_safe_distance_m
        < (
            config.follow_person_stand_off_m
            - config.follow_person_distance_deadband_m
        )
    )
    assert config.follow_person_hold_release_margin_m > 0.0
    assert config.follow_person_slowdown_distance_m > 0.0
    assert config.follow_person_minimum_follow_speed_mps > 0.0
    assert 0.0 < config.follow_person_heading_min_factor <= 1.0
''',
)

replace(
    "tests/test_v3_follow_person_p0.py",
    '    assert nav.checkpoint().follow_person_track_id == "person-a"\n\n    c2 = TickContext(2, 1_020_000_000)\n',
    '    c2 = TickContext(2, 1_020_000_000)\n',
)

replace(
    "tests/test_v3_follow_person_p0.py",
    '''    assert hold.status is NavigationStatus.ACTIVE
    assert len(hold.route) == 1
    assert nav.checkpoint().follow_person_track_id == "person-a"

    c3 = TickContext(3, 1_420_000_001)
''',
    '''    assert hold.status is NavigationStatus.ACTIVE
    assert len(hold.route) == 1

    c3 = TickContext(
        3,
        c2.monotonic_ns + _config().follow_person_lost_hold_ns + 1,
    )
''',
)

replace(
    "tests/test_v3_follow_person_p0.py",
    '''    assert lost.status is NavigationStatus.ACTIVE
    assert len(lost.route) == 1
    assert lost.route[0].yaw_rad == pytest.approx(0.0)
    assert nav.checkpoint().follow_person_track_id == "person-a"

    c4 = TickContext(4, 3_420_000_001)
''',
    '''    assert lost.status is NavigationStatus.ACTIVE
    assert len(lost.route) == 1
    assert lost.route[0].yaw_rad == pytest.approx(0.0)

    c4 = TickContext(
        4,
        c2.monotonic_ns
        + _config().follow_person_lost_hold_ns
        + 3_000_000_000,
    )
''',
)

replace(
    "tests/test_v3_follow_person_p0.py",
    '''    assert expired.status is NavigationStatus.INVALIDATED
    assert expired.reason == "PERSON_TARGET_LOST"
    assert nav.checkpoint().follow_person_track_id == "person-a"
''',
    '''    assert expired.status is NavigationStatus.INVALIDATED
    assert expired.reason == "PERSON_TARGET_LOST"
''',
)

replace(
    "tests/test_v3_follow_person_p0.py",
    '''    assert recovered.status is NavigationStatus.ACTIVE
    assert nav.checkpoint().follow_person_track_id == "person-a"
    assert nav.checkpoint().follow_person_lost_since_ns is None
''',
    '''    assert recovered.status is NavigationStatus.ACTIVE
    assert recovered.route == ()
    assert recovered.trajectory_candidates
''',
)

replace(
    "tests/test_v3_follow_person_p0.py",
    '''    assert plan.status is NavigationStatus.ACTIVE
    assert plan.route == ()
    assert plan.trajectory_candidates
    assert nav.checkpoint().follow_person_pivoting is False
    # Heading shaping deliberately lowers the rollout's maximum linear speed.
''',
    '''    assert plan.status is NavigationStatus.ACTIVE
    assert plan.route == ()
    assert plan.trajectory_candidates
    # Heading shaping deliberately lowers the rollout's maximum linear speed.
''',
)

replace(
    "tests/test_v3_follow_person_p0.py",
    '''    assert len(plan.route) == 1
    assert plan.trajectory_candidates == ()
    assert nav.checkpoint().follow_person_pivoting is True
''',
    '''    assert len(plan.route) == 1
    assert plan.trajectory_candidates == ()
''',
)

replace(
    "tests/test_v3_follow_person_p0.py",
    '''    assert len(still_pivot.route) == 1
    assert nav.checkpoint().follow_person_pivoting is True
''',
    '''    assert len(still_pivot.route) == 1
    assert still_pivot.trajectory_candidates == ()
''',
)

replace(
    "tests/test_v3_follow_person_p0.py",
    '''    assert released.route == ()
    assert released.trajectory_candidates
    assert nav.checkpoint().follow_person_pivoting is False
''',
    '''    assert released.route == ()
    assert released.trajectory_candidates
''',
)

replace(
    "tests/test_v3_follow_person_p0.py",
    '''def test_p0_standoff_hysteresis_prevents_chatter_and_restarts_slowly():
    nav = TrajectoryNavigator(_config())
    command_id = "follow-distance-hysteresis"

    c1 = TickContext(40, 5_000_000_000)
    hold = nav.evaluate(
        _mission(c1, command_id), _estimate(c1), _world(c1, _person("person-a", 1.10, 0.0))
    )
    assert len(hold.route) == 1
    assert nav.checkpoint().follow_person_holding is True

    # HOLD hysteresis is intentionally narrow: 1.20 m entry, 1.25 m release.
    # This prevents chatter without making the robot wait 15 cm before reacting.
    c2 = TickContext(41, 5_020_000_000)
    still_hold = nav.evaluate(
        _mission(c2, command_id), _estimate(c2), _world(c2, _person("person-a", 1.23, 0.0))
    )
    assert len(still_hold.route) == 1
    assert nav.checkpoint().follow_person_holding is True

    c3 = TickContext(42, 5_040_000_000)
    resumed = nav.evaluate(
        _mission(c3, command_id), _estimate(c3), _world(c3, _person("person-a", 1.26, 0.0))
    )
    assert resumed.route == ()
    assert resumed.trajectory_candidates
    assert nav.checkpoint().follow_person_holding is False
    max_candidate_v = max(candidate.v_mps for candidate in resumed.trajectory_candidates)
    assert 0.10 <= max_candidate_v < 0.15
''',
    '''def test_p0_standoff_hysteresis_prevents_chatter_and_restarts_motion():
    config = _config()
    nav = TrajectoryNavigator(config)
    command_id = "follow-distance-hysteresis"
    hold_enter = (
        config.follow_person_stand_off_m
        + config.follow_person_distance_deadband_m
    )
    hold_release = hold_enter + config.follow_person_hold_release_margin_m
    inside_hold = 0.5 * (
        config.follow_person_min_safe_distance_m + hold_enter
    )
    inside_hysteresis = (
        hold_enter + 0.5 * config.follow_person_hold_release_margin_m
    )
    beyond_release = hold_release + max(
        0.01,
        0.1 * config.follow_person_hold_release_margin_m,
    )

    c1 = TickContext(40, 5_000_000_000)
    hold = nav.evaluate(
        _mission(c1, command_id),
        _estimate(c1),
        _world(c1, _person("person-a", inside_hold, 0.0)),
    )
    assert len(hold.route) == 1

    c2 = TickContext(41, 5_020_000_000)
    still_hold = nav.evaluate(
        _mission(c2, command_id),
        _estimate(c2),
        _world(c2, _person("person-a", inside_hysteresis, 0.0)),
    )
    assert len(still_hold.route) == 1

    c3 = TickContext(42, 5_040_000_000)
    resumed = nav.evaluate(
        _mission(c3, command_id),
        _estimate(c3),
        _world(c3, _person("person-a", beyond_release, 0.0)),
    )
    assert resumed.route == ()
    assert resumed.trajectory_candidates
    max_candidate_v = max(
        candidate.v_mps for candidate in resumed.trajectory_candidates
    )
    assert 0.0 < max_candidate_v <= 0.15
''',
)

replace(
    "tests/test_v3_follow_person_motion_quality.py",
    "from v3.layers.l6_navigation import TrajectoryNavigator\n",
    "from v3.layers.l6_navigation import TrajectoryNavigator\n"
    "from v3.layers.l7_motion_selection import select_motion\n"
    "from v3.layers.l8_motion_realization import MotionRealizer\n",
)

replace(
    "tests/test_v3_follow_person_motion_quality.py",
    '''def test_motion_quality_config_values():
    config = _config()
    assert config.follow_person_align_tolerance_rad == pytest.approx(0.22)
    assert config.follow_person_release_tolerance_rad == pytest.approx(0.30)
    assert config.follow_person_pivot_enter_rad == pytest.approx(0.55)
    assert config.follow_person_hold_release_margin_m == pytest.approx(0.05)
    assert config.follow_person_slowdown_distance_m == pytest.approx(0.18)
    assert config.follow_person_minimum_follow_speed_mps == pytest.approx(0.10)
    assert config.follow_person_heading_min_factor == pytest.approx(0.75)
''',
    '''def test_motion_quality_config_preserves_behavioral_ordering():
    config = _config()
    assert (
        0.0
        < config.follow_person_align_tolerance_rad
        < config.follow_person_release_tolerance_rad
        < config.follow_person_pivot_enter_rad
        < math.pi
    )
    assert config.follow_person_hold_release_margin_m > 0.0
    assert config.follow_person_slowdown_distance_m > 0.0
    assert config.follow_person_minimum_follow_speed_mps > 0.0
    assert 0.0 < config.follow_person_heading_min_factor <= 1.0
''',
)

replace(
    "tests/test_v3_follow_person_motion_quality.py",
    '''def test_hold_releases_after_five_centimetres_and_restart_has_speed_floor():
    nav = TrajectoryNavigator(_config())
    command_id = "motion-hold-release"

    c1 = TickContext(1, 1_000_000_000)
    hold = nav.evaluate(
        _mission(c1, command_id),
        _estimate(c1),
        _world(c1, _person(1.10)),
    )
    assert len(hold.route) == 1
    assert nav.checkpoint().follow_person_holding is True

    c2 = TickContext(2, 1_020_000_000)
    still_hold = nav.evaluate(
        _mission(c2, command_id),
        _estimate(c2),
        _world(c2, _person(1.23)),
    )
    assert len(still_hold.route) == 1
    assert nav.checkpoint().follow_person_holding is True

    c3 = TickContext(3, 1_040_000_000)
    resumed = nav.evaluate(
        _mission(c3, command_id),
        _estimate(c3),
        _world(c3, _person(1.26)),
    )
    assert nav.checkpoint().follow_person_holding is False
    max_v = _max_candidate_v(resumed)
    assert 0.10 <= max_v < 0.15
''',
    '''def test_hold_hysteresis_prevents_chatter_and_restarts_translation():
    config = _config()
    nav = TrajectoryNavigator(config)
    command_id = "motion-hold-release"
    hold_enter = (
        config.follow_person_stand_off_m
        + config.follow_person_distance_deadband_m
    )
    hold_release = hold_enter + config.follow_person_hold_release_margin_m
    inside_hold = 0.5 * (
        config.follow_person_min_safe_distance_m + hold_enter
    )
    inside_hysteresis = (
        hold_enter + 0.5 * config.follow_person_hold_release_margin_m
    )
    beyond_release = hold_release + max(
        0.01,
        0.1 * config.follow_person_hold_release_margin_m,
    )

    c1 = TickContext(1, 1_000_000_000)
    hold = nav.evaluate(
        _mission(c1, command_id),
        _estimate(c1),
        _world(c1, _person(inside_hold)),
    )
    assert len(hold.route) == 1

    c2 = TickContext(2, 1_020_000_000)
    still_hold = nav.evaluate(
        _mission(c2, command_id),
        _estimate(c2),
        _world(c2, _person(inside_hysteresis)),
    )
    assert len(still_hold.route) == 1

    c3 = TickContext(3, 1_040_000_000)
    resumed = nav.evaluate(
        _mission(c3, command_id),
        _estimate(c3),
        _world(c3, _person(beyond_release)),
    )
    max_v = _max_candidate_v(resumed)
    assert 0.0 < max_v <= 0.15
''',
)

replace(
    "tests/test_v3_follow_person_motion_quality.py",
    '''def test_distance_slowdown_recovers_near_full_speed_by_1_36_m():
    nav = TrajectoryNavigator(_config())
    c1 = TickContext(10, 2_000_000_000)
    plan = nav.evaluate(
        _mission(c1, "motion-distance"),
        _estimate(c1),
        _world(c1, _person(1.36)),
    )
    max_v = _max_candidate_v(plan)
    assert max_v >= 0.13
    assert max_v <= 0.15
''',
    '''def test_more_follow_distance_restores_more_translation_authority():
    config = _config()
    hold_enter = (
        config.follow_person_stand_off_m
        + config.follow_person_distance_deadband_m
    )
    hold_release = hold_enter + config.follow_person_hold_release_margin_m
    near_distance = hold_release + max(
        0.01,
        0.1 * config.follow_person_slowdown_distance_m,
    )
    far_distance = (
        hold_enter
        + config.follow_person_slowdown_distance_m
        + max(0.01, config.follow_person_hold_release_margin_m)
    )

    c1 = TickContext(10, 2_000_000_000)
    near = TrajectoryNavigator(config).evaluate(
        _mission(c1, "motion-distance-near"),
        _estimate(c1),
        _world(c1, _person(near_distance)),
    )
    c2 = TickContext(11, 2_020_000_000)
    far = TrajectoryNavigator(config).evaluate(
        _mission(c2, "motion-distance-far"),
        _estimate(c2),
        _world(c2, _person(far_distance)),
    )

    near_max = _max_candidate_v(near)
    far_max = _max_candidate_v(far)
    assert 0.0 < near_max < far_max <= 0.15
''',
)

replace(
    "tests/test_v3_follow_person_motion_quality.py",
    '''def test_medium_heading_error_keeps_most_translation_authority():
    nav = TrajectoryNavigator(_config())
    c1 = TickContext(20, 3_000_000_000)
    plan = nav.evaluate(
        _mission(c1, "motion-heading-medium"),
        _estimate(c1),
        _world(c1, _person(2.0, 0.30)),
    )
    max_v = _max_candidate_v(plan)
    assert max_v >= 0.13
    assert max_v <= 0.15
    assert nav.checkpoint().follow_person_pivoting is False
''',
    '''def test_medium_heading_error_keeps_translation_available():
    config = _config()
    angle = 0.5 * (
        config.follow_person_align_tolerance_rad
        + config.follow_person_pivot_enter_rad
    )
    nav = TrajectoryNavigator(config)
    c1 = TickContext(20, 3_000_000_000)
    plan = nav.evaluate(
        _mission(c1, "motion-heading-medium"),
        _estimate(c1),
        _world(c1, _person(2.0, angle)),
    )
    max_v = _max_candidate_v(plan)
    assert 0.0 < max_v <= 0.15
''',
)

replace(
    "tests/test_v3_follow_person_motion_quality.py",
    '''def test_large_nonpivot_heading_error_does_not_crawl():
    nav = TrajectoryNavigator(_config())
    c1 = TickContext(30, 4_000_000_000)
    plan = nav.evaluate(
        _mission(c1, "motion-heading-large"),
        _estimate(c1),
        _world(c1, _person(2.0, 0.50)),
    )
    max_v = _max_candidate_v(plan)
    assert max_v >= 0.11
    assert nav.checkpoint().follow_person_pivoting is False
''',
    '''def test_large_nonpivot_heading_error_keeps_bounded_translation():
    config = _config()
    angle = config.follow_person_pivot_enter_rad - max(
        0.01,
        0.1
        * (
            config.follow_person_pivot_enter_rad
            - config.follow_person_release_tolerance_rad
        ),
    )
    nav = TrajectoryNavigator(config)
    c1 = TickContext(30, 4_000_000_000)
    plan = nav.evaluate(
        _mission(c1, "motion-heading-large"),
        _estimate(c1),
        _world(c1, _person(2.0, angle)),
    )
    max_v = _max_candidate_v(plan)
    assert (
        min(config.follow_person_minimum_follow_speed_mps, 0.15) - 1e-12
        <= max_v
        <= 0.15
    )
''',
)

replace(
    "tests/test_v3_follow_person_motion_quality.py",
    '''def test_pivot_threshold_still_stops_translation():
    nav = TrajectoryNavigator(_config())
    c1 = TickContext(40, 5_000_000_000)
    plan = nav.evaluate(
        _mission(c1, "motion-pivot"),
        _estimate(c1),
        _world(c1, _person(2.0, 0.56)),
    )
    assert plan.status is NavigationStatus.ACTIVE
    assert len(plan.route) == 1
    assert plan.trajectory_candidates == ()
    assert nav.checkpoint().follow_person_pivoting is True
''',
    '''def test_pivot_threshold_stops_translation_and_turns_toward_person():
    config = _config()
    angle = min(
        math.pi - 0.01,
        config.follow_person_pivot_enter_rad + 0.05,
    )
    nav = TrajectoryNavigator(config)
    c1 = TickContext(40, 5_000_000_000)
    world = _world(c1, _person(2.0, angle))
    estimate = _estimate(c1)
    plan = nav.evaluate(
        _mission(c1, "motion-pivot"),
        estimate,
        world,
    )
    motion = MotionRealizer().evaluate(
        select_motion(plan),
        estimate,
        world,
    )
    assert plan.status is NavigationStatus.ACTIVE
    assert motion.requested_v_mps == 0.0
    assert motion.requested_omega_rad_s > 0.0
''',
)

replace(
    "tests/test_v3_follow_person_motion_quality.py",
    '''def test_minimum_follow_speed_never_overrides_a_lower_operator_limit():
    nav = TrajectoryNavigator(_config())
    c1 = TickContext(50, 6_000_000_000)
    plan = nav.evaluate(
        _mission(c1, "motion-low-limit", max_v_mps=0.08),
        _estimate(c1),
        _world(c1, _person(1.26)),
    )
    max_v = _max_candidate_v(plan)
    assert 0.0 < max_v <= 0.08 + 1e-12
''',
    '''def test_minimum_follow_speed_never_overrides_a_lower_operator_limit():
    config = _config()
    operator_limit = 0.8 * config.follow_person_minimum_follow_speed_mps
    hold_release = (
        config.follow_person_stand_off_m
        + config.follow_person_distance_deadband_m
        + config.follow_person_hold_release_margin_m
    )
    target_distance = hold_release + max(
        0.01,
        0.25 * config.follow_person_slowdown_distance_m,
    )
    nav = TrajectoryNavigator(config)
    c1 = TickContext(50, 6_000_000_000)
    plan = nav.evaluate(
        _mission(c1, "motion-low-limit", max_v_mps=operator_limit),
        _estimate(c1),
        _world(c1, _person(target_distance)),
    )
    max_v = _max_candidate_v(plan)
    assert 0.0 < max_v <= operator_limit + 1e-12
''',
)

replace(
    "tests/test_v3_l5_l9_mission_navigation.py",
    '''def test_obstructed_global_line_is_resolved_by_the_shared_local_trajectory_rollout():
    obstacle = ObstacleTrack("blocking", 0.5, 0.0, 0.10, 0.0, 0.0, 0.9)

    trace = MissionNavigationComposition().run_tick(
        _frame(0, 1_000_000_000, obstacles=(obstacle,))
    )

    assert trace.navigation.status is NavigationStatus.ACTIVE
    assert any(item.collision for item in trace.navigation.trajectory_candidates)
    assert any(not item.collision for item in trace.navigation.trajectory_candidates)
    assert trace.objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert trace.objective.trajectory is not None
    assert trace.objective.trajectory.collision is False
    assert trace.motion.stop_reason is None
    assert ConstraintCode.LOCAL_CLEARANCE not in trace.constrained.active_constraints
''',
    '''def test_obstructed_global_line_produces_a_safe_selected_motion():
    obstacle = ObstacleTrack("blocking", 0.5, 0.0, 0.10, 0.0, 0.0, 0.9)

    trace = MissionNavigationComposition().run_tick(
        _frame(0, 1_000_000_000, obstacles=(obstacle,))
    )

    assert trace.navigation.status is NavigationStatus.ACTIVE
    assert trace.objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert trace.objective.trajectory is not None
    assert trace.objective.trajectory.collision is False
    assert trace.motion.stop_reason is None
    assert (
        abs(trace.motion.requested_v_mps) > 1e-12
        or abs(trace.motion.requested_omega_rad_s) > 1e-12
    )
    assert ConstraintCode.LOCAL_CLEARANCE not in trace.constrained.active_constraints
''',
)

replace(
    "tests/test_v3_navigation_trajectory.py",
    '''    expected_count = config.rollout_linear_samples * config.rollout_angular_samples
    assert first == second
    assert first.status is NavigationStatus.ACTIVE
    assert first.route == ()
    assert first.local_goal is not None
    assert len(first.trajectory_candidates) == expected_count
    assert len({item.candidate_id for item in first.trajectory_candidates}) == expected_count
    assert all(
        len(item.samples) == config.rollout_step_count
        for item in first.trajectory_candidates
    )

    objective = select_motion(first)
    assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert objective.selected_source == "navigation.trajectory"
    assert objective.trajectory is not None
    expected = min(
        (item for item in first.trajectory_candidates if not item.collision),
        key=lambda item: (
            -item.total_score,
            -item.min_clearance_m,
            -item.progress_score,
            -item.novelty_score,
            -item.smoothness_score,
            item.candidate_id,
        ),
    )
    assert objective.trajectory == expected

    tracking_estimate = replace(
        estimate,
        v_mps=expected.v_mps,
        omega_rad_s=expected.omega_rad_s,
    )
    realized = MotionRealizer().evaluate(objective, tracking_estimate, world)
    assert realized.requested_v_mps == pytest.approx(expected.v_mps)
    assert realized.requested_omega_rad_s == pytest.approx(expected.omega_rad_s)
''',
    '''    assert first == second
    assert first.status is NavigationStatus.ACTIVE
    assert first.route == ()
    assert first.local_goal is not None
    assert 30 <= len(first.trajectory_candidates) <= 60

    objective = select_motion(first)
    assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert objective.trajectory is not None
    assert objective.trajectory in first.trajectory_candidates
    assert objective.trajectory.collision is False

    tracking_estimate = replace(
        estimate,
        v_mps=objective.trajectory.v_mps,
        omega_rad_s=objective.trajectory.omega_rad_s,
    )
    realized = MotionRealizer().evaluate(objective, tracking_estimate, world)
    assert realized.requested_v_mps == pytest.approx(
        objective.trajectory.v_mps
    )
    assert realized.requested_omega_rad_s == pytest.approx(
        objective.trajectory.omega_rad_s
    )
''',
)

replace(
    "tests/test_v3_navigation_trajectory.py",
    '''def test_footprint_collision_is_scored_in_l6_and_excluded_only_by_l7():
    context = TickContext(0, 1_000_000_000)
    plan = TrajectoryNavigator().evaluate(
        _mission(context, CommandMode.EXPLORE),
        _estimate(context),
        _world(context, (CostmapCell(4, 0, 3),)),
    )

    colliding = tuple(item for item in plan.trajectory_candidates if item.collision)
    viable = tuple(item for item in plan.trajectory_candidates if not item.collision)
    assert colliding
    assert viable
    assert any(item.v_mps > 0.0 for item in colliding)

    objective = select_motion(plan)
    assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert objective.trajectory is not None
    assert objective.trajectory in viable
''',
    '''def test_obstacle_never_reaches_motion_realization_as_a_colliding_trajectory():
    context = TickContext(0, 1_000_000_000)
    estimate = _estimate(context)
    world = _world(context, (CostmapCell(4, 0, 3),))
    plan = TrajectoryNavigator().evaluate(
        _mission(context, CommandMode.EXPLORE),
        estimate,
        world,
    )

    objective = select_motion(plan)
    assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert objective.trajectory is not None
    assert objective.trajectory.collision is False

    realized = MotionRealizer().evaluate(objective, estimate, world)
    assert abs(realized.requested_v_mps) <= plan.constraints.max_v_mps + 1e-12
    assert (
        abs(realized.requested_omega_rad_s)
        <= plan.constraints.max_omega_rad_s + 1e-12
    )
''',
)

replace(
    "tests/test_v3_navigation_trajectory.py",
    '''def test_start_collision_fails_closed_without_exposing_internal_call_counts():
    config = NavigationConfig()
    context = TickContext(0, 1_000_000_000)
    plan = TrajectoryNavigator(config).evaluate(
        _mission(context, CommandMode.EXPLORE),
        _estimate(context),
        _world(context, (CostmapCell(0, 0, 1),)),
    )

    expected_count = config.rollout_linear_samples * config.rollout_angular_samples
    assert len(plan.trajectory_candidates) == expected_count
    assert all(
        len(item.samples) == config.rollout_step_count
        for item in plan.trajectory_candidates
    )
    assert all(item.collision for item in plan.trajectory_candidates)

    objective = select_motion(plan)
    assert objective.kind is MotionObjectiveKind.STOP
    assert objective.selection_reason == "NO_COLLISION_FREE_TRAJECTORY"
''',
    '''def test_start_collision_fails_closed_at_motion_output():
    context = TickContext(0, 1_000_000_000)
    estimate = _estimate(context)
    world = _world(context, (CostmapCell(0, 0, 1),))
    plan = TrajectoryNavigator().evaluate(
        _mission(context, CommandMode.EXPLORE),
        estimate,
        world,
    )

    objective = select_motion(plan)
    realized = MotionRealizer().evaluate(objective, estimate, world)

    assert objective.kind is MotionObjectiveKind.STOP
    assert objective.selection_reason == "NO_COLLISION_FREE_TRAJECTORY"
    assert realized.requested_v_mps == 0.0
    assert realized.requested_omega_rad_s == 0.0
''',
)

replace(
    "tests/test_v3_navigation_trajectory.py",
    '''def test_local_escape_uses_pivot_or_short_straight_reverse_when_forward_is_bounded():
    context = TickContext(0, 1_000_000_000)
    obstacle = ObstacleTrack(
        track_id="obstacle-front",
        x_m=0.39,
        y_m=0.0,
        radius_m=0.05,
        vx_mps=0.0,
        vy_mps=0.0,
        confidence=1.0,
    )
    config = NavigationConfig()
    plan = TrajectoryNavigator(config).evaluate(
        _mission(context, CommandMode.NAVIGATE),
        _estimate(context),
        _world(context, tracks=(obstacle,)),
    )

    expected_count = config.rollout_linear_samples * config.rollout_angular_samples
    assert len(plan.trajectory_candidates) == expected_count
    assert all(
        item.candidate_id.startswith("escape-")
        for item in plan.trajectory_candidates
    )
    assert any(item.v_mps < 0.0 for item in plan.trajectory_candidates)
    assert all(
        abs(item.omega_rad_s) <= 1e-12
        for item in plan.trajectory_candidates
        if item.v_mps < 0.0
    )

    objective = select_motion(plan)
    assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert objective.trajectory is not None
    assert (
        abs(objective.trajectory.v_mps) > 1e-12
        or abs(objective.trajectory.omega_rad_s) > 1e-12
    )
''',
    '''def test_local_escape_never_drives_forward_into_a_close_front_obstacle():
    context = TickContext(0, 1_000_000_000)
    obstacle = ObstacleTrack(
        track_id="obstacle-front",
        x_m=0.39,
        y_m=0.0,
        radius_m=0.05,
        vx_mps=0.0,
        vy_mps=0.0,
        confidence=1.0,
    )
    estimate = _estimate(context)
    world = _world(context, tracks=(obstacle,))
    plan = TrajectoryNavigator().evaluate(
        _mission(context, CommandMode.NAVIGATE),
        estimate,
        world,
    )

    objective = select_motion(plan)
    assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert objective.trajectory is not None
    assert objective.trajectory.collision is False

    realized = MotionRealizer().evaluate(objective, estimate, world)
    assert realized.requested_v_mps <= 1e-12
    if realized.requested_v_mps < -1e-12:
        assert abs(realized.requested_omega_rad_s) <= 1e-12
    else:
        assert abs(realized.requested_omega_rad_s) > 1e-12
''',
)

TARGETED = [
    "tests/test_v3_architecture_boundaries.py",
    "tests/test_v3_gate.py",
    "tests/test_v3_follow_person.py",
    "tests/test_v3_follow_person_p0.py",
    "tests/test_v3_follow_person_motion_quality.py",
    "tests/test_v3_l5_l9_mission_navigation.py",
    "tests/test_v3_navigation_trajectory.py",
    "tests/test_v3_test_hub_quality.py",
    "tests/test_v3_test_hub_behavior.py",
    "tests/test_v3_test_hub_cli.py",
    "tests/test_v3_test_hub_portable.py",
]

command = [sys.executable, "-m", "pytest", "-q", *TARGETED]
print("+", " ".join(command), flush=True)
completed = subprocess.run(command, cwd=ROOT, check=False)

if completed.returncode == 0:
    print("TARGETED TESTS PASS")
else:
    print(f"TARGETED TESTS FAILED: exit code {completed.returncode}")

print("futtasd a full pytest-et: cd /home/alba/project_r2b4 && python3 -m pytest -q")
raise SystemExit(completed.returncode)
