# R2B4_FOLLOW_PERSON_P0_V2_20260923
from __future__ import annotations
from v3_config_fixtures import configured

import json
from pathlib import Path

from v3_config_fixtures import navigation_from_control as v3_navigation_config_from_mapping
from v3.contracts import CommandMode, CommandRequest, DataField, ObstacleTrack, RobotEstimate, RollingLocalCostmap, TickContext, WorldSnapshot
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import FollowPersonEvidence, TrajectoryNavigator
from v3.test_hub_task_evidence import _navigation_configuration

ROOT = (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))

def _config():
    raw = json.loads((ROOT / "conf/vezerles.json").read_text(encoding="utf-8"))
    return v3_navigation_config_from_mapping(raw)

def _estimate(c: TickContext) -> RobotEstimate:
    covariance = tuple(0.01 if i % 6 == 0 else 0.0 for i in range(25))
    return RobotEstimate(c, "R2B4_BOOT_ROBOT_MAP", 0.0, 0.0, 0.0, 0.0, 0.0, covariance)

def _person(uid: str) -> ObstacleTrack:
    return ObstacleTrack(uid, 2.0, 0.0, 0.3, 0.0, 0.0, 0.9)

def _world(c: TickContext, *tracks: ObstacleTrack) -> WorldSnapshot:
    costmap = RollingLocalCostmap("R2B4_BOOT_ROBOT_MAP", 1, 0.1, 2.5, (), 1, 0)
    return WorldSnapshot(c, "R2B4_BOOT_ROBOT_MAP", 1, tuple(tracks), 0, costmap)

def _mission(c: TickContext):
    return configured(MissionManager, ).evaluate(CommandRequest(c, "p0-v2-follow", CommandMode.FOLLOW_PERSON, (DataField("max_v_mps", 0.15), DataField("max_omega_rad_s", 0.30)), c.tick_id))

def test_follow_evidence_exposes_locked_uid_and_config():
    config = _config().navigation; nav = configured(TrajectoryNavigator, config); c = TickContext(1, 1_000_000_000)
    nav.evaluate(_mission(c), _estimate(c), _world(c, _person("person-7")))
    evidence = nav.follow_person_evidence
    assert isinstance(evidence, FollowPersonEvidence)
    assert evidence.state == "FOLLOW" and evidence.locked_target_uid == "person-7"
    assert evidence.target_visible is True
    assert evidence.search_yaw_tolerance_rad == config.follow_person_search_yaw_tolerance_rad

class _Reader:
    def __init__(self, configuration): self.configuration = configuration
    def first_json(self, _topic): return object(), {"configuration": self.configuration}

def test_testhub_reads_navigation_from_resolved_runtime():
    nav = {"follow_person_min_confidence": 0.6, "follow_person_retention_min_confidence": 0.45, "follow_person_align_tolerance_rad": 0.22, "follow_person_release_tolerance_rad": 0.30, "follow_person_stand_off_m": 1.05, "follow_person_distance_deadband_m": 0.15, "follow_person_min_safe_distance_m": 0.75, "follow_person_lost_hold_ns": 400_000_000, "follow_person_search_timeout_ns": 2_000_000_000, "follow_person_search_sweep_rad": 0.45, "follow_person_search_step_ns": 400_000_000, "follow_person_search_yaw_tolerance_rad": 0.08, "coverage_cell_size_m": 0.25, "coverage_max_cells": 512, "local_goal_max_age_ns": 8_000_000_000}
    world = {"person_track_max_age_ns": 500_000_000, "person_track_reacquire_max_age_ns": 2_500_000_000, "person_track_max_association_distance_m": 0.75, "person_track_max_speed_mps": 6.0}
    configuration = {"resolved_runtime": {"composition": {"live_control": {"control": {"navigation": nav, "world_model": world}}}}}
    resolved, source = _navigation_configuration(_Reader(configuration))
    assert source.startswith("CAPTURE_RUNTIME.resolved_runtime")
    assert resolved["follow_person"]["minimum_confidence"] == 0.6
    assert resolved["person_tracking"]["reacquire_max_age_ns"] == 2_500_000_000
