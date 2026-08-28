#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import ast
from pathlib import Path
from types import SimpleNamespace

from controller.behavior_motion_interface import BehaviorMotionInterface
from controller.encoder_reliability import EncoderReliabilityLayer
from controller.heading_turn_controller import HeadingTurnController
from controller.motion_qa_monitor import MotionQAMonitor
from controller.motion_semantics_engine import MotionSemanticsEngine


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _top_level_classes(relative_path: str) -> set[str]:
    source = (PROJECT_ROOT / relative_path).read_text(encoding="utf-8")
    tree = ast.parse(source)
    return {
        node.name
        for node in tree.body
        if isinstance(node, ast.ClassDef)
    }


def test_readiness_owners_are_physically_split_from_compatibility_facade():
    readiness_classes = _top_level_classes("controller/motion_readiness.py")
    assert readiness_classes == set()
    assert "BehaviorMotionInterface" in _top_level_classes(
        "controller/behavior_motion_interface.py"
    )
    assert "EncoderReliabilityLayer" in _top_level_classes(
        "controller/encoder_reliability.py"
    )
    assert "HeadingTurnController" in _top_level_classes(
        "controller/heading_turn_controller.py"
    )
    assert "MotionQAMonitor" in _top_level_classes(
        "controller/motion_qa_monitor.py"
    )
    assert "MotionSemanticsEngine" in _top_level_classes(
        "controller/motion_semantics_engine.py"
    )


def test_composition_root_imports_leaf_owners_directly():
    source = (PROJECT_ROOT / "controller/components.py").read_text(encoding="utf-8")
    assert (
        "from controller.behavior_motion_interface import BehaviorMotionInterface"
        in source
    )
    assert "from controller.encoder_reliability import EncoderReliabilityLayer" in source
    assert "from controller.heading_turn_controller import HeadingTurnController" in source
    assert "from controller.motion_qa_monitor import MotionQAMonitor" in source
    assert "from controller.motion_semantics_engine import MotionSemanticsEngine" in source
    assert "from controller.motion_readiness import" not in source


def test_direct_owner_imports_resolve_to_separate_modules():
    assert EncoderReliabilityLayer.__module__ == "controller.encoder_reliability"
    assert HeadingTurnController.__module__ == "controller.heading_turn_controller"
    assert MotionSemanticsEngine.__module__ == "controller.motion_semantics_engine"


def test_l11_monitor_remains_observer_only_and_reports_quality():
    monitor = MotionQAMonitor()
    status = monitor.update(
        semantic_status={"semantic_state": "IDLE"},
        ekf_state={"x": 0.0, "y": 0.0, "theta_deg": 0.0},
        v_target=0.0,
        omega_target=0.0,
        v_cmd=0.0,
        v_l_raw=0.0,
        v_r_raw=0.0,
        pwm_l=0.0,
        pwm_r=0.0,
        dt=0.02,
        now=1.0,
    )
    assert status["quality_state"] == "NOMINAL"
    assert status["semantic_state"] == "IDLE"
    assert monitor.last_status == status


def test_l4_heading_target_uses_public_behavior_status_without_motor_access():
    ekf = SimpleNamespace(
        get_state=lambda: {"x": 1.25, "y": -0.5, "theta_deg": 10.0}
    )
    ctrl = SimpleNamespace(ekf=ekf)
    adapter = BehaviorMotionInterface(ctrl, lambda source: bool(source))

    assert adapter.set_target_heading(95.0, source="AI") is True
    assert ctrl.behavior_motion_status["target_heading_deg"] == 95.0
    assert ctrl.motion_public_target["target_pose"] == {
        "x": 1.25,
        "y": -0.5,
        "theta_deg": 95.0,
    }
    assert not hasattr(ctrl, "motion_executor")


def test_l4_source_has_no_lower_platform_or_actuator_dependency():
    source = (
        PROJECT_ROOT / "controller/behavior_motion_interface.py"
    ).read_text(encoding="utf-8").lower()
    for forbidden in (
        "motion_executor",
        "motion_controller",
        "speed_map",
        "wheel_pi",
        "candidate_motor_output",
    ):
        assert forbidden not in source


def test_l11_source_does_not_import_control_layers():
    source = (
        PROJECT_ROOT / "controller/motion_qa_monitor.py"
    ).read_text(encoding="utf-8")
    assert "from controller." not in source
    assert "import controller." not in source
