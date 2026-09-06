#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import ast
from pathlib import Path
from types import SimpleNamespace

from controller.behavior_motion_interface import BehaviorMotionInterface
from controller.motion_qa_monitor import MotionQAMonitor


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _top_level_classes(relative_path: str) -> set[str]:
    source = (PROJECT_ROOT / relative_path).read_text(encoding="utf-8")
    tree = ast.parse(source)
    return {
        node.name
        for node in tree.body
        if isinstance(node, ast.ClassDef)
    }


def test_l4_and_l11_ownership_are_physically_split_from_motion_readiness():
    readiness_classes = _top_level_classes("controller/motion_readiness.py")
    assert "BehaviorMotionInterface" not in readiness_classes
    assert "MotionQAMonitor" not in readiness_classes
    assert "BehaviorMotionInterface" in _top_level_classes(
        "controller/behavior_motion_interface.py"
    )
    assert "MotionQAMonitor" in _top_level_classes(
        "controller/motion_qa_monitor.py"
    )


def test_composition_root_imports_leaf_owners_directly():
    source = (PROJECT_ROOT / "controller/components.py").read_text(encoding="utf-8")
    assert (
        "from controller.behavior_motion_interface import BehaviorMotionInterface"
        in source
    )
    assert "from controller.motion_qa_monitor import MotionQAMonitor" in source
    readiness_import = source.split(
        "from controller.motion_readiness import (", 1
    )[1].split(")", 1)[0]
    assert "BehaviorMotionInterface" not in readiness_import
    assert "MotionQAMonitor" not in readiness_import


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
