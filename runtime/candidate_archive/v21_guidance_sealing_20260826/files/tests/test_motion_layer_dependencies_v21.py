"""Static sealing checks for V2.1 ownership and dependency boundaries."""

from __future__ import annotations

import ast
import inspect
from pathlib import Path

from controller.motion_guidance import MotionGuidance
from controller.motion_semantics_engine import MotionSemanticsEngine
from replayer.contracts import (
    LAYER_L6_INTENT_RESOLVER,
    LAYER_L7A_MOTION_GUIDANCE,
    LAYER_L8_MOTION_CONTROLLER,
    REPLAYABLE_LAYER_ORDER_V21,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _source(relative_path: str) -> str:
    return (PROJECT_ROOT / relative_path).read_text(encoding="utf-8")


def _imports(relative_path: str) -> set[str]:
    tree = ast.parse(_source(relative_path))
    imports: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.add(node.module or "")
    return imports


def test_t001_l2a_and_l7a_public_compute_ports_have_no_shared_controller():
    assert list(inspect.signature(MotionGuidance.compute).parameters) == [
        "self",
        "guidance",
    ]
    assert list(inspect.signature(MotionSemanticsEngine.compute).parameters) == [
        "self",
        "guidance",
    ]
    for relative in (
        "controller/encoder_reliability.py",
        "controller/heading_turn_controller.py",
        "controller/motion_semantics_engine.py",
        "controller/motion_guidance.py",
    ):
        source = _source(relative)
        assert "AlbaController" not in source
        assert "def compute(self, ctrl" not in source


def test_t002_l2a_l7a_dependency_contract_has_no_orchestrator_or_clock_import():
    for relative in (
        "controller/encoder_reliability.py",
        "controller/motion_semantics_engine.py",
        "controller/motion_guidance.py",
    ):
        imports = _imports(relative)
        assert "cont" not in imports
        assert "controller.motion_readiness" not in imports
        assert "time" not in imports
        assert "pathlib" not in imports


def test_t011_only_motion_guidance_creates_normal_physical_commands():
    producers = []
    for path in (PROJECT_ROOT / "controller").glob("*.py"):
        if "PhysicalMotionCommand(" in path.read_text(encoding="utf-8"):
            producers.append(path.relative_to(PROJECT_ROOT).as_posix())
    assert producers == ["controller/motion_guidance.py"]


def test_t011_cont_is_orchestrator_and_does_not_apply_post_resolver_policy():
    source = _source("cont.py")
    assert "guidance_component.compute(" in source
    for forbidden in (
        "global_motion_policy.apply(",
        "global_motion_policy.build_context(",
        "obstacle_avoidance.tick(",
        "motion_semantics.compute(",
        "MotionGuidance.physical_command(",
        "PhysicalMotionCommand(",
    ):
        assert forbidden not in source


def test_t011_heading_turn_closed_loop_has_one_l7a_owner():
    guidance_source = _source("controller/motion_guidance.py")
    assert "self._heading_controller.tick(" in guidance_source
    assert "self._heading_controller.start(" in guidance_source
    for relative in ("state.py", "cont.py", "control_loop.py"):
        source = _source(relative)
        assert "heading_controller.tick(" not in source
        assert "heading_controller.start(" not in source
        assert "heading_controller.cancel(" not in source


def test_t012_replayer_seals_resolver_guidance_controller_order():
    assert REPLAYABLE_LAYER_ORDER_V21[:3] == (
        LAYER_L6_INTENT_RESOLVER,
        LAYER_L7A_MOTION_GUIDANCE,
        LAYER_L8_MOTION_CONTROLLER,
    )
    adapter_source = _source("replayer/adapters.py")
    assert "resolver_guidance_intent_lineage_mismatch" in adapter_source
    assert "guidance_l8_physical_lineage_mismatch" in adapter_source
