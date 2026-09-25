"""V3 architecture tests: guard capabilities, not implementation trivia."""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from v3.import_guard import validate_v3_imports


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _write(root: Path, relative: str, source: str) -> None:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(source, encoding="utf-8")


def _codes(root: Path) -> set[str]:
    return {item.code for item in validate_v3_imports(root)}


def _imports(relative: str) -> set[str]:
    path = PROJECT_ROOT / relative
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    modules: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            modules.add(node.module)
    return modules


def _imports_any(modules: set[str], forbidden: set[str]) -> bool:
    return any(
        module == prefix or module.startswith(prefix + ".")
        for module in modules
        for prefix in forbidden
    )


def test_current_v3_source_tree_has_no_import_boundary_violation():
    assert validate_v3_imports(PROJECT_ROOT) == ()


@pytest.mark.parametrize(
    ("adapter_path", "edge_import", "illegal_path"),
    (
        (
            "v3/adapters/picamera2_camera.py",
            "from picamera2 import Picamera2\nfrom libcamera import controls\n",
            "v3/layers/l4_world_model/illegal_camera_dependency.py",
        ),
        (
            "v3/adapters/litert_person_detector.py",
            "from ai_edge_litert.interpreter import Interpreter\n",
            "v3/layers/l4_world_model/illegal_litert_dependency.py",
        ),
    ),
)
def test_hardware_third_party_dependencies_are_edge_scoped(
    tmp_path,
    adapter_path,
    edge_import,
    illegal_path,
):
    _write(tmp_path, adapter_path, edge_import)
    assert validate_v3_imports(tmp_path) == ()

    _write(tmp_path, illegal_path, edge_import)
    violations = validate_v3_imports(tmp_path)
    assert any(
        item.path == illegal_path
        and item.code == "PROJECT_OR_THIRD_PARTY_IMPORT_NOT_ALLOWED"
        for item in violations
    )


@pytest.mark.parametrize(
    ("relative", "forbidden"),
    (
        ("v3_bounded_runtime.py", {"lgpio", "signal"}),
        ("v3_hardware_runtime.py", {"lgpio", "smbus2"}),
        ("v3_bounded_config.py", {"lgpio", "signal", "time"}),
        (
            "v3/adapters/l6_planner_process.py",
            {
                "v3.adapters.gpio_motor",
                "v3.ports",
                "v3.layers.l12_safety_final",
                "v3.adapters.resident_command",
            },
        ),
    ),
)
def test_non_owner_components_do_not_gain_forbidden_authority(relative, forbidden):
    assert not _imports_any(_imports(relative), forbidden), relative


def test_non_v3_project_imports_are_rejected(tmp_path):
    _write(
        tmp_path,
        "v3/contracts/bad.py",
        "import obsolete_runtime\nfrom operator_gui import api\n",
    )
    violations = validate_v3_imports(tmp_path)
    assert {item.imported_module for item in violations} == {
        "obsolete_runtime",
        "operator_gui.api",
    }
    assert {item.code for item in violations} == {
        "PROJECT_OR_THIRD_PARTY_IMPORT_NOT_ALLOWED"
    }


def test_from_import_cannot_hide_a_non_v3_dependency(tmp_path):
    _write(
        tmp_path,
        "v3/layers/l5_command_mission/bad.py",
        "from obsolete_runtime import commands\n",
    )
    violations = validate_v3_imports(tmp_path)
    assert "PROJECT_OR_THIRD_PARTY_IMPORT_NOT_ALLOWED" in {
        item.code for item in violations
    }
    assert "obsolete_runtime.commands" in {
        item.imported_module for item in violations
    }


def test_layer_cannot_import_another_layer_or_an_adapter(tmp_path):
    _write(
        tmp_path,
        "v3/layers/l8_motion_realization/bad.py",
        "from v3.layers.l3_state_estimation import estimator\n"
        "from v3.adapters import lidar\n"
        "import v3.import_guard\n",
    )
    codes = _codes(tmp_path)
    assert "CROSS_LAYER_IMPLEMENTATION_IMPORT" in codes
    assert "LAYER_IMPORTS_ADAPTER" in codes
    assert "LAYER_INTERNAL_IMPORT_NOT_ALLOWED" in codes


def test_helpers_of_the_same_numbered_layer_are_not_cross_layer_imports(tmp_path):
    _write(tmp_path, "v3/layers/l4_world_model.py",
           "from v3.layers.l4_temporal_occupancy import ScanCellEvidence\n")
    assert validate_v3_imports(tmp_path) == ()


def test_process_device_imports_stay_confined_to_named_edges(tmp_path):
    _write(tmp_path, "v3/adapters/process_encoder_backend.py", "import lgpio\n")
    _write(tmp_path, "v3/adapters/process_lidar_port.py", "import serial\n")
    assert validate_v3_imports(tmp_path) == ()
    _write(tmp_path, "v3/layers/l3_state_estimation.py", "import lgpio\nimport serial\n")
    assert {item.imported_module for item in validate_v3_imports(tmp_path)} == {"lgpio", "serial"}


def test_only_l12_can_import_the_final_writer_port(tmp_path):
    _write(
        tmp_path,
        "v3/layers/l11_actuator_control/bad.py",
        "from v3.ports import MotorWriter\n",
    )
    assert "FINAL_WRITER_PORT_OUTSIDE_L12" in _codes(tmp_path)

    (tmp_path / "v3/layers/l11_actuator_control/bad.py").unlink()
    _write(
        tmp_path,
        "v3/layers/l12_safety_final.py",
        "from v3.ports import MotorWriter\n",
    )
    _write(tmp_path, "v3/ports.py", "class MotorWriter:\n    pass\n")
    assert validate_v3_imports(tmp_path) == ()


def test_contracts_cannot_depend_on_layer_adapter_or_composition(tmp_path):
    _write(
        tmp_path,
        "v3/contracts/bad.py",
        "import v3.layers.l3_state_estimation\n"
        "import v3.adapters.lidar\n"
        "import v3.composition.root\n",
    )
    violations = validate_v3_imports(tmp_path)
    assert [item.code for item in violations].count(
        "CONTRACTS_DEPEND_ON_IMPLEMENTATION"
    ) == 3


def test_shared_device_health_policy_exception_is_l12_only(tmp_path):
    _write(
        tmp_path,
        "v3/layers/l12_safety_final.py",
        "from v3.device_health_policy import critical_device_health_view\n",
    )
    assert validate_v3_imports(tmp_path) == ()

    _write(
        tmp_path,
        "v3/layers/l11_actuator_control.py",
        "from v3.device_health_policy import critical_device_health_view\n",
    )
    violations = validate_v3_imports(tmp_path)
    assert any(
        item.path == "v3/layers/l11_actuator_control.py"
        and item.code == "LAYER_INTERNAL_IMPORT_NOT_ALLOWED"
        for item in violations
    )


def test_dynamic_import_is_rejected_as_static_guard_bypass(tmp_path):
    _write(
        tmp_path,
        "v3/contracts/bad.py",
        "import importlib\n"
        "from importlib import import_module as load\n"
        "first = __import__('obsolete_runtime')\n"
        "second = importlib.import_module('external.module')\n"
        "third = load('hidden.module')\n",
    )
    violations = validate_v3_imports(tmp_path)
    assert [item.code for item in violations].count("DYNAMIC_IMPORT") == 4


def test_unknown_third_party_root_requires_explicit_allowlist(tmp_path):
    _write(tmp_path, "v3/math/solver.py", "import numpy\nimport brain\n")
    violations = validate_v3_imports(tmp_path)
    assert {item.imported_module for item in violations} == {"brain"}

    _write(tmp_path, "v3/math/solver.py", "import numpy\n")
    assert validate_v3_imports(
        tmp_path,
        approved_third_party=frozenset({"numpy"}),
    ) == ()


def test_syntax_error_fails_closed(tmp_path):
    _write(tmp_path, "v3/contracts/bad.py", "def broken(:\n")
    violations = validate_v3_imports(tmp_path)
    assert len(violations) == 1
    assert violations[0].code == "UNPARSEABLE_SOURCE"
