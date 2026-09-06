import ast
from pathlib import Path

from v3.import_guard import validate_v3_imports


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _write(root: Path, relative: str, source: str) -> None:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(source, encoding="utf-8")


def _codes(root: Path, *, allowlist: frozenset[str] = frozenset()) -> set[str]:
    return {item.code for item in validate_v3_imports(root, donor_allowlist=allowlist)}


def test_current_v3_source_tree_has_no_import_boundary_violation():
    assert validate_v3_imports(PROJECT_ROOT) == ()


def test_live_idle_entrypoint_cannot_reintroduce_legacy_runtime_authority():
    path = PROJECT_ROOT / "v3_idle_runtime.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=path.name)
    imported_roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_roots.update(alias.name.split(".", 1)[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_roots.add(node.module.split(".", 1)[0])

    assert not imported_roots & {
        "config_manager",
        "cont",
        "controller",
        "driver",
        "fastgui",
        "motion_executor",
        "robot_state",
        "state",
    }


def test_native_motor_pwm_planner_remains_io_and_writer_capability_free():
    path = PROJECT_ROOT / "v3" / "adapters" / "motor_pwm.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imported_modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_modules.add("." * node.level + node.module)

    assert imported_modules <= {
        "__future__",
        "dataclasses",
        "enum",
        "math",
        "v3.contracts",
    }


def test_native_motor_writer_boundary_has_no_hardware_or_runtime_authority():
    path = PROJECT_ROOT / "v3" / "adapters" / "motor_writer.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imported_modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_modules.add("." * node.level + node.module)

    assert imported_modules <= {
        "__future__",
        ".motor_pwm",
        "typing",
        "v3.contracts",
    }


def test_native_live_input_aggregator_has_no_legacy_or_runtime_authority():
    path = PROJECT_ROOT / "v3" / "adapters" / "live_inputs.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imported_modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_modules.add(node.module)

    assert imported_modules <= {
        "__future__",
        "collections.abc",
        "dataclasses",
        "typing",
        "v3.contracts",
    }


def test_native_live_encoder_source_has_no_legacy_or_runtime_authority():
    path = PROJECT_ROOT / "v3" / "adapters" / "live_encoder.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imported_modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_modules.add("." * node.level + node.module)

    assert imported_modules <= {
        "__future__",
        "dataclasses",
        ".live_inputs",
        "math",
        "typing",
        "v3.contracts",
    }


def test_native_live_imu_source_has_no_legacy_or_runtime_authority():
    path = PROJECT_ROOT / "v3" / "adapters" / "live_imu.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imported_modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_modules.add("." * node.level + node.module)

    assert imported_modules <= {
        "__future__",
        "dataclasses",
        ".live_inputs",
        "math",
        "typing",
        "v3.contracts",
    }


def test_native_live_lidar_source_has_no_legacy_or_runtime_authority():
    path = PROJECT_ROOT / "v3" / "adapters" / "live_lidar.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imported_modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_modules.add("." * node.level + node.module)

    assert imported_modules <= {
        "__future__",
        "dataclasses",
        ".live_inputs",
        "math",
        "typing",
        "v3.contracts",
    }


def test_live_input_composition_has_no_hardware_clock_or_runtime_authority():
    path = PROJECT_ROOT / "v3" / "composition" / "live_inputs.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imported_modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_modules.add("." * node.level + node.module)

    assert imported_modules <= {
        "__future__",
        "dataclasses",
        ".input_shadow",
        "v3.adapters.live_encoder",
        "v3.adapters.live_imu",
        "v3.adapters.live_inputs",
        "v3.adapters.live_lidar",
        "v3.contracts",
        "v3.engine",
        "v3.layers.l2_admission",
        "v3.layers.l3_state_estimation",
        "v3.layers.l4_world_model",
    }
    imported_names = {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom)
        for alias in node.names
    }
    assert "NativeStateEstimator" in imported_names
    assert "ShadowStateEstimator" not in imported_names


def test_native_l3_ekf_has_no_legacy_numpy_or_runtime_authority():
    path = PROJECT_ROOT / "v3" / "layers" / "l3_state_estimation.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imported_modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_modules.add(node.module)

    assert imported_modules <= {
        "__future__",
        "dataclasses",
        "math",
        "v3.contracts",
    }


def test_legacy_motor_driver_is_library_only_without_standalone_motion_entrypoint():
    path = PROJECT_ROOT / "driver" / "motor.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

    main_guards = []
    for node in tree.body:
        if not isinstance(node, ast.If) or not isinstance(node.test, ast.Compare):
            continue
        comparison = node.test
        if (
            isinstance(comparison.left, ast.Name)
            and comparison.left.id == "__name__"
            and len(comparison.ops) == 1
            and isinstance(comparison.ops[0], ast.Eq)
            and len(comparison.comparators) == 1
            and isinstance(comparison.comparators[0], ast.Constant)
            and comparison.comparators[0].value == "__main__"
        ):
            main_guards.append(node)

    assert main_guards == []


def test_legacy_motor_driver_has_no_directional_convenience_write_paths():
    path = PROJECT_ROOT / "driver" / "motor.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    motor_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "AlbaMotor"
    )
    method_names = {
        node.name for node in motor_class.body if isinstance(node, ast.FunctionDef)
    }

    assert {"set_pwm", "stop", "close"} <= method_names
    assert {"forward", "backward"}.isdisjoint(method_names)


def test_legacy_motor_driver_has_no_config_or_file_authority():
    path = PROJECT_ROOT / "driver" / "motor.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imported_roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_roots.update(alias.name.split(".", 1)[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_roots.add(node.module.split(".", 1)[0])

    motor_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "AlbaMotor"
    )
    method_names = {
        node.name for node in motor_class.body if isinstance(node, ast.FunctionDef)
    }

    assert "config_manager" not in imported_roots
    assert "json" not in imported_roots
    assert "_load_config" not in method_names
    assert not any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "open"
        for node in ast.walk(tree)
    )


def test_legacy_motion_capable_set_pwm_is_only_the_final_control_commit():
    call_sites = []
    excluded_roots = {".git", ".venv", "build", "dist", "logs", "runtime", "tests"}
    for path in PROJECT_ROOT.rglob("*.py"):
        relative = path.relative_to(PROJECT_ROOT)
        if relative.parts and relative.parts[0] in excluded_roots:
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "set_pwm"
            ):
                call_sites.append(
                    (
                        relative.as_posix(),
                        ast.unparse(node.func.value),
                        tuple(ast.unparse(argument) for argument in node.args),
                    )
                )

    assert sorted(call_sites) == [
        ("cont.py", "self.motor_l", ("pwm_l",)),
        ("cont.py", "self.motor_r", ("pwm_r",)),
    ]


def test_legacy_authority_gui_tool_and_test_imports_are_always_forbidden(tmp_path):
    _write(
        tmp_path,
        "v3/contracts/bad.py",
        "import cont\nimport fastgui\nimport tools\nimport tests\nimport config_manager\n",
    )

    violations = validate_v3_imports(tmp_path)

    assert {item.imported_module for item in violations} == {
        "config_manager",
        "cont",
        "fastgui",
        "tests",
        "tools",
    }
    assert {item.code for item in violations} == {"FORBIDDEN_IMPORT"}


def test_legacy_project_import_is_only_allowed_in_allowlisted_donor_adapter(tmp_path):
    _write(tmp_path, "v3/layers/l11_actuator_control/bad.py", "import driver.encoder\n")
    assert "LEGACY_IMPORT_OUTSIDE_DONOR_ADAPTER" in _codes(tmp_path)

    (tmp_path / "v3/layers/l11_actuator_control/bad.py").unlink()
    _write(
        tmp_path,
        "v3/adapters/legacy_donors/encoder.py",
        "import driver.encoder\n",
    )
    assert "DONOR_IMPORT_NOT_ALLOWLISTED" in _codes(tmp_path)
    assert validate_v3_imports(
        tmp_path,
        donor_allowlist=frozenset({"driver.encoder"}),
    ) == ()

    _write(
        tmp_path,
        "v3/adapters/legacy_donors/encoder.py",
        "from controller import commands\n",
    )
    violations = validate_v3_imports(
        tmp_path,
        donor_allowlist=frozenset({"controller.commands"}),
    )
    assert {item.code for item in violations} == {"FORBIDDEN_IMPORT"}


def test_from_import_cannot_hide_legacy_dependency(tmp_path):
    _write(
        tmp_path,
        "v3/layers/l5_command_mission/bad.py",
        "from controller import commands\n",
    )

    violations = validate_v3_imports(tmp_path)

    assert "FORBIDDEN_IMPORT" in {item.code for item in violations}
    assert "controller.commands" in {item.imported_module for item in violations}


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

    assert [item.code for item in violations].count("CONTRACTS_DEPEND_ON_IMPLEMENTATION") == 3


def test_dynamic_import_is_rejected_as_static_guard_bypass(tmp_path):
    _write(
        tmp_path,
        "v3/contracts/bad.py",
        "import importlib\n"
        "from importlib import import_module as load\n"
        "first = __import__('cont')\n"
        "second = importlib.import_module('driver.motor')\n"
        "third = load('state')\n",
    )

    violations = validate_v3_imports(tmp_path)

    assert [item.code for item in violations].count("DYNAMIC_IMPORT") == 4


def test_unknown_project_or_third_party_root_requires_explicit_allowlist(tmp_path):
    _write(tmp_path, "v3/math/solver.py", "import numpy\nimport brain\n")

    violations = validate_v3_imports(tmp_path)

    assert {item.imported_module for item in violations} == {"brain", "numpy"}
    assert {item.code for item in violations} == {"THIRD_PARTY_IMPORT_NOT_ALLOWLISTED"}

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
