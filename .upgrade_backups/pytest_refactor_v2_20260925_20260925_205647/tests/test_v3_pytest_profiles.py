"""Contract tests for the R2B4 pytest/Test Hub profile registry."""

from __future__ import annotations

from pathlib import Path

from v3.pytest_profiles import (
    get_pytest_profile,
    markers_for_test_path,
    pytest_profile_names,
    resolve_pytest_files,
    resolve_pytest_targets,
)


PROJECT_ROOT = (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))


def test_all_profiles_resolve_to_existing_tests():
    names = pytest_profile_names()
    assert names[-1] == "full"
    assert len(names) == len(set(names))
    for name in names:
        files = resolve_pytest_files(PROJECT_ROOT, name)
        assert files
        assert all((PROJECT_ROOT / relative).is_file() for relative in files)
        assert all(relative.startswith("tests/test_") for relative in files)


def test_full_profile_delegates_to_pytest_testpaths_without_explicit_targets():
    assert resolve_pytest_targets(PROJECT_ROOT, "full") == ()
    assert get_pytest_profile("full").explicit_targets is False


def test_testhub_profile_is_pattern_based_and_contains_profile_contract_test():
    files = resolve_pytest_files(PROJECT_ROOT, "testhub")
    assert "tests/test_v3_test_hub_portable.py" in files
    assert "tests/test_v3_test_hub_quality.py" in files
    assert "tests/test_v3_pytest_profiles.py" in files
    assert "tests/test_v3_runtime_correlation.py" in files


def test_async_profile_covers_new_completion_and_process_boundaries():
    files = resolve_pytest_files(PROJECT_ROOT, "async")
    assert "tests/test_v3_async_completion_boundary_fix.py" in files
    assert "tests/test_v3_control_process_isolation.py" in files
    assert "tests/test_v3_l6_planner_process.py" in files
    assert "tests/test_v3_multirate_inputs.py" in files


def test_markers_are_derived_from_the_same_profile_registry():
    markers = set(markers_for_test_path("tests/test_v3_async_completion_boundary_fix.py"))
    assert {"contract", "async_boundary"} <= markers
    hub_markers = set(markers_for_test_path("tests/test_v3_test_hub_quality.py"))
    assert "testhub" in hub_markers
    assert "voice" not in hub_markers
