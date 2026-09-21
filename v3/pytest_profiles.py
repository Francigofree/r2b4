"""R2B4 pytest profile registry shared by pytest and the offline Test Hub.

This module is development/diagnostic infrastructure only.  It owns no robot
runtime state and is intentionally limited to deterministic test selection.
"""

from __future__ import annotations

from dataclasses import dataclass
from fnmatch import fnmatchcase
from pathlib import Path


@dataclass(frozen=True, slots=True)
class PytestProfile:
    name: str
    marker: str | None
    description: str
    patterns: tuple[str, ...]
    explicit_targets: bool = True


_PROFILES: tuple[PytestProfile, ...] = (
    PytestProfile(
        "gate",
        "gate",
        "Fast high-signal V3 validation before wider regression.",
        ("tests/test_v3_gate.py",),
    ),
    PytestProfile(
        "contract",
        "contract",
        "V3 authority, layer-boundary, isolation and safety contracts.",
        (
            "tests/test_v3_*contract*.py",
            "tests/test_v3_architecture_boundaries.py",
            "tests/test_v3_control_process_isolation.py",
            "tests/test_v3_sterile_edges.py",
            "tests/test_v3_async_completion_boundary_fix.py",
            "tests/test_v3_observation.py",
            "tests/test_v3_gate.py",
        ),
    ),
    PytestProfile(
        "async",
        "async_boundary",
        "Async edge, completion/input-closure, process transport and multirate tests.",
        (
            "tests/test_v3_async_*.py",
            "tests/test_v3_control_process_isolation.py",
            "tests/test_v3_l6_planner_process.py",
            "tests/test_v3_multirate_*.py",
            "tests/test_v3_process_imu_device.py",
            "tests/test_v3_process_runtime.py",
            "tests/test_v3_runtime_phase_timing.py",
            "tests/test_v3_sterile_edges.py",
        ),
    ),
    PytestProfile(
        "runtime",
        "runtime",
        "TickEngine, composition, bounded/resident/process runtime and scheduling tests.",
        (
            "tests/test_v3_*runtime*.py",
            "tests/test_v3_bounded_*.py",
            "tests/test_v3_tick_engine.py",
            "tests/test_v3_execution.py",
            "tests/test_v3_native_control_composition.py",
            "tests/test_v3_control_process_isolation.py",
            "tests/test_v3_multirate_*.py",
        ),
    ),
    PytestProfile(
        "replay",
        "replay",
        "Capture, replay, MCAP and deterministic evidence-path tests.",
        (
            "tests/test_v3_*replay*.py",
            "tests/test_v3_*capture*.py",
            "tests/test_v3_mcap_*.py",
        ),
    ),
    PytestProfile(
        "control",
        "control",
        "Mission/planner/motion/safety/motor tests for the canonical control chain.",
        (
            "tests/test_v3_l5_l9_*.py",
            "tests/test_v3_l6_*.py",
            "tests/test_v3_l7_*.py",
            "tests/test_v3_l8_*.py",
            "tests/test_v3_l9_*.py",
            "tests/test_v3_l10_*.py",
            "tests/test_v3_l11_*.py",
            "tests/test_v3_motion_*.py",
            "tests/test_v3_motor_*.py",
            "tests/test_v3_safety_*.py",
            "tests/test_v3_command_gateway*.py",
            "tests/test_v3_native_control_composition.py",
        ),
    ),
    PytestProfile(
        "perception",
        "perception",
        "Sensor-edge, localization, L4/world-model and perception tests.",
        (
            "tests/test_v3_l4_*.py",
            "tests/test_v3_*lidar*.py",
            "tests/test_v3_*imu*.py",
            "tests/test_v3_*encoder*.py",
            "tests/test_v3_*person*.py",
            "tests/test_r2b4_lidar_*.py",
        ),
    ),
    PytestProfile(
        "testhub",
        "testhub",
        "Offline Test Hub, portable evidence, quality and correlation tests.",
        (
            "tests/test_v3_test_hub_*.py",
            "tests/test_v3_runtime_correlation.py",
            "tests/test_v3_mcap_e2e.py",
            "tests/test_v3_pytest_profiles.py",
        ),
    ),
    PytestProfile(
        "voice",
        "voice",
        "Human/robot conversation, prompting, voice and robot-context tests.",
        (
            "tests/test_voice_*.py",
            "tests/test_r2b4_voice_*.py",
            "tests/test_r2b4_conversation_*.py",
            "tests/test_r2b4_prompting*.py",
            "tests/test_r2b4_*llm*.py",
            "tests/test_r2b4_ready_speaker.py",
            "tests/test_r2b4_robot_context*.py",
        ),
    ),
    PytestProfile(
        "providers",
        "providers",
        "External AI/provider adapter tests, kept separate from robot-core gates.",
        (
            "tests/test_r2b4_gemini_*.py",
            "tests/test_r2b4_groq_*.py",
            "tests/test_r2b4_llm_provider.py",
        ),
    ),
    PytestProfile(
        "full",
        None,
        "Entire pytest suite selected by pytest testpaths.",
        ("tests/test_*.py",),
        explicit_targets=False,
    ),
)

_PROFILE_BY_NAME = {profile.name: profile for profile in _PROFILES}


def pytest_profile_names() -> tuple[str, ...]:
    return tuple(profile.name for profile in _PROFILES)


def get_pytest_profile(name: str) -> PytestProfile:
    try:
        return _PROFILE_BY_NAME[name]
    except KeyError as exc:
        allowed = ", ".join(pytest_profile_names())
        raise ValueError(f"unknown pytest profile {name!r}; expected one of: {allowed}") from exc


def resolve_pytest_files(project_root: str | Path, name: str) -> tuple[str, ...]:
    root = Path(project_root).resolve()
    profile = get_pytest_profile(name)
    matched: set[str] = set()
    for pattern in profile.patterns:
        for path in root.glob(pattern):
            if path.is_file() and path.suffix == ".py":
                matched.add(path.relative_to(root).as_posix())
    if not matched:
        raise FileNotFoundError(f"pytest profile {name!r} matched no test files under {root}")
    return tuple(sorted(matched))


def resolve_pytest_targets(project_root: str | Path, name: str) -> tuple[str, ...]:
    profile = get_pytest_profile(name)
    files = resolve_pytest_files(project_root, name)
    return files if profile.explicit_targets else ()


def markers_for_test_path(relative_path: str | Path) -> tuple[str, ...]:
    normalized = Path(relative_path).as_posix()
    markers: list[str] = []
    for profile in _PROFILES:
        if profile.marker is None:
            continue
        if any(fnmatchcase(normalized, pattern) for pattern in profile.patterns):
            markers.append(profile.marker)
    return tuple(markers)


__all__ = [
    "PytestProfile",
    "get_pytest_profile",
    "markers_for_test_path",
    "pytest_profile_names",
    "resolve_pytest_files",
    "resolve_pytest_targets",
]
