"""R2B4 pytest layer registry for offline Test Hub integration.

The canonical developer/agent runner is :mod:`v3.test_runner`. This module owns
no robot runtime state and only exposes the current CORE/FEATURE/DEEP layout to
offline consumers that still need explicit pytest targets.
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
        "core",
        None,
        "Safety, canonical authority, runtime composition and basic control-chain tests.",
        ("tests/core/test_*.py",),
    ),
    PytestProfile(
        "feature",
        None,
        "User-visible robot behavior: Follow, Room Cruise, localization, perception and motion.",
        ("tests/feature/test_*.py",),
    ),
    PytestProfile(
        "deep",
        None,
        "Process/worker failure, delayed async completion, replay and fault-injection tests.",
        ("tests/deep/test_*.py",),
    ),
    PytestProfile(
        "full",
        None,
        "Entire curated CORE + FEATURE + DEEP pytest suite.",
        ("tests/core/test_*.py", "tests/feature/test_*.py", "tests/deep/test_*.py"),
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
