"""Offline pytest profile registry.

Permanent regression selection is owned by the gate/scenario manifests.
Developer packs remain discoverable but are not promoted into the mandatory
release suite merely because a test file exists.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from v3.test_runner import MANIFESTS, PACKS, _load_manifest, _release_targets


@dataclass(frozen=True, slots=True)
class PytestProfile:
    name: str
    description: str
    explicit_targets: bool = True


_PROFILES = (
    PytestProfile("gate", "Small mandatory robot invariant gate."),
    PytestProfile("release", "Gate plus user-visible robot scenarios."),
    PytestProfile("all", "All optional developer packs.", explicit_targets=False),
    PytestProfile("core", "Compatibility developer pack: former core tests.", explicit_targets=False),
    PytestProfile("feature", "Compatibility developer pack: former feature tests.", explicit_targets=False),
    PytestProfile("deep", "Compatibility developer pack: former deep tests.", explicit_targets=False),
)
_PROFILE_BY_NAME = {profile.name: profile for profile in _PROFILES}
_ALIASES = {"quick": "gate", "full": "release"}


def pytest_profile_names() -> tuple[str, ...]:
    return tuple(profile.name for profile in _PROFILES)


def get_pytest_profile(name: str) -> PytestProfile:
    name = _ALIASES.get(name, name)
    try:
        return _PROFILE_BY_NAME[name]
    except KeyError as exc:
        allowed = ", ".join(pytest_profile_names())
        raise ValueError(f"unknown pytest profile {name!r}; expected one of: {allowed}") from exc


def resolve_pytest_files(project_root: str | Path, name: str) -> tuple[str, ...]:
    root = Path(project_root).resolve()
    name = _ALIASES.get(name, name)
    get_pytest_profile(name)
    if name in {"gate", "release"}:
        targets = _load_manifest("gate") if name == "gate" else _release_targets()
        return tuple(sorted({target.split("::", 1)[0] for target in targets}))
    path = root / "tests" / "packs"
    if name in {"core", "feature", "deep"}:
        path = path / name
    files = tuple(
        sorted(p.relative_to(root).as_posix() for p in path.rglob("test_*.py") if p.is_file())
    )
    if not files:
        raise FileNotFoundError(f"pytest profile {name!r} matched no test files under {root}")
    return files


def resolve_pytest_targets(project_root: str | Path, name: str) -> tuple[str, ...]:
    name = _ALIASES.get(name, name)
    get_pytest_profile(name)
    if name == "gate":
        return _load_manifest("gate")
    if name == "release":
        return _release_targets()
    return ()


def markers_for_test_path(relative_path: str | Path) -> tuple[str, ...]:
    # Markers are legacy developer metadata only; they no longer select gates.
    return ()


__all__ = [
    "PytestProfile",
    "get_pytest_profile",
    "markers_for_test_path",
    "pytest_profile_names",
    "resolve_pytest_files",
    "resolve_pytest_targets",
]
