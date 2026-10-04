from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TESTS = ROOT / "tests"
PACKS = TESTS / "packs"
MANIFESTS = {
    "gate": TESTS / "gate" / "manifest.json",
    "scenarios": TESTS / "scenarios" / "manifest.json",
    "endurance": TESTS / "endurance" / "manifest.json",
}

# Compatibility surface used only by launcher help/completion. It is not a
# second test-selection authority; manifests and pack directories are.
FOCUSED = ("release", "pack", "all", "endurance")

ALIASES = {
    "quick": "gate",
    "full": "release",
}


def _load_manifest(name: str) -> tuple[str, ...]:
    try:
        path = MANIFESTS[name]
    except KeyError as exc:
        raise ValueError(f"unknown manifest: {name}") from exc
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema") != "R2B4_PYTEST_CONTRACT_MANIFEST_V1":
        raise ValueError(f"invalid pytest manifest schema: {path}")
    tests = payload.get("tests")
    budget = payload.get("budget")
    if not isinstance(tests, list) or not isinstance(budget, int) or budget <= 0:
        raise ValueError(f"invalid pytest manifest structure: {path}")
    targets = []
    for row in tests:
        target = row.get("id") if isinstance(row, dict) else None
        if not isinstance(target, str) or not target.startswith("tests/"):
            raise ValueError(f"invalid pytest target in {path}: {target!r}")
        source = target.split("::", 1)[0]
        if not (ROOT / source).is_file():
            raise FileNotFoundError(f"pytest manifest target is missing: {target}")
        targets.append(target)
    if len(targets) > budget:
        raise ValueError(
            f"{name} contract budget exceeded: {len(targets)} logical tests > {budget}"
        )
    if len(set(targets)) != len(targets):
        raise ValueError(f"duplicate pytest target in {path}")
    return tuple(targets)


def _release_targets() -> tuple[str, ...]:
    return tuple(dict.fromkeys((*_load_manifest("gate"), *_load_manifest("scenarios"))))


def _pack_names() -> tuple[str, ...]:
    if not PACKS.is_dir():
        return ()
    return tuple(sorted(path.name for path in PACKS.iterdir() if path.is_dir() and not path.name.startswith(".")))


def _run(
    targets: tuple[str, ...] | list[str],
    *,
    extra=(),
    fail_fast: bool = False,
    marker: str | None = None,
) -> int:
    cmd = [sys.executable, "-m", "pytest", "-q", "--tb=short", *targets]
    if marker is not None:
        cmd += ["-m", marker]
    if fail_fast:
        cmd += ["--maxfail=1"]
    else:
        cmd += ["--durations=5", "--durations-min=0.5"]
    cmd.extend(extra)
    env = os.environ.copy()
    env.setdefault("PYTEST_DISABLE_PLUGIN_AUTOLOAD", "1")
    return subprocess.call(cmd, cwd=ROOT, env=env)


def _print_help() -> None:
    packs = " ".join(_pack_names()) or "(none)"
    print("./r test                  mandatory robot gate (small, fail-fast)")
    print("./r test release          gate + user-visible robot scenarios")
    print("./r test full             alias of release")
    print("./r test pack NAME        optional developer pack")
    print("./r test core|feature|deep compatibility aliases for pack NAME")
    print("./r test all              every developer pack, endurance excluded")
    print("./r test endurance        explicit long synthetic validation")
    print("./r test tests/...        one exact pytest file/node")
    print("./r test MODE --collect-only / --lf forwards pytest options")
    print("Available packs: " + packs)


def main(argv=None) -> int:
    from v3.runtime_performance import apply_host_affinity

    apply_host_affinity(ROOT, "diagnostics")
    args = list(sys.argv[1:] if argv is None else argv)
    mode = args.pop(0) if args else "gate"
    mode = ALIASES.get(mode, mode)

    if mode == "gate":
        return _run(list(_load_manifest("gate")), extra=args, fail_fast=True)
    if mode == "release":
        return _run(list(_release_targets()), extra=args)
    if mode == "endurance":
        return _run(list(_load_manifest("endurance")), extra=args, marker="endurance")
    if mode == "all":
        return _run([str(PACKS)], extra=args)
    if mode == "pack":
        if not args or args[0].startswith("-"):
            _print_help()
            return 0 if not args else 2
        name = args.pop(0)
        path = PACKS / name
        if name not in _pack_names() or not path.is_dir():
            print(f"Unknown pytest pack: {name}", file=sys.stderr)
            return 2
        return _run([str(path)], extra=args)
    if mode in {"core", "feature", "deep"}:
        path = PACKS / mode
        if not path.is_dir():
            print(f"Missing compatibility pack: {mode}", file=sys.stderr)
            return 2
        return _run([str(path)], extra=args)
    if mode in {"list", "help", "-h", "--help"}:
        _print_help()
        return 0

    if mode.startswith("tests/"):
        source = mode.split("::", 1)[0]
        path = (ROOT / source).resolve()
        if path.is_file() and path.is_relative_to(TESTS):
            return _run([mode], extra=args)

    print(f"Unknown test mode: {mode}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
