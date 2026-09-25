from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TESTS = ROOT / "tests"

FOCUSED = {
    "follow": ("feature", "follow"),
    "roomcruise": ("feature", "room_cruise or roomcruise or cruise or explore"),
    "room_cruise": ("feature", "room_cruise or roomcruise or cruise or explore"),
    "localization": ("feature", "localization or pose or estimator"),
    "perception": ("feature", "perception or person or camera or lidar"),
    "motion": ("feature", "motion or forward or turn or trajectory"),
    "async": ("deep", "async or worker or delayed or deadline"),
    "process": ("deep", "process or crash or isolation or worker"),
    "replay": ("deep", "replay or checkpoint or restore or determin"),
}


def _run(paths, k=None):
    cmd = [sys.executable, "-m", "pytest", "-q", *[str(p) for p in paths]]
    if k:
        cmd += ["-k", k]
    return subprocess.call(cmd, cwd=ROOT)


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    mode = args.pop(0) if args else "core"
    if args:
        print("Unexpected extra arguments. Use: ./r test [full|follow|roomcruise|localization|perception|motion|async|process|replay]", file=sys.stderr)
        return 2
    if mode in {"core", "quick"}:
        return _run([TESTS / "core"])
    if mode == "full":
        return _run([TESTS / "core", TESTS / "feature", TESTS / "deep"])
    if mode in FOCUSED:
        layer, expr = FOCUSED[mode]
        return _run([TESTS / layer], expr)
    if mode in {"list", "help", "-h", "--help"}:
        print("./r test            CORE (~20-30)")
        print("./r test follow     Follow scenarios")
        print("./r test full       CORE + FEATURE + DEEP (~80-140, hard cap 150)")
        print("Other focused modes: roomcruise localization perception motion async process replay")
        return 0
    print(f"Unknown test mode: {mode}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
