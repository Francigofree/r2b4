"""R2B4 Test Hub Next: simple P0-P1 agent-facing entry point.

Default, no-argument use:
    python3 -m v3.test_hub_next

It finds the newest runtime/captures/*.mcap, runs the existing Test Hub V2
diagnosis, and adds a 5 Hz event-preserving agent view.  The authoritative
capture/replay/diagnosis path remains unchanged.
"""

from __future__ import annotations

import argparse
import json
from collections.abc import Mapping, Sequence
from pathlib import Path

from .test_hub_analysis import analyze_capture
from .test_hub_v2 import diagnose_run
from .test_hub_views import SUPPORTED_HZ, build_run_view, compare_views
from .mcap_reader import McapReader

DEFAULT_CAPTURE_DIR = Path("runtime/captures")
DEFAULT_HZ = 5


def latest_capture(capture_dir: Path = DEFAULT_CAPTURE_DIR) -> Path:
    candidates = [path for path in capture_dir.glob("*.mcap") if path.is_file() and not path.is_symlink()]
    if not candidates:
        raise FileNotFoundError(f"no MCAP capture found under {capture_dir}")
    return max(candidates, key=lambda path: path.stat().st_mtime_ns)


def resolve_capture(value: str | None) -> Path:
    path = Path(value) if value else latest_capture()
    if path.suffix.lower() != ".mcap":
        raise ValueError("capture must be an .mcap file")
    if not path.is_file() or path.is_symlink():
        raise FileNotFoundError(path)
    return path


def default_output_dir(capture: Path) -> Path:
    return capture.with_name(capture.name + ".evidence_next")


def _load_triage_from_dir(output_dir: Path) -> Mapping[str, object] | None:
    path = output_dir / "triage.json"
    if not path.is_file():
        return None
    value = json.loads(path.read_text(encoding="utf-8"))
    return value if isinstance(value, Mapping) else None


def _diagnose_once(capture: Path, output_dir: Path, replay_mode: str) -> dict[str, object]:
    # The canonical backend refuses nonempty destinations. Never reuse a verdict
    # from a different capture, source revision or requested replay scope.
    return diagnose_run(capture, output_dir, replay_mode=replay_mode,
                        project_root=Path(__file__).resolve().parents[1])


def run_default(
    capture: Path,
    *,
    output_dir: Path | None = None,
    hz: int = DEFAULT_HZ,
    replay_mode: str = "incident",
) -> dict[str, object]:
    if hz not in SUPPORTED_HZ:
        raise ValueError(f"hz must be one of {SUPPORTED_HZ}")
    destination = output_dir or default_output_dir(capture)
    base = _diagnose_once(capture, destination, replay_mode)
    triage = _load_triage_from_dir(destination)
    if triage is None:
        triage = analyze_capture(McapReader(capture))

    view_path = destination / f"overview_{hz}hz.ndjson"
    view = build_run_view(capture, hz=hz, output_path=view_path, triage=triage)
    agent_view_path = destination / "agent_view.json"
    drilldowns = []
    for incident in view["effective_incidents"][:12]:
        if not isinstance(incident, Mapping):
            continue
        tick = incident.get("tick_id")
        layer = incident.get("layer")
        if isinstance(tick, int):
            start = max(0, tick - 10)
            end = tick + 10
            layers = "L8,L9,L10,L11,L12"
            if isinstance(layer, str) and layer.startswith("L"):
                try:
                    number = int(layer[1:])
                    lower = max(1, number - 2)
                    upper = min(12, number + 2)
                    layers = ",".join(f"L{i}" for i in range(lower, upper + 1))
                except ValueError:
                    pass
            drilldowns.append({
                "incident_id": incident.get("id"),
                "reason": incident.get("reason"),
                "tick_range": [start, end],
                "command": (
                    f"python3 -m v3.test_hub_v2 query {capture} "
                    f"--ticks {start}:{end} --layers {layers}"
                ),
            })

    agent_view = {
        "schema": "R2B4_TEST_HUB_NEXT_V1",
        "status": base.get("status"),
        "evidence_status": base.get("evidence_status"),
        "replay_status": base.get("replay_status"),
        "authority": {
            "capture": str(capture.resolve()),
            "v2_evidence_index": str((destination / "evidence_index.json").resolve()),
            "new_files_are_derived_only": True,
        },
        "default_view_hz": hz,
        "overview": str(view_path.resolve()),
        "data_coverage": view["data_coverage"],
        "phases": view["phases"],
        "effective_incidents": view["effective_incidents"],
        "suppressed_agent_noise": view["suppressed_agent_noise"],
        "incident_drilldowns": drilldowns,
        "how_to_drill_down": {
            "exact_ticks": (
                "python3 -m v3.test_hub_v2 query <capture.mcap> "
                "--ticks START:END --layers L8,L9,L10,L11,L12"
            ),
            "raw_lidar": (
                "python3 -m v3.test_hub_v2 query <capture.mcap> "
                "--topic /r2b4/raw_lidar --full --max-bytes 2097152"
            ),
            "other_view_rates": (
                "python3 -m v3.test_hub_next view <capture.mcap> --hz 1|5|10"
            ),
        },
    }
    agent_view_path.write_text(
        json.dumps(agent_view, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return {
        "status": base.get("status"),
        "evidence_status": base.get("evidence_status"),
        "replay_status": base.get("replay_status"),
        "capture": str(capture.resolve()),
        "output_dir": str(destination.resolve()),
        "agent_view": str(agent_view_path.resolve()),
        "overview": str(view_path.resolve()),
        "note": "MCAP and existing V2 evidence remain authoritative; Next files are derived agent views.",
    }


def _load_or_build_view(path: Path, hz: int) -> Mapping[str, object]:
    if path.suffix.lower() == ".mcap":
        return build_run_view(path, hz=hz)
    if path.is_dir():
        agent = path / "agent_view.json"
        if agent.is_file():
            payload = json.loads(agent.read_text(encoding="utf-8"))
            overview = payload.get("overview") if isinstance(payload, Mapping) else None
            if isinstance(overview, str) and Path(overview).is_file():
                # Rebuild from authority path if possible; avoids parsing NDJSON back into a second schema.
                authority = payload.get("authority")
                capture = authority.get("capture") if isinstance(authority, Mapping) else None
                if isinstance(capture, str) and Path(capture).is_file():
                    return build_run_view(Path(capture), hz=hz)
        raise ValueError(f"evidence directory lacks a usable agent_view.json: {path}")
    raise ValueError("compare inputs must be MCAP files or Next evidence directories")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="R2B4 Test Hub Next. No arguments = analyze newest MCAP at 5 Hz."
    )
    commands = parser.add_subparsers(dest="command")

    run = commands.add_parser("run", help="full default Test Hub run")
    run.add_argument("capture", nargs="?", help="default: newest runtime/captures/*.mcap")
    run.add_argument("--hz", type=int, choices=SUPPORTED_HZ, default=DEFAULT_HZ)
    run.add_argument("--output-dir")
    run.add_argument("--replay", choices=("off", "incident", "full"), default="incident")

    view = commands.add_parser("view", help="cheap readable view only")
    view.add_argument("capture", nargs="?", help="default: newest runtime/captures/*.mcap")
    view.add_argument("--hz", type=int, choices=SUPPORTED_HZ, default=DEFAULT_HZ)
    view.add_argument("--output")

    compare = commands.add_parser("compare", help="objective before/after comparison")
    compare.add_argument("before")
    compare.add_argument("after")
    compare.add_argument("--hz", type=int, choices=SUPPORTED_HZ, default=DEFAULT_HZ)
    compare.add_argument("--output")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    command = args.command or "run"
    try:
        if command == "run":
            capture_value = getattr(args, "capture", None)
            capture = resolve_capture(capture_value)
            output_dir = Path(args.output_dir) if getattr(args, "output_dir", None) else None
            result = run_default(
                capture,
                output_dir=output_dir,
                hz=getattr(args, "hz", DEFAULT_HZ),
                replay_mode=getattr(args, "replay", "incident"),
            )
        elif command == "view":
            capture = resolve_capture(args.capture)
            output = (
                Path(args.output)
                if args.output
                else capture.with_name(capture.name + f".overview_{args.hz}hz.ndjson")
            )
            result = build_run_view(capture, hz=args.hz, output_path=output)
            result = {
                "status": "PASS",
                "capture": str(capture.resolve()),
                "hz": args.hz,
                "overview": str(output.resolve()),
                "phase_count": len(result["phases"]),
                "event_count": len(result["events"]),
            }
        else:
            before_path = Path(args.before)
            after_path = Path(args.after)
            result = compare_views(
                _load_or_build_view(before_path, args.hz),
                _load_or_build_view(after_path, args.hz),
            )
            if args.output:
                out = Path(args.output)
                out.parent.mkdir(parents=True, exist_ok=True)
                with out.open("x", encoding="utf-8") as handle:
                    handle.write(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n")
                result = {**result, "output": str(out.resolve())}
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False))
        return 2 if result.get("status") in {"FAIL", "ERROR"} else 0
    except (OSError, TypeError, ValueError, RuntimeError) as exc:
        print(json.dumps({"status": "ERROR", "error": str(exc)}, ensure_ascii=False, indent=2))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "DEFAULT_HZ",
    "default_output_dir",
    "latest_capture",
    "main",
    "resolve_capture",
    "run_default",
]
