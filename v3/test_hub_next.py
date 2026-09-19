"""R2B4 Test Hub: one agent-friendly offline pipeline over MCAP authority.

Default, no-argument use:
    python3 -m v3.test_hub

A finished MCAP is converted into one portable ``.evidence/`` directory with
canonical diagnosis, compact full-run views, exact incident slices, bounded raw
LiDAR incident evidence and a checkpoint-aware replay sweep.  The robot runtime
is not an analysis authority and does not import this module.
"""

from __future__ import annotations

import argparse
import json
from collections.abc import Mapping, Sequence
from pathlib import Path

from .test_hub_analysis import analyze_capture
from .test_hub_motion_quality import (
    compare_motion_quality_sources,
    write_motion_quality,
)
from .test_hub_localization_quality import (
    compare_localization_quality_sources,
    write_localization_quality,
)
from .test_hub_behavior import build_behavior_evidence
from .test_hub_v2 import diagnose_run
from .test_hub_views import SUPPORTED_HZ, build_run_view, compare_views
from .mcap_reader import McapReader
from .test_hub_portable import (
    DEFAULT_REPLAY_WINDOW_TICKS,
    group_incidents,
    replay_sweep,
    run_pytest,
    runtime_performance_summary,
    write_incident_slices,
    write_lidar_summary,
    write_portable_manifest,
    write_raw_lidar_incident_slices,
)

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
    return path.resolve()


def default_output_dir(capture: Path) -> Path:
    # One canonical derived directory.  No parallel .evidence + .evidence_next trees.
    return capture.with_suffix(".evidence")


def _load_json(path: Path) -> Mapping[str, object] | None:
    if not path.is_file():
        return None
    value = json.loads(path.read_text(encoding="utf-8"))
    return value if isinstance(value, Mapping) else None


def _write_json(path: Path, payload: Mapping[str, object]) -> Path:
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    return path


def _diagnose_once(capture: Path, output_dir: Path, replay_mode: str) -> dict[str, object]:
    # The canonical backend refuses nonempty destinations. Never reuse a verdict
    # from a different capture, source revision or requested replay scope.
    return diagnose_run(
        capture,
        output_dir,
        replay_mode=replay_mode,
        project_root=Path(__file__).resolve().parents[1],
    )


def run_default(
    capture: Path,
    *,
    output_dir: Path | None = None,
    hz: int = DEFAULT_HZ,
    replay_mode: str = "incident",
    replay_sweep_enabled: bool = True,
    sweep_window_ticks: int = DEFAULT_REPLAY_WINDOW_TICKS,
    pytest_scope: str = "off",
) -> dict[str, object]:
    """Create one self-contained agent evidence bundle for a finished MCAP."""
    if hz not in SUPPORTED_HZ:
        raise ValueError(f"hz must be one of {SUPPORTED_HZ}")
    if pytest_scope not in {"off", "testhub", "full"}:
        raise ValueError("pytest_scope must be off, testhub or full")

    capture = resolve_capture(str(capture))
    destination = output_dir or default_output_dir(capture)
    base = _diagnose_once(capture, destination, replay_mode)
    triage = _load_json(destination / "triage.json")
    if triage is None:
        triage = analyze_capture(McapReader(capture))

    reader = McapReader(capture)
    behavior = build_behavior_evidence(reader, destination, triage=triage)
    capture_sha256 = reader.sha256()

    view_path = destination / f"overview_{hz}hz.ndjson"
    view = build_run_view(capture, hz=hz, output_path=view_path, triage=triage)

    incident_groups = group_incidents(
        view.get("effective_incidents") if isinstance(view.get("effective_incidents"), Sequence) else ()
    )
    exact_slices = write_incident_slices(
        reader,
        destination / "incident_slices",
        incident_groups,
    )

    lidar_summary, tick_refs = write_lidar_summary(
        capture,
        destination / "lidar_summary.ndjson",
        reader=reader,
    )
    raw_lidar_slices = write_raw_lidar_incident_slices(
        reader,
        destination / "raw_lidar_incidents",
        incident_groups,
        tick_refs,
    )

    inspect_payload = _load_json(destination / "inspect.json") or {}
    performance = runtime_performance_summary(inspect_payload)
    performance_path = _write_json(destination / "runtime_performance.json", performance)

    sweep: Mapping[str, object] | None = None
    if replay_sweep_enabled and replay_mode != "off":
        final_event = inspect_payload.get("final_event")
        structure = inspect_payload.get("structure")
        integrity_verified = bool(
            isinstance(structure, Mapping)
            and structure.get("valid") is True
            and structure.get("data_crc_ok") is True
            and structure.get("summary_crc_ok") is True
            and structure.get("chunk_crc_ok") is True
            and inspect_payload.get("integrity_error") is None
        )
        sweep = replay_sweep(
            capture,
            project_root=Path(__file__).resolve().parents[1],
            window_ticks=sweep_window_ticks,
            authority_sha256=capture_sha256,
            verified_final_event=(
                final_event
                if integrity_verified and isinstance(final_event, Mapping)
                else None
            ),
            structure_already_verified=integrity_verified,
        )
        _write_json(destination / "replay_sweep.json", sweep)

    pytest_payload: Mapping[str, object] | None = None
    if pytest_scope != "off":
        pytest_payload = run_pytest(Path(__file__).resolve().parents[1], scope=pytest_scope)
        _write_json(destination / "pytest_result.json", pytest_payload)

    base_status = str(base.get("status") or "ERROR")
    overall_status = base_status
    if sweep is not None and sweep.get("status") in {"MISMATCH", "ERROR"}:
        overall_status = "FAIL"
    if pytest_payload is not None and pytest_payload.get("status") != "PASS":
        overall_status = "FAIL"

    replay_status = base.get("replay_status")
    replay_sweep_status = (
        sweep.get("status") if sweep is not None else "OFF"
    )

    behavior_episode_path = destination / "behavior_episodes.ndjson"
    behavior_episode_source = behavior_episode_path if behavior_episode_path.is_file() else None

    motion_quality_path = destination / "motion_quality.json"
    motion_segments_path = destination / "motion_quality_segments.ndjson"
    try:
        motion_quality = write_motion_quality(
            reader,
            motion_quality_path,
            motion_segments_path,
            behavior_episodes_path=behavior_episode_source,
        )
    except (OSError, TypeError, ValueError, RuntimeError) as exc:
        motion_quality = {
            "schema": "R2B4_TEST_HUB_MOTION_QUALITY_V1",
            "status": "ERROR",
            "error": str(exc),
            "findings": [],
        }
        _write_json(motion_quality_path, motion_quality)
        motion_segments_path.write_text("", encoding="utf-8")

    localization_quality_path = destination / "localization_quality.json"
    localization_events_path = destination / "localization_events.ndjson"
    try:
        localization_quality = write_localization_quality(
            reader,
            localization_quality_path,
            localization_events_path,
            behavior_episodes_path=behavior_episode_source,
        )
    except (OSError, TypeError, ValueError, RuntimeError) as exc:
        localization_quality = {
            "schema": "R2B4_TEST_HUB_LOCALIZATION_QUALITY_V1",
            "status": "ERROR",
            "error": str(exc),
            "findings": [],
        }
        _write_json(localization_quality_path, localization_quality)
        localization_events_path.write_text("", encoding="utf-8")

    agent_view_path = destination / "agent_view.json"
    agent_view = {
        "schema": "R2B4_TEST_HUB_AGENT_V1",
        "status": overall_status,
        "diagnosis_status": base.get("diagnosis_status"),
        "evidence_status": base.get("evidence_status"),
        "behavior_status": base.get("behavior_status"),
        "behavior": behavior,
        "replay_status": replay_status,
        "replay_sweep_status": replay_sweep_status,
        "authority": {
            "capture_name": capture.name,
            "capture_sha256": capture_sha256,
            "capture_local_path": str(capture),
            "mcap_remains_authority": True,
            "portable_remote_analysis": True,
        },
        "default_view_hz": hz,
        "overview": view_path.name,
        "timeline": "timeline.ndjson",
        "lidar_summary": Path(str(lidar_summary["path"])).name,
        "runtime_performance": performance_path.name,
        "replay_sweep": "replay_sweep.json" if sweep is not None else None,
        "pytest_result": "pytest_result.json" if pytest_payload is not None else None,
        "data_coverage": view.get("data_coverage"),
        "phases": view.get("phases"),
        "incident_groups": incident_groups,
        "incident_slices": exact_slices,
        "raw_lidar_incident_slices": raw_lidar_slices,
        "suppressed_agent_noise": view.get("suppressed_agent_noise"),
        "quality": {
            "motion": {
                "status": motion_quality.get("status"),
                "summary": motion_quality_path.name,
                "details": motion_segments_path.name,
                "finding_count": (
                    len(motion_quality.get("findings", ()))
                    if isinstance(motion_quality.get("findings"), Sequence)
                    else 0
                ),
            },
            "localization": {
                "status": localization_quality.get("status"),
                "summary": localization_quality_path.name,
                "events": localization_events_path.name,
                "finding_count": (
                    len(localization_quality.get("findings", ()))
                    if isinstance(localization_quality.get("findings"), Sequence)
                    else 0
                ),
            },
        },
        "remote_analysis_policy": {
            "normal_analysis_requires_mcap": False,
            "exact_nonexported_tick_or_raw_point_request_requires_local_mcap": True,
            "full_raw_lidar_is_exported_only_around_representative_incidents": True,
        },
    }
    _write_json(agent_view_path, agent_view)

    manifest_path = write_portable_manifest(
        destination,
        capture,
        capture_sha256=capture_sha256,
        replay_sweep_payload=sweep,
        pytest_payload=pytest_payload,
    )

    return {
        "status": overall_status,
        "evidence_status": base.get("evidence_status"),
        "behavior_status": base.get("behavior_status"),
        "replay_status": replay_status,
        "replay_sweep_status": replay_sweep_status,
        "capture": str(capture),
        "output_dir": str(destination.resolve()),
        "agent_view": str(agent_view_path.resolve()),
        "overview": str(view_path.resolve()),
        "portable_manifest": str(manifest_path.resolve()),
        "incident_group_count": len(incident_groups),
        "exact_incident_slice_count": len(exact_slices),
        "raw_lidar_incident_slice_count": len(raw_lidar_slices),
        "pytest_status": pytest_payload.get("status") if pytest_payload else "OFF",
        "motion_quality_status": motion_quality.get("status"),
        "localization_quality_status": localization_quality.get("status"),
        "note": "One .evidence directory is the portable agent package; MCAP remains local authority.",
    }


def run_pending(
    capture_dir: str | Path = DEFAULT_CAPTURE_DIR,
    *,
    hz: int = DEFAULT_HZ,
    replay_mode: str = "incident",
    sweep_window_ticks: int = DEFAULT_REPLAY_WINDOW_TICKS,
) -> dict[str, object]:
    """Convert every finished MCAP that does not yet have its .evidence bundle."""
    root = Path(capture_dir)
    captures = sorted(
        (path for path in root.glob("*.mcap") if path.is_file() and not path.is_symlink()),
        key=lambda path: path.stat().st_mtime_ns,
    )
    converted: list[dict[str, object]] = []
    skipped: list[str] = []
    failures: list[dict[str, object]] = []
    for capture in captures:
        destination = default_output_dir(capture)
        if destination.is_dir() and (destination / "portable_manifest.json").is_file():
            skipped.append(capture.name)
            continue
        if destination.exists():
            failures.append({
                "capture": capture.name,
                "error": f"existing incomplete/nonportable evidence target: {destination}",
            })
            continue
        try:
            result = run_default(
                capture,
                output_dir=destination,
                hz=hz,
                replay_mode=replay_mode,
                replay_sweep_enabled=replay_mode != "off",
                sweep_window_ticks=sweep_window_ticks,
                pytest_scope="off",
            )
            converted.append({
                "capture": capture.name,
                "status": result.get("status"),
                "replay_status": result.get("replay_status"),
                "output_dir": result.get("output_dir"),
            })
        except (OSError, TypeError, ValueError, RuntimeError) as exc:
            failures.append({"capture": capture.name, "error": str(exc)})
    return {
        "status": "FAIL" if failures else "PASS",
        "capture_dir": str(root.resolve()),
        "found_count": len(captures),
        "converted_count": len(converted),
        "skipped_count": len(skipped),
        "failure_count": len(failures),
        "converted": converted,
        "skipped": skipped,
        "failures": failures,
    }


def _read_view_file(path: Path) -> Mapping[str, object]:
    header: dict[str, object] | None = None
    windows: list[dict[str, object]] = []
    events: list[dict[str, object]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        if not isinstance(row, dict):
            continue
        if row.get("row_type") == "header" and header is None:
            header = row
        elif row.get("row_type") == "window":
            windows.append(row)
        elif row.get("row_type") == "event":
            events.append(row)
    if header is None:
        raise ValueError(f"overview lacks header: {path}")
    result = {key: value for key, value in header.items() if key != "row_type"}
    result["windows"] = windows
    result["events"] = events
    return result


def _load_or_build_view(path: Path, hz: int) -> Mapping[str, object]:
    if path.suffix.lower() == ".mcap":
        return build_run_view(path, hz=hz)
    if path.is_dir():
        agent_path = path / "agent_view.json"
        if agent_path.is_file():
            payload = json.loads(agent_path.read_text(encoding="utf-8"))
            overview = payload.get("overview") if isinstance(payload, Mapping) else None
            if isinstance(overview, str):
                candidate = Path(overview)
                if not candidate.is_absolute():
                    candidate = path / candidate
                if candidate.is_file():
                    return _read_view_file(candidate)
        fallback = path / f"overview_{hz}hz.ndjson"
        if fallback.is_file():
            return _read_view_file(fallback)
        raise ValueError(f"evidence directory lacks a usable overview: {path}")
    raise ValueError("compare inputs must be MCAP files or portable evidence directories")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="R2B4 Test Hub. No arguments = convert newest MCAP to portable evidence."
    )
    commands = parser.add_subparsers(dest="command")

    run = commands.add_parser("run", help="convert/analyze one finished MCAP")
    run.add_argument("capture", nargs="?", help="default: newest runtime/captures/*.mcap")
    run.add_argument("--hz", type=int, choices=SUPPORTED_HZ, default=DEFAULT_HZ)
    run.add_argument("--output-dir")
    run.add_argument("--replay", choices=("off", "incident", "full"), default="incident")
    run.add_argument("--no-sweep", action="store_true", help="skip full-run bounded replay sweep")
    run.add_argument("--sweep-window-ticks", type=int, default=DEFAULT_REPLAY_WINDOW_TICKS)
    run.add_argument("--pytest", choices=("off", "testhub", "full"), default="off")

    view = commands.add_parser("view", help="cheap readable view only")
    view.add_argument("capture", nargs="?", help="default: newest runtime/captures/*.mcap")
    view.add_argument("--hz", type=int, choices=SUPPORTED_HZ, default=DEFAULT_HZ)
    view.add_argument("--output")

    batch = commands.add_parser("batch", help="convert every unprocessed MCAP in a capture directory")
    batch.add_argument("--capture-dir", default=str(DEFAULT_CAPTURE_DIR))
    batch.add_argument("--hz", type=int, choices=SUPPORTED_HZ, default=DEFAULT_HZ)
    batch.add_argument("--replay", choices=("off", "incident", "full"), default="incident")
    batch.add_argument("--sweep-window-ticks", type=int, default=DEFAULT_REPLAY_WINDOW_TICKS)

    compare = commands.add_parser("compare", help="objective before/after comparison")
    compare.add_argument("before")
    compare.add_argument("after")
    compare.add_argument("--hz", type=int, choices=SUPPORTED_HZ, default=DEFAULT_HZ)
    compare.add_argument("--output")

    tests = commands.add_parser("test", help="run pytest under the Test Hub umbrella")
    tests.add_argument("--scope", choices=("testhub", "full"), default="testhub")
    tests.add_argument("--output")
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
                replay_sweep_enabled=not getattr(args, "no_sweep", False),
                sweep_window_ticks=getattr(args, "sweep_window_ticks", DEFAULT_REPLAY_WINDOW_TICKS),
                pytest_scope=getattr(args, "pytest", "off"),
            )
        elif command == "view":
            capture = resolve_capture(args.capture)
            output = (
                Path(args.output)
                if args.output
                else capture.with_name(capture.name + f".overview_{args.hz}hz.ndjson")
            )
            view_result = build_run_view(capture, hz=args.hz, output_path=output)
            result = {
                "status": "PASS",
                "capture": str(capture.resolve()),
                "hz": args.hz,
                "overview": str(output.resolve()),
                "phase_count": len(view_result["phases"]),
                "event_count": len(view_result["events"]),
            }
        elif command == "batch":
            result = run_pending(
                args.capture_dir,
                hz=args.hz,
                replay_mode=args.replay,
                sweep_window_ticks=args.sweep_window_ticks,
            )
        elif command == "compare":
            before_path = Path(args.before)
            after_path = Path(args.after)
            result = compare_views(
                _load_or_build_view(before_path, args.hz),
                _load_or_build_view(after_path, args.hz),
            )
            result = dict(result)
            result["motion_quality"] = compare_motion_quality_sources(before_path, after_path)
            result["localization_quality"] = compare_localization_quality_sources(before_path, after_path)
            if args.output:
                out = Path(args.output)
                out.parent.mkdir(parents=True, exist_ok=True)
                with out.open("x", encoding="utf-8") as handle:
                    handle.write(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n")
                result = {**result, "output": str(out.resolve())}
        else:
            result = run_pytest(Path(__file__).resolve().parents[1], scope=args.scope)
            if args.output:
                out = Path(args.output)
                if out.exists():
                    raise FileExistsError(out)
                _write_json(out, result)
                result = {**result, "output": str(out.resolve())}

        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False))
        return 2 if result.get("status") in {"FAIL", "ERROR", "MISMATCH"} else 0
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
    "run_pending",
]
