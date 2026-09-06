"""Small run-bound Test Hub V3 over the offline Replayer V3 authority."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
from collections.abc import Mapping, Sequence
from pathlib import Path

from .capture import LAYER_ORDER, V3_CAPTURE_SCHEMA
from .replay import (
    ReplaySelection,
    V3_REPLAY_STATUS_MATCH,
    V3ReplayError,
    inspect_capture,
    replay_capture,
    verify_replay_result,
    write_replay_result,
)


V3_DIAGNOSIS_SCHEMA = "R2B4_V3_DIAGNOSIS_V1"
V3_EVIDENCE_INDEX_SCHEMA = "R2B4_V3_EVIDENCE_INDEX_V1"


class V3TestHubError(RuntimeError):
    """A run-bound validation request or evidence destination is invalid."""


def validate_run(
    capture_path: str | Path,
    output_dir: str | Path,
    *,
    selection: ReplaySelection | None = None,
    physics_config_path: str | Path = "conf/fizika.json",
    speed_map_config_path: str | Path = "conf/speed_map.json",
    hardware_config_path: str | Path = "conf/hardver.json",
    project_root: str | Path | None = None,
    capture_source_manifest_path: str | Path | None = None,
) -> dict[str, object]:
    """Create one immutable-style evidence directory without profiles or pointers."""

    destination = Path(output_dir)
    if destination.is_symlink():
        raise V3TestHubError("output directory must not be a symlink")
    if destination.exists() and (not destination.is_dir() or any(destination.iterdir())):
        raise V3TestHubError("output directory must be absent or empty")
    destination.mkdir(parents=True, exist_ok=True)

    inspected = inspect_capture(capture_path)
    replay = replay_capture(
        capture_path,
        selection=selection,
        physics_config_path=physics_config_path,
        speed_map_config_path=speed_map_config_path,
        hardware_config_path=hardware_config_path,
        project_root=project_root,
        capture_source_manifest_path=capture_source_manifest_path,
    )
    inspect_path = _write_json(inspected, destination / "inspect.json")
    replay_path = write_replay_result(replay, destination / "replay_result.json")
    diagnosis = _diagnosis(inspected, replay)
    diagnosis_path = _write_json(diagnosis, destination / "diagnosis.json")
    artifacts = {
        path.name: {"sha256": _sha256_file(path), "size_bytes": path.stat().st_size}
        for path in (inspect_path, replay_path, diagnosis_path)
    }
    index: dict[str, object] = {
        "schema": V3_EVIDENCE_INDEX_SCHEMA,
        "run_id": destination.name,
        "created_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "status": diagnosis["status"],
        "capture": {
            "path": str(Path(capture_path).resolve()),
            "sha256": _sha256_file(Path(capture_path)),
            "capture_id": inspected.get("capture_id"),
        },
        "scope": replay.get("scope"),
        "artifacts": artifacts,
    }
    index["evidence_sha256"] = _payload_sha256(index)
    index_path = _write_json(index, destination / "evidence_index.json")
    return {
        "status": diagnosis["status"],
        "run_id": destination.name,
        "output_dir": str(destination.resolve()),
        "capture_status": inspected.get("execution_status"),
        "replay_status": replay.get("status"),
        "first_divergence": replay.get("first_divergence"),
        "scope": replay.get("scope"),
        "evidence_index": str(index_path.resolve()),
        "evidence_sha256": index["evidence_sha256"],
    }


def verify_evidence(index_path: str | Path) -> dict[str, object]:
    """Verify the run-bound index checksum and every indexed artifact."""

    path = Path(index_path)
    if path.is_symlink() or not path.is_file():
        raise V3TestHubError("evidence index must be a regular non-symlink file")
    try:
        index = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise V3TestHubError("evidence index must contain valid UTF-8 JSON") from exc
    if not isinstance(index, dict):
        raise V3TestHubError("evidence index root must be an object")
    checksum_ok = (
        index.get("schema") == V3_EVIDENCE_INDEX_SCHEMA
        and isinstance(index.get("evidence_sha256"), str)
        and index["evidence_sha256"] == _payload_sha256(index)
    )
    artifacts = index.get("artifacts")
    artifact_rows: dict[str, object] = {}
    if isinstance(artifacts, Mapping):
        for name, raw_row in artifacts.items():
            safe_name = str(name)
            row = raw_row if isinstance(raw_row, Mapping) else {}
            artifact_path = path.parent / safe_name
            safe = Path(safe_name).name == safe_name and safe_name not in {"", ".", ".."}
            regular = safe and not artifact_path.is_symlink() and artifact_path.is_file()
            actual_hash = _sha256_file(artifact_path) if regular else None
            actual_size = artifact_path.stat().st_size if regular else None
            matches = bool(
                regular
                and row.get("sha256") == actual_hash
                and row.get("size_bytes") == actual_size
            )
            artifact_rows[safe_name] = {
                "regular_file": regular,
                "sha256": actual_hash,
                "size_bytes": actual_size,
                "matches_index": matches,
            }
    expected_names = {"inspect.json", "replay_result.json", "diagnosis.json"}
    artifacts_ok = set(artifact_rows) == expected_names and all(
        isinstance(row, Mapping) and row.get("matches_index") is True
        for row in artifact_rows.values()
    )
    valid = checksum_ok and artifacts_ok
    return {
        "status": "PASS" if valid else "FAIL",
        "index_path": str(path.resolve()),
        "index_checksum_ok": checksum_ok,
        "artifacts_ok": artifacts_ok,
        "artifacts": artifact_rows,
    }


def _diagnosis(
    inspected: Mapping[str, object],
    replay: Mapping[str, object],
) -> dict[str, object]:
    raw_diagnostics = replay.get("diagnostics")
    raw_layers = (
        raw_diagnostics.get("layers")
        if isinstance(raw_diagnostics, Mapping)
        else None
    )
    selected_layers = set(
        replay.get("scope", {}).get("resolved", {}).get("layers", LAYER_ORDER)
        if isinstance(replay.get("scope"), Mapping)
        else LAYER_ORDER
    )
    layers: dict[str, object] = {}
    general_capture = inspected.get("schema") == V3_CAPTURE_SCHEMA
    for layer in LAYER_ORDER:
        if isinstance(raw_layers, Mapping) and isinstance(raw_layers.get(layer), Mapping):
            layers[layer] = {
                **dict(raw_layers[layer]),
                "selected": True,
                "authority": "REPLAYER_V3",
            }
        elif general_capture:
            layers[layer] = {
                "selected": False,
                "compared_tick_count": 0,
                "mismatch_count": 0,
                "authority": "OUT_OF_SCOPE",
            }
        else:
            layers[layer] = {
                "selected": layer in selected_layers,
                "compared_tick_count": None,
                "mismatch_count": (
                    1
                    if isinstance(replay.get("first_divergence"), Mapping)
                    and replay["first_divergence"].get("layer") == layer
                    else 0
                ),
                "authority": "LEGACY_COMPATIBILITY" if layer in selected_layers else "OUT_OF_SCOPE",
            }
    status = "PASS" if replay.get("status") == V3_REPLAY_STATUS_MATCH else "FAIL"
    return {
        "schema": V3_DIAGNOSIS_SCHEMA,
        "status": status,
        "capture_id": inspected.get("capture_id"),
        "capture_execution_status": inspected.get("execution_status"),
        "replay_status": replay.get("status"),
        "scope": replay.get("scope"),
        "layers": layers,
        "first_divergence": replay.get("first_divergence"),
    }


def _write_json(payload: Mapping[str, object], path: Path) -> Path:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    return path


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _payload_sha256(value: Mapping[str, object]) -> str:
    unsigned = dict(value)
    unsigned.pop("evidence_sha256", None)
    return hashlib.sha256(
        json.dumps(
            unsigned,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()


def _add_selection_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--start-tick-id", type=int)
    parser.add_argument("--end-tick-id", type=int)
    parser.add_argument("--start-monotonic-ns", type=int)
    parser.add_argument("--end-monotonic-ns", type=int)
    parser.add_argument("--start-layer", default="L1")
    parser.add_argument("--end-layer", default="L12")


def _selection(args: argparse.Namespace) -> ReplaySelection:
    return ReplaySelection(
        start_tick_id=args.start_tick_id,
        end_tick_id=args.end_tick_id,
        start_monotonic_ns=args.start_monotonic_ns,
        end_monotonic_ns=args.end_monotonic_ns,
        start_layer=args.start_layer,
        end_layer=args.end_layer,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    inspect_parser = commands.add_parser("inspect")
    inspect_parser.add_argument("capture_path")
    replay_parser = commands.add_parser("replay")
    replay_parser.add_argument("capture_path")
    replay_parser.add_argument("--output", required=True)
    _add_selection_arguments(replay_parser)
    validate_parser = commands.add_parser("validate")
    validate_parser.add_argument("capture_path")
    validate_parser.add_argument("--output-dir", required=True)
    _add_selection_arguments(validate_parser)
    for command_parser in (replay_parser, validate_parser):
        command_parser.add_argument("--physics-config", default="conf/fizika.json")
        command_parser.add_argument("--speed-map-config", default="conf/speed_map.json")
        command_parser.add_argument("--hardware-config", default="conf/hardver.json")
        command_parser.add_argument("--project-root", default=".")
        command_parser.add_argument("--capture-source-manifest")
    verify_parser = commands.add_parser("verify-result")
    verify_parser.add_argument("result_path")
    evidence_parser = commands.add_parser("verify-evidence")
    evidence_parser.add_argument("index_path")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "inspect":
            output = inspect_capture(args.capture_path)
        elif args.command == "verify-result":
            output = verify_replay_result(args.result_path)
        elif args.command == "verify-evidence":
            output = verify_evidence(args.index_path)
        elif args.command == "replay":
            result = replay_capture(
                args.capture_path,
                selection=_selection(args),
                physics_config_path=args.physics_config,
                speed_map_config_path=args.speed_map_config,
                hardware_config_path=args.hardware_config,
                project_root=args.project_root,
                capture_source_manifest_path=args.capture_source_manifest,
            )
            path = write_replay_result(result, args.output)
            output = {
                "status": result["status"],
                "result_path": str(path.resolve()),
                "result_sha256": result["result_sha256"],
                "first_divergence": result["first_divergence"],
                "scope": result.get("scope"),
            }
        else:
            output = validate_run(
                args.capture_path,
                args.output_dir,
                selection=_selection(args),
                physics_config_path=args.physics_config,
                speed_map_config_path=args.speed_map_config,
                hardware_config_path=args.hardware_config,
                project_root=args.project_root,
                capture_source_manifest_path=args.capture_source_manifest,
            )
        print(json.dumps(output, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if output.get("status") in {"PASS", V3_REPLAY_STATUS_MATCH} else 2
    except (OSError, TypeError, ValueError, V3ReplayError, V3TestHubError) as exc:
        print(json.dumps({"status": "ERROR", "error": str(exc)}, indent=2), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "V3_DIAGNOSIS_SCHEMA",
    "V3_EVIDENCE_INDEX_SCHEMA",
    "V3TestHubError",
    "validate_run",
    "verify_evidence",
]
