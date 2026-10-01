"""Evidence selection and verified evidence context for R2B4 DIAG."""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

from tools.mcap_evidence.reader import EvidenceBundle, EvidenceBundleError

from .contracts import EvidenceFacts


class DiagEvidenceError(RuntimeError):
    pass


def _is_bundle(path: Path) -> bool:
    return (
        path.is_dir()
        and not path.is_symlink()
        and (path / "manifest.json").is_file()
        and not (path / "manifest.json").is_symlink()
    )


def resolve_evidence(root: Path, value: str | Path | None) -> Path:
    """Resolve an explicit bundle or the newest local ``*.evidence`` bundle."""
    root = Path(root).resolve()
    if value is None or str(value) == "latest":
        capture_dir = root / "runtime" / "captures"
        candidates = [path for path in capture_dir.glob("*.evidence") if _is_bundle(path)]
        if not candidates:
            raise DiagEvidenceError(f"no EVI evidence bundle found under {capture_dir}")
        return max(
            candidates,
            key=lambda path: ((path / "manifest.json").stat().st_mtime_ns, path.name),
        ).resolve()

    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        candidate = root / candidate
    if candidate.name == "manifest.json":
        candidate = candidate.parent
    if not _is_bundle(candidate):
        raise DiagEvidenceError(f"not a sealed EVI evidence bundle: {candidate}")
    return candidate.resolve()


@dataclass(frozen=True, slots=True)
class DiagContext:
    project_root: Path
    bundle: EvidenceBundle
    facts: EvidenceFacts


def _int(mapping: dict[str, object], key: str, default: int = 0) -> int:
    value = mapping.get(key, default)
    return int(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else default


def _facts(bundle: EvidenceBundle) -> EvidenceFacts:
    manifest = bundle.manifest
    coverage = bundle.coverage
    messages = coverage.get("messages") if isinstance(coverage.get("messages"), dict) else {}
    compiler = manifest.get("compiler") if isinstance(manifest.get("compiler"), dict) else {}
    source = manifest.get("source") if isinstance(manifest.get("source"), dict) else {}
    first_ns, last_ns = bundle.message_time_bounds()
    fields = bundle.field_census()
    verification = bundle.verification
    return EvidenceFacts(
        path=bundle.root,
        source_path=str(source.get("path")) if source.get("path") is not None else None,
        source_snapshot_size=int(source.get("snapshot_size")) if isinstance(source.get("snapshot_size"), int) else None,
        source_sha256=str(source.get("sha256")) if source.get("sha256") is not None else None,
        compiler_status=str(manifest.get("compiler_status")) if manifest.get("compiler_status") is not None else None,
        source_integrity=str(manifest.get("source_integrity")) if manifest.get("source_integrity") is not None else None,
        compiler_git_commit=str(compiler.get("git_commit")) if compiler.get("git_commit") is not None else None,
        compiler_git_dirty=bool(compiler.get("git_dirty")) if compiler.get("git_dirty") is not None else None,
        message_count=bundle.message_count(),
        json_decoded_messages=_int(messages, "json_decoded"),
        quarantined_messages=_int(messages, "quarantined"),
        normalized_views=_int(coverage, "normalized_views"),
        topics=bundle.topic_counts(),
        view_counts=bundle.view_counts(),
        field_path_count=len(fields),
        message_start_time_ns=first_ns,
        message_end_time_ns=last_ns,
        tick_rate_hz_estimate=bundle.topic_rate_hz("/r2b4/tick"),
        verification_status=str(verification.get("status", "UNKNOWN")),
        verification_source_checked=bool(verification.get("source_checked", False)),
    )


def open_context(root: Path, evidence_value: str | Path | None, *, verification: str = "full") -> DiagContext:
    path = resolve_evidence(root, evidence_value)
    try:
        bundle = EvidenceBundle.open(path, verification=verification)
    except EvidenceBundleError as exc:
        raise DiagEvidenceError(str(exc)) from exc
    return DiagContext(project_root=Path(root).resolve(), bundle=bundle, facts=_facts(bundle))


__all__ = ["DiagContext", "DiagEvidenceError", "open_context", "resolve_evidence"]
