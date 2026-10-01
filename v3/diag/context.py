"""Capture selection and MCAP-only context construction for R2B4 DIAG."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from v3.mcap_reader import McapReadError, McapReader

from .contracts import CaptureFacts
from .coverage import CaptureCoverage, collect_capture_coverage


class DiagCaptureError(RuntimeError):
    pass


def resolve_capture(root: Path, value: str | Path | None) -> Path:
    if value is None or str(value) == "latest":
        capture_dir = root / "runtime" / "captures"
        candidates = [path for path in capture_dir.glob("*.mcap") if path.is_file() and not path.is_symlink()]
        if not candidates:
            raise DiagCaptureError(f"no MCAP capture found under {capture_dir}")
        return max(candidates, key=lambda path: (path.stat().st_mtime_ns, path.name))

    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        candidate = root / candidate
    if candidate.suffix.lower() != ".mcap":
        raise DiagCaptureError("DIAG accepts MCAP input only")
    if candidate.is_symlink() or not candidate.is_file():
        raise DiagCaptureError(f"capture is not a regular non-symlink MCAP file: {candidate}")
    return candidate


def _metadata_bool(metadata: dict[str, str], key: str, default: bool) -> bool:
    raw = metadata.get(key)
    if raw is None:
        return default
    return raw.strip().lower() == "true"


@dataclass(slots=True)
class DiagContext:
    root: Path
    reader: McapReader
    facts: CaptureFacts
    finalization: dict[str, object]
    _coverage: CaptureCoverage | None = field(default=None, init=False, repr=False)

    def coverage(self) -> CaptureCoverage:
        if self._coverage is None:
            self._coverage = collect_capture_coverage(self.reader)
        return self._coverage


def open_context(root: Path, capture_value: str | Path | None) -> DiagContext:
    path = resolve_capture(root, capture_value)
    try:
        reader = McapReader(path)
        structure = reader.inspect(verify_chunks=False)
        if not structure.data_crc_ok or not structure.summary_crc_ok:
            detail = "; ".join(structure.errors) or "MCAP CRC validation failed"
            raise DiagCaptureError(detail)
        finalization = reader.capture_integrity(require_raw_evidence=False)
    except McapReadError as exc:
        raise DiagCaptureError(str(exc)) from exc

    metadata = reader.latest_metadata("r2b4.capture") or {}
    try:
        tick_sample_hz = int(metadata.get("tick_sample_hz", "50"))
    except (TypeError, ValueError) as exc:
        raise DiagCaptureError("invalid r2b4.capture tick_sample_hz metadata") from exc

    integrity = finalization.get("integrity")
    integrity_map = integrity if isinstance(integrity, dict) else {}
    raw_requested = _metadata_bool(metadata, "raw_evidence_requested", True)
    raw_complete = integrity_map.get("raw_evidence_complete", integrity_map.get("complete")) is True

    facts = CaptureFacts(
        path=path,
        file_size=structure.file_size,
        mcap_profile=structure.profile,
        mcap_library=structure.library,
        tick_sample_hz=tick_sample_hz,
        raw_evidence_requested=raw_requested,
        raw_evidence_complete=raw_complete,
        topics=tuple(sorted(reader.channels_by_topic)),
        message_count=structure.message_count,
        message_start_time_ns=structure.message_start_time,
        message_end_time_ns=structure.message_end_time,
        data_crc_ok=structure.data_crc_ok,
        summary_crc_ok=structure.summary_crc_ok,
        integrity_complete=True,
    )
    return DiagContext(root=root, reader=reader, facts=facts, finalization=finalization)


__all__ = ["DiagCaptureError", "DiagContext", "open_context", "resolve_capture"]
