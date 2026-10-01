"""Read-only API for sealed MCAP Evidence Compiler bundles.

This module is intentionally evidence-only: it never opens the source MCAP.  It
provides the stable consumption surface used by tools.diag and other offline
consumers, so they do not depend on bundle shard names or SQLite details.
"""
from __future__ import annotations

from contextlib import ExitStack
from dataclasses import dataclass
import json
from pathlib import Path
import re
import sqlite3
from typing import Iterator

from .index import read_only
from .query import bundle_file, read_row
from .schemas import MANIFEST
from .verify import verify, verify_manifest

_VIEW_FILE = re.compile(r"^normalized/(.+)_(\d{5})_(\d{5})\.ndjson$")
_VIEW_NAME = re.compile(r"^[A-Za-z0-9_./-]+$")


class EvidenceBundleError(RuntimeError):
    """Raised when a path is not a readable sealed EVI bundle."""


@dataclass(frozen=True, slots=True)
class EvidenceViewRow:
    view: str
    message_id: str
    source_pointer: str
    topic: str | None
    log_time_ns: int
    tick_id: object | None
    payload: object

    def as_dict(self) -> dict[str, object]:
        return {
            "view": self.view,
            "message_id": self.message_id,
            "source_pointer": self.source_pointer,
            "topic": self.topic,
            "log_time_ns": self.log_time_ns,
            "tick_id": self.tick_id,
            "payload": self.payload,
        }


@dataclass(frozen=True, slots=True)
class EvidenceBundle:
    root: Path
    manifest: dict[str, object]
    coverage: dict[str, object]
    integrity: dict[str, object]
    verification: dict[str, object]

    @classmethod
    def open(cls, path: str | Path, *, verification: str = "full") -> "EvidenceBundle":
        raw = Path(path).expanduser()
        root = raw.parent if raw.name == "manifest.json" else raw
        if root.is_symlink() or not root.is_dir():
            raise EvidenceBundleError(f"evidence bundle is not a regular directory: {root}")
        root = root.resolve()
        manifest_path = root / "manifest.json"
        if manifest_path.is_symlink() or not manifest_path.is_file():
            raise EvidenceBundleError(f"evidence manifest missing: {manifest_path}")
        try:
            if verification == "full":
                verification_result = verify(root)
                manifest = json.loads(manifest_path.read_bytes())
            elif verification == "manifest":
                manifest = verify_manifest(root)
                verification_result = {
                    "status": "PASS",
                    "scope": "SEALED_MANIFEST",
                    "source_checked": False,
                    "compiler_status": manifest.get("compiler_status"),
                }
            else:
                raise ValueError("verification must be 'full' or 'manifest'")
            coverage = json.loads(bundle_file(root, "coverage.json").read_bytes())
            integrity = json.loads(bundle_file(root, "integrity.json").read_bytes())
        except (OSError, ValueError, json.JSONDecodeError, sqlite3.Error) as exc:
            raise EvidenceBundleError(str(exc)) from exc
        if manifest.get("schema") != MANIFEST:
            raise EvidenceBundleError("not an MCAP Evidence Compiler bundle")
        return cls(
            root=root,
            manifest=manifest,
            coverage=coverage,
            integrity=integrity,
            verification=verification_result,
        )

    def _connect(self) -> sqlite3.Connection:
        try:
            return read_only(self.root)
        except sqlite3.Error as exc:
            raise EvidenceBundleError(f"cannot open evidence index: {exc}") from exc

    @staticmethod
    def _validate_view(view: str) -> str:
        if not view or view.startswith("/") or ".." in Path(view).parts or not _VIEW_NAME.fullmatch(view):
            raise ValueError(f"invalid evidence view name: {view!r}")
        return view

    @staticmethod
    def _decode_tick_id(raw: str | None) -> object | None:
        if raw is None:
            return None
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return raw

    def view_names(self) -> tuple[str, ...]:
        db = self._connect()
        names: set[str] = set()
        try:
            for (file_name,) in db.execute("SELECT DISTINCT file FROM views ORDER BY file"):
                match = _VIEW_FILE.match(str(file_name))
                if match:
                    names.add(match.group(1))
        finally:
            db.close()
        return tuple(sorted(names))

    def has_view(self, view: str) -> bool:
        view = self._validate_view(view)
        pattern = f"normalized/{view}_?????_?????.ndjson"
        db = self._connect()
        try:
            return db.execute("SELECT 1 FROM views WHERE file GLOB ? LIMIT 1", (pattern,)).fetchone() is not None
        finally:
            db.close()

    def view_count(self, view: str) -> int:
        view = self._validate_view(view)
        pattern = f"normalized/{view}_?????_?????.ndjson"
        db = self._connect()
        try:
            return int(db.execute("SELECT count(*) FROM views WHERE file GLOB ?", (pattern,)).fetchone()[0])
        finally:
            db.close()

    def view_counts(self) -> dict[str, int]:
        db = self._connect()
        result: dict[str, int] = {}
        try:
            for file_name, count in db.execute("SELECT file,count(*) FROM views GROUP BY file ORDER BY file"):
                match = _VIEW_FILE.match(str(file_name))
                if match:
                    name = match.group(1)
                    result[name] = result.get(name, 0) + int(count)
        finally:
            db.close()
        return dict(sorted(result.items()))

    def iter_view(
        self,
        view: str,
        *,
        start_ns: int | None = None,
        end_ns: int | None = None,
        limit: int | None = None,
    ) -> Iterator[EvidenceViewRow]:
        view = self._validate_view(view)
        if limit is not None and limit < 1:
            raise ValueError("limit must be positive")
        clauses = ["v.file GLOB ?"]
        params: list[object] = [f"normalized/{view}_?????_?????.ndjson"]
        if start_ns is not None:
            clauses.append("m.log_time_ns>=?")
            params.append(f"{int(start_ns):020d}")
        if end_ns is not None:
            clauses.append("m.log_time_ns<=?")
            params.append(f"{int(end_ns):020d}")
        sql = (
            "SELECT v.message_id,v.pointer,v.file,v.byte_offset,v.byte_length,"
            "m.topic,m.log_time_ns,m.tick_id FROM views v JOIN messages m ON m.message_id=v.message_id "
            "WHERE " + " AND ".join(clauses) + " ORDER BY m.log_time_ns,m.message_id,v.pointer"
        )
        if limit is not None:
            sql += " LIMIT ?"
            params.append(int(limit))

        db = self._connect()
        try:
            rows = db.execute(sql, params)
            with ExitStack() as stack:
                streams: dict[str, object] = {}
                for message_id, pointer, file_name, offset, length, topic, log_ns, tick_raw in rows:
                    relative = str(file_name)
                    stream = streams.get(relative)
                    if stream is None:
                        stream = stack.enter_context(bundle_file(self.root, relative).open("rb"))
                        streams[relative] = stream
                    stream.seek(int(offset))
                    raw = stream.read(int(length))
                    if len(raw) != int(length):
                        raise EvidenceBundleError(f"truncated evidence row: {relative}@{offset}")
                    row = json.loads(raw)
                    if row.get("message_id") != message_id or row.get("source_pointer") != pointer:
                        raise EvidenceBundleError(f"evidence index/row identity mismatch: {message_id}")
                    yield EvidenceViewRow(
                        view=view,
                        message_id=str(message_id),
                        source_pointer=str(pointer),
                        topic=topic,
                        log_time_ns=int(log_ns),
                        tick_id=self._decode_tick_id(tick_raw),
                        payload=row.get("payload"),
                    )
        finally:
            db.close()

    def source_message(self, message_id: str) -> dict[str, object]:
        db = self._connect()
        try:
            row = db.execute(
                "SELECT file,byte_offset,byte_length FROM messages WHERE message_id=?",
                (message_id,),
            ).fetchone()
        finally:
            db.close()
        if row is None:
            raise KeyError(f"unknown evidence message_id: {message_id}")
        return read_row(self.root, *row)

    def topic_counts(self) -> dict[str, int]:
        db = self._connect()
        try:
            rows = db.execute(
                "SELECT coalesce(topic,'<unknown>'),count(*) FROM messages GROUP BY topic ORDER BY topic"
            ).fetchall()
        finally:
            db.close()
        return {str(topic): int(count) for topic, count in rows}

    def message_count(self) -> int:
        db = self._connect()
        try:
            return int(db.execute("SELECT count(*) FROM messages").fetchone()[0])
        finally:
            db.close()

    def message_time_bounds(self) -> tuple[int | None, int | None]:
        db = self._connect()
        try:
            first, last = db.execute("SELECT min(log_time_ns),max(log_time_ns) FROM messages").fetchone()
        finally:
            db.close()
        return (int(first) if first is not None else None, int(last) if last is not None else None)

    def topic_rate_hz(self, topic: str) -> float | None:
        db = self._connect()
        try:
            count, first, last = db.execute(
                "SELECT count(*),min(log_time_ns),max(log_time_ns) FROM messages WHERE topic=?",
                (topic,),
            ).fetchone()
        finally:
            db.close()
        if int(count) < 2 or first is None or last is None or int(last) <= int(first):
            return None
        return (int(count) - 1) * 1_000_000_000.0 / (int(last) - int(first))

    def view_alignment(self, views: tuple[str, ...]) -> dict[str, object]:
        names = tuple(self._validate_view(view) for view in views)
        if not names:
            return {"views": [], "per_view_messages": {}, "common_messages": 0}
        db = self._connect()
        try:
            per_view: dict[str, int] = {}
            for view in names:
                pattern = f"normalized/{view}_?????_?????.ndjson"
                per_view[view] = int(db.execute(
                    "SELECT count(DISTINCT message_id) FROM views WHERE file GLOB ?", (pattern,)
                ).fetchone()[0])
            clauses = [
                "EXISTS (SELECT 1 FROM views v WHERE v.message_id=m.message_id AND v.file GLOB ?)"
                for _ in names
            ]
            params = [f"normalized/{view}_?????_?????.ndjson" for view in names]
            common = int(db.execute(
                "SELECT count(*) FROM messages m WHERE " + " AND ".join(clauses), params
            ).fetchone()[0])
        finally:
            db.close()
        return {"views": list(names), "per_view_messages": per_view, "common_messages": common}

    def field_census(self, *, prefix: str | None = None) -> dict[str, int]:
        db = self._connect()
        try:
            if prefix is None:
                rows = db.execute("SELECT path,count(*) FROM fields GROUP BY path ORDER BY path")
            else:
                rows = db.execute(
                    "SELECT path,count(*) FROM fields WHERE path LIKE ? GROUP BY path ORDER BY path",
                    (prefix + "%",),
                )
            return {str(path): int(count) for path, count in rows}
        finally:
            db.close()


__all__ = ["EvidenceBundle", "EvidenceBundleError", "EvidenceViewRow"]
