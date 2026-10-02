"""Bounded read-only source/document/config tools for AgentCore."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from collections.abc import Mapping

from .agent_contracts import AgentToolSpec

_DENY_PARTS = frozenset({
    ".git", "runtime", ".upgrade_backups", "__pycache__", ".pytest_cache",
    ".mypy_cache", ".venv", "venv", "old",
})
_SOURCE_SUFFIXES = frozenset({".py", ".sh"})
_DOC_SUFFIXES = frozenset({".md"})
_CONFIG_FILES = frozenset({"hardver.json", "fizika.json", "speed_map.json", "vezerles.json"})


def _args(value: Mapping[str, object], allowed: set[str]) -> dict[str, object]:
    raw = dict(value)
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ValueError("unknown arguments: " + ", ".join(unknown))
    return raw


def _safe_relative(root: Path, raw: object, *, suffixes: frozenset[str], allow_root_r: bool = False) -> Path:
    if not isinstance(raw, str) or not raw.strip():
        raise ValueError("path must be non-empty text")
    relative = Path(raw.strip())
    if relative.is_absolute() or ".." in relative.parts or any(part in _DENY_PARTS for part in relative.parts):
        raise ValueError("path is outside the readable R2B4 surface")
    path = root / relative
    if path.is_symlink() or not path.is_file() or not path.resolve().is_relative_to(root):
        raise ValueError("path is not a regular repository file")
    if path.name.startswith("."):
        raise ValueError("hidden files are not exposed to the LLM")
    if path.suffix not in suffixes and not (allow_root_r and relative == Path("r")):
        raise ValueError("file type is not exposed by this tool")
    return path


def _candidate_files(root: Path, suffixes: frozenset[str], *, allow_root_r: bool = False):
    for path in root.rglob("*"):
        if not path.is_file() or path.is_symlink():
            continue
        relative = path.relative_to(root)
        if any(part in _DENY_PARTS for part in relative.parts) or path.name.startswith("."):
            continue
        if path.suffix in suffixes or (allow_root_r and relative == Path("r")):
            yield path


def _search(root: Path, value: Mapping[str, object], *, docs: bool) -> object:
    args = _args(value, {"query", "limit"})
    query = args.get("query")
    if not isinstance(query, str) or not query.strip():
        raise ValueError("query must be non-empty text")
    query_cf = query.casefold()
    limit = args.get("limit", 20)
    if not isinstance(limit, int) or isinstance(limit, bool) or not 1 <= limit <= 50:
        raise ValueError("limit must be an integer within [1, 50]")
    suffixes = _DOC_SUFFIXES if docs else _SOURCE_SUFFIXES
    hits: list[dict[str, object]] = []
    for path in _candidate_files(root, suffixes, allow_root_r=not docs):
        try:
            if path.stat().st_size > 2_000_000:
                continue
            lines = path.read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeError):
            continue
        for number, line in enumerate(lines, 1):
            if query_cf in line.casefold():
                hits.append({
                    "path": str(path.relative_to(root)),
                    "line": number,
                    "text": line.strip()[:500],
                })
                if len(hits) >= limit:
                    return {"query": query, "hits": hits, "truncated": True}
    return {"query": query, "hits": hits, "truncated": False}


def _read(root: Path, value: Mapping[str, object], *, docs: bool) -> object:
    args = _args(value, {"path", "start_line", "end_line"})
    path = _safe_relative(
        root,
        args.get("path"),
        suffixes=_DOC_SUFFIXES if docs else _SOURCE_SUFFIXES,
        allow_root_r=not docs,
    )
    start = args.get("start_line", 1)
    end = args.get("end_line")
    if not isinstance(start, int) or isinstance(start, bool) or start < 1:
        raise ValueError("start_line must be a positive integer")
    if end is None:
        end = start + 199
    if not isinstance(end, int) or isinstance(end, bool) or end < start:
        raise ValueError("end_line must be >= start_line")
    if end - start + 1 > 240:
        raise ValueError("at most 240 lines may be read at once")
    lines = path.read_text(encoding="utf-8").splitlines()
    selected = lines[start - 1 : end]
    text = "\n".join(f"{start + index}: {line}" for index, line in enumerate(selected))
    if len(text) > 20_000:
        text = text[:20_000] + "\n...[truncated]"
    return {
        "path": str(path.relative_to(root)),
        "start_line": start,
        "end_line": start + max(0, len(selected) - 1),
        "content": text,
    }


def _config_read(root: Path, value: Mapping[str, object]) -> object:
    args = _args(value, {"config", "path"})
    name = args.get("config", "vezerles.json")
    if not isinstance(name, str) or name not in _CONFIG_FILES:
        raise ValueError("config must be one of the four production config files")
    path = root / "conf" / name
    if path.is_symlink() or not path.is_file():
        raise ValueError("production config is missing or unsafe")
    raw = path.read_bytes()
    document = json.loads(raw)
    dotted = args.get("path")
    value_out: object = document
    if dotted is not None:
        if not isinstance(dotted, str) or not dotted.strip():
            raise ValueError("path must be non-empty text")
        for part in dotted.split("."):
            if not isinstance(value_out, Mapping) or part not in value_out:
                raise KeyError(f"unknown config path: {dotted}")
            value_out = value_out[part]
    return {
        "config": name,
        "path": dotted,
        "value": value_out,
        "sha256": hashlib.sha256(raw).hexdigest(),
    }


def build_source_tools(project_root: Path):
    root = Path(project_root).resolve()
    return (
        (
            AgentToolSpec(
                "source.search",
                "Search current R2B4 Python/shell source and tests. Runtime, old backups, hidden and secret files are excluded.",
                "READ",
                {"query": "required string", "limit": "optional integer 1..50"},
            ),
            lambda args: _search(root, args, docs=False),
        ),
        (
            AgentToolSpec(
                "source.read",
                "Read a bounded line range from current R2B4 Python/shell source or tests.",
                "READ",
                {"path": "required repository-relative path", "start_line": "optional positive integer", "end_line": "optional integer; max 240 lines"},
            ),
            lambda args: _read(root, args, docs=False),
        ),
        (
            AgentToolSpec(
                "docs.search",
                "Search current R2B4 Markdown contracts and documentation.",
                "READ",
                {"query": "required string", "limit": "optional integer 1..50"},
            ),
            lambda args: _search(root, args, docs=True),
        ),
        (
            AgentToolSpec(
                "docs.read",
                "Read a bounded line range from current R2B4 Markdown contracts/documentation.",
                "READ",
                {"path": "required repository-relative .md path", "start_line": "optional positive integer", "end_line": "optional integer; max 240 lines"},
            ),
            lambda args: _read(root, args, docs=True),
        ),
        (
            AgentToolSpec(
                "config.read",
                "Read the active production JSON config or one dotted path. This tool never modifies config.",
                "READ",
                {"config": "optional hardver.json|fizika.json|speed_map.json|vezerles.json", "path": "optional dotted path"},
            ),
            lambda args: _config_read(root, args),
        ),
    )


__all__ = ["build_source_tools"]
