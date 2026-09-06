#!/usr/bin/env python3
"""
R2B4 GitHub backup/push helper.

Default policy, aligned with the current R2B4 repository:
- stage normal repository/source/config changes;
- archive every terminal agent candidate compactly;
- preserve terminal-candidate run evidence/logs that otherwise live only
  inside ignored candidate workspaces;
- force-add canonical live/test evidence under logs/session_* and logs/archive;
- preserve runtime capture/result/status/command/evidence artifacts;
- route capture files and every large staged file through Git LFS;
- repair a local-only commit history that already contains oversized normal
  Git blobs by creating a local safety branch and soft-resetting to origin;
- never force-push.

Not uploaded by default:
- runtime/agent_workspaces full duplicated trees;
- logs/latest symlink/pointer (run-identified sessions are authority);
- transient workspace leases;
- backup copies such as runtime/gitre.py.old / .002.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shlex
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

DEFAULT_REPO = Path("/home/alba/project_r2b4")
WORKSPACE_ROOT = Path("runtime/agent_workspaces")
ARCHIVE_ROOT = Path("runtime/candidate_archive")
TERMINAL_RESEAL_STATES = {"READY", "SUPERSEDED"}
META_FILES = ("audit.json", "diff.json", "evidence.json", "reseal.json")

DEFAULT_LFS_THRESHOLD_MIB = 50
GITHUB_HARD_LIMIT_BYTES = 100 * 1024 * 1024
LFS_POINTER_MAX_BYTES = 2048
LFS_POINTER_HEADER = b"version https://git-lfs.github.com/spec/v1\n"

RUNTIME_EVIDENCE_SUFFIXES = (
    "_capture.json",
    "_result.json",
    "_status.json",
    "_command.json",
    "_summary.json",
    "_diagnosis.json",
    "_inspect.json",
    "_replay_result.json",
    "_evidence_index.json",
)
RUNTIME_EVIDENCE_DIR_MARKERS = (
    "evidence",
    "replay",
    "incident",
)

# New/untracked files matching these are operational/transient, not durable evidence.
TRANSIENT_NEW_PATH_PREFIXES = (
    "runtime/agent_coordination/leases/",
)
TRANSIENT_NEW_BASENAME_PREFIXES = (
    "gitre.py.",
)


class GitCommandError(RuntimeError):
    pass


def utc_tag() -> str:
    return (
        datetime.now(timezone.utc)
        .strftime("%Y%m%dT%H%M%SZ")
    )


def human_mib(size_bytes: int) -> str:
    return f"{size_bytes / (1024 * 1024):.2f} MiB"


def run(
    cmd: list[str],
    cwd: Path,
    *,
    capture: bool = False,
    check: bool = True,
    input_text: str | None = None,
) -> str:
    print("+", shlex.join(cmd))
    result = subprocess.run(
        cmd,
        cwd=cwd,
        text=True,
        input=input_text,
        capture_output=capture,
    )
    if check and result.returncode != 0:
        if capture and result.stdout:
            print(result.stdout, end="")
        if capture and result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        raise GitCommandError(
            f"Parancs hibával leállt ({result.returncode}): {shlex.join(cmd)}"
        )
    return result.stdout.strip() if capture else ""


def git_quiet(
    cmd: list[str],
    cwd: Path,
    *,
    input_text: str | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=cwd,
        text=True,
        input=input_text,
        capture_output=True,
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path) -> dict:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Hibás JSON: {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise RuntimeError(f"JSON objektum szükséges: {path}")
    return payload


def write_json(path: Path, payload: dict) -> None:
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def safe_relative(root: Path, relative: str) -> Path:
    if not isinstance(relative, str) or not relative.strip():
        raise RuntimeError("Üres candidate fájlútvonal.")
    path = Path(relative)
    if path.is_absolute():
        raise RuntimeError(f"Abszolút candidate fájlútvonal tiltott: {relative}")
    resolved = (root / path).resolve(strict=False)
    try:
        resolved.relative_to(root.resolve())
    except ValueError as exc:
        raise RuntimeError(
            f"Candidate fájl kilép a workspace-ből: {relative}"
        ) from exc
    return resolved


def iter_regular_files(root: Path) -> Iterable[Path]:
    if not root.exists():
        return
    if root.is_symlink():
        return
    if root.is_file():
        yield root
        return
    for path in sorted(root.rglob("*")):
        if path.is_symlink():
            continue
        if path.is_file():
            yield path


def copy_file_if_changed(source: Path, target: Path) -> tuple[bool, str, int]:
    size = source.stat().st_size
    source_sha = sha256_file(source)

    if target.is_file() and not target.is_symlink():
        if target.stat().st_size == size and sha256_file(target) == source_sha:
            return False, source_sha, size

    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    return True, source_sha, size


def is_runtime_evidence_file(path: Path) -> bool:
    name = path.name.lower()
    if name.endswith(RUNTIME_EVIDENCE_SUFFIXES):
        return True
    # Preserve common run-identified structured artifacts even if their exact
    # suffix evolves.
    if path.suffix.lower() in {".json", ".jsonl", ".txt", ".csv", ".svg"}:
        if any(marker in name for marker in ("evidence", "diagnosis", "replay", "incident")):
            return True
    return False


def candidate_evidence_sources(tree: Path) -> list[tuple[Path, Path]]:
    """
    Return (source, relative_to_candidate_tree) evidence roots/files.

    Candidate source changes remain in files/.  This collects only run evidence
    that the original compact archive intentionally did not duplicate.
    """
    out: list[tuple[Path, Path]] = []

    logs_root = tree / "logs"
    if logs_root.is_dir():
        for child in sorted(logs_root.iterdir()):
            if child.is_symlink():
                continue
            if child.name.startswith("session_") or child.name == "archive":
                out.append((child, child.relative_to(tree)))
        latest = logs_root / "latest"
        # latest is normally a symlink and is explicitly non-authoritative.
        # If an older runtime left it as a real directory, preserve it too.
        if latest.is_dir() and not latest.is_symlink():
            out.append((latest, latest.relative_to(tree)))

    runtime_root = tree / "runtime"
    if runtime_root.is_dir():
        for child in sorted(runtime_root.iterdir()):
            if child.is_symlink():
                continue
            if child.is_file() and is_runtime_evidence_file(child):
                out.append((child, child.relative_to(tree)))
            elif child.is_dir() and any(
                marker in child.name.lower()
                for marker in RUNTIME_EVIDENCE_DIR_MARKERS
            ):
                # Do not recurse into infrastructure/workspace roots merely
                # because a task name contains an evidence-like word.
                if child.name not in {
                    "agent_workspaces",
                    "candidate_archive",
                    "agent_coordination",
                    "agent_runtime",
                }:
                    out.append((child, child.relative_to(tree)))

    return out


def sync_candidate_evidence(
    tree: Path,
    archive_dir: Path,
    manifest: dict,
) -> tuple[int, int]:
    evidence_manifest = manifest.get("evidence_files")
    if not isinstance(evidence_manifest, dict):
        evidence_manifest = {}

    changed_count = 0
    copied_bytes = 0

    for source_root, relative_root in candidate_evidence_sources(tree):
        if source_root.is_file():
            files = [source_root]
            source_base = source_root.parent
            rel_base = relative_root.parent
        else:
            files = list(iter_regular_files(source_root))
            source_base = source_root
            rel_base = relative_root

        for source in files:
            inside = (
                Path(source.name)
                if source_root.is_file()
                else source.relative_to(source_base)
            )
            candidate_relative = (rel_base / inside).as_posix()
            target = archive_dir / "evidence" / candidate_relative

            changed, digest, size = copy_file_if_changed(source, target)
            if changed:
                changed_count += 1
                copied_bytes += size

            evidence_manifest[candidate_relative] = {
                "archive_path": (
                    Path("evidence") / candidate_relative
                ).as_posix(),
                "sha256": digest,
                "size_bytes": size,
            }

    manifest["evidence_files"] = dict(sorted(evidence_manifest.items()))
    manifest["evidence_file_count"] = len(evidence_manifest)
    manifest["evidence_note"] = (
        "Run-identified candidate evidence copied from candidate logs/runtime; "
        "logs/latest symlink is not authority and duplicated full workspace trees are not archived."
    )
    return changed_count, copied_bytes


def _candidate_core_payload(
    repo: Path,
    task_dir: Path,
    *,
    output_dir: Path,
    audit: dict,
    reseal: dict,
    diff: dict,
    candidate_fingerprint: str,
) -> dict:
    task_id = task_dir.name
    tree = task_dir / "tree"
    meta = task_dir / "meta"

    archived_meta: dict[str, str] = {}
    archived_files: dict[str, dict] = {}
    changed_rows: list[dict] = []

    meta_out = output_dir / "meta"
    meta_out.mkdir(parents=True, exist_ok=True)
    for name in META_FILES:
        source = meta / name
        if source.is_file():
            target = meta_out / name
            shutil.copy2(source, target)
            archived_meta[name] = sha256_file(target)

    marker = tree / ".r2b4_candidate.json"
    if marker.is_file():
        shutil.copy2(marker, output_dir / ".r2b4_candidate.json")

    task_state = tree / "runtime" / "agent_coordination" / "current_change.json"
    if task_state.is_file():
        shutil.copy2(task_state, output_dir / "task_manifest.json")

    rows = diff.get("files")
    if not isinstance(rows, list):
        raise RuntimeError(f"{task_id}: diff.json files mezője hibás.")

    for row in rows:
        if not isinstance(row, dict):
            raise RuntimeError(f"{task_id}: hibás diff sor.")
        relative = str(row.get("path", ""))
        change = str(row.get("change", "")).upper()
        if change not in {"ADDED", "MODIFIED", "DELETED"}:
            raise RuntimeError(
                f"{task_id}: ismeretlen diff change: {change}"
            )
        changed_rows.append({"path": relative, "change": change})

        if change == "DELETED":
            continue

        source = safe_relative(tree, relative)
        if source.is_symlink() or not source.is_file():
            raise RuntimeError(
                f"{task_id}: a candidate módosított fájlja hiányzik vagy "
                f"nem regular file: {relative}"
            )
        target = output_dir / "files" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        archived_files[relative] = {
            "sha256": sha256_file(target),
            "size_bytes": target.stat().st_size,
        }

    return {
        "schema": "R2B4_GITHUB_CANDIDATE_ARCHIVE_V2",
        "task_id": task_id,
        "archived_at_utc": (
            datetime.now(timezone.utc)
            .replace(microsecond=0)
            .isoformat()
            .replace("+00:00", "Z")
        ),
        "source_workspace": str((WORKSPACE_ROOT / task_id).as_posix()),
        "reseal_state": str(reseal.get("state", "")).upper(),
        "base_fingerprint": audit.get("base_fingerprint"),
        "candidate_fingerprint": candidate_fingerprint,
        "changed_file_count": len(changed_rows),
        "changes": changed_rows,
        "archived_files": archived_files,
        "meta_sha256": archived_meta,
        "note": (
            "Terminal candidate archive: changed source/config files, compact "
            "workflow metadata and run-identified evidence/logs. The duplicated "
            "full workspace tree is intentionally not stored."
        ),
    }


def archive_candidate(repo: Path, task_dir: Path) -> str:
    task_id = task_dir.name
    tree = task_dir / "tree"
    meta = task_dir / "meta"
    diff_path = meta / "diff.json"
    audit_path = meta / "audit.json"
    reseal_path = meta / "reseal.json"

    if not (
        tree.is_dir()
        and diff_path.is_file()
        and audit_path.is_file()
        and reseal_path.is_file()
    ):
        return "SKIP_ACTIVE"

    audit = read_json(audit_path)
    reseal = read_json(reseal_path)
    diff = read_json(diff_path)

    if str(audit.get("status", "")).upper() != "PASS":
        return "SKIP_FAILED"

    reseal_state = str(reseal.get("state", "")).upper()
    if reseal_state not in TERMINAL_RESEAL_STATES:
        return "SKIP_ACTIVE"

    candidate_fingerprint = str(
        reseal.get("candidate_fingerprint")
        or audit.get("candidate_fingerprint")
        or diff.get("candidate_fingerprint")
        or ""
    )
    if not candidate_fingerprint:
        raise RuntimeError(
            f"{task_id}: hiányzik a candidate fingerprint."
        )

    archive_dir = repo / ARCHIVE_ROOT / task_id
    archive_manifest = archive_dir / "archive_manifest.json"

    if archive_manifest.is_file():
        existing = read_json(archive_manifest)
        if (
            str(existing.get("candidate_fingerprint", ""))
            != candidate_fingerprint
        ):
            raise RuntimeError(
                f"{task_id}: a már archivált candidate fingerprintje eltér; "
                "nem írom felül automatikusan."
            )

        changed, copied_bytes = sync_candidate_evidence(
            tree, archive_dir, existing
        )
        if changed:
            existing["evidence_synced_at_utc"] = (
                datetime.now(timezone.utc)
                .replace(microsecond=0)
                .isoformat()
                .replace("+00:00", "Z")
            )
            write_json(archive_manifest, existing)
            print(
                f"Candidate evidence frissítve: {task_id} "
                f"({changed} fájl, {human_mib(copied_bytes)})"
            )
            return "EVIDENCE_UPDATED"
        return "ALREADY_ARCHIVED"

    temp_dir = repo / ARCHIVE_ROOT / f".{task_id}.tmp"
    if temp_dir.exists():
        shutil.rmtree(temp_dir)
    temp_dir.mkdir(parents=True, exist_ok=False)

    try:
        manifest = _candidate_core_payload(
            repo,
            task_dir,
            output_dir=temp_dir,
            audit=audit,
            reseal=reseal,
            diff=diff,
            candidate_fingerprint=candidate_fingerprint,
        )
        changed, copied_bytes = sync_candidate_evidence(
            tree, temp_dir, manifest
        )
        if changed:
            manifest["evidence_synced_at_utc"] = (
                datetime.now(timezone.utc)
                .replace(microsecond=0)
                .isoformat()
                .replace("+00:00", "Z")
            )
        write_json(temp_dir / "archive_manifest.json", manifest)

        archive_dir.parent.mkdir(parents=True, exist_ok=True)
        temp_dir.replace(archive_dir)
    except Exception:
        if temp_dir.exists():
            shutil.rmtree(temp_dir)
        raise

    if copied_bytes:
        print(
            f"Candidate archiválva: {task_id} "
            f"(evidence: {human_mib(copied_bytes)})"
        )
    else:
        print(f"Candidate archiválva: {task_id}")
    return "ARCHIVED"


def archive_candidates(repo: Path) -> None:
    workspace_root = repo / WORKSPACE_ROOT
    if not workspace_root.is_dir():
        print(
            "Candidate workspace könyvtár nincs; nincs mit archiválni."
        )
        return

    (repo / ARCHIVE_ROOT).mkdir(parents=True, exist_ok=True)
    counts = {
        "ARCHIVED": 0,
        "EVIDENCE_UPDATED": 0,
        "ALREADY_ARCHIVED": 0,
        "SKIP_ACTIVE": 0,
        "SKIP_FAILED": 0,
    }

    for task_dir in sorted(workspace_root.iterdir()):
        if not task_dir.is_dir() or task_dir.is_symlink():
            continue
        status = archive_candidate(repo, task_dir)
        counts[status] += 1

    print(
        "Candidate archive: "
        f"új={counts['ARCHIVED']}, "
        f"evidence-frissített={counts['EVIDENCE_UPDATED']}, "
        f"már fent={counts['ALREADY_ARCHIVED']}, "
        f"aktív/lezáratlan={counts['SKIP_ACTIVE']}, "
        f"hibás audit={counts['SKIP_FAILED']}"
    )


def ensure_git_lfs(repo: Path) -> None:
    if shutil.which("git-lfs") is None:
        # `git lfs` may still exist even when the binary name lookup differs.
        probe = git_quiet(["git", "lfs", "version"], repo)
        if probe.returncode != 0:
            raise RuntimeError(
                "Git LFS nincs telepítve. Telepítsd egyszer:\n"
                "  sudo apt update\n"
                "  sudo apt install git-lfs\n"
                "majd futtasd újra a gitre.py-t."
            )
    run(["git", "lfs", "version"], repo)
    run(["git", "lfs", "install", "--local"], repo)


def quote_attribute_pattern(path: str) -> str:
    if "\n" in path or "\r" in path:
        raise RuntimeError(
            f"Érvénytelen .gitattributes útvonal: {path!r}"
        )
    # Git attributes accepts a C-style quoted pattern.
    if any(ch.isspace() for ch in path) or any(
        ch in path for ch in ('"', "\\", "#")
    ):
        return json.dumps(path, ensure_ascii=False)
    return path


def git_dir_path(repo: Path) -> Path:
    raw = run(
        ["git", "rev-parse", "--git-dir"],
        repo,
        capture=True,
    )
    path = Path(raw)
    if not path.is_absolute():
        path = repo / path
    return path.resolve()


def ensure_local_info_attributes(repo: Path, paths: Iterable[str]) -> None:
    """Activate exact LFS rules locally via .git/info/attributes.

    This is deliberately stored under .git because the canonical repository
    root may be write-protected by the agent workflow.  The same rules are also
    persisted in scoped tracked .gitattributes files below.
    """

    unique = sorted(
        {str(p).replace(os.sep, "/") for p in paths if str(p)}
    )
    if not unique:
        return

    info_dir = git_dir_path(repo) / "info"
    info_dir.mkdir(parents=True, exist_ok=True)
    attributes = info_dir / "attributes"

    try:
        existing = (
            attributes.read_text(encoding="utf-8")
            if attributes.is_file()
            else ""
        )
    except OSError as exc:
        raise RuntimeError(
            f"Nem olvasható .git/info/attributes: {exc}"
        ) from exc

    lines = existing.splitlines()
    normalized = {line.strip() for line in lines if line.strip()}
    changed = False

    for relative in unique:
        pattern = quote_attribute_pattern("/" + relative.lstrip("/"))
        rule = f"{pattern} filter=lfs diff=lfs merge=lfs -text"
        if rule not in normalized:
            lines.append(rule)
            normalized.add(rule)
            changed = True

    if changed or not attributes.exists():
        content = "\n".join(lines).rstrip("\n") + "\n"
        temp = info_dir / "attributes.gitre.tmp"
        try:
            temp.write_text(content, encoding="utf-8")
            temp.replace(attributes)
        except OSError as exc:
            try:
                if temp.exists():
                    temp.unlink()
            except OSError:
                pass
            raise RuntimeError(
                f"Nem írható .git/info/attributes: {exc}"
            ) from exc


def scoped_attribute_target(relative: str) -> tuple[Path | None, str]:
    """Return tracked .gitattributes path and rule relative to its directory.

    R2B4's canonical repository root can be intentionally non-writable, while
    runtime/ and logs/ are writable producers of durable evidence.  Therefore
    LFS attributes are persisted under the first path component instead of the
    repository root whenever possible.
    """

    path = Path(relative)
    parts = path.parts
    if len(parts) < 2:
        # Root-level large files are very unusual in this repo.  Local LFS
        # filtering still works via .git/info/attributes; no protected-root
        # write is attempted.
        return None, relative
    attr_path = Path(parts[0]) / ".gitattributes"
    pattern = Path(*parts[1:]).as_posix()
    return attr_path, pattern


def ensure_lfs_rules(repo: Path, paths: Iterable[str]) -> int:
    """Persist LFS rules without writing the protected repository root."""

    unique = sorted(
        {str(p).replace(os.sep, "/") for p in paths if str(p)}
    )
    if not unique:
        return 0

    # Highest-precedence local attributes guarantee that the clean filter is
    # active immediately, independent of .gitignore and before restaging.
    ensure_local_info_attributes(repo, unique)

    grouped: dict[Path, list[str]] = {}
    root_only: list[str] = []
    for relative in unique:
        attr_path, pattern = scoped_attribute_target(relative)
        if attr_path is None:
            root_only.append(relative)
            continue
        grouped.setdefault(attr_path, []).append(pattern)

    added = 0
    for attr_relative, patterns in sorted(
        grouped.items(),
        key=lambda item: item[0].as_posix(),
    ):
        attributes = repo / attr_relative
        parent = attributes.parent
        if not parent.is_dir():
            raise RuntimeError(
                f"Hiányzik az LFS attribute szülőkönyvtára: {parent}"
            )
        if attributes.exists() and (
            attributes.is_symlink() or not attributes.is_file()
        ):
            raise RuntimeError(
                f"{attributes} nem regular file; nem írható biztonságosan."
            )

        try:
            existing_text = (
                attributes.read_text(encoding="utf-8")
                if attributes.is_file()
                else ""
            )
        except OSError as exc:
            raise RuntimeError(
                f"Nem olvasható {attr_relative}: {exc}"
            ) from exc

        lines = existing_text.splitlines()
        normalized = {line.strip() for line in lines if line.strip()}
        file_changed = False

        for pattern_text in sorted(set(patterns)):
            pattern = quote_attribute_pattern(pattern_text)
            rule = f"{pattern} filter=lfs diff=lfs merge=lfs -text"
            if rule not in normalized:
                lines.append(rule)
                normalized.add(rule)
                added += 1
                file_changed = True
                print(
                    f"LFS attribute hozzáadva: "
                    f"{attr_relative.as_posix()} :: {pattern_text}"
                )

        if file_changed or not attributes.exists():
            content = "\n".join(lines).rstrip("\n") + "\n"
            temp = parent / ".gitattributes.gitre.tmp"
            try:
                temp.write_text(content, encoding="utf-8")
                temp.replace(attributes)
            except OSError as exc:
                try:
                    if temp.exists():
                        temp.unlink()
                except OSError:
                    pass
                raise RuntimeError(
                    f"Nem hozható létre/frissíthető "
                    f"{attr_relative.as_posix()}: {exc}"
                ) from exc

        # -f is intentional because logs/ is globally ignored.
        run(
            ["git", "add", "-f", "--", attr_relative.as_posix()],
            repo,
        )

    if root_only:
        print(
            "MEGJEGYZÉS: root-szintű nagy fájl(ok) LFS filtere "
            ".git/info/attributes-ban aktív, mert a repo gyökere védett:"
        )
        for relative in root_only:
            print(f"  {relative}")

    # Prove that Git actually sees LFS for every target.
    for relative in unique:
        if git_attr_filter(repo, relative) != "lfs":
            raise RuntimeError(
                f"A Git nem látja az LFS filtert ehhez: {relative}"
            )

    return added


def path_is_tracked_at_head(repo: Path, relative: str) -> bool:
    result = git_quiet(
        ["git", "cat-file", "-e", f"HEAD:{relative}"],
        repo,
    )
    return result.returncode == 0


def unstage_new_transients(repo: Path) -> None:
    staged = staged_paths(repo)
    for relative in staged:
        if path_is_tracked_at_head(repo, relative):
            continue

        path = Path(relative)
        transient = any(
            relative.startswith(prefix)
            for prefix in TRANSIENT_NEW_PATH_PREFIXES
        ) or any(
            path.name.startswith(prefix)
            for prefix in TRANSIENT_NEW_BASENAME_PREFIXES
        )

        if transient:
            run(
                ["git", "reset", "-q", "HEAD", "--", relative],
                repo,
                check=False,
            )
            print(f"Tranziens fájl kihagyva: {relative}")


def stage_important_logs(repo: Path) -> int:
    logs = repo / "logs"
    if not logs.is_dir():
        print("Live log könyvtár nincs.")
        return 0

    roots: list[Path] = []
    for child in sorted(logs.iterdir()):
        if child.is_symlink():
            if child.name == "latest":
                continue
            continue
        if child.name.startswith("session_") or child.name == "archive":
            roots.append(child)

    latest = logs / "latest"
    if latest.is_dir() and not latest.is_symlink():
        roots.append(latest)

    for root in roots:
        relative = root.relative_to(repo).as_posix()
        run(["git", "add", "-f", "--", relative], repo)

    print(
        f"Live/test log gyökerek staged: {len(roots)} "
        "(session_* + archive; latest symlink nem authority)"
    )
    return len(roots)


def stage_runtime_evidence(repo: Path) -> int:
    runtime = repo / "runtime"
    if not runtime.is_dir():
        return 0

    paths: list[Path] = []
    for child in sorted(runtime.iterdir()):
        if child.is_symlink():
            continue
        if child.is_file() and is_runtime_evidence_file(child):
            paths.append(child)
        elif child.is_dir() and any(
            marker in child.name.lower()
            for marker in RUNTIME_EVIDENCE_DIR_MARKERS
        ):
            if child.name not in {
                "agent_workspaces",
                "candidate_archive",
                "agent_coordination",
                "agent_runtime",
            }:
                paths.append(child)

    for path in paths:
        run(
            ["git", "add", "-f", "--", path.relative_to(repo).as_posix()],
            repo,
        )

    # Candidate archive may contain .log files, which the repository-wide
    # *.log ignore rule would otherwise exclude.
    candidate_archive = repo / ARCHIVE_ROOT
    if candidate_archive.is_dir():
        run(
            [
                "git",
                "add",
                "-f",
                "--",
                candidate_archive.relative_to(repo).as_posix(),
            ],
            repo,
        )

    print(
        f"Runtime evidence staged: {len(paths)} közvetlen artefaktum/gyökér "
        "+ candidate archive"
    )
    return len(paths)


def staged_paths(repo: Path) -> list[str]:
    result = subprocess.run(
        [
            "git",
            "diff",
            "--cached",
            "--name-only",
            "-z",
            "--diff-filter=ACMR",
        ],
        cwd=repo,
        capture_output=True,
    )
    if result.returncode != 0:
        raise GitCommandError(
            "Nem sikerült lekérni a staged fájlokat."
        )
    return [
        item.decode("utf-8", "surrogateescape")
        for item in result.stdout.split(b"\0")
        if item
    ]


def git_attr_filter(repo: Path, relative: str) -> str:
    result = git_quiet(
        ["git", "check-attr", "filter", "--", relative],
        repo,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"Nem sikerült Git attribute-ot ellenőrizni: {relative}"
        )
    output = result.stdout.strip()
    # path: filter: lfs
    return output.rsplit(":", 1)[-1].strip() if output else ""


def should_lfs(
    repo: Path,
    relative: str,
    threshold_bytes: int,
) -> tuple[bool, int]:
    path = repo / relative
    if path.is_symlink() or not path.is_file():
        return False, 0
    size = path.stat().st_size

    # Captures are durable replay evidence and can grow rapidly; always put
    # them in LFS even when today's instance happens to be small.
    if path.name.lower().endswith("_capture.json"):
        return True, size
    if size >= threshold_bytes:
        return True, size

    # Existing scoped LFS rules are relevant mainly to durable artifacts.
    # Avoid a noisy/expensive check for every ordinary source file.
    if relative.startswith(("runtime/", "logs/")):
        if git_attr_filter(repo, relative) == "lfs":
            return True, size
    return False, size


def index_blob_size(repo: Path, relative: str) -> int:
    output = run(
        ["git", "cat-file", "-s", f":{relative}"],
        repo,
        capture=True,
    )
    try:
        return int(output)
    except ValueError as exc:
        raise RuntimeError(
            f"Nem értelmezhető staged blob méret: {relative}: {output}"
        ) from exc


def verify_lfs_pointer(
    repo: Path,
    relative: str,
    working_size: int,
) -> None:
    blob_size = index_blob_size(repo, relative)
    if blob_size > LFS_POINTER_MAX_BYTES:
        raise RuntimeError(
            f"LFS pointer helyett nagy blob maradt staged állapotban: "
            f"{relative}: index={human_mib(blob_size)}, "
            f"working={human_mib(working_size)}"
        )

    result = subprocess.run(
        ["git", "show", f":{relative}"],
        cwd=repo,
        capture_output=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"Nem olvasható vissza a staged LFS pointer: {relative}"
        )
    payload = result.stdout
    if not payload.startswith(LFS_POINTER_HEADER):
        raise RuntimeError(
            f"A staged fájl nem szabványos Git LFS pointer: {relative}"
        )

    size_line = None
    for line in payload.splitlines():
        if line.startswith(b"size "):
            size_line = line
            break
    if size_line is None:
        raise RuntimeError(
            f"Az LFS pointerből hiányzik a size mező: {relative}"
        )
    try:
        pointer_size = int(size_line.split(b" ", 1)[1])
    except Exception as exc:
        raise RuntimeError(
            f"Hibás LFS pointer size mező: {relative}"
        ) from exc

    if pointer_size != working_size:
        raise RuntimeError(
            f"LFS pointer méreteltérés: {relative}: "
            f"pointer={pointer_size}, working={working_size}"
        )

    print(
        f"LFS pointer OK: {relative} "
        f"({human_mib(working_size)} -> {blob_size} byte pointer)"
    )


def convert_staged_large_files_to_lfs(
    repo: Path,
    threshold_bytes: int,
) -> list[str]:
    targets: list[tuple[str, int]] = []

    for relative in staged_paths(repo):
        use_lfs, size = should_lfs(
            repo, relative, threshold_bytes
        )
        if use_lfs:
            targets.append((relative, size))

    if not targets:
        print("LFS: nincs konvertálandó staged fájl.")
        return []

    # Add exact rules ourselves. Do not rely on `git lfs track`, because some
    # git-lfs builds have returned success while failing to create
    # .gitattributes.
    ensure_lfs_rules(repo, [relative for relative, _ in targets])

    for relative, _ in targets:
        if git_attr_filter(repo, relative) != "lfs":
            raise RuntimeError(
                f"A Git nem látja az LFS filtert ehhez: {relative}"
            )

    for relative, size in targets:
        # Force re-index through the LFS clean filter. -f is intentional:
        # canonical logs are ignored by .gitignore but explicitly durable here.
        run(
            ["git", "rm", "--cached", "-f", "--ignore-unmatch", "--", relative],
            repo,
        )
        run(["git", "add", "-f", "--", relative], repo)
        verify_lfs_pointer(repo, relative, size)

    print(
        f"LFS: {len(targets)} fájl pointerként staged "
        f"(küszöb: {threshold_bytes // (1024 * 1024)} MiB; "
        "*_capture.json mindig LFS)."
    )
    return [relative for relative, _ in targets]


def verify_staged_blob_policy(
    repo: Path,
    threshold_bytes: int,
) -> None:
    problems: list[tuple[str, int]] = []

    for relative in staged_paths(repo):
        size = index_blob_size(repo, relative)
        if size >= threshold_bytes:
            problems.append((relative, size))

    if problems:
        lines = "\n".join(
            f"  {path}: {human_mib(size)}"
            for path, size in problems
        )
        raise RuntimeError(
            "Nagy közvetlen Git blob maradt staged állapotban; "
            "LFS pointer szükséges:\n" + lines
        )


def rev_list_objects(repo: Path, revision_range: str) -> list[tuple[str, str]]:
    output = run(
        ["git", "rev-list", "--objects", revision_range],
        repo,
        capture=True,
    )
    rows: list[tuple[str, str]] = []
    for line in output.splitlines():
        parts = line.split(" ", 1)
        oid = parts[0].strip()
        path = parts[1] if len(parts) > 1 else ""
        if oid:
            rows.append((oid, path))
    return rows


def large_blobs_in_range(
    repo: Path,
    revision_range: str,
    threshold_bytes: int,
) -> list[tuple[str, int, str]]:
    rows = rev_list_objects(repo, revision_range)
    if not rows:
        return []

    oids = [oid for oid, _ in rows]
    path_by_oid: dict[str, str] = {}
    for oid, path in rows:
        if path and oid not in path_by_oid:
            path_by_oid[oid] = path

    batch = subprocess.run(
        [
            "git",
            "cat-file",
            "--batch-check=%(objectname) %(objecttype) %(objectsize)",
        ],
        cwd=repo,
        text=True,
        input="\n".join(oids) + "\n",
        capture_output=True,
    )
    if batch.returncode != 0:
        raise GitCommandError(
            "Nem sikerült ellenőrizni az unpushed Git objektumokat."
        )

    found: list[tuple[str, int, str]] = []
    for line in batch.stdout.splitlines():
        parts = line.split()
        if len(parts) != 3:
            continue
        oid, object_type, size_text = parts
        if object_type != "blob":
            continue
        try:
            size = int(size_text)
        except ValueError:
            continue
        if size >= threshold_bytes:
            found.append((path_by_oid.get(oid, oid), size, oid))

    return sorted(found, key=lambda row: (-row[1], row[0]))


def remote_divergence(repo: Path, remote_ref: str) -> tuple[int, int]:
    output = run(
        [
            "git",
            "rev-list",
            "--left-right",
            "--count",
            f"{remote_ref}...HEAD",
        ],
        repo,
        capture=True,
    )
    parts = output.split()
    if len(parts) != 2:
        raise RuntimeError(
            f"Nem értelmezhető rev-list eredmény: {output}"
        )
    return int(parts[0]), int(parts[1])


def unique_backup_branch(repo: Path) -> str:
    base = f"gitre_lfs_backup_{utc_tag()}"
    candidate = base
    counter = 1
    while (
        git_quiet(
            ["git", "show-ref", "--verify", "--quiet", f"refs/heads/{candidate}"],
            repo,
        ).returncode
        == 0
    ):
        counter += 1
        candidate = f"{base}_{counter}"
    return candidate


def repair_unpushed_large_history(
    repo: Path,
    branch: str,
    threshold_bytes: int,
) -> bool:
    remote_ref = f"origin/{branch}"
    run(["git", "fetch", "origin", branch], repo)

    if (
        git_quiet(
            ["git", "show-ref", "--verify", "--quiet", f"refs/remotes/{remote_ref}"],
            repo,
        ).returncode
        != 0
    ):
        raise RuntimeError(
            f"Hiányzik a remote branch: {remote_ref}"
        )

    remote_only, local_only = remote_divergence(repo, remote_ref)
    if remote_only > 0:
        raise RuntimeError(
            f"{remote_ref} {remote_only} commit-tal előrébb van. "
            "Automatikus history-javítás/push leállítva; előbb integrálni kell "
            "a remote változásokat."
        )

    if local_only == 0:
        return False

    large = large_blobs_in_range(
        repo,
        f"{remote_ref}..HEAD",
        threshold_bytes,
    )
    if not large:
        return False

    print(
        "\nNagy normál Git blob található a még fel nem pusholt commitokban:"
    )
    for path, size, _ in large:
        print(f"  {path}  {human_mib(size)}")

    backup = unique_backup_branch(repo)
    run(["git", "branch", backup, "HEAD"], repo)
    print(f"Biztonsági branch létrehozva: {backup}")

    # Safe because remote_only == 0. Soft reset preserves the complete current
    # index/worktree while removing only local-only commits from main.
    run(["git", "reset", "--soft", remote_ref], repo)
    print(
        "A hibás unpushed commit-történet leválasztva a main-ről; "
        "a teljes aktuális állapot staged/working tree-ben megmaradt."
    )
    return True


def verify_unpushed_history(
    repo: Path,
    branch: str,
    threshold_bytes: int,
) -> None:
    problems = large_blobs_in_range(
        repo,
        f"origin/{branch}..HEAD",
        threshold_bytes,
    )
    if not problems:
        return
    lines = "\n".join(
        f"  {path}: {human_mib(size)}"
        for path, size, _ in problems
    )
    raise RuntimeError(
        "A commit után is nagy normál Git blob maradt az unpushed historyban:\n"
        + lines
    )


def print_lfs_summary(repo: Path) -> None:
    result = git_quiet(["git", "lfs", "ls-files"], repo)
    if result.returncode != 0:
        raise RuntimeError(
            "Nem sikerült lekérni a Git LFS fájllistát."
        )
    lines = [line for line in result.stdout.splitlines() if line.strip()]
    print(f"\nGit LFS tracked fájlok: {len(lines)}")
    for line in lines[-20:]:
        print("  " + line)
    if len(lines) > 20:
        print(f"  ... +{len(lines) - 20} további")


def ensure_repo(repo: Path) -> str:
    if not repo.is_dir():
        raise RuntimeError(
            f"A repository könyvtár nem létezik: {repo}"
        )

    inside = run(
        ["git", "rev-parse", "--is-inside-work-tree"],
        repo,
        capture=True,
    )
    if inside != "true":
        raise RuntimeError(f"Nem Git working tree: {repo}")

    branch = run(
        ["git", "branch", "--show-current"],
        repo,
        capture=True,
    )
    if not branch:
        raise RuntimeError(
            "Detached HEAD állapot; automatikus push leállítva."
        )

    remotes = run(["git", "remote"], repo, capture=True).splitlines()
    if "origin" not in remotes:
        raise RuntimeError("Nincs 'origin' nevű Git remote.")

    return branch


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "R2B4 source + terminal candidates + live/test evidence/logs "
            "commitolása és GitHub-ra pusholása Git LFS támogatással."
        )
    )
    parser.add_argument(
        "-m",
        "--message",
        default="Update R2B4 system",
    )
    parser.add_argument(
        "--repo",
        default=str(DEFAULT_REPO),
    )
    parser.add_argument(
        "--lfs-threshold-mib",
        type=int,
        default=DEFAULT_LFS_THRESHOLD_MIB,
        help=(
            "E méret fölött minden staged regular file Git LFS-be kerül "
            f"(alapértelmezés: {DEFAULT_LFS_THRESHOLD_MIB} MiB)."
        ),
    )
    parser.add_argument(
        "--no-candidates",
        action="store_true",
        help="Ne frissítse a terminal candidate archívumokat.",
    )
    parser.add_argument(
        "--no-live-logs",
        action="store_true",
        help="Ne force-addolja a logs/session_* és logs/archive evidence-et.",
    )
    args = parser.parse_args()

    if args.lfs_threshold_mib <= 0:
        raise RuntimeError("--lfs-threshold-mib pozitív kell legyen.")

    threshold_bytes = args.lfs_threshold_mib * 1024 * 1024
    # Keep a hard upper guard below GitHub's absolute 100 MiB rejection.
    if threshold_bytes >= GITHUB_HARD_LIMIT_BYTES:
        raise RuntimeError(
            "--lfs-threshold-mib legyen 100 MiB alatt."
        )

    repo = Path(args.repo).expanduser().resolve()
    branch = ensure_repo(repo)

    print(f"\nRepository: {repo}")
    print(f"Branch:     {branch}")
    print(
        f"LFS policy: *_capture.json mindig; egyéb fájl >= "
        f"{args.lfs_threshold_mib} MiB\n"
    )

    ensure_git_lfs(repo)

    # Repair history first, before generating any new archive material.
    repair_unpushed_large_history(
        repo,
        branch,
        threshold_bytes,
    )

    if not args.no_candidates:
        archive_candidates(repo)
        print()

    # Normal repo changes/source/config/runtime artifacts.
    run(["git", "add", "-A"], repo)

    # Canonical logs are intentionally ignored by the repo's broad log ignore
    # rules, so durable run-identified evidence must be force-added.
    if not args.no_live_logs:
        stage_important_logs(repo)

    stage_runtime_evidence(repo)
    unstage_new_transients(repo)

    # Convert every large staged file and every capture to LFS. This also
    # catches candidate evidence/log archives and any future large artifact,
    # regardless of filename.
    convert_staged_large_files_to_lfs(
        repo,
        threshold_bytes,
    )

    # Scoped runtime/.gitattributes and logs/.gitattributes files are staged
    # by ensure_lfs_rules(); the protected repository root is never written.
    unstage_new_transients(repo)

    verify_staged_blob_policy(
        repo,
        threshold_bytes,
    )

    staged = subprocess.run(
        ["git", "diff", "--cached", "--quiet"],
        cwd=repo,
    ).returncode

    if staged == 1:
        print("\nStaged összegzés:")
        run(["git", "status", "--short"], repo)
        print()
        run(["git", "commit", "-m", args.message], repo)
    elif staged == 0:
        print("Nincs új commitolandó változás.")
    else:
        raise RuntimeError(
            "Nem sikerült ellenőrizni a staged változásokat."
        )

    # Final fail-closed proof before network push.
    verify_unpushed_history(
        repo,
        branch,
        threshold_bytes,
    )
    print_lfs_summary(repo)

    run(["git", "push", "origin", branch], repo)

    print(
        "\nKÉSZ: source/config + terminal candidate archive/evidence + "
        "run-identified live/test logs + runtime artifacts GitHub-ra feltöltve."
    )
    run(["git", "status", "--short", "--branch"], repo)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, GitCommandError) as exc:
        print(f"\nHIBA: {exc}", file=sys.stderr)
        raise SystemExit(1)
