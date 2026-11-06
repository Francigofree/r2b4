"""Private tmpfs locations for the live command and status file protocols."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import stat


_TMPFS_ROOT = Path('/dev/shm')
_MAILBOX_NAMES = frozenset({'v3_command.json', 'v3_status.json'})


def _directory(project_root: Path | str) -> Path:
    identity = hashlib.sha256(os.fsencode(str(Path(project_root).resolve()))).hexdigest()[:16]
    return _TMPFS_ROOT / f'r2b4-live-{os.geteuid()}-{identity}'


def live_io_directory(project_root: Path | str, *, create: bool = True) -> Path:
    """Return the same trusted RAM directory to every client of this project."""
    path = _directory(project_root)
    # Fail closed rather than silently put a live mailbox back on disk.
    mounts = Path('/proc/self/mountinfo').read_text().splitlines()
    if not any(line.split(' - ', 1)[0].split()[4] == str(_TMPFS_ROOT)
               and line.split(' - ', 1)[1].split()[0] == 'tmpfs'
               for line in mounts if ' - ' in line):
        raise ValueError('live mailbox root must be a tmpfs mount')
    parent_fd = os.open(_TMPFS_ROOT, os.O_RDONLY | os.O_DIRECTORY | getattr(os, 'O_NOFOLLOW', 0))
    try:
        if create:
            try:
                os.mkdir(path.name, 0o700, dir_fd=parent_fd)
            except FileExistsError:
                pass
        try:
            descriptor = os.open(path.name, os.O_RDONLY | os.O_DIRECTORY | getattr(os, 'O_NOFOLLOW', 0),
                                 dir_fd=parent_fd)
        except FileNotFoundError:
            if not create:
                return path
            raise
        except OSError as exc:
            raise ValueError('live mailbox directory must not be a symlink') from exc
        try:
            metadata = os.fstat(descriptor)
            if metadata.st_uid != os.geteuid() or stat.S_IMODE(metadata.st_mode) != 0o700:
                raise ValueError('live mailbox directory must be owned by this user with mode 0700')
        finally:
            os.close(descriptor)
    finally:
        os.close(parent_fd)
    return path


def runtime_mailbox_path(value: str | Path | None, project_root: Path | str, *, filename: str) -> Path:
    """Permit existing runtime paths or the exact private RAM mailbox name."""
    if filename not in _MAILBOX_NAMES:
        raise ValueError('unknown live mailbox filename')
    root = Path(project_root).resolve()
    path = live_io_directory(root) / filename if value is None else Path(value)
    if not path.is_absolute():
        path = root / path
    canonical = _directory(root) / filename
    if path == canonical:
        live_io_directory(root)
        try:
            metadata = path.lstat()
        except FileNotFoundError:
            return path
        if (not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != os.geteuid()
                or stat.S_IMODE(metadata.st_mode) != 0o600):
            raise ValueError('live mailbox must be a regular user-owned file with mode 0600')
        return path
    # Resolving an alias must never broaden the tmpfs allowlist. The historical
    # runtime tree remains available to explicit offline/unit-test callers.
    runtime = root / 'runtime'
    if runtime.is_symlink():
        raise ValueError('canonical runtime directory must not be a symlink')
    path = path.resolve(strict=False)
    try:
        path.relative_to(runtime.resolve())
    except ValueError as exc:
        raise ValueError('mailbox paths must stay below runtime or use the canonical private RAM filename') from exc
    return path
