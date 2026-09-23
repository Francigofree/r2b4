#!/usr/bin/env python3
"""Restore one backup created by the R2B4 HRI P0 installer."""
from __future__ import annotations
import argparse, json, os, tempfile
from pathlib import Path


def atomic_write(path: Path, data: bytes, mode: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp = tempfile.mkstemp(prefix=f'.{path.name}.restore.', dir=path.parent)
    try:
        os.fchmod(fd, mode)
        with os.fdopen(fd, 'wb') as h:
            h.write(data); h.flush(); os.fsync(h.fileno())
        os.replace(temp, path)
    finally:
        try: os.unlink(temp)
        except FileNotFoundError: pass


def main() -> int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--root', default='/home/alba/project_r2b4')
    ap.add_argument('--backup', required=True)
    a=ap.parse_args()
    root=Path(a.root).resolve(); backup=Path(a.backup).resolve()
    manifest=json.loads((backup/'manifest.json').read_text(encoding='utf-8'))
    existed=manifest['existed']
    for rel, did_exist in existed.items():
        target=root/rel; saved=backup/rel
        if did_exist:
            if not saved.is_file(): raise RuntimeError(f'missing backup file: {saved}')
            atomic_write(target, saved.read_bytes(), saved.stat().st_mode & 0o777)
        else:
            try: target.unlink()
            except FileNotFoundError: pass
    print(f'RESTORE PASS: {backup}')
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
