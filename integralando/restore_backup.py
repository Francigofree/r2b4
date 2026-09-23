#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, os, tempfile
from pathlib import Path

def atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(data); handle.flush(); os.fsync(handle.fileno())
        os.replace(tmp, path)
    finally:
        try: os.unlink(tmp)
        except FileNotFoundError: pass

def main() -> int:
    p=argparse.ArgumentParser(); p.add_argument('--root',required=True); p.add_argument('--backup',required=True); a=p.parse_args()
    root=Path(a.root).resolve(); backup=Path(a.backup).resolve()
    m=json.loads((backup/'manifest.json').read_text(encoding='utf-8'))
    for rel in m['original_files']: atomic_write(root/rel,(backup/'original'/rel).read_bytes())
    for rel in m['new_files']:
        try:(root/rel).unlink()
        except FileNotFoundError:pass
    print(f"RESTORE PASS: {backup}")
    return 0
if __name__=='__main__': raise SystemExit(main())
