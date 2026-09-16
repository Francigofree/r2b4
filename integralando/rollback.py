#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", nargs="?", default="/home/alba/project_r2b4")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    repo = Path(args.repo).resolve()
    package = Path(__file__).resolve().parent
    backup = package / "backup"
    manifest_path = backup / "manifest.json"
    if not manifest_path.exists():
        raise SystemExit("ERROR: backup/manifest.json not found.")

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    for rel, info in manifest["files"].items():
        target = repo / rel
        if target.exists() and not args.force:
            if sha256(target) != info["after_sha256"]:
                raise SystemExit(
                    f"ERROR: {rel} changed after installation; "
                    "refusing rollback. Inspect it or use --force."
                )

    for rel, info in manifest["files"].items():
        src = backup / rel
        dst = repo / rel
        shutil.copy2(src, dst)
        if sha256(dst) != info["before_sha256"]:
            raise SystemExit(f"ERROR: rollback verification failed for {rel}")
        print(f"restored: {rel}")

    print("PASS: rollback complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
