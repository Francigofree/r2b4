#!/usr/bin/env python3

import subprocess
import sys
from pathlib import Path


def run(*args: str, check: bool = True) -> subprocess.CompletedProcess:
    print("+", " ".join(args))
    return subprocess.run(args, check=check)


def main() -> int:
    message = " ".join(sys.argv[1:]).strip() or "Update R2B4 system"

    root = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()

    repo = Path(root)
    print(f"repo: {repo}")

    run("git", "-C", root, "add", "-A")

    staged = subprocess.run(
        ["git", "-C", root, "diff", "--cached", "--quiet"]
    ).returncode

    if staged == 0:
        print("Nincs commitolandó változás.")
    else:
        run("git", "-C", root, "commit", "-m", message)

    run("git", "-C", root, "push", "origin", "main")

    print("Push kész.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())