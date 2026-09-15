#!/usr/bin/env python3

import subprocess
import sys
from pathlib import Path


# GitHub 100 MB-os fajlkorlatja alatt hagyunk egy kis biztonsagi tartalekot.
MAX_MCAP_BYTES = 95 * 1024 * 1024


def run(*args: str, check: bool = True) -> subprocess.CompletedProcess:
    print("+", " ".join(args))
    return subprocess.run(args, check=check)


def exclude_oversized_mcaps(root: str) -> None:
    staged = subprocess.run(
        [
            "git",
            "-C",
            root,
            "diff",
            "--cached",
            "--name-only",
            "--diff-filter=ACMR",
            "-z",
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout

    for relative in filter(None, staged.split("\0")):
        if not relative.lower().endswith(".mcap"):
            continue

        path = Path(root) / relative
        if not path.is_file():
            continue

        size = path.stat().st_size
        if size <= MAX_MCAP_BYTES:
            continue

        size_mib = size / (1024 * 1024)
        limit_mib = MAX_MCAP_BYTES / (1024 * 1024)
        print(
            f"MCAP kihagyva: {relative} "
            f"({size_mib:.2f} MiB > {limit_mib:.0f} MiB limit)"
        )

        # Csak a commitbol vesszuk ki, a fajl a lemezen megmarad.
        run("git", "-C", root, "restore", "--staged", "--", relative)


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

    # A GitHub meretkorlatjat tullepo MCAP-ok ne keruljenek commitba.
    exclude_oversized_mcaps(root)

    staged = subprocess.run(
        ["git", "-C", root, "diff", "--cached", "--quiet"]
    ).returncode

    if staged == 0:
        print("Nincs commitolando valtozas.")
    else:
        run("git", "-C", root, "commit", "-m", message)

    run("git", "-C", root, "push", "origin", "main")

    print("Push kesz.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
