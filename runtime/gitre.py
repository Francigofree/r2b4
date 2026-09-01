#!/usr/bin/env python3

import argparse
import subprocess
import sys
from pathlib import Path

DEFAULT_REPO = Path("/home/alba/project_r2b4")


def run(cmd, cwd, capture=False):
    print("+", " ".join(cmd))
    result = subprocess.run(
        cmd,
        cwd=cwd,
        text=True,
        capture_output=capture,
    )
    if result.returncode != 0:
        if capture:
            if result.stdout:
                print(result.stdout, end="")
            if result.stderr:
                print(result.stderr, end="", file=sys.stderr)
        sys.exit(result.returncode)
    return result.stdout.strip() if capture else ""


def main():
    parser = argparse.ArgumentParser(
        description="Az aktuális R2B4 repository változásainak commitolása és GitHub-ra pusholása."
    )
    parser.add_argument(
        "-m",
        "--message",
        default="Update R2B4 system",
        help='Commit üzenet. Alapértelmezés: "Update R2B4 system"',
    )
    parser.add_argument(
        "--repo",
        default=str(DEFAULT_REPO),
        help=f"Git repository útvonala. Alapértelmezés: {DEFAULT_REPO}",
    )
    args = parser.parse_args()

    repo = Path(args.repo).expanduser().resolve()

    if not repo.is_dir():
        print(f"HIBA: A repository könyvtár nem létezik: {repo}", file=sys.stderr)
        sys.exit(1)

    inside = run(
        ["git", "rev-parse", "--is-inside-work-tree"],
        repo,
        capture=True,
    )
    if inside != "true":
        print(f"HIBA: Nem Git working tree: {repo}", file=sys.stderr)
        sys.exit(1)

    branch = run(
        ["git", "branch", "--show-current"],
        repo,
        capture=True,
    )
    if not branch:
        print("HIBA: Detached HEAD állapot; automatikus push leállítva.", file=sys.stderr)
        sys.exit(1)

    remotes = run(["git", "remote"], repo, capture=True).splitlines()
    if "origin" not in remotes:
        print("HIBA: Nincs 'origin' nevű Git remote.", file=sys.stderr)
        sys.exit(1)

    print(f"\nRepository: {repo}")
    print(f"Branch:     {branch}\n")

    # Minden módosított, új és törölt fájl stagingbe kerül.
    run(["git", "add", "-A"], repo)

    # Csak akkor készít commitot, ha a staging area ténylegesen változott.
    staged = subprocess.run(
        ["git", "diff", "--cached", "--quiet"],
        cwd=repo,
    ).returncode

    if staged == 1:
        run(["git", "status", "--short"], repo)
        print()
        run(["git", "commit", "-m", args.message], repo)
    elif staged == 0:
        print("Nincs új commitolandó változás.")
    else:
        print("HIBA: Nem sikerült ellenőrizni a staged változásokat.", file=sys.stderr)
        sys.exit(staged)

    # Az aktuális branch feltöltése GitHub-ra.
    run(["git", "push", "origin", branch], repo)

    print("\nKÉSZ: a repository GitHub-ra feltöltve.")
    run(["git", "status", "--short", "--branch"], repo)


if __name__ == "__main__":
    main()
