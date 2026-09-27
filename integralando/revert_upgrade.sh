#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="${1:-.}"
cd "$REPO"
ROOT="$(git rev-parse --show-toplevel 2>/dev/null)" || { echo 'ERROR: not inside a git repository' >&2; exit 1; }
cd "$ROOT"
git apply -R --check "$SCRIPT_DIR/upgrade.patch"
git apply -R "$SCRIPT_DIR/upgrade.patch"
printf 'Upgrade reverted.\n'
