#!/usr/bin/env bash
set -euo pipefail
ROOT="${1:-/home/alba/project_r2b4}"
HERE="$(cd "$(dirname "$0")" && pwd)"
python3 "$HERE/apply_upgrade.py" --check "$ROOT"
python3 "$HERE/apply_upgrade.py" "$ROOT"
