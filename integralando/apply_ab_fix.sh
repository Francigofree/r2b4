#!/usr/bin/env bash
set -euo pipefail
ROOT="${1:-/home/alba/project_r2b4}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SCRIPT_DIR/apply_ab_fix.py" "$ROOT"
