#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 /home/alba/project_r2b4/.upgrade_backups/r_launcher_er2_stream_YYYYMMDD_HHMMSS" >&2
  exit 2
fi

ROOT="${R2B4_ROOT:-/home/alba/project_r2b4}"
BACKUP="$1"

[[ -f "$BACKUP/r" && -f "$BACKUP/v3/launcher_cli.py" && -f "$BACKUP/r2b4_er2/cli.py" ]] || {
  echo "ERROR: invalid backup directory: $BACKUP" >&2
  exit 2
}

install -m 0755 "$BACKUP/r" "$ROOT/r"
install -m 0644 "$BACKUP/v3/launcher_cli.py" "$ROOT/v3/launcher_cli.py"
install -m 0644 "$BACKUP/r2b4_er2/cli.py" "$ROOT/r2b4_er2/cli.py"

if [[ -f "$BACKUP/tests/feature/test_er2_launcher_stream_cli.py" ]]; then
  install -m 0644 "$BACKUP/tests/feature/test_er2_launcher_stream_cli.py" \
                  "$ROOT/tests/feature/test_er2_launcher_stream_cli.py"
else
  rm -f "$ROOT/tests/feature/test_er2_launcher_stream_cli.py"
fi

cd "$ROOT"
bash -n ./r
python3 -m py_compile v3/launcher_cli.py r2b4_er2/cli.py

echo "ROLLBACK_OK"
