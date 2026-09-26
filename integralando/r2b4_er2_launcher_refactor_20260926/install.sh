#!/usr/bin/env bash
set -euo pipefail

ROOT="${R2B4_ROOT:-/home/alba/project_r2b4}"
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP="$ROOT/.upgrade_backups/r_launcher_er2_stream_${STAMP}"

[[ -d "$ROOT/v3" && -f "$ROOT/pytest.ini" ]] || {
  echo "ERROR: invalid R2B4 root: $ROOT" >&2
  exit 2
}

mkdir -p "$BACKUP/v3" "$BACKUP/r2b4_er2" "$BACKUP/tests/feature"

cp -a "$ROOT/r" "$BACKUP/r"
cp -a "$ROOT/v3/launcher_cli.py" "$BACKUP/v3/launcher_cli.py"
cp -a "$ROOT/r2b4_er2/cli.py" "$BACKUP/r2b4_er2/cli.py"
if [[ -f "$ROOT/tests/feature/test_er2_launcher_stream_cli.py" ]]; then
  cp -a "$ROOT/tests/feature/test_er2_launcher_stream_cli.py" \
        "$BACKUP/tests/feature/test_er2_launcher_stream_cli.py"
fi

install -m 0755 "$PKG_DIR/files/r" "$ROOT/r"
install -m 0644 "$PKG_DIR/files/v3/launcher_cli.py" "$ROOT/v3/launcher_cli.py"
install -m 0644 "$PKG_DIR/files/r2b4_er2/cli.py" "$ROOT/r2b4_er2/cli.py"
install -m 0644 "$PKG_DIR/files/tests/feature/test_er2_launcher_stream_cli.py" \
                "$ROOT/tests/feature/test_er2_launcher_stream_cli.py"

cd "$ROOT"

bash -n ./r
python3 -m py_compile \
  v3/launcher_cli.py \
  r2b4_er2/cli.py \
  tests/feature/test_er2_launcher_stream_cli.py

python3 -m pytest -q tests/feature/test_er2_launcher_stream_cli.py

echo
echo "INSTALL_OK"
echo "Backup: $BACKUP"
echo
echo 'Primary live command:'
echo '  ./r er2 "fordulj pontosan 90 fokkal balra, majd állj meg."'
echo
echo 'Full option surface:'
echo '  ./r er2 "fordulj pontosan 90 fokkal balra, majd állj meg." --camera --tools --speak --json'
