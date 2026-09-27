#!/usr/bin/env bash
set -euo pipefail

BASE_COMMIT="a571d9a3d500e262235f05578dee6c4bf716f595"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="${1:-.}"
FILES=(
  "v3/lidar_estimator.py"
  "v3/adapters/live_lidar.py"
  "tests/feature/test_v3_live_lidar.py"
)

die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
cd "$REPO"
ROOT="$(git rev-parse --show-toplevel 2>/dev/null)" || die "not inside a git repository"
cd "$ROOT"
ACTUAL="$(git rev-parse HEAD)"
[[ "$ACTUAL" == "$BASE_COMMIT" ]] || die "HEAD mismatch: expected $BASE_COMMIT, got $ACTUAL"

for f in "${FILES[@]}"; do
  git diff --quiet -- "$f" || die "working-tree change already present in $f"
  git diff --cached --quiet -- "$f" || die "staged change already present in $f"
done

git apply --check "$SCRIPT_DIR/upgrade.patch"
git apply "$SCRIPT_DIR/upgrade.patch"

rollback() {
  status=$?
  if [[ $status -ne 0 ]]; then
    printf 'Validation failed; reverting upgrade patch.\n' >&2
    git apply -R --check "$SCRIPT_DIR/upgrade.patch" >/dev/null 2>&1 && git apply -R "$SCRIPT_DIR/upgrade.patch" || true
  fi
  exit $status
}
trap rollback ERR

python3 -m py_compile \
  v3/lidar_estimator.py \
  v3/adapters/live_lidar.py \
  tests/feature/test_v3_live_lidar.py

if python3 -c 'import pytest' >/dev/null 2>&1; then
  python3 -m pytest -q tests/feature/test_v3_live_lidar.py
else
  printf 'pytest not installed; syntax validation passed, unit test skipped.\n'
fi

trap - ERR
printf 'Upgrade applied successfully at base %s.\n' "$BASE_COMMIT"
printf 'Changed files:\n'
printf '  %s\n' "${FILES[@]}"
