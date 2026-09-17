#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${1:-/home/alba/project_r2b4}"
PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

python3 "$PACKAGE_ROOT/apply_follow_person_motion_fix.py" "$PROJECT_ROOT" --check
python3 "$PACKAGE_ROOT/apply_follow_person_motion_fix.py" "$PROJECT_ROOT"

python3 -m py_compile \
  "$PROJECT_ROOT/v3/layers/l6_navigation.py" \
  "$PROJECT_ROOT/v3/composition/native_control.py" \
  "$PROJECT_ROOT/tests/test_v3_follow_person_p0.py" \
  "$PROJECT_ROOT/tests/test_v3_follow_person_motion_quality.py"

echo "FOLLOW_PERSON motion-quality install: PASS"
echo "No git commit/hash verification was used."
