#!/usr/bin/env bash
set -euo pipefail

ROOT="${1:-/home/alba/project_r2b4}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH="$HERE/r2b4_follow_person_v2_non_l6.patch"
L6_TOOL="$HERE/apply_l6_follow_person.py"
BASE="e8f8805e436de397dc088445e8d356f1c77eebf9"

cd "$ROOT"
CURRENT="$(git rev-parse HEAD)"
if [[ "$CURRENT" != "$BASE" ]]; then
  echo "ERROR: this package targets HEAD $BASE"
  echo "Current HEAD: $CURRENT"
  echo "No files changed."
  exit 2
fi

TARGETS=(
  v3/contracts/messages.py
  v3/layers/l5_command_mission.py
  v3/adapters/resident_command.py
  v3/layers/l6_navigation.py
  v3/composition/native_control.py
  v3/control_cli.py
  v3/operator_controller.py
  v3/operator_cli.py
  conf/vezerles.json
  tests/test_v3_follow_person.py
)

if ! git diff --quiet -- "${TARGETS[@]}"; then
  echo "ERROR: unstaged changes exist in FOLLOW_PERSON target files. No files changed."
  git status --short -- "${TARGETS[@]}"
  exit 3
fi
if ! git diff --cached --quiet -- "${TARGETS[@]}"; then
  echo "ERROR: staged changes exist in FOLLOW_PERSON target files. No files changed."
  git status --short -- "${TARGETS[@]}"
  exit 4
fi

# Full preflight before any repository modification.
git apply --check "$PATCH"
python3 "$L6_TOOL" v3/layers/l6_navigation.py --check

git apply "$PATCH"
python3 "$L6_TOOL" v3/layers/l6_navigation.py

python3 -m py_compile \
  v3/contracts/messages.py \
  v3/layers/l5_command_mission.py \
  v3/adapters/resident_command.py \
  v3/layers/l6_navigation.py \
  v3/composition/native_control.py \
  v3/control_cli.py \
  v3/operator_controller.py \
  v3/operator_cli.py \
  tests/test_v3_follow_person.py

echo "FOLLOW_PERSON slice V2 applied."
echo
echo "Run targeted regression tests:"
echo "  python3 -m pytest -q tests/test_v3_follow_person.py tests/test_v3_face_person.py tests/test_v3_control_cli.py tests/test_v3_operator_controller.py tests/test_v3_resident_command.py tests/test_v3_l5_l9_mission_navigation.py tests/test_v3_navigation_trajectory.py tests/test_v3_async_l6_planner.py"
echo
echo "Then run the full suite:"
echo "  python3 -m pytest -q"
echo
echo "Only after tests are green:"
echo "  ./r2b4 followperson c full"
