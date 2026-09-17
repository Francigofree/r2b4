#!/usr/bin/env bash
set -euo pipefail

REPO="${1:-/home/alba/project_r2b4}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -d "$REPO/.git" ]]; then
  echo "ERROR: not an R2B4 git repository: $REPO" >&2
  exit 2
fi

cd "$REPO"
python3 "$HERE/apply_follow_person_p0.py" "$REPO" --check
python3 "$HERE/apply_follow_person_p0.py" "$REPO"

echo
echo "FOLLOW_PERSON P0 source update complete."
echo "Run the targeted tests before any live robot test:"
echo "  python3 -m pytest -q tests/test_v3_follow_person.py tests/test_v3_follow_person_p0.py tests/test_v3_face_person.py tests/test_v3_async_l6_planner.py tests/test_v3_navigation_trajectory.py"
