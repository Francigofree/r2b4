#!/usr/bin/env bash
# R2B4 single human/agent launcher bootstrap.
set -euo pipefail
SELF="$(readlink -f "${BASH_SOURCE[0]}")"
SELF_DIR="$(cd "$(dirname "$SELF")" && pwd)"
DEFAULT_ROOT="/home/alba/project_r2b4"
if [[ -n "${R2B4_ROOT:-}" ]]; then
    ROOT="$(readlink -f "$R2B4_ROOT")"
elif [[ -d "$SELF_DIR/v3" && -f "$SELF_DIR/pytest.ini" ]]; then
    ROOT="$SELF_DIR"
elif [[ -d "$DEFAULT_ROOT/v3" && -f "$DEFAULT_ROOT/pytest.ini" ]]; then
    ROOT="$DEFAULT_ROOT"
else
    printf 'ERROR: R2B4 repo not found. Set R2B4_ROOT=/path/to/project_r2b4\n' >&2
    exit 2
fi
export R2B4_ROOT="$ROOT"
cd "$ROOT"

# Launcher-only ER2 shorthand/default normalization.
#   r er2 "PROMPT"   -> r er2 preview "PROMPT" --camera --tools
#   r er2 s "TASK"   -> r er2 stream "TASK"
# Existing explicit ER2 commands remain unchanged.
if [[ "${1:-}" == "er2" && $# -ge 2 ]]; then
    case "$2" in
        s)
            shift 2
            set -- er2 stream "$@"
            ;;
        status|preview|stream|-h|--help)
            ;;
        *)
            if [[ $# -eq 2 ]]; then
                set -- er2 preview "$2" --camera --tools
            else
                PROMPT="$2"
                shift 2
                set -- er2 preview "$PROMPT" "$@"
            fi
            ;;
    esac
fi

exec python3 -m v3.launcher_cli "$@"
