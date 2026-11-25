#!/usr/bin/env bash
# Purpose: start only object recognition and targetTwo circle alignment in the four-module application.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$project_root/scripts/start.sh" "$project_root/config/board-object.json" "$@"
