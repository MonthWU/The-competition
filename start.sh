#!/usr/bin/env bash
# Purpose: select the fixed or dynamic navigation project, defaulting to no radar.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
variant=without_radar
case "${1:-}" in
  without-radar) shift ;;
  with-radar) variant=with_radar; shift ;;
esac
exec bash "$project_root/projects/$variant/start.sh" "$@"
