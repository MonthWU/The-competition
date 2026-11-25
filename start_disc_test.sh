#!/usr/bin/env bash
# Purpose: start the large white-disc center preview using the object camera only.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="${project_root}/src/car26_system${PYTHONPATH:+:${PYTHONPATH}}"
exec python3 -B -u "$project_root/scripts/disc_preview.py" "$@"
