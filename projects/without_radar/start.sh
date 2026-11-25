#!/usr/bin/env bash
# Purpose: start the without_radar project using its own configuration and the shared application.
set -euo pipefail
variant_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$variant_root/../../scripts/start.sh" "$variant_root/config/board.json" "$@"
