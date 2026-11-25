#!/usr/bin/env bash
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/tros/humble/setup.bash
cd "$project_root"
colcon build --packages-select car26_system
