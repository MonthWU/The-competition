#!/usr/bin/env bash
# Purpose: build the native logit candidate filter without starting any device.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$project_root/native/lib"
g++ -std=c++17 -O3 -shared -fPIC "$project_root/native/yolo_candidates.cpp" \
  -o "$project_root/native/lib/libcar26_candidates.so"
