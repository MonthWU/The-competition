#!/usr/bin/env bash
# Purpose: compile only the native OpenCV circle detector; never open cameras or UARTs.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$project_root/native/lib"
g++ -std=c++17 -O3 -shared -fPIC "$project_root/native/target_focus.cpp" \
  -o "$project_root/native/lib/libcar26_focus.so" $(pkg-config --cflags --libs opencv4)
