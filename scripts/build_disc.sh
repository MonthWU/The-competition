#!/usr/bin/env bash
# Purpose: build only the native white-disc detector without opening any device.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$project_root/native/lib"
g++ -std=c++17 -O3 -shared -fPIC "$project_root/native/disc_center.cpp" \
    -o "$project_root/native/lib/libcar26_disc.so" $(pkg-config --cflags --libs opencv4)
