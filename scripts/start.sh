#!/usr/bin/env bash
# Purpose: select work/debug and light_able/light_unable parameters, then run or check the shared application.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
config="${1:-${project_root}/config/system.json}"
if (( $# > 0 )); then shift; fi
mode=work
mode_selected=false
lighting_selected=false
lighting_args=()
check_args=()
while (( $# > 0 )); do
  case "$1" in
    work|debug)
      if "$mode_selected"; then echo 'INVALID_MODE: specify work/debug once' >&2; exit 2; fi
      mode="$1"; mode_selected=true ;;
    light_able|light_unable)
      if "$lighting_selected"; then echo 'INVALID_LIGHTING_MODE: specify lighting once' >&2; exit 2; fi
      lighting_args=(--lighting "$1"); lighting_selected=true ;;
    --lighting)
      if "$lighting_selected" || (( $# < 2 )); then echo 'INVALID_LIGHTING_MODE: missing or duplicate lighting' >&2; exit 2; fi
      shift
      case "$1" in
        light_able|light_unable) lighting_args=(--lighting "$1"); lighting_selected=true ;;
        *) echo 'INVALID_LIGHTING_MODE: expected light_able or light_unable' >&2; exit 2 ;;
      esac ;;
    --check|--check-files|--help|-h) check_args+=("$1") ;;
    *) echo 'INVALID_MODE: expected work/debug, light_able/light_unable or check options' >&2; exit 2 ;;
  esac
  shift
done
export PYTHONPATH="${project_root}/src/car26_system${PYTHONPATH:+:${PYTHONPATH}}"
exec python3 -m car26 --config "$config" --mode "$mode" "${lighting_args[@]}" "${check_args[@]}"
