#!/bin/bash
# Complete flow: obstacle prescan -> QR forwarding -> object detection.
WS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd) || exit 2
source "$WS/scripts/start_common.sh" || exit 2
appli_start all "$@"
