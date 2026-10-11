#!/bin/bash
set -e
LOCK=/tmp/rdk_cjlu_v10.lock
exec 9>"$LOCK"
if ! flock -n 9; then
  echo "RDK_CJLU_26_V10 already running"
  exit 0
fi
cd /root/Monthwu/RDK_vision/RDK_CJLU_26_V10
source /opt/tros/humble/setup.bash
source /root/Monthwu/RDK_vision/RDK_CJLU_26_V10/install/setup.bash
exec ros2 launch /root/Monthwu/RDK_vision/RDK_CJLU_26_V10/launch/system.launch.py
