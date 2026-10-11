#!/bin/bash
set -e
LOCK=/tmp/rdk_cjlu_v11.lock
exec 9>"$LOCK"
if ! flock -n 9; then
  echo "RDK_CJLU_26_V11 already running"
  exit 0
fi
cd /root/Monthwu/RDK_vision/RDK_CJLU_26_V11
source /opt/tros/humble/setup.bash
source /root/Monthwu/RDK_vision/RDK_CJLU_26_V11/install/setup.bash
exec ros2 launch /root/Monthwu/RDK_vision/RDK_CJLU_26_V11/launch/system.launch.py