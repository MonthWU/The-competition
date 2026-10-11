#!/bin/bash
set -e
cd /root/Monthwu/RDK_vision/RDK_25_E_009
source /opt/tros/humble/setup.bash
source /root/Monthwu/RDK_vision/RDK_25_E_009/install/setup.bash
exec ros2 launch launch/system.launch.py
