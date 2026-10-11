#!/bin/bash
set -e
cd /root/Monthwu/RDK_vision/RDK_25_E_008_fix_20260507_191942
source /opt/tros/humble/setup.bash
source /root/Monthwu/RDK_vision/RDK_25_E_008_fix_20260507_191942/install/setup.bash
exec ros2 launch launch/system.launch.py
