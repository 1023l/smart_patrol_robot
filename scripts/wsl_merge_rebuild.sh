#!/bin/bash
set -e
cd /home/inspect/inspect_ws
echo "layout:"; cat install/.colcon_install_layout 2>/dev/null || true
echo "--- grep util ---"
grep -n 'nav2_gradient\|nav2_keepout\|packages' install/_local_setup_util_sh.py | head -40
echo "--- try merge install rebuild ---"
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install --merge-install
source install/setup.bash
echo "AMENT:"
echo "$AMENT_PREFIX_PATH" | tr ':' '\n'
ros2 pkg list | grep -E 'inspect_|nav2_gradient|nav2_keepout' || true
echo MERGE_OK
