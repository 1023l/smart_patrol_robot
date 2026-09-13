#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /home/inspect/inspect_ws
colcon build --symlink-install --packages-select inspect_speed_adapter inspect_navigation
source install/setup.bash
echo "=== PACKAGES ==="
ros2 pkg list | grep inspect || true
ros2 pkg list | grep nav2_gradient || true
ros2 pkg list | grep nav2_keepout || true
echo BUILD_OK
