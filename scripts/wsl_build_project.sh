#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
WS=/home/inspect/inspect_ws
WIN_SRC=/mnt/c/Users/Administrator/Documents/trae_projects/inspect_ws/src

# 确保包软链在
mkdir -p "$WS/src"
for pkg in inspect_floor_manager inspect_interfaces inspect_navigation \
           inspect_rl_avoidance inspect_speed_adapter inspect_vision \
           nav2_gradient_planner nav2_keepout_layer; do
  ln -sfn "$WIN_SRC/$pkg" "$WS/src/$pkg"
done

# 半成品第三方先忽略，避免编挂
if [ -d "$WS/src/teb_local_planner" ]; then
  touch "$WS/src/teb_local_planner/COLCON_IGNORE"
fi
if [ -d "$WS/src/costmap_converter" ]; then
  touch "$WS/src/costmap_converter/COLCON_IGNORE"
fi

# 生成地图
python3 "$WIN_SRC/inspect_navigation/scripts/generate_maps.py" \
  --output-dir "$WIN_SRC/inspect_navigation/maps"
ls -la "$WIN_SRC/inspect_navigation/maps" | head

cd "$WS"
echo "=== COLCON BUILD ==="
colcon build --symlink-install --packages-select \
  inspect_interfaces nav2_gradient_planner nav2_keepout_layer \
  inspect_speed_adapter inspect_navigation inspect_floor_manager inspect_vision \
  2>&1 | tee /tmp/colcon_build.log | tail -40
echo BUILD_EXIT:$?
source install/setup.bash
ros2 pkg list | grep -E 'inspect_|nav2_gradient|nav2_keepout' || true
echo ALL_DONE
