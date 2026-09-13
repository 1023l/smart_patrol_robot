#!/bin/bash
set -e
source /opt/ros/humble/setup.bash

WIN_SRC=/mnt/c/Users/Administrator/Documents/trae_projects/inspect_ws/src
WS=/home/inspect/inspect_ws

# 重建工作区：去掉整目录软链，改按包软链
rm -rf "$WS"
mkdir -p "$WS/src"

for pkg in inspect_floor_manager inspect_interfaces inspect_navigation \
           inspect_rl_avoidance inspect_speed_adapter inspect_vision \
           nav2_gradient_planner nav2_keepout_layer; do
  ln -sfn "$WIN_SRC/$pkg" "$WS/src/$pkg"
done

# 清 Windows 侧误克隆
rm -rf "$WIN_SRC/teb_local_planner" "$WIN_SRC/costmap_converter" || true

cd "$WS/src"
echo "Cloning TEB deps into WSL-only src..."

clone_repo() {
  local url="$1"
  local dir="$2"
  local branch="$3"
  rm -rf "$dir"
  echo ">>> $dir ($branch)"
  if timeout 120 git clone -b "$branch" --depth 1 "https://ghproxy.net/$url" "$dir"; then
    return 0
  fi
  echo "ghproxy failed, try direct..."
  timeout 180 git clone -b "$branch" --depth 1 "$url" "$dir"
}

clone_repo https://github.com/rst-tu-dortmund/teb_local_planner.git teb_local_planner ros2-master
clone_repo https://github.com/ros-planning/costmap_converter.git costmap_converter ros2

ls -la "$WS/src"
echo CLONE_DONE
