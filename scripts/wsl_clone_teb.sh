#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /home/inspect/inspect_ws/src

# 清理半成品
rm -rf teb_local_planner costmap_converter

# 优先走镜像，失败再直连
clone_repo() {
  local url="$1"
  local dir="$2"
  local branch="$3"
  if [ -n "$branch" ]; then
    git clone -b "$branch" --depth 1 "https://gitclone.com/$url" "$dir" \
      || git clone -b "$branch" --depth 1 "https://ghproxy.net/$url" "$dir" \
      || git clone -b "$branch" --depth 1 "$url" "$dir"
  else
    git clone --depth 1 "https://gitclone.com/$url" "$dir" \
      || git clone --depth 1 "https://ghproxy.net/$url" "$dir" \
      || git clone --depth 1 "$url" "$dir"
  fi
}

clone_repo https://github.com/rst-tu-dortmund/teb_local_planner.git teb_local_planner ros2-master
clone_repo https://github.com/ros-planning/costmap_converter.git costmap_converter ros2

# 避免误提交到用户 GitHub：加本地 ignore 标记文件说明
touch teb_local_planner/COLCON_IGNORE 2>/dev/null || true
# 先不 ignore，要编的话不能 COLCON_IGNORE teb
rm -f teb_local_planner/COLCON_IGNORE

ls -la
echo CLONE_DONE
