#!/bin/bash
# WSL/Gazebo 下 lifecycle_manager 并行启动时，controller_server 的配置响应
# 偶尔会超过其客户端等待时间。这里按顺序配置、激活，避免整条导航链被卡住。

export ROS_DISABLE_DAEMON=1

NODES=(
  controller_server
  smoother_server
  planner_server
  behavior_server
  bt_navigator
  waypoint_follower
  velocity_smoother
)

transition() {
  local node="$1"
  local action="$2"
  local expected="$3"
  for attempt in 1 2 3; do
    echo "[导航激活] ${action} ${node}（第 ${attempt} 次）"
    timeout 30 ros2 lifecycle set "/${node}" "${action}" || true
    sleep 1
    # ros2 lifecycle set 在回调失败时也可能打印 successful，必须核对真实状态。
    if timeout 10 ros2 lifecycle get "/${node}" 2>/dev/null | grep -q "^${expected} "; then
      echo "[导航激活] ${node} -> ${expected}"
      return 0
    fi
    sleep 2
  done
  echo "[导航激活] ${node} 未进入 ${expected}，停止后续激活"
  return 1
}

for node in "${NODES[@]}"; do
  transition "${node}" configure inactive || exit 1
done

for node in "${NODES[@]}"; do
  transition "${node}" activate active || exit 1
done

echo "[导航激活] 全部导航节点已 active"
