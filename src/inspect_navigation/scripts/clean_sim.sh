#!/bin/bash
# 清理因强制关闭或超时测试残留的 ROS/Gazebo 进程，避免同名节点争抢服务。

pkill -9 -f 'ros2 launch inspect_navigation' 2>/dev/null || true
pkill -9 -f '/opt/ros/humble/lib/nav2_' 2>/dev/null || true
pkill -9 -f '/opt/ros/humble/lib/rviz2/rviz2' 2>/dev/null || true
pkill -9 -f 'goal_pose_relay.py' 2>/dev/null || true
pkill -9 -f 'seven_sector_speed_node|nav_metrics_node|floor_manager' 2>/dev/null || true
pkill -9 -f 'robot_state_publisher|spawn_entity.py|controller_manager' 2>/dev/null || true
killall -9 gzserver gzclient 2>/dev/null || true

echo "巡检仿真残留进程已清理"
