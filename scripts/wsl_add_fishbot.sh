#!/bin/bash
set -e
source /opt/ros/humble/setup.bash

FISHBOT_SRC=/mnt/c/Users/Administrator/Documents/trae_projects/ros2bookcode-master/chapt6/chapt6_ws/src/fishbot_description
WS=/home/inspect/inspect_ws

ln -sfn "$FISHBOT_SRC" "$WS/src/fishbot_description"

export DEBIAN_FRONTEND=noninteractive
sudo apt-get install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  2>&1 | tee /tmp/apt_fishbot.log | tail -20

cd "$WS"
colcon build --symlink-install --merge-install --packages-select fishbot_description inspect_navigation
source install/setup.bash

echo "=== check pkgs ==="
ros2 pkg prefix fishbot_description
ros2 pkg prefix nav2_gradient_planner
echo SETUP_OK
