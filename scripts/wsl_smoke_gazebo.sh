#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/inspect/inspect_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:/home/inspect/inspect_ws/install/share/fishbot_description/world"
timeout 50 ros2 launch fishbot_description gazebo_sim.launch.py 2>&1 | tee /tmp/gazebo_launch.log | tail -80
echo LAUNCH_EXIT:$?
