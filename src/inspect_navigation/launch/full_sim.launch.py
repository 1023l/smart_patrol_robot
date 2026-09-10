"""
一键仿真：Gazebo(fishbot) + Nav2 巡检导航 + RViz

用法（Ubuntu-22.04 / Humble）：
  source /opt/ros/humble/setup.bash
  source ~/inspect_ws/install/setup.bash
  # 先清掉残留 gzserver / 旧 launch
  ros2 launch inspect_navigation full_sim.launch.py

RViz 里：
  1) 2D Pose Estimate 对准小车大致位姿（地图与房间不一致时尤其必要）
  2) Nav2 Goal / 2D Goal Pose 下目标
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('inspect_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    floor = LaunchConfiguration('floor')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo_sim.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 等 Gazebo spawn + 控制器 + /scan 起来再启 Nav2（WSL 偏慢）
    navigation = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'inspect_navigation.launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'use_rviz': use_rviz,
                    'floor': floor,
                    # TimerAction 内嵌 launch 时显式传递，避免默认值丢失
                    'params_file': params_file,
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('floor', default_value='1'),
        gazebo,
        navigation,
    ])
