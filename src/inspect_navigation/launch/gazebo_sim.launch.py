"""
Gazebo 仿真：直接复用鱼香 ROS 第6章 fishbot_description。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    fishbot_share = FindPackageShare('fishbot_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用仿真时钟'),
        # fishbot 自带：Gazebo world + spawn + ros2_control 差速/雷达
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                fishbot_share, 'launch', 'gazebo_sim.launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
