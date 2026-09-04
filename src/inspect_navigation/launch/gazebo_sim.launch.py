"""
Gazebo 仿真环境：巡检机器人（差速底盘 + 360° 激光雷达）
==========================================================
复用 fishbot 模型（ros2bookcode chapt6/chapt8 的 fishbot_description），
提供差速运动学 + 激光雷达 + IMU 仿真。

启动内容：
  gazebo（空世界或厂房世界）+ robot_state_publisher + 巡检机器人 spawn
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用仿真时钟'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='启动 RViz'),
    ]

    # 尝试复用 fishbot_description（ros2bookcode 教材机器人：差速+雷达+IMU）
    # 若工作区未包含该包，按 gitee 鱼香 ROS 教程仓库安装
    fishbot_share = FindPackageShare('fishbot_description')

    # 机器人状态发布（URDF/xacro）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': PathJoinSubstitution([
                fishbot_share, 'urdf', 'fishbot.urdf.xacro']),
        }],
    )

    # 关节状态发布（车轮可视化）
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Gazebo：先尝试教程的仿真启动（含 ros2_control / 雷达插件 / spawn）
    # 该 launch 属于 fishbot_description 包，含 gazebo_ros 初始化与机器人生成
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            fishbot_share, 'launch', 'gazebo_sim.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription(
        declare_args + [
            robot_state_publisher,
            joint_state_publisher,
            gazebo_sim,
        ]
    )
