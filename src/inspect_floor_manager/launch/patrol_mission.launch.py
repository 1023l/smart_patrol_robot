"""巡逻任务节点 launch：导航 + 可选视觉巡检。

注意：运行前请先启动 floor_manager、Nav2；视觉巡检需同时启动 inspect_vision。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('inspect_floor_manager')
    default_points_config = os.path.join(pkg_share, 'config', 'patrol_points.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'patrol_points_config',
            default_value=default_points_config,
            description='巡检点配置 yaml 路径'),
        DeclareLaunchArgument(
            'rounds',
            default_value='2',
            description='巡逻轮数（完整遍历全部楼层记为一轮）'),
        DeclareLaunchArgument(
            'enable_vision',
            default_value='true',
            description='到点后是否调用 /inspect_point 视觉巡检'),
        Node(
            package='inspect_floor_manager',
            executable='patrol_mission',
            name='patrol_mission',
            output='screen',
            parameters=[{
                'patrol_points_config': LaunchConfiguration('patrol_points_config'),
                'rounds': ParameterValue(LaunchConfiguration('rounds'), value_type=int),
                'enable_vision': ParameterValue(
                    LaunchConfiguration('enable_vision'), value_type=bool),
            }]),
    ])
