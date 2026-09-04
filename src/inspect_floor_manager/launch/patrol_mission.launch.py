"""巡逻任务节点 launch 文件：单独启动 patrol_mission 节点。

注意：运行前请先启动 floor_manager 与 Nav2 导航栈。
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

    patrol_points_config_arg = DeclareLaunchArgument(
        'patrol_points_config',
        default_value=default_points_config,
        description='巡检点配置 yaml 路径')

    rounds_arg = DeclareLaunchArgument(
        'rounds',
        default_value='2',
        description='巡逻轮数（完整遍历全部楼层记为一轮）')

    patrol_mission_node = Node(
        package='inspect_floor_manager',
        executable='patrol_mission',
        name='patrol_mission',
        output='screen',
        parameters=[{
            'patrol_points_config': LaunchConfiguration('patrol_points_config'),
            'rounds': ParameterValue(LaunchConfiguration('rounds'), value_type=int),
        }])

    return LaunchDescription([patrol_points_config_arg, rounds_arg, patrol_mission_node])
