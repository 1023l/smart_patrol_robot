"""楼层管理节点 launch 文件：启动 floor_manager 节点。

maps_dir 通过 launch 参数注入，默认指向 inspect_navigation 包
share 目录下的 maps 目录（分层子地图存放位置）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('inspect_floor_manager')
    default_floors_config = os.path.join(pkg_share, 'config', 'floors.yaml')

    # 默认地图目录：inspect_navigation 包 share 目录下的 maps 目录；
    # 若 inspect_navigation 尚未安装，则退化为空字符串，由调用方显式传入。
    try:
        nav_share = get_package_share_directory('inspect_navigation')
        default_maps_dir = os.path.join(nav_share, 'maps')
    except Exception:
        default_maps_dir = ''

    maps_dir_arg = DeclareLaunchArgument(
        'maps_dir',
        default_value=default_maps_dir,
        description='楼层地图目录（默认为 inspect_navigation 包 share 目录下的 maps）')

    floors_config_arg = DeclareLaunchArgument(
        'floors_config',
        default_value=default_floors_config,
        description='楼层配置 yaml 路径')

    floor_manager_node = Node(
        package='inspect_floor_manager',
        executable='floor_manager',
        name='floor_manager',
        output='screen',
        parameters=[{
            'maps_dir': LaunchConfiguration('maps_dir'),
            'floors_config': LaunchConfiguration('floors_config'),
        }])

    return LaunchDescription([maps_dir_arg, floors_config_arg, floor_manager_node])
