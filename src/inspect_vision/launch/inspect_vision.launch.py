"""到点视觉巡检 launch。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('inspect_vision')
    default_params = os.path.join(pkg_share, 'config', 'inspect_vision.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='视觉巡检参数文件'),
        DeclareLaunchArgument(
            'results_dir', default_value='/tmp/inspect_results',
            description='巡检结果落盘目录'),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/color/image_raw',
            description='相机图像话题'),
        Node(
            package='inspect_vision',
            executable='inspect_node',
            name='inspect_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'results_dir': LaunchConfiguration('results_dir'),
                    'image_topic': LaunchConfiguration('image_topic'),
                },
            ],
        ),
    ])
