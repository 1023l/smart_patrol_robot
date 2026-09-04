"""
局部运动优化模块启动文件：七扇区调速节点 + 导航指标观测节点
配套 TEB 控制器（teb_controller.yaml 中的 FollowPath 段并入 nav2_params）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('inspect_speed_adapter')

    default_params = os.path.join(pkg_share, 'config', 'speed_adapter.yaml')

    params_file = LaunchConfiguration('params_file')

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='调速与指标节点参数文件路径',
    )

    # 七扇区自适应调速节点：/scan → 环境分类 → speed_limit（作用于 TEB）
    seven_sector_node = Node(
        package='inspect_speed_adapter',
        executable='seven_sector_speed_node',
        name='seven_sector_speed',
        output='screen',
        parameters=[params_file],
        remappings=[
            # 雷达话题按实际机器人改名
            ('/scan', '/scan'),
        ],
    )

    # 导航指标观测节点：规划时延 / 任务完成率 / 路径平滑度 / 冻结检测
    nav_metrics_node = Node(
        package='inspect_speed_adapter',
        executable='nav_metrics_node',
        name='nav_metrics',
        output='screen',
        parameters=[params_file],
        remappings=[
            # Nav2 规划器输出的全局路径话题
            ('plan', '/plan'),
            # 控制器最终输出指令（在速度平滑器之后）
            ('cmd_vel', '/cmd_vel'),
            # NavigateToPose 动作状态数组（bt_navigator 的动作服务器）
            ('navigate_to_pose_status', '/navigate_to_pose/_action/status'),
        ],
    )

    return LaunchDescription([
        declare_params,
        seven_sector_node,
        nav_metrics_node,
    ])
