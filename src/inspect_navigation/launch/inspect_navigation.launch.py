"""
工业巡检导航系统总启动文件
=================================
组装全部自研模块：
  Nav2 核心（规划器=梯度A* / 控制器=TEB / keepout层 / 四级脱困BT）
  + 七扇区调速 + 导航指标 + 楼层管理

用法：
  # 首次使用先生成 5 层地图（Ubuntu + Python3 + numpy）
  #   python3 scripts/generate_maps.py --output-dir maps/
  ros2 launch inspect_navigation inspect_navigation.launch.py [floor:=1]

配套：
  Gazebo 仿真机器人（含雷达）另起：
    ros2 launch inspect_navigation gazebo_sim.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('inspect_navigation')
    speed_adapter_share = get_package_share_directory('inspect_speed_adapter')
    floor_manager_share = get_package_share_directory('inspect_floor_manager')

    # ---- 启动参数 ----
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    floor = LaunchConfiguration('floor')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用仿真时钟'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Nav2 全量参数文件'),
        DeclareLaunchArgument(
            'floor', default_value='1',
            description='初始楼层（1~5，运行期可通过 /switch_floor 服务切换）'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='生命周期节点自动激活'),
        DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='使用容器组合模式'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='启动 RViz 可视化'),
    ]

    # 初始楼层地图路径（floor_manager 会通过 load_map 动态切换后续楼层）
    initial_map = os.path.join(pkg_share, 'maps', ['floor', '.yaml'])

    # 按参数拼接初始地图路径（launch 表达式能力有限，这里直接用 floor 参数
    # 的字符串拼接在 Node 的 map 参数里做不了，改用 map_server 的 yaml 参数）
    # 简化处理：map_server 先加载 floor1，跨层由 floor_manager 接管
    default_map = os.path.join(pkg_share, 'maps', 'floor1.yaml')

    # ---- Nav2 生命周期节点组（标准 bringup 风格）----
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # map_server：初始加载 floor1（后续楼层由 floor_manager 的 load_map 切换）
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': default_map},
        ],
    )

    nav2_group = GroupAction([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                params_file,
                # 行为树绝对路径注入（相对路径按 cwd 解析不可靠）
                {'default_nav_to_pose_bt_xml': os.path.join(
                    pkg_share, 'behavior', 'inspect_recoveries_bt.xml')},
            ],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),
    ])

    # 生命周期管理器：统一激活 Nav2 节点
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes + ['map_server']},
        ],
    )

    # AMCL 定位（楼层切换时由 floor_manager 发 /initialpose 重定位）
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
    )

    # ---- 自研模块组 ----
    # 七扇区调速 + 导航指标（inspect_speed_adapter）
    speed_adapter_group = GroupAction([
        Node(
            package='inspect_speed_adapter',
            executable='seven_sector_speed_node',
            name='seven_sector_speed',
            output='screen',
            parameters=[os.path.join(
                speed_adapter_share, 'config', 'speed_adapter.yaml')],
            remappings=[('scan', '/scan')],
        ),
        Node(
            package='inspect_speed_adapter',
            executable='nav_metrics_node',
            name='nav_metrics',
            output='screen',
            parameters=[os.path.join(
                speed_adapter_share, 'config', 'speed_adapter.yaml')],
            remappings=[
                ('plan', '/plan'),
                ('cmd_vel', '/cmd_vel'),
                ('navigate_to_pose_status', '/navigate_to_pose/_action/status'),
            ],
        ),
    ])

    # 楼层管理（maps_dir 指向本包 maps 目录）
    floor_manager = Node(
        package='inspect_floor_manager',
        executable='floor_manager',
        name='floor_manager',
        output='screen',
        parameters=[{
            'maps_dir': os.path.join(pkg_share, 'maps'),
            'floors_config': os.path.join(
                floor_manager_share, 'config', 'floors.yaml'),
        }],
    )

    # RViz（可选）
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'inspect_nav.rviz')],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declare_args + [
            map_server,
            nav2_group,
            amcl,
            speed_adapter_group,
            floor_manager,
            lifecycle_manager,
            rviz,
        ]
    )
