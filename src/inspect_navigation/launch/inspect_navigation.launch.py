"""
工业巡检导航系统总启动文件
=================================
组装全部自研模块：
  Nav2 核心（规划器=梯度A* / 控制器=TEB / keepout层 / 四级脱困BT）
  + 七扇区调速 + 导航指标 + 楼层管理

用法：
  # 推荐一键：Gazebo + Nav2 + RViz
  ros2 launch inspect_navigation full_sim.launch.py

  # 或分终端：
  #   ros2 launch inspect_navigation gazebo_sim.launch.py
  #   ros2 launch inspect_navigation inspect_navigation.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, TimerAction
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
            'use_rviz', default_value='true',
            description='启动 RViz 可视化'),
    ]

    # map_server 先加载 floor1，跨层由 floor_manager 通过 load_map 接管
    default_map = os.path.join(pkg_share, 'maps', 'floor1.yaml')

    # ---- Nav2 生命周期节点组（标准 bringup 风格）----
    # 定位与导航必须分开管理。若导航插件配置失败，不能连带阻塞地图和 AMCL。
    localization_nodes = [
        'map_server',
        'amcl',
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
            remappings=[('cmd_vel', 'cmd_vel_nav')],
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
            remappings=[('cmd_vel', 'cmd_vel_nav')],
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

    # 定位生命周期管理器：地图和 AMCL 独立上线，不受规划/控制插件影响
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': localization_nodes},
        ],
    )

    # WSL/Gazebo 同时启动时，标准管理器可能在 controller_server 配置完成前
    # 超时，导致整条导航链停在 unconfigured。改为延迟后逐节点顺序激活。
    activate_navigation = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash',
                    os.path.join(pkg_share, 'scripts', 'activate_navigation.sh'),
                ],
                output='screen',
            )
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

    # RViz（默认开；必须 use_sim_time，否则激光/代价地图时间戳对不上）
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'inspect_nav.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declare_args + [
            map_server,
            nav2_group,
            amcl,
            speed_adapter_group,
            floor_manager,
            lifecycle_manager_localization,
            activate_navigation,
            rviz,
        ]
    )
