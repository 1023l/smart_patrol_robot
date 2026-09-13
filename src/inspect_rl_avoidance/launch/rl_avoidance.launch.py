# -*- coding: utf-8 -*-
"""同时启动 RL 避障推理节点与速度多路选择节点。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ONNX 模型默认路径：包安装目录下 config/rl_policy.onnx
    default_onnx = os.path.join(
        get_package_share_directory("inspect_rl_avoidance"),
        "config", "rl_policy.onnx")

    onnx_path_arg = DeclareLaunchArgument(
        "onnx_path",
        default_value=default_onnx,
        description="PPO 策略 ONNX 模型文件路径",
    )

    # RL 推理节点：ONNX 推理 + 代价地图安全盾
    rl_avoidance_node = Node(
        package="inspect_rl_avoidance",
        executable="rl_avoidance",
        name="rl_avoidance",
        output="screen",
        parameters=[{"onnx_path": LaunchConfiguration("onnx_path")}],
    )

    # 速度多路选择节点：在传统控制器与 RL 输出之间切换
    velocity_mux_node = Node(
        package="inspect_rl_avoidance",
        executable="velocity_mux",
        name="velocity_mux",
        output="screen",
    )

    return LaunchDescription([
        onnx_path_arg,
        rl_avoidance_node,
        velocity_mux_node,
    ])
