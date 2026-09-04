# -*- coding: utf-8 -*-
"""ament_python 包安装配置：注册两个节点并安装 config 与 launch 目录。"""

import os
from setuptools import setup

package_name = 'inspect_rl_avoidance'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 目录：随包安装，供 ros2 launch 使用
        (os.path.join('share', package_name, 'launch'),
            ['launch/rl_avoidance.launch.py']),
        # config 目录：存放 ONNX 模型文件（训练生成后复制到此处）
        (os.path.join('share', package_name, 'config'),
            ['config/README.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='inspect_rl_avoidance',
    maintainer_email='dev@example.com',
    description=(
        'PPO 强化学习 + 传统规划安全盾混合避障模块，'
        '解决人群密集场景下 TEB 局部规划器冻结卡死问题'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # RL 推理节点：ONNX 策略 + 代价地图安全盾
            'rl_avoidance = inspect_rl_avoidance.rl_avoidance_node:main',
            # 速度多路选择节点：在传统控制器与 RL 输出之间切换
            'velocity_mux = inspect_rl_avoidance.velocity_mux_node:main',
        ],
    },
)
