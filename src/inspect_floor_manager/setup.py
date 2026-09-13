import os

from setuptools import find_packages
from setuptools import setup

package_name = 'inspect_floor_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            ['config/floors.yaml', 'config/patrol_points.yaml']),
        (os.path.join('share', package_name, 'launch'),
            ['launch/floor_manager.launch.py', 'launch/patrol_mission.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='inspect',
    maintainer_email='inspect@example.com',
    description='工业厂房跨楼层巡检管理：分层子地图切换、电梯口 AMCL 重定位与跨楼层巡逻任务',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'floor_manager = inspect_floor_manager.floor_manager_node:main',
            'patrol_mission = inspect_floor_manager.patrol_mission_node:main',
        ],
    },
)
