import os

from setuptools import find_packages
from setuptools import setup

package_name = 'inspect_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            ['config/inspect_vision.yaml']),
        (os.path.join('share', package_name, 'launch'),
            ['launch/inspect_vision.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='inspect',
    maintainer_email='inspect@example.com',
    description='到点视觉巡检：抓图、检测、结果落盘',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inspect_node = inspect_vision.inspect_node:main',
            'demo_offline = inspect_vision.demo_offline:main',
        ],
    },
)
