cmake_minimum_required(VERSION 3.8)
project(inspect_navigation)

find_package(ament_cmake REQUIRED)

# 安装资源目录：参数 / 地图 / 行为树 / launch / 脚本
# 机器人模型与 Gazebo 仿真复用 fishbot_description（差速底盘 + 360°雷达）
install(DIRECTORY
  config maps behavior launch scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
