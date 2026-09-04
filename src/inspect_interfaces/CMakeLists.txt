cmake_minimum_required(VERSION 3.8)
project(inspect_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 消息与服务接口定义
rosidl_generate_interfaces(${PROJECT_NAME}
  # 禁区管理：静态 YAML 加载 + 运行期话题动态增删共用
  "msg/KeepoutZone.msg"
  "srv/SwitchMap.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
