cmake_minimum_required(VERSION 3.8)
project(inspect_speed_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# ---- 七扇区自适应调速节点 ----
add_executable(seven_sector_speed_node src/seven_sector_speed_node.cpp)
target_include_directories(seven_sector_speed_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(seven_sector_speed_node
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs nav2_msgs)

# ---- 导航指标观测节点 ----
add_executable(nav_metrics_node src/nav_metrics_node.cpp)
target_include_directories(nav_metrics_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(nav_metrics_node
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs std_srvs action_msgs)

install(TARGETS seven_sector_speed_node nav_metrics_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs std_srvs action_msgs nav2_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
