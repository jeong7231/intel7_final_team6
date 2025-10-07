cmake_minimum_required(VERSION 3.8)
project(turtlebot3_dumpbox)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(dumpbox_heartbeat src/common/heartbeat_node.cpp)
ament_target_dependencies(dumpbox_heartbeat
  rclcpp
  std_msgs
)

add_executable(dumpbox_servo_controller src/robot/servo_controller_node.cpp)
ament_target_dependencies(dumpbox_servo_controller
  rclcpp
  std_msgs
  std_srvs
)

add_executable(dumpbox_mission_executor src/remote/mission_executor_node.cpp)
ament_target_dependencies(dumpbox_mission_executor
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  std_srvs
  tf2_ros
)

install(TARGETS
  dumpbox_heartbeat
  dumpbox_servo_controller
  dumpbox_mission_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config include
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
