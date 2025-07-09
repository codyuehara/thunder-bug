cmake_minimum_required(VERSION 3.8)
project(joy_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(joy_control_node src/main.cpp)

ament_target_dependencies(joy_control_node
  rclcpp
  geometry_msgs
  std_msgs
  ros2_msgs
)

install(TARGETS
  joy_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

