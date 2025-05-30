cmake_minimum_required(VERSION 3.8)
project(gate_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gate_msgs REQUIRED) 

add_executable(gate_publisher_node src/gate_publisher.cpp)

ament_target_dependencies(gate_publisher_node
  rclcpp
  geometry_msgs
  gate_msgs
)

install(TARGETS
  gate_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

