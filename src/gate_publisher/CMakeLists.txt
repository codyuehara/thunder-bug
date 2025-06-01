cmake_minimum_required(VERSION 3.8)
project(gate_publisher)

set(Bullet_INCLUDE_DIRS "/usr/include/bullet")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gate_msgs REQUIRED) 
find_package(Bullet REQUIRED) 

add_executable(gate_publisher_node src/gate_publisher.cpp)
target_include_directories(gate_publisher_node PRIVATE ${Bullet_INCLUDE_DIRS})
target_link_libraries(gate_publisher_node BulletDynamics BulletCollision LinearMath)

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

