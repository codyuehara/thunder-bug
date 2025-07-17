cmake_minimum_required(VERSION 3.8)
project(racetrack)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

message(STATUS "YAML-CPP_INCLUDE_DIRS: ${yaml-cpp_INCLUDE_DIRS}")
message(STATUS "YAML-CPP_LIBRARIES: ${yaml-cpp_LIBRARIES}")
add_executable(racetrack_node src/racetrack.cpp)

ament_target_dependencies(racetrack_node
  rclcpp
  geometry_msgs
  std_msgs
  ros2_msgs
  sensor_msgs
  yaml-cpp
)

target_link_libraries(racetrack_node yaml-cpp)

target_include_directories(racetrack_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  racetrack_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY include/ DESTINATION include/)

ament_package()

