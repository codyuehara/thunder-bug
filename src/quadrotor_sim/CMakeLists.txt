cmake_minimum_required(VERSION 3.8)
project(quadrotor_sim)

set(Bullet_INCLUDE_DIRS "/usr/include/bullet")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(Bullet REQUIRED) 

add_executable(quadrotor_sim_node src/main.cpp)
target_include_directories(quadrotor_sim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
#target_link_libraries(quadrotor_sim_node BulletDynamics BulletCollision LinearMath)

ament_target_dependencies(quadrotor_sim_node
  rclcpp
  geometry_msgs
  std_msgs
  ros2_msgs
  sensor_msgs
)

install(TARGETS
  quadrotor_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY include/ DESTINATION include/)

ament_package()

