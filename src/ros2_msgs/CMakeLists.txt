cmake_minimum_required(VERSION 3.8)
project(ros2_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gate.msg"
  "msg/GateArray.msg"
  "msg/JoyControl.msg"
  "msg/MotorCommands.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
  )

ament_package()

