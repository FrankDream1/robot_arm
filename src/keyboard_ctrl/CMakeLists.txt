cmake_minimum_required(VERSION 3.5)
project(keyboard_ctrl)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(keyboard_ctrl_node src/keyboard_ctrl_node.cpp)
ament_target_dependencies(keyboard_ctrl_node rclcpp std_msgs sensor_msgs)

install(TARGETS
  keyboard_ctrl_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()