cmake_minimum_required(VERSION 3.5)
project(serial_comm)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(serial_comm_node src/serial_comm_node.cpp)
ament_target_dependencies(serial_comm_node rclcpp std_msgs sensor_msgs)

install(TARGETS
  serial_comm_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()