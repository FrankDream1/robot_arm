cmake_minimum_required(VERSION 3.5)
project(auto_ctrl)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(auto_ctrl_node
  src/controller.cpp
  src/main.cpp
)
ament_target_dependencies(auto_ctrl_node
  rclcpp std_msgs geometry_msgs sensor_msgs
)

install(TARGETS auto_ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()