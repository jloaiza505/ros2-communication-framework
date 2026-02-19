cmake_minimum_required(VERSION 3.8)
project(listener_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

install(
  PROGRAMS
    listener_pkg/listener_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME listener_node
)

install(
  DIRECTORY listener_pkg
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
