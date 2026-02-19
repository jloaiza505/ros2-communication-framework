cmake_minimum_required(VERSION 3.8)
project(talker_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

install(
  PROGRAMS
    talker_pkg/talker_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME talker_node
)

install(
  DIRECTORY talker_pkg
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
