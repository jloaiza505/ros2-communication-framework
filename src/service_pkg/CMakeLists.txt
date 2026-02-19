cmake_minimum_required(VERSION 3.8)
project(service_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

install(
  PROGRAMS
    service_pkg/service_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME service_server
)

install(
  PROGRAMS
    service_pkg/service_client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME service_client
)

install(
  DIRECTORY service_pkg
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
