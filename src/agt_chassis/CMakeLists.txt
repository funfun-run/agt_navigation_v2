cmake_minimum_required(VERSION 3.8)
project(agt_chassis)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
