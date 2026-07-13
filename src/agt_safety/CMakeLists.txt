cmake_minimum_required(VERSION 3.8)
project(agt_safety)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/tracked_safety_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
