cmake_minimum_required(VERSION 3.8)
project(agt_evaluation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
