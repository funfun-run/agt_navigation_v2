cmake_minimum_required(VERSION 3.8)
project(agt_sensor_adapters)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config src
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
