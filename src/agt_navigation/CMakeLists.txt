cmake_minimum_required(VERSION 3.8)
project(agt_navigation)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/differential_drive_simulator.py
  scripts/goal_pose_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config behavior_trees maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
