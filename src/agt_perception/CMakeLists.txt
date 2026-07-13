cmake_minimum_required(VERSION 3.8)
project(agt_perception)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(local_obstacle_filter src/local_obstacle_filter.cpp)
target_compile_features(local_obstacle_filter PRIVATE cxx_std_17)
ament_target_dependencies(local_obstacle_filter geometry_msgs rclcpp sensor_msgs tf2 tf2_ros)

install(TARGETS local_obstacle_filter DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
