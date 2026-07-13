cmake_minimum_required(VERSION 3.8)
project(agt_localization)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(relocalization_core REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(relocalization_node src/relocalization_node.cpp)
target_compile_features(relocalization_node PRIVATE cxx_std_17)
target_include_directories(relocalization_node PRIVATE include ${PCL_INCLUDE_DIRS})
ament_target_dependencies(relocalization_node
  geometry_msgs pcl_conversions rclcpp relocalization_core sensor_msgs std_msgs tf2 tf2_ros
)
target_link_libraries(relocalization_node
  relocalization_core::relocalization_core ${PCL_LIBRARIES}
)

install(TARGETS relocalization_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
