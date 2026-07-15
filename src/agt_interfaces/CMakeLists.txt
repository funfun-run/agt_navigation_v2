cmake_minimum_required(VERSION 3.8)
project(agt_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteCoverageTask.action"
  DEPENDENCIES action_msgs std_msgs geometry_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_execute_coverage_task_serialization
    test/test_execute_coverage_task_serialization.py
  )
  ament_add_gtest(
    test_execute_coverage_task_cpp
    test/test_execute_coverage_task.cpp
  )
  rosidl_get_typesupport_target(
    cpp_typesupport_target
    ${PROJECT_NAME}
    rosidl_typesupport_cpp
  )
  target_link_libraries(
    test_execute_coverage_task_cpp
    "${cpp_typesupport_target}"
  )
endif()

ament_package()
