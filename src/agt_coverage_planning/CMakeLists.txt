cmake_minimum_required(VERSION 3.8)
project(agt_coverage_planning)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/coverage_request_adapter.py
  scripts/coverage_path_validator.py
  scripts/coverage_path_repair.py
  scripts/coverage_task_server.py
  scripts/coverage_time_simulator.py
  scripts/coverage_variant_comparator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_coverage_adapter test/test_coverage_adapter.py)
  ament_add_pytest_test(
    test_coverage_preview_launch test/test_coverage_preview_launch.py
  )
  ament_add_pytest_test(
    test_coverage_request_adapter_node
    test/test_coverage_request_adapter_node.py
  )
  ament_add_pytest_test(test_path_semantics test/test_path_semantics.py)
  ament_add_pytest_test(test_time_simulation test/test_time_simulation.py)
  ament_add_pytest_test(
    test_time_simulator_node test/test_time_simulator_node.py
  )
  ament_add_pytest_test(
    test_variant_comparison test/test_variant_comparison.py
  )
  ament_add_pytest_test(
    test_variant_comparator_node test/test_variant_comparator_node.py
  )
  ament_add_pytest_test(
    test_coverage_comparison_launch test/test_coverage_comparison_launch.py
  )
  ament_add_pytest_test(test_path_repair test/test_path_repair.py)
  ament_add_pytest_test(test_path_repair_node test/test_path_repair_node.py)
  ament_add_pytest_test(test_path_validator test/test_path_validator.py)
  ament_add_pytest_test(test_coverage_task test/test_coverage_task.py)
  ament_add_pytest_test(
    test_coverage_task_server_node
    test/test_coverage_task_server_node.py
  )
  ament_add_pytest_test(
    test_path_validator_node test/test_path_validator_node.py
  )
endif()

ament_package()
