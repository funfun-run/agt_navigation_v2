cmake_minimum_required(VERSION 3.8)
project(agt_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_bunker_qt5_baseline test/test_bunker_qt5_baseline.py
  )
endif()

ament_package()
