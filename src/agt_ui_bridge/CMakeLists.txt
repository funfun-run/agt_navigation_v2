cmake_minimum_required(VERSION 3.8)
project(agt_ui_bridge)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/map_io_bridge.py
  scripts/map_editor_qt5.py
  scripts/start_ros_qt5_gui_app.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_map_io test/test_map_io.py)
endif()

ament_package()
