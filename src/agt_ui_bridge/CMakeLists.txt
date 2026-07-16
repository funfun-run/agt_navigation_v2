cmake_minimum_required(VERSION 3.8)
project(agt_ui_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/map_io_bridge.py
  scripts/map_editor_qt5.py
  scripts/semantic_editor_qt5.py
  scripts/semantic_map_server.py
  scripts/start_ros_qt5_gui_app.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_map_io test/test_map_io.py)
  ament_add_pytest_test(test_map_transform test/test_map_transform.py)
  ament_add_pytest_test(test_semantic_io test/test_semantic_io.py)
  ament_add_pytest_test(test_semantic_validation test/test_semantic_validation.py)
  ament_add_pytest_test(test_semantic_scene test/test_semantic_scene.py)
  ament_add_pytest_test(test_semantic_editor test/test_semantic_editor.py)
  ament_add_pytest_test(test_semantic_map_server test/test_semantic_map_server.py)
  ament_add_pytest_test(test_semantic_rasterizer test/test_semantic_rasterizer.py)
  ament_add_pytest_test(test_coverage_preview test/test_coverage_preview.py)
endif()

ament_package()
