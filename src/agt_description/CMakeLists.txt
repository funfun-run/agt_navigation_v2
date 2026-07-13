cmake_minimum_required(VERSION 3.8)
project(agt_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_tf_contract test/test_tf_contract.py)
endif()

ament_package()
