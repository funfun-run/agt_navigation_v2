cmake_minimum_required(VERSION 3.8)
project(relocalization_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration)
find_package(ndt_omp_ros2 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/map_store.cpp
  src/scan_preprocessor.cpp
  src/relocalizer.cpp
  src/backends/icp_backend.cpp
  src/backends/ndt_backend.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${ndt_omp_ros2_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    OpenMP::OpenMP_CXX
    ${PCL_LIBRARIES}
    ${ndt_omp_ros2_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_relocalizer test/test_relocalizer.cpp)
  if(TARGET test_relocalizer)
    target_link_libraries(test_relocalizer ${PROJECT_NAME} ${PCL_LIBRARIES})
    target_include_directories(test_relocalizer PRIVATE ${PCL_INCLUDE_DIRS})
  endif()

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
