find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Qt5 COMPONENTS Widgets Concurrent Svg REQUIRED)

file(GLOB_RECURSE SOURCE_FILE CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.cc)
list(FILTER SOURCE_FILE EXCLUDE REGEX "(.*_test\\.cpp$)|(.*_test\\.cc$)")

file(GLOB_RECURSE UI_FILE CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB_RECURSE QRC_FILE CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR} resource/*.qrc)
file(GLOB_RECURSE INCLUDE_FILE CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR} *.h *.hpp)

qt5_wrap_ui(QT_UI_HPP ${UI_FILE})
qt5_wrap_cpp(QT_MOC_HPP ${INCLUDE_FILE})
qt5_add_resources(QT_RESOUCE_HPP ${QRC_FILE})

if(WIN32)
  add_executable(${PROJECT_NAME} WIN32
    ${SOURCE_FILE}
    ${INCLUDE_FILE}
    ${UI_FILE}
    ${QT_MOC_HPP}
    ${QRC_FILE}
    ${QT_RESOUCE_HPP})
else()
  add_executable(${PROJECT_NAME}
    ${SOURCE_FILE}
    ${INCLUDE_FILE}
    ${UI_FILE}
    ${QT_MOC_HPP}
    ${QRC_FILE}
    ${QT_RESOUCE_HPP})
endif()

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Core Qt5::Concurrent Qt5::Svg dockwidget::dockwidget nlohmann_json common basic framework channel ${OpenCV_LIBS})
set_target_properties(${PROJECT_NAME} PROPERTIES
RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
BUILD_RPATH "$ORIGIN:$ORIGIN/lib"
INSTALL_RPATH "$ORIGIN:$ORIGIN/lib"
)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin)

if(UNIX)
  file(GLOB LINUX_BIN_FILES "${CMAKE_SOURCE_DIR}/install/linux/bin/*")
  foreach(BIN_FILE ${LINUX_BIN_FILES})
    get_filename_component(FILENAME ${BIN_FILE} NAME)
    add_custom_command(
      TARGET ${PROJECT_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy
        ${BIN_FILE}
        ${CMAKE_BINARY_DIR}/${FILENAME}
      COMMENT "Copying ${FILENAME} to build directory"
    )
    if(FILENAME MATCHES "\\.(sh|py)$")
      add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND chmod 755
          ${CMAKE_BINARY_DIR}/${FILENAME}
        COMMENT "Setting execute permissions on ${FILENAME}"
      )
    endif()
  endforeach()
elseif(WIN32)
  file(GLOB WINDOWS_BIN_FILES "${CMAKE_SOURCE_DIR}/install/windows/bin/*")
  foreach(BIN_FILE ${WINDOWS_BIN_FILES})
    get_filename_component(FILENAME ${BIN_FILE} NAME)
    add_custom_command(
      TARGET ${PROJECT_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy
        ${BIN_FILE}
        ${CMAKE_BINARY_DIR}/${FILENAME}
      COMMENT "Copying ${FILENAME} to build directory"
    )
  endforeach()
endif()
