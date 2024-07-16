cmake_minimum_required(VERSION 3.8)

# source: https://github.com/MarcTestier/ros2_cmake_test/blob/master/foo/CMakeLists.txt
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/package.xml package_xml_str)
if(NOT package_xml_str MATCHES "<name>([A-Za-z0-9_]+)</name>")
  message(FATAL_ERROR "Could not parse project name from package.xml.")
endif()
set(PACKAGE_NAME ${CMAKE_MATCH_1})
if(NOT package_xml_str MATCHES "<version>([0-9]+\.[0-9]+\.[0-9]+)</version>")
  message(FATAL_ERROR "Could not parse project version from package.xml.")
endif()
set(PACKAGE_VERSION ${CMAKE_MATCH_1})
message(STATUS "Package name: ${PACKAGE_NAME}\nPackage version: ${PACKAGE_VERSION}")

project(${PACKAGE_NAME} VERSION ${PACKAGE_VERSION})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files(
  urdf/robotino.urdf.xacro
  TARGET xacro_target
  INSTALL DESTINATION urdf
)

install(DIRECTORY launch meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
