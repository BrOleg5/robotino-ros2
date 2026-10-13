cmake_minimum_required(VERSION 3.20)

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

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "/usr/local/robotino/api2/cmake" "/opt/robotino/cmake")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(RobotinoAPI2 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rto_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)

add_executable(
  robotino_node
  src/robotino_node.cpp
  src/RobotinoNode.cpp
  src/BumperROS.cpp
  src/ComROS.cpp
  src/DistanceSensorArrayROS.cpp
  src/MotorArrayROS.cpp
  src/OmniDriveROS.cpp
  src/LaserRangeFinderROS.cpp
  src/OdometryROS.cpp)

target_link_libraries(
  robotino_node
  ${REC_ROBOTINO_API2_LIBRARY}
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${builtin_interfaces_TARGETS}
  ${rto_msgs_TARGETS}
)

target_include_directories(
  robotino_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  ${ROBOTINOAPI2_INCLUDE_DIR}
)

add_executable(
  robotino_camera_node
  src/robotino_camera_node.cpp
  src/RobotinoCameraNode.cpp
  src/ComROS.cpp
  src/CameraROS.cpp)

target_link_libraries(
  robotino_camera_node
  ${REC_ROBOTINO_API2_LIBRARY}
  rclcpp::rclcpp
  camera_info_manager::camera_info_manager
  ${sensor_msgs_TARGETS}
)

target_include_directories(
  robotino_camera_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  ${ROBOTINOAPI2_INCLUDE_DIR}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  robotino_node
  robotino_camera_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
