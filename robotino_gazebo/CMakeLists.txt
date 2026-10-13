cmake_minimum_required(VERSION 3.20)
project(robotino_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/sim_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The repository has no copyright headers in source files
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
