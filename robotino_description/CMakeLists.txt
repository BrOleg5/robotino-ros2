cmake_minimum_required(VERSION 3.8)
project(robotino_description)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch model rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES robotino.urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
