cmake_minimum_required(VERSION 3.8)
project(hb_testing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts as executables
install(PROGRAMS
  src/camera_testing.py
  DESTINATION lib/${PROJECT_NAME}
)

# Also install in share for the current launch file approach
install(DIRECTORY
  src
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
