cmake_minimum_required(VERSION 3.8)
project(tortoisebot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies - REMOVE the gazebo_ros line
find_package(ament_cmake REQUIRED)
# Don't find_package for ros_gz_sim here since we're only installing files

# Install directories
install(DIRECTORY
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()