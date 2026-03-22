cmake_minimum_required(VERSION 3.10)
project(sfm)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# SFM Algorithm library
add_library(sfm_algorithm_lib
  src/sfm_algorithm.cpp
)
ament_target_dependencies(sfm_algorithm_lib
  nav2_costmap_2d
)

# SFM Controller plugin (must be shared for pluginlib)
add_library(sfm_controller_plugin SHARED
  src/sfm_controller.cpp
)
ament_target_dependencies(sfm_controller_plugin
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav2_msgs
  pluginlib
  visualization_msgs
  tf2_ros
  tf2
  std_msgs
)
target_link_libraries(sfm_controller_plugin
  sfm_algorithm_lib
)

# Install libraries
install(TARGETS
  sfm_algorithm_lib
  sfm_controller_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Export plugin description file (required by pluginlib)
pluginlib_export_plugin_description_file(nav2_core sfm_controller_plugin.xml)

# Install plugin description file
install(FILES sfm_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install config
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install docs
install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
