cmake_minimum_required(VERSION 3.5)
project(auna_pure_pursuit)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Add libraries
add_library(auna_pure_pursuit_controller SHARED 
  src/auna_pure_pursuit_controller/auna_pure_pursuit_controller.cpp
  src/auna_pure_pursuit_controller/collision_checker.cpp
  src/auna_pure_pursuit_controller/path_handler.cpp
  src/auna_pure_pursuit_controller/parameter_handler.cpp
)

# Target dependencies
set(auna_pure_pursuit_controller_dependencies rclcpp geometry_msgs nav2_costmap_2d pluginlib nav_msgs nav2_util nav2_core tf2 tf2_geometry_msgs)
ament_target_dependencies(auna_pure_pursuit_controller ${auna_pure_pursuit_controller_dependencies})

install(TARGETS
  auna_pure_pursuit_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_core auna_pure_pursuit_controller.xml)

ament_package()

