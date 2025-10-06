cmake_minimum_required(VERSION 3.8)
project(auna_waypoints)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include)

# Add libraries
add_library(nav2_waypoint_publisher_node SHARED
            src/nav2_waypoint_publisher/nav2_waypoint_publisher.cpp)

# Create executables
add_executable(nav2_waypoint_publisher
                src/nav2_waypoint_publisher/nav2_waypoint_publisher_main.cpp)

# Target dependencies
set(nav2_waypoint_publisher_dependencies
    rclcpp geometry_msgs nav2_msgs rclcpp_action tf2_ros tf2_geometry_msgs)

ament_target_dependencies(nav2_waypoint_publisher_node
                          ${nav2_waypoint_publisher_dependencies})
ament_target_dependencies(nav2_waypoint_publisher
                          ${nav2_waypoint_publisher_dependencies})

# Link libraries
target_link_libraries(nav2_waypoint_publisher nav2_waypoint_publisher_node)
target_link_libraries(nav2_waypoint_publisher_node yaml-cpp)

# Install directories
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Install executables
install(TARGETS nav2_waypoint_publisher DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/curve_fitting.py DESTINATION lib/${PROJECT_NAME})

# Install libraries
install(
  TARGETS nav2_waypoint_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
