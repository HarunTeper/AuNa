cmake_minimum_required(VERSION 3.8)
project(auna_gazebo)

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
find_package(auna_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# Add libraries
add_library(robot_name_publisher_node SHARED src/robot_name_publisher/robot_name_publisher.cpp)

# Create executables
add_executable(robot_name_publisher src/robot_name_publisher/robot_name_publisher_main.cpp)

# Target dependencies
set(robot_name_publisher_dependencies
  rclcpp
  gazebo_msgs
  auna_msgs
)

ament_target_dependencies(robot_name_publisher_node ${robot_name_publisher_dependencies})
ament_target_dependencies(robot_name_publisher ${robot_name_publisher_dependencies})

# Link libraries
target_link_libraries(robot_name_publisher robot_name_publisher_node)

# Install directories
install(DIRECTORY
  launch
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install executables
install(TARGETS
  robot_name_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  robot_name_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
