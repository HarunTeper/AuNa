cmake_minimum_required(VERSION 3.5)
project(auna_f110)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(auna_common REQUIRED)
find_package(teleop_twist_joy REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Add libraries
add_library(vesc_start_node SHARED src/vesc_start/vesc_start.cpp)

# Create Cpp executable
add_executable(vesc_start src/vesc_start/vesc_start_main.cpp)

# Target dependencies
set(vesc_start_dependencies rclcpp geometry_msgs nav_msgs)
ament_target_dependencies(vesc_start_node ${vesc_start_dependencies})
ament_target_dependencies(vesc_start ${vesc_start_dependencies})

# Link libraries
target_link_libraries(vesc_start vesc_start_node)

# Install Cpp executables
install(TARGETS
  vesc_start
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  vesc_start_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY 
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
