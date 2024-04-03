cmake_minimum_required(VERSION 3.5)
project(auna_physical)

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

find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Add libraries
add_library(cmd_vel_to_ackermann_node SHARED src/cmd_vel_to_ackermann/cmd_vel_to_ackermann.cpp)
add_library(vicon_tf_converter_node SHARED src/vicon/vicon_tf_converter.cpp)
add_library(waypoint_publisher_node SHARED src/waypoint_publisher/waypoint_publisher.cpp)
add_library(vesc_start_node SHARED src/vesc_start/vesc_start.cpp)

# Create Cpp executable
add_executable(cmd_vel_to_ackermann src/cmd_vel_to_ackermann/cmd_vel_to_ackermann_main.cpp)
add_executable(vicon_tf_converter src/vicon/vicon_tf_converter_main.cpp)
add_executable(waypoint_publisher src/waypoint_publisher/waypoint_publisher_main.cpp)
add_executable(vesc_start src/vesc_start/vesc_start_main.cpp)

# Target dependencies
set(cmd_vel_to_ackermann_dependencies rclcpp std_srvs geometry_msgs ackermann_msgs)
ament_target_dependencies(cmd_vel_to_ackermann_node ${cmd_vel_to_ackermann_dependencies})
ament_target_dependencies(cmd_vel_to_ackermann ${cmd_vel_to_ackermann_dependencies})
set(vicon_tf_converter_dependencies rclcpp geometry_msgs tf2 tf2_ros)
ament_target_dependencies(vicon_tf_converter_node ${vicon_tf_converter_dependencies})
ament_target_dependencies(vicon_tf_converter ${vicon_tf_converter_dependencies})
set(waypoint_publisher_dependencies rclcpp geometry_msgs nav2_msgs rclcpp_action tf2_ros tf2_geometry_msgs)
ament_target_dependencies(waypoint_publisher_node ${waypoint_publisher_dependencies})
ament_target_dependencies(waypoint_publisher ${waypoint_publisher_dependencies})
set(vesc_start_dependencies rclcpp geometry_msgs nav_msgs)
ament_target_dependencies(vesc_start_node ${waypoint_publisher_dependencies})
ament_target_dependencies(vesc_start ${waypoint_publisher_dependencies})

# Link libraries
target_link_libraries(cmd_vel_to_ackermann cmd_vel_to_ackermann_node)
target_link_libraries(vicon_tf_converter vicon_tf_converter_node)
target_link_libraries(waypoint_publisher waypoint_publisher_node)
target_link_libraries(vesc_start vesc_start_node)

# Install Cpp executables
install(TARGETS
  cmd_vel_to_ackermann
  vicon_tf_converter
  waypoint_publisher
  vesc_start
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  cmd_vel_to_ackermann_node
  vicon_tf_converter_node
  waypoint_publisher_node
  vesc_start_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/curve_fitting.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
