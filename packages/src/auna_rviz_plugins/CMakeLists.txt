cmake_minimum_required(VERSION 3.5)
project(auna_rviz_plugins)

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
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
# find_package(std_srvs REQUIRED) # No longer needed
find_package(geometry_msgs REQUIRED)

# Find Qt5
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Include Cpp "include" directory
include_directories(include)

# Enable Qt's Meta-Object Compiler
set(CMAKE_AUTOMOC ON)

# Add libraries for each plugin
add_library(cmd_vel_mux_panel SHARED
    src/cmd_vel_mux_panel.cpp
    # Add header if needed for AUTOMOC, usually not required if Q_OBJECT is in header
)
add_library(global_emergency_stop_panel SHARED
    # src/global_emergency_stop_panel.cpp # Source file to be added later
)

# Target dependencies
ament_target_dependencies(cmd_vel_mux_panel PUBLIC
    rclcpp
    pluginlib
    rviz_common
    geometry_msgs
)
target_link_libraries(cmd_vel_mux_panel PUBLIC Qt5::Widgets)

ament_target_dependencies(global_emergency_stop_panel PUBLIC
    rclcpp
    pluginlib
    rviz_common
    geometry_msgs
)
target_link_libraries(global_emergency_stop_panel PUBLIC Qt5::Widgets)

# Install both libraries
install(TARGETS cmd_vel_mux_panel global_emergency_stop_panel
  EXPORT export_${PROJECT_NAME} # Use a unique export name
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export targets for other packages to find
install(EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_package()

