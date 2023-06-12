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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_srvs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Add libraries
add_library(auna_rviz_plugins SHARED 
    src/control_panel/control_panel.cpp
)

# Target dependencies
set(auna_rviz_plugins_dependencies rclcpp pluginlib rviz_common std_srvs)
ament_target_dependencies(auna_rviz_plugins ${auna_rviz_plugins_dependencies})

install(TARGETS auna_rviz_plugins
  EXPORT auna_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_package()

