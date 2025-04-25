cmake_minimum_required(VERSION 3.8)
project(auna_rviz_plugins)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)

# Find Qt5 packages
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Tell CMake to run Qt's moc when necessary
set(CMAKE_AUTOMOC ON)

set(auna_rviz_plugins_headers_to_moc
  include/auna_rviz_plugins/cmd_vel_mux_panel.hpp
  include/auna_rviz_plugins/global_emergency_stop_panel.hpp
)

foreach(header "${auna_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(auna_rviz_plugins_moc_files "${header}")
endforeach()

set(auna_rviz_plugins_source_files
  src/cmd_vel_mux_panel.cpp
  src/global_emergency_stop_panel.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${auna_rviz_plugins_moc_files}
  ${auna_rviz_plugins_source_files}
)

# Now that target exists, set include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Specify dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  geometry_msgs
  pluginlib
  std_srvs
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Core
  rviz_common::rviz_common
  pluginlib::pluginlib
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install includes
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rviz_common
  geometry_msgs
  pluginlib
  std_srvs
  Qt5
)

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export plugin description
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_package()