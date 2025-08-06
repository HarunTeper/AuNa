cmake_minimum_required(VERSION 3.8)
project(auna_control)

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
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(auna_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)

# Find Qt5 packages
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

# --- cmd_vel_multiplexer_node ---
add_executable(cmd_vel_multiplexer_node src/cmd_vel_multiplexer_node.cpp src/cmd_vel_multiplexer_main.cpp)
target_include_directories(cmd_vel_multiplexer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cmd_vel_multiplexer_node
  rclcpp
  rclcpp_components
  geometry_msgs
  ackermann_msgs
  auna_msgs
  std_srvs
  yaml-cpp
)
target_link_libraries(cmd_vel_multiplexer_node yaml-cpp)
install(TARGETS cmd_vel_multiplexer_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- RViz Plugin ---
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

set(auna_control_headers_to_moc
  include/auna_control/control_panel.hpp
  include/auna_control/control_panel_ros_interface.hpp
)

foreach(header "${auna_control_headers_to_moc}")
  qt5_wrap_cpp(auna_control_moc_files "${header}")
endforeach()

set(auna_control_source_files
  src/control_panel.cpp
  src/control_panel_ros_interface.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${auna_control_moc_files}
  ${auna_control_source_files}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  geometry_msgs
  ackermann_msgs
  nav_msgs
  sensor_msgs
  pluginlib
  std_srvs
  auna_msgs
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Core
  rviz_common::rviz_common
)

# Install plugin library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install includes
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}/
)

# Export plugin description
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# --- General ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
