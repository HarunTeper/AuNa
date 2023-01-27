cmake_minimum_required(VERSION 3.5)
project(laser_proc)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(laser_proc SHARED
  src/laser_proc.cpp
  src/laser_publisher.cpp
)
target_include_directories(laser_proc
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(laser_proc
  rclcpp
  sensor_msgs
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "LASER_PROC_BUILDING_LIBRARY")

## Laser Proc component
add_library(laser_proc_component SHARED src/laser_proc_component.cpp)
target_link_libraries(laser_proc_component laser_proc)
ament_target_dependencies(laser_proc_component
  class_loader
)
rclcpp_components_register_node(laser_proc_component
  PLUGIN "laser_proc::LaserProcComponent"
  EXECUTABLE laser_proc_node
)

install(TARGETS laser_proc laser_proc_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(include)
ament_export_libraries(laser_proc)

ament_package()
