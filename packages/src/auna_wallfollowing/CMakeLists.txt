cmake_minimum_required(VERSION 3.8)
project(auna_wallfollowing)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Create executable
add_executable(wallfollowing src/wallfollowing.cpp)

# Target dependencies
ament_target_dependencies(wallfollowing
  rclcpp
  geometry_msgs
  ackermann_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

# Include directories
target_include_directories(wallfollowing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install executables
install(TARGETS 
  wallfollowing 
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY 
  launch/ 
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY 
  config/ 
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY 
  include/ 
  DESTINATION include/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
