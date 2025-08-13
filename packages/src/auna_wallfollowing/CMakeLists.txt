cmake_minimum_required(VERSION 3.8)
project(auna_wallfollowing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies find_package(ament_cmake REQUIRED) uncomment the following
# section in order to fill in further dependencies manually.
# find_package(<dependency> REQUIRED)

foreach(
  PACKAGE
  ament_cmake
  ament_cmake_python
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs)

  find_package(${PACKAGE} REQUIRED)

endforeach()

add_executable(wallfollowing src/wallfollowing.cpp)
ament_target_dependencies(
  wallfollowing
  rclcpp
  geometry_msgs
  ackermann_msgs
  nav_msgs
  sensor_msgs
  std_msgs)

# Include directory
target_include_directories(wallfollowing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS wallfollowing DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Install config files
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# Install include directories
install(DIRECTORY include/ DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
