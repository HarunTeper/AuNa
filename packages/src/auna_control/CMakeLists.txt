cmake_minimum_required(VERSION 3.8)
project(auna_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED) # Add for component support
find_package(geometry_msgs REQUIRED)
find_package(auna_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Add C++17 support which includes better parameter handling
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(cmd_vel_multiplexer_node src/cmd_vel_multiplexer_node.cpp)
ament_target_dependencies(cmd_vel_multiplexer_node
  rclcpp
  rclcpp_components
  geometry_msgs
  auna_msgs
  std_srvs
)

install(TARGETS cmd_vel_multiplexer_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
