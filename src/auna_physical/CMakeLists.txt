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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Add libraries
add_library(cmd_vel_to_ackermann_node SHARED src/converter/cmd_vel_to_ackermann.cpp)

# Create Cpp executable
add_executable(cmd_vel_to_ackermann src/converter/cmd_vel_to_ackermann_main.cpp)

# Target dependencies
set(cmd_vel_to_ackermann_dependencies rclcpp std_msgs geometry_msgs ackermann_msgs)
ament_target_dependencies(cmd_vel_to_ackermann_node ${cmd_vel_to_ackermann_dependencies})
ament_target_dependencies(cmd_vel_to_ackermann ${cmd_vel_to_ackermann_dependencies})

# Link libraries
target_link_libraries(cmd_vel_to_ackermann cmd_vel_to_ackermann_node)

# Install Cpp executables
install(TARGETS
  cmd_vel_to_ackermann
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  cmd_vel_to_ackermann_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()