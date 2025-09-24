cmake_minimum_required(VERSION 3.8)
project(auna_template)

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
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Add libraries
add_library(sample_node_lib SHARED src/sample_node/sample_node.cpp)

# Create executables
add_executable(sample_node src/sample_node/sample_node_main.cpp)

# Target dependencies
set(sample_node_dependencies 
  rclcpp 
  std_msgs
)

ament_target_dependencies(sample_node_lib ${sample_node_dependencies})

# Link libraries
target_link_libraries(sample_node sample_node_lib)

# Install directories
install(DIRECTORY 
  config 
  launch 
  DESTINATION share/${PROJECT_NAME}
)

# Install executables
install(TARGETS 
  sample_node 
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(
  TARGETS sample_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
