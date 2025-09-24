cmake_minimum_required(VERSION 3.8)
project(auna_cacc)

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
find_package(etsi_its_cam_msgs REQUIRED)
find_package(etsi_its_msgs_utils REQUIRED)
find_package(auna_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Include directories
include_directories(include)

# Add libraries
add_library(cacc_controller_node SHARED src/cacc_controller/cacc_controller.cpp)

# Create executables
add_executable(cacc_controller src/cacc_controller/cacc_controller_main.cpp)

# Target dependencies
set(cacc_controller_dependencies 
  rclcpp 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  etsi_its_cam_msgs 
  etsi_its_msgs_utils 
  auna_msgs 
  rcl_interfaces 
  tf2
)

ament_target_dependencies(cacc_controller_node ${cacc_controller_dependencies})
ament_target_dependencies(cacc_controller ${cacc_controller_dependencies})

# Link libraries
target_link_libraries(cacc_controller cacc_controller_node)

# Install directories
install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install executables
install(TARGETS
  cacc_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS 
  cacc_controller_node
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
