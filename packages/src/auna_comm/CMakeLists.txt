cmake_minimum_required(VERSION 3.5)
project(auna_comm)

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

# Replace auna_its_msgs with etsi_its_cam_msgs and etsi_its_msgs_utils
find_package(etsi_its_cam_msgs REQUIRED)
find_package(etsi_its_msgs_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Add libraries
add_library(cam_communication_node SHARED src/cam_communication/cam_communication.cpp)
add_library(cam_receiver_node SHARED src/cam_receiver/cam_receiver.cpp)
include_directories(include)
# Create Cpp executable
add_executable(cam_communication src/cam_communication/cam_communication_main.cpp)
add_executable(cam_receiver src/cam_receiver/cam_receiver_main.cpp)
# Target dependencies
# Updated dependencies to include etsi_its_cam_msgs and etsi_its_msgs_utils
set(cam_communication_dependencies rclcpp std_msgs etsi_its_cam_msgs etsi_its_msgs_utils nav_msgs geometry_msgs tf2)
ament_target_dependencies(cam_communication_node ${cam_communication_dependencies})
ament_target_dependencies(cam_communication ${cam_communication_dependencies})
ament_target_dependencies(cam_receiver_node ${cam_communication_dependencies})
ament_target_dependencies(cam_receiver ${cam_communication_dependencies})


# Link libraries
target_link_libraries(cam_communication cam_communication_node)
target_link_libraries(cam_receiver cam_receiver_node)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Cpp executables
install(TARGETS
  cam_communication
  cam_receiver
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  cam_communication_node
  cam_receiver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()