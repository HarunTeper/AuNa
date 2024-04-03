cmake_minimum_required(VERSION 3.5)
project(auna_omnet)

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

find_package(auna_its_msgs REQUIRED)
find_package(auna_msgs REQUIRED)
find_package(etsi_its_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(std_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Add libraries
add_library(omnet_transmitter_node SHARED src/omnet/omnet_transmitter/omnet_transmitter.cpp)
add_library(omnet_receiver_node SHARED src/omnet/omnet_receiver/omnet_receiver.cpp)
add_library(omnet_cam_filter_node SHARED src/omnet/omnet_cam_filter/omnet_cam_filter.cpp)

# Create Cpp executable
add_executable(omnet_transmitter src/omnet/omnet_transmitter/omnet_transmitter_main.cpp)
add_executable(omnet_receiver src/omnet/omnet_receiver/omnet_receiver_main.cpp)
add_executable(omnet_cam_filter src/omnet/omnet_cam_filter/omnet_cam_filter_main.cpp)

# Target dependencies
set(omnet_transmitter_dependencies rclcpp geometry_msgs nav_msgs etsi_its_msgs tf2)
ament_target_dependencies(omnet_transmitter_node ${omnet_transmitter_dependencies})
ament_target_dependencies(omnet_transmitter ${omnet_transmitter_dependencies})
set(omnet_receiver_dependencies rclcpp etsi_its_msgs auna_its_msgs)
ament_target_dependencies(omnet_receiver_node ${omnet_receiver_dependencies})
ament_target_dependencies(omnet_receiver ${omnet_receiver_dependencies})
set(omnet_cam_filter_dependencies rclcpp auna_its_msgs auna_msgs)
ament_target_dependencies(omnet_cam_filter_node ${omnet_cam_filter_dependencies})
ament_target_dependencies(omnet_cam_filter ${omnet_cam_filter_dependencies})

# Link libraries
target_link_libraries(omnet_transmitter omnet_transmitter_node)
target_link_libraries(omnet_receiver omnet_receiver_node)
target_link_libraries(omnet_cam_filter omnet_cam_filter_node)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Cpp executables
install(TARGETS
  omnet_transmitter
  omnet_receiver
  omnet_cam_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  omnet_transmitter_node
  omnet_receiver_node
  omnet_cam_filter_node
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
