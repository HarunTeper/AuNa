
cmake_minimum_required(VERSION 3.5)
project(auna_ground_truth)


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
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(etsi_its_cam_msgs REQUIRED)
find_package(etsi_its_msgs_utils REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Add libraries (node implementations)
add_library(ground_truth_cam_node SHARED src/ground_truth_cam/ground_truth_cam.cpp)
add_library(ground_truth_pose_publisher_node SHARED src/ground_truth_pose_publisher/ground_truth_pose_publisher.cpp)
add_library(ground_truth_transform_node SHARED src/ground_truth_transform/ground_truth_transform.cpp)

# Add executables (main files)
add_executable(ground_truth_cam_main src/ground_truth_cam/ground_truth_cam_main.cpp)
add_executable(ground_truth_pose_publisher_main src/ground_truth_pose_publisher/ground_truth_pose_publisher_main.cpp)
add_executable(ground_truth_transform_main src/ground_truth_transform/ground_truth_transform_main.cpp)

# Target dependencies
set(ground_truth_cam_dependencies rclcpp gazebo_msgs etsi_its_cam_msgs tf2 etsi_its_msgs_utils)
ament_target_dependencies(ground_truth_cam_node ${ground_truth_cam_dependencies})
ament_target_dependencies(ground_truth_cam_main ${ground_truth_cam_dependencies})

set(ground_truth_pose_publisher_dependencies rclcpp geometry_msgs tf2_ros nav_msgs tf2_geometry_msgs)
ament_target_dependencies(ground_truth_pose_publisher_node ${ground_truth_pose_publisher_dependencies})
ament_target_dependencies(ground_truth_pose_publisher_main ${ground_truth_pose_publisher_dependencies})

set(ground_truth_transform_dependencies rclcpp gazebo_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(ground_truth_transform_node ${ground_truth_transform_dependencies})
ament_target_dependencies(ground_truth_transform_main ${ground_truth_transform_dependencies})

# Link libraries
target_link_libraries(ground_truth_cam_main ground_truth_cam_node)
target_link_libraries(ground_truth_pose_publisher_main ground_truth_pose_publisher_node)
target_link_libraries(ground_truth_transform_main ground_truth_transform_node)

# Install Cpp executables
install(TARGETS
  ground_truth_cam_main
  ground_truth_pose_publisher_main
  ground_truth_transform_main
  DESTINATION lib/${PROJECT_NAME}
)


# Install Cpp libraries
install(TARGETS
  ground_truth_cam_node
  ground_truth_pose_publisher_node
  ground_truth_transform_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


ament_package()
