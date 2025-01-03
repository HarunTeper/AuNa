cmake_minimum_required(VERSION 3.5)
project(auna_gazebo)

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

find_package(auna_common REQUIRED)
find_package(auna_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(etsi_its_msgs REQUIRED)
find_package(etsi_its_msgs_utils REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Add libraries
add_library(robot_name_publisher_node SHARED src/robot_name_publisher/robot_name_publisher.cpp)
add_library(global_tf_node SHARED src/transformations/global_tf/global_tf.cpp)
add_library(localization_pose_publisher_node SHARED src/transformations/localization_pose_publisher/localization_pose_publisher.cpp)
add_library(ground_truth_cam_node SHARED src/transformations/ground_truth_cam/ground_truth_cam.cpp)
add_library(ground_truth_localization_node SHARED src/transformations/ground_truth_localization/ground_truth_localization.cpp)

# Create Cpp executable
add_executable(robot_name_publisher src/robot_name_publisher/robot_name_publisher_main.cpp)
add_executable(global_tf src/transformations/global_tf/global_tf_main.cpp)
add_executable(localization_pose_publisher src/transformations/localization_pose_publisher/localization_pose_publisher_main.cpp)
add_executable(ground_truth_cam src/transformations/ground_truth_cam/ground_truth_cam_main.cpp)
add_executable(ground_truth_localization 
  src/transformations/ground_truth_localization/ground_truth_localization.cpp
  src/transformations/ground_truth_localization/ground_truth_localization_main.cpp)

# Target dependencies
set(robot_name_publisher_dependencies rclcpp gazebo_msgs auna_msgs)
ament_target_dependencies(robot_name_publisher_node ${robot_name_publisher_dependencies})
ament_target_dependencies(robot_name_publisher ${robot_name_publisher_dependencies})
set(global_tf_dependencies rclcpp std_msgs gazebo_msgs geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs)
ament_target_dependencies(global_tf_node ${global_tf_dependencies})
ament_target_dependencies(global_tf ${global_tf_dependencies})
set(localization_pose_publisher_dependencies rclcpp std_msgs geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs)
ament_target_dependencies(localization_pose_publisher_node ${localization_pose_publisher_dependencies})
ament_target_dependencies(localization_pose_publisher ${localization_pose_publisher_dependencies})
set(ground_truth_cam_dependencies rclcpp std_msgs gazebo_msgs  tf2 etsi_its_msgs etsi_its_msgs_utils)
ament_target_dependencies(ground_truth_cam_node ${ground_truth_cam_dependencies})
ament_target_dependencies(ground_truth_cam ${ground_truth_cam_dependencies})
set(ground_truth_localization_dependencies rclcpp std_msgs geometry_msgs gazebo_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(ground_truth_localization_node ${ground_truth_localization_dependencies})
ament_target_dependencies(ground_truth_localization ${ground_truth_localization_dependencies})

# Link libraries
target_link_libraries(robot_name_publisher robot_name_publisher_node)
target_link_libraries(global_tf global_tf_node)
target_link_libraries(localization_pose_publisher localization_pose_publisher_node)
target_link_libraries(ground_truth_cam ground_truth_cam_node)
target_link_libraries(ground_truth_localization ground_truth_localization_node)

install(DIRECTORY 
  config
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Cpp executables
install(TARGETS
  robot_name_publisher
  global_tf
  localization_pose_publisher
  ground_truth_cam
  ground_truth_localization
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  robot_name_publisher_node
  global_tf_node
  localization_pose_publisher_node
  ground_truth_cam_node
  ground_truth_localization
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
