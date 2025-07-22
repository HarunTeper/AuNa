cmake_minimum_required(VERSION 3.5)
project(auna_tf)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

add_library(global_tf_node SHARED src/global_tf/global_tf.cpp)
add_library(localization_pose_publisher_node SHARED src/localization_pose_publisher/localization_pose_publisher.cpp)

add_executable(global_tf src/global_tf/global_tf_main.cpp)
add_executable(localization_pose_publisher src/localization_pose_publisher/localization_pose_publisher_main.cpp)

set(global_tf_dependencies rclcpp std_msgs geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs gazebo_msgs)
ament_target_dependencies(global_tf_node ${global_tf_dependencies})
ament_target_dependencies(global_tf ${global_tf_dependencies})
set(localization_pose_publisher_dependencies rclcpp std_msgs geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs)
ament_target_dependencies(localization_pose_publisher_node ${localization_pose_publisher_dependencies})
ament_target_dependencies(localization_pose_publisher ${localization_pose_publisher_dependencies})

target_link_libraries(global_tf global_tf_node)
target_link_libraries(localization_pose_publisher localization_pose_publisher_node)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Cpp executables
install(TARGETS
  global_tf
  localization_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  global_tf_node
  localization_pose_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
