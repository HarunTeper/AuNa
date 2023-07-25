cmake_minimum_required(VERSION 3.5)
project(auna_gazebo)

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

find_package(auna_common REQUIRED)
find_package(auna_its_msgs REQUIRED)
find_package(auna_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Add libraries
add_library(gazebo_models_node SHARED src/gazebo_models/gazebo_models.cpp)
add_library(global_tf_node SHARED src/transformations/global_tf/global_tf.cpp)
add_library(localization_pose_node SHARED src/transformations/localization_pose/localization_pose.cpp)
add_library(simulation_pose_node SHARED src/transformations/simulation_pose/simulation_pose.cpp)
add_library(gazebo_pose_node SHARED src/transformations/gazebo_pose/gazebo_pose.cpp)

# Create Cpp executable
add_executable(gazebo_models src/gazebo_models/gazebo_models_main.cpp)
add_executable(global_tf src/transformations/global_tf/global_tf_main.cpp)
add_executable(localization_pose src/transformations/localization_pose/localization_pose_main.cpp)
add_executable(simulation_pose src/transformations/simulation_pose/simulation_pose_main.cpp)
add_executable(gazebo_pose src/transformations/gazebo_pose/gazebo_pose_main.cpp)

# Target dependencies
set(gazebo_models_dependencies rclcpp gazebo_msgs auna_msgs)
ament_target_dependencies(gazebo_models_node ${gazebo_models_dependencies})
ament_target_dependencies(gazebo_models ${gazebo_models_dependencies})
set(global_tf_dependencies rclcpp std_msgs gazebo_msgs geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs)
ament_target_dependencies(global_tf_node ${global_tf_dependencies})
ament_target_dependencies(global_tf ${global_tf_dependencies})
set(localization_pose_dependencies rclcpp std_msgs geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs)
ament_target_dependencies(localization_pose_node ${localization_pose_dependencies})
ament_target_dependencies(localization_pose ${localization_pose_dependencies})
set(simulation_pose_dependencies rclcpp std_msgs gazebo_msgs auna_its_msgs tf2)
ament_target_dependencies(simulation_pose_node ${simulation_pose_dependencies})
ament_target_dependencies(simulation_pose ${simulation_pose_dependencies})
set(gazebo_pose_dependencies rclcpp std_msgs geometry_msgs gazebo_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(gazebo_pose_node ${gazebo_pose_dependencies})
ament_target_dependencies(gazebo_pose ${gazebo_pose_dependencies})

# Link libraries
target_link_libraries(gazebo_models gazebo_models_node)
target_link_libraries(global_tf global_tf_node)
target_link_libraries(localization_pose localization_pose_node)
target_link_libraries(simulation_pose simulation_pose_node)
target_link_libraries(gazebo_pose gazebo_pose_node)

install(DIRECTORY 
  config
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Cpp executables
install(TARGETS
  gazebo_models
  global_tf
  localization_pose
  simulation_pose
  gazebo_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  gazebo_models_node
  global_tf_node
  localization_pose_node
  simulation_pose_node
  gazebo_pose_node
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
