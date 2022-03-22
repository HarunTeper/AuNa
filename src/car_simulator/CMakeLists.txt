cmake_minimum_required(VERSION 3.5)
project(car_simulator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_its_msgs REQUIRED)
find_package(etsi_its_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)


add_executable(localization_pose src/transformations/localization_pose.cpp)
ament_target_dependencies(localization_pose rclcpp std_msgs geometry_msgs nav_msgs tf2_ros tf2_kdl)

add_executable(simulation_pose src/transformations/simulation_pose.cpp)
ament_target_dependencies(simulation_pose rclcpp std_msgs geometry_msgs nav_msgs gazebo_msgs ros_its_msgs)

add_executable(globalTF src/transformations/global_tf.cpp)
ament_target_dependencies(globalTF rclcpp std_msgs gazebo_msgs geometry_msgs tf2_ros tf2_kdl)

add_executable(arteryInfo src/omnet/artery_info.cpp)
ament_target_dependencies(arteryInfo rclcpp std_msgs gazebo_msgs)

add_executable(omnetTX src/omnet/omnetTX.cpp)
ament_target_dependencies(omnetTX rclcpp std_msgs geometry_msgs nav_msgs etsi_its_msgs ros_its_msgs)

add_executable(omnetRX src/omnet/omnetRX.cpp)
ament_target_dependencies(omnetRX rclcpp std_msgs etsi_its_msgs ros_its_msgs)

install(TARGETS
  localization_pose
  simulation_pose
  arteryInfo
  globalTF
  omnetTX
  omnetRX
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  config
  launch
  models
  rviz
  worlds
  maps
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/teleoperation/teleop_node.py
  scripts/waypoint_publisher.py
  scripts/tf_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
