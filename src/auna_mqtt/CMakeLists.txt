cmake_minimum_required(VERSION 3.5)
project(auna_mqtt)

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

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(auna_its_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(PahoMqttCpp REQUIRED)
link_libraries(${PAHO_MQTT_CPP_LIBRARIES})
find_package(nlohmann_json REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Add libraries
add_library(mqtt_waypoint_receiver_node SHARED src/mqtt_waypoint_receiver/mqtt_waypoint_receiver.cpp)

# Create Cpp executable
add_executable(mqtt_waypoint_receiver src/mqtt_waypoint_receiver/mqtt_waypoint_receiver_main.cpp)

target_include_directories(mqtt_waypoint_receiver_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(mqtt_waypoint_receiver_node
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")

# Target dependencies
set(mqtt_waypoint_receiver_dependencies rclcpp geometry_msgs nav2_msgs rclcpp_action rclcpp_components tf2_ros PahoMqttCpp nlohmann_json)
ament_target_dependencies(mqtt_waypoint_receiver_node ${mqtt_waypoint_receiver_dependencies})
ament_target_dependencies(mqtt_waypoint_receiver ${mqtt_waypoint_receiver_dependencies})

# Link libraries
target_link_libraries(mqtt_waypoint_receiver mqtt_waypoint_receiver_node)
target_link_libraries(mqtt_waypoint_receiver PahoMqttCpp::paho-mqttpp3)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Cpp executables
install(TARGETS
  mqtt_waypoint_receiver
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  mqtt_waypoint_receiver_node
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
