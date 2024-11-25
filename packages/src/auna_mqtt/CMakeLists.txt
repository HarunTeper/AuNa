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

find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(PahoMqttCpp REQUIRED)
link_libraries(${PAHO_MQTT_CPP_LIBRARIES})
find_package(nlohmann_json REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Add static libraries
add_library(mqtt_callback STATIC src/mqtt_callback/mqtt_callback.cpp)
set_target_properties( mqtt_callback PROPERTIES PREFIX lib )
target_link_libraries( mqtt_callback mqtt_waypoint_receiver_node )

# Add libraries
add_library(mqtt_waypoint_receiver_node STATIC src/mqtt_waypoint_receiver/mqtt_waypoint_receiver.cpp)

# Create Cpp executable
add_executable(mqtt_waypoint_receiver src/mqtt_waypoint_receiver/mqtt_waypoint_receiver_main.cpp)

target_include_directories(mqtt_waypoint_receiver_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(mqtt_waypoint_receiver_node
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")

# Target dependencies
set(mqtt_waypoint_receiver_dependencies rclcpp nav2_msgs rclcpp_action tf2_ros tf2_geometry_msgs PahoMqttCpp nlohmann_json)
ament_target_dependencies(mqtt_waypoint_receiver_node ${mqtt_waypoint_receiver_dependencies})
ament_target_dependencies(mqtt_waypoint_receiver ${mqtt_waypoint_receiver_dependencies})
set(mqtt_callback_dependencies rclcpp PahoMqttCpp nlohmann_json)
ament_target_dependencies(mqtt_callback ${mqtt_callback_dependencies})

# Link libraries
target_link_libraries(mqtt_waypoint_receiver mqtt_waypoint_receiver_node)
target_link_libraries(mqtt_waypoint_receiver_node mqtt_callback)
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
  mqtt_callback
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
