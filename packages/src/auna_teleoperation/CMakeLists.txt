cmake_minimum_required(VERSION 3.8)
project(auna_teleoperation)

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

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install executables
install(PROGRAMS
  auna_teleoperation/keyboard_teleop_hold.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_teleop_hold
)

install(PROGRAMS
  auna_teleoperation/teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
