cmake_minimum_required(VERSION 3.5)
project(etsi_its_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AccelerationControl.msg"
  "msg/ActionID.msg"
  "msg/Altitude.msg"
  "msg/BasicVehicleContainerHighFrequency.msg"
  "msg/BasicVehicleContainerLowFrequency.msg"
  "msg/CAM.msg"
  "msg/CauseCode.msg"
  "msg/Curvature.msg"
  "msg/CurvatureCalculationMode.msg"
  "msg/DeltaReferencePosition.msg"
  "msg/DENM.msg"
  "msg/DriveDirection.msg"
  "msg/EventPoint.msg"
  "msg/ExteriorLights.msg"
  "msg/Heading.msg"
  "msg/InformationQuality.msg"
  "msg/ItsPduHeader.msg"
  "msg/LocationContainer.msg"
  "msg/LongitudinalAcceleration.msg"
  "msg/ManagementContainer.msg"
  "msg/PathDeltaTime.msg"
  "msg/PathHistory.msg"
  "msg/PathPoint.msg"
  "msg/PositionConfidenceEllipse.msg"
  "msg/ReferencePosition.msg"
  "msg/RelevanceDistance.msg"
  "msg/RelevanceTrafficDirection.msg"
  "msg/SituationContainer.msg"
  "msg/Speed.msg"
  "msg/StationType.msg"
  "msg/VehicleLength.msg"
  "msg/VehicleRole.msg"
  "msg/VehicleWidth.msg"
  "msg/YawRate.msg"
  DEPENDENCIES builtin_interfaces std_msgs
 )

install(DIRECTORY include/
DESTINATION include/
)
 
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()