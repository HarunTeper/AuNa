#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_auna_its_msgs_CAM and auna_its_msgs::msg::CAM

void convertFromBus(auna_its_msgs::msg::CAM& msgPtr, SL_Bus_auna_its_msgs_CAM const* busPtr)
{
  const std::string rosMessageType("auna_its_msgs/CAM");

  msgPtr.curv =  busPtr->curv;
  msgPtr.drive_direction =  busPtr->drive_direction;
  convertFromBus(msgPtr.header, &busPtr->header);
  convertFromBusVariablePrimitiveArray(msgPtr.robot_name, busPtr->robot_name, busPtr->robot_name_SL_Info);
  msgPtr.theta =  busPtr->theta;
  msgPtr.thetadot =  busPtr->thetadot;
  msgPtr.v =  busPtr->v;
  msgPtr.vdot =  busPtr->vdot;
  msgPtr.vehicle_length =  busPtr->vehicle_length;
  msgPtr.vehicle_width =  busPtr->vehicle_width;
  msgPtr.x =  busPtr->x;
  msgPtr.y =  busPtr->y;
  msgPtr.z =  busPtr->z;
}

void convertToBus(SL_Bus_auna_its_msgs_CAM* busPtr, const auna_its_msgs::msg::CAM& msgPtr)
{
  const std::string rosMessageType("auna_its_msgs/CAM");

  busPtr->curv =  msgPtr.curv;
  busPtr->drive_direction =  msgPtr.drive_direction;
  convertToBus(&busPtr->header, msgPtr.header);
  convertToBusVariablePrimitiveArray(busPtr->robot_name, busPtr->robot_name_SL_Info, msgPtr.robot_name, slros::EnabledWarning(rosMessageType, "robot_name"));
  busPtr->theta =  msgPtr.theta;
  busPtr->thetadot =  msgPtr.thetadot;
  busPtr->v =  msgPtr.v;
  busPtr->vdot =  msgPtr.vdot;
  busPtr->vehicle_length =  msgPtr.vehicle_length;
  busPtr->vehicle_width =  msgPtr.vehicle_width;
  busPtr->x =  msgPtr.x;
  busPtr->y =  msgPtr.y;
  busPtr->z =  msgPtr.z;
}


// Conversions between SL_Bus_builtin_interfaces_Time and builtin_interfaces::msg::Time

void convertFromBus(builtin_interfaces::msg::Time& msgPtr, SL_Bus_builtin_interfaces_Time const* busPtr)
{
  const std::string rosMessageType("builtin_interfaces/Time");

  msgPtr.nanosec =  busPtr->nanosec;
  msgPtr.sec =  busPtr->sec;
}

void convertToBus(SL_Bus_builtin_interfaces_Time* busPtr, const builtin_interfaces::msg::Time& msgPtr)
{
  const std::string rosMessageType("builtin_interfaces/Time");

  busPtr->nanosec =  msgPtr.nanosec;
  busPtr->sec =  msgPtr.sec;
}


// Conversions between SL_Bus_example_interfaces_Float64 and example_interfaces::msg::Float64

void convertFromBus(example_interfaces::msg::Float64& msgPtr, SL_Bus_example_interfaces_Float64 const* busPtr)
{
  const std::string rosMessageType("example_interfaces/Float64");

  msgPtr.data =  busPtr->data;
}

void convertToBus(SL_Bus_example_interfaces_Float64* busPtr, const example_interfaces::msg::Float64& msgPtr)
{
  const std::string rosMessageType("example_interfaces/Float64");

  busPtr->data =  msgPtr.data;
}


// Conversions between SL_Bus_geometry_msgs_Point and geometry_msgs::msg::Point

void convertFromBus(geometry_msgs::msg::Point& msgPtr, SL_Bus_geometry_msgs_Point const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  msgPtr.x =  busPtr->x;
  msgPtr.y =  busPtr->y;
  msgPtr.z =  busPtr->z;
}

void convertToBus(SL_Bus_geometry_msgs_Point* busPtr, const geometry_msgs::msg::Point& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  busPtr->x =  msgPtr.x;
  busPtr->y =  msgPtr.y;
  busPtr->z =  msgPtr.z;
}


// Conversions between SL_Bus_geometry_msgs_Pose and geometry_msgs::msg::Pose

void convertFromBus(geometry_msgs::msg::Pose& msgPtr, SL_Bus_geometry_msgs_Pose const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  convertFromBus(msgPtr.orientation, &busPtr->orientation);
  convertFromBus(msgPtr.position, &busPtr->position);
}

void convertToBus(SL_Bus_geometry_msgs_Pose* busPtr, const geometry_msgs::msg::Pose& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  convertToBus(&busPtr->orientation, msgPtr.orientation);
  convertToBus(&busPtr->position, msgPtr.position);
}


// Conversions between SL_Bus_geometry_msgs_PoseStamped and geometry_msgs::msg::PoseStamped

void convertFromBus(geometry_msgs::msg::PoseStamped& msgPtr, SL_Bus_geometry_msgs_PoseStamped const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseStamped");

  convertFromBus(msgPtr.header, &busPtr->header);
  convertFromBus(msgPtr.pose, &busPtr->pose);
}

void convertToBus(SL_Bus_geometry_msgs_PoseStamped* busPtr, const geometry_msgs::msg::PoseStamped& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseStamped");

  convertToBus(&busPtr->header, msgPtr.header);
  convertToBus(&busPtr->pose, msgPtr.pose);
}


// Conversions between SL_Bus_geometry_msgs_PoseWithCovariance and geometry_msgs::msg::PoseWithCovariance

void convertFromBus(geometry_msgs::msg::PoseWithCovariance& msgPtr, SL_Bus_geometry_msgs_PoseWithCovariance const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseWithCovariance");

  convertFromBusFixedPrimitiveArray(msgPtr.covariance, busPtr->covariance);
  convertFromBus(msgPtr.pose, &busPtr->pose);
}

void convertToBus(SL_Bus_geometry_msgs_PoseWithCovariance* busPtr, const geometry_msgs::msg::PoseWithCovariance& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseWithCovariance");

  convertToBusFixedPrimitiveArray(busPtr->covariance, msgPtr.covariance, slros::NoopWarning());
  convertToBus(&busPtr->pose, msgPtr.pose);
}


// Conversions between SL_Bus_geometry_msgs_Quaternion and geometry_msgs::msg::Quaternion

void convertFromBus(geometry_msgs::msg::Quaternion& msgPtr, SL_Bus_geometry_msgs_Quaternion const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  msgPtr.w =  busPtr->w;
  msgPtr.x =  busPtr->x;
  msgPtr.y =  busPtr->y;
  msgPtr.z =  busPtr->z;
}

void convertToBus(SL_Bus_geometry_msgs_Quaternion* busPtr, const geometry_msgs::msg::Quaternion& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  busPtr->w =  msgPtr.w;
  busPtr->x =  msgPtr.x;
  busPtr->y =  msgPtr.y;
  busPtr->z =  msgPtr.z;
}


// Conversions between SL_Bus_geometry_msgs_Twist and geometry_msgs::msg::Twist

void convertFromBus(geometry_msgs::msg::Twist& msgPtr, SL_Bus_geometry_msgs_Twist const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Twist");

  convertFromBus(msgPtr.angular, &busPtr->angular);
  convertFromBus(msgPtr.linear, &busPtr->linear);
}

void convertToBus(SL_Bus_geometry_msgs_Twist* busPtr, const geometry_msgs::msg::Twist& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Twist");

  convertToBus(&busPtr->angular, msgPtr.angular);
  convertToBus(&busPtr->linear, msgPtr.linear);
}


// Conversions between SL_Bus_geometry_msgs_TwistWithCovariance and geometry_msgs::msg::TwistWithCovariance

void convertFromBus(geometry_msgs::msg::TwistWithCovariance& msgPtr, SL_Bus_geometry_msgs_TwistWithCovariance const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/TwistWithCovariance");

  convertFromBusFixedPrimitiveArray(msgPtr.covariance, busPtr->covariance);
  convertFromBus(msgPtr.twist, &busPtr->twist);
}

void convertToBus(SL_Bus_geometry_msgs_TwistWithCovariance* busPtr, const geometry_msgs::msg::TwistWithCovariance& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/TwistWithCovariance");

  convertToBusFixedPrimitiveArray(busPtr->covariance, msgPtr.covariance, slros::NoopWarning());
  convertToBus(&busPtr->twist, msgPtr.twist);
}


// Conversions between SL_Bus_geometry_msgs_Vector3 and geometry_msgs::msg::Vector3

void convertFromBus(geometry_msgs::msg::Vector3& msgPtr, SL_Bus_geometry_msgs_Vector3 const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  msgPtr.x =  busPtr->x;
  msgPtr.y =  busPtr->y;
  msgPtr.z =  busPtr->z;
}

void convertToBus(SL_Bus_geometry_msgs_Vector3* busPtr, const geometry_msgs::msg::Vector3& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  busPtr->x =  msgPtr.x;
  busPtr->y =  msgPtr.y;
  busPtr->z =  msgPtr.z;
}


// Conversions between SL_Bus_nav_msgs_Odometry and nav_msgs::msg::Odometry

void convertFromBus(nav_msgs::msg::Odometry& msgPtr, SL_Bus_nav_msgs_Odometry const* busPtr)
{
  const std::string rosMessageType("nav_msgs/Odometry");

  convertFromBusVariablePrimitiveArray(msgPtr.child_frame_id, busPtr->child_frame_id, busPtr->child_frame_id_SL_Info);
  convertFromBus(msgPtr.header, &busPtr->header);
  convertFromBus(msgPtr.pose, &busPtr->pose);
  convertFromBus(msgPtr.twist, &busPtr->twist);
}

void convertToBus(SL_Bus_nav_msgs_Odometry* busPtr, const nav_msgs::msg::Odometry& msgPtr)
{
  const std::string rosMessageType("nav_msgs/Odometry");

  convertToBusVariablePrimitiveArray(busPtr->child_frame_id, busPtr->child_frame_id_SL_Info, msgPtr.child_frame_id, slros::EnabledWarning(rosMessageType, "child_frame_id"));
  convertToBus(&busPtr->header, msgPtr.header);
  convertToBus(&busPtr->pose, msgPtr.pose);
  convertToBus(&busPtr->twist, msgPtr.twist);
}


// Conversions between SL_Bus_std_msgs_Header and std_msgs::msg::Header

void convertFromBus(std_msgs::msg::Header& msgPtr, SL_Bus_std_msgs_Header const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr.frame_id, busPtr->frame_id, busPtr->frame_id_SL_Info);
  convertFromBus(msgPtr.stamp, &busPtr->stamp);
}

void convertToBus(SL_Bus_std_msgs_Header* busPtr, const std_msgs::msg::Header& msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->frame_id, busPtr->frame_id_SL_Info, msgPtr.frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  convertToBus(&busPtr->stamp, msgPtr.stamp);
}

