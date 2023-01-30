#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include "rclcpp/rclcpp.hpp"
#include <auna_its_msgs/msg/cam.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <example_interfaces/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include "CACC_Center_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(auna_its_msgs::msg::CAM& msgPtr, SL_Bus_auna_its_msgs_CAM const* busPtr);
void convertToBus(SL_Bus_auna_its_msgs_CAM* busPtr, const auna_its_msgs::msg::CAM& msgPtr);

void convertFromBus(builtin_interfaces::msg::Time& msgPtr, SL_Bus_builtin_interfaces_Time const* busPtr);
void convertToBus(SL_Bus_builtin_interfaces_Time* busPtr, const builtin_interfaces::msg::Time& msgPtr);

void convertFromBus(example_interfaces::msg::Float64& msgPtr, SL_Bus_example_interfaces_Float64 const* busPtr);
void convertToBus(SL_Bus_example_interfaces_Float64* busPtr, const example_interfaces::msg::Float64& msgPtr);

void convertFromBus(geometry_msgs::msg::Point& msgPtr, SL_Bus_geometry_msgs_Point const* busPtr);
void convertToBus(SL_Bus_geometry_msgs_Point* busPtr, const geometry_msgs::msg::Point& msgPtr);

void convertFromBus(geometry_msgs::msg::Pose& msgPtr, SL_Bus_geometry_msgs_Pose const* busPtr);
void convertToBus(SL_Bus_geometry_msgs_Pose* busPtr, const geometry_msgs::msg::Pose& msgPtr);

void convertFromBus(geometry_msgs::msg::PoseStamped& msgPtr, SL_Bus_geometry_msgs_PoseStamped const* busPtr);
void convertToBus(SL_Bus_geometry_msgs_PoseStamped* busPtr, const geometry_msgs::msg::PoseStamped& msgPtr);

void convertFromBus(geometry_msgs::msg::PoseWithCovariance& msgPtr, SL_Bus_geometry_msgs_PoseWithCovariance const* busPtr);
void convertToBus(SL_Bus_geometry_msgs_PoseWithCovariance* busPtr, const geometry_msgs::msg::PoseWithCovariance& msgPtr);

void convertFromBus(geometry_msgs::msg::Quaternion& msgPtr, SL_Bus_geometry_msgs_Quaternion const* busPtr);
void convertToBus(SL_Bus_geometry_msgs_Quaternion* busPtr, const geometry_msgs::msg::Quaternion& msgPtr);

void convertFromBus(geometry_msgs::msg::Twist& msgPtr, SL_Bus_geometry_msgs_Twist const* busPtr);
void convertToBus(SL_Bus_geometry_msgs_Twist* busPtr, const geometry_msgs::msg::Twist& msgPtr);

void convertFromBus(geometry_msgs::msg::TwistWithCovariance& msgPtr, SL_Bus_geometry_msgs_TwistWithCovariance const* busPtr);
void convertToBus(SL_Bus_geometry_msgs_TwistWithCovariance* busPtr, const geometry_msgs::msg::TwistWithCovariance& msgPtr);

void convertFromBus(geometry_msgs::msg::Vector3& msgPtr, SL_Bus_geometry_msgs_Vector3 const* busPtr);
void convertToBus(SL_Bus_geometry_msgs_Vector3* busPtr, const geometry_msgs::msg::Vector3& msgPtr);

void convertFromBus(nav_msgs::msg::Odometry& msgPtr, SL_Bus_nav_msgs_Odometry const* busPtr);
void convertToBus(SL_Bus_nav_msgs_Odometry* busPtr, const nav_msgs::msg::Odometry& msgPtr);

void convertFromBus(std_msgs::msg::Header& msgPtr, SL_Bus_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_std_msgs_Header* busPtr, const std_msgs::msg::Header& msgPtr);


#endif
