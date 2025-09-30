// Copyright 2025 Harun Teper
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef AUNA_TF__GLOBAL_TF__HPP_
#define AUNA_TF__GLOBAL_TF__HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

// #include "gazebo_msgs/srv/get_model_list.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "gazebo_msgs/srv/get_model_list.hpp"

class GlobalTF : public rclcpp::Node
{
public:
  GlobalTF();

  void service_timer_callback();
  void model_srv_callback(
    const rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedFuture future);
  void tf_callback(
    const tf2_msgs::msg::TFMessage::SharedPtr msg, const std::string & robot_name, bool is_static);

private:
// TF Broadcaster
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

// Timer and Service Client
  rclcpp::TimerBase::SharedPtr service_timer_;
  rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedPtr modelClient_;

// Robot model vector and subscribers for local tf topics.
  std::vector<rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr> tf_subscribers_;
  std::vector<std::string> robot_models_;
};

#endif  // AUNA_TF__GLOBAL_TF__HPP_
