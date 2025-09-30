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

#ifndef AUNA_OMNET__OMNET_TRANSMITTER_HPP_
#define AUNA_OMNET__OMNET_TRANSMITTER_HPP_

#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>

#include "etsi_its_cam_msgs/msg/cam.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

class OmnetTransmitter : public rclcpp::Node
{
public:
  explicit OmnetTransmitter(const std::string & robot_name);

private:
  void cam_callback();
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Publisher<etsi_its_cam_msgs::msg::CAM>::SharedPtr publisher;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    pose_subscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

  std::string robot_name_;
  float latitude_ = 0.0;
  float longitude_ = 0.0;
  float altitude_ = 0.0;
  float heading_ = 0.0;
  float speed_ = 0.0;
  float acceleration_ = 0.0;
  float yaw_rate_ = 0.0;
  float curvature_ = 0.0;

  // Odometry rate for acceleration
  double publish_period_ = 100.0;
  double scale_factor_ = 10.0;
};

#endif  // AUNA_OMNET__OMNET_TRANSMITTER_HPP_

