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


#include "auna_f110/vesc_start.hpp"

VescStart::VescStart() : Node("vesc_start")
{
  subscription_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 2, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { callback_odometry(msg); });
  publisher_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 2);
  timer_publish_cmd_vel_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&VescStart::callback_timer_publish_cmd_vel, this));
}

// Publish cmd_vel to start the vesc
void VescStart::callback_timer_publish_cmd_vel()
{
  geometry_msgs::msg::Twist cmd_vel_msg;

  cmd_vel_msg.linear.x = 0.0;
  cmd_vel_msg.angular.z = 0.0;

  publisher_cmd_vel_->publish(cmd_vel_msg);
}

// Subscribe to odom to start the vesc
void VescStart::callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  (void)msg;
  RCLCPP_INFO(this->get_logger(), "Vesc started");
  timer_publish_cmd_vel_->cancel();
  rclcpp::shutdown();
}
