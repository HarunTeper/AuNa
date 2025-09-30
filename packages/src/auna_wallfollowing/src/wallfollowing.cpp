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

#include "auna_wallfollowing/wallfollowing.hpp"

#include <iostream>

using namespace std;

WallFollow::WallFollow()
: Node("wallfollowing")
{
  // Declare and get parameters
  declare_parameters();

  // Initialize ROS2 interfaces
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    lidarscan_topic_, 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));
  drive_pub_ =
    this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "WallFollow node initialized.");
}

void WallFollow::declare_parameters()
{
  // Declare PID parameters
  this->declare_parameter("kp", 0.25);
  this->declare_parameter("kd", 0.0);
  this->declare_parameter("ki", 0.0);

  // Declare controller parameters
  this->declare_parameter("desired_distance", 0.5);
  this->declare_parameter("velocity", 1.5);
  this->declare_parameter("max_steering_angle", 0.4189);
  this->declare_parameter("min_velocity", 1.0);
  this->declare_parameter("max_velocity", 2.0);
  this->declare_parameter("error_threshold", 1.0);

  // Declare angle parameters
  this->declare_parameter("angle_a", M_PI / 4);
  this->declare_parameter("angle_b", M_PI / 2);
  this->declare_parameter("lookahead_distance", 1.0);

  // Declare topic names
  this->declare_parameter("lidarscan_topic", std::string("scan"));
  this->declare_parameter("drive_topic", std::string("cmd_vel/wallfollowing"));

  // Get parameters
  kp_ = this->get_parameter("kp").as_double();
  kd_ = this->get_parameter("kd").as_double();
  ki_ = this->get_parameter("ki").as_double();

  desired_distance_ = this->get_parameter("desired_distance").as_double();
  velocity_ = this->get_parameter("velocity").as_double();
  max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
  min_velocity_ = this->get_parameter("min_velocity").as_double();
  max_velocity_ = this->get_parameter("max_velocity").as_double();
  error_threshold_ = this->get_parameter("error_threshold").as_double();

  angle_a_ = this->get_parameter("angle_a").as_double();
  angle_b_ = this->get_parameter("angle_b").as_double();
  lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();

  lidarscan_topic_ = this->get_parameter("lidarscan_topic").as_string();
  drive_topic_ = this->get_parameter("drive_topic").as_string();

  // Initialize control variables
  prev_error_ = 0.0;
  integral_ = 0.0;
}

double WallFollow::get_range(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan, double angle)
{
  if (angle < scan->angle_min || angle > scan->angle_max) {
    return -1.0;
  }

  int index = static_cast<int>(std::round((angle - scan->angle_min) / scan->angle_increment));
  if (index < 0 || index >= static_cast<int>(scan->ranges.size())) {
    return -1.0;
  }

  float dist = scan->ranges[index];
  if (std::isnan(dist) || std::isinf(dist)) {
    return -1.0;
  }

  return static_cast<double>(dist);
}

double WallFollow::get_error(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr scan, double desired_distance)
{
  double a = get_range(scan, angle_a_);
  double b = get_range(scan, angle_b_);

  if (a == 0.0 || b == 0.0) {
    return 0.0;
  }

  double swing = angle_b_ - angle_a_;

  double alpha = atan((a * cos(swing) - b) / (a * sin(swing)));

  double Dt = b * cos(alpha);

  double Dt1 = Dt + lookahead_distance_ * sin(alpha);

  return desired_distance - Dt1;
}

void WallFollow::pid_control(double error, double velocity)
{
  double derivative = error - prev_error_;
  integral_ += error;

  double angle = kp_ * error + kd_ * derivative + ki_ * integral_;

  RCLCPP_DEBUG(this->get_logger(), "Angle to steer -> %f", angle);

  prev_error_ = error;

  if (angle < -max_steering_angle_) {
    angle = -max_steering_angle_;
  }
  if (angle > max_steering_angle_) {
    angle = max_steering_angle_;
  }

  if (std::abs(error) > error_threshold_) {
    velocity = min_velocity_;
  } else {
    velocity = max_velocity_;
  }

  auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
  drive_msg.header.stamp = this->now();
  drive_msg.drive.speed = velocity;
  drive_msg.drive.steering_angle = -angle;
  drive_pub_->publish(drive_msg);
}

void WallFollow::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
  double error = get_error(scan_msg, desired_distance_);

  pid_control(error, velocity_);
}

double WallFollow::radiansToDegree(const double & angleInRadians)
{
  return angleInRadians * (180.0 / M_PI);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollow>());
  rclcpp::shutdown();
  return 0;
}
