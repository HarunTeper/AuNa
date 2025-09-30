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


#ifndef AUNA_WALLFOLLOWING__WALLFOLLOWING_HPP_
#define AUNA_WALLFOLLOWING__WALLFOLLOWING_HPP_

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <cmath>
#include <string>

class WallFollow : public rclcpp::Node
{
public:
WallFollow();

private:
// PID parameters
double kp_;
double kd_;
double ki_;

// Control variables
double prev_error_;
double integral_;

// Controller parameters
double desired_distance_;
double velocity_;
double max_steering_angle_;
double min_velocity_;
double max_velocity_;
double error_threshold_;

// Angle parameters for wall detection
double angle_a_;
double angle_b_;
double lookahead_distance_;

// Topic names
std::string lidarscan_topic_;
std::string drive_topic_;

// ROS2 interfaces
rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

/**
 * @brief Get range measurement at a specific angle from laser scan
 * @param scan Laser scan message
 * @param angle Angle in radians
 * @return Range measurement or -1.0 if invalid
 */
double get_range(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan, double angle);

/**
 * @brief Calculate error between desired and actual distance to wall
 * @param scan Laser scan message
 * @param desired_distance Desired distance to maintain from wall
 * @return Error value
 */
double get_error(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan, double desired_distance);

/**
 * @brief PID control for wall following
 * @param error Error from desired wall distance
 * @param velocity Base velocity
 */
void pid_control(double error, double velocity);

/**
 * @brief Callback for laser scan messages
 * @param scan_msg Laser scan message
 */
void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

/**
 * @brief Convert radians to degrees
 * @param angleInRadians Angle in radians
 * @return Angle in degrees
 */
double radiansToDegree(const double & angleInRadians);

/**
 * @brief Declare and get parameters from parameter server
 */
void declare_parameters();
};

#endif  // AUNA_WALLFOLLOWING__WALLFOLLOWING_HPP_
