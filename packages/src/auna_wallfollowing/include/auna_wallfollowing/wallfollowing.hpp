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

#include <cmath>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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
  // Timestamp of the previous control update, used to compute the elapsed
  // time (dt) between successive scan callbacks. Zero until the first update.
  rclcpp::Time prev_time_;
  bool prev_time_valid_;

  // Controller parameters
  double desired_distance_;
  double velocity_;
  double max_steering_angle_;
  double min_velocity_;
  double max_velocity_;
  double error_threshold_;

  // Bounds on the measured control period. Samples outside this range are
  // treated as unreliable (dropped scans, debugger pauses, bag seeks) and the
  // rate-dependent PID terms are skipped for that cycle.
  double min_dt_;
  double max_dt_;

  // Clamp on the magnitude of the integral term to prevent windup once the
  // integral is scaled by dt.
  double max_integral_;

  // Angle parameters for wall detection
  double angle_a_;
  double angle_b_;
  double lookahead_distance_;

  // Topic names
  std::string lidarscan_topic_;
  std::string drive_topic_;

  // ROS2 interfaces
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
    drive_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  /**
   * @brief Get range measurement at a specific angle from laser scan
   * @param scan Laser scan message
   * @param angle Angle in radians
   * @return Range measurement or -1.0 if invalid
   */
  double get_range(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan,
    double angle);

  /**
   * @brief Calculate error between desired and actual distance to wall
   * @param scan Laser scan message
   * @param desired_distance Desired distance to maintain from wall
   * @return Error value
   */
  double get_error(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan,
    double desired_distance);

  /**
   * @brief PID control for wall following
   *
   * The integral and derivative terms are scaled by the measured elapsed time
   * between consecutive scan callbacks, so that the tuned ki_ and kd_ gains
   * keep their physical meaning (1/s and s respectively) independently of the
   * LiDAR callback rate.
   *
   * @param error Error from desired wall distance
   * @param velocity Base velocity
   * @param dt Elapsed time since the previous control update, in seconds.
   *           Values <= 0 disable the integral and derivative contributions.
   */
  void pid_control(double error, double velocity, double dt);

  /**
   * @brief Callback for laser scan messages
   * @param scan_msg Laser scan message
   */
  void scan_callback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

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
