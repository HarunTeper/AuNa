#include "rclcpp/rclcpp.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <cmath>
#include <string>

using namespace std;

class WallFollow : public rclcpp::Node
{
public:
  WallFollow() : Node("wallfollowing")
  {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));
    drive_pub_ =
      this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

    RCLCPP_INFO(this->get_logger(), "WallFollow node initialized.");
  }

private:
  double kp = 0.25;
  double kd = 0;
  double ki = 0;

  double prev_error = 0.0;
  double integral = 0.0;

  std::string lidarscan_topic = "scan";
  std::string drive_topic = "cmd_vel/wallfollowing";

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  double get_range(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan, double angle)
  {
    if (angle < scan->angle_min || angle > scan->angle_max) return -1.0;

    int index = static_cast<int>(std::round((angle - scan->angle_min) / scan->angle_increment));
    if (index < 0 || index >= static_cast<int>(scan->ranges.size())) return -1.0;

    float dist = scan->ranges[index];
    if (std::isnan(dist) || std::isinf(dist)) return -1.0;

    return static_cast<double>(dist);
  }

  double get_error(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan, double desired_distance)
  {
    double angle_a = M_PI / 4;
    double angle_b = M_PI / 2;

    double a = get_range(scan, angle_a);
    double b = get_range(scan, angle_b);

    if (a == 0.0 || b == 0.0) return 0.0;

    double swing = angle_b - angle_a;

    double alpha = atan((a * cos(swing) - b) / (a * sin(swing)));

    double Dt = b * cos(alpha);

    double L = 1.0;
    double Dt1 = Dt + L * sin(alpha);

    return desired_distance - Dt1;
  }

  void pid_control(double error, double velocity)
  {
    double derivative = error - prev_error;
    integral += error;

    double angle = kp * error + kd * derivative + ki * integral;

    cout << "Angle to steer -> " << (angle) << endl;

    prev_error = error;

    if (angle < -0.4189) angle = -0.4189;
    if (angle > 0.4189) angle = 0.4189;

    if (std::abs(error) > 1.0)
      velocity = 1.0;
    else
      velocity = 2.0;

    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = this->now();
    drive_msg.drive.speed = velocity;
    drive_msg.drive.steering_angle = -angle;
    drive_pub_->publish(drive_msg);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
  {
    double desired_distance = 0.5;
    double velocity = 1.5;

    double error = get_error(scan_msg, desired_distance);

    pid_control(error, velocity);
  }

  double radiansToDegree(const double & angleInRadians) { return angleInRadians * (180.0 / M_PI); }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollow>());
  rclcpp::shutdown();
  return 0;
}