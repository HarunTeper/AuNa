
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class VescStart : public rclcpp::Node
{
public:
  VescStart();

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odometry_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
  rclcpp::TimerBase::SharedPtr timer_publish_cmd_vel_;

  void callback_timer_publish_cmd_vel();
  void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
};