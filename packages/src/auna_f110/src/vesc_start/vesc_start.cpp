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
