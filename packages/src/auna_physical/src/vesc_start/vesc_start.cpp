#include "auna_physical/vesc_start.hpp"


VescStart::VescStart() : Node("vesc_start")
{
    odometry_subscription = this->create_subscription<nav_msgs::msg::Odometry>("odom", 2, [this](const nav_msgs::msg::Odometry::SharedPtr msg){odometry_callback(msg);});
    cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 2);
    timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&VescStart::timer_callback, this));
}

// Publish cmd_vel to start the vesc
void VescStart::timer_callback()
{
    geometry_msgs::msg::Twist cmd_vel_msg;

    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = 0.0;

    cmd_vel_publisher->publish(cmd_vel_msg);
}

// Subscribe to odom to start the vesc
void VescStart::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    (void)msg;
    RCLCPP_INFO(this->get_logger(), "Vesc started");
    timer->cancel();
    rclcpp::shutdown();
}
