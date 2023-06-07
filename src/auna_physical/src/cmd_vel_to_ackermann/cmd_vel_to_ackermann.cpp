#include "auna_physical/cmd_vel_to_ackermann.hpp"

CmdVelToAckermann::CmdVelToAckermann() : Node("cmd_vel_to_ackermann_node")
{
    cmd_vel_subscription = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 2, [this](const geometry_msgs::msg::Twist::SharedPtr msg){cmd_vel_callback(msg);});
    ackermann_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 2);
}

// Convert cmd_vel to ackermann msg
void CmdVelToAckermann::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;

    ackermann_msg.header.stamp = this->get_clock()->now();

    ackermann_msg.drive.speed = msg->linear.x;
    ackermann_msg.drive.steering_angle = msg->angular.z;

    ackermann_publisher->publish(ackermann_msg);
}
