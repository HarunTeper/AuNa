#include "auna_physical/cmd_vel_to_ackermann.hpp"

CmdVelToAckermann::CmdVelToAckermann() : Node("cmd_vel_to_ackermann_node")
{
    cmd_vel_subscription = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 2, [this](const geometry_msgs::msg::Twist::SharedPtr msg){cmd_vel_callback(msg);});
    ackermann_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 2);

    this->declare_parameter<bool>("convert_yaw_to_steering_angle", false);
    this->declare_parameter<double>("wheelbase", 0.0);

    this->get_parameter<bool>("convert_yaw_to_steering_angle", convert_yaw_to_steering_angle_);
    this->get_parameter<double>("wheelbase", wheelbase_);

}

// Convert cmd_vel to ackermann msg
void CmdVelToAckermann::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;

    ackermann_msg.header.stamp = this->get_clock()->now();

    ackermann_msg.drive.speed = msg->linear.x;

    if(convert_yaw_to_steering_angle_)
    {
        if(fabs(msg->linear.x) < 0.001)
        {
            ackermann_msg.drive.steering_angle = atan(wheelbase_ * msg->angular.z);
        }
        else
        {
            // use ackermann_msg.drive.steering_angle = atan(wheelbase_ * msg->angular.z / msg->linear.x); to calculate the steering angle for positive velocies, but swap sign for negative velocities
            ackermann_msg.drive.steering_angle = atan(wheelbase_ * msg->angular.z / fabs(msg->linear.x));
        }
    }
    else
    {
        ackermann_msg.drive.steering_angle = msg->angular.z;
    }

    ackermann_publisher->publish(ackermann_msg);
}
