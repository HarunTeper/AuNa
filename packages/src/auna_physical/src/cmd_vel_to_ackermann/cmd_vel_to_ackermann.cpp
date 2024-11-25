#include "auna_physical/cmd_vel_to_ackermann.hpp"

CmdVelToAckermann::CmdVelToAckermann() : Node("cmd_vel_to_ackermann_node")
{
    cmd_vel_subscription = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 2, [this](const geometry_msgs::msg::Twist::SharedPtr msg){cmd_vel_callback(msg);});
    ackermann_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 2);

    this->declare_parameter<bool>("convert_yaw_to_steering_angle", false);
    this->declare_parameter<double>("wheelbase", 0.0);

    this->get_parameter<bool>("convert_yaw_to_steering_angle", convert_yaw_to_steering_angle_);
    this->get_parameter<double>("wheelbase", wheelbase_);

    emergency_stop_active_ = false;

    //service server for emergency stop
    emergency_stop_service = this->create_service<std_srvs::srv::Empty>("/emergency_stop_enable", [this](const std_srvs::srv::Empty::Request::SharedPtr request, const std_srvs::srv::Empty::Response::SharedPtr response){emergency_stop_callback(request, response);});

    //service server for emergency stop disable
    emergency_stop_disable_service = this->create_service<std_srvs::srv::Empty>("/emergency_stop_disable", [this](const std_srvs::srv::Empty::Request::SharedPtr request, const std_srvs::srv::Empty::Response::SharedPtr response){emergency_stop_disable_callback(request, response);});

}

// Emergency stop callback
void CmdVelToAckermann::emergency_stop_callback(const std_srvs::srv::Empty::Request::SharedPtr request, const std_srvs::srv::Empty::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Emergency stop activated");
    emergency_stop_active_ = true;
    (void)request;
    (void)response;
}

// Emergency stop disable callback
void CmdVelToAckermann::emergency_stop_disable_callback(const std_srvs::srv::Empty::Request::SharedPtr request, const std_srvs::srv::Empty::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Emergency stop disabled");
    emergency_stop_active_ = false;
    (void)request;
    (void)response;
}

// Convert cmd_vel to ackermann msg
void CmdVelToAckermann::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;

    ackermann_msg.header.stamp = this->get_clock()->now();

    if(emergency_stop_active_)
    {
        ackermann_msg.drive.speed = 0.0;
        ackermann_msg.drive.steering_angle = 0.0;
        ackermann_publisher->publish(ackermann_msg);
        return;
    }

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
