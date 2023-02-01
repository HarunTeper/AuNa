
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class CmdVelToAckermann : public rclcpp::Node
{
    public:
        CmdVelToAckermann();
    private:
        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription;
};