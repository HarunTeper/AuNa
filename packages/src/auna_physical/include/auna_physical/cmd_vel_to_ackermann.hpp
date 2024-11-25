
#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/empty.hpp"
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

        //service server for emergency stop
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr emergency_stop_service;
        void emergency_stop_callback(const std_srvs::srv::Empty::Request::SharedPtr request, const std_srvs::srv::Empty::Response::SharedPtr response);

        //service server for emergency stop disable
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr emergency_stop_disable_service;
        void emergency_stop_disable_callback(const std_srvs::srv::Empty::Request::SharedPtr request, const std_srvs::srv::Empty::Response::SharedPtr response);


        bool convert_yaw_to_steering_angle_;
        double wheelbase_;
        bool emergency_stop_active_;
};