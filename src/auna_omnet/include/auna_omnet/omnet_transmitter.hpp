#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "etsi_its_msgs/msg/cam.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>

class OmnetTransmitter : public rclcpp::Node
{
    public:
        OmnetTransmitter(std::string robot_name);
    private:
        void cam_callback();
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        rclcpp::TimerBase::SharedPtr timer;

        rclcpp::Publisher<etsi_its_msgs::msg::CAM>::SharedPtr publisher;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

        std::string robot_name_;
        float latitude_ = 0.0;
        float longitude_ = 0.0;
        float altitude_ = 0.0;
        float heading_ = 0.0;
        float speed_ = 0.0;
        float acceleration_ = 0.0;
        float yaw_rate_ = 0.0;
        float curvature_ = 0.0;

        // Odometry rate for acceleration
        float publish_period_ = 100;
        double scale_factor_ = 10;
};
