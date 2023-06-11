
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

class ViconTFConverter : public rclcpp::Node
{
    public:
        ViconTFConverter(std::string name);
    private:
        void vicon_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener listener_;
        tf2_ros::TransformBroadcaster broadcaster_;

        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr vicon_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

        std::string name_;
        int publish_milliseconds_ = 100;
};