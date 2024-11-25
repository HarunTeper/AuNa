#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "gazebo_msgs/srv/get_model_list.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class GlobalTF : public rclcpp::Node
{
    public:
    GlobalTF();

    void service_timer_callback();
    void model_srv_callback(const rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedFuture future);
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

    private:
    // TF Broadcaster
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // Timer and Service Client
    rclcpp::TimerBase::SharedPtr service_timer_;
    rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedPtr modelClient_;

    // Robot model vector and subscribers for local tf topics.
    std::vector<rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr> tf_subscribers_;
    std::vector<std::string> robot_models_;
};