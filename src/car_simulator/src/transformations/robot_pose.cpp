#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::placeholders;


// Creates a node that publishes the global robot pose to the topic /namespace/robot_pose. The published transformation is in reference to the map frame.
// If there is no transformation from the map frame to the odom frame, the odometry pose is published as the global pose.
class RobotPose : public rclcpp::Node
{
    public:
        // Create a publisher, subscriber and prefix. Initialize the transform buffer and listener.
        RobotPose(std::string prefix)
        : Node("robot_pose_node"),buffer(this->get_clock()), listener(buffer)
        {
            publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose", 10);
            subscription = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&RobotPose::topic_callback, this, _1));
            this->prefix = prefix;
        }

    private:
        // Receives the odometry pose and transforms it to the global pose with the map frame_id.
        void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
        {
            geometry_msgs::msg::PoseStamped robot_pose;
            robot_pose.header.frame_id = prefix+"odom";
            robot_pose.header.stamp = msg->header.stamp;
            robot_pose.pose = msg->pose.pose;


            geometry_msgs::msg::PoseStamped new_robot_pose;

            geometry_msgs::msg::TransformStamped transformStamped;

            try
            {
                if (this->buffer._frameExists("map"))
                {
                    transformStamped = this->buffer.lookupTransform("map",prefix+"odom",robot_pose.header.stamp);
                }
                else
                {
                    transformStamped.header.frame_id = "map";
                    transformStamped.header.stamp = robot_pose.header.stamp;
                    transformStamped.child_frame_id = prefix+"odom";
                }
            }
            catch (tf2::TransformException &ex)
            {
                return;
            }

            tf2::doTransform(robot_pose,new_robot_pose,transformStamped);
            this->publisher->publish(new_robot_pose);
          
        }

        tf2_ros::Buffer buffer;
        tf2_ros::TransformListener listener;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
        std::string prefix;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // Set prefix for topic and frame_id, if first argument is non-empty.
    if(argc > 1)
    {
      rclcpp::spin(std::make_shared<RobotPose>(argv[1]));
    }
    else
    {
      rclcpp::spin(std::make_shared<RobotPose>(""));
    }

    rclcpp::shutdown();
    return 0;
}