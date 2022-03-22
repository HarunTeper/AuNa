#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::placeholders;


// Creates a node that publishes the global robot pose to the topic /namespace/localization_pose. The published transformation is in reference to the map frame.
// If there is no transformation from the map frame to the odom frame, the odometry pose is published as the global pose.
class LocalizationPose : public rclcpp::Node
{
    public:
        // Create a publisher, subscriber and prefix. Initialize the transform buffer and listener.
        LocalizationPose(std::string prefix)
        : Node("localization_pose_node"),buffer(this->get_clock()), listener(buffer)
        {
            publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("localization_pose", 10);
            subscription = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&LocalizationPose::topic_callback, this, _1));
            this->prefix = prefix;
        }

    private:
        // Receives the odometry pose and transforms it to the global pose with the map frame_id.
        void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
        {
            // Create a pose message for the latest odomery post
            geometry_msgs::msg::PoseStamped localization_pose;
            localization_pose.header.frame_id = prefix+"odom";
            localization_pose.header.stamp = msg->header.stamp;
            localization_pose.pose = msg->pose.pose;
            // Prepare the new pose and transformation between map and odom
            geometry_msgs::msg::PoseStamped new_localization_pose;
            geometry_msgs::msg::TransformStamped transformStamped;
            try
            {
                // Look up the transformation between map and odom
                if (this->buffer._frameExists("map"))
                {
                    transformStamped = this->buffer.lookupTransform("map",prefix+"odom",localization_pose.header.stamp);
                }
                else
                {
                    // If no transformation is available, create an identity transformation
                    transformStamped.header.frame_id = "map";
                    transformStamped.header.stamp = localization_pose.header.stamp;
                    transformStamped.child_frame_id = prefix+"odom";
                }
            }
            catch (tf2::TransformException &ex)
            {
                return;
            }
            // Transform the odom pose to the map frame and publish it
            tf2::doTransform(localization_pose,new_localization_pose,transformStamped);
            this->publisher->publish(new_localization_pose);
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
      rclcpp::spin(std::make_shared<LocalizationPose>(argv[1]));
    }
    else
    {
      rclcpp::spin(std::make_shared<LocalizationPose>(""));
    }

    rclcpp::shutdown();
    return 0;
}