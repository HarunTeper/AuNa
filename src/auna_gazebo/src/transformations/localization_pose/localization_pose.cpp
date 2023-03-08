#include "auna_gazebo/localization_pose.hpp"

// Create a publisher, subscriber and prefix. Initialize the transform buffer and listener.
LocalizationPose::LocalizationPose(std::string prefix) : Node("localization_pose_node"),buffer(this->get_clock()), listener(buffer)
{
    publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("localization_pose", 2);
    subscription = this->create_subscription<nav_msgs::msg::Odometry>("odom", 2, [this](const nav_msgs::msg::Odometry::SharedPtr msg){odom_callback(msg);});
    if(prefix == "")
    {
        this->prefix = prefix;
    }
    else
    {
        this->prefix = prefix+"/";
    }
}


// Receives the odometry pose and transforms it to the global pose with the map frame_id.
void LocalizationPose::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
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


