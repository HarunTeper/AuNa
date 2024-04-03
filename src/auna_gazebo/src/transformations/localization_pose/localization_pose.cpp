#include "auna_gazebo/localization_pose.hpp"

// Create a publisher, subscriber and prefix. Initialize the transform buffer and listener.
LocalizationPose::LocalizationPose(std::string prefix) : Node("localization_pose_node"),buffer(this->get_clock()), listener(buffer)
{
    publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("global_pose", 2);
    if(prefix == "")
    {
        this->prefix = prefix;
    }
    else
    {
        this->prefix = prefix+"/";
    }
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this](){timer_callback();});
}

void LocalizationPose::timer_callback(){
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
        transformStamped = this->buffer.lookupTransform(prefix+"odom",prefix+"base_link",tf2::TimePointZero);
        transformStamped = this->buffer.lookupTransform("map",prefix+"base_link",tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        return;
    }

    geometry_msgs::msg::PoseStamped localization_pose;
    localization_pose.header.frame_id = "map";
    localization_pose.header.stamp = this->get_clock()->now();
    localization_pose.pose.position.x = transformStamped.transform.translation.x;
    localization_pose.pose.position.y = transformStamped.transform.translation.y;
    localization_pose.pose.position.z = transformStamped.transform.translation.z;
    localization_pose.pose.orientation.x = transformStamped.transform.rotation.x;
    localization_pose.pose.orientation.y = transformStamped.transform.rotation.y;
    localization_pose.pose.orientation.z = transformStamped.transform.rotation.z;
    localization_pose.pose.orientation.w = transformStamped.transform.rotation.w;

    this->publisher->publish(localization_pose);
}
