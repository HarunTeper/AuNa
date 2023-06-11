#include "auna_physical/vicon_tf_converter.hpp"

ViconTFConverter::ViconTFConverter(std::string name): Node("vicon_tf_converter"), buffer_(this->get_clock()), listener_(buffer_), broadcaster_(this)
{
    //vicon_sub for vicon_callback using lambda function
    vicon_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("vicon/pose", 2, [this](const geometry_msgs::msg::TransformStamped::SharedPtr msg) -> void {vicon_callback(msg);});
    //publisher for posestamped msg
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("global_pose", 2);
    name_ = name;
}

void ViconTFConverter::vicon_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg){

    //publish posestamped msg
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = msg->transform.translation.x;
    pose_msg.pose.position.y = msg->transform.translation.y;
    pose_msg.pose.position.z = msg->transform.translation.z;
    pose_msg.pose.orientation.x = msg->transform.rotation.x;
    pose_msg.pose.orientation.y = msg->transform.rotation.y;
    pose_msg.pose.orientation.z = msg->transform.rotation.z;
    pose_msg.pose.orientation.w = msg->transform.rotation.w;
    pose_pub_->publish(pose_msg);


    // tf2::Quaternion q_vicon(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
    // tf2::Transform map_to_base(q_vicon, tf2::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z));

    // geometry_msgs::msg::TransformStamped odom_to_base_link_lookup;
    // try
    // {
    //     if (name_ == "")
    //     {
    //         odom_to_base_link_lookup = buffer_.lookupTransform("odom","base_link", tf2::TimePointZero);
    //     }
    //     else
    //     {
    //         odom_to_base_link_lookup = buffer_.lookupTransform(name_+"/odom",name_+"/base_link", tf2::TimePointZero);
    //     }
    // }
    // catch (tf2::TransformException &ex)
    // {
    //     return;
    // }
    // tf2::Quaternion q_ob(odom_to_base_link_lookup.transform.rotation.x, odom_to_base_link_lookup.transform.rotation.y, odom_to_base_link_lookup.transform.rotation.z, odom_to_base_link_lookup.transform.rotation.w); 
    // tf2::Transform odom_to_base(q_ob, tf2::Vector3(odom_to_base_link_lookup.transform.translation.x, odom_to_base_link_lookup.transform.translation.y, odom_to_base_link_lookup.transform.translation.z));

    // tf2::Transform map_to_odom = map_to_base*odom_to_base.inverse();

    // geometry_msgs::msg::TransformStamped map_to_odom_transform_msg;
    // map_to_odom_transform_msg.transform.translation.x = map_to_odom.getOrigin()[0];
    // map_to_odom_transform_msg.transform.translation.y = map_to_odom.getOrigin()[1];
    // map_to_odom_transform_msg.transform.translation.z = map_to_odom.getOrigin()[2];
    // map_to_odom_transform_msg.transform.rotation.x = map_to_odom.getRotation().getX();
    // map_to_odom_transform_msg.transform.rotation.y = map_to_odom.getRotation().getY();
    // map_to_odom_transform_msg.transform.rotation.z = map_to_odom.getRotation().getZ();
    // map_to_odom_transform_msg.transform.rotation.w = map_to_odom.getRotation().getW();
    // map_to_odom_transform_msg.header.frame_id = "map";
    // if (name_ == "")
    // {
    //     map_to_odom_transform_msg.child_frame_id = "odom";
    // }
    // else
    // {
    //     map_to_odom_transform_msg.child_frame_id = name_+"/odom";
    // }
    // auto stamp = tf2_ros::fromMsg(msg->header.stamp) + tf2::durationFromSec(0.5);
    // map_to_odom_transform_msg.header.stamp = tf2_ros::toMsg(stamp);
    // broadcaster_.sendTransform(map_to_odom_transform_msg);



    geometry_msgs::msg::TransformStamped reference_transform;
    reference_transform.transform.translation.x = msg->transform.translation.x;
    reference_transform.transform.translation.y = msg->transform.translation.y;
    reference_transform.transform.translation.z = msg->transform.translation.z;
    reference_transform.transform.rotation.x = msg->transform.rotation.x;
    reference_transform.transform.rotation.y = msg->transform.rotation.y;
    reference_transform.transform.rotation.z = msg->transform.rotation.z;
    reference_transform.transform.rotation.w = msg->transform.rotation.w;
    reference_transform.header.frame_id = "map";
    if (name_ == "")
    {
        reference_transform.child_frame_id = "odom";
    }
    else
    {
        reference_transform.child_frame_id = name_+"/odom";
    }
    auto stamp_reference = tf2_ros::fromMsg(msg->header.stamp) + tf2::durationFromSec(0.5);
    reference_transform.header.stamp = tf2_ros::toMsg(stamp_reference);
    broadcaster_.sendTransform(reference_transform);

    //publish the identity transform between odom and base_link
    geometry_msgs::msg::TransformStamped identity_transform;
    identity_transform.transform.translation.x = 0;
    identity_transform.transform.translation.y = 0;
    identity_transform.transform.translation.z = 0;
    identity_transform.transform.rotation.x = 0;
    identity_transform.transform.rotation.y = 0;
    identity_transform.transform.rotation.z = 0;
    identity_transform.transform.rotation.w = 1;
    if (name_ == "")
    {
        identity_transform.header.frame_id = "odom";
        identity_transform.child_frame_id = "base_link";
    }
    else
    {
        identity_transform.header.frame_id = name_+"/odom";
        identity_transform.child_frame_id = name_+"/base_link";
    }
    auto stamp_identity = tf2_ros::fromMsg(msg->header.stamp) + tf2::durationFromSec(0.5);
    identity_transform.header.stamp = tf2_ros::toMsg(stamp_identity);
    broadcaster_.sendTransform(identity_transform);

}
