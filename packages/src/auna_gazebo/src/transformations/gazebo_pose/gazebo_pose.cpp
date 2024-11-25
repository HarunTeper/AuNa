#include "auna_gazebo/gazebo_pose.hpp"

// Create the publisher, timer and service client
GazeboPose::GazeboPose(std::string name): Node("model_state_node"), buffer_(this->get_clock()), listener_(buffer_), broadcaster_(this)
{
    modelClient_ = this->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
    service_timer_ = this->create_wall_timer(std::chrono::milliseconds(publish_milliseconds_), [this](){service_timer_callback();});
    this->name_ = name;
}

// Timer callback to periodically call a service request for the model state
void GazeboPose::service_timer_callback()
{
    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = name_;
    while (!modelClient_->wait_for_service(std::chrono::seconds(5))) 
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    auto result = modelClient_->async_send_request(request,std::bind(&GazeboPose::model_srv_callback,this,std::placeholders::_1));
}

// Read the requested entity state and publish the received pose to simulation_pose
void GazeboPose::model_srv_callback(const rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture future)
{
    auto result = future.get();
    auto entity = result.get();

    tf2::Quaternion q(entity->state.pose.orientation.x, entity->state.pose.orientation.y, entity->state.pose.orientation.z, entity->state.pose.orientation.w);
    tf2::Transform map_to_base(q, tf2::Vector3(entity->state.pose.position.x, entity->state.pose.position.y, entity->state.pose.position.z));

    geometry_msgs::msg::TransformStamped odom_to_base_link_lookup;
    try
    {
        if (name_ == "")
        {
            odom_to_base_link_lookup = buffer_.lookupTransform("odom","base_link", tf2::TimePointZero);
        }
        else
        {
            odom_to_base_link_lookup = buffer_.lookupTransform(name_+"/odom",name_+"/base_link", tf2::TimePointZero);
        }
    }
    catch (tf2::TransformException &ex)
    {
        return;
    }
    tf2::Quaternion q_ob(odom_to_base_link_lookup.transform.rotation.x, odom_to_base_link_lookup.transform.rotation.y, odom_to_base_link_lookup.transform.rotation.z, odom_to_base_link_lookup.transform.rotation.w); 
    tf2::Transform odom_to_base(q_ob, tf2::Vector3(odom_to_base_link_lookup.transform.translation.x, odom_to_base_link_lookup.transform.translation.y, odom_to_base_link_lookup.transform.translation.z));

    tf2::Transform map_to_odom = map_to_base*odom_to_base.inverse();

    geometry_msgs::msg::TransformStamped map_to_odom_transform_msg;
    map_to_odom_transform_msg.transform.translation.x = map_to_odom.getOrigin()[0];
    map_to_odom_transform_msg.transform.translation.y = map_to_odom.getOrigin()[1];
    map_to_odom_transform_msg.transform.translation.z = map_to_odom.getOrigin()[2];
    map_to_odom_transform_msg.transform.rotation.x = map_to_odom.getRotation().getX();
    map_to_odom_transform_msg.transform.rotation.y = map_to_odom.getRotation().getY();
    map_to_odom_transform_msg.transform.rotation.z = map_to_odom.getRotation().getZ();
    map_to_odom_transform_msg.transform.rotation.w = map_to_odom.getRotation().getW();
    map_to_odom_transform_msg.header.frame_id = "map";
    if (name_ == "")
    {
        map_to_odom_transform_msg.child_frame_id = "odom";
    }
    else
    {
        map_to_odom_transform_msg.child_frame_id = name_+"/odom";
    }
    auto stamp = tf2_ros::fromMsg(entity->header.stamp);
    map_to_odom_transform_msg.header.stamp = tf2_ros::toMsg(stamp+tf2::durationFromSec(1.0));
    broadcaster_.sendTransform(map_to_odom_transform_msg);
}