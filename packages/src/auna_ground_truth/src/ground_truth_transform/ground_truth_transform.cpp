#include "auna_ground_truth/ground_truth_transform.hpp"

// Create the publisher, timer and service client
GroundTruthTransform::GroundTruthTransform()
: Node("ground_truth_transform_node"),
  buffer_(this->get_clock()),
  listener_(buffer_),
  broadcaster_(this)
{
  modelClient_ = this->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
  service_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(publish_milliseconds_), [this]() { service_timer_callback(); });
  std::string ns = this->get_namespace();
  if (!ns.empty()) {
    this->name_ = ns.substr(1);  // Remove the first character (typically a slash)
  } else {
    this->name_ = ns;  // Keep it empty if namespace is empty
  }

  RCLCPP_INFO(
    this->get_logger(), "Ground truth localization node initialized with namespace: %s",
    name_.c_str());
}

// Timer callback to periodically call a service request for the model state
void GroundTruthTransform::service_timer_callback()
{
  auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
  request->name = name_;
  while (!modelClient_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  auto result = modelClient_->async_send_request(
    request, std::bind(&GroundTruthTransform::model_srv_callback, this, std::placeholders::_1));
}

// Read the requested entity state and publish the received pose to simulation_pose
void GroundTruthTransform::model_srv_callback(
  const rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture future)
{
  auto result = future.get();
  auto entity = result.get();

  tf2::Quaternion q(
    entity->state.pose.orientation.x, entity->state.pose.orientation.y,
    entity->state.pose.orientation.z, entity->state.pose.orientation.w);
  tf2::Transform map_to_base(
    q,
    tf2::Vector3(
      entity->state.pose.position.x, entity->state.pose.position.y, entity->state.pose.position.z));

  geometry_msgs::msg::TransformStamped map_to_odom_lookup;
  geometry_msgs::msg::TransformStamped odom_to_base_link_lookup;
  
  std::string highest_frame = "map";  // Default to map if no transforms found

  if (buffer_.canTransform("odom", "base_link", tf2::TimePointZero)) {
    odom_to_base_link_lookup = buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
    highest_frame = "odom";
  } else {
    odom_to_base_link_lookup.header.frame_id = "odom";
    odom_to_base_link_lookup.child_frame_id = "base_link";
    odom_to_base_link_lookup.transform.translation.x = 0.0;
    odom_to_base_link_lookup.transform.translation.y = 0.0;
    odom_to_base_link_lookup.transform.translation.z = 0.0;
    odom_to_base_link_lookup.transform.rotation.x = 0.0;
    odom_to_base_link_lookup.transform.rotation.y = 0.0;
    odom_to_base_link_lookup.transform.rotation.z = 0.0;
    odom_to_base_link_lookup.transform.rotation.w = 1.0;
  }
  // Set to identity if not found
  if (buffer_.canTransform("map", "odom", tf2::TimePointZero)) {
    map_to_odom_lookup = buffer_.lookupTransform("map", "odom", tf2::TimePointZero);
    highest_frame = "map";
  } else {
    map_to_odom_lookup.header.frame_id = "map";
    map_to_odom_lookup.child_frame_id = "odom";
    map_to_odom_lookup.transform.translation.x = 0.0;
    map_to_odom_lookup.transform.translation.y = 0.0;
    map_to_odom_lookup.transform.translation.z = 0.0;
    map_to_odom_lookup.transform.rotation.x = 0.0;
    map_to_odom_lookup.transform.rotation.y = 0.0;
    map_to_odom_lookup.transform.rotation.z = 0.0;
    map_to_odom_lookup.transform.rotation.w = 1.0;
  }

  tf2::Quaternion q_mo(
    map_to_odom_lookup.transform.rotation.x, map_to_odom_lookup.transform.rotation.y,
    map_to_odom_lookup.transform.rotation.z, map_to_odom_lookup.transform.rotation.w);
  tf2::Transform map_to_odom(
    q_mo, tf2::Vector3(
      map_to_odom_lookup.transform.translation.x,
      map_to_odom_lookup.transform.translation.y,
      map_to_odom_lookup.transform.translation.z));

  tf2::Quaternion q_ob(
    odom_to_base_link_lookup.transform.rotation.x, odom_to_base_link_lookup.transform.rotation.y,
    odom_to_base_link_lookup.transform.rotation.z, odom_to_base_link_lookup.transform.rotation.w);
  tf2::Transform odom_to_base_link(
    q_ob, tf2::Vector3(
            odom_to_base_link_lookup.transform.translation.x,
            odom_to_base_link_lookup.transform.translation.y,
            odom_to_base_link_lookup.transform.translation.z));

  tf2::Transform total_transform = map_to_odom * odom_to_base_link;

  tf2::Transform gazebo_transform = map_to_base * total_transform.inverse();

  geometry_msgs::msg::TransformStamped map_to_odom_transform_msg;
  map_to_odom_transform_msg.transform.translation.x = map_to_odom.getOrigin()[0];
  map_to_odom_transform_msg.transform.translation.y = map_to_odom.getOrigin()[1];
  map_to_odom_transform_msg.transform.translation.z = map_to_odom.getOrigin()[2];
  map_to_odom_transform_msg.transform.rotation.x = map_to_odom.getRotation().getX();
  map_to_odom_transform_msg.transform.rotation.y = map_to_odom.getRotation().getY();
  map_to_odom_transform_msg.transform.rotation.z = map_to_odom.getRotation().getZ();
  map_to_odom_transform_msg.transform.rotation.w = map_to_odom.getRotation().getW();
  map_to_odom_transform_msg.header.frame_id = "gazebo_world";
  map_to_odom_transform_msg.child_frame_id = highest_frame;
  auto stamp = tf2_ros::fromMsg(entity->header.stamp);
  map_to_odom_transform_msg.header.stamp = tf2_ros::toMsg(stamp + tf2::durationFromSec(1.0));
  broadcaster_.sendTransform(map_to_odom_transform_msg);
}