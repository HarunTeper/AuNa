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

// Publish the transform from gazebo_world to ground_truth_base_link using the pose from the future
void GroundTruthTransform::model_srv_callback(
  const rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture future)
{
  auto result = future.get();
  auto entity = result.get();

  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = entity->header.stamp;
  transform_msg.header.frame_id = "gazebo_world";
  transform_msg.child_frame_id = "ground_truth_base_link";
  transform_msg.transform.translation.x = entity->state.pose.position.x;
  transform_msg.transform.translation.y = entity->state.pose.position.y;
  transform_msg.transform.translation.z = entity->state.pose.position.z;
  transform_msg.transform.rotation = entity->state.pose.orientation;

  broadcaster_.sendTransform(transform_msg);
}
