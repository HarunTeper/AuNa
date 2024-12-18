// ground_truth_cam.cpp
#include "auna_gazebo/ground_truth_cam.hpp"

// Create the publisher, timer and service client
GroundTruthCam::GroundTruthCam(std::string name) : Node("ground_truth_cam_node")
{
  publisher_ = this->create_publisher<etsi_its_msgs::msg::CAM>("ground_truth_cam", 2);
  modelClient_ = this->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
  service_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(publish_milliseconds_), [this]() { service_timer_callback(); });
  this->name_ = name;
}

// Timer callback remains the same
void GroundTruthCam::service_timer_callback()
{
  // No changes needed here
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
    request, std::bind(&GroundTruthCam::model_srv_callback, this, std::placeholders::_1));
}

void GroundTruthCam::model_srv_callback(
  const rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture future)
{
  auto result = future.get();
  auto entity = result.get();

  etsi_its_msgs::msg::CAM cam_msg;

  // Basic header
  cam_msg.header.frame_id = this->name_;
  cam_msg.header.stamp = entity->header.stamp;

  // ITS PDU Header
  cam_msg.its_header.protocol_version = 1;
  cam_msg.its_header.message_id = 2;
  cam_msg.its_header.station_id = 1;

  // Station type
  cam_msg.station_type.value = 5;  // passenger car

  // Reference Position
  cam_msg.reference_position.latitude = entity->state.pose.position.x * scale_factor_;
  cam_msg.reference_position.longitude = entity->state.pose.position.y * scale_factor_;
  cam_msg.reference_position.altitude.value = entity->state.pose.position.z * scale_factor_;

  // Calculate heading from quaternion
  tf2::Quaternion quat(
    entity->state.pose.orientation.x, entity->state.pose.orientation.y,
    entity->state.pose.orientation.z, entity->state.pose.orientation.w);
  quat.normalize();
  tf2::Matrix3x3 matrix(quat);
  tf2Scalar roll, pitch, yaw;
  matrix.getRPY(roll, pitch, yaw);

  // High frequency container
  auto & hf = cam_msg.high_frequency_container;

  // Heading
  hf.heading.value =
    static_cast<uint16_t>((yaw * 180 / M_PI - 360 * floor(yaw * 180 / M_PI / 360)) * 10);
  hf.heading.confidence = 1;

  // Speed
  float current_speed =
    sqrt(pow(entity->state.twist.linear.x, 2) + pow(entity->state.twist.linear.y, 2)) *
    scale_factor_;
  hf.speed.value = static_cast<uint16_t>(current_speed * 100);
  hf.speed.confidence = 1;

  // Drive direction calculation
  float len_x = entity->state.twist.linear.x /
                (abs(entity->state.twist.linear.x) + abs(entity->state.twist.linear.y));
  float len_y = entity->state.twist.linear.y /
                (abs(entity->state.twist.linear.x) + abs(entity->state.twist.linear.y));
  float len_h = sqrt(pow(len_x, 2) + pow(len_y, 2));
  float velocity_heading = acos(len_x / len_h) * 180 / M_PI;
  velocity_heading = (len_y >= 0) * velocity_heading + (len_y < 0) * (360 - velocity_heading);

  // Vehicle dimensions
  hf.vehicle_length.value = static_cast<uint16_t>(0.49 * scale_factor_ * 10);
  hf.vehicle_width.value = static_cast<uint16_t>(0.18 * scale_factor_ * 10);

  // Acceleration
  float acceleration =
    (current_speed - this->speed_) / ((double)publish_milliseconds_ / 1000) * scale_factor_;
  hf.longitudinal_acceleration.value = static_cast<int16_t>(acceleration * 10);
  hf.longitudinal_acceleration.confidence = 1;

  // Curvature
  float curvature = entity->state.twist.angular.z / std::max(0.01f, current_speed) / scale_factor_;
  hf.curvature.value = static_cast<int16_t>(curvature * 10000);
  hf.curvature.confidence = 1;

  // Update stored speed
  this->speed_ = current_speed;

  // Set low frequency container flag
  cam_msg.has_low_frequency_container = false;

  publisher_->publish(cam_msg);
}