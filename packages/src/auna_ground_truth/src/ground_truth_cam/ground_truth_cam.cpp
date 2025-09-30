// Copyright 2025 Harun Teper
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "auna_ground_truth/ground_truth_cam.hpp"

#include <etsi_its_cam_msgs/msg/detail/heading__struct.hpp>
#include <etsi_its_cam_msgs/msg/detail/high_frequency_container__struct.hpp>

// Create the publisher, timer and service client
GroundTruthCam::GroundTruthCam()
: Node("ground_truth_cam_node")
{
  publisher_ = this->create_publisher<etsi_its_cam_msgs::msg::CAM>(
    "ground_truth_cam", 2);
  modelClient_ = this->create_client<gazebo_msgs::srv::GetEntityState>(
    "/get_entity_state");
  service_timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds(publish_milliseconds_),
    [this]() {service_timer_callback();});
  std::string ns = this->get_namespace();
  if (!ns.empty()) {
    this->name_ =
      ns.substr(1);    // Remove the first character (typically a slash)
  } else {
    this->name_ = ns;  // Keep it empty if namespace is empty
  }
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
        rclcpp::get_logger("rclcpp"),
        "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"),
      "service not available, waiting again...");
  }
  auto result = modelClient_->async_send_request(
    request, std::bind(
      &GroundTruthCam::model_srv_callback, this,
      std::placeholders::_1));
}

void GroundTruthCam::model_srv_callback(
  const rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture
  future)
{
  auto entity = future.get();
  if (!entity->success) {
    RCLCPP_WARN(
      this->get_logger(), "Entity '%s' not found in the simulation",
      name_.c_str());
    return;
  }
  etsi_its_cam_msgs::msg::CAM cam_msg;

  // Determine yaw from orientation quaternion
  tf2::Quaternion quat(
    entity->state.pose.orientation.x, entity->state.pose.orientation.y,
    entity->state.pose.orientation.z, entity->state.pose.orientation.w);
  quat.normalize();
  tf2::Matrix3x3 matrix(quat);
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  matrix.getRPY(roll, pitch, yaw);

  // Calculate theta and thetadot from yaw and angular velocity in degrees
  double heading = yaw * 180 / M_PI - 360 * floor(yaw * 180 / M_PI / 360);
  // double thetadot = entity->state.twist.angular.z * 180 / M_PI;

  // Set heading in CAM message
  etsi_its_cam_msgs::access::setHeading(cam_msg, heading);

  // Determine the speed and drive direction
  float len_x =
    entity->state.twist.linear.x /
    (abs(entity->state.twist.linear.x) + abs(entity->state.twist.linear.y));
  float len_y =
    entity->state.twist.linear.y /
    (abs(entity->state.twist.linear.x) + abs(entity->state.twist.linear.y));
  float len_h = sqrt(pow(len_x, 2) + pow(len_y, 2));
  float velocity_heading = acos(len_x / len_h) * 180 / M_PI;
  velocity_heading =
    (len_y >= 0) * velocity_heading + (len_y < 0) * (360 - velocity_heading);

  // Calculate speed and acceleration
  this->speed_ = sqrt(
    pow(entity->state.twist.linear.x, 2) +
    pow(entity->state.twist.linear.y, 2)) *
    scale_factor_;
  // float vdot = (this->speed_ - old_speed) / ((double)publish_milliseconds_ /
  // 1000) * scale_factor_;

  // Set speed in CAM message
  etsi_its_cam_msgs::access::setSpeed(cam_msg, this->speed_);

  // Set vehicle dimensions
  etsi_its_cam_msgs::access::setVehicleDimensions(
    cam_msg,
    0.49 * scale_factor_,     // vehicle_length
    0.18 * scale_factor_);    // vehicle_width

  publisher_->publish(cam_msg);
}
